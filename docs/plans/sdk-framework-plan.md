# RoboForge SDK: an Android-Studio-like home for robotics stacks

## Context

RoboForge today is an IDE plus scaffolding. Its "modules" (MuJoCo, MAVSDK, PX4, Nav2…)
each drop a standalone boilerplate script into a new project
(`crates/roboforge-core/modules/*/files/*.py`). The scripts share no API, so every
combination (MuJoCo + MAVLink, MuJoCo + ROS, ROS + MAVLink, all three) needs its own
glue, and none exists for MuJoCo + MAVLink at all.

The goal: go beyond the IDE to an **SDK + IDE that feel like Android Studio** — one
place to install every library, start from a template, run on a simulator, and inspect
what is happening — for robotics, SLAM and neural-rendering work alike.

Decisions taken with the user:
- Python first; IDE and scaffolding stay in Rust.
- MuJoCo + MAVLink supports both PX4-in-the-loop and MAVLink vehicle emulation.
- First milestone is a thin vertical slice, then widen.
- Robots: quadrotor, arm, wheeled, and custom URDF/MJCF (incl. humanoids).
- Lightning / Transformers / nerfstudio are **inspiration, not templates to copy**.
- Renderers must be hackable and simulators replaceable (Gazebo, Isaac, your own).
- SLAM and Gaussian-splatting experiments are first-class, including for graphics people.

## What exists, and the weaknesses to cover

| Existing approach | What it does well | Weakness this design must avoid |
|---|---|---|
| ROS 2 + ros2_control + per-simulator bridges (ros_gz, mujoco_ros2_control) | Standard middleware, hardware abstraction | Heavy boilerplate (packages, XML, launch files); one bridge per simulator; no experiment or dataset layer; sim time vs wall time is easy to get wrong |
| PX4 SITL + Gazebo; Pegasus (Isaac + PX4) | Real autopilot in the loop | Tied to one simulator each. PX4 talks to most simulators over MAVLink HIL messages but to Gazebo over its own transport, so nothing transfers. MuJoCo has no MAVLink path at all |
| Isaac Lab, Genesis, ManiSkill | Fast GPU-parallel learning | Locked to one engine and its renderer; heavy GPU and install requirements; ROS and autopilots are afterthoughts; aimed at training, not at running the same code on a robot |
| RoboVerse / MetaSim | One API over several simulators; can pair one engine's physics with another's renderer | Built for robot-learning benchmarks: no middleware or autopilot story, no deployment path. Evidence on how well the cross-simulator abstraction holds up was not found |
| Drake | Rigorous "systems with typed ports" model | Steep learning curve; own physics; not built around ROS / MAVLink |
| LeRobot, robosuite | Datasets and policies | Manipulation-centric; thin on simulation, middleware and aerial vehicles |
| nerfstudio | Pluggable methods, data parsers, viewer | Offline and batch only: no live sensor stream, no robot or simulator in the loop |
| Gaussian-splatting SLAM (MonoGS, SplaTAM, …) | State-of-the-art results | Every method is its own repo with its own data loader and forked rasteriser; no shared live interface, no common benchmark, and no clear winner |
| Lightning, Transformers, mmcv (the inspirations) | Low boilerplate, registries, hubs | Hidden control flow that is hard to debug; config inheritance that is hard to trace; one giant package |

Design rules that follow:
1. **A library first, a framework second.** Every part works on its own; the app loop is optional and you can always write the loop yourself.
2. **No simulator, renderer or middleware is privileged.** Each sits behind a small interface and is swappable.
3. **Physics and rendering are separate** — the single biggest gap in the tools above.
4. **Explicit over magic:** flat configs with no inheritance, a printable fully-resolved config, no global state, time handled explicitly (lockstep, deterministic replay).
5. **Small core, plugins outside it**, discovered by entry points, each installable from the IDE's SDK Manager.
6. **Same code for simulation and the real robot:** swapping the simulator for "none" is a config change.
7. **Easy to install, headless-first, CPU-friendly by default.**

## Design

### A signal bus instead of bridges

Simulators, renderers and transports never talk to each other directly. Everything
publishes and subscribes typed signals on an in-process bus (`state/joint`, `state/pose`,
`sensor/imu`, `sensor/camera/<name>`, `cmd/actuators`, `clock`…). N backends × M
transports needs N + M adapters, not N × M bridges.

### Concepts (package `sdk/python/roboforge/`)

| Concept | Role | First implementations |
|---|---|---|
| `Robot` | Model + which sensors/actuators map to which signals | zoo: quadrotor, arm, diff-drive; `from_mjcf`, `from_urdf` with profiles `Multirotor`, `Arm`, `DiffDrive`, `Articulated` (generic; humanoids) |
| `Physics` | Steps the world; publishes state; consumes commands | `MujocoPhysics`; `NoPhysics` (real hardware / external). Later: Gazebo, Isaac as external-process backends |
| `Renderer` | Turns world state + a camera into frames (RGB, depth, segmentation, custom outputs) | MuJoCo's renderer; **splat renderer** (render a Gaussian-splat scene as the camera). Later: Gazebo's rendering engine, Isaac RTX, your own |
| `Link` | Transport adapter between bus and outside world | `Ros2Link`; `MavlinkLink(mode="vehicle"｜"hil"｜"client")` |
| `Controller` | User code: `setup`, `step(obs, t) -> commands`, `on_reset` | `MultirotorController`, `JointPD`, `DiffDriveController` |
| `Callback` | Cross-cutting hooks | `Viewer`, `Recorder`, `RateMonitor` |
| `App` | Optional loop owner: clock, lifecycle, link threads | `App(robot, physics, renderer, links, controller).run()` or `app.step()` |

**Hackable rendering.** `Renderer` is a small protocol (`render(world_state, camera) -> Frame`)
with pre/post-pass hooks, so graphics users can add a pass, emit extra buffers, or
replace the renderer outright while keeping the physics. Any physics pairs with any
renderer: MuJoCo dynamics with a Gaussian-splat scene, or Gazebo dynamics with your
own rasteriser.

**Drop-in simulator replacement.** A replacement implements `Physics` (and optionally
`Renderer`) and is registered by name. To the rest of the stack it is indistinguishable
because compatibility lives in the links: `MavlinkLink(mode="hil")` speaks PX4's
simulator protocol, and `Ros2Link(profile="gazebo"｜"isaac")` publishes the topic names
and types those simulators' ROS bridges use. This is replacement at the interface
level; it does not load Gazebo or Isaac plugins.

**MAVLink modes** (pymavlink): `vehicle` — the app is a MAVLink vehicle, so
QGroundControl / MAVSDK connect with no PX4; `hil` — the app is PX4 SITL's simulator,
lockstepped; `client` — talks to an existing autopilot (the ROS + MAVLink case).

**Three levels of control.** (1) Config only: `app.yaml`, `python -m roboforge run app.yaml physics.realtime=false`.
(2) Subclass a `Controller` / `Callback` / `Renderer`. (3) Raw: `physics.raw`
(`MjModel`, `MjData`), `ros.raw` (rclpy node), `mav.raw` (pymavlink connection), your own
loop, your own registered backend.

### From PID loops to VLAs and world models

One control interface has to hold a 10-line PID loop and a multi-gigabyte
vision-language-action model without the simple case paying for the complex one.

- **The contract stays tiny:** a controller is anything with `step(obs, t) -> action`.
  A PID loop is a plain function or a 10-line class; nothing else is required.
- **Declared inputs and outputs (optional):** a controller may state which signals it
  reads (with history length) and what it emits. Simple controllers skip this and get
  the robot profile's defaults; learned policies use it to get image stacks, proprioception
  and a language instruction (`cmd/instruction` is just another signal).
- **Every controller has its own rate.** The app schedules them independently, so a
  500 Hz PID, a 50 Hz MPC and a 5 Hz VLA coexist.
- **Hierarchy by composition:** `Stack([vla @ 5 Hz, planner @ 50 Hz, pid @ 500 Hz])` —
  each layer's action is the next layer's setpoint. Swapping the top layer from a
  joystick to a VLA changes one line.
- **Slow models never block the loop:** a policy can run asynchronously (thread, process,
  or remote inference server). Action chunks are buffered and played out at the control
  rate, and a configurable fallback (hold, hover, stop) applies when inference is late.
- **`Policy`** is a `Controller` with model lifecycle added: `load` / `from_pretrained`,
  device placement, action chunking, normalisation statistics. Wrappers for LeRobot
  policies and VLA checkpoints are plugins, not core.
- **World models plug in at two places:** as a `Physics` backend (a learned simulator you
  can step and render, for imagination rollouts) and as a model inside a planner
  controller (MPC over the learned dynamics).
- **Training hooks:** `app.as_env()` exposes any stack as a Gymnasium environment, and
  `Dataset` exports to the LeRobot / RLDS layouts that VLA training expects.

### Experiments: SLAM, NeRF, Gaussian splatting

Sensor streams and ground truth are on the same bus, so perception work reuses everything.

| Concept | Role | First implementations |
|---|---|---|
| `Dataset` | A recorded run in a standard layout, convertible between formats | record from any app; export nerfstudio, TUM / EuRoC, rosbag2 |
| `Method` | Uniform wrapper around an external system: install, feed, read trajectory/map | ORB-SLAM3, FAST-LIO, slam_toolbox, Cartographer; nerfstudio (`nerfacto`, `splatfacto`), gsplat; Gaussian SLAM (MonoGS, SplaTAM) |
| `Rasterizer` | The splat rasteriser a method or the splat renderer uses | gsplat; the reference differentiable rasteriser; your own |
| `Experiment` | Run methods on a dataset or live; metrics; compare | ATE / RPE vs simulator ground truth; PSNR / SSIM / LPIPS |

This closes a loop none of the tools above offer: fly in simulation → record → reconstruct
a splat → use that splat as the simulator's rendered world.

## The Android Studio experience (IDE)

| Android Studio | RoboForge |
|---|---|
| SDK Manager | Existing SDK Manager, extended: the `roboforge` SDK, every backend, and every `Method` are versioned components with install recipes |
| New Project templates | **Stack picker**: simulator × renderer × middleware × autopilot link × robot, then a summary step (same style as Import) |
| Device / emulator manager | **Simulator Manager**: named simulator setups (physics + renderer + world + robot) you start, stop and switch |
| Run configurations | Existing Run / Launch configurations, generated from `app.yaml` |
| Logcat | **Signal monitor**: live values and rates of bus signals |
| Layout Inspector | **Stack view**: components and which signals each link carries |
| Gradle sync / doctor | Sync SDKs + `roboforge doctor` |

## Milestones

**Milestone 1 — vertical slice**
- SDK core: bus, messages, registry, config, `App`, `Controller`, `Callback`, CLI
  (`run`, `doctor`, `info --json`, `zoo list`).
- `MujocoPhysics` + MuJoCo renderer (camera, IMU); zoo models; URDF/MJCF loading.
- `Ros2Link`; `MavlinkLink` vehicle + client; `hil` behind an experimental flag.
- Control: per-controller rates, `Stack` hierarchy, built-in PID-based controllers,
  asynchronous controllers with action buffering and a late-inference fallback,
  `app.as_env()` (Gymnasium).
- `Recorder` → `Dataset` with ground truth, export to nerfstudio and TUM layouts.
- Examples: MuJoCo + ROS arm, MuJoCo + MAVLink quadrotor, ROS + MAVLink bridge, custom URDF.
- Scaffolding: `roboforge-sdk` and `pymavlink` catalog components; SDK-based modules and
  four "SDK:" presets; scanner support for `app.yaml`, MJCF and `import mujoco`.
- IDE: stack picker, read-only Stack view, generated run/launch entries and VS Code tasks.
- Docs in HTML (`docs/index.html` section + `docs/sdk.html`).

**Milestone 2 — experiments and rendering**
- `Method` wrappers (ORB-SLAM3, FAST-LIO, nerfstudio, gsplat, MonoGS), `Experiment`, metrics.
- Splat renderer, `Rasterizer` registry, renderer hooks; reconstruct → simulate loop.
- `Policy` plugins: LeRobot policies and a VLA checkpoint wrapper (`from_pretrained`,
  action chunking, remote inference); `Dataset` export to LeRobot / RLDS.
- IDE: Experiments panel (datasets, runs, metric table, trajectory plot), Signal monitor.
- Migrate the existing `gs-slam`, `gsplat`, `slam-toolbox` modules onto it.

**Milestone 3 — more simulators**
- Gazebo and Isaac as `Physics` / `Renderer` backends; Gazebo rendering hooks;
  compatibility profiles; Simulator Manager; ArduPilot; lidar; multi-robot.
- World models: a learned `Physics` backend and a model-predictive planner controller.

## Critical files

- New: `sdk/python/roboforge/**`, `sdk/python/examples/**`, `sdk/python/tests/**`, `docs/sdk.html`.
- `crates/roboforge-core/src/catalog.json` — new components.
- `crates/roboforge-core/modules/**` — SDK-based modules (existing `module.json` format).
- `crates/roboforge-core/src/templates.rs` — `PRESETS`.
- `crates/roboforge-core/src/tools.rs` — `folder_tools` gains `app.yaml` / MJCF detection.
- `crates/roboforge/ui/index.html`, `crates/roboforge/src/server.rs` — stack picker, Stack view.
- Reuse: `modules/mujoco/files/mujoco_bridge.py` → `Ros2Link`; `modules/mavsdk/files/mavsdk_bridge.py` → client mode.

## Current state and test environment

- Nothing of the SDK is written yet. One leftover exists from a premature start:
  `sdk/python/.venv/`, an empty, broken virtualenv (it has no pip). First step is to
  delete it and add `.venv/` to `.gitignore`.
- `python3 -m venv` cannot make a working environment here: the system Python 3.12 has
  no `ensurepip` (the `python3.12-venv` package is not installed). `uv` is installed
  (`~/.local/bin/uv`), so the test environment is created with
  `uv venv --python /usr/bin/python3 --system-site-packages sdk/python/.venv` and filled
  with `uv pip install mujoco pymavlink pytest pyyaml gymnasium`. No sudo needed.
- It must be the system Python (not the conda `base` one) so that ROS 2 Jazzy's `rclpy`
  is importable alongside MuJoCo and pymavlink.
- The IDE's own SDK Manager recipe for the `roboforge-sdk` component should use the same
  approach, so a fresh machine gets a working environment without sudo.

## Known risks

- **Abstraction leak across simulators** is the classic failure of unified APIs. Mitigation:
  the interface stays small (state, commands, time), `.raw` is always available, and each
  backend documents what it does not support instead of emulating it badly.
- **PX4-in-the-loop** needs a PX4 SITL build, which this machine does not have. It is built
  and tested against a loopback fake PX4 endpoint, and shipped as experimental unless a real
  PX4 run can be done here.
- **External methods are heavy to install** (ORB-SLAM3 and FAST-LIO build from source;
  nerfstudio needs CUDA PyTorch). Each is verified only as far as it can be built here.
- `mujoco` and `pymavlink` are not installed yet; verification uses a test virtualenv that
  can also see the system ROS Python packages.
- "Drop-in replacement" means interface-level compatibility, not loading Gazebo/Isaac plugins.
- API names here are a proposal: cheap to change before Milestone 2, expensive after.

## Verification (Milestone 1)

1. `pytest sdk/python/tests` — bus, config round-trip, MuJoCo stepping, MAVLink vehicle mode
   over UDP loopback (heartbeat, arm, takeoff, position target), `hil` against the fake endpoint.
2. Run each example headless and assert on output: `/joint_states` and `/clock` advance;
   a MAVLink client arms and climbs; MAVLink telemetry appears on ROS topics; a URDF loads and steps.
3. Control spectrum: a 10-line PID controller stabilises the arm; then a deliberately slow
   stand-in policy (2 Hz, 300 ms latency, emitting action chunks) is stacked above a 200 Hz
   PID, and the test asserts the fast loop's rate never drops and the fallback fires when
   the slow layer is paused. This proves the structure for VLAs without needing one.
4. Record a short MuJoCo flight with a camera, export to nerfstudio and TUM layouts, and check
   image count and poses against simulator ground truth.
5. `cargo test -p roboforge-core` — presets generate, scanner finds `app.yaml` / MJCF.
6. Headless-Chrome UI pass on a separate test port: create a project from the stack picker,
   check Run menu, Launch dialog and Stack view, run one generated launch configuration.

## Sources for the comparison

- RoboVerse / MetaSim: https://arxiv.org/abs/2504.18904 , https://roboverseorg.github.io
- PX4 simulator interface: https://docs.px4.io/main/en/simulation/index
- Pegasus Simulator: https://arxiv.org/pdf/2307.05263
- MonoGS: https://github.com/RPFey/MonoGS ; Splat-SLAM: https://arxiv.org/html/2405.16544v1
