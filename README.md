# RoboForge

A robotics IDE in Rust with an Android-Studio-style SDK manager: every project
pins its own SDK set, and SDKs install side by side under `~/.roboforge`
without touching the system or needing root.

Status: v0.1, a working foundation. See "What is and is not here" before
relying on it.

## Build and run

```bash
cargo build --release          # Rust 1.75+; dependencies: serde, serde_json, libc
./target/release/roboforge     # serves the IDE at http://127.0.0.1:7878 and opens it
```

Host requirements: Linux, `bash`, `git`, `curl`, `tar`, `bzip2`. Everything
else (ROS, compilers, cmake, Python) comes from the SDK environments.

## How isolation works

| Thing | Where | Shared? |
|---|---|---|
| micromamba (one static binary) | `~/.roboforge/tools/` | yes |
| Source SDKs (PX4, XRCE agent, SLAM repos, Isaac Lab) | `~/.roboforge/sdks/<id>/` | yes, versions side by side |
| Environments (ROS + Python packages) | `~/.roboforge/envs/<name>/` | per component set |
| Package cache | `~/.roboforge/mamba/` | yes |

- ROS 1 Noetic and ROS 2 Humble/Jazzy/Kilted/Lyrical/Rolling come from
  RoboStack conda channels, so no distro is tied to an Ubuntu release and any
  number can be installed at once.
- An environment's name is derived from its component set
  (`mujoco__ros2-jazzy`). Two projects with the same selection share it; a
  different selection can never modify it.
- Isaac Sim always gets its own environment (`own-isaac`) because it pins its
  dependency set. It talks to the ROS environment over DDS.
- A project's `roboforge.json` lists its components and run configurations.
  `.roboforge/activate.sh` is generated from it; run configurations execute
  inside that activation.
- Installs are recorded unit by unit in `~/.roboforge/state.json`, so a failed
  or cancelled install resumes.

## Workspaces

A project is a set of folders, like roots in a VS Code multi-root workspace,
listed under `workspaces` in `roboforge.json`:

| kind | default folder | holds |
|---|---|---|
| `ros2` / `ros1` | `ros2_ws/` / `catkin_ws/` | colcon / catkin workspace; the project's package in `src/` |
| `mav` | `mav_ws/` | standalone MAVSDK / PX4 applications, no ROS |
| `sim` | `sim/` | simulator models, worlds, scripts outside ROS |
| `ml` | `ml/` | training and research code |
| `isaac` | `isaac/` | Isaac Sim apps (dedicated Isaac environment) |

Workspaces appear as modules need them, so a ROS 2 + MAVSDK project has
`ros2_ws/` for the bridge node and `mav_ws/` for the standalone app; the
colcon workspace never contains non-ROS files. Each run configuration
declares which workspace it runs in.

Reusing an existing ROS workspace: pick "Reuse an existing ROS workspace" in
the wizard (`--workspace ~/ros2_ws` on the CLI) and the project's package is
created inside it next to your packages. Or set up a workspace in place
("Set up this workspace" in Open, `roboforge init --ros humble`): only
`roboforge.json` and `.roboforge/` are added. More workspaces, including
extra ROS overlays, can be added under Workspaces / `roboforge ws add`;
external ones show as extra roots in the file tree. `roboforge pkg <name>
[--cmake]` creates a new ament_python, ament_cmake or catkin package in a
chosen workspace.

Environment handling in `.roboforge/activate.sh`: activate the SDK
environment (RoboStack's underlay), export SDK paths (`PX4_DIR`, ...), export
`RF_WS_<KIND>` per workspace, overlay every ROS workspace in the listed order
(`install/setup.bash` or `devel/setup.bash`), set `GZ_SIM_RESOURCE_PATH` to
`sim/models`, then export the project's `env` map (`ROS_DOMAIN_ID`,
`ROS_LOCALHOST_ONLY` by default; add `RMW_IMPLEMENTATION` and so on there).
A non-ROS folder inside a root-level ROS workspace gets `COLCON_IGNORE`.

## Platform

Linux. The backend uses bash, Unix process groups and signals, and the SDK
recipes (RoboStack, PX4 SITL, Gazebo, Isaac Sim) target Linux, so on Windows
build and run it inside WSL2 (GUI simulators work via WSLg). macOS is
untested; the build is refused on Windows with a message saying so.

## Modules: mix and match

A project is a ROS choice plus any set of modules. A module is one paradigm
(MoveIt, Nav2, slam_toolbox, MAVSDK, PX4 offboard, MuJoCo, ...) and declares:

- the SDK components it needs (installed into the project's environment),
- files it contributes (nodes, scripts, models, worlds),
- ROS package dependencies and data folders,
- run configurations, including ones that only appear next to another module
  (`Nav2 + SLAM: TurtleBot3 simulation` needs both `nav2` and `slam-toolbox`),
- optional entries in the generated `bringup.launch.py`, each switched with
  `use_<name>:=true`.

All ROS 2 modules land in one package, so their nodes can import each other.
The `mission` module uses that: it sequences drone takeoff/land (MAVSDK
bridge), navigation (Nav2) and arm motion (MoveIt) from whichever of those
the project contains.

Built in (15): `moveit`, `nav2`, `slam-toolbox`, `gs-slam`, `mavsdk`,
`px4-sitl`, `px4-offboard`, `gazebo`, `mujoco`, `isaac`, `isaac-lab`, `torch`,
`pytorch3d`, `gsplat`, `mission`.

Modules can be changed after creation (the Modules button, or
`roboforge add` / `roboforge remove`):

- new module files are created; existing files are never overwritten;
- `package.xml`, `setup.py`, `setup.cfg` and `bringup.launch.py` are managed
  and regenerated. Every `.py` in the package that defines `main()` is
  registered as a node, including ones you wrote. Extra package dependencies
  go in `"depends"` in `roboforge.json`;
- run configurations you added or edited are kept; a removed module's are
  dropped; its files stay and are listed so you can delete them.

### Writing your own module

Create `~/.roboforge/modules/<id>/module.json` and put templates in
`files/`. No rebuild; a module with a built-in id replaces the built-in one.

```json
{
  "id": "lidar-driver",
  "name": "Lidar driver",
  "category": "drivers",
  "summary": "Publishes /scan from the device.",
  "ros": "ros2",
  "components": ["open3d", "group:torch=torch-cpu"],
  "requires": ["nav2"],
  "depends": ["sensor_msgs"],
  "data_dirs": ["config"],
  "files": [{"src": "driver.py", "dest": "@PKGDIR@/@PKG@/lidar_driver.py"}],
  "run": [{"name": "Lidar: driver", "cmd": "ros2 run @PKG@ lidar_driver", "when": ["ros2"]}],
  "launch": [{"name": "lidar", "node": {"package": "@PKG@", "executable": "lidar_driver"}}]
}
```

Placeholders: `@PKG@`, `@NAME@`, `@PKGDIR@`, `@ASSETS@`, `@ROS_DISTRO@`,
`@PX4_BRANCH@`. `when` tokens: `ros2`, `ros1`, `noros`, `mod:<id>`,
`!mod:<id>`. `group:<g>=<default>` means "any member of that SDK group, this
one if the project has none". Built-in modules use the same format; see
`crates/roboforge-core/modules/`.

## Catalog

25 SDK components are built in (`crates/roboforge-core/src/catalog.json`): six
ROS distros, PX4 1.15/1.16, Micro XRCE-DDS agent, MAVSDK, MoveIt, Nav2, Gazebo,
MuJoCo, Isaac Sim, Isaac Lab, PyTorch (CUDA/CPU), PyTorch3D, gsplat, Open3D,
slam_toolbox, MonoGS, SplaTAM, ORB-SLAM3. To change a version or add an SDK,
copy that file to `~/.roboforge/catalog.json` and edit it; no rebuild needed.

## New-project presets

`ros2`, `ros1`, `px4`, `mujoco`, `ros2-px4`, `ros2-mujoco`, `manipulation`,
`navigation`, `mixed`, `full`, `isaac`, `vision3d`, `gs-slam`. A preset only
pre-ticks a ROS choice and modules; any other mix works.

## CLI

```bash
roboforge modules                         # what can be mixed in
roboforge new navigation ~/ws/bot --ros humble --modules moveit,mavsdk
roboforge add mission mujoco --dir ~/ws/bot
roboforge remove mujoco --dir ~/ws/bot
roboforge sdk plan ros2-humble gazebo     # print install commands, change nothing
roboforge sync ~/ws/bot                   # install what the project pins
roboforge run "Build workspace (colcon)" ~/ws/bot
```

## What is and is not here

Verified in development (no GPU, no access to conda/NVIDIA hosts):

- `cargo test`: catalog and module validity, resolver conflicts, plan
  contents and resume, generation of all thirteen presets, module mixing,
  adding and removing modules on an existing project, user modules,
  workspace separation, reuse of an external workspace (activation really
  overlays it), in-place init, adding workspaces and packages.
- Generated Python, XML, SDF and shell files pass syntax checks.
- API and UI smoke tests against the running binary (`tests/ui_smoke.js`,
  jsdom): wizard, module mixing and requirement feedback, the Modules
  dialog on an open project, the Workspaces dialog and New package, file tree, edit and save, run
  configurations, console, job stop, SDK manager, loopback/token checks,
  Ctrl+C stopping child process groups.
- A source-only SDK install end to end (clone, state, removal).

Not verified, expect to debug on first real use:

- No environment install has actually been run. The micromamba, RoboStack,
  pip and Isaac Sim recipes follow upstream documentation, but package names
  per distro (for example `ros-<distro>-ros-gz`, `ros-<distro>-slam-toolbox`)
  and solver compatibility are untested.
- PX4 SITL built against conda-provided Gazebo is untested and the most
  likely thing to need work (a container backend for PX4 is the fallback).
- Generated nodes and scripts have not been run against ROS, MuJoCo, PX4,
  Isaac Sim, PyTorch3D, gsplat, MoveIt, Nav2 or MAVSDK. The MoveIt, Nav2,
  MAVSDK bridge and mission nodes are written against the documented APIs
  and pass syntax checks only.
- The mission node assumes the Panda MoveIt demo and the TurtleBot3 Nav2
  simulation as targets; those are separate simulations, so a real mixed
  robot needs its own URDF, MoveIt config and Nav2 parameters.
- The CodeMirror editor path (loaded from cdnjs when online) is untested;
  offline the editor is a plain textarea with line numbers.

Not built yet:

- Native window (the UI is served to the browser; wrapping the same backend
  in Tauri is the intended next step), PTY terminal (the console runs one
  command per job), LSP (clangd, pyright), debugger, git UI, search.
- ROS graph/topic visualisation beyond `Topics`/`Nodes` listing, URDF and
  splat viewers, PX4 parameter/uORB tooling, firmware flashing.
- Container backend for SDKs that cannot live in conda, macOS/Windows.

## Security model

The backend binds to 127.0.0.1 only, rejects requests whose Host header is
not loopback, and requires a per-session random token on every API call. It
runs commands as you, so do not expose the port.

## Layout

```
crates/roboforge-core/   catalog, resolver, install planner, module registry, generator (+ tests)
crates/roboforge-core/modules/   built-in modules: module.json + files/ each
crates/roboforge/        CLI, job runner, HTTP API, embedded UI (ui/index.html)
tests/ui_smoke.js        jsdom smoke test (npm i jsdom; start the IDE on port 7911 first)
```
