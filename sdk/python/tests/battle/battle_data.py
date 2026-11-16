"""Rendering, recording, datasets, training environments, custom models, the
command line, threads. The parts users script against directly."""
import os
import pathlib
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

import roboforge as rf
from harness import clear_error, rss_mb, scenario

PY = sys.executable


def tmp():
    return pathlib.Path(tempfile.mkdtemp(prefix="rf-battle-"))


def quad(**kw):
    kw.setdefault("controllers", [rf.MultirotorController(rate=250, hover=[0, 0, 1.5])])
    return rf.App(robot="quadrotor", realtime=False, **kw)


@scenario(timeout=120)
def render_sizes_passes_and_threads():
    """Full-HD frames, depth, several passes, a pass that misbehaves, rendering from another thread."""
    r = rf.MujocoRenderer(width=1920, height=1080, fps=5, depth=True)
    app = quad(renderer=r)
    app.run(duration=1.0)
    img, depth = app.bus.latest("sensor/camera/front"), app.bus.latest("sensor/depth/front")
    assert img.data.shape == (1080, 1920, 3) and img.data.dtype == np.uint8, f"colour frame {img.data.shape} {img.data.dtype}"
    assert depth.data.shape == (1080, 1920) and np.isfinite(depth.data).all() and depth.data.min() > 0, "depth frame"
    assert img.data.std() > 5, "the frame is blank"
    app.close()

    r = rf.MujocoRenderer(width=160, height=120, fps=1000)            # faster than physics: one frame per step, not more
    calls = []
    r.add_pass(lambda im, rr: calls.append(im.t))
    r.add_pass(lambda im, rr: rf.Image(data=255 - im.data, camera=im.camera, fovy=im.fovy, pose=im.pose, t=im.t))
    app = quad(renderer=r)
    app.run(steps=100)
    assert len(calls) == 100, f"{len(calls)} frames for 100 steps at fps above the physics rate"
    out = {}
    th = threading.Thread(target=lambda: out.setdefault("img", r.render("front")))     # a GL context belongs to one thread
    th.start(); th.join(30)
    assert "img" in out and out["img"].data.shape == (120, 160, 3), "rendering from a second thread failed"
    app.run(steps=10)                                                # and the loop thread still renders afterwards
    app.close()

    bad = rf.MujocoRenderer(width=64, height=48, fps=50)
    bad.add_pass(lambda im, rr: 1 / 0)
    app = quad(renderer=bad)
    try:
        app.run(steps=30)
    except ZeroDivisionError as e:
        assert any("pass" in n for n in getattr(e, "__notes__", [])), f"a failing render pass is not identified: {getattr(e, '__notes__', [])}"
    else:
        raise AssertionError("a render pass that raises was swallowed")
    app.close()
    wrong = rf.MujocoRenderer(width=64, height=48, fps=50)
    wrong.add_pass(lambda im, rr: "not an image")
    app = quad(renderer=wrong)
    msg = clear_error(lambda: app.run(steps=30), "pass", "Image")
    app.close()
    return f"1080p + depth ok; thread-safe; bad pass: {msg[:90]}"


@scenario(timeout=180)
def record_and_export_edge_cases():
    d = tmp()
    out = []
    # nothing recorded yet / not a dataset / wrong camera
    out.append(clear_error(lambda: rf.Dataset(d / "nope"), "nope"))
    (d / "empty").mkdir()
    out.append(clear_error(lambda: rf.Dataset(d / "empty").export_nerfstudio(d / "x"), "empty"))
    app = quad(renderer=rf.MujocoRenderer(width=96, height=72, fps=10), callbacks=[rf.Recorder(dir=str(d / "run"), rate=50)])
    app.run(duration=2.0)
    app.close()
    ds = rf.Dataset(d / "run")
    n = len(ds.frames())
    assert 19 <= n <= 21, f"{n} frames for 2 s at 10 fps"
    out.append(clear_error(lambda: ds.export_nerfstudio(d / "ns", camera="rear"), "rear", "front"))
    # recording again into the same folder must not mix two runs
    app = quad(renderer=rf.MujocoRenderer(width=96, height=72, fps=10), callbacks=[rf.Recorder(dir=str(d / "run"), rate=50)])
    app.run(duration=1.0)
    app.close()
    n2 = len(rf.Dataset(d / "run").frames())
    assert 9 <= n2 <= 11, f"a second recording into the same folder left {n2} frames (expected the new run's 10, not {n} + 10)"
    pngs = len(list((d / "run").rglob("*.png")))
    assert pngs == n2, f"{pngs} image files on disk for {n2} frames in the index"
    # no renderer: signals only; image exports must say why they cannot work
    app = quad(callbacks=[rf.Recorder(dir=str(d / "blind"), rate=50)])
    app.run(duration=1.0)
    app.close()
    blind = rf.Dataset(d / "blind")
    assert not blind.cameras and len(blind.signals().get("state/pose", [])) >= 49, f"signal-only recording: {blind.cameras}, {list(blind.signals())}"
    out.append(clear_error(lambda: blind.export_nerfstudio(d / "ns2"), "camera"))
    # a run that crashes still leaves a readable dataset
    class Boom(rf.Controller):
        def step(self, obs, t):
            if t > 0.5:
                raise RuntimeError("crash mid-run")
    app = quad(renderer=rf.MujocoRenderer(width=96, height=72, fps=10), controllers=[Boom()], callbacks=[rf.Recorder(dir=str(d / "crash"), rate=50)])
    try:
        app.run(duration=2.0)
    except RuntimeError:
        pass
    app.close()
    assert 4 <= len(rf.Dataset(d / "crash").frames()) <= 6, "the frames recorded before a crash were lost"
    # a place that cannot be written
    ro = d / "readonly"
    ro.mkdir()
    os.chmod(ro, 0o500)
    out.append(clear_error(lambda: quad(callbacks=[rf.Recorder(dir=str(ro / "run"))]).run(steps=5), "readonly"))
    return " | ".join(o[:70] for o in out)


@scenario(timeout=240)
def record_long_run_memory_is_flat():
    """3000 frames and 150000 signal samples: memory must not grow with the length of the recording."""
    d = tmp()
    app = quad(renderer=rf.MujocoRenderer(width=160, height=120, fps=50), callbacks=[rf.Recorder(dir=str(d / "long"), rate=250)])
    app.run(duration=10)
    m0 = rss_mb()
    app.run(duration=50)
    grew = rss_mb() - m0
    app.close()
    ds = rf.Dataset(d / "long")
    n, s = len(ds.frames()), len(ds.signals()["state/pose"])
    assert len(ds.times("state/pose")) == s and ds.signals()["state/pose"].shape == (s, 7)
    assert abs(n - 3000) <= 2 and abs(s - 15000) <= 2, f"{n} frames, {s} pose samples"
    assert grew < 40, f"memory grew {grew:.0f} MB while recording 2500 more frames"
    ns = ds.export_nerfstudio(d / "ns")
    tum = ds.export_tum(d / "tum")
    import json
    tf = json.loads((ns / "transforms.json").read_text())
    assert len(tf["frames"]) == n
    return f"{n} frames, {s} samples, +{grew:.0f} MB, exports ok"


@scenario(timeout=180, needs=("gymnasium",))
def gym_env_checker_and_random_agent():
    from gymnasium.utils.env_checker import check_env
    import gymnasium as gym
    env = rf.App(robot="arm").as_env(reward=lambda o: -float(np.abs(o.joint.position).sum()), max_steps=50)
    assert isinstance(env, gym.Env), "as_env() does not return a gymnasium.Env, so wrappers and vector envs reject it"
    check_env(env, skip_render_check=True)
    rng = np.random.default_rng(0)
    obs, _ = env.reset(seed=0)
    episodes = 0
    for i in range(6000):
        obs, r, term, trunc, info = env.step(env.action_space.sample() * rng.uniform(0, 3))    # up to 3x out of bounds
        assert np.all(np.isfinite(obs)) and np.isfinite(r), f"step {i}: obs {obs} reward {r}"
        if term or trunc:
            obs, _ = env.reset()
            episodes += 1
    assert episodes == 120, f"{episodes} episodes for 6000 steps at max_steps 50"
    msg = clear_error(lambda: env.step(np.array([np.nan, 0, 0])), "nan")
    obs2, _ = env.reset()
    assert np.all(np.isfinite(obs2)), "the environment is unusable after a rejected action"
    env.close()
    return f"check_env passed; 120 random episodes finite; {msg[:60]}"


URDF = """<robot name="twolink">
  <link name="base"><inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="l1"><inertial><origin xyz="0 0 0.15"/><mass value="0.5"/><inertia ixx="0.005" iyy="0.005" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="0 0 0.15"/><geometry>%s</geometry></visual></link>
  <joint name="j1" type="revolute"><parent link="base"/><child link="l1"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="20" velocity="5"/></joint>
</robot>
"""


def humanoid(n=24):
    """A chain of capsules with a free base and one motor per hinge: a stand-in for a many-jointed robot."""
    body, close = "", ""
    for i in range(n):
        body += f'<body name="b{i}" pos="0 0 -0.08"><joint name="j{i}" type="hinge" axis="{i % 2} {(i + 1) % 2} 0" range="-1 1" damping="0.5"/><geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.08" mass="0.2"/>'
        close += "</body>"
    motors = "".join(f'<motor name="m{i}" joint="j{i}" ctrlrange="-5 5"/>' for i in range(n))
    return (f'<mujoco><option timestep="0.002"/><worldbody><geom type="plane" size="5 5 0.1"/>'
            f'<body name="torso" pos="0 0 2.5"><freejoint/><geom type="box" size="0.1 0.1 0.1" mass="3"/>{body}{close}</body></worldbody>'
            f'<actuator>{motors}</actuator></mujoco>')


@scenario(timeout=120)
def models_custom_broken_and_large():
    d = tmp()
    out = []
    (d / "ok.urdf").write_text(URDF % '<cylinder radius="0.02" length="0.3"/>')
    app = rf.App(robot={"urdf": str(d / "ok.urdf")}, realtime=False)
    app.run(duration=2.0)
    q = app.bus.latest("state/joint").position
    assert np.all(np.isfinite(q)) and len(q) == 1
    app.close()
    # a URDF has joints but no motors: one torque motor per joint is added, limited to the URDF's effort
    app = rf.App(robot={"urdf": str(d / "ok.urdf")}, controllers=[rf.JointPD(rate=500, target=[0.7])], realtime=False)
    app.run(duration=3.0)
    assert app.physics.actuator_names == ["j1"] and app.physics.ctrl_range.tolist() == [[-20.0, 20.0]]
    assert abs(app.bus.latest("state/joint").position[0] - 0.7) < 0.05, "the URDF joint did not follow its target"
    app.close()
    # asked not to add motors: the joint controller must say what is missing, not divide by zero
    out.append(clear_error(lambda: rf.App(robot={"urdf": str(d / "ok.urdf"), "motors": False}, controllers=[rf.JointPD()], realtime=False).run(steps=5), "actuator"))
    # a floating base with collision shapes lands on the ground that was added for it
    (d / "float.urdf").write_text((URDF % '<box size="0.1 0.1 0.1"/>').replace('<link name="base">', '<link name="base"><collision><geometry><box size="0.2 0.2 0.1"/></geometry></collision>'))
    app = rf.App(robot={"urdf": str(d / "float.urdf"), "floating_base": True, "position": [0, 0, 1.0]}, callbacks=[rf.Checks()], realtime=False)
    app.run(duration=3.0)
    z = app.bus.latest("state/pose").position[2]
    assert 0.0 < z < 0.2, f"a floating URDF should come to rest on the ground, z={z:.2f}"
    app.close()
    # without collision shapes it falls for ever: the checks must say why
    chk = rf.Checks()
    app = rf.App(robot={"urdf": str(d / "ok.urdf"), "floating_base": True, "position": [0, 0, 1.0]}, callbacks=[chk], realtime=False)
    app.run(duration=4.0)
    app.close()
    assert any(c == "falling" for c, _ in chk.findings), f"free fall was not reported: {chk.findings}"
    # MuJoCo drops a URDF's <visual> geometry unless told otherwise, so a missing visual mesh is not an error...
    (d / "mesh.urdf").write_text(URDF % '<mesh filename="package://my_robot/meshes/link1.stl"/>')
    app = rf.App(robot={"urdf": str(d / "mesh.urdf")}, realtime=False)
    app.run(steps=20)
    app.close()
    # ...but a missing collision mesh is, and the error must name the file
    (d / "cmesh.urdf").write_text((URDF % '<box size="0.1 0.1 0.1"/>').replace("</visual>", '</visual><collision><geometry><mesh filename="meshes/link1.stl"/></geometry></collision>'))
    out.append(clear_error(lambda: rf.App(robot={"urdf": str(d / "cmesh.urdf")}, realtime=False).run(steps=5), "link1.stl"))
    (d / "empty.xml").write_text("<mujoco><worldbody/></mujoco>")
    app = rf.App(robot={"mjcf": str(d / "empty.xml")}, realtime=False)            # a world with nothing in it still steps
    app.run(steps=50)
    app.close()
    out.append(clear_error(lambda: rf.App(robot={"mjcf": str(d / "empty.xml"), "base": "torso"}, realtime=False).run(steps=5), "torso"))
    out.append(clear_error(lambda: rf.App(robot={"mjcf": str(d / "empty.xml"), "profile": "humanoid"}, realtime=False), "humanoid", "articulated"))
    (d / "h.xml").write_text(humanoid(24))
    app = rf.App(robot={"mjcf": str(d / "h.xml"), "base": "torso"}, controllers=[rf.JointPD(kp=8, ki=0, kd=0.4, rate=500, target=np.zeros(24))], realtime=False)
    t0 = time.time()
    app.run(duration=5.0)
    wall = time.time() - t0
    js, imu = app.bus.latest("state/joint"), app.bus.latest("sensor/imu")
    assert len(js.position) == 24 and np.all(np.isfinite(js.position)), "24-joint robot state"
    assert np.all(np.isfinite(imu.accel)) and np.all(np.isfinite(imu.gyro)), "a model without an IMU sensor must still get a finite, derived one"
    env = app.as_env()
    assert env.action_space.shape == (24,) and env.observation_space.shape == (24 * 2 + 3 + 4 + 6,)
    app.close()
    return f"24-joint free-base robot at x{5 / wall:.0f} realtime; " + " | ".join(o[:60] for o in out)


@scenario(timeout=60)
def models_unstable_simulation_fails_fast():
    """Gains far too high: the run must end with an explanation, not hang and not carry on with nan."""
    d = tmp()
    xml = (rf.robot.ZOO / "diffdrive.xml").read_text()
    xml = xml.replace('integrator="implicitfast"', 'integrator="Euler"').replace('kv="0.5"', 'kv="60"').replace(' armature="0.005"', "")
    (d / "stiff.xml").write_text(xml)
    robot = rf.Robot.from_mjcf(d / "stiff.xml", profile="diffdrive", base="chassis")
    app = rf.App(robot=robot, controllers=[rf.controller(lambda o, t: rf.velocity_command((0.4, 0, 0), (0, 0, 0.5)), rate=10), rf.DiffDriveController()], realtime=False)
    t0 = time.time()
    msg = clear_error(lambda: app.run(duration=600), "unstable", "timestep")
    assert time.time() - t0 < 20, "took too long to notice"
    app.close()
    # a clipped actuator cannot blow up: absurd gains on the arm stay finite, they just saturate
    arm = rf.App(robot="arm", controllers=[rf.JointPD(kp=5e6, ki=0, kd=0, rate=500, target=[1.0, 1.0, -1.0])], realtime=False)
    arm.run(duration=20)
    q = arm.bus.latest("state/joint").position
    arm.close()
    assert np.all(np.isfinite(q)) and np.abs(q).max() < 10, f"arm with absurd gains: {q}"
    return msg[:110]


def cli(*args, cwd=None):
    env = dict(os.environ, PYTHONPATH=str(pathlib.Path(rf.__file__).parents[1]))
    return subprocess.run([PY, "-m", "roboforge", *args], capture_output=True, text=True, cwd=cwd, timeout=120, env=env)


@scenario(timeout=180)
def cli_errors_are_one_line():
    """Whatever is wrong, the command line prints one line that says so and exits non-zero: never a traceback."""
    d = tmp()
    good = "roboforge: 1\nrobot: arm\nrealtime: false\ncontrollers:\n  - {type: joint_pd}\n"
    cases = {
        "missing file": (["run", "nope.yaml"], "nope.yaml"),
        "bad yaml": (["run", "bad.yaml"], "bad.yaml"),
        "unknown key": (["info", "typo.yaml"], "controler"),
        "unknown type": (["run", "type.yaml", "--headless"], "joint_pid"),
        "bad argument": (["run", "arg.yaml", "--headless"], "kpp"),
        "bad override": (["run", "good.yaml", "--headless", "physics.substeps"], "path=value"),
        "override index": (["run", "good.yaml", "--headless", "controllers.4.rate=3"], "controllers"),
        "user code raises": (["run", "raises.yaml", "--headless", "--duration", "1"], "ZeroDivisionError"),
        "unstable": (["run", "stiff.yaml", "--headless", "--duration", "600"], "unstable"),
        "port in use": (["run", "port.yaml", "--headless", "--duration", "1"], "already"),
        "export of nothing": (["data", "export", "nowhere", "--format", "nerfstudio", "--out", "out"], "nowhere"),
        "no command": ([], "usage"),
        "unknown command": (["fly"], "fly"),
    }
    (d / "good.yaml").write_text(good)
    (d / "bad.yaml").write_text("robot: [arm\n")
    (d / "typo.yaml").write_text(good.replace("controllers", "controler"))
    (d / "type.yaml").write_text(good.replace("joint_pd", "joint_pid"))
    (d / "arg.yaml").write_text(good.replace("{type: joint_pd}", "{type: joint_pd, kpp: 3}"))
    (d / "raises.yaml").write_text(good.replace("joint_pd", '"boom:Boom"'))
    xml = (rf.robot.ZOO / "diffdrive.xml").read_text()
    (d / "stiff.xml").write_text(xml.replace('integrator="implicitfast"', 'integrator="Euler"').replace('kv="0.5"', 'kv="60"').replace(' armature="0.005"', ""))
    (d / "spin.py").write_text("import roboforge as rf\nclass Spin(rf.Controller):\n    def step(self, obs, t):\n        return rf.velocity_command((0.4, 0, 0), (0, 0, 0.5))\n")
    (d / "stiff.yaml").write_text("roboforge: 1\nrobot: {mjcf: stiff.xml, profile: diffdrive, base: chassis}\nrealtime: false\ncontrollers:\n  - {type: \"spin:Spin\", rate: 10}\n  - {type: diffdrive}\n")
    import socket
    busy = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    busy.bind(("127.0.0.1", 0))
    (d / "port.yaml").write_text(f"roboforge: 1\nphysics: {{type: none}}\nlinks:\n  - {{type: mavlink, mode: client, url: \"udpin:127.0.0.1:{busy.getsockname()[1]}\"}}\n")
    (d / "boom.py").write_text("import roboforge as rf\nclass Boom(rf.Controller):\n    def step(self, obs, t):\n        return 1 / 0\n")
    bad = []
    for name, (args, hint) in cases.items():
        p = cli(*args, cwd=d)
        text = p.stdout + p.stderr
        if p.returncode == 0:
            bad.append(f"{name}: exit 0")
        elif name != "user code raises" and "Traceback" in text:
            bad.append(f"{name}: traceback ({text.strip().splitlines()[-1][:80]})")
        elif hint.lower() not in text.lower():
            bad.append(f"{name}: no mention of {hint!r} ({text.strip()[-90:]!r})")
    # user code failing is the one place a traceback helps: it must point into the user's file
    p = cli("run", "raises.yaml", "--headless", "--duration", "1", cwd=d)
    if "boom.py" not in p.stdout + p.stderr or "Boom" not in p.stdout + p.stderr:
        bad.append("user code raises: the user's file and controller are not shown")
    ok = cli("run", "good.yaml", "--headless", "--duration", "1", cwd=d)
    assert ok.returncode == 0, ok.stderr[-300:]
    assert not bad, "; ".join(bad)
    return f"{len(cases)} kinds of mistake, each one line and a non-zero exit"


@scenario(timeout=60)
def threads_app_running_at_interpreter_exit():
    """A script that leaves an app running in a background thread and exits must exit, not crash in native code."""
    code = ("import threading, time, roboforge as rf\n"
            "app = rf.App(robot='quadrotor', renderer=rf.MujocoRenderer(width=64, height=48, fps=30), controllers=[rf.MultirotorController(rate=250)], realtime=False)\n"
            "app.setup()\n"
            "threading.Thread(target=app.run, daemon=True).start()\n"
            "time.sleep(1.0)\n"
            "print('steps', app.steps)\n")
    codes = []
    for i in range(8):
        p = subprocess.run([PY, "-c", code], capture_output=True, text=True, timeout=30, env=dict(os.environ, PYTHONPATH=str(pathlib.Path(rf.__file__).parents[1])))
        codes.append(p.returncode)
    assert all(c == 0 for c in codes), f"exit codes over 8 runs: {codes} (-11 is a segfault)"
    return "8 clean exits"


@scenario(timeout=120)
def threads_bus_under_contention():
    """Eight threads publishing while subscribers come and go and one subscriber raises."""
    bus = rf.Bus()
    seen = {"n": 0}
    lock = threading.Lock()

    def count(name, msg):
        with lock:
            seen["n"] += 1
    bus.subscribe("load/*", count)
    N, T = 20000, 8
    errors = []

    def pub(k):
        try:
            for i in range(N):
                bus.publish(f"load/{k}", i, source=f"t{k}")
        except Exception as e:
            errors.append(e)

    def churn():
        for i in range(3000):
            bus.subscribe("load/0", lambda n, m: None)()
    ths = [threading.Thread(target=pub, args=(k,)) for k in range(T)] + [threading.Thread(target=churn) for _ in range(2)]
    [t.start() for t in ths]; [t.join() for t in ths]
    assert not errors, errors[:2]
    assert seen["n"] == N * T, f"subscriber saw {seen['n']} of {N * T} messages"
    assert all(bus.count(f"load/{k}") == N for k in range(T)), "publish counts are wrong"
    # a subscriber that raises: the publisher hears about it, and the other subscribers still ran
    got = []
    bus.subscribe("x", lambda n, m: 1 / 0)
    bus.subscribe("x", lambda n, m: got.append(m))
    try:
        bus.publish("x", 1)
    except ZeroDivisionError:
        pass
    else:
        raise AssertionError("a failing subscriber was swallowed silently")
    assert got == [1], "one failing subscriber stopped the others from running"
    assert bus.latest("x") == 1
    return f"{N * T} messages from {T} threads, none lost"
