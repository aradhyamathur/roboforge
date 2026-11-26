"""MuJoCo physics, the bundled robots, control, rendering and recording."""
import json
import os
import subprocess
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

import roboforge as rf  # noqa: E402
from roboforge.frames import yaw_of  # noqa: E402


@pytest.mark.parametrize("platform,display,explicit,expected", [
    ("darwin", None, None, None),
    ("linux", None, None, "egl"),
    ("linux", ":0", None, None),
    ("linux", None, "osmesa", "osmesa"),
])
def test_graphics_default_respects_platform_and_environment(platform, display, explicit, expected):
    env = os.environ.copy()
    for key in ("DISPLAY", "WAYLAND_DISPLAY", "MUJOCO_GL"):
        env.pop(key, None)
    if display is not None:
        env["DISPLAY"] = display
    if explicit is not None:
        env["MUJOCO_GL"] = explicit
    code = f"""
import os, sys, types
sys.platform = {platform!r}
sys.modules["mujoco"] = types.ModuleType("mujoco")
import roboforge.physics.mujoco
assert os.environ.get("MUJOCO_GL") == {expected!r}
"""
    subprocess.run([sys.executable, "-c", code], env=env, check=True)


@pytest.mark.parametrize("platform,display,opens", [
    ("darwin", None, True),
    ("linux", None, False),
    ("linux", ":0", True),
])
def test_viewer_display_detection(monkeypatch, platform, display, opens):
    import mujoco.viewer

    monkeypatch.setattr(sys, "platform", platform)
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    if display is not None:
        monkeypatch.setenv("DISPLAY", display)
    launched = []
    handle = SimpleNamespace(close=lambda: None)
    monkeypatch.setattr(mujoco.viewer, "launch_passive", lambda model, data: launched.append((model, data)) or handle)
    app = SimpleNamespace(physics=SimpleNamespace(model=object(), data=object()))
    viewer = rf.Viewer()
    viewer.on_start(app)
    assert bool(launched) == opens
    viewer.on_stop(app)


def test_arm_follows_joint_targets():
    with rf.App(robot="arm", controllers=[rf.JointPD(rate=500)], realtime=False) as app:
        app.bus.publish("cmd/joint", rf.JointCommand(position=np.array([0.5, 0.6, -0.8])))
        app.run(duration=4.0)
        js = app.bus.latest("state/joint")
        assert js.names == ["yaw", "shoulder", "elbow"]
        assert np.allclose(js.position, [0.5, 0.6, -0.8], atol=0.02), js.position
        assert abs(app.time - 4.0) < 1e-6 and app.steps == 2000


def test_a_ten_line_pid_controller_is_enough():
    """The simplest controller: a function with a PID in it."""
    pid = rf.PID(kp=40.0, ki=15.0, kd=5.0)

    def hold_elbow(obs, t):
        js = obs.joint
        u = np.zeros(3)
        u[2] = float(pid(-1.0 - js.position[2], 0.002, rate=-js.velocity[2]))
        return rf.ActuatorCommand(values=u)

    with rf.App(robot="arm", controller=hold_elbow, realtime=False) as app:
        app.run(duration=3.0)
        assert abs(app.bus.latest("state/joint").position[2] + 1.0) < 0.03


def test_quadrotor_flies_to_setpoint_and_yaw():
    with rf.App(robot="quadrotor", controllers=[rf.MultirotorController(rate=250, hover=[0, 0, 1.5])], realtime=False) as app:
        app.run(duration=6.0)
        assert abs(app.bus.latest("state/pose").position[2] - 1.5) < 0.15
        app.bus.publish("cmd/setpoint", rf.Setpoint(position=np.array([2.0, 1.0, 2.0]), yaw=1.0))
        app.run(duration=10.0)
        pose = app.bus.latest("state/pose")
        assert np.allclose(pose.position, [2.0, 1.0, 2.0], atol=0.15), pose.position
        assert abs(yaw_of(pose.orientation) - 1.0) < 0.05
        # At hover the accelerometer reads +g along body z and the gyro reads nothing.
        imu = app.bus.latest("sensor/imu")
        assert abs(imu.accel[2] - 9.81) < 0.6 and np.linalg.norm(imu.gyro) < 0.2


def test_quadrotor_stays_down_until_armed():
    with rf.App(robot="quadrotor", controllers=[rf.MultirotorController(rate=250, hover=[0, 0, 1.5])], realtime=False) as app:
        app.bus.publish("state/status", rf.VehicleStatus(armed=False))
        app.run(duration=1.0)
        assert app.bus.latest("state/pose").position[2] < 0.1
        assert not np.any(app.bus.latest("cmd/actuators").values)


def test_diffdrive_tracks_velocity_and_stops_on_timeout():
    with rf.App(robot="diffdrive", controllers=[rf.DiffDriveController(timeout=0.5)], realtime=False) as app:
        app.setup()
        for _ in range(8):                                   # keep commanding for 2 s
            app.bus.publish("cmd/velocity", rf.velocity_command((0.4, 0, 0), (0, 0, 0.5)))
            app.run(duration=0.25)
        tw = app.bus.latest("state/twist")
        assert abs(np.linalg.norm(tw.linear[:2]) - 0.4) < 0.05 and abs(tw.angular[2] - 0.5) < 0.06
        app.run(duration=2.0)                                # no more commands: it must stop
        assert np.linalg.norm(app.bus.latest("state/twist").linear[:2]) < 0.02


def test_unstable_simulation_raises_instead_of_hanging(tmp_path):
    """A model whose actuator gain is too stiff for its timestep blows up. MuJoCo then
    resets the state, which used to make a timed run spin forever; it must raise."""
    xml = (rf.robot.ZOO / "diffdrive.xml").read_text()
    xml = xml.replace('integrator="implicitfast"', 'integrator="Euler"').replace('kv="0.5"', 'kv="60"').replace(' armature="0.005"', "")
    model = tmp_path / "stiff.xml"
    model.write_text(xml)
    robot = rf.Robot.from_mjcf(model, profile="diffdrive", base="chassis", params={"wheel_radius": 0.05, "wheel_separation": 0.3})
    app = rf.App(robot=robot, controllers=[rf.DiffDriveController(timeout=100)], realtime=False)
    app.setup()
    app.bus.publish("cmd/velocity", rf.velocity_command((0.4, 0, 0), (0, 0, 0.5)))
    with pytest.raises(RuntimeError, match="became unstable"):
        app.run(duration=5.0)
    app.close()


def test_custom_urdf_loads_as_articulated(tmp_path):
    urdf = tmp_path / "pendulum.urdf"
    urdf.write_text("""<robot name="pendulum">
  <link name="base"><inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/></inertial></link>
  <link name="rod"><inertial><origin xyz="0 0 -0.25"/><mass value="0.5"/><inertia ixx="0.01" iyy="0.01" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="0 0 -0.25"/><geometry><cylinder radius="0.02" length="0.5"/></geometry></visual></link>
  <joint name="swing" type="revolute"><parent link="base"/><child link="rod"/><origin xyz="0 0 1"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="10"/></joint>
</robot>""")
    with rf.App(robot=rf.Robot.from_urdf(urdf), realtime=False) as app:
        app.setup()
        app.physics.data.qpos[0] = 1.0                       # raw access: start it off-centre
        app.run(duration=1.0)
        js = app.bus.latest("state/joint")
        assert js.names == ["swing"] and abs(js.position[0] - 1.0) > 0.2, "the pendulum should have swung"


def test_controllers_keep_their_own_rates():
    calls = {"fast": 0, "slow": 0}

    class Slow(rf.Controller):
        rate = 5

        def step(self, obs, t):
            calls["slow"] += 1

    @rf.controller(rate=200)
    def fast(obs, t):
        calls["fast"] += 1

    with rf.App(robot="arm", controllers=[Slow(), fast], realtime=False) as app:
        app.run(duration=2.0)
        assert calls["slow"] == 10 and calls["fast"] == 400
        assert abs(app.controller_rate(fast) - 200) < 1


def test_slow_policy_does_not_block_the_fast_loop():
    """The structure a VLA needs: a 2 Hz model with 300 ms latency, emitting action
    chunks, above a 200 Hz PID. The fast loop must not slow down, the chunk must be
    played out, and the fallback must take over when the model stalls."""
    stall = {"on": False}

    class SlowPolicy(rf.Controller):
        rate = 2

        def step(self, obs, t):
            while stall["on"]:
                time.sleep(0.01)
            time.sleep(0.3)
            return rf.Chunk(actions=[rf.JointCommand(position=np.array([0.3 + 0.01 * i, 0.4, -0.5])) for i in range(5)], dt=0.1)

    safe = rf.JointCommand(position=np.zeros(3))
    policy = rf.Async(SlowPolicy(), rate=50, timeout=0.6, fallback=safe)
    pd = rf.JointPD(rate=200)
    with rf.App(robot="arm", controllers=[policy, pd], realtime=True) as app:
        t0 = time.monotonic()
        app.run(duration=3.0)
        wall = time.monotonic() - t0
        assert wall < 3.6, f"3 s of app time took {wall:.2f} s: the slow policy is blocking the loop"
        assert abs(app.controller_rate(pd) - 200) < 5
        assert policy.inferences >= 4 and 0.25 < policy.latency < 0.6
        js = app.bus.latest("state/joint")
        assert abs(js.position[1] - 0.4) < 0.08 and js.position[0] > 0.2, "chunked actions should reach the arm"
        stall["on"] = True                                   # the model hangs
        late = policy.late
        app.run(duration=2.5)
        stall["on"] = False
        assert policy.late > late, "the fallback should have fired"
        assert np.allclose(app.bus.latest("cmd/joint").position, 0.0), "fallback action should be in force"


def test_renderer_passes_and_recorder_exports(tmp_path):
    marks = []

    def invert(img, renderer):
        marks.append(img.camera)
        img.data = 255 - img.data
        return img

    renderer = rf.MujocoRenderer(width=160, height=120, fps=5)
    renderer.add_pass(invert)
    rec = rf.Recorder(dir=tmp_path / "run", rate=50)
    ctrl = rf.MultirotorController(rate=250, hover=[0, 0, 1.0])
    with rf.App(robot="quadrotor", renderer=renderer, controllers=[ctrl], callbacks=[rec], realtime=False) as app:
        app.run(duration=2.0)
        img = app.bus.latest("sensor/camera/front")
        assert img.data.shape == (120, 160, 3) and img.data.std() > 5, "the frame should not be blank"
        renderer.passes.clear()
        plain = renderer.render("front").data.astype(int)
        renderer.add_pass(invert)
        inverted = renderer.render("front").data.astype(int)
        assert np.abs((255 - plain) - inverted).max() <= 2, "the pass should have inverted the frame"   # GPU output varies by +-1
        cam_pos, base = img.pose[:3, 3], app.bus.latest("state/pose").position
        assert np.linalg.norm(cam_pos - base) < 0.2, "camera pose should ride on the vehicle"
    assert set(marks) == {"front"}

    ds = rf.Dataset(tmp_path / "run")
    frames = ds.frames()
    assert ds.cameras == ["front"] and len(frames) == 11 and ds.meta["cameras"]["front"]["w"] == 160
    sig = ds.signals()
    assert sig["state/pose"].shape == (101, 7) and sig["state/pose"][-1, 2] > 0.5
    assert len(ds.times("state/pose")) == 101

    ns = ds.export_nerfstudio(tmp_path / "ns")
    tr = json.loads((ns / "transforms.json").read_text())
    assert len(tr["frames"]) == 11 and tr["w"] == 160 and (ns / tr["frames"][0]["file_path"]).is_file()
    assert np.allclose(tr["frames"][-1]["transform_matrix"], frames[-1]["pose"])
    # The camera looks along -z of its own frame: for the forward camera that is world +x.
    assert np.allclose(-np.array(tr["frames"][0]["transform_matrix"])[:3, 2], [1, 0, 0], atol=0.05)

    tum = ds.export_tum(tmp_path / "tum")
    gt = [l.split() for l in (tum / "groundtruth.txt").read_text().splitlines()[1:]]
    assert len(gt) == 11 and len(gt[0]) == 8 and abs(float(gt[-1][3]) - frames[-1]["pose"][2][3]) < 1e-5
    assert len((tum / "rgb.txt").read_text().splitlines()) == 12 and (tum / "rgb" / "000010.png").stat().st_size > 200


def test_gym_env_wraps_a_stack():
    pytest.importorskip("gymnasium")
    env = rf.App(robot="arm").as_env(steps_per_action=5, max_steps=20,
                                     reward=lambda obs: -abs(obs.joint.position[1] - 0.5))
    obs, _ = env.reset()
    assert obs.shape == (19,) and env.action_space.shape == (3,)   # 3+3 joints, 7 pose, 6 twist
    total, truncated = 0.0, False
    for _ in range(20):
        obs, r, terminated, truncated, _ = env.step(env.action_space.sample())
        total += r
    assert truncated and total < 0 and not terminated
    env.close()


def test_config_file_with_user_controller(tmp_path):
    (tmp_path / "my_ctrl.py").write_text(
        "import numpy as np\nimport roboforge as rf\n\n"
        "class Wave(rf.Controller):\n    rate = 20\n    def __init__(self, amp=0.5):\n        self.amp = amp\n"
        "    def step(self, obs, t):\n        return rf.JointCommand(position=np.array([self.amp, 0.3, -0.3]))\n")
    (tmp_path / "app.yaml").write_text(
        "roboforge: 1\nname: wave\nrobot: arm\nphysics: {type: mujoco}\nrealtime: false\n"
        "controllers:\n  - {type: 'my_ctrl:Wave', amp: 0.7}\n  - {type: joint_pd, rate: 500}\nrun: {duration: 3}\n")
    with rf.App.from_config(tmp_path / "app.yaml") as app:
        app.run(duration=3.0)
        assert app.name == "wave" and abs(app.bus.latest("state/joint").position[0] - 0.7) < 0.03
    with rf.App.from_config(tmp_path / "app.yaml", ["controllers.0.amp=-0.4"]) as app:
        app.run(duration=3.0)
        assert abs(app.bus.latest("state/joint").position[0] + 0.4) < 0.03
