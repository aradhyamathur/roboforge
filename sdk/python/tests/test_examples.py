"""Every bundled example starts and runs, the way a user runs it."""
import pathlib
import subprocess
import sys

import pytest

pytest.importorskip("mujoco")

EXAMPLES = pathlib.Path(__file__).resolve().parents[1] / "examples"
APPS = sorted(p.parent.name for p in EXAMPLES.glob("*/app.yaml"))


def test_examples_are_found():
    assert {"mujoco_ros_arm", "mujoco_mavlink_quad", "ros_mavlink_bridge", "diffdrive_ros"} <= set(APPS)


@pytest.mark.parametrize("name", APPS)
def test_example_app_runs(name, tmp_path):
    import shutil
    folder = tmp_path / name                              # a copy: apps may record into their folder
    shutil.copytree(EXAMPLES / name, folder)
    if "ros2" in (folder / "app.yaml").read_text():
        pytest.importorskip("rclpy")
    out = subprocess.run(
        [sys.executable, "-m", "roboforge", "run", "app.yaml", "--headless", "--duration", "1"],
        capture_output=True, text=True, cwd=folder, timeout=120)
    assert out.returncode == 0, out.stderr[-2000:]
    assert "stopped at t=" in out.stdout + out.stderr


def test_user_controller_takes_rate_from_config():
    import roboforge as rf

    class Mine(rf.Controller):
        def step(self, obs, t):
            return None

    assert Mine(rate=50).rate == 50
    assert Mine().rate is None


@pytest.mark.parametrize("sig", ["SIGINT", "SIGTERM"])
def test_stop_signal_shuts_the_app_down_cleanly(sig):
    import signal
    import time
    folder = EXAMPLES / "mujoco_mavlink_quad"
    p = subprocess.Popen([sys.executable, "-m", "roboforge", "run", "app.yaml", "--headless", "--duration", "60"],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, cwd=folder)
    assert "running" in p.stdout.readline()
    time.sleep(1.0)
    p.send_signal(getattr(signal, sig))
    out = p.communicate(timeout=30)[0]
    assert p.returncode == 0, out[-2000:]
    assert "stopped at t=" in out and "Traceback" not in out


def test_wheeled_base_example_drives_in_a_circle(monkeypatch):
    """The user controller's velocity command reaches the wheels (it once went to state/twist)."""
    import numpy as np
    import roboforge as rf
    folder = EXAMPLES / "diffdrive_ros"
    monkeypatch.chdir(folder)
    monkeypatch.syspath_prepend(str(folder))
    monkeypatch.delitem(sys.modules, "controller", raising=False)   # each example has its own controller.py
    app = rf.App.from_config("app.yaml", links=[], callbacks=[], realtime=False)
    app.run(duration=6)
    pose, cmd = app.bus.latest("state/pose"), app.bus.latest("cmd/velocity")
    app.close()
    assert cmd is not None and cmd.linear[0] == pytest.approx(0.3)
    assert np.linalg.norm(pose.position[:2]) > 0.5 and abs(pose.position[1]) > 0.2   # moved, and turned


def test_arm_example_follows_its_wave(monkeypatch):
    import roboforge as rf
    folder = EXAMPLES / "mujoco_ros_arm"
    monkeypatch.chdir(folder)
    monkeypatch.syspath_prepend(str(folder))
    monkeypatch.delitem(sys.modules, "controller", raising=False)   # each example has its own controller.py
    app = rf.App.from_config("app.yaml", links=[], callbacks=[], realtime=False)
    app.run(duration=4)
    q = app.bus.latest("state/joint").position
    app.close()
    assert abs(q[2] + 0.9) < 0.1 and abs(q[1]) > 0.1


SCRIPTS = {   # script -> text its output must contain
    "first_app.py": "reached:",
    "pid_to_policy.py": "PID loop ran at 500 Hz throughout",
    "record_and_export.py": "recorded 251 frames",
    "gym_reach.py": "worst joint error 0.0",
    "render_pass.py": "depth_tint",
    "debugging/signal_tap.py": "state/pose arrives at 500 Hz",
    "debugging/step_through.py": "replayed to t=",
    "debugging/compare_runs.py": "identical, sample for sample",
    "debugging/health_check.py": "verdict:",
    "custom_policy.py": "worst error 0.0",
    "custom_renderer/compare.py": "same camera pose in both: True",
    "custom_solver/compare.py": "agree",
    "custom_slam/evaluate.py": "map: 8 of 8 landmarks found",
    "sb3_ppo.py": "deployed through rf.Policy",
}


@pytest.mark.parametrize("script", sorted(SCRIPTS))
def test_example_script_runs(script, tmp_path):
    if script == "sb3_ppo.py":
        pytest.importorskip("stable_baselines3")
    args = ["8", "--headless"] if script == "pid_to_policy.py" else []
    out = subprocess.run([sys.executable, str(EXAMPLES / script), *args], capture_output=True, text=True, cwd=tmp_path, timeout=180)
    assert out.returncode == 0, out.stderr[-2000:]
    assert SCRIPTS[script] in out.stdout, out.stdout[-1500:]


def test_every_example_is_covered_by_a_test():
    scripts = {str(p.relative_to(EXAMPLES)) for p in EXAMPLES.rglob("*.py")
               if '__name__ == "__main__"' in p.read_text() and p.name != "mission.py"}
    assert scripts - set(SCRIPTS) == {"debugging/mavlink_sniffer.py"}, "add new example scripts to SCRIPTS (the sniffer has its own test)"


def test_broken_app_tutorial_reports_each_mistake_in_turn(tmp_path):
    """The debugging tutorial: run, read the message, fix one thing, run again."""
    import shutil
    for f in (EXAMPLES / "debugging" / "broken_arm").iterdir():
        shutil.copy(f, tmp_path / f.name)
    app = tmp_path / "app.yaml"

    def run():
        p = subprocess.run([sys.executable, "-m", "roboforge", "run", "app.yaml", "--headless", "--duration", "5", "--check", "--no-realtime"],
                           capture_output=True, text=True, cwd=tmp_path, timeout=120)
        return p.returncode, p.stdout + p.stderr

    def fix(old, new):
        text = app.read_text()
        assert old in text
        app.write_text(text.replace(old, new))

    steps = [("Did you mean 'controllers'?", "controlers:", "controllers:"),
             ("has no argument 'rte'. Did you mean 'rate'?", "rte: 20", "rate: 20"),
             ("no camera named 'overveiw'. Its cameras: overview", "overveiw", "overview"),
             ("nothing has published cmd/actuators", "rate: 20}", "rate: 20}\n  - {type: joint_pd, rate: 500}"),
             ("at their limits", "rate: 20}", "rate: 20, degrees: false}")]
    for message, old, new in steps:
        code, out = run()
        assert message in out, f"expected {message!r} in:\n{out[-800:]}"
        assert "Traceback" not in out
        fix(old, new)
    code, out = run()
    assert code == 0 and "roboforge check" not in out and "error" not in out, out[-800:]
    fixed = subprocess.run([sys.executable, "-m", "roboforge", "run", "app_fixed.yaml", "--headless", "--duration", "5", "--check", "--no-realtime"],
                           capture_output=True, text=True, cwd=tmp_path, timeout=120)
    assert fixed.returncode == 0 and "roboforge check" not in fixed.stdout, fixed.stdout[-800:]


def test_mavlink_sniffer_sees_the_vehicle(free_port):
    pytest.importorskip("pymavlink")
    port = free_port()
    app = subprocess.Popen([sys.executable, "-m", "roboforge", "run", "app.yaml", "--headless", "--duration", "20",
                            f"links.0.urls=[udpout:127.0.0.1:{port}]"], cwd=EXAMPLES / "mujoco_mavlink_quad",
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        out = subprocess.run([sys.executable, str(EXAMPLES / "debugging" / "mavlink_sniffer.py"), f"udpin:127.0.0.1:{port}", "3"],
                             capture_output=True, text=True, timeout=60)
    finally:
        app.terminate()
        app.wait(timeout=20)
    assert out.returncode == 0, out.stdout + out.stderr
    assert "HEARTBEAT" in out.stdout and "LOCAL_POSITION_NED" in out.stdout and "disarmed" in out.stdout, out.stdout
    silent = subprocess.run([sys.executable, str(EXAMPLES / "debugging" / "mavlink_sniffer.py"), f"udpin:127.0.0.1:{free_port()}", "1"],
                            capture_output=True, text=True, timeout=60)
    assert silent.returncode == 1 and "nothing arrived" in silent.stdout


def test_urdf_gets_motors_ground_and_an_optional_free_base():
    import roboforge as rf
    urdf = str(EXAMPLES / "custom_urdf" / "twolink.urdf")
    app = rf.App(robot={"urdf": urdf, "profile": "arm"}, controllers=[rf.JointPD(rate=500, kp=40, ki=10, kd=4, target=[0.5, 1.0])], realtime=False)
    app.run(duration=4)
    assert app.physics.actuator_names == ["shoulder", "elbow"]
    assert app.physics.ctrl_range.tolist() == [[-20.0, 20.0], [-12.0, 12.0]], "motors take the URDF's effort limits"
    q = app.bus.latest("state/joint").position
    app.close()
    assert abs(q[0] - 0.5) < 0.05 and abs(q[1] - 1.0) < 0.05
    free = rf.App(robot={"urdf": urdf, "floating_base": True, "position": [0, 0, 1.0]}, realtime=False)
    free.run(duration=3)
    z = float(free.bus.latest("state/pose").position[2])
    free.close()
    assert 0.0 < z < 0.4, f"a free-floating URDF should fall onto the ground that was added for it, z={z:.2f}"


def test_tap_and_checks_callbacks():
    import roboforge as rf
    tap, checks = rf.Tap(every=0.05), rf.Checks()
    app = rf.App(robot="arm", controllers=[rf.controller(lambda o, t: rf.JointCommand(position=[0.1, 0.2, -0.3]), rate=20)],
                 callbacks=[tap, checks], realtime=False)
    app.run(duration=2.5)
    rows = {r["name"]: r for r in tap.rows(app)}
    app.close()
    assert "cmd/joint" in rows and rows["cmd/joint"]["sources"] and rows["state/joint"]["type"] == "JointState"
    assert [c for c, _ in checks.findings] == ["no-command"], checks.findings
