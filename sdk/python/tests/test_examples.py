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
def test_example_app_runs(name):
    folder = EXAMPLES / name
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
