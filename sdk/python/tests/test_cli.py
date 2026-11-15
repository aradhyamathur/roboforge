"""The command line, run the way a user or the IDE runs it."""
import json
import subprocess
import sys

import pytest

pytest.importorskip("mujoco")

CONFIG = """roboforge: 1
name: quad-demo
robot: quadrotor
physics: {type: mujoco}
renderer: {type: mujoco, width: 96, height: 72, fps: 5}
links:
  - {type: mavlink, mode: vehicle, url: "udpout:127.0.0.1:14599"}
controllers:
  - {type: multirotor, rate: 250, hover: [0, 0, 1.0]}
callbacks:
  - {type: viewer}
  - {type: recorder, dir: runs/cli}
run: {duration: 1.5}
"""


def cli(*args, cwd=None):
    return subprocess.run([sys.executable, "-m", "roboforge", *args], capture_output=True, text=True, cwd=cwd, timeout=120)


def test_info_describes_the_wiring_without_starting_links(tmp_path):
    (tmp_path / "app.yaml").write_text(CONFIG)
    r = cli("info", "app.yaml", "--json", cwd=tmp_path)
    assert r.returncode == 0, r.stderr
    info = json.loads(r.stdout)
    assert info["name"] == "quad-demo" and info["robot"]["profile"] == "multirotor"
    assert info["physics"]["type"] == "mujoco" and info["physics"]["actuators"] == ["motor_fl", "motor_rr", "motor_fr", "motor_rl"]
    link = info["links"][0]
    assert link["type"] == "mavlink" and link["mode"] == "vehicle" and "cmd/setpoint" in link["outputs"] and "state/pose" in link["inputs"]
    names = {s["name"]: s for s in info["signals"]}
    assert {"clock", "state/pose", "sensor/imu", "sensor/camera/front", "cmd/actuators"} <= set(names)
    assert names["cmd/actuators"]["sources"] == ["MultirotorController"] and names["state/pose"]["sources"] == ["mujoco"]
    assert info["controllers"][0]["rate"] == 250
    assert not (tmp_path / "runs").exists(), "info must not record anything"


def test_run_with_overrides_records_a_dataset(tmp_path):
    (tmp_path / "app.yaml").write_text(CONFIG)
    # Without the MAVLink link nothing holds the vehicle disarmed, so it climbs to the hover point.
    r = cli("run", "app.yaml", "controllers.0.hover=[0,0,0.6]", "links=[]", "--headless", "--no-realtime", cwd=tmp_path)
    assert r.returncode == 0, r.stderr
    assert "stopped at t=1.50s after 750 steps" in r.stdout
    r = cli("data", "export", "runs/cli", "--format", "nerfstudio", "--out", "ns", cwd=tmp_path)
    assert r.returncode == 0, r.stderr
    tr = json.loads((tmp_path / "ns" / "transforms.json").read_text())
    assert len(tr["frames"]) >= 7 and 0.3 < tr["frames"][-1]["transform_matrix"][2][3] < 0.9


def test_doctor_and_bad_config(tmp_path):
    r = cli("doctor", "--json")
    parts = {p["name"]: p for p in json.loads(r.stdout)["parts"]}
    assert parts["mujoco"]["ok"] and set(parts) == {"mujoco", "mavlink", "ros2", "gym"}
    (tmp_path / "bad.yaml").write_text("robot: arm\ncontrollers:\n  - {type: nope}\n")
    r = cli("run", "bad.yaml", "--headless", cwd=tmp_path)
    assert r.returncode != 0 and "unknown controller 'nope'" in r.stderr
