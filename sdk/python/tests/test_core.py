"""Bus, registry and config: no simulator needed."""
import threading

import numpy as np
import pytest

import roboforge as rf
from roboforge import config, frames, msgs
from roboforge.registry import Registry


def test_bus_latest_subscribe_and_wildcards():
    bus = rf.Bus()
    seen = []
    off = bus.subscribe("sensor/*", lambda n, m: seen.append((n, m)))
    bus.subscribe("clock", lambda n, m: seen.append((n, m)))
    bus.publish("sensor/imu", 1, source="a")
    bus.publish("clock", 2)
    bus.publish("state/pose", 3)
    assert seen == [("sensor/imu", 1), ("clock", 2)]
    assert bus.latest("state/pose") == 3 and bus.latest("nope", "d") == "d"
    off()
    bus.publish("sensor/imu", 4)
    assert len(seen) == 2 and bus.count("sensor/imu") == 2
    row = next(r for r in bus.describe() if r["name"] == "sensor/imu")
    assert row["sources"] == ["a"] and row["count"] == 2 and row["type"] == "int"


def test_bus_is_safe_across_threads():
    bus = rf.Bus()
    total = []
    bus.subscribe("x", lambda n, m: total.append(m))
    threads = [threading.Thread(target=lambda: [bus.publish("x", 1) for _ in range(500)]) for _ in range(8)]
    [t.start() for t in threads]
    [t.join() for t in threads]
    assert bus.count("x") == 4000 and len(total) == 4000


def test_registry_names_paths_and_errors():
    reg = Registry("thing")
    reg.register("pid", "roboforge.control.pid:PID")
    assert type(reg.build({"type": "pid", "kp": 3.0})).__name__ == "PID"
    assert reg.build({"type": "roboforge.control.pid:PID", "kp": 2.0}).kp == 2.0
    obj = object()
    assert reg.build(obj) is obj and reg.build(None) is None
    with pytest.raises(rf.ConfigError, match="unknown thing 'nope'"):
        reg.get("nope")


def test_overrides_and_unknown_keys():
    cfg = {"robot": "arm", "physics": "mujoco", "links": [{"type": "mavlink", "url": "a"}]}
    out, _ = config.resolve(cfg, ["physics.substeps=2", "links.0.url=udpout:1.2.3.4:5", "realtime=false"])
    assert out["physics"] == {"type": "mujoco", "substeps": 2}
    assert out["links"][0]["url"] == "udpout:1.2.3.4:5" and out["realtime"] is False
    assert cfg["physics"] == "mujoco", "the caller's dict must not be modified"
    with pytest.raises(ValueError, match="unknown config keys: phsyics"):
        config.resolve({"phsyics": "mujoco"})
    with pytest.raises(ValueError, match="path=value"):
        config.resolve({}, ["nonsense"])


def test_frame_conversions_round_trip():
    q = frames.mat_to_quat(frames.euler_to_mat(0.2, -0.3, 1.1))
    assert np.allclose(frames.quat_to_euler(q), [0.2, -0.3, 1.1], atol=1e-9)
    assert np.allclose(frames.attitude_ned_to_enu(frames.attitude_enu_to_ned(q)), q, atol=1e-9)
    assert np.allclose(frames.enu_to_ned([1, 2, 3]), [2, 1, -3])
    # Facing east (ENU yaw 0) is NED yaw +90 deg; level stays level.
    r, p, y = frames.mat_to_euler(frames.attitude_enu_to_ned([1, 0, 0, 0]))
    assert np.allclose([r, p, y], [0, 0, np.pi / 2], atol=1e-9)


def test_pid_converges_and_limits():
    pid = rf.PID(kp=2.0, ki=1.0, kd=0.0, limit=5.0)
    x = 0.0
    for _ in range(4000):
        x += pid(1.0 - x, 0.01) * 0.01
    assert abs(x - 1.0) < 1e-2
    assert rf.PID(kp=100.0, limit=5.0)(10.0, 0.01) == 5.0


def test_messages_know_their_signal():
    assert rf.ActuatorCommand.signal == "cmd/actuators" and rf.Setpoint.signal == msgs.CMD_SETPOINT
    img = rf.Image(data=np.zeros((240, 320, 3), np.uint8), fovy=90.0)
    assert img.intrinsics["w"] == 320 and abs(img.intrinsics["fl_y"] - 120.0) < 1e-6
