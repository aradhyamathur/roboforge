"""LeRobot: record a dataset from an app, train a policy, run it as a controller. Skipped without LeRobot."""
import json
import pathlib
import shutil
import subprocess
import sys

import numpy as np
import pytest

import importlib.util

import roboforge as rf  # noqa: E402
from roboforge import lerobot as rl  # noqa: E402  (imports LeRobot only when used)

# A marker, not importorskip at module level: with ROS 2 sourced, its pytest plugins turn a
# module-level skip into "no tests collected" for the whole run.
pytestmark = pytest.mark.skipif(importlib.util.find_spec("lerobot") is None or importlib.util.find_spec("mujoco") is None,
                                reason="LeRobot (or MuJoCo) is not installed in this environment")

EXAMPLE = pathlib.Path(__file__).resolve().parents[1] / "examples" / "lerobot_reach"
SMALL = dict(chunk_size=5, n_action_steps=5, dim_model=32, n_heads=2, dim_feedforward=64, n_encoder_layers=1,
             n_decoder_layers=1, use_vae=False, optimizer_lr=1e-3)


def _expert():
    """The example's Expert, loaded from its file: several examples have a module named `controller`."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("lerobot_reach_controller", EXAMPLE / "controller.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.Expert


def record(root, seconds=12.0, cameras=(), **kw):
    Expert = _expert()
    rec = rf.CALLBACKS.build({"type": "lerobot_recorder", "root": str(root), "fps": 20, "state": ["state/joint"],
                              "environment": ["task/goal"], "action": "cmd/joint", "task": "reach", "episode_seconds": 2.0,
                              "cameras": list(cameras), **kw})
    app = rf.App(robot="arm", controllers=[Expert(period=2.0), rf.JointPD(rate=500)], callbacks=[rec], realtime=False,
                 renderer=rf.MujocoRenderer(width=64, height=48, fps=20) if cameras else None)
    app.run(duration=seconds)
    app.close()
    return rec


def test_recorder_writes_a_lerobot_dataset_with_the_mapping(tmp_path):
    rec = record(tmp_path / "ds")
    assert rec.episodes == 6 and 238 <= rec.frames <= 242
    info = rl.info(tmp_path / "ds")
    assert info["episodes"] == 6 and info["fps"] == 20 and info["robot_type"] == "arm"
    assert info["features"]["observation.state"] == [6] and info["features"]["observation.environment_state"] == [3]
    assert info["features"]["action"] == [3]
    assert info["roboforge"]["state"] == ["state/joint"] and info["roboforge"]["output"] == "joint"
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset("local/ds", root=tmp_path / "ds")
    names = ds.meta.features["observation.state"]["names"]
    assert names[:3] == ["yaw.pos", "shoulder.pos", "elbow.pos"] and names[3:] == ["yaw.vel", "shoulder.vel", "elbow.vel"]
    x = ds[25]
    assert x["task"] == "reach" and x["action"].shape == (3,)


def test_append_mode_and_refusals(tmp_path):
    record(tmp_path / "ds", seconds=4.0)
    with pytest.raises(rf.ConfigError, match="already holds a LeRobot dataset"):
        record(tmp_path / "ds", seconds=1.0)
    rec = record(tmp_path / "ds", seconds=4.0, mode="append")
    assert rl.info(tmp_path / "ds")["episodes"] == 4 and rec.episodes == 2
    with pytest.raises(rf.ConfigError, match="recorded with environment"):
        record(tmp_path / "ds", seconds=1.0, mode="append", environment=[])
    for bad, words in (({"action": "cmd/nope"}, "action must be one of"), ({"fps": 12.5}, "whole number"),
                       ({"state": [], "environment": [], "cameras": []}, "at least one"), ({"mode": "overwrite"}, "new or append"),
                       ({"fsp": 20}, "Did you mean 'fps'")):
        spec = {"type": "lerobot_recorder", "root": str(tmp_path / "x"), **bad}
        with pytest.raises(rf.ConfigError, match=words):
            rf.CALLBACKS.build(spec)


def test_reset_ends_an_episode(tmp_path):
    rec = rf.CALLBACKS.build({"type": "lerobot_recorder", "root": str(tmp_path / "r"), "fps": 10, "state": ["state/joint"],
                              "action": "cmd/actuators"})
    app = rf.App(robot="arm", controllers=[rf.JointPD(rate=500, target=[0.3, 0.2, -0.4])], callbacks=[rec], realtime=False)
    for _ in range(3):
        app.run(duration=1.0)
        app.reset()
    app.close()
    assert rec.episodes == 3 and rl.info(tmp_path / "r")["features"]["action"] == [3]


def test_train_deploy_and_policy_waits_for_inputs(tmp_path):
    record(tmp_path / "ds", seconds=20.0)
    out = rl.train(tmp_path / "ds", tmp_path / "pol", steps=60, batch_size=16, device="cpu", **SMALL)
    side = json.loads((out / "roboforge.json").read_text())
    assert side["policy"] == "act" and side["environment"] == ["task/goal"] and side["fps"] == 20
    pol = rf.CONTROLLERS.build({"type": "lerobot", "path": str(out)})
    assert pol.rate == 20 and pol.output == "joint"
    app = rf.App(robot="arm", controllers=[pol, rf.JointPD(rate=500)], realtime=False)
    app.run(duration=0.5)                                   # no goal yet: the policy waits
    assert app.bus.latest("cmd/joint") is None
    app.bus.publish("task/goal", np.array([0.3, 0.2, -0.5]))
    app.run(duration=1.0)
    cmd = app.bus.latest("cmd/joint")
    assert cmd is not None and cmd.position.shape == (3,) and np.all(np.isfinite(cmd.position))
    app.reset()                                             # clears LeRobot's action queue
    app.run(duration=0.2)
    app.close()


def test_policy_input_mismatch_and_missing_policy(tmp_path):
    record(tmp_path / "ds", seconds=6.0)
    out = rl.train(tmp_path / "ds", tmp_path / "pol", steps=5, batch_size=8, device="cpu", **SMALL)
    app = rf.App(robot="arm", controllers=[rf.CONTROLLERS.build({"type": "lerobot", "path": str(out), "environment": []})], realtime=False)
    with pytest.raises(rf.ConfigError, match="takes .*observation.environment_state"):
        app.setup()
    with pytest.raises(rf.ConfigError, match="does not exist"):
        rf.App(robot="arm", controllers=[rf.CONTROLLERS.build({"type": "lerobot", "path": str(tmp_path / "a" / "b" / "c")})], realtime=False).setup()
    with pytest.raises(rf.ConfigError, match="not a LeRobot dataset"):
        rl.train(tmp_path / "nowhere", tmp_path / "p2", steps=1)
    with pytest.raises(rf.ConfigError, match="does not take"):
        rl.train(tmp_path / "ds", tmp_path / "p3", steps=1, device="cpu", no_such_option=3)


def test_cameras_record_and_feed_a_vision_policy(tmp_path):
    rec = record(tmp_path / "cam", seconds=6.0, cameras=["overview"])
    info = rl.info(tmp_path / "cam")
    assert info["features"]["observation.images.overview"] == [48, 64, 3]
    out = rl.train(tmp_path / "cam", tmp_path / "vpol", steps=4, batch_size=4, device="cpu", pretrained_backbone_weights=None, **SMALL)
    pol = rf.CONTROLLERS.build({"type": "lerobot", "path": str(out)})
    app = rf.App(robot="arm", renderer=rf.MujocoRenderer(width=64, height=48, fps=20), controllers=[pol, rf.JointPD(rate=500)], realtime=False)
    app.setup()
    app.bus.publish("task/goal", np.array([0.3, 0.2, -0.5]))
    app.run(duration=0.5)
    assert app.bus.latest("cmd/joint") is not None
    app.close()
    assert rec.episodes == 3


def test_cli_and_example_workflow(tmp_path):
    ex = tmp_path / "ex"
    shutil.copytree(EXAMPLE, ex, ignore=shutil.ignore_patterns("datasets", "policies", "__pycache__"))
    run = lambda *a: subprocess.run([sys.executable, "-m", "roboforge", *a], cwd=ex, capture_output=True, text=True, timeout=600)
    r = run("run", "record.yaml", "--headless", "--no-realtime", "--duration", "20")
    assert r.returncode == 0 and "saved 10 episodes" in r.stdout, r.stdout + r.stderr
    opts = [f"{k}={str(v).lower() if isinstance(v, bool) else v}" for k, v in SMALL.items()]
    r = run("lerobot", "train", "datasets/arm_reach", "--out", "policies/arm_act", "--steps", "40", "--device", "cpu", "--log-every", "20", *opts)
    assert r.returncode == 0 and "saved the policy" in r.stdout, r.stdout + r.stderr
    r = run("run", "deploy.yaml", "--headless", "--no-realtime", "--duration", "3", "--tap", "3")
    assert r.returncode == 0 and "lerobot" in r.stdout, r.stdout + r.stderr
    r = subprocess.run([sys.executable, "evaluate.py"], cwd=ex, capture_output=True, text=True, timeout=300)
    assert r.returncode == 0 and "worst joint error" in r.stdout, r.stdout + r.stderr
    bad = run("lerobot", "train", "datasets/arm_reach", "--out", "p", "chunk_size")
    assert bad.returncode == 2 and "name=value" in bad.stderr
