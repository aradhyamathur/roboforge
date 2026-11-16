"""LeRobot (Hugging Face) inside RoboForge apps: record datasets, train policies, run them.

    callbacks:
      - {type: lerobot_recorder, root: datasets/arm_reach, fps: 20,
         state: [state/joint], environment: [task/goal], action: cmd/joint, task: "reach the goal"}

    python -m roboforge lerobot train datasets/arm_reach --out policies/arm_act --steps 2000

    controllers:
      - {type: lerobot, path: policies/arm_act}          # or a Hugging Face hub id
      - {type: joint_pd, rate: 500}

Three pieces, each usable on its own:

- `LeRobotRecorder` writes any app's run straight into a `LeRobotDataset` (the
  format LeRobot trains on and the Hugging Face hub hosts), one episode per
  `app.reset()`.
- `train()` trains any LeRobot policy (ACT, Diffusion, ...) on such a dataset,
  using LeRobot's own factories, on a CPU or a GPU.
- `LeRobotPolicy` runs a trained or downloaded policy as an ordinary
  controller: the app's signals become the policy's inputs and its actions
  become commands.

Which signals feed which LeRobot feature is written next to the dataset and
the policy (`roboforge.json`), so a policy trained here deploys with no
further configuration. LeRobot itself is imported only when one of these is
used: `pip install "lerobot[dataset]"`.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Optional

import numpy as np

from . import msgs
from .callbacks import as_row
from .component import Callback
from .control.policy import Policy
from .errors import ConfigError, positive

SIDECAR = "roboforge.json"
OUTPUTS = {msgs.CMD_JOINT: "joint", msgs.CMD_ACTUATORS: "actuators", msgs.CMD_VELOCITY: "velocity", msgs.CMD_SETPOINT: "setpoint"}


def _require():
    try:
        import lerobot  # noqa: F401
        from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: F401
    except ImportError as e:
        raise ImportError(f"LeRobot is not installed ({e}). Install it with: pip install \"lerobot[dataset]\" "
                          f"(LeRobot needs numpy < 2.3; see the LeRobot tutorial for an environment that keeps ROS 2 working)") from None
    import lerobot
    return lerobot


def pick_device(device: str = "auto") -> str:
    """`auto` is `cuda` when PyTorch can use a GPU, then Apple's `mps`, else `cpu`."""
    import torch
    if device != "auto":
        return device
    if torch.cuda.is_available():
        return "cuda"
    if getattr(torch.backends, "mps", None) is not None and torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def _names(signal: str, msg, n: int) -> list:
    """Readable names for the values a signal contributes, for the dataset's metadata."""
    if isinstance(msg, msgs.JointState):
        names = list(msg.names) or [f"joint{i}" for i in range(len(msg.position))]
        return [f"{j}.pos" for j in names] + [f"{j}.vel" for j in names]
    if isinstance(msg, msgs.JointCommand):
        return list(msg.names) if len(msg.names) == n else [f"joint{i}" for i in range(n)]
    if isinstance(msg, msgs.Pose):
        return ["x", "y", "z", "qw", "qx", "qy", "qz"]
    if isinstance(msg, msgs.Twist):
        return ["vx", "vy", "vz", "wx", "wy", "wz"]
    if isinstance(msg, msgs.Imu):
        return ["ax", "ay", "az", "gx", "gy", "gz"]
    if isinstance(msg, msgs.Setpoint):
        return ["x", "y", "z", "yaw"]
    base = signal.replace("/", ".")
    return [f"{base}.{i}" for i in range(n)]


def _list(v) -> list:
    return [] if v is None else ([v] if isinstance(v, str) else list(v))


class LeRobotRecorder(Callback):
    """Record a run as a LeRobot dataset.

    `state` and `environment` list signals whose values are concatenated into
    `observation.state` and `observation.environment_state`; `cameras` adds
    `observation.images.<name>` from `sensor/camera/<name>`; `action` is the
    command signal recorded as `action` (cmd/joint, cmd/actuators,
    cmd/velocity or cmd/setpoint). Frames are taken at `fps` of app time,
    starting once every listed signal has been published.

    Episodes end on `app.reset()`, after `episode_seconds`, on
    `end_episode()`, and when the app closes. `mode: new` refuses to write
    into an existing dataset; `append` adds episodes to it.
    """
    name = "lerobot_recorder"

    def __init__(self, root, repo_id: str = None, fps: float = 20, state=("state/joint",), environment=(), cameras=(),
                 action: str = msgs.CMD_JOINT, task: str = "", use_videos: bool = False, robot_type: str = None,
                 episode_seconds: float = None, mode: str = "new"):
        self.root = Path(root)
        self.repo_id = repo_id or f"local/{self.root.name}"
        self.fps = positive(fps, "lerobot_recorder fps")
        if int(self.fps) != self.fps:
            raise ConfigError(f"lerobot_recorder fps must be a whole number (LeRobot stores it as one), got {fps!r}")
        self.state, self.environment, self.cameras = _list(state), _list(environment), _list(cameras)
        if not self.state and not self.environment and not self.cameras:
            raise ConfigError("lerobot_recorder needs at least one of state, environment or cameras to record")
        if action not in OUTPUTS:
            raise ConfigError(f"lerobot_recorder action must be one of {', '.join(OUTPUTS)}, got {action!r}")
        if mode not in ("new", "append"):
            raise ConfigError(f"lerobot_recorder mode must be new or append, got {mode!r}")
        self.action, self.task, self.use_videos, self.robot_type = action, task, use_videos, robot_type
        self.episode_seconds, self.mode = episode_seconds, mode
        self.inputs = self.state + self.environment + [f"sensor/camera/{c}" for c in self.cameras] + [action]
        self.ds = None
        self.episodes = self.frames = 0

    # --- lifecycle --------------------------------------------------------------
    def on_start(self, app):
        _require()
        self.app = app
        self._robot = self.robot_type or (app.robot.name if app.robot else "roboforge")
        self._in_episode, self._next, self._episode_t0 = 0, app.physics.time, app.physics.time
        if self.mode == "new" and (self.root / "meta" / "info.json").exists():
            raise ConfigError(f"{self.root} already holds a LeRobot dataset. Use mode: append to add episodes, "
                              f"or record to another root.")
        if self.mode == "append" and (self.root / "meta" / "info.json").exists():
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
            self.ds = LeRobotDataset.resume(self.repo_id, root=self.root)
            self._check_sidecar()

    def _check_sidecar(self):
        f = self.root / "meta" / SIDECAR
        if f.is_file():
            old = json.loads(f.read_text())
            mine = self._mapping()
            for k in ("state", "environment", "cameras", "action"):
                if old.get(k) != mine[k]:
                    raise ConfigError(f"appending to {self.root}: it was recorded with {k} {old.get(k)}, this recorder has {mine[k]}")

    def _mapping(self) -> dict:
        return {"roboforge": 1, "fps": int(self.fps), "state": self.state, "environment": self.environment, "cameras": self.cameras,
                "action": self.action, "output": OUTPUTS[self.action], "task": self.task}

    def _create(self, app, values):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        features = {}
        for key, sigs in (("observation.state", self.state), ("observation.environment_state", self.environment)):
            if sigs:
                names = sum((_names(s, app.bus.latest(s), len(values[s])) for s in sigs), [])
                features[key] = {"dtype": "float32", "shape": (len(names),), "names": names}
        for c in self.cameras:
            h, w = values[f"sensor/camera/{c}"].shape[:2]
            features[f"observation.images.{c}"] = {"dtype": "video" if self.use_videos else "image", "shape": (h, w, 3),
                                                   "names": ["height", "width", "channels"]}
        a = values[self.action]
        features["action"] = {"dtype": "float32", "shape": (len(a),), "names": _names(self.action, app.bus.latest(self.action), len(a))}
        self.root.parent.mkdir(parents=True, exist_ok=True)
        self.ds = LeRobotDataset.create(repo_id=self.repo_id, fps=int(self.fps), features=features, root=self.root,
                                        robot_type=self._robot, use_videos=self.use_videos)
        (self.root / "meta" / SIDECAR).write_text(json.dumps(self._mapping(), indent=2))

    def _values(self, app) -> Optional[dict]:
        out = {}
        for s in self.state + self.environment + [self.action]:
            m = app.bus.latest(s)
            row = None if m is None else as_row(m)
            if row is None:
                return None
            out[s] = np.asarray(row, np.float32)
        for c in self.cameras:
            img = app.bus.latest(f"sensor/camera/{c}")
            if img is None:
                return None
            out[f"sensor/camera/{c}"] = np.asarray(img.data, np.uint8)
        return out

    def on_step(self, app, t):
        if t + 1e-9 < self._next:
            return
        self._next = max(self._next + 1.0 / self.fps, t)
        if self.episode_seconds and self._in_episode and t - self._episode_t0 >= self.episode_seconds - 1e-9:
            self.end_episode()
        values = self._values(app)
        if values is None:
            return                          # a listed signal has not been published yet
        if self.ds is None:
            self._create(app, values)
        if not self._in_episode:
            self._episode_t0 = t
        frame = {"action": values[self.action], "task": self._task(app)}
        if self.state:
            frame["observation.state"] = np.concatenate([values[s] for s in self.state])
        if self.environment:
            frame["observation.environment_state"] = np.concatenate([values[s] for s in self.environment])
        for c in self.cameras:
            frame[f"observation.images.{c}"] = values[f"sensor/camera/{c}"]
        try:
            self.ds.add_frame(frame)
        except ValueError as e:
            raise ValueError(f"lerobot_recorder: this frame does not match the dataset's features (a signal changed size?): {e}") from None
        self._in_episode += 1
        self.frames += 1

    def _task(self, app) -> str:
        ins = app.bus.latest(msgs.CMD_INSTRUCTION)
        return ins.text if ins is not None and getattr(ins, "text", "") else (self.task or "")

    def end_episode(self) -> None:
        """Save the frames recorded since the last episode ended as one episode."""
        if self.ds is not None and self._in_episode:
            self.ds.save_episode()
            self.episodes += 1
        self._in_episode = 0

    def discard_episode(self) -> None:
        """Drop the frames recorded since the last episode ended (a failed demonstration)."""
        if self.ds is not None and self._in_episode:
            self.ds.clear_episode_buffer()
        self._in_episode = 0

    def on_reset(self, app):
        self.end_episode()
        self._next = app.physics.time

    def on_stop(self, app):
        self.end_episode()
        if self.ds is not None:
            self.ds.finalize()
            print(f"roboforge: lerobot_recorder saved {self.episodes} episodes, {self.frames} frames, to {self.root}", flush=True)

    def describe(self):
        return {**super().describe(), "root": str(self.root), "fps": self.fps}


# ------------------------------------------------------------------------------ training
def info(root) -> dict:
    """What a LeRobot dataset holds: frames, episodes, fps, features, and the RoboForge mapping if any."""
    _require()
    from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
    root = Path(root)
    if not (root / "meta" / "info.json").is_file():
        raise ConfigError(f"{root} is not a LeRobot dataset (no meta/info.json)")
    meta = LeRobotDatasetMetadata(_repo_id(root), root=root)
    side = root / "meta" / SIDECAR
    return {"root": str(root), "episodes": meta.total_episodes, "frames": meta.total_frames, "fps": meta.fps,
            "robot_type": meta.robot_type, "features": {k: list(v["shape"]) if v.get("shape") is not None else None for k, v in meta.features.items()},
            "roboforge": json.loads(side.read_text()) if side.is_file() else None}


def _repo_id(root: Path) -> str:
    return f"local/{Path(root).name}"


def train(dataset, out, policy: str = "act", steps: int = 2000, batch_size: int = 32, lr: float = None,
          device: str = "auto", seed: int = 0, log_every: int = 200, **policy_options) -> Path:
    """Train a LeRobot policy on a dataset and save it, ready for `LeRobotPolicy`.

    `policy` is any LeRobot policy type (`act`, `diffusion`, ...);
    `policy_options` go to its config (for ACT: `chunk_size`, `dim_model`,
    `n_encoder_layers`, `use_vae`, ...). Saves the weights, LeRobot's pre- and
    post-processors and the dataset's RoboForge mapping to `out`.
    """
    _require()
    import torch
    from lerobot.datasets.factory import resolve_delta_timestamps
    from lerobot.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
    from lerobot.policies.factory import make_policy, make_policy_config, make_pre_post_processors

    dataset, out = Path(dataset), Path(out)
    if not (dataset / "meta" / "info.json").is_file():
        raise ConfigError(f"{dataset} is not a LeRobot dataset (no meta/info.json)")
    device = pick_device(device)
    torch.manual_seed(seed)
    meta = LeRobotDatasetMetadata(_repo_id(dataset), root=dataset)
    try:
        cfg = make_policy_config(policy, device=device, push_to_hub=False, **policy_options)
    except TypeError as e:
        raise ConfigError(f"LeRobot policy {policy!r} does not take one of {sorted(policy_options)}: {e}") from None
    except ValueError as e:
        raise ConfigError(str(e)) from None
    net = make_policy(cfg, ds_meta=meta)
    pre, post = make_pre_post_processors(cfg, dataset_stats=meta.stats)
    data = LeRobotDataset(_repo_id(dataset), root=dataset, delta_timestamps=resolve_delta_timestamps(cfg, meta))
    loader = torch.utils.data.DataLoader(data, batch_size=batch_size, shuffle=True, drop_last=len(data) > batch_size,
                                         generator=torch.Generator().manual_seed(seed))
    opt = torch.optim.AdamW(net.parameters(), lr=lr or cfg.get_optimizer_preset().lr,
                            weight_decay=cfg.get_optimizer_preset().weight_decay)
    print(f"roboforge: training {policy} ({sum(p.numel() for p in net.parameters()):,} parameters) on {meta.total_frames} frames "
          f"from {meta.total_episodes} episodes, on {device}", flush=True)
    net.train()
    step, t0, recent = 0, time.time(), []
    while step < steps:
        for batch in loader:
            batch = pre(batch)
            loss, _ = net(batch)
            opt.zero_grad()
            loss.backward()
            opt.step()
            step += 1
            recent.append(float(loss))
            if step % log_every == 0 or step == steps:
                print(f"  step {step:>6}  loss {np.mean(recent):.4f}  ({time.time() - t0:.0f}s)", flush=True)
                recent = []
            if step >= steps:
                break
    out.mkdir(parents=True, exist_ok=True)
    net.save_pretrained(out)
    pre.save_pretrained(out)
    post.save_pretrained(out)
    side = dataset / "meta" / SIDECAR
    mapping = json.loads(side.read_text()) if side.is_file() else {}
    mapping.update({"policy": policy, "trained_on": str(dataset.resolve()), "steps": steps})
    (out / SIDECAR).write_text(json.dumps(mapping, indent=2))
    print(f"roboforge: saved the policy to {out}", flush=True)
    return out


# ------------------------------------------------------------------------------ deployment
class LeRobotPolicy(Policy):
    """Run a LeRobot policy (a folder from `train`, or a Hugging Face hub id) as a controller.

    Which signals feed the policy comes from the `roboforge.json` saved with
    it; for a policy trained elsewhere, give `state`, `environment` and
    `cameras` (the same lists as the recorder's), and `output` (joint,
    actuators, velocity or setpoint). `rate` defaults to the dataset's fps.
    LeRobot keeps its own queue of chunked actions; `app.reset()` clears it.
    """
    name = "lerobot"

    def __init__(self, path, rate: float = None, device: str = "auto", state=None, environment=None, cameras=None,
                 output: str = None, task: str = None, name: str = ""):
        self.path = str(path)
        self._overrides = {"state": state, "environment": environment, "cameras": cameras, "output": output, "task": task}
        self.device_request = device
        mapping = self._mapping()
        self.output = mapping.get("output") or "joint"
        super().__init__(rate=rate if rate is not None else mapping.get("fps"), name=name)
        self.state, self.environment, self.cameras = (_list(mapping.get(k)) for k in ("state", "environment", "cameras"))
        self.task = mapping.get("task") or ""
        self.net = None

    def _mapping(self) -> dict:
        side = Path(self.path) / SIDECAR
        m = json.loads(side.read_text()) if side.is_file() else {"state": ["state/joint"]}
        m.update({k: v for k, v in self._overrides.items() if v is not None})
        return m

    def setup(self, app):
        _require()
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors
        self._torch = torch
        self.device = pick_device(self.device_request)
        if not Path(self.path).exists() and self.path.count("/") != 1:
            raise ConfigError(f"no LeRobot policy at {self.path}: the folder does not exist. Train one with "
                              f"python -m roboforge lerobot train <dataset> --out {self.path}")
        try:
            cfg = PreTrainedConfig.from_pretrained(self.path)
        except Exception as e:
            where = "on the Hugging Face hub" if not Path(self.path).exists() else "in that folder"
            hint = (f"No folder {self.path} exists here either; train one with python -m roboforge lerobot train <dataset> --out {self.path}, "
                    f"or give a hub id such as lerobot/act_aloha_sim_transfer_cube_human." if not Path(self.path).exists() else
                    "The folder should hold config.json and model.safetensors, as saved by lerobot train or policy.save_pretrained().")
            raise ConfigError(f"could not load a LeRobot policy from {self.path} ({where}): {type(e).__name__}: {e}. {hint}") from None
        cfg.device = self.device
        self.net = get_policy_class(cfg.type).from_pretrained(self.path, config=cfg)
        self.net.to(self.device)
        self.net.eval()
        over = {"device_processor": {"device": self.device}}
        self.pre, self.post = make_pre_post_processors(cfg, pretrained_path=self.path, preprocessor_overrides=over)
        expected = set(getattr(cfg, "input_features", {}) or {})
        mine = ({"observation.state"} if self.state else set()) | ({"observation.environment_state"} if self.environment else set()) \
            | {f"observation.images.{c}" for c in self.cameras}
        if expected and mine != expected:
            raise ConfigError(f"the policy at {self.path} takes {sorted(expected)}, but this controller would give it {sorted(mine)}. "
                              f"Set state, environment and cameras to match how its dataset was recorded.")
        self.config = cfg

    def observe(self, obs):
        torch = self._torch
        batch = {}
        if self.state:
            batch["observation.state"] = torch.as_tensor(np.concatenate([as_row(obs[s]) for s in self.state]), dtype=torch.float32)[None]
        if self.environment:
            batch["observation.environment_state"] = torch.as_tensor(
                np.concatenate([as_row(obs[s]) for s in self.environment]), dtype=torch.float32)[None]
        for c in self.cameras:
            img = obs.camera(c)
            if img is None:
                raise KeyError(f"sensor/camera/{c}")
            batch[f"observation.images.{c}"] = torch.as_tensor(np.asarray(img.data), dtype=torch.float32).permute(2, 0, 1)[None] / 255.0
        ins = obs.get(msgs.CMD_INSTRUCTION)
        batch["task"] = [ins.text if ins is not None and getattr(ins, "text", "") else self.task]
        return batch

    def act(self, batch):
        with self._torch.no_grad():
            action = self.post(self.net.select_action(self.pre(batch)))
        return action[0]

    def step(self, obs, t):
        try:
            batch = self.observe(obs)
        except (KeyError, AttributeError, TypeError):
            return None                     # an input has not been published yet
        from .env import to_numpy
        return self.command(self.decode(to_numpy(self.act(batch)), obs))

    def reset(self):
        if self.net is not None:
            self.net.reset()

    def describe(self):
        return {"type": "lerobot", "path": self.path, "rate": self.rate, "output": self.output}
