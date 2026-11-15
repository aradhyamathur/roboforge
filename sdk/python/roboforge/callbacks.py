"""Hooks around the loop: recording, monitoring, viewing."""
from __future__ import annotations

import json
import shutil
import time
from pathlib import Path

import numpy as np
import yaml

from . import msgs
from .component import Callback
from .data import write_png


class Recorder(Callback):
    """Record a run as a `Dataset` (see `roboforge.data`).

    Records pose, twist, joints and IMU at `rate` Hz (default: every step)
    and every camera frame published. `signals` narrows the list.
    """
    name = "recorder"

    def __init__(self, dir="runs/latest", rate: float = None, signals=None, cameras=None):
        self.dir, self.rate, self.cameras = Path(dir), rate, cameras
        self.signals = signals or [msgs.POSE, msgs.TWIST, msgs.JOINT_STATE, msgs.IMU, msgs.CMD_ACTUATORS]
        self.inputs = list(self.signals) + ["sensor/camera/*"]

    def on_start(self, app):
        try:
            self.dir.mkdir(parents=True, exist_ok=True)
            # A folder holds one run: frames left by an earlier recording would be mixed into this one.
            for old in ("images", "signals.npz", "dataset.yaml"):
                p = self.dir / old
                if p.is_dir():
                    shutil.rmtree(p)
                elif p.exists():
                    p.unlink()
        except OSError as e:
            raise OSError(f"the recorder cannot write to {self.dir}: {e.strerror or e}") from None
        self._rows = {s: [] for s in self.signals}
        self._times = {s: [] for s in self.signals}
        self._frames, self._intr = {}, {}
        self._next = 0.0
        self._robot = None if app.robot is None else app.robot.name
        self._unsub = app.bus.subscribe("sensor/camera/*", self._on_image)

    def _on_image(self, name, img):
        cam = name.rsplit("/", 1)[-1]
        if self.cameras is not None and cam not in self.cameras:
            return
        d = self.dir / "images" / cam
        d.mkdir(parents=True, exist_ok=True)
        frames = self._frames.setdefault(cam, [])
        file = f"{len(frames):06d}.png"
        write_png(d / file, img.data)
        frames.append({"file": file, "t": float(img.t), "pose": np.asarray(img.pose).tolist()})
        self._intr[cam] = img.intrinsics

    def on_step(self, app, t):
        if self.rate:
            if t + 1e-9 < self._next:
                return
            self._next = max(self._next + 1.0 / self.rate, t)
        for s in self.signals:
            m = app.bus.latest(s)
            if m is None:
                continue
            if s in (msgs.POSE,):
                row = np.concatenate([m.position, m.orientation])
            elif s in (msgs.TWIST,):
                row = np.concatenate([m.linear, m.angular])
            elif s == msgs.JOINT_STATE:
                row = np.concatenate([m.position, m.velocity])
            elif s == msgs.IMU:
                row = np.concatenate([m.accel, m.gyro])
            elif s == msgs.CMD_ACTUATORS:
                row = np.asarray(m.values, float)
            else:
                continue
            self._rows[s].append(row)
            self._times[s].append(t)

    def on_stop(self, app):
        self._unsub()
        arrays = {}
        for s, rows in self._rows.items():
            if rows and len({len(r) for r in rows}) == 1:
                key = s.replace("/", "_")
                arrays[key] = np.array(rows)
                arrays[key + "_t"] = np.array(self._times[s])
        np.savez_compressed(self.dir / "signals.npz", **arrays)
        for cam, frames in self._frames.items():
            (self.dir / "images" / cam / "frames.json").write_text(json.dumps(frames))
        meta = {"roboforge": 1, "robot": self._robot, "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "duration": float(app.physics.time), "signals": sorted(k for k in arrays if not k.endswith("_t")),
                "signal_names": {s.replace("/", "_"): s for s in self._rows if s.replace("/", "_") in arrays},
                "cameras": {c: {**self._intr[c], "frames": len(f)} for c, f in self._frames.items()}}
        (self.dir / "dataset.yaml").write_text(yaml.safe_dump(meta, sort_keys=False))


class RateMonitor(Callback):
    """Track how fast the app runs against real time. `report()` returns the numbers;
    with `every` set it also prints them that often."""
    name = "rate_monitor"

    def __init__(self, every: float = None):
        self.every = every

    def on_start(self, app):
        self._wall0, self._sim0, self._last = time.monotonic(), app.physics.time, time.monotonic()

    def report(self, app) -> dict:
        wall = time.monotonic() - self._wall0
        sim = app.physics.time - self._sim0
        return {"sim_time": round(sim, 3), "wall_time": round(wall, 3), "realtime_factor": round(sim / wall, 3) if wall > 0 else 0.0,
                "controllers": {(c.name or type(c).__name__): round(app.controller_rate(c), 1) for c in app.controllers}}

    def on_step(self, app, t):
        if self.every and time.monotonic() - self._last >= self.every:
            self._last = time.monotonic()
            r = self.report(app)
            print(f"roboforge: t={r['sim_time']:.1f}s  x{r['realtime_factor']:.2f} real time  " + "  ".join(f"{k} {v:.0f} Hz" for k, v in r["controllers"].items()), flush=True)


class Viewer(Callback):
    """MuJoCo's interactive viewer window. Closing the window stops the app."""
    name = "viewer"

    def __init__(self, fps: float = 60.0):
        self.fps = fps
        self._v = None

    def on_start(self, app):
        import os
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            print("roboforge: no display, so the viewer is skipped.")
            return
        model = getattr(app.physics, "model", None)
        if model is None:
            print("roboforge: the viewer needs MujocoPhysics; skipped.")
            return
        import mujoco.viewer
        self._v = mujoco.viewer.launch_passive(model, app.physics.data)
        self._last = 0.0

    def on_step(self, app, t):
        if self._v is None:
            return
        if not self._v.is_running():
            app.stop()
            return
        now = time.monotonic()
        if now - self._last >= 1.0 / self.fps:
            self._last = now
            self._v.sync()

    def on_stop(self, app):
        if self._v is not None:
            self._v.close()
            self._v = None
