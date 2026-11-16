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


def as_row(m):
    """One recorded row for a message, or None if it is not recordable.

    Decided by the message's type, not the signal's name, so an estimate
    published as a `Pose` on `slam/pose` records like `state/pose` does. Your
    own message types record if they have an `as_array()` method; plain numpy
    arrays and numbers record as they are.
    """
    if isinstance(m, msgs.Pose):
        return np.concatenate([m.position, m.orientation])
    if isinstance(m, msgs.Twist):
        return np.concatenate([m.linear, m.angular])
    if isinstance(m, msgs.JointState):
        return np.concatenate([m.position, m.velocity])
    if isinstance(m, msgs.Imu):
        return np.concatenate([m.accel, m.gyro])
    if isinstance(m, msgs.ActuatorCommand):
        return np.asarray(m.values, float).reshape(-1)
    if isinstance(m, msgs.JointCommand):
        return np.asarray(m.position, float).reshape(-1)
    if isinstance(m, msgs.Setpoint):
        if m.position is not None:
            return np.append(np.asarray(m.position, float).reshape(-1), float(m.yaw))
        if m.velocity is not None:
            return np.append(np.asarray(m.velocity, float).reshape(-1), float(m.yaw))
        return None
    if hasattr(m, "as_array"):
        return np.asarray(m.as_array(), float).reshape(-1)
    if isinstance(m, (np.ndarray, list, tuple, int, float)) and not isinstance(m, bool):
        try:
            return np.asarray(m, float).reshape(-1)
        except (TypeError, ValueError):
            return None
    return None


class Recorder(Callback):
    """Record a run as a `Dataset` (see `roboforge.data`).

    Records pose, twist, joints and IMU at `rate` Hz (default: every step)
    and every camera frame published. `signals` replaces the list; it may name
    any signal whose messages are poses, twists, joint states, IMU readings,
    commands, arrays, or your own types with `as_array()` (see `as_row`).
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
            row = as_row(m)
            if row is None:
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


def summarize(msg, width: int = 46) -> str:
    """A short, readable rendering of any message, for logs and tables."""
    def num(v):
        a = np.asarray(v, float).reshape(-1)
        body = " ".join(f"{x:.2f}" for x in a[:6])
        return f"[{body}{' ...' if len(a) > 6 else ''}]"
    if isinstance(msg, msgs.Image):
        text = f"{msg.data.shape[1]}x{msg.data.shape[0]} {msg.encoding}"
    elif isinstance(msg, msgs.Pose):
        text = f"pos {num(msg.position)}"
    elif isinstance(msg, msgs.Twist):
        text = f"lin {num(msg.linear)} ang {num(msg.angular)}"
    elif isinstance(msg, msgs.JointState):
        text = f"q {num(msg.position)}"
    elif isinstance(msg, msgs.JointCommand):
        text = f"q* {num(msg.position)}"
    elif isinstance(msg, msgs.Imu):
        text = f"acc {num(msg.accel)}"
    elif isinstance(msg, msgs.ActuatorCommand):
        text = f"u {num(msg.values)}"
    elif isinstance(msg, msgs.Setpoint):
        text = (f"pos {num(msg.position)}" if msg.position is not None else f"vel {num(msg.velocity)}" if msg.velocity is not None else "") + f" yaw {msg.yaw:.2f}"
    elif isinstance(msg, msgs.VehicleStatus):
        text = f"{'armed' if msg.armed else 'disarmed'} {msg.mode} {'landed' if msg.landed else 'in air'}"
    elif isinstance(msg, msgs.Clock):
        text = f"t {msg.t:.3f}"
    else:
        text = repr(msg)
    return text if len(text) <= width else text[:width - 3] + "..."


class Tap(Callback):
    """Print what is on the bus while the app runs: every signal with its rate,
    who publishes it and its latest value. The first thing to add when something
    does not move.

        callbacks:
          - {type: tap, every: 1.0}                 # seconds of app time; or: python -m roboforge run app.yaml --tap

    `signals` narrows the table to names or patterns (`["cmd/*", "state/pose"]`).
    `rows()` returns the same table as data.
    """
    name = "tap"

    def __init__(self, every: float = 1.0, signals=None):
        from .errors import positive
        self.every, self.signals = positive(every, "tap every"), signals

    def on_start(self, app):
        self._wall, self._t, self._counts = time.monotonic(), app.physics.time, {}

    def rows(self, app) -> list:
        import fnmatch
        dt = app.physics.time - self._t
        rows = []
        for d in app.bus.describe():
            if self.signals and not any(fnmatch.fnmatchcase(d["name"], p) for p in self.signals):
                continue
            n = d["count"] - self._counts.get(d["name"], 0)
            rows.append({"name": d["name"], "type": d["type"], "rate": n / dt if dt > 0 else 0.0, "sources": d["sources"],
                         "value": summarize(app.bus.latest(d["name"]))})
        return rows

    def on_step(self, app, t):
        if t - self._t < self.every - 1e-9:        # app time, so a fast (not real-time) run is tapped just the same
            return
        rows = self.rows(app)
        w = max([len(r["name"]) for r in rows] + [6])
        lines = [f"roboforge tap  t={t:.2f}s"] + [
            f"  {r['name']:<{w}}  {r['rate']:7.1f} Hz  {','.join(r['sources']) or '?':<22.22}  {r['value']}" for r in rows]
        print("\n".join(lines), flush=True)
        self._wall, self._t = time.monotonic(), t
        self._counts = {d["name"]: d["count"] for d in app.bus.describe()}


class Checks(Callback):
    """Watch a running app for the usual reasons a robot does not do what was
    expected, and say so once each, in plain words:

    - nothing publishes `cmd/actuators` (no controller drives the motors)
    - the actuators sit at their limits (gains too high, or a target out of reach)
    - a multirotor stays disarmed while a setpoint is waiting
    - the loop cannot keep up with real time
    - a controller runs slower than its declared rate
    - a link is dropping incoming messages

        callbacks:
          - {type: checks}                           # or: python -m roboforge run app.yaml --check

    `findings` holds what was reported, as `(code, text)` pairs.
    """
    name = "checks"

    def __init__(self, after: float = 1.0, every: float = 1.0):
        from .errors import positive
        self.after, self.every = positive(after, "checks after"), positive(every, "checks every")
        self.findings: list = []

    def on_start(self, app):
        self._t0, self._wall0, self._next = app.physics.time, time.monotonic(), app.physics.time + self.after
        self._sat = self._steps = 0
        self._seen = set()
        self._disarmed_since = None

    def _say(self, code, text):
        if code in self._seen:
            return
        self._seen.add(code)
        self.findings.append((code, text))
        print(f"roboforge check: {text}", flush=True)

    def on_step(self, app, t):
        rng = getattr(app.physics, "ctrl_range", None)
        cmd = app.bus.latest(msgs.CMD_ACTUATORS)
        if cmd is not None and rng is not None and len(rng) == len(cmd.values) and len(rng) and np.all(np.isfinite(rng)):
            self._steps += 1
            span = rng[:, 1] - rng[:, 0]
            v = np.asarray(cmd.values, float)
            if np.any((v <= rng[:, 0] + 0.01 * span) | (v >= rng[:, 1] - 0.01 * span)):
                self._sat += 1
        if t < self._next:
            return
        self._next = t + self.every
        elapsed = t - self._t0
        names = getattr(app.physics, "actuator_names", [])
        if names and app.bus.count(msgs.CMD_ACTUATORS) == 0:
            who = ", ".join(c.name or type(c).__name__ for c in app.controllers) or "none"
            self._say("no-command", f"nothing has published cmd/actuators after {elapsed:.1f}s, so the {len(names)} actuators get no input. "
                                    f"Controllers in this app: {who}. A controller that returns joint targets, setpoints or velocities "
                                    f"needs a low-level controller after it (joint_pd, multirotor, diffdrive).")
        if self._steps > 200 and self._sat / self._steps > 0.8:
            self._say("saturated", f"the actuators have been at their limits {100 * self._sat / self._steps:.0f}% of the time. "
                                   f"The gains are too high, or the target cannot be reached (check its units and frame).")
        st = app.bus.latest(msgs.STATUS)
        if st is not None and not st.armed and app.bus.latest(msgs.CMD_SETPOINT) is not None:
            self._disarmed_since = self._disarmed_since if self._disarmed_since is not None else t
            if t - self._disarmed_since > 3.0:
                self._say("disarmed", "a setpoint is waiting but the vehicle is disarmed, so the motors stay off. Arm it first "
                                      "(link.arm(), a MAVLink arm command, or rf.VehicleCommand('arm') on cmd/vehicle).")
        else:
            self._disarmed_since = None
        pose, twist = app.bus.latest(msgs.POSE), app.bus.latest(msgs.TWIST)
        if pose is not None and twist is not None and (pose.position[2] < -5.0 or twist.linear[2] < -30.0):
            self._say("falling", f"the robot is in free fall (height {pose.position[2]:.1f} m, falling at {-twist.linear[2]:.0f} m/s). "
                                 f"The model has no ground under it or no collision geometry: add a ground plane, or <collision> shapes to the URDF.")
        wall = time.monotonic() - self._wall0
        if app.realtime and wall > 3.0 and elapsed / app.speed / wall < 0.9:
            self._say("slow", f"the app runs at x{elapsed / wall:.2f} of real time, below the x{app.speed:g} asked for: each step takes too long. "
                              f"Lower the renderer's size or fps, slow the controllers down, or wrap a slow one in rf.Async.")
        if elapsed > 2.0:
            for c in app.controllers:
                want, got = c.rate, app.controller_rate(c)
                if want and want <= 1.0 / app.physics.dt + 1e-6 and got < 0.9 * want:
                    self._say(f"rate-{id(c)}", f"controller {c.name or type(c).__name__} is declared at {want:g} Hz but runs at {got:.1f} Hz.")
                elif want and want > 1.0 / app.physics.dt + 1e-6:
                    self._say(f"rate-{id(c)}", f"controller {c.name or type(c).__name__} asks for {want:g} Hz, above the physics rate of "
                                               f"{1.0 / app.physics.dt:g} Hz, so it runs once per step. Lower its rate or the physics timestep.")
        for l in app.links:
            if getattr(l, "dropped", 0) > 0:
                self._say(f"dropped-{id(l)}", f"the {l.name} link has dropped {l.dropped} incoming messages so far (bad values or not addressed to this robot); "
                                              f"its own warnings say which.")
