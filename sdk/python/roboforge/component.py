"""Base classes for the parts an app is assembled from.

All of them are optional conveniences: anything with the same methods works.
`inputs` / `outputs` list the bus signals a component consumes and produces;
they are what `roboforge info` and the IDE's stack view display.
"""
from __future__ import annotations


class Component:
    name: str = ""
    inputs: list = []
    outputs: list = []

    def setup(self, app) -> None:
        """Called once, before the first step."""

    def close(self) -> None:
        """Called once, when the app shuts down."""

    def describe(self) -> dict:
        return {"type": self.name or type(self).__name__, "inputs": list(self.inputs), "outputs": list(self.outputs)}


class Physics(Component):
    """Steps the world. Publishes state signals, consumes `cmd/actuators`.

    Required: `dt`, `time`, `step()`, `reset()`. `raw` exposes the backend's
    own objects for anything this interface does not cover.

    Optional, and what each buys: `joint_names`, `actuator_names` (size
    checks), `ctrl_range` and `joint_range` (Gymnasium spaces, saturation
    checks), `camera(name)` (any renderer can draw from it), `allocation`
    (the multirotor controller and the PX4 link can fly it).

    `publish_state(...)` publishes the clock and whatever state is given,
    stamped with the current time: the usual last line of `step()`.
    """
    dt: float = 0.01
    time: float = 0.0
    raw = None

    def step(self) -> None:
        raise NotImplementedError

    def reset(self) -> None:
        pass

    def camera(self, name: str):
        """`(pose, fovy)` of a camera: 4x4 camera-to-world (OpenGL axes) and vertical field of view in degrees."""
        raise NotImplementedError(f"the {self.name or type(self).__name__} physics backend has no cameras")

    def publish_state(self, joint=None, pose=None, twist=None, imu=None, **signals) -> None:
        """Publish `clock` and the given state, all stamped with `self.time`.

        `joint`, `pose`, `twist`, `imu` are messages (or None to skip);
        any other keyword is `signal_name_with_underscores=message`.
        """
        from . import msgs
        t, who = self.time, self.name or type(self).__name__
        self.bus.publish(msgs.CLOCK, msgs.Clock(t=t), who)
        for name, m in ((msgs.JOINT_STATE, joint), (msgs.POSE, pose), (msgs.TWIST, twist), (msgs.IMU, imu)):
            if m is not None:
                if hasattr(m, "t"):
                    m.t = t
                self.bus.publish(name, m, who)
        for key, m in signals.items():
            self.bus.publish(key.replace("__", "/"), m, who)


class Renderer(Component):
    """Turns world state and a camera into an `Image`.

    A renderer of your own implements one method, `render(camera) -> Image`.
    This base class does the rest: `setup` picks the cameras (the robot's, by
    default), `step(t)` publishes `sensor/camera/<name>` at `fps`, and
    `finish(image)` runs the render passes. Where a camera is comes from
    `app.physics.camera(name)`, so a renderer need not know the physics engine.

    `add_pass` appends a function `(image, renderer) -> image` run on every
    frame: a post-process, an extra buffer, or a replacement picture.
    """
    cameras = None
    fps: float = 10.0

    def __init__(self, cameras=None, fps: float = None):
        self.passes: list = []
        if cameras is not None:
            self.cameras = [cameras] if isinstance(cameras, str) else list(cameras)
        if fps is not None:
            self.fps = fps
        self._next_frame = 0.0

    def setup(self, app) -> None:
        self.bus, self.app = app.bus, app
        if self.cameras is None:
            self.cameras = list(app.robot.cameras) if app.robot else []
        self.outputs = [f"sensor/camera/{c}" for c in self.cameras]

    def add_pass(self, fn):
        if not hasattr(self, "passes"):
            self.passes = []
        self.passes.append(fn)
        return fn

    def render(self, camera: str):
        raise NotImplementedError

    def finish(self, img):
        """Run the render passes on a frame, naming the pass if one fails."""
        from .msgs import Image
        for i, p in enumerate(getattr(self, "passes", [])):
            who = getattr(p, "__name__", type(p).__name__)
            try:
                out = p(img, self)
            except Exception as e:
                e.add_note(f"(in render pass {i}, {who}, on camera {img.camera!r})")
                raise
            if out is None:
                continue
            if not isinstance(out, Image):
                raise ValueError(f"render pass {i} ({who}) returned a {type(out).__name__}; a pass returns an rf.Image, "
                                 f"or None to keep the image it was given")
            img = out
        return img

    def step(self, t: float) -> None:
        if not self.cameras or not self.fps:
            return
        if t + 1e-9 < getattr(self, "_next_frame", 0.0):
            return
        self._next_frame = max(getattr(self, "_next_frame", 0.0) + 1.0 / self.fps, t)
        for c in self.cameras:
            self.bus.publish(f"sensor/camera/{c}", self.render(c), self.name or type(self).__name__)


class Link(Component):
    """Connects the bus to something outside the process (ROS 2, MAVLink).

    `before_step` runs before controllers and physics and may block (that is
    how lockstep with an external autopilot works); `after_step` runs after.
    """

    def before_step(self, t: float) -> None:
        pass

    def after_step(self, t: float) -> None:
        pass

    # What arrives from outside is not trusted: a bad message is dropped with a
    # warning, never passed on to crash a controller or the simulation.
    dropped: int = 0

    def reject(self, what: str, why: str) -> None:
        """Count a dropped incoming message and say so, at most once every few seconds per kind."""
        import time
        self.dropped += 1
        last = self.__dict__.setdefault("_warned", {})
        now = time.monotonic()
        seen, since = last.get(what, (0, -1e9))
        if now - since > 5.0:
            more = f" ({seen} more like it since the last warning)" if seen else ""
            print(f"roboforge: {self.name or type(self).__name__} link ignored {what}: {why}{more}", flush=True)
            last[what] = (0, now)
        else:
            last[what] = (seen + 1, since)

    @staticmethod
    def finite(*values) -> bool:
        import numpy as np
        try:
            return all(bool(np.all(np.isfinite(np.asarray(v, float)))) for v in values if v is not None)
        except (TypeError, ValueError):
            return False


class Callback(Component):
    """Hooks around the loop: recording, viewing, monitoring."""

    def on_start(self, app) -> None:
        pass

    def on_step(self, app, t: float) -> None:
        pass

    def on_stop(self, app) -> None:
        pass

    def on_reset(self, app) -> None:
        """Called by `app.reset()`, after the world is back at its start (an episode ended)."""
