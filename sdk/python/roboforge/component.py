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
    """
    dt: float = 0.01
    time: float = 0.0
    raw = None

    def step(self) -> None:
        raise NotImplementedError

    def reset(self) -> None:
        pass


class Renderer(Component):
    """Turns world state and a camera into an `Image`.

    Required: `render(camera) -> Image`. `step(t)` is called every physics
    step and publishes frames at the renderer's own rate. `add_pass` appends a
    function `(image, renderer) -> image` run on every frame, which is how you
    add a post-process, an extra buffer, or replace the picture entirely.
    """

    def __init__(self):
        self.passes: list = []

    def add_pass(self, fn):
        self.passes.append(fn)
        return fn

    def render(self, camera: str):
        raise NotImplementedError

    def step(self, t: float) -> None:
        pass


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
