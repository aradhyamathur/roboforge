"""The app: an optional loop that owns the clock and the lifecycle.

    app = App(robot="quadrotor", links=[MavlinkLink(mode="vehicle")],
              controllers=[MultirotorController(rate=250)])
    app.run(duration=10)

Nothing here is required. `app.step()` advances one physics step, so you can
write the loop yourself; every component also works without an app as long as
it is given a bus.
"""
from __future__ import annotations

import time
from typing import Any, Callable, Optional

from . import registry as reg
from .bus import Bus
from .control import Controller, Obs, _Function
from .robot import Robot


class App:
    def __init__(self, robot=None, physics=None, renderer=None, links=(), controllers=(), controller=None,
                 callbacks=(), realtime: bool = True, speed: float = 1.0, bus: Optional[Bus] = None, name: str = "app"):
        self.name = name
        self.bus = bus or Bus()
        self.robot: Optional[Robot] = None if robot is None else Robot.from_spec(robot)
        if physics is None:
            physics = "mujoco" if self.robot is not None else "none"
        self.physics = reg.PHYSICS.build(physics)
        self.renderer = reg.RENDERERS.build(renderer)
        self.links = [reg.LINKS.build(l) for l in links]
        ctrls = list(controllers) + ([controller] if controller is not None else [])
        self.controllers: list[Controller] = [self._controller(c) for c in ctrls]
        self.callbacks = [reg.CALLBACKS.build(c) for c in callbacks]
        self.realtime, self.speed = realtime, speed
        self.steps = 0
        self._ready = False
        self._stop = False
        self._next: dict[int, float] = {}
        self._runs: dict[int, int] = {}
        self._wall0 = self._sim0 = 0.0

    @staticmethod
    def _controller(c) -> Controller:
        if isinstance(c, (str, dict)):
            return reg.CONTROLLERS.build(c)
        if callable(c) and not isinstance(c, Controller):
            return _Function(c)
        return c

    @classmethod
    def from_config(cls, config, overrides=(), **replace) -> "App":
        """Build from a YAML file or a dict. `overrides` are `a.b=value` strings;
        keyword arguments replace whole components (`controllers=[MyCtrl()]`)."""
        from .config import build_app
        return build_app(config, overrides, **replace)

    # --- lifecycle ---------------------------------------------------------
    def setup(self) -> "App":
        if self._ready:
            return self
        self.physics.setup(self)
        if self.renderer is not None:
            self.renderer.setup(self)
        for c in self.controllers:
            c.setup(self)
        for l in self.links:
            l.setup(self)
        for cb in self.callbacks:
            cb.setup(self)
            cb.on_start(self)
        self._ready = True
        self._wall0, self._sim0 = time.monotonic(), self.physics.time
        return self

    def reset(self) -> None:
        self.setup()
        self.physics.reset()
        for c in self.controllers:
            c.reset()
        self._next.clear()
        self._wall0, self._sim0 = time.monotonic(), self.physics.time

    def stop(self) -> None:
        """Ask a running `run()` to return after the current step."""
        self._stop = True

    def close(self) -> None:
        if not self._ready:
            return
        self._ready = False
        for cb in self.callbacks:
            cb.on_stop(self)
            cb.close()
        for c in self.controllers:
            c.close()
        for l in self.links:
            l.close()
        if self.renderer is not None:
            self.renderer.close()
        self.physics.close()

    def __enter__(self):
        return self.setup()

    def __exit__(self, *exc):
        self.close()

    # --- the loop ----------------------------------------------------------
    @property
    def time(self) -> float:
        return self.physics.time

    def publish(self, action: Any, source: str = "") -> None:
        """Publish what a controller returned: a message, a list, or `{signal: message}`."""
        if action is None:
            return
        if isinstance(action, dict):
            for name, msg in action.items():
                self.bus.publish(name, msg, source)
        elif isinstance(action, (list, tuple)):
            for a in action:
                self.publish(a, source)
        else:
            self.bus.publish(type(action).signal, action, source)

    def step(self) -> None:
        """One physics step: links in, controllers that are due, physics, renderer, links out, callbacks."""
        self.setup()
        t = self.physics.time
        for l in self.links:
            l.before_step(t)
        obs = Obs(self.bus)
        for c in self.controllers:
            key = id(c)
            if c.rate:
                due = self._next.get(key, t)
                if t + 1e-9 < due:
                    continue
                period = 1.0 / c.rate
                self._next[key] = due + period if due + period > t else t + period
            self._runs[key] = self._runs.get(key, 0) + 1
            self.publish(c.step(obs, t), source=c.name or type(c).__name__)
        self.physics.step()
        t = self.physics.time
        if self.renderer is not None:
            self.renderer.step(t)
        for l in self.links:
            l.after_step(t)
        for cb in self.callbacks:
            cb.on_step(self, t)
        self.steps += 1
        if self.realtime:
            ahead = (t - self._sim0) / self.speed - (time.monotonic() - self._wall0)
            if ahead > 0:
                time.sleep(ahead)
            elif ahead < -0.5:      # fell far behind (a breakpoint, a slow start): do not sprint to catch up
                self._wall0 = time.monotonic() - (t - self._sim0) / self.speed

    def run(self, duration: Optional[float] = None, steps: Optional[int] = None, until: Optional[Callable] = None) -> "App":
        """Step until `duration` seconds of app time, `steps` steps, `until(app)` is true, or Ctrl+C."""
        self.setup()
        self._stop = False
        t_end = None if duration is None else self.physics.time + duration
        n = 0
        try:
            while not self._stop:
                if t_end is not None and self.physics.time >= t_end - 1e-9:
                    break
                if steps is not None and n >= steps:
                    break
                if until is not None and until(self):
                    break
                self.step()
                n += 1
        except KeyboardInterrupt:
            pass
        return self

    def as_env(self, **kw):
        """This stack as a Gymnasium environment (see `roboforge.env.GymEnv`)."""
        from .env import GymEnv
        return GymEnv(self, **kw)

    # --- introspection -----------------------------------------------------
    def controller_rate(self, c: Controller) -> float:
        """Measured rate of a controller in Hz of app time."""
        elapsed = self.physics.time - self._sim0
        return self._runs.get(id(c), 0) / elapsed if elapsed > 0 else 0.0

    def describe(self) -> dict:
        """Components and signal wiring, as plain data (used by `roboforge info`)."""
        d = {
            "name": self.name,
            "robot": None if self.robot is None else {"name": self.robot.name, "profile": self.robot.profile, "model": self.robot.model, "cameras": self.robot.cameras},
            "physics": {**self.physics.describe(), "dt": self.physics.dt},
            "renderer": None if self.renderer is None else self.renderer.describe(),
            "links": [l.describe() for l in self.links],
            "controllers": [{"type": c.name or type(c).__name__, "rate": c.rate, "measured_rate": round(self.controller_rate(c), 1)} for c in self.controllers],
            "callbacks": [cb.describe() for cb in self.callbacks],
            "signals": self.bus.describe(),
            "realtime": self.realtime,
        }
        return d
