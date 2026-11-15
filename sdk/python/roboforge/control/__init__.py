"""Controllers: from a ten-line PID loop to a slow learned policy.

The whole contract is one method:

    class Hold(Controller):
        rate = 200                      # Hz; omit to run every physics step
        def step(self, obs, t):
            return JointCommand(position=[0.0, 0.5, -0.5])

`step` may return nothing, one message (published on that message's standard
signal), or a dict of `{signal: message}`. A plain function works too:
`controller(lambda obs, t: ..., rate=50)`.

Several controllers run side by side, each at its own rate, and talk through
the bus: a 5 Hz policy publishes `cmd/setpoint`, a 250 Hz controller turns it
into motor commands. That is all a "hierarchy" is here.

`Async` wraps a controller whose `step` is slow (a large model, a remote
call) so it never blocks the loop: it runs in a worker thread, its results
are buffered (it may return a `Chunk` of several future actions), and a
fallback action is used when it runs late.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

from .. import msgs


class Obs:
    """What a controller sees: the latest value of every signal.

    `obs["state/joint"]`, `obs.get(name, default)`, and shortcuts for the
    common ones: `obs.joint`, `obs.pose`, `obs.twist`, `obs.imu`, `obs.status`,
    `obs.setpoint`, `obs.camera("front")`.
    """

    def __init__(self, bus, snapshot: Optional[dict] = None):
        self._bus = bus
        self._snap = snapshot

    def get(self, name: str, default: Any = None) -> Any:
        if self._snap is not None:
            return self._snap.get(name, default)
        return self._bus.latest(name, default)

    def __getitem__(self, name: str) -> Any:
        v = self.get(name)
        if v is None:
            raise KeyError(f"no signal {name!r} yet. Seen so far: {', '.join(self._bus.names()) or 'nothing'}")
        return v

    def __contains__(self, name: str) -> bool:
        return self.get(name) is not None

    def frozen(self) -> "Obs":
        """A copy that will not change while a slow controller is thinking."""
        return Obs(self._bus, {n: self._bus.latest(n) for n in self._bus.names()})

    joint = property(lambda s: s.get(msgs.JOINT_STATE))
    pose = property(lambda s: s.get(msgs.POSE))
    twist = property(lambda s: s.get(msgs.TWIST))
    imu = property(lambda s: s.get(msgs.IMU))
    status = property(lambda s: s.get(msgs.STATUS))
    setpoint = property(lambda s: s.get(msgs.CMD_SETPOINT))
    instruction = property(lambda s: s.get(msgs.CMD_INSTRUCTION))

    def camera(self, name: str):
        return self.get(msgs.CAMERA.format(name=name))


class Controller:
    """Base class. Override `step`; `setup` and `reset` are optional."""

    rate: Optional[float] = None    # Hz. None: every physics step.
    name: str = ""

    def __init__(self, rate: Optional[float] = None, name: str = ""):
        # Lets `{type: "controller:Mine", rate: 50}` work for a subclass with no __init__.
        if rate is not None:
            self.rate = rate
        if name:
            self.name = name

    def setup(self, app) -> None:
        """Called once before the loop, with the app (robot, physics, bus)."""

    def step(self, obs: Obs, t: float):
        raise NotImplementedError

    def reset(self) -> None:
        """Called when the app is reset."""

    def close(self) -> None:
        """Called when the app shuts down."""


class _Function(Controller):
    def __init__(self, fn: Callable, rate: Optional[float] = None, name: str = ""):
        self.fn, self.rate, self.name = fn, rate, name or getattr(fn, "__name__", "function")

    def step(self, obs, t):
        return self.fn(obs, t)


def controller(fn: Optional[Callable] = None, *, rate: Optional[float] = None, name: str = ""):
    """Turn a function `(obs, t) -> action` into a controller. Usable as a decorator."""
    if fn is None:
        return lambda f: _Function(f, rate, name)
    return _Function(fn, rate, name)


@dataclass
class Chunk:
    """Several future actions from one inference, `dt` seconds apart."""
    actions: list
    dt: float


class Async(Controller):
    """Run a slow controller without slowing the loop.

    The wrapped controller's `step` runs in a worker thread on a frozen copy
    of the observations. This wrapper's own `step` returns immediately with
    the next buffered action. When nothing fresh has arrived for `timeout`
    seconds it returns `fallback` instead: a message, a callable
    `(obs, t) -> action`, `"hold"` (repeat the last action) or `None`.
    """

    def __init__(self, inner: Controller, rate: Optional[float] = None, timeout: float = 1.0, fallback: Any = "hold"):
        self.inner = inner
        self.rate = rate if rate is not None else inner.rate
        self.name = f"async({inner.name or type(inner).__name__})"
        self.timeout, self.fallback = timeout, fallback
        self._lock = threading.Lock()
        self._queue: list = []          # (due_time, action)
        self._last = None
        self._last_result_wall = None
        self._busy = False
        self._stop = False
        self._job = None
        self._wake = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.inferences = 0         # completed calls of the wrapped controller
        self.errors = 0             # calls that raised; the last exception is in `last_error`
        self.last_error: Optional[BaseException] = None
        self.late = 0               # steps answered by the fallback
        self.latency = 0.0          # seconds the last call took

    def setup(self, app):
        self.inner.setup(app)
        self._thread = threading.Thread(target=self._work, name=self.name, daemon=True)
        self._thread.start()

    def _work(self):
        while not self._stop:
            self._wake.wait(0.1)
            if self._stop:
                break
            with self._lock:
                job, self._job = self._job, None
                self._wake.clear()
            if job is None:
                continue
            obs, t = job
            t0 = time.monotonic()
            try:
                result = self.inner.step(obs, t)
            except Exception as e:      # a failing policy must not kill the loop
                result = None
                self.errors += 1
                self.last_error = e
                if self.errors <= 3 or self.errors % 100 == 0:
                    print(f"roboforge: {self.name} raised {type(e).__name__}: {e} (error {self.errors}; the fallback applies)", flush=True)
            with self._lock:
                self.latency = time.monotonic() - t0
                self.inferences += 1
                self._busy = False
                if result is not None:
                    self._last_result_wall = time.monotonic()
                    if isinstance(result, Chunk):
                        # Start the chunk where we are now, not where inference began.
                        self._queue = [(t + self.latency + i * result.dt, a) for i, a in enumerate(result.actions)]
                    else:
                        self._queue = [(t, result)]

    def step(self, obs, t):
        with self._lock:
            if not self._busy:
                self._busy = True
                self._job = (obs.frozen(), t)
                self._wake.set()
            action = None
            while self._queue and self._queue[0][0] <= t + 1e-9:
                action = self._queue.pop(0)[1]
            stale = self._last_result_wall is None or (time.monotonic() - self._last_result_wall) > self.timeout
            if action is not None:
                self._last = action
                return action
            if not stale:
                return None                      # nothing new is due; lower layers keep their target
            self.late += 1
            fb = self.fallback
        if callable(fb):
            return fb(obs, t)
        if isinstance(fb, str):
            return self._last if fb == "hold" else None
        return fb

    def reset(self):
        with self._lock:
            self._queue, self._last, self._last_result_wall = [], None, None
        self.inner.reset()

    def close(self):
        self._stop = True
        self._wake.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)      # a call still in flight is abandoned after that
        self.inner.close()


from .pid import PID  # noqa: E402

__all__ = ["Obs", "Controller", "controller", "Chunk", "Async", "PID"]
