"""Joint-space position control for arms and other articulated robots."""
from __future__ import annotations

import numpy as np

from .. import msgs
from ..errors import ConfigError
from . import Controller
from .pid import PID


class JointPD(Controller):
    """Drive torque actuators so the joints follow `cmd/joint` position targets.

    A `JointCommand` with `names` may address any subset of the joints in any
    order; without names it must give every joint, in the robot's order.
    Gains may be scalars or one value per joint. With no target yet, it holds
    the pose the robot had when the controller first ran.
    """

    def __init__(self, kp=60.0, ki=20.0, kd=6.0, rate=None, limit=None, target=None):
        self.rate = rate
        self.pid = PID(np.asarray(kp, float), np.asarray(ki, float), np.asarray(kd, float), integral_limit=2.0)
        self.limit = limit
        self.target = None if target is None else np.asarray(target, float).reshape(-1)
        self._t = self._seen = None
        self._names = []

    def setup(self, app):
        self._range = getattr(app.physics, "ctrl_range", None)
        self._names = list(getattr(app.physics, "joint_names", []) or [])
        robot = app.robot.name if app.robot else "this app"
        if hasattr(app.physics, "joint_names") and not self._names:
            raise ConfigError(f"the joint_pd controller drives joints, and {robot} has none. "
                              f"Use it with an arm or another articulated robot.")
        nu = len(getattr(app.physics, "actuator_names", self._names))
        if self._names and nu != len(self._names):
            raise ConfigError(f"the joint_pd controller needs one actuator per joint, but {robot} has {len(self._names)} joints "
                              f"({', '.join(self._names)}) and {nu} actuators. Add a <motor> per joint to the model, or write your own controller.")
        if self.target is not None:
            self._check(self.target, "its `target`")

    def _check(self, q, what):
        n = len(self._names)
        if n and len(q) != n:
            raise ValueError(f"{what} has {len(q)} positions but the robot has {n} joints ({', '.join(self._names)})")
        if not np.all(np.isfinite(q)):
            raise ValueError(f"{what} contains nan or inf: {q}")

    def reset(self):
        self.pid.reset()
        self._t = None

    def step(self, obs, t):
        js = obs.joint
        if js is None:
            return None
        cmd = obs.get(msgs.CMD_JOINT)
        if cmd is not None and cmd is not self._seen and len(cmd.position):
            target = np.asarray(cmd.position, float).reshape(-1)
            names = list(getattr(cmd, "names", None) or [])
            if names and self._names and names != self._names:
                # Addressed by name: any subset, in any order. Joints not named keep their target.
                if len(names) != len(target):
                    raise ValueError(f"the cmd/joint target names {len(names)} joints but gives {len(target)} positions")
                unknown = [n for n in names if n not in self._names]
                if unknown:
                    raise ValueError(f"the cmd/joint target names unknown joints: {', '.join(unknown)}. This robot has {', '.join(self._names)}")
                full = (self.target if self.target is not None else js.position).copy()
                for n, p in zip(names, target):
                    full[self._names.index(n)] = p
                target = full
            self._check(target, "the cmd/joint target")
            self.target, self._seen = target, cmd
        if self.target is None:
            self.target = js.position.copy()
        n = min(len(self.target), len(js.position))
        dt = 0.0 if self._t is None else t - self._t
        self._t = t
        u = self.pid(self.target[:n] - js.position[:n], dt, rate=-js.velocity[:n])
        if self.limit is not None:
            u = np.clip(u, -self.limit, self.limit)
        if self._range is not None:
            u = np.clip(u, self._range[:n, 0], self._range[:n, 1])
        return msgs.ActuatorCommand(values=u)
