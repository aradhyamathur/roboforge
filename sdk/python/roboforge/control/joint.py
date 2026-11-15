"""Joint-space position control for arms and other articulated robots."""
from __future__ import annotations

import numpy as np

from .. import msgs
from . import Controller
from .pid import PID


class JointPD(Controller):
    """Drive torque actuators so the joints follow `cmd/joint` position targets.

    Gains may be scalars or one value per joint. With no target yet, it holds
    the pose the robot had when the controller first ran.
    """

    def __init__(self, kp=60.0, ki=20.0, kd=6.0, rate=None, limit=None, target=None):
        self.rate = rate
        self.pid = PID(np.asarray(kp, float), np.asarray(ki, float), np.asarray(kd, float), integral_limit=2.0)
        self.limit = limit
        self.target = None if target is None else np.asarray(target, float)
        self._t = None

    def setup(self, app):
        self._range = getattr(app.physics, "ctrl_range", None)

    def reset(self):
        self.pid.reset()
        self._t = None

    def step(self, obs, t):
        js = obs.joint
        if js is None:
            return None
        cmd = obs.get(msgs.CMD_JOINT)
        if cmd is not None and len(cmd.position):
            self.target = np.asarray(cmd.position, float)
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
