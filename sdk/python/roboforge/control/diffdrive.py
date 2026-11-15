"""Turn a body velocity command into wheel speeds for a differential drive."""
from __future__ import annotations

import numpy as np

from .. import msgs
from . import Controller


class DiffDriveController(Controller):
    """`cmd/velocity` (forward m/s, yaw rad/s) -> left and right wheel rad/s.

    Wheel geometry comes from the robot's `params` (`wheel_radius`,
    `wheel_separation`) unless given here. The command times out: with no new
    command for `timeout` seconds the robot stops.
    """

    def __init__(self, wheel_radius=None, wheel_separation=None, rate=None, timeout=0.5):
        self.rate, self.timeout = rate, timeout
        self.r, self.sep = wheel_radius, wheel_separation
        self._last_cmd = None
        self._last_t = -1e9

    def setup(self, app):
        p = app.robot.params
        self.r = self.r or p.get("wheel_radius", 0.05)
        self.sep = self.sep or p.get("wheel_separation", 0.3)
        self._seen = app.bus.count(msgs.CMD_VELOCITY)
        self._bus = app.bus

    def step(self, obs, t):
        n = self._bus.count(msgs.CMD_VELOCITY)
        if n != self._seen:
            self._seen, self._last_t, self._last_cmd = n, t, obs.get(msgs.CMD_VELOCITY)
        if self._last_cmd is None or t - self._last_t > self.timeout:
            return msgs.ActuatorCommand(values=np.zeros(2))
        v, w = float(self._last_cmd.linear[0]), float(self._last_cmd.angular[2])
        left = (v - w * self.sep / 2.0) / self.r
        right = (v + w * self.sep / 2.0) / self.r
        return msgs.ActuatorCommand(values=np.array([left, right]))
