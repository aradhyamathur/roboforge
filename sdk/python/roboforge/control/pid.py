"""A PID loop, usable on scalars or arrays."""
from __future__ import annotations

import numpy as np


class PID:
    """`u = kp*e + ki*int(e) + kd*de/dt`, with output and integral limits.

        pid = PID(kp=2.0, ki=0.5, kd=0.1, limit=10.0)
        u = pid(target - measured, dt)

    Pass `rate=` (the measured derivative of the error, with its sign) to
    avoid differentiating a noisy or stepping error.
    """

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, limit=None, integral_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit, self.integral_limit = limit, integral_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self._prev = None

    def __call__(self, error, dt: float, rate=None):
        error = np.asarray(error, float)
        self.integral = self.integral + error * dt
        if self.integral_limit is not None:
            self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        if rate is None:
            rate = 0.0 if self._prev is None or dt <= 0 else (error - self._prev) / dt
        self._prev = error
        u = self.kp * error + self.ki * self.integral + self.kd * np.asarray(rate, float)
        if self.limit is not None:
            u = np.clip(u, -self.limit, self.limit)
        return u
