"""No simulator: the world is real (or simulated somewhere else).

Time is the wall clock. State arrives through links (a MAVLink client, ROS 2
topics) instead of from a physics engine. Swapping `mujoco` for `none` in a
config is how the same controllers move from simulation to a robot.
"""
from __future__ import annotations

import time

from .. import msgs
from ..component import Physics


class NoPhysics(Physics):
    name = "none"
    outputs = [msgs.CLOCK]

    def __init__(self, rate: float = 100.0):
        self.dt = 1.0 / rate
        self._t0 = time.monotonic()
        self.time = 0.0

    def setup(self, app):
        self.bus = app.bus
        self._t0 = time.monotonic()
        self.time = 0.0

    def step(self):
        # Advance by one tick; the app's real-time pacing does the waiting.
        self.time += self.dt
        self.bus.publish(msgs.CLOCK, msgs.Clock(t=self.time), "none")

    def reset(self):
        self.time = 0.0
