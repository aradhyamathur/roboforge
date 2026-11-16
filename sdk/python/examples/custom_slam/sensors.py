"""Simulated sensors of your own: wheel odometry and a landmark detector.

A sensor is a callback that reads the true state from the bus after each
physics step and publishes a noisy measurement of it, at its own rate. Its
messages are dataclasses with a `signal` name (and `as_array()`, so the
recorder can keep them). Anything downstream, a SLAM method here, cannot
tell them from a real driver's.
"""
from dataclasses import dataclass, field
from typing import ClassVar

import numpy as np

import roboforge as rf
from roboforge.frames import quat_to_mat, yaw_of


@dataclass
class WheelOdometry:
    """Forward speed (m/s) and turn rate (rad/s), as measured by the wheels."""
    signal: ClassVar[str] = "sensor/odometry"
    v: float = 0.0
    w: float = 0.0
    dt: float = 0.0
    t: float = 0.0

    def as_array(self):
        return [self.v, self.w]


@dataclass
class Landmarks:
    """Range (m) and bearing (rad, from the robot's heading) to each landmark seen, by id."""
    signal: ClassVar[str] = "sensor/landmarks"
    ids: list = field(default_factory=list)
    ranges: np.ndarray = field(default_factory=lambda: np.zeros(0))
    bearings: np.ndarray = field(default_factory=lambda: np.zeros(0))
    t: float = 0.0


# Posts around the robot's circle (world x, y in metres).
WORLD = np.array([[2.0, 0.8], [1.3, 2.4], [-0.4, 2.9], [-2.0, 1.6], [-1.8, -0.6], [-0.3, -1.6], [1.4, -1.0], [2.6, 2.2]])


class Sensors(rf.Callback):
    """Wheel odometry at `odom_rate` with noise and a small turn-rate bias (so
    dead reckoning drifts, as real odometry does); landmark range and bearing
    at `landmark_rate` within `max_range`."""
    name = "sensors"
    outputs = ["sensor/odometry", "sensor/landmarks"]

    def __init__(self, odom_rate=50.0, landmark_rate=10.0, max_range=3.0, speed_noise=0.02, turn_noise=0.02,
                 turn_bias=0.03, range_noise=0.05, bearing_noise=0.02, seed=0):
        self.odom_rate, self.landmark_rate, self.max_range = odom_rate, landmark_rate, max_range
        self.speed_noise, self.turn_noise, self.turn_bias = speed_noise, turn_noise, turn_bias
        self.range_noise, self.bearing_noise = range_noise, bearing_noise
        self.seed = seed

    def on_start(self, app):
        self.rng = np.random.default_rng(self.seed)
        self._next_odom = self._next_lm = app.physics.time

    def on_step(self, app, t):
        pose, twist = app.bus.latest("state/pose"), app.bus.latest("state/twist")
        if t + 1e-9 >= self._next_odom:
            self._next_odom += 1.0 / self.odom_rate
            v = float((quat_to_mat(pose.orientation).T @ twist.linear)[0])           # forward speed, body frame
            w = float(twist.angular[2])
            app.bus.publish("sensor/odometry", WheelOdometry(v=v + self.rng.normal(0, self.speed_noise),
                                                             w=w + self.turn_bias + self.rng.normal(0, self.turn_noise),
                                                             dt=1.0 / self.odom_rate, t=t), self.name)
        if t + 1e-9 >= self._next_lm:
            self._next_lm += 1.0 / self.landmark_rate
            d = WORLD - pose.position[:2]
            r = np.hypot(d[:, 0], d[:, 1])
            seen = np.nonzero(r < self.max_range)[0]
            b = np.arctan2(d[seen, 1], d[seen, 0]) - yaw_of(pose.orientation)
            app.bus.publish("sensor/landmarks", Landmarks(ids=seen.tolist(),
                                                          ranges=r[seen] + self.rng.normal(0, self.range_noise, len(seen)),
                                                          bearings=np.angle(np.exp(1j * (b + self.rng.normal(0, self.bearing_noise, len(seen))))),
                                                          t=t), self.name)
