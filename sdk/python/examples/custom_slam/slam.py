"""A SLAM method of your own: EKF-SLAM with known landmark identities.

It is a callback that subscribes to the sensors' signals and publishes its
estimate as an ordinary `rf.Pose` on `slam/pose`, so everything that takes a
pose can take it instead of ground truth: the recorder, the ROS link (as
odometry), a controller. It also publishes `odom/pose`, odometry alone, to
show what the landmarks buy.

The same shape fits any estimator: wrap ORB-SLAM3, FAST-LIO or a learned
odometry network the same way, reading `sensor/camera/<name>` or
`sensor/imu` instead, and publish `slam/pose`. The evaluation in
`evaluate.py` does not change.
"""
import numpy as np

import roboforge as rf
from roboforge.frames import yaw_of


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class EkfSlam(rf.Callback):
    """State: robot x, y, heading, then x, y of every landmark seen so far."""
    name = "ekf_slam"
    inputs = ["sensor/odometry", "sensor/landmarks"]
    outputs = ["slam/pose", "odom/pose", "slam/landmarks"]

    def __init__(self, speed_noise=0.03, turn_noise=0.05, range_noise=0.05, bearing_noise=0.02, max_landmarks=32):
        self.Q = np.diag([speed_noise ** 2, turn_noise ** 2])         # how much the odometry is trusted
        self.R = np.diag([range_noise ** 2, bearing_noise ** 2])      # how much a landmark sighting is trusted
        self.max_landmarks = max_landmarks

    def on_start(self, app):
        # The map's origin is where the robot starts, the usual SLAM convention.
        p = app.bus.latest("state/pose")
        start = np.array([p.position[0], p.position[1], yaw_of(p.orientation)])
        self.z = float(p.position[2])
        self.mu, self.P = start.copy(), np.diag([1e-6, 1e-6, 1e-6])
        self.slot = {}                                                # landmark id -> index into the state
        self.dead = start.copy()                                      # odometry alone, for comparison
        self.bus = app.bus
        self._subs = [app.bus.subscribe("sensor/odometry", self._predict), app.bus.subscribe("sensor/landmarks", self._update)]

    def on_stop(self, app):
        for unsubscribe in self._subs:
            unsubscribe()

    # --- the filter -------------------------------------------------------------
    def _predict(self, _name, m):
        x, y, th = self.mu[:3]
        dt = m.dt
        self.mu[:3] = [x + m.v * dt * np.cos(th), y + m.v * dt * np.sin(th), wrap(th + m.w * dt)]
        n = len(self.mu)
        F = np.eye(n)
        F[0, 2], F[1, 2] = -m.v * dt * np.sin(th), m.v * dt * np.cos(th)
        G = np.zeros((n, 2))
        G[:3] = [[dt * np.cos(th), 0], [dt * np.sin(th), 0], [0, dt]]
        self.P = F @ self.P @ F.T + G @ self.Q @ G.T
        dx, dy, dth = self.dead
        self.dead = np.array([dx + m.v * dt * np.cos(dth), dy + m.v * dt * np.sin(dth), wrap(dth + m.w * dt)])
        self._publish(m.t)

    def _update(self, _name, m):
        for lid, r, b in zip(m.ids, m.ranges, m.bearings):
            x, y, th = self.mu[:3]
            if lid not in self.slot:
                if len(self.slot) >= self.max_landmarks:
                    continue
                # First sighting: put the landmark where the measurement says, with a wide uncertainty.
                self.slot[lid] = len(self.mu)
                self.mu = np.append(self.mu, [x + r * np.cos(th + b), y + r * np.sin(th + b)])
                P = np.zeros((len(self.mu), len(self.mu)))
                P[:-2, :-2] = self.P
                P[-2:, -2:] = np.eye(2) * 1.0
                self.P = P
            i = self.slot[lid]
            dx, dy = self.mu[i] - x, self.mu[i + 1] - y
            q = dx * dx + dy * dy
            rq = np.sqrt(q)
            predicted = np.array([rq, wrap(np.arctan2(dy, dx) - th)])
            H = np.zeros((2, len(self.mu)))
            H[:, :3] = [[-dx / rq, -dy / rq, 0], [dy / q, -dx / q, -1]]
            H[:, i:i + 2] = [[dx / rq, dy / rq], [-dy / q, dx / q]]
            S = H @ self.P @ H.T + self.R
            K = self.P @ H.T @ np.linalg.inv(S)
            innovation = np.array([r, b]) - predicted
            innovation[1] = wrap(innovation[1])
            self.mu = self.mu + K @ innovation
            self.mu[2] = wrap(self.mu[2])
            self.P = (np.eye(len(self.mu)) - K @ H) @ self.P
        self._publish(m.t)

    def _publish(self, t):
        def pose(x, y, th):
            return rf.Pose(position=np.array([x, y, self.z]), orientation=np.array([np.cos(th / 2), 0, 0, np.sin(th / 2)]), t=t)
        self.bus.publish("slam/pose", pose(*self.mu[:3]), self.name)
        self.bus.publish("odom/pose", pose(*self.dead), self.name)
        self.bus.publish("slam/landmarks", {lid: self.mu[i:i + 2].copy() for lid, i in self.slot.items()}, self.name)
