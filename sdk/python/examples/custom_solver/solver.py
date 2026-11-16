"""A physics backend written from scratch: a two-link arm in plain numpy.

Implementing `rf.Physics` is all it takes for every controller, link and
callback in the SDK to work on top of it. This one solves the arm's
equations of motion directly (rigid bodies, gravity, joint damping, torque
motors) with a fourth-order Runge-Kutta step. It is the same arm as
`custom_urdf/twolink.urdf`, so it can be checked against MuJoCo:
`python3 compare.py`.

What a backend must provide: `dt`, `time`, `setup(app)`, `step()`, `reset()`,
and publish state on the bus after each step (`publish_state` does that). What it may provide, and what
that buys: `joint_names` and `actuator_names` (size checks in joint_pd and
the links), `ctrl_range` (Gymnasium action space, saturation checks), `raw`
(your own objects, for anything the bus does not carry).
"""
import numpy as np

import roboforge as rf

G = 9.81


class TwoLinkArm(rf.Physics):
    """Two links rotating about parallel horizontal axes (y), pointing up at q = 0.

    Angles follow MuJoCo's convention for a hinge about +y: a positive angle
    tips the link towards +x.
    """
    name = "twolink"
    inputs = ["cmd/actuators"]
    outputs = ["clock", "state/joint", "state/pose", "state/twist", "sensor/imu"]

    def __init__(self, timestep=0.002, masses=(0.5, 0.3), lengths=(0.3, 0.3), com=(0.15, 0.15),
                 inertia=(0.005, 0.003), damping=(0.3, 0.2), torque_limits=(20.0, 12.0), start=(0.0, 0.0)):
        from roboforge.errors import positive
        self.dt = positive(timestep, "timestep")
        self.m, self.l, self.c, self.I, self.b = (np.asarray(v, float) for v in (masses, lengths, com, inertia, damping))
        self.limits = np.asarray(torque_limits, float)
        self.start = np.asarray(start, float)
        self.joint_names = ["shoulder", "elbow"]
        self.actuator_names = ["shoulder", "elbow"]
        self.ctrl_range = np.stack([-self.limits, self.limits], axis=1)
        self.joint_range = np.array([[-2.0, 2.0], [-2.3, 2.3]])
        self.time = 0.0

    # --- the part every backend has -------------------------------------
    def setup(self, app):
        self.bus = app.bus
        self.bus.subscribe("cmd/actuators", self._on_command)
        self.reset()

    def reset(self):
        self.q, self.qd = self.start.copy(), np.zeros(2)
        self.tau = np.zeros(2)
        self.time = 0.0
        self._publish()

    def step(self):
        # Classic RK4 on the state (q, qd), with the motor torque held over the step.
        def f(q, qd):
            return qd, self.acceleration(q, qd, self.tau)
        q, qd, h = self.q, self.qd, self.dt
        k1q, k1v = f(q, qd)
        k2q, k2v = f(q + 0.5 * h * k1q, qd + 0.5 * h * k1v)
        k3q, k3v = f(q + 0.5 * h * k2q, qd + 0.5 * h * k2v)
        k4q, k4v = f(q + h * k3q, qd + h * k3v)
        self.q = q + h / 6 * (k1q + 2 * k2q + 2 * k3q + k4q)
        self.qd = qd + h / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
        if not np.all(np.isfinite(self.q)):
            raise rf.SimulationError(f"the two-link solver diverged at t={self.time:.3f}s; lower the timestep or the gains")
        self.time += h
        self._publish()

    @property
    def raw(self):
        return {"q": self.q, "qd": self.qd, "tau": self.tau}

    # --- the physics ----------------------------------------------------------
    def acceleration(self, q, qd, tau):
        """Joint accelerations from the manipulator equation M(q) qdd + C(q, qd) + g(q) = tau - b qd."""
        (m1, m2), (l1, _), (c1, c2), (I1, I2) = self.m, self.l, self.c, self.I
        s2, k2 = np.sin(q[1]), np.cos(q[1])
        M = np.array([[I1 + I2 + m1 * c1 ** 2 + m2 * (l1 ** 2 + c2 ** 2 + 2 * l1 * c2 * k2), I2 + m2 * (c2 ** 2 + l1 * c2 * k2)],
                      [I2 + m2 * (c2 ** 2 + l1 * c2 * k2), I2 + m2 * c2 ** 2]])
        h = m2 * l1 * c2 * s2
        coriolis = np.array([-h * (2 * qd[0] * qd[1] + qd[1] ** 2), h * qd[0] ** 2])
        # Potential energy V = g (m1 c1 cos q1 + m2 (l1 cos q1 + c2 cos(q1 + q2))); this is dV/dq.
        grav = -G * np.array([(m1 * c1 + m2 * l1) * np.sin(q[0]) + m2 * c2 * np.sin(q[0] + q[1]), m2 * c2 * np.sin(q[0] + q[1])])
        return np.linalg.solve(M, tau - self.b * qd - coriolis - grav)

    def _on_command(self, _name, msg):
        u = np.asarray(msg.values, float).reshape(-1)
        if len(u) != 2 or not np.all(np.isfinite(u)):
            raise ValueError(f"cmd/actuators for the two-link arm needs 2 finite values, got {u}")
        self.tau = np.clip(u, -self.limits, self.limits)

    def _publish(self):
        # The clock, and each state message stamped with self.time.
        self.publish_state(joint=rf.JointState(names=self.joint_names, position=self.q.copy(), velocity=self.qd.copy(), effort=self.tau.copy()),
                           pose=rf.Pose(), twist=rf.Twist(),                     # the base is bolted down
                           imu=rf.Imu(accel=np.array([0.0, 0.0, G])))
