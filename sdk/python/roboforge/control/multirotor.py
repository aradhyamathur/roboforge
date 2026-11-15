"""Cascaded position / attitude control for multirotors."""
from __future__ import annotations

import numpy as np

from .. import msgs
from ..frames import quat_to_mat
from . import Controller

G = 9.81


class MultirotorController(Controller):
    """Fly to `cmd/setpoint` (world position + yaw, or a velocity).

    Outer loop: position and velocity error -> desired acceleration.
    Inner loop: geometric attitude control -> body torques.
    Allocation: the map from motor inputs to (thrust, torques) is read from
    the physics backend (`physics.allocation`), so the same controller flies
    any rotor layout the model describes. Motors stay off until the vehicle
    is armed, if anything publishes `state/status`.
    """

    def __init__(self, rate=None, kp_pos=(4.0, 4.0, 6.0), kd_pos=(3.0, 3.0, 4.0), ki_pos=(0.3, 0.3, 1.0),
                 kp_att=(9.0, 9.0, 3.0), kd_att=(1.6, 1.6, 0.8), max_tilt=0.6, max_speed=5.0, hover=None):
        self.rate = rate
        self.kp_pos, self.kd_pos, self.ki_pos = (np.asarray(x, float) for x in (kp_pos, kd_pos, ki_pos))
        self.kp_att, self.kd_att = np.asarray(kp_att, float), np.asarray(kd_att, float)
        self.max_tilt, self.max_speed = max_tilt, max_speed
        self.hover = None if hover is None else np.asarray(hover, float)
        self.reset()

    def setup(self, app):
        alloc = getattr(app.physics, "allocation", None)
        if alloc is None:
            raise RuntimeError("MultirotorController needs a physics backend that reports rotor allocation (MujocoPhysics does).")
        self.A = np.asarray(alloc["matrix"], float)          # [Fz, tx, ty, tz] = A @ ctrl
        self.A_inv = np.linalg.pinv(self.A)
        self.mass = float(alloc["mass"])
        self.inertia = np.asarray(alloc["inertia"], float)
        self.n = self.A.shape[1]

    def reset(self):
        self._int = np.zeros(3)
        self._t = None
        self._hold = None

    def step(self, obs, t):
        pose, twist = obs.pose, obs.twist
        if pose is None or twist is None:
            return None
        status = obs.status
        if status is not None and not status.armed:
            self.reset()
            return msgs.ActuatorCommand(values=np.zeros(self.n))
        dt = 0.0 if self._t is None else t - self._t
        self._t = t
        sp = obs.setpoint
        if sp is None:
            if self._hold is None:
                self._hold = self.hover if self.hover is not None else pose.position.copy()
            sp = msgs.Setpoint(position=self._hold)
        R = quat_to_mat(pose.orientation)
        vel_ff = np.zeros(3) if sp.velocity is None else np.asarray(sp.velocity, float)
        if sp.position is not None:
            e_pos = np.asarray(sp.position, float) - pose.position
            self._int = np.clip(self._int + e_pos * dt, -2.0, 2.0)
            v_des = np.clip(self.kp_pos * e_pos / np.maximum(self.kd_pos, 1e-6), -self.max_speed, self.max_speed) + vel_ff
            acc = self.kd_pos * (v_des - twist.linear) + self.ki_pos * self._int
        else:
            acc = self.kd_pos * (vel_ff - twist.linear)
        acc[2] += G
        # Limit tilt: cap the horizontal part of the demanded acceleration.
        horiz = np.linalg.norm(acc[:2])
        cap = max(acc[2], 1.0) * np.tan(self.max_tilt)
        if horiz > cap:
            acc[:2] *= cap / horiz
        thrust = self.mass * float(acc @ R[:, 2])
        z_des = acc / (np.linalg.norm(acc) or 1.0)
        x_c = np.array([np.cos(sp.yaw), np.sin(sp.yaw), 0.0])
        y_des = np.cross(z_des, x_c)
        y_des /= np.linalg.norm(y_des) or 1.0
        R_des = np.column_stack([np.cross(y_des, z_des), y_des, z_des])
        E = 0.5 * (R_des.T @ R - R.T @ R_des)
        e_R = np.array([E[2, 1], E[0, 2], E[1, 0]])
        torque = self.inertia * (-self.kp_att * e_R - self.kd_att * twist.angular) * 10.0
        u = self.A_inv @ np.array([max(thrust, 0.0), *torque])
        return msgs.ActuatorCommand(values=np.clip(u, 0.0, 1.0))
