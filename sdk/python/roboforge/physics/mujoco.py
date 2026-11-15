"""MuJoCo as the physics backend."""
from __future__ import annotations

import os

# Off-screen rendering without a display needs EGL; this must be decided
# before mujoco is imported. An explicit MUJOCO_GL always wins.
if "MUJOCO_GL" not in os.environ and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
    os.environ["MUJOCO_GL"] = "egl"

import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from .. import msgs  # noqa: E402
from ..component import Physics  # noqa: E402


class MujocoPhysics(Physics):
    """Steps a MuJoCo model and mirrors it on the bus.

    Publishes `clock`, `state/joint` (hinge and slide joints), `state/pose`
    and `state/twist` of the robot's base body, and `sensor/imu`. Applies
    `cmd/actuators` to `data.ctrl`, clipped to each actuator's range.

    For full control use `physics.model` and `physics.data` (also as
    `physics.raw`), or pass `substeps` / `timestep` here.
    """
    name = "mujoco"
    inputs = [msgs.CMD_ACTUATORS]

    def __init__(self, timestep: float = None, substeps: int = 1, keyframe: int = 0, gravity=None):
        self._timestep, self.substeps, self.keyframe, self._gravity = timestep, int(substeps), keyframe, gravity
        self.model = self.data = None
        self.time = 0.0

    # --- setup -------------------------------------------------------------
    def setup(self, app):
        if app.robot is None:
            raise ValueError("MujocoPhysics needs a robot (a model to load).")
        self.bus, self.robot = app.bus, app.robot
        self.model = mujoco.MjModel.from_xml_path(self.robot.model)
        if self._timestep:
            self.model.opt.timestep = self._timestep
        if self._gravity is not None:
            self.model.opt.gravity[:] = self._gravity
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep) * self.substeps
        m = self.model
        self.base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, self.robot.base) if self.robot.base else (1 if m.nbody > 1 else 0)
        if self.base < 0:
            raise ValueError(f"the model has no body named {self.robot.base!r}")
        # int() on both sides: MuJoCo's enum values do not compare reliably with numpy integers.
        kinds = {int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)}
        self._joints = [j for j in range(m.njnt) if int(m.jnt_type[j]) in kinds]
        self.joint_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or f"joint{j}" for j in self._joints]
        self._qadr = np.array([m.jnt_qposadr[j] for j in self._joints], int)
        self._vadr = np.array([m.jnt_dofadr[j] for j in self._joints], int)
        self.actuator_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or f"actuator{i}" for i in range(m.nu)]
        lim = m.actuator_ctrllimited.astype(bool)
        self.ctrl_range = np.where(lim[:, None], m.actuator_ctrlrange, np.array([-np.inf, np.inf]))
        self._acc = self._sensor(mujoco.mjtSensor.mjSENS_ACCELEROMETER)
        self._gyro = self._sensor(mujoco.mjtSensor.mjSENS_GYRO)
        self._cmd = None
        self._prev_v = None
        self.outputs = [msgs.CLOCK, msgs.POSE, msgs.TWIST, msgs.IMU] + ([msgs.JOINT_STATE] if self._joints else [])
        self.bus.subscribe(msgs.CMD_ACTUATORS, self._on_cmd)
        self.reset()

    def _sensor(self, kind):
        m = self.model
        for i in range(m.nsensor):
            if int(m.sensor_type[i]) == int(kind):
                a = m.sensor_adr[i]
                return slice(a, a + m.sensor_dim[i])
        return None

    def _on_cmd(self, _name, msg):
        self._cmd = np.asarray(msg.values, float)

    @property
    def raw(self):
        return self.model, self.data

    @property
    def allocation(self):
        """For multirotors: how actuator inputs map to body thrust and torques.

        `matrix` (4 x n) maps `ctrl` to `[Fz, tx, ty, tz]` in the base frame,
        from each site-attached actuator's gear and lever arm. Returns None if
        the model has no such actuators.
        """
        m = self.model
        cols = []
        com = m.body_ipos[self.base]
        for i in range(m.nu):
            if int(m.actuator_trntype[i]) != int(mujoco.mjtTrn.mjTRN_SITE):
                return None
            site = m.actuator_trnid[i, 0]
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, m.site_quat[site])
            R = R.reshape(3, 3)
            force, torque = R @ m.actuator_gear[i, :3], R @ m.actuator_gear[i, 3:6]
            torque = torque + np.cross(m.site_pos[site] - com, force)
            cols.append([force[2], *torque])
        if not cols:
            return None
        return {"matrix": np.array(cols).T, "mass": float(m.body_subtreemass[self.base]), "inertia": m.body_inertia[self.base].copy()}

    # --- stepping ----------------------------------------------------------
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        if self.model.nkey > self.keyframe >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.keyframe)
        mujoco.mj_forward(self.model, self.data)
        self._cmd, self._prev_v = None, None
        self._bad = int(self.data.warning[int(mujoco.mjtWarning.mjWARN_BADQACC)].number)
        self.time = float(self.data.time)
        self._publish()

    def step(self):
        if self._cmd is not None:
            n = min(len(self._cmd), self.model.nu)
            self.data.ctrl[:n] = np.clip(self._cmd[:n], self.ctrl_range[:n, 0], self.ctrl_range[:n, 1])
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        # MuJoCo resets the state when the simulation blows up, which would make
        # time jump back and a timed run never finish. Say so instead.
        bad = int(self.data.warning[int(mujoco.mjtWarning.mjWARN_BADQACC)].number)
        if bad > self._bad:
            self._bad = bad
            raise RuntimeError(f"the simulation became unstable near t={self.time:.3f}s. "
                               f"Reduce the timestep or the controller/actuator gains, or use an implicit integrator.")
        self.time = float(self.data.time)
        self._publish()

    def _publish(self):
        d, t, pub = self.data, self.time, self.bus.publish
        pub(msgs.CLOCK, msgs.Clock(t=t), "mujoco")
        if self._joints:
            pub(msgs.JOINT_STATE, msgs.JointState(names=self.joint_names, position=d.qpos[self._qadr].copy(),
                                                    velocity=d.qvel[self._vadr].copy(), effort=d.qfrc_actuator[self._vadr].copy(), t=t), "mujoco")
        pos, quat = d.xpos[self.base].copy(), d.xquat[self.base].copy()
        R = d.xmat[self.base].reshape(3, 3)
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, d, mujoco.mjtObj.mjOBJ_BODY, self.base, vel, 0)   # world axes: [angular, linear]
        ang_body, lin_world = R.T @ vel[:3], vel[3:].copy()
        pub(msgs.POSE, msgs.Pose(position=pos, orientation=quat, t=t), "mujoco")
        pub(msgs.TWIST, msgs.Twist(linear=lin_world, angular=ang_body, t=t), "mujoco")
        if self._acc is not None and self._gyro is not None:
            accel, gyro = d.sensordata[self._acc].copy(), d.sensordata[self._gyro].copy()
        else:
            # No IMU in the model: differentiate the base velocity and remove gravity.
            a_world = np.zeros(3) if self._prev_v is None else (lin_world - self._prev_v) / self.dt
            accel, gyro = R.T @ (a_world - self.model.opt.gravity), ang_body
        self._prev_v = lin_world
        pub(msgs.IMU, msgs.Imu(accel=accel, gyro=gyro, orientation=quat, t=t), "mujoco")

    def describe(self):
        d = super().describe()
        if self.model is not None:
            d.update(joints=self.joint_names, actuators=self.actuator_names)
        return d
