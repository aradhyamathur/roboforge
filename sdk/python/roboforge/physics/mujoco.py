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
from ..errors import ConfigError, SimulationError, positive  # noqa: E402


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
        positive(timestep, "physics timestep", allow_none=True)
        positive(substeps, "physics substeps", integer=True)
        self._timestep, self.substeps, self.keyframe, self._gravity = timestep, int(substeps), keyframe, gravity
        self.model = self.data = None
        self.time = 0.0

    # --- setup -------------------------------------------------------------
    def setup(self, app):
        if app.robot is None:
            raise ConfigError("the mujoco physics backend needs a robot (a model to load). Set `robot:`, or use `physics: none`.")
        self.bus, self.robot = app.bus, app.robot
        try:
            self.model = self._load(self.robot)
        except ConfigError:
            raise
        except Exception as e:
            raise ConfigError(f"MuJoCo could not load {self.robot.model}: {' '.join(str(e).split())}") from None
        if self._timestep:
            self.model.opt.timestep = self._timestep
        if self._gravity is not None:
            self.model.opt.gravity[:] = self._gravity
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep) * self.substeps
        m = self.model
        self.base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, self.robot.base) if self.robot.base else (1 if m.nbody > 1 else 0)
        if self.base < 0:
            bodies = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(1, m.nbody)]
            raise ConfigError(f"the model has no body named {self.robot.base!r}. Its bodies: {', '.join(b for b in bodies if b)}")
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

    @staticmethod
    def _load(robot):
        """Compile the robot's model, adding what a bare model lacks (see `Robot`)."""
        urdf = str(robot.model).lower().endswith(".urdf")
        ground = urdf if robot.ground is None else robot.ground
        if robot.motors is False and not robot.floating_base and robot.position is None and not ground:
            return mujoco.MjModel.from_xml_path(robot.model)
        spec = mujoco.MjSpec.from_file(robot.model)
        J = mujoco.mjtJoint
        joints = [j for j in spec.joints if j.type in (J.mjJNT_HINGE, J.mjJNT_SLIDE)]
        motors = (bool(joints) and not spec.actuators) if robot.motors is None else robot.motors
        if not (motors or robot.floating_base or robot.position is not None or ground):
            return mujoco.MjModel.from_xml_path(robot.model)        # nothing to add: load it exactly as written
        if motors:
            driven = {a.target for a in spec.actuators}
            for j in joints:
                if j.name in driven:
                    continue
                a = spec.add_actuator(name=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT, target=j.name)
                lo, hi = j.actfrcrange                              # a URDF's <limit effort=...>
                if hi > lo:
                    a.ctrllimited, a.ctrlrange = mujoco.mjtLimited.mjLIMITED_TRUE, [lo, hi]
        if robot.floating_base or robot.position is not None:
            base = spec.body(robot.base) if robot.base else spec.worldbody.first_body()
            if base is None:
                raise ConfigError(f"the model has no body {'named ' + repr(robot.base) if robot.base else 'to use as the base'}")
            if robot.position is not None:
                pos = np.asarray(robot.position, float).reshape(-1)
                if len(pos) != 3 or not np.all(np.isfinite(pos)):
                    raise ConfigError(f"robot position must be [x, y, z], got {robot.position!r}")
                base.pos = pos
            if robot.floating_base and not any(j.type == J.mjJNT_FREE for j in spec.joints):
                base.add_freejoint()
        if urdf and not spec.geoms:
            print(f"roboforge: {robot.name} has no collision geometry, so it will be invisible and pass through the ground. "
                  f"MuJoCo ignores a URDF's <visual> shapes unless the URDF contains "
                  f"<mujoco><compiler discardvisual=\"false\"/></mujoco>; add that, or add <collision> shapes.", flush=True)
        if ground:
            spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05], name="ground", rgba=[0.86, 0.85, 0.82, 1])
            spec.worldbody.add_light(pos=[0, 0, 4], dir=[0, 0, -1])
        return spec.compile()

    def _sensor(self, kind):
        m = self.model
        for i in range(m.nsensor):
            if int(m.sensor_type[i]) == int(kind):
                a = m.sensor_adr[i]
                return slice(a, a + m.sensor_dim[i])
        return None

    def _on_cmd(self, _name, msg):
        try:
            cmd = np.asarray(msg.values, float).reshape(-1)
        except (TypeError, ValueError):
            raise ValueError(f"cmd/actuators needs a list of numbers, got {msg.values!r:.80}") from None
        nu = self.model.nu
        if len(cmd) != nu:
            raise ValueError(f"cmd/actuators has {len(cmd)} values but {self.robot.name} has {nu} actuators"
                             + (f" ({', '.join(self.actuator_names)})" if nu else "; the model defines none"))
        if not np.all(np.isfinite(cmd)):
            raise ValueError(f"cmd/actuators contains nan or inf: {cmd}. A controller upstream produced it; "
                             f"check its inputs and gains.")
        self._cmd = cmd

    @property
    def raw(self):
        return self.model, self.data

    def camera(self, name: str):
        """`(pose, fovy)`: the camera's 4x4 camera-to-world matrix (OpenGL axes, looks along -z) and vertical field of view in degrees."""
        m = self.model
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, name) if isinstance(name, str) else -1
        if cid < 0:
            have = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(m.ncam)]
            raise ConfigError(f"the model has no camera named {name!r}. Its cameras: {', '.join(h for h in have if h) or 'none'}")
        T = np.eye(4)
        T[:3, :3] = self.data.cam_xmat[cid].reshape(3, 3)
        T[:3, 3] = self.data.cam_xpos[cid]
        return T, float(m.cam_fovy[cid])

    @property
    def joint_range(self):
        """`(n_joints, 2)` position limits; +-inf where a joint is unlimited."""
        m = self.model
        lim = m.jnt_limited[self._joints].astype(bool)
        return np.where(lim[:, None], m.jnt_range[self._joints], np.array([-np.inf, np.inf]))

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
            self.data.ctrl[:] = np.clip(self._cmd, self.ctrl_range[:, 0], self.ctrl_range[:, 1])
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        # MuJoCo resets the state when the simulation blows up, which would make
        # time jump back and a timed run never finish. Say so instead.
        bad = int(self.data.warning[int(mujoco.mjtWarning.mjWARN_BADQACC)].number)
        if bad > self._bad:
            self._bad = bad
            raise SimulationError(f"the simulation became unstable near t={self.time:.3f}s (MuJoCo reported a nan or huge acceleration). "
                                  f"Reduce the timestep or the controller/actuator gains, or use an implicit integrator "
                                  f"(<option integrator=\"implicitfast\"/> in the model).")
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
