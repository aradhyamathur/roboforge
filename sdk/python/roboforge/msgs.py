"""Message types carried on the bus, and the standard signal names.

Conventions, everywhere inside the SDK:

- World frame is ENU (x east, y north, z up); body frame is FLU (x forward,
  y left, z up). This matches MuJoCo and ROS. Links convert at the edge
  (MAVLink uses NED / FRD).
- Quaternions are (w, x, y, z), as in MuJoCo.
- Units are SI: metres, seconds, radians, newtons.
- Arrays are numpy float64 unless stated.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar, Optional

import numpy as np


def _v(n: int = 3) -> np.ndarray:
    return np.zeros(n)


def _q() -> np.ndarray:
    return np.array([1.0, 0.0, 0.0, 0.0])


# --- standard signal names -------------------------------------------------
CLOCK = "clock"
JOINT_STATE = "state/joint"
POSE = "state/pose"
TWIST = "state/twist"
STATUS = "state/status"
IMU = "sensor/imu"
CAMERA = "sensor/camera/{name}"
DEPTH = "sensor/depth/{name}"
CMD_ACTUATORS = "cmd/actuators"
CMD_JOINT = "cmd/joint"
CMD_VELOCITY = "cmd/velocity"
CMD_SETPOINT = "cmd/setpoint"
CMD_VEHICLE = "cmd/vehicle"
CMD_INSTRUCTION = "cmd/instruction"


@dataclass
class Clock:
    """Simulation (or wall) time in seconds."""
    signal: ClassVar[str] = CLOCK
    t: float = 0.0


@dataclass
class JointState:
    signal: ClassVar[str] = JOINT_STATE
    names: list = field(default_factory=list)
    position: np.ndarray = field(default_factory=lambda: _v(0))
    velocity: np.ndarray = field(default_factory=lambda: _v(0))
    effort: np.ndarray = field(default_factory=lambda: _v(0))
    t: float = 0.0


@dataclass
class Pose:
    """Pose of the robot base in the world frame."""
    signal: ClassVar[str] = POSE
    position: np.ndarray = field(default_factory=_v)
    orientation: np.ndarray = field(default_factory=_q)
    t: float = 0.0


@dataclass
class Twist:
    """Velocity. As a state: linear in the world frame, angular in the body
    frame. As a command (`cmd/velocity`): both in the body frame."""
    signal: ClassVar[str] = TWIST
    linear: np.ndarray = field(default_factory=_v)
    angular: np.ndarray = field(default_factory=_v)
    t: float = 0.0


@dataclass
class Imu:
    """Specific force (accelerometer) and angular rate, in the body frame."""
    signal: ClassVar[str] = IMU
    accel: np.ndarray = field(default_factory=_v)
    gyro: np.ndarray = field(default_factory=_v)
    orientation: np.ndarray = field(default_factory=_q)
    t: float = 0.0


@dataclass
class Image:
    """A camera frame. `data` is HxWx3 uint8 (rgb) or HxW float32 metres (depth).

    `pose` is the camera-to-world transform (4x4) in the OpenGL convention
    (camera looks along -z, +y up), which is also what nerfstudio expects.
    """
    signal: ClassVar[str] = CAMERA
    data: np.ndarray = field(default_factory=lambda: np.zeros((0, 0, 3), np.uint8))
    camera: str = ""
    encoding: str = "rgb8"
    fovy: float = 45.0
    pose: np.ndarray = field(default_factory=lambda: np.eye(4))
    t: float = 0.0
    extras: dict = field(default_factory=dict)

    @property
    def intrinsics(self) -> dict:
        h, w = self.data.shape[:2]
        f = 0.5 * h / np.tan(0.5 * np.radians(self.fovy))
        return {"w": int(w), "h": int(h), "fl_x": float(f), "fl_y": float(f), "cx": w / 2.0, "cy": h / 2.0}


@dataclass
class ActuatorCommand:
    """Raw actuator inputs, in the order the physics backend lists them."""
    signal: ClassVar[str] = CMD_ACTUATORS
    values: np.ndarray = field(default_factory=lambda: _v(0))


@dataclass
class JointCommand:
    """Joint targets. Leave a field empty to not command it."""
    signal: ClassVar[str] = CMD_JOINT
    position: np.ndarray = field(default_factory=lambda: _v(0))
    velocity: np.ndarray = field(default_factory=lambda: _v(0))
    effort: np.ndarray = field(default_factory=lambda: _v(0))
    names: list = field(default_factory=list)


@dataclass
class Setpoint:
    """Where a vehicle should be: world-frame position and yaw, optionally
    with a feed-forward velocity. `position=None` means velocity control."""
    signal: ClassVar[str] = CMD_SETPOINT
    position: Optional[np.ndarray] = None
    velocity: Optional[np.ndarray] = None
    yaw: float = 0.0


@dataclass
class VehicleStatus:
    signal: ClassVar[str] = STATUS
    armed: bool = False
    mode: str = "HOLD"
    landed: bool = True


@dataclass
class VehicleCommand:
    """A high-level request: arm, disarm, takeoff, land."""
    signal: ClassVar[str] = CMD_VEHICLE
    name: str = ""
    params: dict = field(default_factory=dict)


@dataclass
class Instruction:
    """A natural-language goal for policies that take one."""
    signal: ClassVar[str] = CMD_INSTRUCTION
    text: str = ""


@dataclass
class VelocityCommand(Twist):
    """A body-frame velocity command. A `Twist` that goes to `cmd/velocity`
    (not `state/twist`) when a controller returns it."""
    signal: ClassVar[str] = CMD_VELOCITY


def velocity_command(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0)) -> VelocityCommand:
    """A body-frame velocity command for `cmd/velocity`."""
    return VelocityCommand(linear=np.asarray(linear, float), angular=np.asarray(angular, float))
