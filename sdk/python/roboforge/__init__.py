"""RoboForge SDK: one small API for robot stacks.

    import roboforge as rf

    app = rf.App(robot="quadrotor",
                 controllers=[rf.MultirotorController(rate=250)],
                 links=[rf.MavlinkLink(mode="vehicle")])
    app.run(duration=10)

Simulators, renderers, ROS 2 and MAVLink sit behind small interfaces and
talk through a signal bus (`roboforge.bus`), so any of them combine. See
`roboforge.app`, `roboforge.control`, `roboforge.config`.
"""
from . import msgs
from .app import App
from .bus import Bus
from .callbacks import Checks, RateMonitor, Recorder, Tap, Viewer, summarize
from .component import Callback, Link, Physics, Renderer
from .control import PID, Async, Chunk, Controller, Obs, controller
from .data import Dataset
from .errors import ConfigError, SimulationError
from .msgs import (ActuatorCommand, Clock, Image, Imu, Instruction, JointCommand, JointState, Pose, Setpoint, Twist,
                   VehicleCommand, VehicleStatus, VelocityCommand, velocity_command)
from .registry import CALLBACKS, CONTROLLERS, LINKS, PHYSICS, RENDERERS
from .robot import Robot

__version__ = "0.1.0"

_LAZY = {
    "MujocoPhysics": "roboforge.physics.mujoco", "NoPhysics": "roboforge.physics.none",
    "MujocoRenderer": "roboforge.render.mujoco",
    "Ros2Link": "roboforge.links.ros2", "MavlinkLink": "roboforge.links.mavlink",
    "MultirotorController": "roboforge.control.multirotor", "JointPD": "roboforge.control.joint",
    "DiffDriveController": "roboforge.control.diffdrive",
}


def __getattr__(name):
    # Backends import their heavy dependency (mujoco, rclpy, pymavlink) only when first used.
    if name in _LAZY:
        import importlib
        return getattr(importlib.import_module(_LAZY[name]), name)
    raise AttributeError(f"module 'roboforge' has no attribute {name!r}")
