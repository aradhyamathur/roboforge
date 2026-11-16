"""Reach a pose with the arm. Used by the "Debug a broken app" tutorial.

The targets were written in degrees by someone thinking in degrees; the SDK
(like ROS and MuJoCo) works in radians. `degrees: false` in app_fixed.yaml
converts them.
"""
import numpy as np
import roboforge as rf

POSE_DEGREES = [40.0, 35.0, -60.0]        # yaw, shoulder, elbow


class Reach(rf.Controller):
    def __init__(self, rate=None, degrees=True):
        super().__init__(rate=rate)
        self.target = np.array(POSE_DEGREES) if degrees else np.radians(POSE_DEGREES)

    def step(self, obs, t):
        return rf.JointCommand(position=self.target)
