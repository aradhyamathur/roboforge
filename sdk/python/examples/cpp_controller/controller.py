"""Joint targets, in Python: the high-level half of a mixed Python / C++ app."""
import numpy as np
import roboforge as rf


class Wave(rf.Controller):
    def step(self, obs, t):
        return rf.JointCommand(position=np.array([0.8 * np.sin(0.6 * t), 0.5 + 0.3 * np.sin(1.1 * t), -0.9]))
