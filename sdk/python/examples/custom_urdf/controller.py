"""Joint targets for the two-link URDF arm. Edit freely."""
import numpy as np
import roboforge as rf


class Swing(rf.Controller):
    def step(self, obs, t):
        # Addressed by joint name, so the order in the URDF does not matter.
        return rf.JointCommand(names=["shoulder", "elbow"], position=np.array([0.6 * np.sin(1.2 * t), 1.0 + 0.4 * np.sin(2.0 * t)]))
