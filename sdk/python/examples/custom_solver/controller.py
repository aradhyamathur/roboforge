"""Joint targets for the two-link arm: the same motion on any solver."""
import numpy as np
import roboforge as rf


class Swing(rf.Controller):
    def step(self, obs, t):
        return rf.JointCommand(names=["shoulder", "elbow"], position=np.array([0.4 * np.sin(1.5 * t), 0.6 + 0.3 * np.sin(2.5 * t)]))
