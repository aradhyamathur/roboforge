"""Controllers for the LeRobot tutorial.

Expert  the demonstrator: every `period` seconds it picks a random goal pose,
        publishes it on task/goal, and moves the arm there along a smooth
        (minimum-jerk) path by publishing joint targets on cmd/joint.
Goals   for deployment: publishes goals from a list on task/goal, one every
        `period` seconds, and leaves the moving to the trained policy.
"""
import numpy as np
import roboforge as rf

LOW, HIGH = np.array([-1.0, -0.2, -1.4]), np.array([1.0, 0.9, -0.4])


class Expert(rf.Controller):
    def __init__(self, rate=20, period=2.0, move_time=1.2, seed=0, goal=None):
        super().__init__(rate=rate)
        self.period, self.move_time = period, move_time
        self.rng = np.random.default_rng(seed)
        self.fixed = None if goal is None else np.asarray(goal, float)     # always this goal (for evaluation)
        self.goal = None

    def step(self, obs, t):
        if obs.joint is None:
            return None
        if self.goal is None or t + 1e-6 >= self.t0 + self.period:
            goal = self.fixed if self.fixed is not None else self.rng.uniform(LOW, HIGH)
            self.goal, self.t0, self.q0 = goal, t, obs.joint.position.copy()
        s = min((t - self.t0) / self.move_time, 1.0)
        blend = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        return {"task/goal": self.goal, "cmd/joint": rf.JointCommand(position=self.q0 + blend * (self.goal - self.q0))}


class Goals(rf.Controller):
    def __init__(self, rate=20, period=2.5, goals=((0.6, 0.5, -0.9), (-0.7, 0.2, -0.6), (0.2, 0.8, -1.2), (-0.3, 0.0, -0.5))):
        super().__init__(rate=rate)
        self.period, self.goals = period, [np.asarray(g, float) for g in goals]

    def step(self, obs, t):
        return {"task/goal": self.goals[int(t / self.period + 1e-6) % len(self.goals)]}
