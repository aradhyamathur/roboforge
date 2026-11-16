"""Train with a standard library: PPO from stable-baselines3, on PyTorch.

The environment comes from `rf.Policy.make_env`, so it is an ordinary
Gymnasium environment and stable-baselines3 takes it as is. The trained
PyTorch network is then deployed through the same `rf.Policy` interface as
the numpy one in custom_policy.py: `act` returns a torch tensor and the SDK
converts it.

Needs: pip install torch stable-baselines3   (the CPU build is enough)

    python3 sb3_ppo.py            # about a minute on a CPU
"""
import time

import numpy as np
import torch
from stable_baselines3 import PPO

import roboforge as rf

LOW, HIGH = np.array([-1.0, -0.2, -1.4]), np.array([1.0, 0.9, -0.4])


class SB3Policy(rf.Policy):
    """A policy whose network is a stable-baselines3 model (a PyTorch module)."""
    output = "joint"

    def __init__(self, model=None, rate=20):
        super().__init__(rate=rate)
        self.model = model

    def observe(self, obs):
        q, qd, goal = obs.joint.position, obs.joint.velocity, obs["task/goal"]
        return np.concatenate([q, 0.2 * qd, goal - q])

    def act(self, x):
        with torch.no_grad():
            action, _, _ = self.model.policy(torch.as_tensor(x)[None], deterministic=True)
        return action[0]                                  # a torch tensor: the SDK converts it

    def decode(self, a, obs):
        # The network works in [-1, 1] (what PPO expects); this makes it a step from where the
        # joints are. It runs in training and on the robot, so the two cannot disagree.
        return obs.joint.position + 0.4 * np.clip(a, -1, 1)


def make_env(policy):
    def reward(obs):
        return -float(np.abs(obs["task/goal"] - obs.joint.position).sum())
    app = rf.App(robot="arm", controllers=[rf.JointPD(rate=500)])
    return policy.make_env(app, reward=reward, max_steps=20,
                           on_reset=lambda rng: {"task/goal": rng.uniform(LOW, HIGH)})     # a new goal every episode


def evaluate(policy, goals):
    app = rf.App(robot="arm", controllers=[policy, rf.JointPD(rate=500)], realtime=False)
    app.setup()
    errors = []
    for g in goals:
        app.bus.publish("task/goal", g)
        app.run(duration=1.5)
        errors.append(float(np.abs(app.bus.latest("state/joint").position - g).max()))
    app.close()
    return errors


def main(steps=24_000):
    torch.manual_seed(0)
    policy = SB3Policy()
    env = make_env(policy)
    model = PPO("MlpPolicy", env, n_steps=1024, batch_size=128, learning_rate=1e-3, gamma=0.95, seed=0, verbose=0)
    goals = np.random.default_rng(7).uniform(LOW, HIGH, (5, 3))
    policy.model = model
    before = evaluate(policy, goals)
    t0 = time.time()
    model.learn(total_timesteps=steps)
    print(f"PPO: {steps} steps in {time.time() - t0:.0f}s")
    after = evaluate(policy, goals)
    print(f"worst joint error on 5 new goals: {max(before):.2f} rad untrained, {max(after):.2f} rad trained")
    model.save("ppo_reach")
    print("deployed through rf.Policy: act() returned a torch tensor, the SDK made the JointCommand")


if __name__ == "__main__":
    main()
