"""Train something, small: the arm learns to reach a pose.

`app.as_env()` turns any app into a Gymnasium environment. Here a linear
policy (9 numbers) is found by random search, which needs no deep-learning
library, no GPU, and about ten seconds. Swap the search for PPO from
stable-baselines3 or your own trainer: the environment is the same.

    python3 gym_reach.py
"""
import numpy as np

import roboforge as rf
from roboforge.env import box

GOAL = np.array([0.6, 0.5, -0.9])


def make_env():
    def reward(obs):
        q, v = obs.joint.position, obs.joint.velocity
        return -float(np.abs(GOAL - q).sum()) - 0.02 * float(np.abs(v).sum())
    # The action is a joint target; the built-in controller underneath turns it into torques.
    # The action space is the joints' range of motion, since actions are joint targets.
    app = rf.App(robot="arm", controllers=[rf.JointPD(rate=500)])
    app.setup()
    return app.as_env(act=lambda a: rf.JointCommand(position=np.asarray(a, float)), reward=reward,
                      action_space=box(app.physics.joint_range),
                      steps_per_action=25, max_steps=40)            # 20 actions per second, 2 s episodes


def episode(env, W):
    """Total reward of one episode under the linear policy `target = W @ [q, 1]`."""
    obs, _ = env.reset()
    total, done = 0.0, False
    while not done:
        q = obs[:3]
        obs, r, terminated, truncated, _ = env.step(W @ np.append(q, 1.0))
        total += r
        done = terminated or truncated
    return total, obs[:3]


def main():
    env = make_env()
    print(f"observation {env.observation_space.shape}, action {env.action_space.shape}")
    rng = np.random.default_rng(0)
    W = np.zeros((3, 4))                                    # start: always command zero
    best, _ = episode(env, W)
    print(f"start      reward {best:8.2f}")
    for it in range(1, 151):
        trial = W + rng.normal(0, 0.15, W.shape)
        score, _ = episode(env, trial)
        if score > best:
            W, best = trial, score
        if it % 30 == 0:
            print(f"episode {it:<3} reward {best:8.2f}")
    _, q = episode(env, W)
    print(f"\nreached {np.round(q, 2)} for goal {GOAL}: worst joint error {np.abs(GOAL - q).max():.2f} rad")
    env.close()


if __name__ == "__main__":
    main()
