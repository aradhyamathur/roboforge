"""A learned policy of your own: define it, train it, save it, deploy it.

The policy is a small neural network (two layers, in plain numpy) that
reaches any goal pose with the arm. It is trained by the cross-entropy
method, an evolution strategy, against an environment the policy makes for
itself; no deep-learning library and no GPU. Training takes under a minute.

What `rf.Policy` gives you:

- `observe(obs)` and `act(x)` are the two halves a trainer needs. Training
  and deployment call the same two, so the preprocessing cannot drift apart
- `output = "joint"` says what the action means; the joint controller
  underneath turns targets into torques, which is far easier to learn on
  than torques
- `make_env(app, reward=...)` is a Gymnasium environment with the right
  spaces; `reset(options={"publish": {...}})` sets a new goal per episode
- `act` may return numpy, a PyTorch tensor or a JAX array

Swap the numpy network for a PyTorch module and the cross-entropy method for
PPO and nothing else changes (see the tutorial).

    python3 custom_policy.py              # trains, saves policy.npz, then deploys it on new goals
    python3 custom_policy.py --deploy     # only the deployment, from policy.npz
"""
import sys
import time

import numpy as np

import roboforge as rf

LOW, HIGH = np.array([-1.2, -0.3, -1.6]), np.array([1.2, 1.0, -0.3])        # where goals are drawn from (rad)


class ReachPolicy(rf.Policy):
    """Sees joint positions, velocities and the goal; outputs joint targets."""
    output = "joint"

    def __init__(self, rate=20, hidden=16, step_size=0.4, params=None, seed=0):
        super().__init__(rate=rate)
        self.hidden, self.step_size = int(hidden), float(step_size)
        sizes = (6 * hidden, hidden, hidden * 3, 3)
        self.size = sum(sizes)
        self.params = np.random.default_rng(seed).normal(0, 0.1, self.size) if params is None else np.asarray(params, float)

    def observe(self, obs):
        return np.concatenate([obs.joint.position, obs.joint.velocity, obs["task/goal"]])

    def act(self, x, params=None):
        q, qd, goal = x[:3], x[3:6], x[6:9]
        h, p = self.hidden, self.params if params is None else params
        W1, b1 = p[:6 * h].reshape(h, 6), p[6 * h:7 * h]
        W2, b2 = p[7 * h:10 * h].reshape(3, h), p[10 * h:]
        features = np.concatenate([goal - q, 0.2 * qd])         # relative inputs generalise across the workspace
        return q + self.step_size * np.tanh(W2 @ np.tanh(W1 @ features + b1) + b2)

    def save(self, path):
        np.savez(path, params=self.params, hidden=self.hidden, step_size=self.step_size, rate=self.rate)

    @classmethod
    def load(cls, path):
        f = np.load(path)
        return cls(params=f["params"], hidden=int(f["hidden"]), step_size=float(f["step_size"]), rate=float(f["rate"]))


def reward(obs):
    return -float(np.abs(obs["task/goal"] - obs.joint.position).sum()) - 0.01 * float(np.abs(obs.joint.velocity).sum())


def episode_return(env, policy, params, goal):
    x, _ = env.reset(options={"publish": {"task/goal": goal}})     # a new goal for this episode
    total, done = 0.0, False
    while not done:
        x, r, terminated, truncated, _ = env.step(policy.act(x, params))
        total += r
        done = terminated or truncated
    return total


def train(policy, iterations=30, population=24, elite=5, goals_per_iteration=3, seed=0):
    """Cross-entropy method: sample weights around a mean, keep the best, move the mean there."""
    rng = np.random.default_rng(seed)
    app = rf.App(robot="arm", controllers=[rf.JointPD(rate=500)])
    env = policy.make_env(app, reward=reward, max_steps=14,       # 20 actions a second, 0.7 s episodes
                          on_reset=lambda rng: {"task/goal": rng.uniform(LOW, HIGH)})
    mean, std = policy.params.copy(), np.full(policy.size, 0.3)
    for it in range(1, iterations + 1):
        goals = rng.uniform(LOW, HIGH, (goals_per_iteration, 3))   # the same goals for every candidate this round
        candidates = mean + std * rng.standard_normal((population, policy.size))
        scores = np.array([np.mean([episode_return(env, policy, c, g) for g in goals]) for c in candidates])
        best = candidates[np.argsort(scores)[-elite:]]
        mean, std = best.mean(axis=0), best.std(axis=0) + 0.02
        if it == 1 or it % 5 == 0:
            print(f"iteration {it:>3}: best {scores.max():7.2f}  mean {scores.mean():7.2f}", flush=True)
    env.close()
    policy.params = mean
    return policy


def deploy(policy, goals):
    """The trained policy as an ordinary controller in an ordinary app, on goals it never saw."""
    checks = rf.Checks()
    app = rf.App(robot="arm", controllers=[policy, rf.JointPD(rate=500)], callbacks=[checks], realtime=False)
    app.setup()
    print(f"\n{'goal (rad)':<26} {'reached':<26} {'error':>6}")
    errors = []
    for goal in goals:
        app.bus.publish("task/goal", goal)                    # a planner, a UI or a language model would set this
        app.run(duration=1.5)
        q = app.bus.latest("state/joint").position
        errors.append(float(np.abs(goal - q).max()))
        print(f"{np.round(goal, 2)!s:<26} {np.round(q, 2)!s:<26} {errors[-1]:6.3f}")
    app.close()
    print(f"\nworst error {max(errors):.3f} rad over {len(goals)} new goals; checks: {checks.findings or 'nothing to report'}")
    return errors


def main():
    if "--deploy" not in sys.argv:
        policy = ReachPolicy()
        print(f"policy: {policy.size} parameters; training on random goals ...")
        t0 = time.time()
        train(policy)
        policy.save("policy.npz")
        print(f"trained in {time.time() - t0:.0f}s, saved to policy.npz")
    policy = ReachPolicy.load("policy.npz")
    deploy(policy, np.random.default_rng(42).uniform(LOW, HIGH, (5, 3)))


if __name__ == "__main__":
    main()
