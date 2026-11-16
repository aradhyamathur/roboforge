"""Step 4: score the trained policy on goals it never saw, against the expert.

    python3 evaluate.py [policies/arm_act]
"""
import pathlib
import sys

import numpy as np

import roboforge as rf

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from controller import HIGH, LOW  # noqa: E402


def reach(goal, policy_path=None, seconds=2.0):
    """Start from the home pose, publish one goal, return the worst joint error after `seconds`."""
    if policy_path:
        first = rf.CONTROLLERS.build({"type": "lerobot", "path": str(policy_path)})
    else:                                   # the expert itself, as the reference
        from controller import Expert
        first = Expert(period=1e9, goal=goal)
    app = rf.App(robot="arm", controllers=[first, rf.JointPD(rate=500)], realtime=False)
    app.setup()
    app.bus.publish("task/goal", goal)
    app.run(duration=seconds)
    err = float(np.abs(app.bus.latest("state/joint").position - goal).max())
    app.close()
    return err


def main():
    policy = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "policies/arm_act")
    goals = np.random.default_rng(123).uniform(LOW, HIGH, (8, 3))
    learned = [reach(g, policy) for g in goals]
    expert = [reach(g) for g in goals]
    print(f"{'goal (rad)':<24} {'policy':>8} {'expert':>8}")
    for g, a, b in zip(goals, learned, expert):
        print(f"{np.round(g, 2)!s:<24} {a:8.3f} {b:8.3f}")
    print(f"\nworst joint error over {len(goals)} new goals: policy {max(learned):.3f} rad, expert {max(expert):.3f} rad")


if __name__ == "__main__":
    main()
