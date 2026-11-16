"""From a PID loop to a slow learned policy, in one app.

Three controllers share the arm, each at its own rate, talking through the bus:

  SlowPolicy   2 Hz    stands in for a large model (a VLA): slow, and it returns a
                       chunk of future joint targets from each inference
  rf.Async     50 Hz   runs that model off the loop, plays the chunk out, and falls
                       back to a safe pose if the model is late
  rf.JointPD   500 Hz  the PID loop that actually drives the motors

Swap SlowPolicy for a real model and nothing else changes.

    python3 pid_to_policy.py [seconds] [--headless]
"""
import sys
import time

import numpy as np

import roboforge as rf


class SlowPolicy(rf.Controller):
    rate = 2

    def step(self, obs, t):
        time.sleep(0.3)                                    # "inference"
        base = 0.6 * np.sin(0.5 * t)
        return rf.Chunk(actions=[rf.JointCommand(position=np.array([base + 0.02 * i, 0.5, -0.8])) for i in range(5)], dt=0.1)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    seconds = float(args[0]) if args else 20.0
    policy = rf.Async(SlowPolicy(), rate=50, timeout=1.0, fallback=rf.JointCommand(position=np.zeros(3)))
    pd = rf.JointPD(rate=500)
    callbacks = [rf.RateMonitor(every=2.0)] + ([] if "--headless" in sys.argv else [rf.Viewer()])
    with rf.App(robot="arm", controllers=[policy, pd], callbacks=callbacks) as app:
        app.run(duration=seconds)
        print(f"policy: {policy.inferences} inferences, {policy.latency * 1000:.0f} ms each, {policy.late} late steps; "
              f"PID loop ran at {app.controller_rate(pd):.0f} Hz throughout")


if __name__ == "__main__":
    main()
