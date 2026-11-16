"""Step an app by hand and look inside it. This is how to debug a controller.

`app.step()` advances exactly one physics step, so you own the loop: print
what you like, stop on a condition, drop into the debugger. With
`realtime=False` nothing waits for the wall clock, and a paused app (a
breakpoint) does not try to catch up afterwards.

    python3 step_through.py            # prints a trace and stops where the error is largest
    python3 step_through.py --pdb      # ...and opens the debugger there
"""
import sys

import numpy as np

import roboforge as rf

TARGET = np.array([0.6, 0.5, -0.9])


def main():
    pd = rf.JointPD(kp=60, ki=20, kd=6, rate=500, target=TARGET)
    app = rf.App(robot="arm", controllers=[pd], realtime=False)
    app.setup()                                           # everything is built; nothing has moved yet

    model, data = app.physics.raw                         # MuJoCo's own objects, for anything the bus does not carry
    print(f"model: {model.nq} positions, {model.nu} actuators ({', '.join(app.physics.actuator_names)}), dt {app.physics.dt * 1000:.0f} ms\n")
    print(f"{'t':>6}  {'joint positions':<24} {'error':>7}  {'torques':<24}")

    start = app.bus.latest("state/joint").position.copy()
    direction = np.sign(TARGET - start)                   # which way each joint has to travel
    worst = (0.0, 0.0)
    while app.time < 2.0:
        app.step()
        q = app.bus.latest("state/joint").position
        u = app.bus.latest("cmd/actuators").values
        err = float(np.abs(TARGET - q).max())
        if app.steps % 100 == 0:
            print(f"{app.time:6.2f}  {np.round(q, 3)!s:<24} {err:7.3f}  {np.round(u, 2)!s:<24}")
        overshoot = float(((q - TARGET) * direction).max())   # how far past its target any joint has gone
        if overshoot > worst[0]:
            worst = (overshoot, app.time)

    print(f"\nlargest overshoot: {worst[0]:.3f} rad at t={worst[1]:.2f}s")
    print(f"settled to within {np.abs(TARGET - app.bus.latest('state/joint').position).max():.4f} rad after {app.time:.1f}s")

    # Go back and stop exactly there: runs are deterministic, so the same step reproduces the same state.
    app.reset()
    app.run(until=lambda a: a.time >= worst[1])
    print(f"\nreplayed to t={app.time:.3f}s: joints {np.round(app.bus.latest('state/joint').position, 3)}, "
          f"velocities {np.round(app.bus.latest('state/joint').velocity, 2)}")
    if "--pdb" in sys.argv:
        breakpoint()                                      # look at app, pd.pid, data.qfrc_actuator ... then `c` to continue
    app.close()


if __name__ == "__main__":
    main()
