"""Is this machine fast enough? Runs each bundled robot briefly and reports
how many times faster than real time it goes, with and without a camera, and
which optional parts are available.

    python3 health_check.py
"""
import time

import roboforge as rf
from roboforge.cli import doctor

CONTROLLERS = {
    "quadrotor": lambda: [rf.MultirotorController(rate=250, hover=[0, 0, 1.5])],
    "arm": lambda: [rf.JointPD(rate=500, target=[0.5, 0.4, -0.8])],
    "diffdrive": lambda: [rf.controller(lambda o, t: rf.velocity_command((0.3, 0, 0), (0, 0, 0.5)), rate=10), rf.DiffDriveController()],
}


def speed(robot, renderer=None, seconds=5.0):
    app = rf.App(robot=robot, renderer=renderer, controllers=CONTROLLERS[robot](), realtime=False)
    app.setup()
    t0 = time.perf_counter()
    app.run(duration=seconds)
    wall = time.perf_counter() - t0
    app.close()
    return seconds / wall


def main():
    rep = doctor()
    print(f"roboforge {rep['version']}, Python {rep['python']}, display: {'yes' if rep['display'] else 'no'}")
    for p in rep["parts"]:
        print(f"  {'ok     ' if p['ok'] else 'MISSING'} {p['name']:<8} {p['what']}" + ("" if p["ok"] else f"   fix: {p['fix']}"))

    print(f"\n{'robot':<10} {'physics only':>14} {'+ 320x240 camera at 30 fps':>28}")
    slowest = float("inf")
    for robot in CONTROLLERS:
        plain = speed(robot)
        try:
            cam = speed(robot, rf.MujocoRenderer(width=320, height=240, fps=30))
            cam_text = f"x{cam:.0f} real time"
        except Exception as e:                              # no OpenGL here: physics still works
            cam, cam_text = plain, f"no rendering ({type(e).__name__})"
        slowest = min(slowest, plain, cam)
        print(f"{robot:<10} {'x%.0f real time' % plain:>14} {cam_text:>28}")

    if slowest >= 2:
        print(f"\nverdict: fine. The slowest case runs at x{slowest:.0f}, so real-time apps have room to spare.")
    elif slowest >= 1:
        print(f"\nverdict: just enough (x{slowest:.1f}). Keep cameras small and close other programs.")
    else:
        print(f"\nverdict: too slow for real time (x{slowest:.1f}). Use --no-realtime, or lower the camera size and rate.")


if __name__ == "__main__":
    main()
