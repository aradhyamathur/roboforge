"""A first app: a robot, a controller you wrote, and a look at what happened.

    python3 first_app.py
    mjpython first_app.py --viewer --duration 30  # interactive window on macOS
"""
import argparse

import numpy as np

import roboforge as rf

TARGET = np.array([0.5, 0.4, -0.8])           # joint angles in radians: yaw, shoulder, elbow


# A controller is anything with step(obs, t). A plain function is enough.
@rf.controller(rate=500)
def hold_pose(obs, t):
    q, v = obs.joint.position, obs.joint.velocity
    torque = 40.0 * (TARGET - q) - 4.0 * v     # a PD loop: pull towards the target, damp the motion
    return rf.ActuatorCommand(values=torque)   # one value per motor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="open the interactive MuJoCo viewer")
    parser.add_argument("--duration", type=float, default=3.0, help="seconds of simulated time (default: 3)")
    args = parser.parse_args()
    app = rf.App(robot="arm", controllers=[hold_pose], realtime=args.viewer,
                 callbacks=[rf.Viewer()] if args.viewer else [])
    with app:
        app.run(duration=args.duration)

        q = app.bus.latest("state/joint").position
        print("joint names: ", app.physics.joint_names)
        print("target:      ", TARGET)
        print("reached:     ", np.round(q, 3))
        print("steps:       ", app.steps, f"({app.physics.dt * 1000:.0f} ms each)")
        print("signals:     ", ", ".join(app.bus.names()))


if __name__ == "__main__":
    main()
