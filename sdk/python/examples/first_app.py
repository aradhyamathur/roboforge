"""A first app: a robot, a controller you wrote, and a look at what happened.

    python3 first_app.py
"""
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
    app = rf.App(robot="arm", controllers=[hold_pose], realtime=False)
    app.run(duration=3.0)                      # three seconds of simulated time

    q = app.bus.latest("state/joint").position
    print("joint names: ", app.physics.joint_names)
    print("target:      ", TARGET)
    print("reached:     ", np.round(q, 3))
    print("steps:       ", app.steps, f"({app.physics.dt * 1000:.0f} ms each)")
    print("signals:     ", ", ".join(app.bus.names()))
    app.close()


if __name__ == "__main__":
    main()
