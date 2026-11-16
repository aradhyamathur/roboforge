"""Run one app on two solvers and compare: the numpy two-link arm in this
folder, and MuJoCo with the same arm from custom_urdf/twolink.urdf.

Only the physics line differs. The controllers, the joint names and the
targets are the same, which is the point of the Physics interface. The two
solvers integrate differently (RK4 here, Euler with implicit damping in
MuJoCo), so they agree closely but not to the last digit.

    python3 compare.py
"""
import pathlib
import time

import numpy as np

import roboforge as rf

HERE = pathlib.Path(__file__).resolve().parent
URDF = HERE.parent / "custom_urdf" / "twolink.urdf"


def run(physics=None, robot=None, seconds=5.0):
    rec = []
    log = rf.controller(lambda obs, t: rec.append(obs.joint.position.copy()), rate=100, name="log")
    app = rf.App.from_config(HERE / "app.yaml", robot=robot, **({"physics": physics} if physics else {}), realtime=False)
    app.controllers.append(log)
    t0 = time.perf_counter()
    app.run(duration=seconds)
    wall = time.perf_counter() - t0
    app.close()
    return np.array(rec), seconds / wall


def main():
    ours, speed_ours = run()
    mj, speed_mj = run(physics={"type": "mujoco"}, robot={"urdf": str(URDF), "profile": "arm"})
    n = min(len(ours), len(mj))
    diff = np.abs(ours[:n] - mj[:n])
    print(f"{'solver':<16} {'speed':>16}   final joints (rad)")
    print(f"{'numpy RK4':<16} {'x%.0f real time' % speed_ours:>16}   {np.round(ours[n - 1], 3)}")
    print(f"{'MuJoCo':<16} {'x%.0f real time' % speed_mj:>16}   {np.round(mj[n - 1], 3)}")
    print(f"\nover {n} samples: largest difference {diff.max():.4f} rad, mean {diff.mean():.4f} rad")
    print("agree" if diff.max() < 0.03 else "DISAGREE: check the model parameters against the URDF")


if __name__ == "__main__":
    main()
