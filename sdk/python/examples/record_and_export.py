"""Fly, record, and export a dataset for NeRF / Gaussian splatting and for SLAM.

The camera poses in the export are simulator ground truth, so there is no
COLMAP step: train directly, or use them to score a SLAM system.

    python3 record_and_export.py
    ns-train splatfacto --data runs/flight/nerfstudio        # if nerfstudio is installed
"""
import numpy as np

import roboforge as rf

WAYPOINTS = [(0, 0, 1.5), (3, 0, 1.5), (3, 3, 2.0), (0, 3, 1.5), (0, 0, 1.5)]


@rf.controller(rate=1)
def tour(obs, t):
    x, y, z = WAYPOINTS[min(int(t / 5.0), len(WAYPOINTS) - 1)]
    return rf.Setpoint(position=np.array([x, y, z], float), yaw=0.3 * t)


def main():
    app = rf.App(robot="quadrotor",
                 renderer=rf.MujocoRenderer(width=640, height=480, fps=10),
                 controllers=[tour, rf.MultirotorController(rate=250)],
                 callbacks=[rf.Recorder(dir="runs/flight", rate=50)],
                 realtime=False)
    app.run(duration=25)
    app.close()
    ds = rf.Dataset("runs/flight")
    print(f"recorded {len(ds.frames())} frames from camera {ds.cameras[0]!r}")
    print("nerfstudio:", ds.export_nerfstudio("runs/flight/nerfstudio"))
    print("TUM:       ", ds.export_tum("runs/flight/tum"))


if __name__ == "__main__":
    main()
