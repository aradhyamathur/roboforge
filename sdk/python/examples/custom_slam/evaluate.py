"""Run the SLAM app, then score it against the simulator's ground truth.

    python3 evaluate.py               # one minute of simulated driving, about five seconds

Writes runs/slam/{groundtruth,slam,odometry}.txt in TUM format, which the
usual tools read (for example `evo_ape tum groundtruth.txt slam.txt -p`).
"""
import pathlib

import numpy as np

import roboforge as rf

HERE = pathlib.Path(__file__).resolve().parent


def main(seconds=60.0):
    app = rf.App.from_config(HERE / "app.yaml")
    app.run(duration=seconds)
    found = app.bus.latest("slam/landmarks")
    app.close()

    ds = rf.Dataset("runs/slam")
    t_gt, gt = ds.trajectory("state/pose")
    print(f"{'estimate':<16} {'ATE (m)':>8} {'RPE over 1 s (m)':>18} {'final error (m)':>16}")
    for name in ("odom/pose", "slam/pose"):
        t, est = ds.trajectory(name)
        i, j = rf.eval.associate(t, t_gt)                   # match samples by time
        ate = rf.eval.ate(est[i], gt[j])
        rpe = rf.eval.rpe(est[i], gt[j], delta=20)          # 20 samples at 20 Hz: one second
        final = np.linalg.norm(est[i][-1, :2] - gt[j][-1, :2])
        label = "odometry only" if name == "odom/pose" else "EKF-SLAM"
        print(f"{label:<16} {ate:8.3f} {rpe:18.3f} {final:16.3f}")
        rf.eval.write_tum(pathlib.Path("runs/slam") / ("odometry.txt" if name == "odom/pose" else "slam.txt"), t[i], est[i])
    rf.eval.write_tum("runs/slam/groundtruth.txt", t_gt, gt)

    from sensors import WORLD
    err = [np.linalg.norm(p - WORLD[k]) for k, p in found.items()]
    print(f"\nmap: {len(found)} of {len(WORLD)} landmarks found, placed within {max(err):.3f} m (mean {np.mean(err):.3f} m)")
    print("trajectories in TUM format: runs/slam/groundtruth.txt, slam.txt, odometry.txt")


if __name__ == "__main__":
    import sys
    sys.path.insert(0, str(HERE))
    main()
