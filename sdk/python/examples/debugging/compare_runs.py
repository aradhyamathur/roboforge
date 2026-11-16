"""Record two runs and compare them: the way to answer "did my change help?"

Records the arm following the same motion with two sets of gains, then loads
both recordings and prints the tracking error of each. Also checks that a run
repeated with the same settings gives exactly the same numbers, which is what
makes comparisons like this trustworthy.

    python3 compare_runs.py
"""
import numpy as np

import roboforge as rf


@rf.controller(rate=50)
def motion(obs, t):
    return rf.JointCommand(position=np.array([0.8 * np.sin(0.8 * t), 0.5 + 0.3 * np.sin(1.3 * t), -0.9]))


def record(name, **gains):
    app = rf.App(robot="arm", controllers=[motion, rf.JointPD(rate=500, **gains)],
                 callbacks=[rf.Recorder(dir=f"runs/{name}", rate=100)], realtime=False)
    app.run(duration=10)
    app.close()
    return rf.Dataset(f"runs/{name}")


def tracking_error(ds):
    """RMS and worst distance between where the joints were and where they were told to be."""
    t = ds.times("state/joint")
    q = ds.signals()["state/joint"][:, :3]
    want = np.stack([0.8 * np.sin(0.8 * t), 0.5 + 0.3 * np.sin(1.3 * t), np.full_like(t, -0.9)], axis=1)
    err = np.abs(q - want)[t > 1.0]                        # skip the first second, while it catches up
    return float(np.sqrt((err ** 2).mean())), float(err.max())


def main():
    soft = record("soft", kp=15, ki=0, kd=2)
    stiff = record("stiff", kp=120, ki=40, kd=10)
    again = record("stiff_again", kp=120, ki=40, kd=10)

    print(f"{'run':<8} {'rms error':>10} {'worst':>8}   (radians)")
    for name, ds in (("soft", soft), ("stiff", stiff)):
        rms, worst = tracking_error(ds)
        print(f"{name:<8} {rms:10.4f} {worst:8.4f}")

    same = np.array_equal(stiff.signals()["state/joint"], again.signals()["state/joint"])
    print(f"\nthe stiff run repeated: {'identical, sample for sample' if same else 'DIFFERENT (something is not deterministic)'}")
    print("recordings are in runs/: signals.npz holds the arrays, dataset.yaml describes them")


if __name__ == "__main__":
    main()
