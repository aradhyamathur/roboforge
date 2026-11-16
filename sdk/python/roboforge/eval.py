"""Score an estimate against ground truth: the numbers SLAM and state
estimation papers report, and the file format their tools read.

    t, gt = dataset.trajectory("state/pose")          # ground truth from the simulator
    _, est = dataset.trajectory("slam/pose")          # your estimate, recorded alongside
    rf.eval.ate(est, gt)                              # absolute trajectory error, metres (RMSE)
    rf.eval.rpe(est, gt, delta=10)                    # relative pose error over 10 samples
    rf.eval.write_tum("est.txt", t, est)              # for evo, the TUM tools, ORB-SLAM3's scripts

Positions are `(N, 3)`; full poses are `(N, 7)` rows of position then
quaternion `(w, x, y, z)`, which is how `Dataset.trajectory` returns them.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np


def _xyz(a) -> np.ndarray:
    a = np.asarray(a, float)
    if a.ndim != 2 or a.shape[1] not in (3, 7):
        raise ValueError(f"expected (N, 3) positions or (N, 7) poses, got shape {a.shape}")
    return a[:, :3]


def align(est, gt, scale: bool = False):
    """The rotation, translation (and scale) that best map `est` onto `gt`
    (Umeyama's method). Returns `(R, t, s)`; apply as `s * R @ p + t`.
    Use it for estimators with their own origin (visual odometry, monocular
    SLAM), and `scale=True` when the scale is unknown too."""
    P, Q = _xyz(est), _xyz(gt)
    if len(P) != len(Q) or len(P) < 3:
        raise ValueError(f"need two trajectories of equal length >= 3, got {len(P)} and {len(Q)}")
    mp, mq = P.mean(0), Q.mean(0)
    X, Y = P - mp, Q - mq
    U, D, Vt = np.linalg.svd(Y.T @ X / len(P))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = float(np.trace(np.diag(D) @ S) / X.var(0).sum()) if scale else 1.0
    return R, mq - s * R @ mp, s


def ate(est, gt, align_first: bool = False, scale: bool = False) -> float:
    """Absolute trajectory error: RMS distance between matching positions, in metres."""
    P, Q = _xyz(est), _xyz(gt)
    if len(P) != len(Q):
        raise ValueError(f"trajectories differ in length ({len(P)} and {len(Q)}); use associate() first")
    if align_first:
        R, t, s = align(P, Q, scale)
        P = (s * (R @ P.T)).T + t
    return float(np.sqrt(np.mean(np.sum((P - Q) ** 2, axis=1))))


def rpe(est, gt, delta: int = 1) -> float:
    """Relative position error: RMS error of the motion over `delta` samples
    (drift per segment), independent of where each trajectory starts."""
    P, Q = _xyz(est), _xyz(gt)
    if len(P) != len(Q) or len(P) <= delta:
        raise ValueError(f"need two trajectories of equal length > delta ({len(P)}, {len(Q)}, delta {delta})")
    dP, dQ = P[delta:] - P[:-delta], Q[delta:] - Q[:-delta]
    return float(np.sqrt(np.mean(np.sum((dP - dQ) ** 2, axis=1))))


def associate(t_est, t_gt, max_dt: float = 0.02):
    """Index pairs `(i, j)` matching each estimate time to the nearest ground-truth
    time within `max_dt` seconds: for estimates published at their own rate."""
    t_est, t_gt = np.asarray(t_est, float), np.asarray(t_gt, float)
    j = np.clip(np.searchsorted(t_gt, t_est), 1, len(t_gt) - 1)
    j = np.where(np.abs(t_gt[j - 1] - t_est) < np.abs(t_gt[j] - t_est), j - 1, j)
    ok = np.abs(t_gt[j] - t_est) <= max_dt
    return np.nonzero(ok)[0], j[ok]


def write_tum(path, t, poses) -> Path:
    """Write a trajectory in TUM format: `t x y z qx qy qz qw` per line.
    `poses` is `(N, 7)` (position, quaternion w x y z) or `(N, 3)` (identity rotation)."""
    t, poses = np.asarray(t, float).reshape(-1), np.asarray(poses, float)
    if poses.ndim != 2 or poses.shape[1] not in (3, 7) or len(poses) != len(t):
        raise ValueError(f"expected N times and (N, 3) or (N, 7) poses, got {len(t)} and {poses.shape}")
    if poses.shape[1] == 3:
        poses = np.hstack([poses, np.tile([1.0, 0, 0, 0], (len(poses), 1))])
    lines = ["# timestamp tx ty tz qx qy qz qw"] + [
        f"{ti:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {p[4]:.6f} {p[5]:.6f} {p[6]:.6f} {p[3]:.6f}" for ti, p in zip(t, poses)]
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")
    return path


def read_tum(path):
    """`(t, poses)` from a TUM trajectory file, poses as `(N, 7)` with quaternion `(w, x, y, z)`."""
    rows = np.loadtxt(path, comments="#", ndmin=2)
    if rows.shape[1] != 8:
        raise ValueError(f"{path}: a TUM trajectory has 8 columns, found {rows.shape[1]}")
    return rows[:, 0], np.hstack([rows[:, 1:4], rows[:, 7:8], rows[:, 4:7]])
