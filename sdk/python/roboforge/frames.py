"""Rotation helpers and the ENU/FLU <-> NED/FRD conversions links need."""
from __future__ import annotations

import numpy as np

# ENU -> NED for world vectors, and FLU -> FRD for body vectors.
_T_WORLD = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_T_BODY = np.diag([1.0, -1.0, -1.0])


def quat_to_mat(q) -> np.ndarray:
    w, x, y, z = np.asarray(q, float) / (np.linalg.norm(q) or 1.0)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(m) -> np.ndarray:
    m = np.asarray(m, float)
    tr = np.trace(m)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q if q[0] >= 0 else -q


def mat_to_euler(m) -> np.ndarray:
    """Roll, pitch, yaw (ZYX) of a rotation matrix."""
    m = np.asarray(m, float)
    pitch = -np.arcsin(np.clip(m[2, 0], -1.0, 1.0))
    return np.array([np.arctan2(m[2, 1], m[2, 2]), pitch, np.arctan2(m[1, 0], m[0, 0])])


def quat_to_euler(q) -> np.ndarray:
    return mat_to_euler(quat_to_mat(q))


def euler_to_mat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def yaw_of(q) -> float:
    return float(quat_to_euler(q)[2])


def enu_to_ned(v) -> np.ndarray:
    """World vector ENU -> NED (also its own inverse)."""
    return _T_WORLD @ np.asarray(v, float)


ned_to_enu = enu_to_ned


def flu_to_frd(v) -> np.ndarray:
    """Body vector FLU -> FRD (also its own inverse)."""
    return _T_BODY @ np.asarray(v, float)


frd_to_flu = flu_to_frd


def attitude_enu_to_ned(q_enu_flu) -> np.ndarray:
    """Rotation matrix body(FRD)->world(NED) from a body(FLU)->world(ENU) quaternion."""
    return _T_WORLD @ quat_to_mat(q_enu_flu) @ _T_BODY


def attitude_ned_to_enu(r_ned_frd) -> np.ndarray:
    """Quaternion body(FLU)->world(ENU) from a body(FRD)->world(NED) rotation matrix."""
    return mat_to_quat(_T_WORLD @ np.asarray(r_ned_frd, float) @ _T_BODY)
