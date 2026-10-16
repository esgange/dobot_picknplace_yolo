"""Strict calibration poses: metres and fixed-axis XYZ angles in degrees.

Rotation is Rz(rz) @ Ry(ry) @ Rx(rx), matching CATARM's readable format.
The format changes disk representation only; runtime transforms remain matrices.
"""

import math

import numpy as np


FORMAT = "xyz_rxryrz_m_deg_v1"
AXES = ("x", "y", "z", "rx", "ry", "rz")
POSE_FIELDS = frozenset(("format", *AXES))


def pose_from_matrix(matrix, label: str) -> dict:
    transform = np.asarray(matrix, dtype=np.float64)
    if (transform.shape != (4, 4) or not np.all(np.isfinite(transform))
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-12, rtol=0)):
        raise ValueError(f"{label} must be a finite homogeneous transform")
    rotation = transform[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0)
            or abs(float(np.linalg.det(rotation)) - 1.0) > 1e-5):
        raise ValueError(f"{label} must contain a proper rotation")
    cy = math.hypot(rotation[0, 0], rotation[1, 0])
    if cy > 1e-12:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        # At gimbal lock choose the equivalent, deterministic yaw=0 solution.
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    pitch = math.atan2(-rotation[2, 0], cy)
    return {
        "format": FORMAT,
        **dict(zip(AXES[:3], map(float, transform[:3, 3]))),
        **dict(zip(AXES[3:], map(math.degrees, (roll, pitch, yaw)))),
    }


def matrix_from_pose(value, label: str) -> np.ndarray:
    if not isinstance(value, dict) or set(value) != POSE_FIELDS:
        raise ValueError(f"{label} must contain exactly format, x, y, z, rx, ry, rz")
    if value["format"] != FORMAT:
        raise ValueError(f"{label}.format must be {FORMAT}")
    for key in AXES:
        try:
            if type(value[key]) not in (int, float) or not math.isfinite(value[key]):
                raise ValueError()
        except (ValueError, OverflowError) as exc:
            raise ValueError(f"{label}.{key} must be a finite number") from exc
    rx, ry, rz = (math.radians(math.fmod(value[k], 360.0)) for k in AXES[3:])
    sx, sy, sz = map(math.sin, (rx, ry, rz))
    cx, cy, cz = map(math.cos, (rx, ry, rz))
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = [
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ]
    transform[:3, 3] = [value[k] for k in AXES[:3]]
    return transform
