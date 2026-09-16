"""Controller pose conversions plus the shared read-only CR10 kinematics."""

import math

import numpy as np

from camera_calibration_gui.calibration_core import rotation_matrix_to_rpy_deg
from item_perception_yolo.pick_planning import Cr10Kinematics, rpy_matrix


def pose_matrix(values):
    if len(values) != 6 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError("Pose requires six finite mm/degree values")
    result = np.eye(4)
    result[:3, 3] = np.asarray(values[:3], dtype=float) / 1000
    result[:3, :3] = rpy_matrix(*np.deg2rad(values[3:]))
    return result


def pose_values(matrix):
    return [*map(float, matrix[:3, 3] * 1000),
            *rotation_matrix_to_rpy_deg(matrix[:3, :3])]
