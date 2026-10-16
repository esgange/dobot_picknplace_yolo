"""Disk representation preserves geometry, including singular Euler attitudes."""

import math

import numpy as np
import pytest

from camera_calibration_gui.calibration_format import (
    FORMAT, matrix_from_pose, pose_from_matrix,
)


def _pose(rx=23., ry=-41., rz=137.):
    return dict(format=FORMAT, x=.125, y=-.031, z=.744, rx=rx, ry=ry, rz=rz)


@pytest.mark.parametrize("angles", [
    (23., -41., 137.), (31., 90., -57.), (31., -90., -57.),
    (-171., 89.9999999, 176.), (100., -89.9999999, -83.),
    (180., 0., -180.), (721., -361., 1081.),
])
def test_readable_pose_preserves_full_transform(angles):
    # Independent reference: multiply the three elementary axis rotations.
    rx, ry, rz = map(math.radians, angles)
    sx, sy, sz = map(math.sin, (rx, ry, rz))
    cx, cy, cz = map(math.cos, (rx, ry, rz))
    mx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    my = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    mz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    expected = np.eye(4)
    expected[:3, :3] = mz @ my @ mx
    expected[:3, 3] = [.125, -.031, .744]
    np.testing.assert_allclose(matrix_from_pose(_pose(*angles), "test"), expected, atol=1e-12)
    saved = pose_from_matrix(expected, "test")
    assert set(saved) == {"format", "x", "y", "z", "rx", "ry", "rz"}
    np.testing.assert_allclose(matrix_from_pose(saved, "test"), expected, atol=1e-12)


@pytest.mark.parametrize("value", [None, True, "12", float("nan"), float("inf"), 10**1000])
@pytest.mark.parametrize("key", ["x", "rx"])
def test_non_numeric_or_non_finite_coordinates_are_rejected(key, value):
    pose = _pose()
    pose[key] = value
    with pytest.raises(ValueError, match="finite number"):
        matrix_from_pose(pose, "test")


@pytest.mark.parametrize("change", [
    {"format": "xyz_rxryrz_mm_deg_v1"},
    {"rotation_xyzw": dict(x=0., y=0., z=0., w=1.)},
    {"translation_m": dict(x=0., y=0., z=0.)},
    {"unit": "rad"},
])
def test_mixed_formats_and_implicit_units_are_rejected(change):
    with pytest.raises(ValueError):
        matrix_from_pose({**_pose(), **change}, "test")


@pytest.mark.parametrize("matrix", [np.zeros((4, 4)), np.diag([1, 1, -1, 1]),
                                       np.diag([1, 1, 2, 1]), np.eye(3)])
def test_writer_rejects_non_rigid_inputs(matrix):
    with pytest.raises(ValueError):
        pose_from_matrix(matrix, "test")
