"""Plane height is defined in base Z, independently of local normal direction."""

import numpy as np
import pytest

from item_perception_yolo.pick_planning import rpy_matrix
from item_perception_yolo.surface_guard import HeightPlane, SurfaceGeometryError


@pytest.mark.parametrize("tilted", [False, True])
@pytest.mark.parametrize("flipped", [False, True])
@pytest.mark.parametrize("height,below", [(.12, False), (0., False), (-.001, True)])
def test_surface_height_uses_local_xy_and_either_normal(tilted, flipped, height, below):
    plane = np.eye(4)
    if tilted:
        plane[:3, :3] = rpy_matrix(.2, -.3, .4)
    plane[:3, 3] = [.1, -.2, .3]
    point = (plane @ [.4, .5, 0., 1.])[:3]
    expected_z = point[2]
    point[2] += height
    if flipped:
        plane = plane @ np.diag([1., -1., -1., 1.])
    before = point.copy()
    result = HeightPlane(plane, "bin floor").measure(point)
    assert result.plane_z_m == pytest.approx(expected_z)
    assert result.surface_z_m == before[2]
    assert result.below_plane is below
    assert np.array_equal(point, before)  # Never clamp or offset the measurement.
    if below:
        assert "below bin floor" in result.reason and "deficit=1.000 mm" in result.reason


@pytest.mark.parametrize("z,below", [(0., False), (-1e-6, False), (-1.0001e-6, True)])
def test_only_one_micrometre_arithmetic_tolerance(z, below):
    result = HeightPlane(np.eye(4), "tray plane").measure([0., 0., z])
    assert result.below_plane is below


@pytest.mark.parametrize("damage", ["shape", "nan", "scale", "reflection", "vertical"])
def test_invalid_reference_is_geometry_error(damage):
    plane = np.eye(4)
    if damage == "shape":
        plane = np.eye(3)
    elif damage == "nan":
        plane[2, 3] = np.nan
    elif damage == "scale":
        plane[0, 0] = 2.
    elif damage == "reflection":
        plane[0, 0] = -1.
    else:
        plane[:3, :3] = rpy_matrix(0., np.pi / 2, 0.)
    with pytest.raises(SurfaceGeometryError, match="Invalid tray plane"):
        HeightPlane(plane, "tray plane")


@pytest.mark.parametrize("point", [[0., 0.], [0., 0., np.nan], [np.inf, 0., 1.], None])
def test_invalid_surface_is_geometry_error(point):
    with pytest.raises(SurfaceGeometryError, match="finite base-frame coordinates"):
        HeightPlane(np.eye(4), "bin floor").measure(point)
