"""Shared Link6 robot-camera clearance decisions are deterministic and pure."""

import numpy as np
import pytest

from item_perception_yolo.pick_planning import (
    camera_body_corners, camera_body_footprint, camera_body_pose_from_rgb,
    camera_link_from_rgb_optical,
    candidate_pose_in_base, pick_attitude, point_in_polygon, rpy_matrix, select_pick_attitude)


ROI = [[-0.2, -0.15], [-0.2, 0.15], [0.2, 0.15], [0.2, -0.15]]


def item(x=0.0, y=0.0, yaw_deg=0.0):
    # yaw_deg describes the physical long side; the pose's X is its short side.
    yaw = np.deg2rad(yaw_deg + 90.0)
    return candidate_pose_in_base(
        np.eye(4), (x, y, 0.1),
        (0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)))


def camera_offset(x=0.05, y=0.0):
    value = np.eye(4)
    value[:2, 3] = [x, y]
    return value


def choose(candidate, camera=None):
    return select_pick_attitude(
        np.eye(4), candidate, 0.0, 90.0, np.eye(4),
        camera_offset() if camera is None else camera, ROI)


def test_normal_inside_is_retained_and_boundary_is_inclusive():
    normal = choose(item(0.0))
    boundary = choose(item(0.14577))  # Link origin .19577 + 4.23 mm to front glass.
    assert normal.accepted and normal.mirrored is False
    assert boundary.accepted and boundary.mirrored is False
    assert boundary.selected_camera_platform_xy == pytest.approx((0.19577, 0.0))
    assert max(p[0] for p in boundary.selected_camera_footprint_xy) == pytest.approx(.2)
    assert point_in_polygon((0.2, 0.0), ROI)


def test_normal_outside_uses_exact_180_degree_tool_z_mirror():
    selected = choose(item(0.18))
    assert selected.accepted and selected.mirrored is True
    assert selected.normal_camera_platform_xy == pytest.approx((0.23, 0.0))
    assert selected.mirrored_camera_platform_xy == pytest.approx((0.13, 0.0))
    normal = select_pick_attitude(
        np.eye(4), item(0.18), 0.0, 90.0, np.eye(4), np.eye(4), ROI)
    assert np.allclose(normal.rotation.T @ selected.rotation,
                       np.diag([-1.0, -1.0, 1.0]), atol=1e-12)
    assert np.allclose(selected.rotation[:, 2], np.eye(3)[:, 2])
    assert abs(float(np.dot(selected.rotation[:, 1], normal.rotation[:, 1]))) == 1.0


def test_both_attitudes_outside_rejects_candidate():
    selected = choose(item(0.3))
    assert not selected.accepted
    assert selected.rotation is None and selected.planned_link6 is None
    assert not point_in_polygon(selected.normal_camera_platform_xy, ROI)
    assert not point_in_polygon(selected.mirrored_camera_platform_xy, ROI)


def test_center_inside_but_housing_crosses_edge_uses_safe_mirror():
    selected = choose(item(.15))
    assert point_in_polygon(selected.normal_camera_platform_xy, ROI)
    assert selected.accepted and selected.mirrored
    assert max(x for x, _ in selected.normal_camera_footprint_xy) > .2
    assert all(point_in_polygon(p, ROI) for p in selected.selected_camera_footprint_xy)


def test_both_centers_inside_but_bodies_outside_rejects_candidate():
    selected = choose(item(.198), camera=np.eye(4))
    assert point_in_polygon(selected.normal_camera_platform_xy, ROI)
    assert point_in_polygon(selected.mirrored_camera_platform_xy, ROI)
    assert not selected.accepted and selected.selected_camera_footprint_xy is None


def test_camera_width_is_local_y_and_rotates_with_mounting():
    pose = np.eye(4)
    corners = camera_body_corners(pose)
    assert np.ptp(corners, axis=0) == pytest.approx((.030, .090, .025))
    assert corners.mean(axis=0) == pytest.approx((-.01077, -.025, 0.))
    pose[:3, :3] = rpy_matrix(0, 0, np.pi / 2)
    assert np.ptp(camera_body_corners(pose), axis=0) == pytest.approx((.090, .030, .025))
    assert camera_body_corners(pose).mean(axis=0) == pytest.approx((.025, -.01077, 0.))


def test_rgb_reference_matches_documented_glass_and_housing_bounds():
    # RGB lens: 11 mm left of case center, 2.21 mm behind front glass.
    # Test in optical axes, independently of the planner's Link6/platform chain.
    rgb_from_link = np.linalg.inv(camera_link_from_rgb_optical())
    corners = camera_body_corners(rgb_from_link)
    assert corners.min(axis=0) == pytest.approx((-.034, -.0125, -.02779))
    assert corners.max(axis=0) == pytest.approx((.056, .0125, .00221))
    assert camera_body_pose_from_rgb(np.eye(4))[:3, 3] == pytest.approx((.011, 0., -.01279))
    link_corners = camera_body_corners(np.eye(4))
    assert link_corners.min(axis=0) == pytest.approx((-.02577, -.070, -.0125))
    assert link_corners.max(axis=0) == pytest.approx((.00423, .020, .0125))


def test_offset_body_crossing_side_wall_mirrors_even_when_centered_box_would_fit():
    selected = choose(item(y=-.09), camera=np.eye(4))
    # A centered 90 mm box reaches only -.135; the real rearward/sideways
    # offset takes the normal housing to -.16, across the -.15 green wall.
    assert selected.accepted and selected.mirrored
    assert min(y for _, y in selected.normal_camera_footprint_xy) == pytest.approx(-.16)
    assert all(point_in_polygon(p, ROI) for p in selected.selected_camera_footprint_xy)


def test_tilted_housing_projects_all_eight_corners_in_platform_frame():
    base_platform, camera = np.eye(4), np.eye(4)
    base_platform[:3, :3] = rpy_matrix(.2, -.3, .4)
    base_platform[:3, 3] = [.2, -.1, .1]
    camera[:3, :3] = rpy_matrix(.7, .3, -.4)
    camera[:3, 3] = [.05, .02, -.03]
    roi = [[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]
    selected = select_pick_attitude(
        np.eye(4), item(), 0., 90., base_platform, camera, roi)
    expected = np.linalg.inv(base_platform) @ selected.planned_link6 @ camera
    corners = camera_body_corners(expected)
    footprint = selected.selected_camera_footprint_xy
    assert len(footprint) == 6
    assert footprint == camera_body_footprint(expected)
    assert all(point_in_polygon(p[:2], footprint) for p in corners)
    assert np.ptp(np.asarray(footprint), axis=0) == pytest.approx(np.ptp(corners[:, :2], axis=0))


def test_camera_body_cannot_cross_narrow_bin_even_with_center_inside():
    with pytest.raises(ValueError, match="convex"):
        select_pick_attitude(np.eye(4), item(), 0., 0., np.eye(4), np.eye(4),
                             [[-1., -1.], [1., -1.], [0., 0.], [-1., 1.]])
    selected = select_pick_attitude(
        np.eye(4), item(), 0., 0., np.eye(4), np.eye(4),
        [[-.2, -.04], [.2, -.04], [.2, .04], [-.2, .04]])
    assert not selected.accepted  # 90 mm body does not fit in an 80 mm bin.


def test_each_candidate_is_planned_independently_from_home_with_offset():
    first = select_pick_attitude(
        np.eye(4), item(0.0, yaw_deg=70), 20.0, 90.0,
        np.eye(4), camera_offset(0.01), ROI)
    second = select_pick_attitude(
        np.eye(4), item(0.0, yaw_deg=-70), 20.0, 90.0,
        np.eye(4), camera_offset(0.01), ROI)
    assert first.accepted and second.accepted
    assert abs(first.rotation_from_home_deg) <= 90
    assert abs(second.rotation_from_home_deg) <= 90
    assert first.rotation_from_home_deg == pytest.approx(-second.rotation_from_home_deg)


@pytest.mark.parametrize("home_yaw", [0, 30, 90, 180])
@pytest.mark.parametrize("equivalent", [-180, 0, 180])
@pytest.mark.parametrize("offset", [20, 45])
def test_equal_travel_uses_same_ccw_choice_for_equivalent_axis_frames(home_yaw, equivalent, offset):
    home = np.eye(4)
    home[:3, :3] = rpy_matrix(0., 0., np.deg2rad(home_yaw))
    candidate = item(yaw_deg=home_yaw + 90 + equivalent)
    rotation, travel, direction = pick_attitude(home, candidate, offset)
    expected_tool = rpy_matrix(0., 0., np.deg2rad(home_yaw - (90 - offset)))
    assert direction == "ccw"
    assert travel == pytest.approx(-(90 - offset))
    assert np.allclose(rotation, expected_tool, atol=1e-12)
