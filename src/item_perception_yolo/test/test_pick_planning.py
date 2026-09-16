"""Shared Link6 robot-camera clearance decisions are deterministic and pure."""

import numpy as np
import pytest

from item_perception_yolo.pick_planning import (
    candidate_pose_in_base, point_in_polygon, select_pick_attitude)


ROI = [[-0.2, -0.15], [-0.2, 0.15], [0.2, 0.15], [0.2, -0.15]]


def item(x=0.0, y=0.0, yaw_deg=0.0):
    yaw = np.deg2rad(yaw_deg)
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
    boundary = choose(item(0.15))
    assert normal.accepted and normal.mirrored is False
    assert boundary.accepted and boundary.mirrored is False
    assert boundary.selected_camera_platform_xy == pytest.approx((0.2, 0.0))
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
