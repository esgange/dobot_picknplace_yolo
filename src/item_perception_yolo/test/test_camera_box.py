"""Housing visualization uses the calibrated planner geometry without robot commands."""

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.time import Time
from visualization_msgs.msg import Marker

import item_perception_yolo.camera_box as box_module
from item_perception_yolo.camera_box import (
    RobotCameraBox, TeachingCameraMount, camera_box_marker, mount_message)
from item_perception_yolo.pick_planning import camera_body_corners, rpy_matrix


@pytest.mark.parametrize('values', [[0., 0., 0., 0., 0., 0.],
                                  [80., -50., 20., 35., -20., 70.]])
def test_marker_follows_link6_with_the_rgb_housing_offset_rotation_and_size(values):
    mount = np.eye(4)
    mount[:3, :3] = rpy_matrix(*np.deg2rad(values[3:]))
    mount[:3, 3] = np.array(values[:3]) / 1000.
    marker = camera_box_marker(mount, Time(seconds=1).to_msg())
    assert marker.type == Marker.CUBE and marker.frame_locked
    assert marker.header.frame_id == 'Link6'
    assert marker.lifetime.sec == 3 and marker.color.a > 0
    p, q = marker.pose.position, marker.pose.orientation
    from camera_calibration_gui.calibration_core import quaternion_to_rotation_matrix
    displayed = np.eye(4)
    displayed[:3, :3] = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
    displayed[:3, 3] = [p.x, p.y, p.z]
    link_from_body = np.array([[0., 0., 1., -.01077], [-1., 0., 0., -.025],
                               [0., -1., 0., 0.], [0., 0., 0., 1.]])
    assert np.allclose(displayed, mount @ link_from_body)
    assert (marker.scale.x, marker.scale.y, marker.scale.z) == (.090, .025, .030)
    box_vertices = np.array([[x, y, z] for x in (-.045, .045)
                             for y in (-.0125, .0125) for z in (-.015, .015)])
    shown = box_vertices @ displayed[:3, :3].T + displayed[:3, 3]
    assert np.allclose(shown, camera_body_corners(mount))


def source_node():
    return SimpleNamespace(root=None, fatal_error=None, events=Mock(),
                           create_publisher=Mock(return_value=Mock()),
                           create_timer=Mock(return_value=Mock()),
                           get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10)))


def test_teaching_mount_tracks_selected_camera_and_invalidates_changed_files(monkeypatch):
    node = source_node()
    loader = Mock(side_effect=lambda camera, **_: camera)
    monkeypatch.setattr(box_module, 'validate_selected_robot_camera', loader)
    owner = TeachingCameraMount(node)
    publisher = owner.publisher
    assert not publisher.publish.call_args.args[0].poses
    first = SimpleNamespace(reference_from_camera_link=np.eye(4))
    second = SimpleNamespace(reference_from_camera_link=np.eye(4))
    second.reference_from_camera_link[0, 3] = .2
    owner.set_camera(first)
    assert publisher.publish.call_args.args[0].poses[0].position.x == 0.
    owner.set_camera(second)
    loader.assert_called_with(second, root=None)
    assert publisher.publish.call_args.args[0].poses[0].position.x == .2
    loader.side_effect = ValueError('Selected calibration hash changed')
    owner.publish()
    assert not publisher.publish.call_args.args[0].poses and owner.camera is None
    node.events.record.assert_called_once()
    loader.reset_mock()
    owner.publish()
    loader.assert_not_called()  # No automatic latest-file substitution or reload.


def test_selection_clear_and_shutdown_cannot_republish_an_old_mount(monkeypatch):
    monkeypatch.setattr(box_module, 'validate_selected_robot_camera', lambda camera, **_: camera)
    owner = TeachingCameraMount(source_node())
    camera = SimpleNamespace(reference_from_camera_link=np.eye(4))
    owner.set_camera(camera)
    owner.clear()
    owner.publish()
    assert not owner.publisher.publish.call_args.args[0].poses
    owner.set_camera(camera)
    owner.close()
    owner.timer.cancel.assert_called_once()
    assert not owner.publisher.publish.call_args.args[0].poses
    owner.publisher.reset_mock()
    owner.publish()
    owner.set_camera(camera)
    owner.publisher.publish.assert_not_called()


@pytest.fixture
def display():
    node = SimpleNamespace(last_stamp_ns=0, last_receipt=None, publisher=Mock(),
                           get_logger=lambda: Mock(),
                           get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10)))
    node.clear = lambda: RobotCameraBox.clear(node)
    return node


@pytest.mark.parametrize('invalid', ['empty', 'stale', 'future', 'repeated', 'frame',
                                    'multiple', 'quaternion', 'nan'])
def test_display_clears_invalid_or_unavailable_mount(display, invalid):
    good = mount_message(np.eye(4), Time(seconds=9).to_msg())
    RobotCameraBox.receive_mount(display, good)
    assert display.publisher.publish.call_args.args[0].action == Marker.ADD
    message = mount_message(np.eye(4), Time(seconds=10).to_msg())
    if invalid == 'empty':
        message.poses = []
    elif invalid == 'stale':
        message.header.stamp = Time(seconds=6).to_msg()
    elif invalid == 'future':
        message.header.stamp = Time(seconds=11).to_msg()
    elif invalid == 'repeated':
        message = good
    elif invalid == 'frame':
        message.header.frame_id = 'base_link'
    elif invalid == 'multiple':
        message.poses *= 2
    elif invalid == 'quaternion':
        message.poses[0].orientation.w = 0.
    else:
        message.poses[0].position.x = float('nan')
    RobotCameraBox.receive_mount(display, message)
    assert display.publisher.publish.call_args.args[0].action == Marker.DELETE


def test_display_expires_after_owner_disappears_and_accepts_new_mount(display, monkeypatch):
    now = [100.]
    monkeypatch.setattr(box_module.time, 'monotonic', lambda: now[0])
    RobotCameraBox.receive_mount(display, mount_message(np.eye(4), Time(seconds=9).to_msg()))
    now[0] += 2.4
    RobotCameraBox.expire(display)
    assert display.publisher.publish.call_args.args[0].action == Marker.ADD
    now[0] += .1
    RobotCameraBox.expire(display)
    assert display.publisher.publish.call_args.args[0].action == Marker.DELETE
    changed = np.eye(4)
    changed[1, 3] = .3
    RobotCameraBox.receive_mount(display, mount_message(changed, Time(seconds=10).to_msg()))
    assert display.publisher.publish.call_args.args[0].action == Marker.ADD
    assert display.publisher.publish.call_args.args[0].pose.position.y == pytest.approx(.275)


def test_marker_rejects_nonrigid_pose():
    with pytest.raises(ValueError):
        camera_box_marker(np.zeros((4, 4)), Time().to_msg())
