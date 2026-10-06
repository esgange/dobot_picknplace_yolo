"""Housing visualization uses the calibrated planner geometry without robot commands."""

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.time import Time
from visualization_msgs.msg import Marker

import robot_controller.camera_box as box_module
from robot_controller.camera_box import RobotCameraBox, camera_box_marker
from robot_controller.kinematics import pose_matrix
from item_perception_yolo.pick_planning import camera_body_corners


@pytest.mark.parametrize('values', [[0., 0., 0., 0., 0., 0.],
                                  [80., -50., 20., 35., -20., 70.]])
def test_marker_follows_link6_with_the_rgb_housing_offset_rotation_and_size(values):
    mount = pose_matrix(values)
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


def test_invalid_calibration_clears_box_and_logs_once_then_recovers(monkeypatch):
    node = SimpleNamespace(root=None, last_status=None, publisher=Mock(),
                           get_logger=lambda: logger,
                           get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=1)))
    logger = Mock()
    node.clear = lambda: RobotCameraBox.clear(node)
    loader = Mock(side_effect=ValueError('Missing camera calibration'))
    monkeypatch.setattr(box_module, 'latest_robot_camera_calibration', loader)
    for _ in range(2):
        RobotCameraBox.publish_box(node)
        assert node.publisher.publish.call_args.args[0].action == Marker.DELETE
    logger.warning.assert_called_once()
    loader.side_effect = None
    loader.return_value = SimpleNamespace(
        reference_from_camera_link=np.eye(4), path=SimpleNamespace(name='robot.yaml'), sha256='abc')
    RobotCameraBox.publish_box(node)
    assert node.publisher.publish.call_args.args[0].action == Marker.ADD
    logger.info.assert_called_once()


def test_marker_rejects_nonrigid_pose():
    with pytest.raises(ValueError):
        camera_box_marker(np.zeros((4, 4)), Time().to_msg())
