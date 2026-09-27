"""An upward display normal must never change a pose, its XY axes or handedness."""

from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

from camera_calibration_gui.calibration_core import (
    quaternion_to_rotation_matrix, rotation_matrix_to_quaternion)
from item_perception_yolo.pick_planning import rpy_matrix
from item_perception_yolo.pose_guides import pose_guide_markers, PoseGuidePublisher


@pytest.mark.parametrize("roll,pitch,yaw", [
    (0., 0., 0.), (np.pi, 0., 0.), (2.5, .3, -.8), (.2, -.4, 1.1),
    (np.pi / 2, 0., 0.), (-np.pi / 2, 0., 0.), (0., np.pi / 2, 0.)])
def test_guides_point_up_without_changing_real_frame(roll, pitch, yaw):
    rotation = rpy_matrix(roll, pitch, yaw)
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.header.stamp.sec = 123
    t, q = transform.transform.translation, transform.transform.rotation
    t.x, t.y, t.z = .3, -.5, .12
    q.x, q.y, q.z, q.w = rotation_matrix_to_quaternion(rotation)
    original = deepcopy(transform)
    markers = pose_guide_markers(transform, "test", 4)
    for axis, marker in enumerate(markers):
        assert marker.header == original.header
        assert marker.pose.orientation == original.transform.rotation
        assert marker.pose.position.x == t.x and marker.pose.position.y == t.y
        assert marker.pose.position.z == t.z
        assert marker.id == 16 + axis and marker.type == Marker.ARROW
        assert getattr(marker.color, ("r", "g", "b")[axis]) == 1.
        tip = marker.points[1]
        delta = rotation @ np.array([tip.x, tip.y, tip.z])
        assert np.linalg.norm(delta) == pytest.approx(.04)
        if axis < 2:
            assert np.allclose(delta, rotation[:, axis] * .04)
        else:
            assert delta[2] >= -1e-12
            assert np.linalg.norm(np.cross(delta, rotation[:, 2])) < 1e-12
            assert marker.ns == "test_up" and not marker.text
    assert transform == original
    # Message ownership is separate; renderer changes cannot alter a published TF.
    markers[0].pose.orientation.x = .123
    markers[0].header.stamp.sec = 999
    assert transform == original
    assert np.linalg.det(quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)) == pytest.approx(1.)


def test_marker_batches_replace_and_clear_without_tf_or_command_publishers():
    node = SimpleNamespace(create_publisher=MagicMock(return_value=MagicMock()))
    preview = PoseGuidePublisher(node, "/test/pose_guides", "test")
    pose = TransformStamped()
    pose.header.frame_id = "base_link"
    pose.transform.rotation.x = 1.
    pose.transform.rotation.w = 0.
    preview.publish([pose, pose])
    message = preview.publisher.publish.call_args.args[0]
    assert len(message.markers) == 7 and message.markers[0].action == Marker.DELETEALL
    assert all(m.lifetime.sec == 2 and m.lifetime.nanosec == 500000000
               for m in message.markers[1:])
    preview.publish([pose])
    assert len(preview.publisher.publish.call_args.args[0].markers) == 4
    preview.clear()
    message = preview.publisher.publish.call_args.args[0]
    assert len(message.markers) == 1 and message.markers[0].action == Marker.DELETEALL
    preview.clear()
    assert preview.publisher.publish.call_count == 3
    assert node.create_publisher.call_count == 1


def test_guide_up_requires_base_frame():
    with pytest.raises(ValueError, match="base_link"):
        pose_guide_markers(TransformStamped(), "test")
