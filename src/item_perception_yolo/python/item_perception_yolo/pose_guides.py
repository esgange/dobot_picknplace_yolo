"""RViz-only XY axes and an upward normal; never a replacement pose or TF."""

from copy import deepcopy

from geometry_msgs.msg import Point
from rclpy.duration import Duration
from visualization_msgs.msg import Marker, MarkerArray


# Match RViz Humble TF axes at Marker Scale 1: 0.2 m long, 0.02 m cylinder width.
AXIS_LENGTH_M = .2
AXIS_SHAFT_DIAMETER_M = .02


def pose_guide_markers(transform, namespace, identifier=0):
    """Keep XY exact; choose the local Z sign with nonnegative base_link Z.

    These independent arrows are intentionally not a rotation matrix. A vertical
    surface has no upward-facing normal; its horizontal local Z stays unchanged.
    The caller's transform and every controller-facing pose remain untouched.
    """
    if transform.header.frame_id != "base_link":
        raise ValueError("Pose guides require a base_link transform")
    q = transform.transform.rotation
    z_sign = -1. if 1. - 2. * (q.x * q.x + q.y * q.y) < -1e-12 else 1.
    markers = []
    for axis in range(3):
        marker = Marker()
        marker.header = deepcopy(transform.header)
        marker.ns = namespace + ("_up" if axis == 2 else "_xy")
        marker.id = identifier * 4 + axis
        marker.type, marker.action = Marker.ARROW, Marker.ADD
        t = transform.transform.translation
        marker.pose.position = Point(x=t.x, y=t.y, z=t.z)
        marker.pose.orientation = deepcopy(q)
        endpoint = [0., 0., 0.]
        endpoint[axis] = AXIS_LENGTH_M * (z_sign if axis == 2 else 1.)
        marker.points = [Point(), Point(x=endpoint[0], y=endpoint[1], z=endpoint[2])]
        marker.scale.x = AXIS_SHAFT_DIAMETER_M
        marker.scale.y, marker.scale.z = .04, .04
        marker.color.a = 1.
        setattr(marker.color, ("r", "g", "b")[axis], 1.)
        marker.lifetime = Duration(seconds=2.5).to_msg()
        markers.append(marker)
    return markers


class PoseGuidePublisher:
    """Small teaching-only marker batch with explicit removal and bounded lifetime."""

    def __init__(self, node, topic, namespace):
        self.publisher = node.create_publisher(MarkerArray, topic, 1)
        self.namespace = namespace
        self.visible = False

    def publish(self, transforms):
        markers = [Marker(action=Marker.DELETEALL)]
        for index, transform in enumerate(transforms):
            markers.extend(pose_guide_markers(transform, self.namespace, index))
        self.publisher.publish(MarkerArray(markers=markers))
        self.visible = len(markers) > 1

    def clear(self):
        if self.visible:
            self.publisher.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
            self.visible = False
