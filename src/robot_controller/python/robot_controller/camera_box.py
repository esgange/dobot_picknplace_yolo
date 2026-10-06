"""Read-only RViz housing marker attached to calibrated Link6; no robot clients."""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker

from camera_calibration_gui.calibration_core import rotation_matrix_to_quaternion
from item_perception_yolo.pick_planning import CAMERA_BODY_SIZE_RGB_M, camera_body_pose
from item_perception_yolo.platform_teach_core import workspace_root
from item_perception_yolo.station_calibration import latest_robot_camera_calibration


CAMERA_BOX_TOPIC = "/robot_controller/robot_camera_body"


def camera_box_marker(link6_from_camera, stamp):
    """Same RGB-optical housing offset and size used by the clearance planner."""
    pose = camera_body_pose(link6_from_camera)
    marker = Marker()
    marker.header.frame_id, marker.header.stamp = "Link6", stamp
    marker.ns, marker.id = "robot_camera_body", 0
    marker.type, marker.action = Marker.CUBE, Marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, pose[:3, 3])
    q = marker.pose.orientation
    q.x, q.y, q.z, q.w = rotation_matrix_to_quaternion(pose[:3, :3])
    marker.scale.x, marker.scale.y, marker.scale.z = CAMERA_BODY_SIZE_RGB_M
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .1, .8, .65
    marker.frame_locked = True  # Follow live Link6 TF without publishing a competing camera TF.
    marker.lifetime.sec = 3
    return marker


class RobotCameraBox(Node):
    def __init__(self):
        super().__init__("robot_camera_box")
        self.root = workspace_root()
        self.last_status = None
        self.publisher = self.create_publisher(
            Marker, CAMERA_BOX_TOPIC,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1., self.publish_box)
        self.publish_box()

    def clear(self):
        marker = Marker()
        marker.ns, marker.id, marker.action = "robot_camera_body", 0, Marker.DELETE
        self.publisher.publish(marker)

    def publish_box(self):
        try:
            # Match controller/preview's strict current station selection. Reload
            # only this read-only display; never change a running controller's config.
            camera = latest_robot_camera_calibration(self.root)
            marker = camera_box_marker(camera.reference_from_camera_link,
                                       self.get_clock().now().to_msg())
            status = str(camera.path), camera.sha256
        except (OSError, ValueError, RuntimeError) as exc:
            self.clear()
            status = str(exc)
            if status != self.last_status:
                self.get_logger().warning(f"Camera body unavailable: {exc}")
        else:
            self.publisher.publish(marker)
            if status != self.last_status:
                self.get_logger().info(
                    f"Gemini 335 body 90×25×30 mm; RGB optical center offset "
                    f"(+11, 0, -12.79) mm, nominal mechanical model; "
                    f"calibration: {camera.path.name}")
        self.last_status = status


def main():
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required")
    rclpy.init()
    node = None
    try:
        node = RobotCameraBox()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.clear()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
