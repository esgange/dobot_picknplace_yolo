"""Read-only RViz housing marker attached to calibrated Link6; no robot clients."""

import os
import threading
import time

import numpy as np
from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker

from camera_calibration_gui.calibration_core import (
    quaternion_to_rotation_matrix, rotation_matrix_to_quaternion)
from .pick_planning import CAMERA_BODY_SIZE_RGB_M, camera_body_pose, rigid_matrix
from .item_teach_calibration import validate_selected_robot_camera


CAMERA_BOX_TOPIC = "/item_teach/robot_camera_body"
CAMERA_MOUNT_TOPIC = "/item_teach/robot_camera_mount"
MOUNT_MAX_AGE_SEC = 2.5


def mount_message(matrix, stamp):
    """One validated Link6-relative mount, or an empty array to hide the body."""
    message = PoseArray()
    message.header.frame_id, message.header.stamp = "Link6", stamp
    if matrix is not None:
        matrix = rigid_matrix(matrix, "Item Teach camera mount")
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
        q = pose.orientation
        q.x, q.y, q.z, q.w = rotation_matrix_to_quaternion(matrix[:3, :3])
        message.poses = [pose]
    return message


class TeachingCameraMount:
    """Publish only Item Teach's selected, hash-validated mount; no catalog scan."""
    def __init__(self, node):
        self.node = node
        self.lock = threading.RLock()
        self.camera = None
        self.closed = False
        self.publisher = node.create_publisher(
            PoseArray, CAMERA_MOUNT_TOPIC,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = node.create_timer(1., self.publish)
        self.publish()

    def set_camera(self, camera):
        with self.lock:
            if not self.closed:
                self.camera = camera
                self.publish()

    def clear(self):
        with self.lock:
            self.camera = None
            self.publisher.publish(mount_message(None, self.node.get_clock().now().to_msg()))

    def publish(self):
        with self.lock:
            if self.closed:
                return
            matrix = None
            if self.camera is not None and not self.node.fatal_error:
                try:
                    camera = validate_selected_robot_camera(self.camera, root=self.node.root)
                    matrix = camera.reference_from_camera_link
                except (OSError, ValueError, RuntimeError) as exc:
                    self.camera = None  # Explicit reload required after invalidation.
                    self.node.events.record("WARNING", "camera_body_cleared", str(exc))
            self.publisher.publish(mount_message(matrix, self.node.get_clock().now().to_msg()))

    def close(self):
        with self.lock:
            self.closed = True
            self.timer.cancel()
            self.clear()


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
        self.last_stamp_ns = 0
        self.last_receipt = None
        self.publisher = self.create_publisher(
            Marker, CAMERA_BOX_TOPIC,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            PoseArray, CAMERA_MOUNT_TOPIC, self.receive_mount,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.1, self.expire)
        self.clear()

    def clear(self):
        self.last_receipt = None
        marker = Marker()
        marker.header.frame_id = "Link6"
        marker.ns, marker.id, marker.action = "robot_camera_body", 0, Marker.DELETE
        self.publisher.publish(marker)

    def receive_mount(self, message):
        if not message.poses:
            self.clear()
            return
        try:
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if (message.header.frame_id != "Link6" or len(message.poses) != 1
                    or stamp <= self.last_stamp_ns or not 0 <= age < MOUNT_MAX_AGE_SEC):
                raise ValueError("Invalid or stale Item Teach camera mount")
            p, q = message.poses[0].position, message.poses[0].orientation
            quaternion = np.array([q.x, q.y, q.z, q.w])
            if (not np.isfinite(quaternion).all()
                    or abs(float(quaternion @ quaternion) - 1.) > 1e-6):
                raise ValueError("Invalid Item Teach camera quaternion")
            matrix = np.eye(4)
            matrix[:3, :3] = quaternion_to_rotation_matrix(*quaternion)
            matrix[:3, 3] = [p.x, p.y, p.z]
            marker = camera_box_marker(matrix, message.header.stamp)
        except ValueError as exc:
            self.clear()
            self.get_logger().warning(f"Camera body unavailable: {exc}")
        else:
            self.publisher.publish(marker)
            self.last_stamp_ns = stamp
            self.last_receipt = time.monotonic()

    def expire(self):
        if self.last_receipt is not None and (
                time.monotonic() - self.last_receipt >= MOUNT_MAX_AGE_SEC
                or not 0 <= (self.get_clock().now().nanoseconds - self.last_stamp_ns) / 1e9
                < MOUNT_MAX_AGE_SEC):
            self.clear()


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
