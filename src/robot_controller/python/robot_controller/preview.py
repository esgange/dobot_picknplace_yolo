"""TF-only Home/Pick planner; this process never creates a Dobot command client."""

import os
from pathlib import Path
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster

from camera_calibration_gui.calibration_core import rotation_matrix_to_quaternion
from item_perception_yolo.platform_teach_core import workspace_root
from robot_controller_interfaces.srv import Preview

from .candidates import (
    CANDIDATE_SERVICE, CANONICAL_CANDIDATE_PROVIDERS, CandidateClient)
from .configuration import load_configuration
from .controller import PackageEventLogger
from .kinematics import Cr10Kinematics
from .motion import Target, candidate_pose_in_base, pick_targets


class RobotControllerPreview(rclpy.node.Node):
    def __init__(self):
        super().__init__("robot_controller_preview")
        self.root = workspace_root()
        model = Path(get_package_share_directory("cra_description")) / "urdf/cr10_robot.xacro"
        self.kinematics = Cr10Kinematics(model)
        self.events = PackageEventLogger(self.root, "robot_controller_preview")
        self.client = CandidateClient(self, self.root)
        self.cancel = threading.Event()
        self.lock = threading.Lock()
        self.configuration = None
        self.targets = ()
        self.frames = ()
        self.broadcaster = TransformBroadcaster(self)
        group = ReentrantCallbackGroup()
        self.service = self.create_service(
            Preview, "/robot_controller/preview", self._preview, callback_group=group)
        self.create_timer(0.1, self._broadcast)
        self.events.record(
            "INFO", "preview_started",
            "TF-only preview started; no Dobot command clients exist in this process")

    def wait_control(self, seconds):
        self.cancel.wait(seconds)

    def check_detector_owner(self):
        providers = []
        for name, namespace in self.get_node_names_and_namespaces():
            names = self.get_service_names_and_types_by_node(name, namespace)
            if CANDIDATE_SERVICE in (service for service, _types in names):
                providers.append((name, namespace))
        if (len(providers) != 1
                or providers[0] not in CANONICAL_CANDIDATE_PROVIDERS):
            raise ValueError(
                "Preview requires exactly one canonical item_detect/item_teach pose service")

    def _clear(self):
        self.cancel.set()
        self.configuration = None
        self.targets = self.frames = ()

    def _preview(self, request, response):
        if not self.lock.acquire(blocking=False):
            response.success = False
            response.message = "Another preview request is active"
            return response
        try:
            self._clear()
            self.cancel.clear()
            if request.operation == request.CLEAR:
                response.success = True
                response.message = "Preview TFs cleared"
                response.configuration_id = ""
                response.tf_frames = []
                return response
            if request.operation not in (request.HOME, request.PICK):
                raise ValueError("Unknown preview operation")
            config = load_configuration(
                request.item_teach_file,
                request.bin_teach_file if request.operation == request.PICK else "",
                self.root, self.kinematics, deployment=False)
            targets = [Target(
                "home", config.home_matrix.copy(),
                config.profile["speed"]["travel_percent"],
                config.profile["acceleration"]["travel_percent"], config.home_joints)]
            if request.operation == request.PICK:
                batch = self.client.request(
                    config, save_debug_images=False, cancel=self.cancel.is_set)
                for index, candidate in enumerate(batch.candidates, 1):
                    item_pose = candidate_pose_in_base(
                        config.selection.station.platform.base_from_platform,
                        candidate.position_m, candidate.quaternion)
                    plan = pick_targets(
                        config.home_matrix, item_pose, config.profile, index)
                    targets.extend(plan)
                    if index == len(batch.candidates):
                        targets.append(Target(
                            f"p{index}_return_home", config.home_matrix.copy(),
                            config.profile["speed"]["travel_percent"],
                            config.profile["acceleration"]["travel_percent"],
                            config.home_joints))
            frames = tuple(f"robot_controller_preview_{target.name}_{index}"
                           for index, target in enumerate(targets, 1))
            self.configuration = config
            self.targets, self.frames = tuple(targets), frames
            response.success = True
            response.message = f"Published {len(frames)} TF-only planned targets"
            response.configuration_id = config.configuration_id
            response.tf_frames = list(frames)
            self.events.record(
                "INFO", "preview_installed", response.message,
                configuration_id=config.configuration_id, operation=int(request.operation),
                frames=list(frames))
        except Exception as exc:
            self._clear()
            response.success = False
            response.message = str(exc)
            response.configuration_id = ""
            response.tf_frames = []
            self.events.record("ERROR", "preview_failed", str(exc))
        finally:
            self.lock.release()
        return response

    def _broadcast(self):
        config, targets, frames = self.configuration, self.targets, self.frames
        if config is None:
            return
        try:
            config.validate_sources(self.root)
            messages = []
            stamp = self.get_clock().now().to_msg()
            for target, frame in zip(targets, frames):
                message = TransformStamped()
                message.header.stamp = stamp
                message.header.frame_id = "base_link"
                message.child_frame_id = frame
                translation = message.transform.translation
                translation.x, translation.y, translation.z = map(float, target.matrix[:3, 3])
                rotation = message.transform.rotation
                rotation.x, rotation.y, rotation.z, rotation.w = (
                    rotation_matrix_to_quaternion(target.matrix[:3, :3]))
                messages.append(message)
            self.broadcaster.sendTransform(messages)
        except Exception as exc:
            self._clear()
            self.events.record("WARNING", "preview_cleared", str(exc))

    def close_runtime(self):
        self._clear()
        self.client.close()


def main(args=None):
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required")
    rclpy.init(args=args)
    node = RobotControllerPreview()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close_runtime()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
