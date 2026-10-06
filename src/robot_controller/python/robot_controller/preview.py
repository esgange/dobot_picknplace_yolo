"""Read-only Home/Pick/Place route preview; no robot command clients."""

import os
from dataclasses import replace
from pathlib import Path
import threading
import math

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from dobot_msgs_v4.msg import RobotStatus

from camera_calibration_gui.calibration_core import rotation_matrix_to_quaternion
from item_perception_yolo.platform_teach_core import _parse_env_file, workspace_root
from item_perception_yolo.pick_planning import select_pick_attitude
from robot_controller_interfaces.srv import Preview

from .candidates import (
    CANDIDATE_SERVICE, CANONICAL_CANDIDATE_PROVIDERS, CandidateClient)
from .configuration import load_configuration
from .events import PackageEventLogger
from .errors import FeedbackFailure, OperationCanceled
from .feedback import FeedbackMonitor
from .kinematics import Cr10Kinematics
from .motion import (candidate_pose_in_base, candidate_exit_transit, cartesian_home_targets,
                     home_targets, pick_targets, pick_tray_target, pose_reached,
                     tray_detect_targets)
from .pick_session import return_targets
from .placement import TRAY_SPEED_PERCENT, place_targets, validate_target
from .tray_client import TrayClient


class RobotControllerPreview(rclpy.node.Node):
    def __init__(self):
        super().__init__("robot_controller_preview")
        self.root = workspace_root()
        model = Path(get_package_share_directory("cra_description")) / "urdf/cr10_robot.xacro"
        self.kinematics = Cr10Kinematics(model)
        self.events = PackageEventLogger(self.root, "robot_controller_preview")
        self.bringup_node = _parse_env_file(self.root / ".env")["DOBOT_ROBOT_NODE_NAME"]
        self.monitor = FeedbackMonitor(lambda: self.get_clock().now().nanoseconds)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(
            RobotStatus, "/dobot_msgs_v4/msg/RobotStatus", self.monitor.update_status, 10)
        self.create_subscription(String, "/dobot_bringup_ros2/msg/FeedInfo", self._on_feed, 10)
        self.client = CandidateClient(self, self.root)
        self.trays = TrayClient(self)
        self.cancel = threading.Event()
        self.lock = threading.Lock()
        self.state_lock = threading.RLock()
        self.generation = 0
        self.configuration = None
        self.origin = None
        self.request_origin = None
        self.targets = ()
        self.frames = ()
        self.broadcaster = TransformBroadcaster(self)
        group = ReentrantCallbackGroup()
        self.service = self.create_service(
            Preview, "/robot_controller/preview_v2", self._preview, callback_group=group)
        self.create_timer(0.1, self._broadcast)
        self.events.record(
            "INFO", "preview_started",
            "TF-only preview started; no Dobot command clients exist in this process")

    def wait_control(self, seconds):
        self.cancel.wait(seconds)

    def _service_providers(self, absolute_name):
        providers = []
        for name, namespace in self.get_node_names_and_namespaces():
            names = self.get_service_names_and_types_by_node(name, namespace)
            if absolute_name in (service for service, _types in names):
                providers.append((name, namespace))
        return providers

    def check_detector_owner(self):
        providers = self._service_providers(CANDIDATE_SERVICE)
        if (len(providers) != 1
                or providers[0] not in CANONICAL_CANDIDATE_PROVIDERS):
            raise ValueError(
                "Preview requires exactly one canonical item_detect/item_teach pose service")

    def _on_joints(self, message):
        try:
            self.monitor.update_joints(message)
        except FeedbackFailure as exc:
            self._clear()
            self.events.record("WARNING", "invalid_preview_joints", str(exc))

    def _on_feed(self, message):
        try:
            self.monitor.update_feed(message.data)
        except FeedbackFailure as exc:
            self._clear()
            self.events.record("WARNING", "invalid_preview_feedback", str(exc))

    def _snapshot(self):
        for topic in ("/joint_states", "/dobot_msgs_v4/msg/RobotStatus",
                      "/dobot_bringup_ros2/msg/FeedInfo"):
            publishers = self.get_publishers_info_by_topic(topic)
            if (len(publishers) != 1 or publishers[0].node_name != self.bringup_node
                    or publishers[0].node_namespace != "/"):
                raise FeedbackFailure(f"Preview requires canonical robot feedback on {topic}")
        sample = self.monitor.snapshot(require_enabled=False)
        if (sample.feed["RunningStatus"] or sample.feed["isRunQueuedCmd"]
                or not (sample.robot_enabled or sample.feed["robot_mode"] in (4, 9, 10))):
            raise FeedbackFailure("Stop robot motion before planning a preview")
        return sample

    def wait_for_resume(self):
        if self.cancel.is_set():
            raise OperationCanceled("Preview cleared or canceled")

    def _check_observation(self):
        self.wait_for_resume()
        current = self.kinematics.forward(self._snapshot().joints)
        if self.request_origin is not None and not pose_reached(
                current, self.request_origin, translation_m=.005, rotation_deg=1.):
            raise FeedbackFailure("Robot position changed during preview observation")

    def operation_progress(self, _phase, message):
        self._check_observation()
        self.events.record("INFO", "preview_observation", message)

    def _clear(self):
        with self.state_lock:
            self.generation += 1
            self.cancel.set()
            self.configuration = self.origin = None
            self.targets = self.frames = ()

    def _preview(self, request, response):
        # CLEAR must pre-empt a request blocked in perception. The running owner
        # observes cancellation before installing or publishing any new targets.
        if request.operation == request.CLEAR:
            self._clear()
            response.success = True
            response.message = "Preview TFs cleared"
            return response
        with self.state_lock:
            generation = self.generation
        if not self.lock.acquire(blocking=False):
            response.success = False
            response.message = "Another preview request is active"
            return response
        try:
            with self.state_lock:
                if generation != self.generation:
                    raise OperationCanceled("Preview cleared before planning started")
                self.configuration = self.origin = None
                self.targets = self.frames = ()
                self.cancel.clear()
            if request.operation not in (request.HOME, request.PICK, request.PLACE):
                raise ValueError("Unknown preview operation")
            config = load_configuration(
                request.item_teach_file,
                request.bin_teach_file if request.operation == request.PICK else "",
                self.root, self.kinematics, deployment=False,
                tray_path=(request.tray_teach_file
                           if request.operation in (request.PICK, request.PLACE) else ""))
            sample = self._snapshot()
            current = self.kinematics.forward(sample.joints)
            self.request_origin = current.copy()
            rates = dict(speed_percent=config.profile["speed"]["travel_percent"],
                         acceleration_percent=config.profile["acceleration"]["travel_percent"])
            at_home = np.allclose(sample.joints, config.home_joints,
                                  atol=math.radians(1.), rtol=0)
            targets = []
            tray_travel_only = False
            if request.operation == request.HOME:
                if not pose_reached(current, config.home_matrix,
                                    translation_m=.005, rotation_deg=1.):
                    targets.extend(cartesian_home_targets(current, config.home_matrix, **rates))
            if request.operation == request.PICK:
                tray_target = pick_tray_target(config.tray, config.profile)
                if not at_home:
                    targets.extend(home_targets(current, config.home_matrix,
                                                config.home_joints, **rates))
                batch = self.client.request(
                    config, save_debug_images=False, cancel=self.cancel.is_set)
                for index, candidate in enumerate(batch.candidates, 1):
                    item_pose = candidate_pose_in_base(
                        config.selection.station.platform.base_from_platform,
                        candidate.position_m, candidate.quaternion)
                    attitude = select_pick_attitude(
                        config.home_matrix, item_pose, config.profile["pick_rotation"],
                        config.profile["motion"]["standoff_height"],
                        config.selection.station.platform.base_from_platform,
                        config.selection.robot_camera.reference_from_camera_link,
                        [[point.x_m, point.y_m] for point in config.selection.bin.points])
                    if not attitude.accepted:
                        raise ValueError(
                            "Detector returned a candidate whose normal and 180-degree "
                            "robot-camera bodies extend outside the Bin ROI")
                    plan = pick_targets(
                        config.home_matrix, item_pose, config.profile, index,
                        rotation=attitude.rotation)
                    targets.extend(plan)
                    exit_transit = candidate_exit_transit(plan[5].matrix, plan)
                    targets.append(exit_transit)
                    targets.append(replace(tray_target, name=f"p{index}_success_tray_detect"))
                    # Both successful and missed picks use the Safety Z exit;
                    # only missed/put-back branches continue through Home.
                    targets.extend(replace(t, name=f"p{index}_return_{t.name}") for t in
                                   home_targets(exit_transit.matrix, config.home_matrix,
                                                config.home_joints, **rates))
                    release, retreat = return_targets(plan)
                    targets.extend(replace(t, name=f"p{index}_put_back_{t.name}")
                                   for t in (release, *retreat))
            if request.operation == request.PLACE:
                x, y, rotation = validate_target(request.x_mm, request.y_mm, request.rotation_deg)
                if config.tray is None or config.tray.detect_joints is None:
                    raise ValueError("Load a Tray Teach with a recorded Tray Detect Pose")

                def check_tray_position():
                    self._check_observation()
                    if not np.allclose(self._snapshot().joints, config.tray.detect_joints,
                                       atol=math.radians(1.), rtol=0):
                        raise ValueError("Not at Tray Detect position; tray detection blocked")

                if not np.allclose(sample.joints, config.tray.detect_joints,
                                   atol=math.radians(1.), rtol=0):
                    targets.extend(tray_detect_targets(
                        config.tray.detect_matrix, config.tray.detect_joints,
                        speed_percent=TRAY_SPEED_PERCENT,
                        acceleration_percent=rates["acceleration_percent"]))
                    tray_travel_only = True
                else:
                    check_tray_position()
                    surface = self.trays.request(
                        config, x, y, require_held_item=False, check_state=check_tray_position)
                    check_tray_position()
                    targets.extend(place_targets(config.tray.detect_matrix, surface,
                                                 config.profile, rotation, config.home_matrix))
            config.validate_sources(self.root)
            self._check_observation()
            frames = tuple(f"robot_controller_preview_{target.name}_{index}"
                           for index, target in enumerate(targets, 1))
            with self.state_lock:
                self.wait_for_resume()
                self.configuration, self.origin = config, current.copy()
                self.targets, self.frames = tuple(targets), frames
            response.success = True
            response.message = (f"Published {len(frames)} TF-only planned targets; robot unchanged"
                                if frames else "Already at Home; no motion targets required")
            if request.operation == request.PICK:
                response.message += "; one fresh batch, all candidate/return branches"
                if not batch.candidates:
                    response.message += "; no valid item candidates"
            if tray_travel_only:
                response.message += ("; Tray Detect travel only; placement targets require "
                                     "a fresh tray observation at that pose")
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
            self.request_origin = None
            self.lock.release()
        return response

    def _broadcast(self):
        with self.state_lock:
            config, targets, frames = self.configuration, self.targets, self.frames
            origin = self.origin
        if config is None:
            return
        try:
            config.validate_sources(self.root)
            if not pose_reached(self.kinematics.forward(self._snapshot().joints), origin,
                                translation_m=.005, rotation_deg=1.):
                raise FeedbackFailure("Robot moved; request a new preview")
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
            with self.state_lock:
                if self.configuration is config and not self.cancel.is_set():
                    self.broadcaster.sendTransform(messages)
        except Exception as exc:
            with self.state_lock:
                if self.configuration is config:
                    self._clear()
            self.events.record("WARNING", "preview_cleared", str(exc))

    def close_runtime(self):
        self._clear()
        self.client.close()
        self.trays.close()


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
