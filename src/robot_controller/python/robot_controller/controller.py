"""Shared service-driven controller with explicit TF-only/Live selection."""

from dataclasses import replace
import json
import math
import os
import signal
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from dobot_msgs_v4.srv import SpeedFactor
from item_perception_interfaces.srv import GetItemPoses
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from camera_calibration_gui.calibration_core import rotation_matrix_to_quaternion

from item_perception_yolo.item_teach_core import load_item_profile, utc_now
from item_perception_yolo.platform_teach_core import (
    _parse_env_file, load_robot_lan1_ip, workspace_root,
)
from .kinematics import Cr10Kinematics
from .motion import home_targets, pick_targets, PickExecutor
from .profiles import load_selection, runtime_selection
from .ui_state import load_state


class PackageEventLogger:
    def __init__(self, root):
        self.path = Path(root) / "logs/robot_controller/events.jsonl"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()

    def record(self, level, event, message, **fields):
        payload = {"timestamp_utc": utc_now(), "package": "robot_controller",
                   "node": "robot_controller", "level": level, "event": event,
                   "message": message, **fields}
        with self.lock:
            count = 0
            if self.path.exists():
                with self.path.open(encoding="utf-8") as stream:
                    count = sum(bool(line.strip()) for line in stream)
            with self.path.open("w" if count >= 1000 else "a", encoding="utf-8") as stream:
                stream.write(json.dumps(payload, sort_keys=True) + "\n")


def inspect_profile(path, *, root, robot_ip, publisher_node, deployment=False):
    path = Path(path).expanduser()
    profile, digest = load_item_profile(path, root=root, deployment=deployment)
    home = profile["home"]
    return {
        "state": "PROFILE_VALIDATED", "item_teach_file": str(Path(path).resolve()),
        "profile_sha256": digest, "item_name": profile["item"]["name"],
        "model_sha256": profile["model"]["sha256"], "home": home,
        "current_robot_lan1_ip": robot_ip, "current_feedback_publisher": publisher_node,
        "home_identity_policy": "recording_provenance_only",
        "requested_pose_count": profile["retry"]["pose_candidates"],
        "maximum_detections": profile["yolo"]["max_detections"],
        "speed": profile["speed"],
        "acceleration": profile["acceleration"],
        "execution_enabled": False, "inference_enabled": False,
        "message": "Profile and copied model integrity validated; no model inference or motion.",
    }


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.root = workspace_root()
        self.robot_ip = load_robot_lan1_ip(self.root)
        self.publisher_node = "/" + _parse_env_file(self.root / ".env")["DOBOT_ROBOT_NODE_NAME"]
        self.events = PackageEventLogger(self.root)
        self.state_lock = threading.RLock()
        self.headless = self.declare_parameter("headless", False).value
        if type(self.headless) is not bool:
            raise ValueError("headless must be an explicit Boolean mode flag")
        self.live = False
        self.debug = True
        self.debug_images = False
        self.debug_capture_status = "OFF"
        try:
            self.ui_prefill = (None if self.headless else load_state(
                self.root / "logs/robot_controller/last_session.json"))
        except ValueError as exc:
            self.events.record("FATAL", "invalid_ui_prefill", str(exc))
            raise  # Validate before even constructing a real command transport.
        self.selection = None
        self.execution_state = "DEBUG" if self.debug else "INITIALIZING"
        self.execution_message = "TF-only; no hardware commands" if self.debug else "Starting"
        self.holding_item = False
        self.startup_settings_applied = False
        self.global_speed_percent = None  # Unknown until a successful SpeedFactor response.
        self.global_speed_message = "Live OFF; no global speed command"
        self.fatal_error = None
        self.cancel = threading.Event()
        self.shutdown_requested = threading.Event()
        self.action_lock = threading.Lock()
        self.action_thread = None
        self.stop_thread = None
        self.stop_future = None
        self.stop_recovery_abort = threading.Event()
        self.last_prepick = None
        self.last_gripper = None
        self.joint_feedback = self.feed_feedback = self.robot_feedback = None
        self.feed_sequence = 0
        self.controller_timer = None
        self.controller_progress_at = None
        self.preview_targets = ()
        self.preview_digest = None
        self.preview_sources = ()
        model_path = Path(get_package_share_directory("cra_description")) / "urdf/cr10_robot.xacro"
        self.kinematics = Cr10Kinematics(model_path)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        from dobot_msgs_v4.msg import RobotStatus
        self.create_subscription(
            RobotStatus, "/dobot_msgs_v4/msg/RobotStatus", self._on_status, 10)
        self.create_subscription(String, "/dobot_bringup_ros2/msg/FeedInfo", self._on_feed, 10)
        self.summary = {
            "state": "UNCONFIGURED", "execution_enabled": False,
            "inference_enabled": False, "message": "Select an item profile explicitly.",
        }
        self.profile_path = None
        self.home_reference = None
        self.home_reference_joints = None
        self.pose_lock = threading.Lock()
        self.pose_client = self.create_client(GetItemPoses, "/item_detect/get_item_poses")
        self.publisher = self.create_publisher(
            String, "/robot_controller/status", QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        selected = self.declare_parameter("item_teach_file", "").value
        bin_path = self.declare_parameter("bin_teach_file", "").value
        if self.headless:
            if selected or bin_path:
                raise ValueError(
                    "Headless loads runtime_teach/; explicit file overrides forbidden")
            self.selection = runtime_selection(self.root)
            self._load(self.selection.item_path)
        elif selected:
            self._load(selected)
            if bin_path:
                self.selection = load_selection(selected, bin_path, self.root)
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_service(Trigger, "/robot_controller/validate_profile", self._validate)
        self.create_service(Trigger, "/robot_controller/request_item_poses", self._request_poses,
                            callback_group=ReentrantCallbackGroup())
        for name, action in (("go_home", "home"), ("pick_item", "pick")):
            self.create_service(Trigger, f"/robot_controller/{name}",
                                lambda req, res, a=action: self._start_service(a, res),
                                callback_group=ReentrantCallbackGroup())
        self.create_service(Trigger, "/robot_controller/stop", self._stop_service,
                            callback_group=ReentrantCallbackGroup())
        self.create_service(Trigger, "/robot_controller/enable_robot", self._enable_robot_service,
                            callback_group=ReentrantCallbackGroup())
        self.create_service(SpeedFactor, "/robot_controller/set_global_speed",
                            self._set_global_speed_service,
                            callback_group=ReentrantCallbackGroup())
        self.create_service(SetBool, "/robot_controller/set_live", self._set_live_service,
                            callback_group=ReentrantCallbackGroup())
        self.create_service(SetBool, "/robot_controller/set_debug_images",
                            self._set_debug_images_service,
                            callback_group=ReentrantCallbackGroup())
        self.create_timer(1.0, self._publish)
        self.create_timer(0.1, self._supervise, callback_group=ReentrantCallbackGroup())
        self.hardware = None
        if self.headless:
            startup = self._set_live_service(SetBool.Request(data=True), SimpleNamespace())
            if not startup.success:
                raise RuntimeError(f"Headless Live startup rejected: {startup.message}")
        self._publish()
        self.events.record("INFO", "node_started", "Explicit actions; no automatic home/pick",
                           headless=self.headless, live=self.live,
                           cr10_model_sha256=self.kinematics.sha256)

    def _load(self, path):
        path = Path(path).expanduser()
        summary = inspect_profile(
            Path(path), root=self.root, robot_ip=self.robot_ip, publisher_node=self.publisher_node,
            deployment=self.headless,
        )
        home_joints = tuple(summary["home"]["positions_rad"])
        home_reference = self.kinematics.forward(home_joints)
        self.summary = summary
        self.profile_path = Path(path).resolve()
        self.home_reference = home_reference
        self.home_reference_joints = home_joints
        self.events.record("INFO", "profile_validated", summary["message"],
                           profile_sha256=summary["profile_sha256"], path=str(self.profile_path))
        self._publish()

    def _on_parameters(self, parameters):
        if getattr(self, "headless", False):
            return SetParametersResult(successful=False, reason="Runtime catalog is immutable")
        if (hasattr(self, "action_thread") and self.action_thread is not None
                and self.action_thread.is_alive()):
            return SetParametersResult(successful=False, reason="Controller operation active")
        if (len(parameters) != 1 or parameters[0].name != "item_teach_file"
                or type(parameters[0].value) is not str or not parameters[0].value):
            return SetParametersResult(
                successful=False, reason="Set exactly one non-empty item_teach_file string",
            )
        guard = getattr(self, "action_lock", None)
        if guard is not None and not guard.acquire(blocking=False):
            return SetParametersResult(successful=False, reason="Controller action active")
        try:
            if hasattr(self, "clear_preview"):
                self.clear_preview()
                self.selection = None
            self._load(parameters[0].value)
        except (ValueError, OSError) as exc:
            self.events.record("ERROR", "profile_rejected", str(exc))
            return SetParametersResult(successful=False, reason=str(exc))
        finally:
            if guard is not None:
                guard.release()
        return SetParametersResult(successful=True, reason=self.summary["message"])

    def _validate(self, _request, response):
        guard = getattr(self, "action_lock", None)
        if guard is not None and not guard.acquire(blocking=False):
            response.success, response.message = False, "Controller action active"
            return response
        try:
            if self.profile_path is None:
                raise ValueError("No item profile selected")
            self._load(self.profile_path)
        except (ValueError, OSError) as exc:
            self.summary = {"state": "PROFILE_INVALID", "execution_enabled": False,
                            "inference_enabled": False, "message": str(exc)}
            self.events.record("ERROR", "validation_failed", str(exc))
            if hasattr(self, "clear_preview"):
                self.clear_preview()
                self.selection = None
                self.profile_path = None
                self.home_reference = None
                self.home_reference_joints = None
            self._publish()
            response.success, response.message = False, str(exc)
        else:
            response.success, response.message = True, json.dumps(self.summary, sort_keys=True)
        finally:
            if guard is not None:
                guard.release()
        return response

    def _publish(self):
        value = dict(self.summary)
        if hasattr(self, "execution_state"):
            value.update(live=self.live, headless=self.headless,
                         startup_settings_applied=getattr(self, "startup_settings_applied", False),
                         global_speed_percent=getattr(self, "global_speed_percent", None),
                         global_speed_message=getattr(self, "global_speed_message", ""),
                         debug_images=self.debug_images,
                         debug_capture_status=self.debug_capture_status,
                         execution_state=self.execution_state,
                         execution_message=self.execution_message,
                         holding_item=self.holding_item,
                         execution_enabled=(self.live and self.profile_path is not None
                                            and self.execution_state in
                                            ("READY", "BUSY", "HOLDING", "NO_PICK")),
                         tf_frames=[f"robot_controller_debug_{t.name}"
                                    for t in self.preview_targets])
        self.publisher.publish(String(data=json.dumps(value, sort_keys=True)))

    def _request_poses(self, _request, response):
        if not self.pose_lock.acquire(blocking=False):
            response.success, response.message = False, "A candidate request is already active"
            return response
        try:
            if self.profile_path is None:
                raise ValueError("Select an item profile first")
            path = self.profile_path
            deployment = path.parent == Path(self.root) / "runtime_teach"
            profile, digest = load_item_profile(path, root=self.root, deployment=deployment)
            if not self.pose_client.service_is_ready():
                raise ValueError(
                    "Detector service unavailable; explicitly enable Armed on detector")
            if hasattr(self, "check_detector_owner"):
                self.check_detector_owner()
            count = profile["retry"]["pose_candidates"]
            with self.state_lock:
                save_debug_images = self.debug_images
                if save_debug_images:
                    self.debug_capture_status = "REQUESTED; waiting for annotated image pair"
            request = GetItemPoses.Request(max_candidates=count, profile_sha256=digest,
                                           save_debug_images=save_debug_images)
            future = self.pose_client.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            deadline = time.monotonic() + profile["quality"]["request_timeout_sec"] + 1
            while not done.wait(0.05):
                if getattr(self, "cancel", threading.Event()).is_set():
                    future.cancel()
                    raise ValueError("Detector request cancelled")
                if time.monotonic() >= deadline:
                    future.cancel()
                    raise ValueError("Detector response timeout; no automatic retry")
            result = future.result()
            if self.profile_path != path or file_profile_digest(
                    path, self.root, deployment=deployment) != digest:
                raise ValueError("Controller profile changed during detection")
            if not result.success:
                raise ValueError(f"Detector {result.status}: {result.message}")
            evidence = json.loads(result.diagnostics_json)
            if (result.header.frame_id != "platform_reference"
                    or evidence["profile_sha256"] != digest):
                raise ValueError("Detector frame/profile mismatch")
            capture = evidence.get("debug_capture")
            if (type(capture) is not dict
                    or set(capture) != {"requested", "rgb_path", "depth_path", "error"}
                    or type(capture["requested"]) is not bool
                    or any(type(capture[key]) is not str
                           for key in ("rgb_path", "depth_path", "error"))
                    or capture["requested"] is not save_debug_images):
                raise ValueError("Detector debug-capture diagnostics are malformed")
            paths = (capture["rgb_path"], capture["depth_path"])
            if save_debug_images:
                if bool(capture["error"]) == bool(all(paths)):
                    raise ValueError("Detector debug-capture result is inconsistent")
                if all(paths):
                    directory = (Path(self.root) / "debug/pick_img").resolve()
                    resolved = tuple(Path(path).resolve() for path in paths)
                    if any(path.parent != directory or path.suffix != ".png"
                           for path in resolved):
                        raise ValueError("Detector debug image path escaped debug/pick_img")
                    self.debug_capture_status = "SAVED: " + " | ".join(map(str, resolved))
                else:
                    self.debug_capture_status = "SAVE WARNING: " + capture["error"]
            elif any(paths) or capture["error"]:
                raise ValueError("Detector saved unrequested debug images")
            else:
                self.debug_capture_status = "OFF"
            now_ns = self.get_clock().now().nanoseconds
            stamps = [s.sec * 1_000_000_000 + s.nanosec for s in
                      (result.header.stamp, result.depth_stamp)]
            if any(s <= 0 or not 0 <= (now_ns-s)/1e9 <= profile["quality"]["result_max_age_sec"]
                   for s in stamps):
                raise ValueError("Returned observation is stale or future-dated")
            if abs(stamps[0]-stamps[1])/1e9 > profile["quality"]["sync_tolerance_sec"]:
                raise ValueError("Detector RGB/depth timestamps are not synchronized")
            if len(result.candidates) > count:
                raise ValueError("Detector exceeded requested candidate count")
            ids = set()
            targets = []
            previous_distance = -1
            for i, candidate in enumerate(result.candidates):
                p, q = candidate.pose.position, candidate.pose.orientation
                numbers = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                           candidate.confidence, candidate.center_distance)
                if (candidate.id in ids or candidate.priority != i+1
                        or not candidate.id or not profile["yolo"]["confidence"] <=
                        candidate.confidence <= 1
                        or candidate.center_distance < 0
                        or candidate.class_id not in profile["yolo"]["class_ids"]
                        or not all(math.isfinite(v) for v in numbers)
                        or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w - 1) > 1e-5
                        or candidate.center_distance < previous_distance):
                    raise ValueError("Malformed/duplicate/unordered detector candidate")
                ids.add(candidate.id)
                previous_distance = candidate.center_distance
                targets.append({"id": candidate.id, "priority": candidate.priority,
                                "position_m": [p.x, p.y, p.z],
                                "quaternion": [q.x, q.y, q.z, q.w],
                                "class_id": candidate.class_id,
                                "confidence": candidate.confidence})
            summary = {"batch_id": result.batch_id, "status": result.status,
                       "frame": "platform_reference", "targets": targets,
                       "execution_enabled": False, "profile_sha256": digest,
                       "observation_stamp_ns": stamps[0], "depth_stamp_ns": stamps[1],
                       "evidence": evidence}
            self.events.record("INFO", "item_candidates_received", result.message, **summary)
            response.success, response.message = True, json.dumps(summary, allow_nan=False)
        except Exception as exc:
            if getattr(self, "debug_images", False):
                self.debug_capture_status = f"REQUEST FAILED: {exc}"
            self.events.record("ERROR", "candidate_request_failed", str(exc))
            response.success, response.message = False, str(exc)
        finally:
            self.pose_lock.release()
        return response

    def set_execution_state(self, state, message):
        with self.state_lock:
            if self.cancel.is_set() and state in ("READY", "HOLDING", "NO_PICK", "DEBUG"):
                state, message = ("DEBUG" if self.debug else "FAILED"), "Action cancelled"
            self.execution_state, self.execution_message = state, message
        self.events.record("INFO", "controller_state", message, state=state)
        self._publish()

    def check_cancelled(self):
        if self.cancel.is_set() or self.shutdown_requested.is_set() or not rclpy.ok():
            raise RuntimeError("Controller action cancelled; no further motion/I/O")

    def _sole_publisher(self, topic):
        endpoints = self.get_publishers_info_by_topic(topic)
        if (len(endpoints) != 1 or endpoints[0].node_namespace != "/"
                or "/" + endpoints[0].node_name != self.publisher_node):
            raise ValueError(f"{topic} requires sole canonical publisher {self.publisher_node}")

    def check_command_owner(self, service):
        if not self.live or self.debug:
            raise RuntimeError("Hardware command forbidden while Live is OFF")
        nodes = self.get_node_names_and_namespaces()
        if nodes.count((self.get_name(), self.get_namespace())) != 1:
            raise ValueError("Duplicate controller identity; command authority ambiguous")
        if any(name in ("motion_debug_gui", "gripper_control_gui") for name, _ in nodes):
            raise ValueError("Competing legacy motion/gripper application must be stopped")
        owned = self.get_service_names_and_types_by_node(self.publisher_node[1:], "/")
        if f"/dobot_bringup_ros2/srv/{service}" not in [name for name, _ in owned]:
            raise ValueError("Command service is not provided by canonical Dobot bringup")
        for name, namespace in set(nodes):
            if (name, namespace) == (self.publisher_node[1:], "/"):
                continue
            services = self.get_service_names_and_types_by_node(name, namespace)
            if f"/dobot_bringup_ros2/srv/{service}" in [n for n, _ in services]:
                raise ValueError("Duplicate command-service provider; command authority ambiguous")

    def check_detector_owner(self):
        providers = []
        for name, namespace in self.get_node_names_and_namespaces():
            services = self.get_service_names_and_types_by_node(name, namespace)
            if "/item_detect/get_item_poses" in [service for service, _ in services]:
                providers.append((name, namespace))
        allowed = (("item_detect", "/"), ("item_teach", "/"))
        if len(providers) != 1 or providers[0] not in allowed:
            raise ValueError("Pose service requires exactly one canonical Detect/Teach provider")

    def _on_joints(self, message):
        try:
            names, positions = message.name, message.position
            if len(names) != 6 or set(names) != {f"joint{i}" for i in range(1, 7)}:
                raise ValueError("Actual joints must be exactly joint1 through joint6")
            if len(positions) != 6 or not all(math.isfinite(v) for v in positions):
                raise ValueError("Actual joints require six finite radians")
            values = tuple(float(positions[names.index(f"joint{i}")]) for i in range(1, 7))
            stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
            if stamp <= 0 or not 0 <= message.header.stamp.nanosec < 1_000_000_000:
                raise ValueError("Actual joints require a nonzero source timestamp")
            with self.state_lock:
                self.joint_feedback = values, stamp, time.monotonic()
        except ValueError as exc:
            with self.state_lock:
                self.joint_feedback = None
            self.events.record("WARNING", "invalid_joint_feedback", str(exc))

    def _on_status(self, message):
        with self.state_lock:
            self.robot_feedback = (bool(message.is_connected), bool(message.is_enable),
                                   time.monotonic())

    def _on_feed(self, message):
        try:
            feed = json.loads(message.data)
            for key in ("robot_mode", "digital_input_bits", "digital_outputs", "controller_timer",
                        "isRunQueuedCmd", "RunningStatus", "ErrorStatus", "CollisionStates",
                        "isPauseCmdFlag", "userCoordinate", "toolCoordinate", "EnableStatus"):
                if type(feed[key]) is not int or feed[key] < 0:
                    raise ValueError(f"Canonical FeedInfo {key} must be a nonnegative integer")
            for key in ("tool_vector_actual", "q_actual"):
                values = feed[key]
                finite = all(type(v) in (int, float) and math.isfinite(v) for v in values)
                if len(values) != 6 or not finite:
                    raise ValueError(f"Canonical FeedInfo {key} must have six finite values")
            now = time.monotonic()
            with self.state_lock:
                if self.controller_timer != feed["controller_timer"]:
                    self.controller_timer = feed["controller_timer"]
                    self.controller_progress_at = now
                self.feed_sequence += 1
                self.feed_feedback = feed, now
        except (ValueError, KeyError, TypeError, OverflowError) as exc:
            with self.state_lock:
                self.feed_feedback = None
            self.events.record("WARNING", "invalid_robot_feedback", str(exc))

    def current_joints(self):
        self._sole_publisher("/joint_states")
        with self.state_lock:
            feedback = self.joint_feedback
        if feedback is None:
            raise ValueError("No valid canonical actual joint feedback")
        values, stamp, received = feedback
        if (not 0 <= (self.get_clock().now().nanoseconds-stamp)/1e9 <= 1
                or time.monotonic()-received > 1):
            raise ValueError("Actual joint feedback is stale/future-dated")
        return values

    def feedback_snapshot(self, *, enabled):
        self._sole_publisher("/dobot_msgs_v4/msg/RobotStatus")
        self._sole_publisher("/dobot_bringup_ros2/msg/FeedInfo")
        self.current_joints()
        with self.state_lock:
            status, feedback = self.robot_feedback, self.feed_feedback
            progress, sequence = self.controller_progress_at, self.feed_sequence
        now = time.monotonic()
        if (status is None or feedback is None or not status[0]
                or now-status[2] > 1 or now-feedback[1] > 1
                or progress is None or now-progress > 1):
            raise ValueError("Canonical robot connection/feedback is unavailable or stale")
        feed = feedback[0]
        if enabled:
            blockers = []
            # Vendor isEnable() means robot_mode==5, NOT enabled while moving.
            # Never infer power from mode alone: motion still requires EnableStatus==1.
            if not status[1] and feed["robot_mode"] not in (7, 8):
                blockers.append("RobotStatus.is_enable=False")
            if feed["EnableStatus"] != 1:
                blockers.append(f"EnableStatus={feed['EnableStatus']} (required 1)")
            if feed["robot_mode"] not in (5, 7, 8):
                blockers.append(f"robot_mode={feed['robot_mode']} (expected 5/7/8)")
            for key in ("ErrorStatus", "CollisionStates", "isPauseCmdFlag"):
                if feed[key]:
                    blockers.append(f"{key}={feed[key]}")
            for key in ("userCoordinate", "toolCoordinate"):
                if feed[key] != 0:
                    blockers.append(f"nonzero user/tool: {key}={feed[key]} (required 0)")
            if blockers:
                raise ValueError("Robot readiness blocked: " + "; ".join(blockers))
        return {"feed": feed, "enabled": status[1], "sequence": sequence}

    def _initialize(self):
        try:
            self.hardware.initialize()
        except (ValueError, RuntimeError) as exc:
            self.set_execution_state("FAILED", f"Startup recovery failed: {exc}")
            self.events.record("ERROR", "initialization_failed", str(exc))
        except Exception as exc:
            self.set_execution_state("FAILED", str(exc))
            self.fatal_error = f"Controller initialization failed: {exc}"
            self.events.record("FATAL", "initialization_failed", self.fatal_error)
        finally:
            if self.action_lock.locked():
                self.action_lock.release()

    def _set_live_service(self, request, response):
        requested = bool(request.data)
        if self.headless and not requested:
            response.success = False
            response.message = "Headless controller is permanently Live; stop the node to disarm"
            return response
        if requested == self.live:
            response.success = True
            response.message = f"Live already {'ON' if self.live else 'OFF'}"
            return response
        if self.fatal_error is not None:
            response.success, response.message = False, self.fatal_error
            return response
        if ((self.action_thread is not None and self.action_thread.is_alive())
                or (self.stop_thread is not None and self.stop_thread.is_alive())):
            response.success, response.message = False, "Controller operation active"
            return response
        if self.holding_item:
            response.success, response.message = (
                False, "Cannot turn Live OFF while holding an item")
            return response
        if not self.action_lock.acquire(blocking=False):
            response.success, response.message = False, "Controller action busy"
            return response
        try:
            self.clear_preview()
            self.cancel.clear()
            self.stop_future = None
            if requested:
                from .hardware import DobotHardware
                self.live, self.debug = True, False
                self.startup_settings_applied = False
                self.global_speed_percent = None
                self.global_speed_message = "Initializing SpeedFactor 100%"
                self.execution_state = "INITIALIZING"
                self.execution_message = "Live requested; initializing robot"
                self.hardware = DobotHardware(self)
                self.action_thread = threading.Thread(target=self._initialize, daemon=True)
                self.action_thread.start()
                self.events.record("WARNING", "live_enabled", "Live ON; initialization started")
                response.success = True
                response.message = "Live ON requested; wait for READY"
                self._publish()
                return response
            if self.hardware is not None:
                self.hardware.close()
            self.hardware = None
            self.startup_settings_applied = False
            self.global_speed_percent = None
            self.global_speed_message = "Live OFF; no global speed command"
            self.live, self.debug = False, True
            self.execution_state = "DEBUG"
            self.execution_message = "Live OFF; TF-only previews"
            self.events.record("INFO", "live_disabled", self.execution_message)
            response.success, response.message = True, self.execution_message
            self._publish()
        except Exception as exc:
            if requested:
                self.hardware = None
                self.live, self.debug = False, True
                self.execution_state = "DEBUG"
                self.execution_message = f"Live enable rejected: {exc}"
            response.success, response.message = False, str(exc)
            self.events.record("ERROR", "live_change_failed", str(exc), requested=requested)
            self._publish()
        finally:
            if not requested and self.action_lock.locked():
                self.action_lock.release()
            elif requested and not response.success and self.action_lock.locked():
                self.action_lock.release()
        return response

    def _set_global_speed_service(self, request, response):
        """A bounded setting response, never an automatically resumed motion action."""
        acquired = False
        accepted = False
        try:
            with self.state_lock:
                if type(request.ratio) is not int or not 1 <= request.ratio <= 100:
                    raise ValueError("Global speed must be an integer from 1 through 100")
                if not self.live or self.debug or self.hardware is None:
                    raise ValueError("Global speed requires Live ON")
                if (not self.startup_settings_applied or self.fatal_error is not None
                        or self.shutdown_requested.is_set()):
                    raise ValueError("Global speed requires completed, non-fatal startup")
                if (self.execution_state not in ("READY", "HOLDING", "NO_PICK")
                        or self.hardware.moving or self.cancel.is_set()
                        or (self.action_thread is not None and self.action_thread.is_alive())
                        or (self.stop_thread is not None and self.stop_thread.is_alive())):
                    raise ValueError(
                        "Global speed requires an idle controller; no active recovery")
                if self.stop_future is not None and not self.stop_future.done():
                    raise ValueError("Stop response pending; global speed not sent")
                acquired = self.action_lock.acquire(blocking=False)
                if not acquired:
                    raise ValueError("Controller action busy; global speed not sent")
                self.check_command_owner("SpeedFactor")
                previous = self.execution_state
                accepted = True
                self.global_speed_message = f"SpeedFactor {request.ratio}%: waiting for response"
                self.set_execution_state(
                    "SPEED_SETTING", self.global_speed_message)
            self.hardware.set_global_speed(request.ratio)
            with self.state_lock:
                self.check_cancelled()
                self.global_speed_message = f"Global SpeedFactor set to {request.ratio}%"
                self.set_execution_state(previous, self.global_speed_message)
            response.res = 0  # Actual successful robot response, not asynchronous acceptance.
        except Exception as exc:
            response.res = -1
            self.global_speed_message = (
                f"Global speed request failed: {exc}" if accepted else
                f"Global speed rejected: {exc}")
            if accepted:
                self.cancel.set()
                self.set_execution_state("FAILED", f"SpeedFactor: {exc}")
            self.events.record("ERROR" if accepted else "WARNING", "global_speed_rejected",
                               str(exc), ratio=request.ratio)
        finally:
            if acquired:
                self.action_lock.release()
            self._publish()
        return response

    def _enable_robot_service(self, _request, response):
        with self.state_lock:
            return RobotController._handle_enable_robot_service(self, response)

    def _handle_enable_robot_service(self, response):
        acquired = False
        try:
            if not self.live or self.debug or self.hardware is None:
                raise ValueError("Turn Live ON and complete startup before Enable Robot")
            if self.fatal_error is not None or self.shutdown_requested.is_set():
                raise ValueError("Controller is terminating; Enable Robot is forbidden")
            if not self.startup_settings_applied:
                raise ValueError(
                    "Startup settings are incomplete; Enable Robot cannot bypass startup")
            if self.global_speed_percent is None:
                raise ValueError("Global SpeedFactor is unknown; use Stop / Clear to "
                                 "reinitialize before enabling actions")
            if self.holding_item or self.hardware.moving:
                raise ValueError("Enable Robot requires idle controller with no held item")
            if ((self.action_thread is not None and self.action_thread.is_alive())
                    or (self.stop_thread is not None and self.stop_thread.is_alive())):
                raise ValueError("Controller operation active; Enable Robot not sent")
            acquired = self.action_lock.acquire(blocking=False)
            if not acquired:
                raise ValueError("Controller action busy; Enable Robot not sent")
            self.check_command_owner("EnableRobot")
            snapshot = self.feedback_snapshot(enabled=False)
            feed = snapshot["feed"]
            if (feed["robot_mode"] not in (4, 5) or feed["isRunQueuedCmd"]
                    or feed["RunningStatus"] or feed["ErrorStatus"] or feed["CollisionStates"]
                    or feed["isPauseCmdFlag"] or feed["digital_input_bits"] & 1
                    or feed["userCoordinate"] or feed["toolCoordinate"]):
                raise ValueError("Enable Robot requires fault-free idle feedback and DI1 OFF")
            if self.stop_future is not None and not self.stop_future.done():
                raise ValueError("Stop response pending; Enable Robot not sent")
            self.clear_preview()
            self.cancel.clear()
            self.stop_future = None
            self.set_execution_state("ENABLING", "Explicit Enable Robot accepted")
            self.action_thread = threading.Thread(target=self._run_enable_robot, daemon=True)
            self.action_thread.start()
            acquired = False  # The worker owns release after dispatch/confirmation.
            response.success = True
            response.message = "Enable Robot accepted; watch /robot_controller/status"
        except Exception as exc:
            response.success, response.message = False, str(exc)
            self.events.record("WARNING", "enable_robot_rejected", str(exc))
        finally:
            if acquired:
                self.action_lock.release()
        return response

    def _run_enable_robot(self):
        try:
            self.hardware.enable_robot()
        except Exception as exc:
            self.cancel.set()
            self.set_execution_state("FAILED", f"EnableRobot: {exc}")
            self.events.record("ERROR", "enable_robot_failed", str(exc))
        finally:
            self.action_lock.release()

    def _set_debug_images_service(self, request, response):
        requested = bool(request.data)
        with self.state_lock:
            self.debug_images = requested
            self.debug_capture_status = (
                "ON; next pose request saves one annotated RGB/depth pair"
                if requested else "OFF")
        response.success = True
        response.message = self.debug_capture_status
        self.events.record(
            "INFO", "debug_images_changed", response.message, enabled=requested)
        self._publish()
        return response

    def apply_teach(self, item, bin_path):
        if self.headless or (self.action_thread is not None and self.action_thread.is_alive()):
            raise ValueError("Cannot change teach selection in headless/active controller")
        if not self.action_lock.acquire(blocking=False):
            raise ValueError("Cannot change teach selection during an action")
        try:
            self.clear_preview()
            self.selection = None
            self._load(item)
            if bin_path:
                self.selection = load_selection(item, bin_path, self.root)
                warning = self.selection.warning()
                if warning:
                    self.events.record("WARNING", "bin_platform_mismatch", warning)
            return self.summary
        finally:
            self.action_lock.release()

    def _start_service(self, action, response):
        try:
            self.start_action(action)
            response.success = True
            response.message = "Action accepted; watch /robot_controller/status"
        except Exception as exc:
            response.success, response.message = False, str(exc)
            self.events.record("WARNING", "controller_action_rejected", str(exc), action=action)
        return response

    def start_action(self, action):
        if self.profile_path is None or action not in ("home", "pick"):
            raise ValueError("Load valid Item Teach before Home or Pick")
        permitted = ("READY", "HOLDING", "NO_PICK") if self.live else ("DEBUG",)
        if self.execution_state not in permitted:
            detail = self.execution_message
            if self.live and self.hardware is not None:
                try:
                    self.feedback_snapshot(enabled=True)
                except ValueError as exc:
                    detail = str(exc)
            hint = (" Check whether the emergency stop is pressed; use Stop / Clear "
                    "and wait for READY. No robot motion was sent.") if self.live else ""
            raise ValueError(f"Robot not READY ({self.execution_state}): {detail}.{hint}")
        if self.action_thread is not None and self.action_thread.is_alive():
            raise ValueError("One controller action is already active")
        if self.stop_thread is not None and self.stop_thread.is_alive():
            raise ValueError("Stop/recovery is active")
        if action == "pick" and (self.selection is None or self.holding_item):
            raise ValueError("Pick requires station/bin binding and no item already held")
        if not self.action_lock.acquire(blocking=False):
            raise ValueError("Controller action busy")
        try:
            self.clear_preview()
            self.cancel.clear()
            self.stop_future = None
            self.set_execution_state("BUSY", f"{'TF preview' if self.debug else 'Real'} {action}")
            self.action_thread = threading.Thread(target=self._run_action,
                                                  args=(action,), daemon=True)
            self.action_thread.start()
        except Exception:
            self.action_lock.release()
            raise

    def _loaded_home_reference(self, profile):
        home_joints = tuple(profile["home"]["positions_rad"])
        if (self.home_reference is None or self.home_reference_joints != home_joints):
            raise ValueError("Loaded Home FK reference is unavailable or does not match profile")
        return self.home_reference.copy()

    def home(self, profile, *, require_suction=False, forbid_suction=False, preceding=()):
        self.check_cancelled()
        home = self._loaded_home_reference(profile)
        current = (preceding[-1].matrix if preceding else
                   self.kinematics.forward(self.current_joints()) if self.debug else
                   self.hardware.current_pose())
        targets = home_targets(current, home, profile["home"]["positions_rad"],
                               speed_percent=profile["speed"]["travel_percent"],
                               acceleration_percent=profile["acceleration"]["travel_percent"])
        if not self.debug:
            self.hardware.move_batch((*preceding, *targets), require_suction=require_suction,
                                     forbid_suction=forbid_suction)
        return (*preceding, *targets)

    def _remember_prepick(self, target, gripper):
        with self.state_lock:
            self.last_prepick = target
            self.last_gripper = dict(gripper)

    def pick(self, profile, digest):
        with self.state_lock:
            self.last_prepick = None
            self.last_gripper = None
        self.selection.validate(self.root)
        if not self.pose_client.service_is_ready():
            raise ValueError("Detector must be independently armed before Pick/Home travel")
        self.check_detector_owner()
        if not self.debug and self.hardware.sensor(True, 0, settling_sec=0):
            raise ValueError("DI1 is already active; do not drop/repick a possibly held item")
        home_plan = self.home(profile)
        response = self._request_poses(None, SimpleNamespace())
        if not response.success:
            raise ValueError(response.message)
        batch = json.loads(response.message)
        evidence = batch["evidence"]
        for key, expected in (("camera_sha256", self.selection.station.camera.sha256),
                              ("platform_sha256", self.selection.station.platform.sha256),
                              ("bin_sha256", self.selection.bin.sha256),
                              ("model_sha256", profile["model"]["sha256"])):
            if evidence.get(key) != expected:
                raise ValueError(f"Detector/controller {key} mismatch; no pick")
        stamp = min(batch["observation_stamp_ns"], batch["depth_stamp_ns"])

        def check(index):
            self.check_cancelled()
            self.selection.validate(self.root)
            if (not 0 <= (self.get_clock().now().nanoseconds-stamp)/1e9 <=
                    profile["quality"]["result_max_age_sec"]):
                raise ValueError(f"Candidate {index} expired; explicitly request another pick")

        check(1)
        home = self._loaded_home_reference(profile)
        plans = []
        for index, candidate in enumerate(batch["targets"], 1):
            xyz = self.selection.station.platform.base_from_platform @ np.array(
                [*candidate["position_m"], 1.0])
            plans.append(pick_targets(home, xyz[:3], profile, index))
        if self.debug:
            preview = list(home_plan)
            for index, plan in enumerate(plans, 1):
                preview.extend(plan[:4])
                returned = self.home(profile, preceding=plan[4:])
                preview.extend(replace(target, name=f"p{index}_{target.name}")
                               if target.name.startswith("home") else target
                               for target in returned)
            self.install_preview(preview, digest)
            self.set_execution_state("DEBUG", f"TF-only pick targets: {len(plans)} candidates")
            return
        outcome = PickExecutor(self.hardware, finish_home=True).run(
            plans, profile, check=check,
            return_home=lambda **kw: self.home(profile, **kw),
            remember_prepick=self._remember_prepick)
        self.holding_item = outcome["holding_item"]
        if not outcome["picked"]:
            with self.state_lock:
                self.last_prepick = None
                self.last_gripper = None
        self.set_execution_state("HOLDING" if outcome["picked"] else "NO_PICK",
                                 "Pick complete; item held with suction at Home" if
                                 outcome["picked"] else
                                 "No item picked; batch exhausted and Home completed")

    def _run_action(self, action):
        try:
            profile, digest = load_item_profile(
                self.profile_path, root=self.root, deployment=self.headless)
            if digest != self.summary["profile_sha256"]:
                raise ValueError("Loaded Item Teach changed; explicitly reload before action")
            if action == "home":
                if (not self.debug and not self.holding_item and
                        self.hardware.sensor(True, 0, settling_sec=0)):
                    raise ValueError("Unexpected DI1; held-item state unknown, Home blocked")
                targets = self.home(profile, require_suction=self.holding_item)
                self.install_preview(targets, digest)
                self.set_execution_state("DEBUG" if self.debug else
                                         ("HOLDING" if self.holding_item else "READY"),
                                         "Home previewed" if self.debug else "Home completed")
                return
            self.pick(profile, digest)
        except Exception as exc:
            self.clear_preview()
            if self.hardware is not None and getattr(self.hardware, "moving", False):
                self.request_stop()
            self.set_execution_state("DEBUG" if self.debug else "FAILED", str(exc))
            self.events.record("ERROR", "action_failed", str(exc), action=action)
        finally:
            self.action_lock.release()

    def clear_preview(self):
        with self.state_lock:
            self.preview_targets, self.preview_digest, self.preview_sources = (), None, ()

    def install_preview(self, targets, digest):
        if not self.debug:
            return
        if file_profile_digest(self.profile_path, self.root, deployment=self.headless) != digest:
            raise ValueError("Teach profile changed during TF preview calculation")
        paths = [self.profile_path, self.profile_path.with_suffix(".pt")]
        if self.selection is not None:
            self.selection.validate(self.root)
            paths.extend((self.selection.bin.path, self.selection.station.camera.path,
                          self.selection.station.platform.path))
        sources = tuple((path, source_signature(path)) for path in paths)
        with self.state_lock:
            self.preview_targets, self.preview_digest = tuple(targets), digest
            self.preview_sources = sources
        self.events.record("INFO", "debug_targets", "TF-only targets installed",
                           frames=[f"robot_controller_debug_{t.name}" for t in targets])

    def _supervise(self):
        with self.state_lock:
            targets, sources = self.preview_targets, self.preview_sources
        if targets:
            try:
                if self.cancel.is_set() or any(source_signature(p) != sig for p, sig in sources):
                    raise ValueError("Debug preview cancelled/profile changed")
                transforms = []
                for target in targets:
                    message = TransformStamped()
                    message.header.stamp = self.get_clock().now().to_msg()
                    message.header.frame_id = "base_link"
                    message.child_frame_id = f"robot_controller_debug_{target.name}"
                    p, q = message.transform.translation, message.transform.rotation
                    p.x, p.y, p.z = map(float, target.matrix[:3, 3])
                    q.x, q.y, q.z, q.w = rotation_matrix_to_quaternion(target.matrix[:3, :3])
                    transforms.append(message)
                self.tf_broadcaster.sendTransform(transforms)
            except Exception as exc:
                self.clear_preview()
                self.events.record("WARNING", "debug_targets_cleared", str(exc))
        if self.hardware is not None and self.execution_state in (
                "READY", "BUSY", "HOLDING", "NO_PICK"):
            try:
                self.feedback_snapshot(enabled=True)
                if self.holding_item and not self.feed_feedback[0]["digital_input_bits"] & 1:
                    raise ValueError("Suction lost while holding item")
            except ValueError as exc:
                self.cancel.set()
                self.clear_preview()
                if self.hardware.moving:
                    self.request_stop()
                self.set_execution_state("FAILED", str(exc))

    def request_stop(self):
        self.cancel.set()
        self.clear_preview()
        if self.hardware is not None and self.stop_future is None:
            self.stop_future = self.hardware.stop()
        return self.stop_future

    def stop(self):
        return self.request_stop()

    def _stop_service(self, _request, response):
        with self.state_lock:
            return RobotController._handle_stop_service(self, response)

    def _handle_stop_service(self, response):
        if not self.live:
            self.cancel.set()
            self.clear_preview()
            self.set_execution_state("DEBUG", "TF preview cleared; Live is OFF")
            response.success, response.message = True, self.execution_message
            return response
        if self.stop_thread is not None and self.stop_thread.is_alive():
            with self.state_lock:
                self.stop_recovery_abort.set()
                self.cancel.set()
            future = self.hardware.stop()
            self.stop_future = future
            self.set_execution_state("FAILED", "Stop requested again; return recovery cancelled")
            response.success = future is not None
            response.message = self.execution_message
            return response
        self.stop_recovery_abort.clear()
        active = (self.action_thread if self.action_thread is not None
                  and self.action_thread.is_alive() else None)
        future = self.stop()
        if future is None:
            response.success, response.message = False, "Stop service unavailable"
            return response
        self.set_execution_state("STOPPING", "Stop sent; confirming stationary feedback")
        self.stop_thread = threading.Thread(
            target=self._complete_operator_stop, args=(future, active), daemon=True)
        self.stop_thread.start()
        response.success = True
        response.message = "Stop accepted; watch /robot_controller/status"
        return response

    def _complete_operator_stop(self, future, active_action):
        acquired = False

        def check_recovery_allowed():
            if (self.stop_recovery_abort.is_set() or self.shutdown_requested.is_set()
                    or not rclpy.ok()):
                raise ValueError("Stop recovery cancelled; no further return motion or release")

        try:
            self.hardware.confirm_stop(future, allow_not_ready=True)
            check_recovery_allowed()
            if active_action is not None:
                active_action.join(timeout=5)
                if active_action.is_alive():
                    raise ValueError("Interrupted action did not terminate after Stop")
            acquired = self.action_lock.acquire(timeout=5)
            if not acquired:
                raise ValueError("Controller action did not release after Stop")
            snapshot = self.feedback_snapshot(enabled=False)
            check_recovery_allowed()
            suction = bool(snapshot["feed"]["digital_input_bits"] & 1)
            if not suction:
                if self.holding_item:
                    raise ValueError(
                        "DI1 cleared after Stop; previously held item state is unknown")
                with self.state_lock:
                    check_recovery_allowed()
                    self.cancel.clear()
                    self.stop_future = None
                    self.last_prepick = None
                    self.last_gripper = None
                self.set_execution_state("RECOVERING", "Stop/Clear: recovering idle robot")
                check_recovery_allowed()
                if self.startup_settings_applied and self.global_speed_percent is not None:
                    self.hardware.recover_idle(stop_already_confirmed=True)
                else:
                    # Explicit Stop/Clear restores incomplete or ambiguous startup
                    # settings; unanswered normal responses still forbid overlap.
                    self.hardware.initialize()
                    if self.execution_state != "READY":
                        raise ValueError(self.execution_message)
                check_recovery_allowed()
                if self.profile_path is not None:
                    profile, digest = load_item_profile(
                        self.profile_path, root=self.root, deployment=self.headless)
                    if digest != self.summary["profile_sha256"]:
                        raise ValueError("Item Teach changed; reload before Stop/Clear Home")
                    self.set_execution_state(
                        "RECOVERING", "Stop/Clear: Home from fresh actual Link6 pose")
                    check_recovery_allowed()
                    self.home(profile, forbid_suction=True)
                    check_recovery_allowed()
                    self.set_execution_state("READY", "Stop/Clear confirmed; Home completed")
                else:
                    self.set_execution_state(
                        "READY", "Stop/Clear confirmed; robot re-enabled. "
                        "Load Item Teach for Home recovery")
                return
            self.feedback_snapshot(enabled=True)
            with self.state_lock:
                target, gripper = self.last_prepick, self.last_gripper
            if target is None or gripper is None:
                raise ValueError("DI1 is ON but no validated last pre-pick target exists")
            self.holding_item = True
            with self.state_lock:
                check_recovery_allowed()
                self.cancel.clear()
                self.stop_future = None
            self.set_execution_state("RECOVERING", "Returning held item to last pre-pick pose")
            check_recovery_allowed()
            self.hardware.move(target, require_suction=True)
            check_recovery_allowed()
            self.hardware.output(13, False)
            self.hardware.output(1, True)
            if gripper["use_grip"]:
                self.hardware.output(2, False)
                self.hardware.output(14, True)
            self.holding_item = False
            with self.state_lock:
                self.last_prepick = None
                self.last_gripper = None
            self.set_execution_state("READY", "Stopped; item returned and released at pre-pick")
        except Exception as exc:
            self.cancel.set()
            if self.hardware is not None and getattr(self.hardware, "moving", False):
                self.request_stop()
            self.set_execution_state("FAILED", f"Stop/recovery failed: {exc}")
            self.events.record("ERROR", "stop_recovery_failed", str(exc))
        finally:
            if acquired:
                self.action_lock.release()

    def close_runtime(self):
        with self.state_lock:
            self.stop_recovery_abort.set()
            self.cancel.set()
        if self.hardware is not None and self.hardware.moving:
            self.request_stop()
        else:
            self.cancel.set()
            self.clear_preview()
        if self.action_thread is not None:
            self.action_thread.join(timeout=5)
        if self.stop_thread is not None:
            self.stop_thread.join(timeout=5)
        if self.hardware is not None:
            self.hardware.close()


def file_profile_digest(path, root, *, deployment=False):
    return load_item_profile(path, root=root, deployment=deployment)[1]


def source_signature(path):
    if path.is_symlink():
        raise ValueError("Selected artifact replaced by symlink")
    stat = path.stat()
    return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns


def main(args=None):
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required"
        )
    # Keep DDS alive long enough to request Stop before teardown on Ctrl-C/SIGTERM.
    shutdown_requested = threading.Event()
    previous_handlers = {number: signal.getsignal(number)
                         for number in (signal.SIGINT, signal.SIGTERM)}
    for number in previous_handlers:
        signal.signal(number, lambda _number, _frame: shutdown_requested.set())
    node = None
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = RobotController()
        node.shutdown_requested = shutdown_requested
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        gui_thread = None
        try:
            if node.headless:
                while (rclpy.ok() and node.fatal_error is None
                       and not shutdown_requested.is_set()):
                    executor.spin_once(timeout_sec=0.1)
            else:
                from .gui import run_gui
                gui_thread = run_gui(node, executor)
            if node.fatal_error is not None:
                raise RuntimeError(node.fatal_error)
        finally:
            node.close_runtime()
            executor.shutdown(timeout_sec=2)
            if gui_thread is not None:
                gui_thread.join(timeout=2)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.events.record("INFO", "node_stopped", "Controller stopped; no automatic release",
                               live=node.live)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for number, handler in previous_handlers.items():
            signal.signal(number, handler)
