"""Shared explicit-action controller with immutable real/TF-only launch mode."""

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
from std_srvs.srv import Trigger
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
        self.debug = self.declare_parameter("debug", True).value
        self.headless = self.declare_parameter("headless", False).value
        if type(self.debug) is not bool or type(self.headless) is not bool:
            raise ValueError("debug/headless must be explicit Boolean mode flags")
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
        self.fatal_error = None
        self.cancel = threading.Event()
        self.shutdown_requested = threading.Event()
        self.action_lock = threading.Lock()
        self.action_thread = None
        self.stop_future = None
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
        self.create_subscription(RobotStatus, "/dobot_msgs_v4/msg/RobotStatus", self._on_status, 10)
        self.create_subscription(String, "/dobot_bringup_ros2/msg/FeedInfo", self._on_feed, 10)
        self.summary = {
            "state": "UNCONFIGURED", "execution_enabled": False,
            "inference_enabled": False, "message": "Select an item profile explicitly.",
        }
        self.profile_path = None
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
                raise ValueError("Headless loads runtime_teach/; explicit file overrides forbidden")
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
        self.create_timer(1.0, self._publish)
        self.create_timer(0.1, self._supervise, callback_group=ReentrantCallbackGroup())
        self.hardware = None
        if not self.debug:
            from .hardware import DobotHardware
            self.hardware = DobotHardware(self)
            self.action_thread = threading.Thread(target=self._initialize, daemon=True)
            self.action_thread.start()
        self._publish()
        self.events.record("INFO", "node_started", "Explicit actions; no automatic home/pick",
                           headless=self.headless, debug=self.debug,
                           cr10_model_sha256=self.kinematics.sha256)

    def _load(self, path):
        path = Path(path).expanduser()
        summary = inspect_profile(
            Path(path), root=self.root, robot_ip=self.robot_ip, publisher_node=self.publisher_node,
            deployment=self.headless,
        )
        self.summary = summary
        self.profile_path = Path(path).resolve()
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
            value.update(debug=self.debug, headless=self.headless,
                         execution_state=self.execution_state,
                         execution_message=self.execution_message,
                         holding_item=self.holding_item,
                         execution_enabled=(not self.debug and self.profile_path is not None
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
            request = GetItemPoses.Request(max_candidates=count, profile_sha256=digest)
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
                                "position_m": [p.x, p.y, p.z], "quaternion": [q.x, q.y, q.z, q.w],
                                "class_id": candidate.class_id, "confidence": candidate.confidence})
            summary = {"batch_id": result.batch_id, "status": result.status,
                       "frame": "platform_reference", "targets": targets,
                       "execution_enabled": False, "profile_sha256": digest,
                       "observation_stamp_ns": stamps[0], "depth_stamp_ns": stamps[1],
                       "evidence": evidence}
            self.events.record("INFO", "item_candidates_received", result.message, **summary)
            response.success, response.message = True, json.dumps(summary, allow_nan=False)
        except Exception as exc:
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
        if self.cancel.is_set() or not rclpy.ok():
            raise RuntimeError("Controller action cancelled; no further motion/I/O")

    def _sole_publisher(self, topic):
        endpoints = self.get_publishers_info_by_topic(topic)
        if (len(endpoints) != 1 or endpoints[0].node_namespace != "/"
                or "/" + endpoints[0].node_name != self.publisher_node):
            raise ValueError(f"{topic} requires sole canonical publisher {self.publisher_node}")

    def check_command_owner(self, service):
        if self.debug:
            raise RuntimeError("Hardware command forbidden in TF-only debug mode")
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
                        "isPauseCmdFlag", "userCoordinate", "toolCoordinate"):
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
        if (enabled and (not status[1] or feed["robot_mode"] not in (5, 7, 8)
                         or feed["ErrorStatus"] or feed["CollisionStates"] or feed["isPauseCmdFlag"]
                         or feed["userCoordinate"] != 0 or feed["toolCoordinate"] != 0)):
            raise ValueError("Robot fault/pause/disabled or nonzero user/tool; commands blocked")
        return {"feed": feed, "enabled": status[1], "sequence": sequence}

    def _initialize(self):
        try:
            self.hardware.initialize()
        except Exception as exc:
            self.set_execution_state("FAILED", str(exc))
            self.fatal_error = f"Controller initialization failed: {exc}"
            self.events.record("FATAL", "initialization_failed", self.fatal_error)

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
        return response

    def start_action(self, action):
        if (self.profile_path is None or action not in ("home", "pick")
                or self.execution_state not in ("DEBUG", "READY", "HOLDING", "NO_PICK")):
            raise ValueError("Load valid teach files and complete initialization before an action")
        if self.action_thread is not None and self.action_thread.is_alive():
            raise ValueError("One controller action is already active")
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

    def _home(self, profile, *, require_suction=False):
        self.check_cancelled()
        home = self.kinematics.forward(profile["home"]["positions_rad"])
        current = (self.kinematics.forward(self.current_joints()) if self.debug
                   else self.hardware.current_pose())
        targets = home_targets(current, home, profile["home"]["positions_rad"],
                               speed_percent=profile["speed"]["travel_percent"],
                               acceleration_percent=profile["acceleration"]["travel_percent"])
        if not self.debug:
            for target in targets:
                self.hardware.move(target, require_suction=require_suction)
        return targets

    def _run_action(self, action):
        try:
            profile, digest = load_item_profile(
                self.profile_path, root=self.root, deployment=self.headless)
            if digest != self.summary["profile_sha256"]:
                raise ValueError("Loaded Item Teach changed; explicitly reload before action")
            home = self.kinematics.forward(profile["home"]["positions_rad"])
            if action == "home":
                if (not self.debug and not self.holding_item and
                        self.hardware.sensor(True, 0, settling_sec=0)):
                    raise ValueError("Unexpected DI1; held-item state unknown, Home blocked")
                targets = self._home(profile, require_suction=self.holding_item)
                self.install_preview(targets, digest)
                self.set_execution_state("DEBUG" if self.debug else
                                         ("HOLDING" if self.holding_item else "READY"),
                                         "Home previewed" if self.debug else "Home completed")
                return
            self.selection.validate(self.root)
            motion = profile["motion"]
            if motion["zheight_offset"] < max(motion["prepick_height"], motion["retract_height"]):
                raise ValueError("zheight_offset must be >= prepick_height and retract_height")
            if not self.pose_client.service_is_ready():
                raise ValueError("Detector must be independently armed before Pick/Home travel")
            self.check_detector_owner()
            if not self.debug and self.hardware.sensor(True, 0, settling_sec=0):
                raise ValueError("DI1 is already active; do not drop/repick a possibly held item")
            home_plan = self._home(profile)
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
            plans = []
            for index, candidate in enumerate(batch["targets"], 1):
                xyz = self.selection.station.platform.base_from_platform @ np.array(
                    [*candidate["position_m"], 1.0])
                plans.append(pick_targets(home, xyz[:3], profile, index))
            if self.debug:
                self.install_preview((*home_plan, *(t for plan in plans for t in plan)), digest)
                self.set_execution_state("DEBUG", f"TF-only pick targets: {len(plans)} candidates")
            else:
                outcome = PickExecutor(self.hardware, finish_home=False).run(
                    plans, profile, check=check,
                    return_home=lambda **kw: self._home(profile, **kw))
                self.holding_item = outcome["holding_item"]
                self.set_execution_state("HOLDING" if outcome["picked"] else "NO_PICK",
                                         "Pick complete; suction stays on at final retract" if
                                         outcome["picked"] else "No item picked; batch exhausted")
        except Exception as exc:
            self.clear_preview()
            if self.hardware is not None and self.hardware.moving:
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

    def _stop_service(self, _request, response):
        future = self.request_stop()
        self.set_execution_state("DEBUG" if self.debug else "FAILED",
                                 "Debug TF cleared" if self.debug else
                                 "Stop requested; hardware confirmation is not assumed")
        response.success = self.debug or future is not None
        response.message = self.execution_message
        return response

    def close_runtime(self):
        if self.hardware is not None and self.hardware.moving:
            self.request_stop()
        else:
            self.cancel.set()
            self.clear_preview()
        if self.action_thread is not None:
            self.action_thread.join(timeout=5)


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
                               debug=node.debug)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for number, handler in previous_handlers.items():
            signal.signal(number, handler)
