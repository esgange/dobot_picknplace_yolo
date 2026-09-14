"""A deliberately non-actuating, explicit-profile first controller stage."""

import json
import math
import os
from pathlib import Path
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from item_perception_interfaces.srv import GetItemPoses

from item_perception_yolo.item_teach_core import load_item_profile, utc_now
from item_perception_yolo.platform_teach_core import (
    _parse_env_file, load_robot_lan1_ip, workspace_root,
)


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


def inspect_profile(path, *, root, robot_ip, publisher_node):
    path = Path(path).expanduser().resolve()
    profile, digest = load_item_profile(path, root=root)
    home = profile["home"]
    return {
        "state": "PROFILE_VALIDATED_NOT_ARMED", "item_teach_file": str(Path(path).resolve()),
        "profile_sha256": digest, "item_name": profile["item"]["name"],
        "model_sha256": profile["model"]["sha256"], "home": home,
        "current_robot_lan1_ip": robot_ip, "current_feedback_publisher": publisher_node,
        "home_identity_policy": "recording_provenance_only",
        "requested_pose_count": profile["retry"]["retry_limit"],
        "maximum_detections": profile["yolo"]["max_detections"],
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
        if selected:
            self._load(selected)
        self.add_on_set_parameters_callback(self._set_parameters)
        self.create_service(Trigger, "/robot_controller/validate_profile", self._validate)
        self.create_service(Trigger, "/robot_controller/request_item_poses", self._request_poses,
                            callback_group=ReentrantCallbackGroup())
        self.create_timer(1.0, self._publish)
        self._publish()
        self.events.record("INFO", "node_started", "Validation-only; no hardware clients")

    def _load(self, path):
        path = Path(path).expanduser().resolve()
        summary = inspect_profile(
            Path(path), root=self.root, robot_ip=self.robot_ip, publisher_node=self.publisher_node,
        )
        self.summary = summary
        self.profile_path = Path(path).resolve()
        self.events.record("INFO", "profile_validated", summary["message"],
                           profile_sha256=summary["profile_sha256"], path=str(self.profile_path))
        self._publish()

    def _set_parameters(self, parameters):
        if (len(parameters) != 1 or parameters[0].name != "item_teach_file"
                or type(parameters[0].value) is not str or not parameters[0].value):
            return SetParametersResult(
                successful=False, reason="Set exactly one non-empty item_teach_file string",
            )
        try:
            self._load(parameters[0].value)
        except (ValueError, OSError) as exc:
            self.events.record("ERROR", "profile_rejected", str(exc))
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True, reason=self.summary["message"])

    def _validate(self, _request, response):
        try:
            if self.profile_path is None:
                raise ValueError("No item profile selected")
            self._load(self.profile_path)
        except (ValueError, OSError) as exc:
            self.summary = {"state": "PROFILE_INVALID", "execution_enabled": False,
                            "inference_enabled": False, "message": str(exc)}
            self.events.record("ERROR", "validation_failed", str(exc))
            self._publish()
            response.success, response.message = False, str(exc)
        else:
            response.success, response.message = True, json.dumps(self.summary, sort_keys=True)
        return response

    def _publish(self):
        self.publisher.publish(String(data=json.dumps(self.summary, sort_keys=True)))

    def _request_poses(self, _request, response):
        if not self.pose_lock.acquire(blocking=False):
            response.success, response.message = False, "A candidate request is already active"
            return response
        try:
            if self.profile_path is None:
                raise ValueError("Select an item profile first")
            path = self.profile_path
            profile, digest = load_item_profile(path, root=self.root)
            if not self.pose_client.service_is_ready():
                raise ValueError(
                    "Detector service unavailable; explicitly enable Armed on detector")
            count = profile["retry"]["retry_limit"]
            request = GetItemPoses.Request(max_candidates=count, profile_sha256=digest)
            future = self.pose_client.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(profile["quality"]["request_timeout_sec"] + 1):
                future.cancel()
                raise ValueError("Detector response timeout; no automatic retry")
            result = future.result()
            if self.profile_path != path or file_profile_digest(path, self.root) != digest:
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
                       "execution_enabled": False, "profile_sha256": digest}
            self.events.record("INFO", "item_candidates_received", result.message, **summary)
            response.success, response.message = True, json.dumps(summary, allow_nan=False)
        except Exception as exc:
            self.events.record("ERROR", "candidate_request_failed", str(exc))
            response.success, response.message = False, str(exc)
        finally:
            self.pose_lock.release()
        return response


def file_profile_digest(path, root):
    return load_item_profile(path, root=root)[1]


def main(args=None):
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required"
        )
    rclpy.init(args=args)
    node = None
    try:
        node = RobotController()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown(timeout_sec=2)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.events.record("INFO", "node_stopped", "Controller stopped; no actuation occurred")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
