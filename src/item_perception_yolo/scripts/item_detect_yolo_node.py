#!/usr/bin/env python3
import math
import os
import re
import shutil
import subprocess
import time
import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
warnings.filterwarnings("ignore", message="CUDA initialization:.*", category=UserWarning)

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from dobot_msgs_v4.srv import MovJ
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker


WINDOW_NAME = "item_detect_view"
BIN_CAMERA_COLOR_TOPIC = "/bin_camera/color/image_raw"
BIN_CAMERA_DEPTH_TOPIC = "/bin_camera/depth/image_raw"
BIN_CAMERA_INFO_TOPIC = "/bin_camera/color/camera_info"
BIN_CAMERA_TOPIC_PREFIX = "/bin_camera"
SUPPORTED_YOLO_VERSIONS = ("yolo11", "yolo26")
TOP_BAR_HEIGHT = 206
PREVIEW_CANVAS_WIDTH = 1080
PREVIEW_CANVAS_HEIGHT = 680
BUTTON_HEIGHT = 38
DROPDOWN_ROW_HEIGHT = 34
MAX_DROPDOWN_ROWS = 7
SLIDER_HIT_PADDING = 14
SEEK_WINDOW_MIN_SEC = 1.0
SEEK_WINDOW_MAX_SEC = 60.0
SEEK_DECAY_MIN_SEC = 0.1
SEEK_DECAY_MAX_SEC = 1.0
DETECT_ROI_MARGIN_MIN_PX = 0.0
DETECT_ROI_MARGIN_MAX_PX = 160.0
METERS_TO_MM = 1000.0


def workspace_root() -> Path:
    def looks_like_root(path: Path) -> bool:
        return (
            (path / "src").exists() and
            (
                (path / "README.md").exists()
                or (path / "src" / "dobot_msgs_v4").exists()
            )
        )

    def find_from(start: Path) -> Optional[Path]:
        path = start.expanduser().resolve()
        if path.is_file():
            path = path.parent
        for candidate in (path, *path.parents):
            if looks_like_root(candidate):
                return candidate
        return None

    for name in ("DOBOT_PICKN_PLACE_ROOT", "DOBOT_WORKSPACE_ROOT"):
        value = os.environ.get(name)
        if value:
            return find_from(Path(value)) or Path(value).expanduser().resolve()

    candidates = [Path.cwd(), Path(__file__).resolve()]
    for name in ("COLCON_PREFIX_PATH", "AMENT_PREFIX_PATH"):
        for token in os.environ.get(name, "").split(os.pathsep):
            if not token:
                continue
            prefix = Path(token)
            candidates.append(prefix)
            if "install" in prefix.parts:
                candidates.append(Path(*prefix.parts[:prefix.parts.index("install")]))

    for candidate in candidates:
        found = find_from(candidate)
        if found is not None:
            return found
    return Path.cwd().resolve()


def workspace_path(*parts: str) -> Path:
    return workspace_root().joinpath(*parts)


def normalize_calibration_type(value: object) -> str:
    normalized = []
    for ch in str(value or ""):
        if ch.isalnum():
            normalized.append(ch.lower())
        elif ch in ("_", "-"):
            normalized.append("_")
    return "".join(normalized)


def detect_yolo_version_in_text(value: object) -> Optional[str]:
    text = str(value or "").strip().lower()
    if not text:
        return None
    match = re.search(r"yolo\s*v?[\s_-]*(\d+)", text)
    if match:
        version = f"yolo{match.group(1)}"
        if version in SUPPORTED_YOLO_VERSIONS:
            return version
    for token in re.findall(r"[a-z]+|\d+", text):
        if token in ("26", "11"):
            version = f"yolo{token}"
            if version in SUPPORTED_YOLO_VERSIONS:
                return version
    return None


def detect_yolo_version(*values: object) -> Optional[str]:
    for value in values:
        version = detect_yolo_version_in_text(value)
        if version is not None:
            return version
    return None


def normalize_yolo_version(*values: object) -> str:
    version = detect_yolo_version(*values)
    if version is None:
        raise ValueError("YOLO version marker is required: use yolo11 or yolo26")
    return version


def yolo_version_values_for_model_path(model_path: Path) -> List[str]:
    try:
        resolved = model_path.resolve()
    except Exception:
        resolved = model_path
    values = [resolved.name]
    values.extend(parent.name for parent in resolved.parents if parent.name)
    return values


def yolo_item_token_for_model_path(model_path: Path) -> str:
    stem = model_path.stem.strip().lower()
    match = re.match(r"^(?P<item>.+?)[_-]yolo\s*v?(?:11|26)(?:[_-].*)?$", stem)
    if match:
        return match.group("item").strip("_- ")
    return stem


def yolo_detection_backend(version: object) -> str:
    return f"{normalize_yolo_version(version)}_seg_pt"


def yolo_version_label(version: object) -> str:
    return normalize_yolo_version(version).upper()


def supported_yolo_label() -> str:
    return "/".join(yolo_version_label(version) for version in SUPPORTED_YOLO_VERSIONS)


@dataclass
class Button:
    name: str
    rect: Tuple[int, int, int, int]
    enabled: bool = True


@dataclass
class ItemProfile:
    path: Path
    label: str
    item_name: str = "item"
    class_id: int = 0
    class_name: str = "item"
    associated_bin_name: str = ""
    teach_date: str = ""
    detection_backend: str = ""
    yolo_version: str = ""
    model_path: str = ""
    model_pt_path: str = ""
    model_dir: str = ""
    bin_teach_file: str = ""
    color_topic: str = BIN_CAMERA_COLOR_TOPIC
    depth_topic: str = BIN_CAMERA_DEPTH_TOPIC
    camera_info_topic: str = BIN_CAMERA_INFO_TOPIC
    overlay_topic: str = "bin_overlay"
    roi_points: List[Tuple[float, float]] = field(default_factory=list)
    teach_joints_deg: List[float] = field(default_factory=list)
    has_teach_joints: bool = False


@dataclass
class Detection:
    score: float
    class_id: int
    box: Tuple[int, int, int, int]
    mask: np.ndarray
    center: Tuple[float, float]
    corners: List[Tuple[float, float]]


@dataclass
class Pose3D:
    origin: np.ndarray
    rotation: np.ndarray


@dataclass
class DetectionPose:
    pose: Pose3D
    pick_pixel: Tuple[float, float]


def resolve_path(path_text: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(path_text))).resolve()


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def station_config_value(*keys: str) -> str:
    try:
        values = {}
        with workspace_path("station_config").open("r", encoding="utf-8") as stream:
            for raw_line in stream:
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[len("export "):].strip()
                if "=" not in line:
                    continue
                key, value = line.split("=", 1)
                values[key.strip()] = value.strip().strip("'\"")
    except OSError:
        return ""
    for key in keys:
        value = values.get(key)
        if value:
            return value
    return ""


def resolve_robot_ip_address(value: object = "") -> str:
    requested = str(value or "").strip()
    if requested:
        return requested
    env_ip = os.environ.get("ROBOT_IP_ADDRESS", "").strip()
    if env_ip:
        return env_ip
    return station_config_value("ROBOT_IP_ADDRESS", "ip_address")


def rotate_vector_by_quaternion(vector: np.ndarray, quaternion_xyzw: np.ndarray) -> np.ndarray:
    q = quaternion_xyzw.astype(np.float64)
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12 or not np.isfinite(norm):
        return vector
    q = q / norm
    q_vec = q[:3]
    t = 2.0 * np.cross(q_vec, vector)
    return vector + q[3] * t + np.cross(q_vec, t)


def fit_text(text: str, max_chars: int) -> str:
    return text if len(text) <= max_chars else text[: max(0, max_chars - 3)] + "..."


def rotation_to_quaternion(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(max(1e-12, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(max(1e-12, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(max(1e-12, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 1e-12:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return qx, qy, qz, qw


def pose_to_msg(pose: Pose3D) -> Pose:
    msg = Pose()
    msg.position.x = float(pose.origin[0])
    msg.position.y = float(pose.origin[1])
    msg.position.z = float(pose.origin[2])
    qx, qy, qz, qw = rotation_to_quaternion(pose.rotation)
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw
    return msg


def lower_left_corner_index(corners: List[Tuple[float, float]]) -> int:
    if not corners:
        return -1
    return max(range(len(corners)), key=lambda i: (corners[i][1], -corners[i][0]))


def corner_center(corners: List[Tuple[float, float]]) -> Tuple[float, float]:
    if not corners:
        return 0.0, 0.0
    points = np.asarray(corners, dtype=np.float32)
    center = np.mean(points, axis=0)
    return float(center[0]), float(center[1])


def polygon_signed_area(points: List[Tuple[float, float]]) -> float:
    if len(points) < 3:
        return 0.0
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 2:
        return 0.0
    x = pts[:, 0]
    y = pts[:, 1]
    return float(0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def radial_inset_polygon(
    points: List[Tuple[float, float]],
    margin_px: float,
) -> List[Tuple[float, float]]:
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < 3 or margin_px <= 0.0:
        return [(float(point[0]), float(point[1])) for point in pts]
    center = np.mean(pts, axis=0)
    adjusted = []
    for point in pts:
        vector = center - point
        distance = float(np.linalg.norm(vector))
        if distance <= 1e-6 or not math.isfinite(distance):
            adjusted.append(point)
            continue
        step = min(float(margin_px), distance * 0.85)
        adjusted.append(point + (vector / distance) * step)
    return [(float(point[0]), float(point[1])) for point in adjusted]


def inset_polygon(
    points: List[Tuple[float, float]],
    margin_px: float,
) -> List[Tuple[float, float]]:
    margin = float(max(0.0, margin_px))
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < 3 or margin <= 0.0:
        return [(float(point[0]), float(point[1])) for point in pts]

    area = polygon_signed_area([(float(point[0]), float(point[1])) for point in pts])
    if abs(area) < 1e-6:
        return radial_inset_polygon(points, margin)

    lines = []
    for index in range(len(pts)):
        start = pts[index]
        end = pts[(index + 1) % len(pts)]
        edge = end - start
        length = float(np.linalg.norm(edge))
        if length <= 1e-6 or not math.isfinite(length):
            return radial_inset_polygon(points, margin)
        left_normal = np.array([-edge[1], edge[0]], dtype=np.float64) / length
        inward_normal = left_normal if area > 0.0 else -left_normal
        lines.append((start + inward_normal * margin, edge))

    inset_points = []
    for index in range(len(lines)):
        prev_point, prev_dir = lines[(index - 1) % len(lines)]
        curr_point, curr_dir = lines[index]
        cross = float(prev_dir[0] * curr_dir[1] - prev_dir[1] * curr_dir[0])
        if abs(cross) <= 1e-9 or not math.isfinite(cross):
            return radial_inset_polygon(points, margin)
        delta = curr_point - prev_point
        t = float((delta[0] * curr_dir[1] - delta[1] * curr_dir[0]) / cross)
        inset_points.append(prev_point + t * prev_dir)

    inset_pts = np.asarray(inset_points, dtype=np.float64)
    if not np.isfinite(inset_pts).all():
        return radial_inset_polygon(points, margin)
    inset_area = polygon_signed_area([(float(point[0]), float(point[1])) for point in inset_pts])
    if area * inset_area <= 0.0 or abs(inset_area) < 4.0:
        return radial_inset_polygon(points, margin)

    original_contour = pts[:, :2].astype(np.float32)
    for point in inset_pts:
        if cv2.pointPolygonTest(original_contour, (float(point[0]), float(point[1])), False) < -1.5:
            return radial_inset_polygon(points, margin)
    return [(float(point[0]), float(point[1])) for point in inset_pts]


def normalize(vec: np.ndarray) -> Optional[np.ndarray]:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-9 or not np.isfinite(norm):
        return None
    return vec / norm


class ItemDetectYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("item_detect")
        self.bridge = CvBridge()

        self.profiles_dir = resolve_path(
            self.declare_parameter(
                "profiles_dir", str(workspace_path("teach", "item_teach_yolo"))
            ).value)
        self.model_root = resolve_path(
            self.declare_parameter(
                "model_root", str(workspace_path("teach", "item_teach_yolo"))
            ).value)
        self.selected_model_path: Optional[Path] = None
        self.selected_profile_path: Optional[Path] = None
        self.runtime_settings_path = resolve_path(
            str(self.declare_parameter(
                "runtime_settings_file",
                str(workspace_path("config", "item_perception_yolo", "item_detect_yolo_runtime_settings.yaml")),
            ).value))
        self.selected_model_export_path = resolve_path(
            str(self.declare_parameter(
                "selected_model_export_file",
                str(workspace_path("config", "item_perception_yolo", "item_detect_yolo_selected_model.txt")),
            ).value))
        self.selected_profile_export_path = resolve_path(
            str(self.declare_parameter(
                "selected_profile_export_file",
                str(workspace_path(
                    "config",
                    "item_perception_yolo",
                    "item_detect_yolo_selected_profile.txt",
                )),
            ).value))
        self.startup_error_log_path = resolve_path(
            str(self.declare_parameter(
                "startup_error_log_file",
                str(workspace_path(
                    "config",
                    "item_perception_yolo",
                    "item_detect_yolo_startup_error.yaml",
                )),
            ).value))
        self.selected_profile_topic = str(
            self.declare_parameter("selected_profile_topic", "item_detect/selected_profile").value
        ).strip() or "item_detect/selected_profile"
        self.color_topic = self.normalize_topic(
            self.declare_parameter("color_topic", BIN_CAMERA_COLOR_TOPIC).value,
            BIN_CAMERA_COLOR_TOPIC)
        self.depth_topic = self.normalize_topic(
            self.declare_parameter("depth_topic", BIN_CAMERA_DEPTH_TOPIC).value,
            BIN_CAMERA_DEPTH_TOPIC)
        self.camera_info_topic = self.normalize_topic(
            self.declare_parameter("camera_info_topic", BIN_CAMERA_INFO_TOPIC).value,
            BIN_CAMERA_INFO_TOPIC)
        self.overlay_topic = self.declare_parameter("overlay_topic", "bin_overlay").value
        self.use_profile_camera_topics = as_bool(
            self.declare_parameter("use_profile_camera_topics", True).value)
        self.allowed_camera_topic_prefix = self.normalize_camera_topic_prefix(
            self.declare_parameter("allowed_camera_topic_prefix", BIN_CAMERA_TOPIC_PREFIX).value
        )
        self.seek_pose_topic = self.declare_parameter("bin_pose_topic", "item_seek_pose").value
        self.item_pose_array_topic = self.declare_parameter("bin_item_pose_array_topic", "bin_item_poses").value
        self.item_cube_marker_topic = self.declare_parameter("bin_cube_marker_topic", "bin_cube_marker").value
        self.seek_service_name = self.declare_parameter("seek_service", "item_detect/seek").value
        self.repick_service_name = self.declare_parameter("repick_service", "item_detect/repick").value
        self.seek_complete_service_name = self.declare_parameter(
            "seek_complete_service", "item_detect/seek_complete").value
        self.seek_status_service_name = self.declare_parameter(
            "seek_status_service", "item_detect/seek_status").value
        self.go_to_teach_service_name = self.declare_parameter(
            "go_to_teach_service", "item_detect/go_to_teach").value
        self.movj_service_name = self.declare_parameter("movj_service", "/dobot_bringup_ros2/srv/MovJ").value
        self.camera_frame = str(self.declare_parameter("camera_frame", "").value).strip()
        self.use_calibration = as_bool(self.declare_parameter("use_calibration", True).value)
        self.publish_static_calibration_tf = as_bool(
            self.declare_parameter("publish_static_calibration_tf", True).value)
        self.calibration_parent_frame = str(
            self.declare_parameter("calibration_parent_frame", "base_link").value).strip() or "base_link"
        self.calibration_child_frame = self.declare_parameter(
            "calibration_child_frame", "bin_calibrated_camera_link").value
        self.calibration_file = self.declare_parameter("calibration_file", "").value
        self.platform_calibration_file = str(
            self.declare_parameter("platform_calibration_file", "").value or ""
        ).strip()
        self.platform_calibration_dir = resolve_path(str(
            self.declare_parameter(
                "platform_calibration_dir",
                str(workspace_path("calibration")),
            ).value
        ))
        self.platform_parent_frame = str(
            self.declare_parameter("platform_parent_frame", "base_link").value
        ).strip().strip("/") or "base_link"
        self.platform_frame = str(
            self.declare_parameter("platform_frame", "platform_reference").value
        ).strip().strip("/") or "platform_reference"
        self.platform_translation = (0.0, 0.0, 0.0)
        self.platform_rotation = (0.0, 0.0, 0.0, 1.0)
        self.platform_calibration_loaded = False
        self.robot_ip_address = resolve_robot_ip_address(
            self.declare_parameter("robot_ip_address", "").value
        )
        self.tf_lookup_timeout_sec = float(
            self.declare_parameter("tf_lookup_timeout_sec", 0.1).value
        )
        self.headless = as_bool(self.declare_parameter("headless", False).value)
        self.start_visualization = (
            as_bool(self.declare_parameter("start_visualization", True).value) and
            not self.headless)
        self.publish_overlay = as_bool(self.declare_parameter("publish_overlay", True).value)
        self.yolo_imgsz = int(self.declare_parameter("yolo_imgsz", 640).value)
        self.yolo_conf = float(np.clip(float(self.declare_parameter("yolo_conf", 0.35).value), 0.0, 1.0))
        self.detect_roi_margin_px = float(np.clip(
            float(self.declare_parameter("detect_roi_margin_px", 0.0).value),
            DETECT_ROI_MARGIN_MIN_PX,
            DETECT_ROI_MARGIN_MAX_PX,
        ))
        self.seek_window_sec = float(self.declare_parameter("seek_window_sec", 60.0).value)
        self.seek_decay_sec = float(self.declare_parameter("seek_decay_sec", 1.0).value)
        self.seek_snapshots_dir = resolve_path(str(
            self.declare_parameter(
                "seek_snapshots_dir",
                str(workspace_path("debug files", "seek_frames")),
            ).value
        ))
        default_seek_diag_path = resolve_path(
            os.environ.get(
                "ITEM_SEEK_DIAGNOSTIC_IMAGE_PATH",
                str(
                    resolve_path(
                        os.environ.get(
                            "DETECTION_SNAPSHOT_ROOT",
                            str(workspace_path("debug files")),
                        )
                    ) / "item_seek.png"
                ),
            )
        )
        self.seek_diagnostic_image_enabled = as_bool(
            self.declare_parameter("seek_diagnostic_image_enabled", True).value
        )
        self.seek_diagnostic_image_path = resolve_path(str(
            self.declare_parameter(
                "seek_diagnostic_image_path",
                str(default_seek_diag_path),
            ).value
        ))
        self.yolo_iou = float(self.declare_parameter("yolo_iou", 0.45).value)
        self.mask_threshold = float(self.declare_parameter("mask_threshold", 0.5).value)
        self.max_inference_hz = float(self.declare_parameter("max_inference_hz", 8.0).value)
        requested_pt_device = str(self.declare_parameter("pt_device", "cpu").value).strip()
        self.pt_device = "cpu"
        if requested_pt_device and requested_pt_device.lower() != "cpu":
            self.get_logger().warn(
                f"Ignoring pt_device={requested_pt_device}; .pt item detection is CPU-only."
            )

        self.latest_depth_m: Optional[np.ndarray] = None
        self.latest_info: Optional[CameraInfo] = None
        self.latest_detections: List[Detection] = []
        self.latest_detection_poses: List[Optional[DetectionPose]] = []
        self.selected_detection: Optional[Detection] = None
        self.selected_pose: Optional[Pose3D] = None
        self.peak_pixel: Optional[Tuple[int, int]] = None
        self.last_inference_time = 0.0
        self.inference_count = 0
        self.last_inference_duration_ms = 0.0
        self.inference_duration_ema_ms = 0.0
        self.inference_fps_ema = 0.0
        self.last_camera_render_time = 0.0
        self.status = "Loading YOLO profiles"
        self.yolo_enabled = False
        self.seek_mode_active = False
        self.seek_result_latched = False
        self.seek_auto_yolo_active = False
        self.seek_started_time = 0.0
        self.last_seek_pose: Optional[Pose3D] = None
        self.last_seek_pose_time = 0.0
        self.last_seek_debug_frame: Optional[np.ndarray] = None
        self.last_seek_raw_frame: Optional[np.ndarray] = None
        self.last_seek_overlay_frame: Optional[np.ndarray] = None
        self.last_seek_debug_stamp_ns = 0
        self.last_seek_debug_frame_id = ""
        self.last_seek_debug_score: Optional[float] = None
        self.go_to_teach_in_progress = False
        self.pending_delete_profile: Optional[Path] = None
        self.pending_delete_deadline = 0.0
        self.delete_confirm_active = False
        self.delete_confirm_dialog_rect = (0, 0, 0, 0)
        self.delete_confirm_cancel_rect = (0, 0, 0, 0)
        self.delete_confirm_accept_rect = (0, 0, 0, 0)
        self.debug_images_enabled = False
        self.view_mode = "RGB"
        self.active_slider: Optional[str] = None

        self.buttons: Dict[str, Button] = {}
        self.slider_rects: Dict[str, Tuple[int, int, int, int]] = {}
        self.preview_scale = 1.0
        self.preview_rect = (0, TOP_BAR_HEIGHT, PREVIEW_CANVAS_WIDTH, PREVIEW_CANVAS_HEIGHT)
        self.rendered_window_size = (0, 0)

        self.profiles: List[ItemProfile] = []
        self.selected_profile_index = -1
        self.active_profile: Optional[ItemProfile] = None
        self.active_bin_teach_path: Optional[Path] = None
        self.bin_teach_roi_frame = ""
        self.bin_teach_roi_corners: List[Tuple[float, float, float]] = []
        self.bin_teach_roi_points: List[Tuple[float, float]] = []
        self.pt_model = None
        self.pt_class_names: Dict[int, str] = {}
        self.ignore_yaml_profiles = True
        self.calibration_translation = (0.0, 0.0, 0.0)
        self.calibration_rotation = (0.0, 0.0, 0.0, 1.0)

        camera_topic_error = self.camera_topic_validation_error(
            self.color_topic,
            self.depth_topic,
            self.camera_info_topic,
        )
        if camera_topic_error:
            self.fail_startup(
                camera_topic_error,
                error_kind="invalid_item_camera_topics",
                color_topic=self.color_topic,
                depth_topic=self.depth_topic,
                camera_info_topic=self.camera_info_topic,
                allowed_camera_topic_prefix=self.allowed_camera_topic_prefix,
            )
        if not self.use_calibration:
            self.fail_startup(
                "use_calibration=false is not supported for YOLO item detect; "
                "calibration_file and platform_calibration_file are required.",
                field="use_calibration",
            )
        if not str(self.calibration_file or "").strip():
            self.fail_startup(
                "calibration_file is required for YOLO item detect.",
                field="calibration_file",
            )
        reason = self.load_calibration_from_file(resolve_path(str(self.calibration_file)))
        if reason:
            self.fail_startup(
                f"Failed to load calibration file '{self.calibration_file}': {reason}",
                field="calibration_file",
                path=str(self.calibration_file),
            )
        if self.camera_frame and self.camera_frame != self.calibration_child_frame:
            self.get_logger().warn(
                "camera_frame (%s) differs from calibration_child_frame (%s). "
                "Using calibration_child_frame for YOLO item outputs.",
                self.camera_frame,
                self.calibration_child_frame,
            )
        self.camera_frame = self.calibration_child_frame

        self.load_runtime_ui_settings()

        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, 5)
        self.seek_pose_pub = self.create_publisher(PoseStamped, self.seek_pose_topic, 10)
        self.item_pose_array_pub = self.create_publisher(PoseArray, self.item_pose_array_topic, 10)
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.item_marker_pub = self.create_publisher(Marker, self.item_cube_marker_topic, marker_qos)
        selected_profile_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.selected_profile_pub = self.create_publisher(
            String,
            self.selected_profile_topic,
            selected_profile_qos,
        )
        self.color_sub = None
        self.depth_sub = None
        self.info_sub = None
        self.configure_camera_subscriptions(
            self.color_topic,
            self.depth_topic,
            self.camera_info_topic,
            force=True)
        self.camera_status_timer = self.create_timer(0.5, self.render_no_camera_topics_overlay)
        self.movj_client = self.create_client(MovJ, self.movj_service_name)
        self.create_service(Trigger, self.seek_service_name, self.handle_seek)
        self.create_service(Trigger, self.repick_service_name, self.handle_repick)
        self.create_service(Trigger, self.seek_complete_service_name, self.handle_seek_complete)
        self.create_service(Trigger, self.seek_status_service_name, self.handle_seek_status)
        self.create_service(Trigger, self.go_to_teach_service_name, self.handle_go_to_teach)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_calibration_tf()
        self.load_and_publish_platform_calibration_tf()
        self.refresh_profiles()

        if self.start_visualization:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | getattr(cv2, "WINDOW_GUI_NORMAL", 0))
            self.resize_window_if_needed(
                PREVIEW_CANVAS_WIDTH,
                TOP_BAR_HEIGHT + PREVIEW_CANVAS_HEIGHT)
            cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)

        self.get_logger().info(
            f"item_detect YOLO ready. profiles_dir={self.profiles_dir} "
            f"pose_topic={self.seek_pose_topic} array_topic={self.item_pose_array_topic} "
            f"seek_service={self.seek_service_name} repick_service={self.repick_service_name} "
            f"selected_profile_topic={self.selected_profile_topic} "
            f"seek_diagnostic_image={'on' if self.seek_diagnostic_image_enabled else 'off'} "
            f"seek_diagnostic_path={self.seek_diagnostic_image_path} "
            f"output_frame={self.camera_frame or 'incoming camera frame'} "
            f"calibration_tf={self.calibration_parent_frame}->{self.calibration_child_frame} "
            f"platform_tf={self.platform_parent_frame}->{self.platform_frame} "
            f"allowed_camera_topic_prefix={self.allowed_camera_topic_prefix or '<disabled>'}")

    def normalize_topic(self, value, fallback: str) -> str:
        topic = str(value or "").strip()
        if not topic:
            return fallback
        return topic if topic.startswith("/") else f"/{topic}"

    def normalize_camera_topic_prefix(self, value) -> str:
        prefix = str(value or "").strip().rstrip("/")
        if not prefix:
            return ""
        return prefix if prefix.startswith("/") else f"/{prefix}"

    def camera_topic_validation_error(self, color_topic, depth_topic, camera_info_topic) -> Optional[str]:
        prefix = self.allowed_camera_topic_prefix
        if not prefix:
            return None
        topics = {
            "color": self.normalize_topic(color_topic, ""),
            "depth": self.normalize_topic(depth_topic, ""),
            "camera_info": self.normalize_topic(camera_info_topic, ""),
        }
        invalid = [
            f"{name}={topic or '<empty>'}"
            for name, topic in topics.items()
            if not topic or (topic != prefix and not topic.startswith(prefix + "/"))
        ]
        if not invalid:
            return None
        return (
            "Item detect camera topics must use the bin camera prefix "
            f'"{prefix}"; invalid topics: {", ".join(invalid)}. '
            "For a custom fixed bin camera, set allowed_camera_topic_prefix to that camera namespace."
        )

    def fail_startup(self, reason: str, **details) -> None:
        self.log_configuration_error(reason, **details)
        raise RuntimeError(reason)

    def log_configuration_error(self, reason: str, **details) -> None:
        error_kind = str(details.pop("error_kind", "") or "").strip()
        payload = {
            "node": "item_detect",
            "error": str(reason),
            "details": {key: value for key, value in details.items() if value not in (None, "")},
            "time_unix_sec": time.time(),
        }
        if error_kind:
            payload["error_kind"] = error_kind
        try:
            self.startup_error_log_path.parent.mkdir(parents=True, exist_ok=True)
            self.startup_error_log_path.write_text(
                yaml.safe_dump(payload, sort_keys=False),
                encoding="utf-8",
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to write startup error log: {exc}")
        self.get_logger().error(reason)

    def configure_camera_subscriptions(
        self,
        color_topic,
        depth_topic,
        camera_info_topic,
        force: bool = False,
    ) -> bool:
        next_color_topic = self.normalize_topic(color_topic, self.color_topic)
        next_depth_topic = self.normalize_topic(depth_topic, self.depth_topic)
        next_camera_info_topic = self.normalize_topic(camera_info_topic, self.camera_info_topic)
        validation_error = self.camera_topic_validation_error(
            next_color_topic,
            next_depth_topic,
            next_camera_info_topic,
        )
        if validation_error:
            self.status = validation_error
            self.log_configuration_error(
                validation_error,
                error_kind="invalid_item_camera_topics",
                color_topic=next_color_topic,
                depth_topic=next_depth_topic,
                camera_info_topic=next_camera_info_topic,
                allowed_camera_topic_prefix=self.allowed_camera_topic_prefix,
            )
            return False
        topics_changed = (
            next_color_topic != self.color_topic or
            next_depth_topic != self.depth_topic or
            next_camera_info_topic != self.camera_info_topic)
        if not force and not topics_changed:
            return False

        for attr in ("color_sub", "depth_sub", "info_sub"):
            sub = getattr(self, attr, None)
            if sub is not None:
                self.destroy_subscription(sub)
                setattr(self, attr, None)

        self.color_topic = next_color_topic
        self.depth_topic = next_depth_topic
        self.camera_info_topic = next_camera_info_topic
        self.latest_depth_m = None
        self.latest_info = None
        self.last_camera_render_time = 0.0

        self.color_sub = self.create_subscription(Image, self.color_topic, self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_callback, 10)
        self.get_logger().info(
            "YOLO detect camera topics: "
            f"color={self.color_topic} depth={self.depth_topic} info={self.camera_info_topic}")
        return True

    def refresh_profiles(self) -> None:
        self.profiles = []
        self.latest_detections = []
        self.latest_detection_poses = []
        self.selected_detection = None
        self.selected_pose = None
        self.peak_pixel = None
        self.active_profile = None
        self.selected_profile_index = -1
        self.pt_model = None
        self.pt_class_names = {}
        self.selected_model_path = None
        self.selected_profile_path = None
        self.clear_active_bin_teach_roi()
        if self.profiles_dir.exists():
            model_paths_seen = set()
            if not self.ignore_yaml_profiles:
                for path in self.profile_yaml_paths():
                    profile = self.load_profile(path)
                    if profile is not None:
                        self.profiles.append(profile)
                        try:
                            model_paths_seen.add(Path(profile.model_path).resolve())
                        except Exception:
                            pass
            for path in self.profile_pt_paths():
                try:
                    resolved = path.resolve()
                except Exception:
                    resolved = path
                if resolved in model_paths_seen:
                    continue
                profile = self.profile_from_model_path(path)
                if profile is not None:
                    self.profiles.append(profile)
                    model_paths_seen.add(resolved)
        if not self.profiles:
            self.active_profile = None
            self.selected_profile_index = -1
            self.pt_model = None
            self.pt_class_names = {}
            self.selected_model_path = None
            self.selected_profile_path = None
            self.status = f"No paired {supported_yolo_label()} .pt + YAML profiles in {self.profiles_dir}"
            self.save_selected_model_export_file()
            self.save_selected_profile_export_file()
            self.publish_selected_profile()
            return
        self.status = (
            f"Open Model to load a {supported_yolo_label()} .pt profile and its paired bin teach ROI"
        )
        self.save_selected_model_export_file()
        self.save_selected_profile_export_file()
        self.publish_selected_profile()

    def profile_yaml_paths(self) -> List[Path]:
        patterns = ("*.yaml", "*/*.yaml", "profiles/*.yaml", "models/*/*.yaml")
        paths: List[Path] = []
        seen = set()
        for pattern in patterns:
            for path in self.profiles_dir.glob(pattern):
                if not path.is_file():
                    continue
                resolved = path.resolve()
                if resolved in seen:
                    continue
                seen.add(resolved)
                paths.append(path)
        return sorted(paths, key=lambda p: p.stat().st_mtime, reverse=True)

    def profile_pt_paths(self) -> List[Path]:
        patterns = ("*.pt", "*/*.pt", "models/*/*.pt", "models/*/weights/*.pt")
        paths: List[Path] = []
        seen = set()
        for pattern in patterns:
            for path in self.profiles_dir.glob(pattern):
                if not path.is_file():
                    continue
                resolved = path.resolve()
                if resolved in seen:
                    continue
                seen.add(resolved)
                paths.append(path)
        return sorted(paths, key=lambda p: p.stat().st_mtime, reverse=True)

    def set_active_bin_teach_roi(
        self,
        path: Path,
        frame: str,
        corners: List[Tuple[float, float, float]],
    ) -> None:
        self.active_bin_teach_path = path.resolve()
        self.bin_teach_roi_frame = str(frame or "").strip().strip("/")
        self.bin_teach_roi_corners = list(corners)
        self.bin_teach_roi_points = []

    def clear_active_bin_teach_roi(self) -> None:
        self.active_bin_teach_path = None
        self.bin_teach_roi_frame = ""
        self.bin_teach_roi_corners = []
        self.bin_teach_roi_points = []

    def profile_params_from_root(self, root: Dict) -> Optional[Dict]:
        if not isinstance(root, dict):
            return None
        for key in ("item_detect", "item_yolo", "teach"):
            node = root.get(key, {})
            if not isinstance(node, dict):
                continue
            candidate = node.get("ros__parameters")
            if isinstance(candidate, dict):
                return candidate
        return None

    def resolve_profile_model_path(self, profile_path: Path, configured_path: str) -> Optional[Path]:
        text = str(configured_path or "").strip()
        if not text:
            return None
        configured = Path(text).expanduser()
        if configured.suffix.lower() != ".pt":
            return None
        if configured.is_absolute():
            return configured.resolve()
        return (profile_path.parent / configured).resolve()

    def resolve_profile_bin_teach_path(self, profile_path: Path, configured_path: str) -> Optional[Path]:
        text = str(configured_path or "").strip()
        if not text:
            return None
        configured = Path(text).expanduser()
        if configured.is_absolute():
            return configured.resolve()
        return (profile_path.parent / configured).resolve()

    def load_profile_bin_teach_roi(self, profile: ItemProfile) -> bool:
        bin_teach_path = self.resolve_profile_bin_teach_path(profile.path, profile.bin_teach_file)
        if bin_teach_path is None:
            self.status = f"Profile {profile.path.name} missing required bin_teach_file"
            self.log_configuration_error(
                self.status,
                error_kind="missing_bin_teach_file",
                profile_file=str(profile.path),
            )
            return False
        if not bin_teach_path.exists() or not bin_teach_path.is_file() or bin_teach_path.stat().st_size <= 0:
            self.status = f"Profile {profile.path.name} bin_teach_file is missing: {bin_teach_path}"
            self.log_configuration_error(
                self.status,
                error_kind="missing_bin_teach_file",
                profile_file=str(profile.path),
                bin_teach_file=str(bin_teach_path),
            )
            return False
        try:
            frame, corners = self.load_bin_teach_platform_roi(bin_teach_path)
        except Exception as exc:
            self.status = f"Profile {profile.path.name} bin_teach_file is not readable: {bin_teach_path.name}"
            self.log_configuration_error(
                f"{self.status}: {exc}",
                error_kind="invalid_bin_teach_file",
                profile_file=str(profile.path),
                bin_teach_file=str(bin_teach_path),
            )
            return False
        if not frame or len(corners) != 4:
            self.status = (
                f"Profile {profile.path.name} bin_teach_file has no usable "
                f"platform_roi_corners: {bin_teach_path.name}"
            )
            self.log_configuration_error(
                self.status,
                error_kind="invalid_bin_teach_file",
                profile_file=str(profile.path),
                bin_teach_file=str(bin_teach_path),
            )
            return False
        self.set_active_bin_teach_roi(bin_teach_path, frame, corners)
        profile.bin_teach_file = str(bin_teach_path)
        self.get_logger().info(f"Loaded profile bin teach ROI: {self.active_bin_teach_path}")
        return True

    def load_profile(
        self,
        path: Path,
        override_model_path: Optional[Path] = None,
    ) -> Optional[ItemProfile]:
        try:
            root = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            params = self.profile_params_from_root(root)
            if not isinstance(params, dict):
                return None
            if override_model_path is not None:
                model_path_resolved = override_model_path.resolve()
            else:
                model_path = (
                    params.get("model_pt_path") or
                    params.get("trained_model_path") or
                    params.get("model_path") or
                    ""
                )
                model_path_resolved = self.resolve_profile_model_path(path, str(model_path))
                if model_path_resolved is None:
                    return None
                if model_path_resolved.suffix.lower() != ".pt":
                    return None
            raw_detection_backend = str(params.get("detection_backend", "")).strip()
            try:
                yolo_version = normalize_yolo_version(*yolo_version_values_for_model_path(model_path_resolved))
            except ValueError:
                self.status = (
                    f"Profile {path.name} model path missing YOLO version; "
                    "the .pt filename/path must contain yolo11 or yolo26"
                )
                self.get_logger().error(self.status)
                return None
            backend_version = detect_yolo_version(raw_detection_backend)
            metadata_version = detect_yolo_version(
                params.get("yolo_version", ""),
                params.get("model_family", ""),
            )
            if metadata_version and metadata_version != yolo_version:
                self.status = (
                    f"Profile {path.name} YOLO metadata mismatch: "
                    f"model path is {yolo_version}, YAML says {metadata_version}"
                )
                self.get_logger().error(self.status)
                return None
            if raw_detection_backend and backend_version != yolo_version:
                self.status = (
                    f"Profile {path.name} detection_backend mismatch: "
                    f"model path is {yolo_version}, backend is {raw_detection_backend}"
                )
                self.get_logger().error(self.status)
                return None
            detection_backend = raw_detection_backend or yolo_detection_backend(yolo_version)
            roi_points: List[Tuple[float, float]] = []
            item_node = root.get("item", {}) if isinstance(root, dict) else {}
            bin_teach_file = str(params.get("bin_teach_file", "")).strip()
            bin_teach_path = self.resolve_profile_bin_teach_path(path, bin_teach_file)
            if bin_teach_path is None:
                self.status = f"Profile {path.name} missing required bin_teach_file"
                self.log_configuration_error(
                    self.status,
                    error_kind="missing_bin_teach_file",
                    profile_file=str(path),
                )
                return None
            if not bin_teach_path.exists() or not bin_teach_path.is_file() or bin_teach_path.stat().st_size <= 0:
                self.status = f"Profile {path.name} bin_teach_file is missing: {bin_teach_path}"
                self.log_configuration_error(
                    self.status,
                    error_kind="missing_bin_teach_file",
                    profile_file=str(path),
                    bin_teach_file=str(bin_teach_path),
                )
                return None
            try:
                bin_frame, bin_corners = self.load_bin_teach_platform_roi(bin_teach_path)
            except Exception as exc:
                self.status = f"Profile {path.name} bin_teach_file is not readable: {bin_teach_path.name}"
                self.log_configuration_error(
                    f"{self.status}: {exc}",
                    error_kind="invalid_bin_teach_file",
                    profile_file=str(path),
                    bin_teach_file=str(bin_teach_path),
                )
                return None
            if not bin_frame or len(bin_corners) != 4:
                self.status = (
                    f"Profile {path.name} bin_teach_file has no usable platform_roi_corners: "
                    f"{bin_teach_path.name}"
                )
                self.log_configuration_error(
                    self.status,
                    error_kind="invalid_bin_teach_file",
                    profile_file=str(path),
                    bin_teach_file=str(bin_teach_path),
                )
                return None
            item_name = str(
                params.get("item_name", "") or
                params.get("class_name", "") or
                (item_node.get("display_name", "") if isinstance(item_node, dict) else "") or
                path.stem
            )
            bin_name = str(params.get("associated_bin_name", ""))
            teach_date = str(params.get("teach_date", ""))
            label = item_name
            if bin_name:
                label += f" @ {bin_name}"
            if teach_date:
                label += f" | {teach_date}"
            else:
                label += f" | {path.name}"
            teach_joints = self.parse_teach_joints(params.get("teach_joints_deg", []))
            return ItemProfile(
                path=path,
                label=label,
                item_name=item_name,
                class_id=int(params.get("class_id", 0)),
                class_name=str(params.get("class_name", item_name)),
                associated_bin_name=bin_name,
                teach_date=teach_date,
                detection_backend=detection_backend,
                yolo_version=yolo_version,
                model_path=str(model_path_resolved),
                model_pt_path=str(model_path_resolved),
                model_dir=str(resolve_path(str(params.get("model_dir", "")))) if params.get("model_dir") else "",
                bin_teach_file=str(bin_teach_path),
                color_topic=str(params.get("color_topic", self.color_topic)),
                depth_topic=str(params.get("depth_topic", self.depth_topic)),
                camera_info_topic=str(params.get("camera_info_topic", self.camera_info_topic)),
                overlay_topic=str(params.get("overlay_topic", self.overlay_topic)),
                roi_points=roi_points,
                teach_joints_deg=teach_joints,
                has_teach_joints=as_bool(params.get("has_teach_joints", False)) or len(teach_joints) >= 6,
            )
        except Exception as exc:
            self.get_logger().warn(f"Skipping YOLO profile {path}: {exc}")
            return None

    def model_path_for_runtime(self, path: Path) -> Optional[Path]:
        candidate = path.resolve()
        if candidate.is_dir():
            for name in ("best.pt", "model.pt"):
                model_path = candidate / name
                if model_path.exists():
                    return model_path.resolve()
            pt_files = sorted(candidate.glob("*.pt"), key=lambda p: p.stat().st_mtime, reverse=True)
            if pt_files:
                return pt_files[0].resolve()
            weights_pt_files = sorted(
                candidate.glob("weights/*.pt"),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
            return weights_pt_files[0].resolve() if weights_pt_files else None
        if candidate.suffix.lower() == ".pt":
            return candidate
        return None

    def metadata_yaml_for_model(self, model_path: Path) -> Optional[Path]:
        search_dir = model_path if model_path.is_dir() else model_path.parent
        model_token = model_path.stem.lower()
        item_token = yolo_item_token_for_model_path(model_path)
        candidates = [
            search_dir / f"{model_path.stem}.yaml",
        ]
        if item_token:
            candidates.extend([
                search_dir / f"{item_token}.yaml",
                search_dir / f"item_{item_token}.yaml",
            ])
        seen = set()
        expected_model_path = model_path.resolve()
        for candidate in candidates:
            resolved = candidate.resolve()
            if resolved in seen or not candidate.exists() or not candidate.is_file():
                continue
            seen.add(resolved)
            try:
                root = yaml.safe_load(candidate.read_text(encoding="utf-8")) or {}
                params = self.profile_params_from_root(root)
                if not isinstance(params, dict):
                    continue
                configured_model = (
                    params.get("model_pt_path") or
                    params.get("trained_model_path") or
                    params.get("model_path") or
                    ""
                )
                referenced_model_path = self.resolve_profile_model_path(
                    candidate,
                    str(configured_model),
                )
                if referenced_model_path is None or referenced_model_path.resolve() != expected_model_path:
                    continue
            except Exception:
                continue
            profile = self.load_profile(candidate, override_model_path=model_path)
            if profile is None:
                continue
            profile_item_token = profile.item_name.lower().replace(" ", "_")
            candidate_token = candidate.stem.lower()
            if (
                candidate_token == model_token
                or (item_token and candidate_token == item_token)
                or (item_token and candidate_token == f"item_{item_token}")
                or (profile_item_token and item_token and profile_item_token == item_token)
            ):
                try:
                    if Path(profile.model_path).resolve() == expected_model_path:
                        return candidate
                except Exception:
                    continue
        return None

    def profile_from_model_path(self, path: Path) -> Optional[ItemProfile]:
        if path.suffix.lower() in (".yaml", ".yml"):
            return self.load_profile(path)
        model_path = self.model_path_for_runtime(path)
        if model_path is None or not model_path.exists():
            self.status = f"Open Model: no .pt model found for {path}"
            return None
        metadata_path = self.metadata_yaml_for_model(model_path)
        if metadata_path is not None:
            profile = self.load_profile(metadata_path, override_model_path=model_path)
            if profile is not None:
                self.get_logger().info(
                    f"Using YOLO PT profile YAML {metadata_path} for model {model_path}"
                )
                profile.model_path = str(model_path)
                try:
                    profile_model_dir = resolve_path(profile.model_dir) if profile.model_dir else None
                except Exception:
                    profile_model_dir = None
                if profile_model_dir != model_path.parent.resolve():
                    profile.model_dir = ""
                profile.label = profile.label or model_path.parent.name
                return profile

        self.status = (
            f"Open Model: {model_path.name} has no paired YAML metadata; "
            "bin_teach_file is required"
        )
        self.log_configuration_error(
            self.status,
            error_kind="missing_yolo_yaml_pair",
            model_path=str(model_path),
            profiles_dir=str(self.profiles_dir),
        )
        return None

    def select_model_path(self, path: Path) -> bool:
        profile = self.profile_from_model_path(path)
        if profile is None:
            return False
        for index, existing in enumerate(self.profiles):
            try:
                if Path(existing.model_path).resolve() == Path(profile.model_path).resolve():
                    self.profiles[index] = profile
                    return self.select_profile(index)
            except Exception:
                pass
        self.profiles.insert(0, profile)
        return self.select_profile(0)

    def open_model_dialog(self) -> Optional[Path]:
        start_dir = self.model_root if self.model_root.exists() else self.profiles_dir
        filename_arg = str(start_dir)
        if filename_arg and not filename_arg.endswith("/"):
            filename_arg += "/"
        yolo_label = supported_yolo_label()
        command = (
            "if command -v zenity >/dev/null 2>&1; then "
            f"zenity --file-selection --title='Open {yolo_label} PT Model' "
            f"--filename={self.shell_quote(filename_arg)} "
            f"--file-filter='{yolo_label} PT models | *.pt' "
            "--file-filter='All files | *'; "
            "elif command -v kdialog >/dev/null 2>&1; then "
            f"kdialog --title 'Open {yolo_label} PT Model' --getopenfilename "
            f"{self.shell_quote(str(start_dir))} '{yolo_label} PT models (*.pt)'; "
            "fi 2>/dev/null"
        )
        try:
            result = subprocess.run(
                command,
                shell=True,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                timeout=300,
            )
        except Exception as exc:
            self.status = f"Open Model failed: {exc}"
            return None
        selected = result.stdout.strip()
        if not selected:
            return None
        return resolve_path(selected)

    def request_open_model(self) -> None:
        self.status = f"Open Model: select {supported_yolo_label()} .pt model"
        selected_path = self.open_model_dialog()
        if selected_path is None:
            self.status = "Open Model cancelled"
            return
        if self.select_model_path(selected_path):
            self.save_runtime_ui_settings()
            self.save_selected_model_export_file()
            self.status = f"Loaded {self.profile_label()}"

    @staticmethod
    def shell_quote(text: str) -> str:
        return "'" + text.replace("'", "'\"'\"'") + "'"

    def parse_points(self, value) -> List[Tuple[float, float]]:
        if not isinstance(value, list):
            return []
        points: List[Tuple[float, float]] = []
        if value and not isinstance(value[0], list):
            for i in range(0, len(value) - 1, 2):
                points.append((float(value[i]), float(value[i + 1])))
        else:
            for point in value:
                if isinstance(point, list) and len(point) >= 2:
                    points.append((float(point[0]), float(point[1])))
        return points

    def load_bin_teach_platform_roi(
        self,
        bin_teach_path: Path,
    ) -> Tuple[str, List[Tuple[float, float, float]]]:
        root = yaml.safe_load(bin_teach_path.read_text(encoding="utf-8")) or {}
        if not isinstance(root, dict):
            return "", []
        bin_node = root.get("bin_teach", {})
        if not isinstance(bin_node, dict):
            return "", []
        return self.parse_platform_roi_corners(bin_node)

    def parse_platform_roi_corners(
        self,
        params: Dict,
    ) -> Tuple[str, List[Tuple[float, float, float]]]:
        root = params.get("platform_roi_corners", {})
        if not isinstance(root, dict):
            return "", []
        frame = str(root.get("coordinate_frame", "")).strip().strip("/")
        points_node = root.get("points", [])
        if not frame or not isinstance(points_node, list):
            return "", []
        points: List[Tuple[float, float, float]] = []
        for point_node in points_node:
            if not isinstance(point_node, dict):
                continue
            position = point_node.get("position", {})
            if not isinstance(position, dict):
                continue
            try:
                point = (
                    float(position.get("x", 0.0)),
                    float(position.get("y", 0.0)),
                    float(position.get("z", 0.0)),
                )
            except (TypeError, ValueError):
                continue
            if np.isfinite(point).all():
                points.append(point)
        return (frame, points) if len(points) == 4 else ("", [])

    def parse_teach_joints(self, value) -> List[float]:
        if not isinstance(value, list) or len(value) < 6:
            return []
        try:
            return [float(v) for v in value[:6]]
        except (TypeError, ValueError):
            return []

    def select_profile(self, index: int) -> bool:
        if index < 0 or index >= len(self.profiles):
            return False
        profile = self.profiles[index]
        model_path = Path(profile.model_path)
        if not model_path.exists():
            self.status = f"Model missing: {model_path}"
            return False
        try:
            model_yolo_version = normalize_yolo_version(*yolo_version_values_for_model_path(model_path))
        except ValueError:
            self.status = (
                f"Model {model_path.name} missing YOLO version; "
                "filename/path must contain yolo11 or yolo26"
            )
            self.get_logger().error(self.status)
            return False
        metadata_version = detect_yolo_version(profile.yolo_version)
        if metadata_version and metadata_version != model_yolo_version:
            self.status = (
                f"Model {model_path.name} YOLO version mismatch: "
                f"filename/path is {model_yolo_version}, profile is {metadata_version}"
            )
            self.get_logger().error(self.status)
            return False
        profile.yolo_version = model_yolo_version
        backend_version = detect_yolo_version(profile.detection_backend)
        if backend_version and backend_version != profile.yolo_version:
            self.status = (
                f"Model {model_path.name} detection_backend mismatch: "
                f"filename/path is {profile.yolo_version}, backend is {profile.detection_backend}"
            )
            self.get_logger().error(self.status)
            return False
        if not backend_version:
            profile.detection_backend = yolo_detection_backend(profile.yolo_version)
        profile.roi_points = []
        if not self.load_profile_bin_teach_roi(profile):
            return False
        if self.use_profile_camera_topics:
            validation_error = self.camera_topic_validation_error(
                profile.color_topic,
                profile.depth_topic,
                profile.camera_info_topic,
            )
            if validation_error:
                self.status = f"Profile {profile.path.name} has invalid camera topics"
                self.log_configuration_error(
                    validation_error,
                    error_kind="invalid_profile_camera_topics",
                    profile_file=str(profile.path),
                    color_topic=profile.color_topic,
                    depth_topic=profile.depth_topic,
                    camera_info_topic=profile.camera_info_topic,
                    allowed_camera_topic_prefix=self.allowed_camera_topic_prefix,
                )
                return False
        self.selected_profile_index = index
        self.active_profile = profile
        if self.use_profile_camera_topics:
            self.configure_camera_subscriptions(
                profile.color_topic,
                profile.depth_topic,
                profile.camera_info_topic)
        if not self.load_pt_model(model_path):
            self.selected_model_path = model_path
            self.pending_delete_profile = None
            self.pending_delete_deadline = 0.0
            self.delete_confirm_active = False
            self.save_selected_model_export_file()
            self.save_selected_profile_export_file()
            self.publish_selected_profile()
            return False
        self.selected_model_path = model_path
        self.status = f"Loaded {self.profile_label()}"
        self.pending_delete_profile = None
        self.pending_delete_deadline = 0.0
        self.delete_confirm_active = False
        self.save_selected_model_export_file()
        self.save_selected_profile_export_file()
        self.publish_selected_profile()
        return True

    def load_pt_model(self, model_path: Path) -> bool:
        os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
        self.pt_model = None
        self.pt_class_names = {}
        self.inference_count = 0
        self.last_inference_duration_ms = 0.0
        self.inference_duration_ema_ms = 0.0
        self.inference_fps_ema = 0.0
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.status = f"{supported_yolo_label()} .pt unavailable: install ultralytics for PT weights"
            self.get_logger().error(f"{self.status}: {exc}")
            return False

        try:
            self.pt_model = YOLO(str(model_path))
        except Exception as exc:
            yolo_label = (
                yolo_version_label(self.active_profile.yolo_version)
                if self.active_profile is not None
                else supported_yolo_label()
            )
            self.status = f"Failed to load {yolo_label} .pt model: {model_path.name}"
            self.get_logger().error(f"{self.status}: {exc}")
            return False
        names = getattr(self.pt_model, "names", None)
        if isinstance(names, dict):
            self.pt_class_names = {int(key): str(value) for key, value in names.items()}
        elif isinstance(names, (list, tuple)):
            self.pt_class_names = {index: str(value) for index, value in enumerate(names)}
        else:
            self.pt_class_names = {}
        if self.active_profile is not None and self.pt_class_names:
            class_name = self.pt_class_names.get(self.active_profile.class_id)
            if class_name:
                self.active_profile.class_name = class_name
        yolo_label = (
            yolo_version_label(self.active_profile.yolo_version)
            if self.active_profile is not None
            else supported_yolo_label()
        )
        self.get_logger().info(f"Loaded {yolo_label} PT model: {model_path}")
        return True

    def load_calibration_from_file(self, path: Path) -> str:
        try:
            if not path.exists():
                return "File does not exist"
            if path.stat().st_size <= 0:
                return "Calibration file is empty"
            root = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except Exception as exc:
            return f"Could not read YAML: {exc}"
        if not isinstance(root, dict):
            return "Calibration YAML is not a map"

        transform = root.get("transform", {})
        if not isinstance(transform, dict):
            return "Missing 'transform' key"
        translation = transform.get("translation", {})
        rotation = transform.get("rotation", {})
        if not isinstance(translation, dict) or not isinstance(rotation, dict):
            return "Missing rotation/translation keys"

        params = root.get("parameters", {})
        if not isinstance(params, dict):
            params = {}
        calibration_type = str(params.get("calibration_type", "")).strip()
        if normalize_calibration_type(calibration_type) != "eye_on_base":
            return (
                "Expected eye-to-hand calibration YAML with "
                "parameters.calibration_type=eye_on_base, got "
                f"'{calibration_type or '<missing>'}'"
            )

        parent_frame = str(
            params.get("robot_base_frame") or
            params.get("transform_parent_frame") or
            ""
        ).strip()
        if parent_frame and parent_frame != self.calibration_parent_frame:
            self.get_logger().warn(
                "Calibration YAML parent frame is %s but YOLO detect was configured with %s. "
                "Using YAML parent frame so eye-to-hand TF matches camera_calibration.",
                parent_frame,
                self.calibration_parent_frame,
            )
            self.calibration_parent_frame = parent_frame

        child_frame = str(params.get("transform_child_frame") or "").strip()
        if child_frame and child_frame != self.calibration_child_frame:
            self.get_logger().warn(
                "Calibration YAML child frame is %s but YOLO detect was configured with %s. "
                "Using YAML child frame so eye-to-hand TF matches camera_calibration.",
                child_frame,
                self.calibration_child_frame,
            )
            self.calibration_child_frame = child_frame

        try:
            qx = float(rotation.get("x", 0.0))
            qy = float(rotation.get("y", 0.0))
            qz = float(rotation.get("z", 0.0))
            qw = float(rotation.get("w", 1.0))
            tx = float(translation.get("x", 0.0))
            ty = float(translation.get("y", 0.0))
            tz = float(translation.get("z", 0.0))
        except Exception as exc:
            return f"Failed to parse rotation/translation: {exc}"

        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-9 or not math.isfinite(norm):
            return "Invalid quaternion (zero norm)"
        inv_norm = 1.0 / norm
        self.calibration_translation = (tx, ty, tz)
        self.calibration_rotation = (qx * inv_norm, qy * inv_norm, qz * inv_norm, qw * inv_norm)
        return ""

    def publish_calibration_tf(self) -> None:
        if not self.use_calibration or not self.publish_static_calibration_tf or not self.calibration_file:
            return
        try:
            tx, ty, tz = self.calibration_translation
            qx, qy, qz, qw = self.calibration_rotation
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.calibration_parent_frame
            msg.child_frame_id = self.calibration_child_frame
            msg.transform.translation.x = tx
            msg.transform.translation.y = ty
            msg.transform.translation.z = tz
            msg.transform.rotation.x = qx
            msg.transform.rotation.y = qy
            msg.transform.rotation.z = qz
            msg.transform.rotation.w = qw
            self.static_tf_broadcaster.sendTransform(msg)
        except Exception as exc:
            self.get_logger().warn(f"Calibration TF publish failed: {exc}")

    def load_platform_calibration_from_file(self, path: Path) -> str:
        try:
            if not path.exists():
                return "File does not exist"
            if path.stat().st_size <= 0:
                return "Platform calibration file is empty"
            root = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except Exception as exc:
            return f"Could not read YAML: {exc}"
        if not isinstance(root, dict):
            return "Platform calibration YAML is not a map"

        metadata = root.get("metadata", {})
        if not isinstance(metadata, dict):
            metadata = {}
        calibration_type = str(metadata.get("calibration_type", "")).strip()
        if normalize_calibration_type(calibration_type) != "platform_reference":
            return (
                "Expected platform calibration YAML with "
                "metadata.calibration_type=platform_reference, got "
                f"'{calibration_type or '<missing>'}'"
            )

        transform = root.get("transform", {})
        if not isinstance(transform, dict):
            return "Missing 'transform' key"
        translation = transform.get("translation", {})
        rotation = transform.get("rotation", {})
        if not isinstance(translation, dict) or not isinstance(rotation, dict):
            return "Missing platform rotation/translation keys"

        try:
            qx = float(rotation.get("x", 0.0))
            qy = float(rotation.get("y", 0.0))
            qz = float(rotation.get("z", 0.0))
            qw = float(rotation.get("w", 1.0))
            tx = float(translation.get("x", 0.0))
            ty = float(translation.get("y", 0.0))
            tz = float(translation.get("z", 0.0))
        except Exception as exc:
            return f"Failed to parse platform rotation/translation: {exc}"

        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-9 or not math.isfinite(norm):
            return "Invalid platform quaternion (zero norm)"
        if not np.isfinite([tx, ty, tz, qx, qy, qz, qw]).all():
            return "Platform transform has non-finite value"

        parent_frame = str(
            metadata.get("transform_parent_frame") or self.platform_parent_frame
        ).strip().strip("/")
        child_frame = str(
            metadata.get("transform_child_frame") or self.platform_frame
        ).strip().strip("/")
        if not parent_frame or not child_frame:
            return "Missing platform transform parent/child frame"

        inv_norm = 1.0 / norm
        self.platform_parent_frame = parent_frame
        self.platform_frame = child_frame
        self.platform_translation = (tx, ty, tz)
        self.platform_rotation = (qx * inv_norm, qy * inv_norm, qz * inv_norm, qw * inv_norm)
        self.platform_calibration_file = str(path)
        self.platform_calibration_loaded = True
        return ""

    def load_and_publish_platform_calibration_tf(self) -> None:
        if not self.platform_calibration_file:
            self.fail_startup(
                "platform_calibration_file is required for YOLO item detect.",
                field="platform_calibration_file",
            )
        path = resolve_path(self.platform_calibration_file)
        reason = self.load_platform_calibration_from_file(path)
        if reason:
            self.fail_startup(
                f"Failed to load platform calibration file '{path}': {reason}",
                field="platform_calibration_file",
                path=str(path),
            )
        self.publish_platform_calibration_tf()
        self.get_logger().info(
            f"Loaded platform calibration {self.platform_parent_frame}->{self.platform_frame}: {path}"
        )

    def publish_platform_calibration_tf(self) -> None:
        if not self.platform_calibration_loaded or not self.publish_static_calibration_tf:
            return
        try:
            tx, ty, tz = self.platform_translation
            qx, qy, qz, qw = self.platform_rotation
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.platform_parent_frame
            msg.child_frame_id = self.platform_frame
            msg.transform.translation.x = tx
            msg.transform.translation.y = ty
            msg.transform.translation.z = tz
            msg.transform.rotation.x = qx
            msg.transform.rotation.y = qy
            msg.transform.rotation.z = qz
            msg.transform.rotation.w = qw
            self.static_tf_broadcaster.sendTransform(msg)
        except Exception as exc:
            self.get_logger().warn(f"Platform calibration TF publish failed: {exc}")

    def resolved_camera_frame_id(self, header, info: Optional[CameraInfo]) -> str:
        if self.camera_frame:
            return self.camera_frame
        if getattr(header, "frame_id", ""):
            return str(header.frame_id)
        if info is not None and info.header.frame_id:
            return str(info.header.frame_id)
        return "camera_color_optical_frame"

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}")
            return
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)
        depth_m[~np.isfinite(depth_m)] = np.nan
        depth_m[depth_m <= 0.0] = np.nan
        self.latest_depth_m = depth_m

    def info_callback(self, msg: CameraInfo) -> None:
        self.latest_info = msg

    def color_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Color conversion failed: {exc}")
            return
        depth_m = self.latest_depth_m
        info = self.latest_info
        if depth_m is not None and depth_m.shape[:2] != frame.shape[:2]:
            depth_m = cv2.resize(depth_m, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
        stamp = msg.header.stamp
        frame_id = self.resolved_camera_frame_id(msg.header, info)

        now = time.monotonic()
        min_period = 1.0 / max(0.1, self.max_inference_hz)
        if now - self.last_inference_time >= min_period:
            self.last_inference_time = now
            self.process_frame(frame, depth_m, info)

        base_view = self.display_frame(frame, depth_m)
        needs_overlay = self.publish_overlay or self.seek_diagnostic_image_enabled or self.debug_images_enabled
        output = self.render_overlay(base_view, depth_m) if needs_overlay else base_view.copy()
        if self.publish_overlay:
            overlay_msg = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")
            overlay_msg.header.stamp = stamp
            overlay_msg.header.frame_id = frame_id
            self.overlay_pub.publish(overlay_msg)
        self.publish_pose_outputs(stamp, frame_id, frame, output)
        if self.start_visualization:
            view = self.build_ui(output)
            cv2.imshow(WINDOW_NAME, view)
            cv2.waitKey(1)
        self.last_camera_render_time = time.monotonic()

    def process_frame(self, frame: np.ndarray, depth_m: Optional[np.ndarray], info: Optional[CameraInfo]) -> None:
        profile = self.active_profile
        self.selected_detection = None
        self.selected_pose = None
        self.peak_pixel = None
        self.latest_detection_poses = []
        bin_roi_ready = self.update_bin_teach_roi_points(info) if info is not None else False
        if profile is None or self.pt_model is None:
            self.latest_detections = []
            self.latest_detection_poses = []
            return
        if info is None:
            self.latest_detections = []
            self.latest_detection_poses = []
            self.status = "Waiting for camera info for bin teach ROI projection"
            return
        if not bin_roi_ready:
            self.latest_detections = []
            self.latest_detection_poses = []
            source = (
                self.active_bin_teach_path.name
                if self.active_bin_teach_path is not None
                else "bin teach"
            )
            self.status = f"Waiting for bin teach ROI projection from {source}"
            return
        profile.roi_points = list(self.bin_teach_roi_points)
        if not self.yolo_enabled:
            self.latest_detections = []
            self.latest_detection_poses = []
            if not (self.status or "").startswith("YOLO OFF"):
                self.status = "YOLO OFF | live inference paused"
            return
        roi = self.roi_crop(frame, profile.roi_points)
        if roi is None:
            self.latest_detections = []
            self.latest_detection_poses = []
            self.status = "Profile ROI is outside image"
            return
        crop, rect, roi_mask = roi
        detections = self.run_yolo(crop, roi_mask, rect)
        self.latest_detections = detections
        if not detections:
            self.latest_detection_poses = []
            self.status = f"No {profile.class_name} mask above {self.yolo_conf * 100.0:.0f}%"
            return

        selected_index = max(range(len(detections)), key=lambda i: detections[i].score)
        selected = detections[selected_index]
        self.selected_detection = selected

        detection_peaks: List[Optional[Tuple[int, int]]] = [None] * len(detections)
        if depth_m is not None:
            for index, detection in enumerate(detections):
                mask = self.mask_for_frame(detection.mask, depth_m.shape[:2])
                detection_peaks[index] = self.find_highest_peak(depth_m, mask)
            self.peak_pixel = detection_peaks[selected_index]

        if depth_m is not None and info is not None:
            for index, detection in enumerate(detections):
                self.latest_detection_poses.append(
                    self.estimate_detection_pose(detection, depth_m, info))
            selected_detection_pose = self.latest_detection_poses[selected_index]
            if selected_detection_pose is not None:
                self.selected_pose = selected_detection_pose.pose
        else:
            self.latest_detection_poses = [None] * len(detections)

        pose_count = sum(1 for detection_pose in self.latest_detection_poses if detection_pose is not None)
        self.status = (
            f"Detected {len(detections)} {profile.class_name} mask(s) | "
            f"best confidence {selected.score * 100.0:.0f}% | poses {pose_count}"
        )

    def update_bin_teach_roi_points(self, info: Optional[CameraInfo]) -> bool:
        if info is None:
            self.bin_teach_roi_points = []
            return False
        if not self.bin_teach_roi_frame or len(self.bin_teach_roi_corners) != 4:
            self.bin_teach_roi_points = []
            return False
        projected_geometry = self.project_platform_geometry(
            self.bin_teach_roi_frame,
            self.bin_teach_roi_corners,
            info,
        )
        if projected_geometry is None:
            self.bin_teach_roi_points = []
            return False
        self.bin_teach_roi_points = self.detect_roi_points(projected_geometry)
        return len(self.bin_teach_roi_points) >= 3

    def project_platform_geometry(
        self,
        platform_roi_frame: str,
        platform_roi_corners: List[Tuple[float, float, float]],
        info: CameraInfo,
    ) -> Optional[List[Tuple[float, float]]]:
        platform_roi_frame = str(platform_roi_frame or "").strip().strip("/")
        if not platform_roi_frame or len(platform_roi_corners) != 4:
            return None
        camera_frame = (
            self.camera_frame or
            str(getattr(info.header, "frame_id", "") or "")
        ).strip().strip("/")
        if not camera_frame:
            return None
        if info.k[0] <= 1e-6 or info.k[4] <= 1e-6:
            return None

        if camera_frame == platform_roi_frame:
            translation = np.zeros(3, dtype=np.float64)
            quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        else:
            try:
                transform = self.tf_buffer.lookup_transform(
                    camera_frame,
                    platform_roi_frame,
                    Time(),
                    timeout=Duration(seconds=max(0.01, self.tf_lookup_timeout_sec)),
                )
            except Exception:
                return None
            translation = np.array([
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                float(transform.transform.translation.z),
            ], dtype=np.float64)
            quaternion = np.array([
                float(transform.transform.rotation.x),
                float(transform.transform.rotation.y),
                float(transform.transform.rotation.z),
                float(transform.transform.rotation.w),
            ], dtype=np.float64)

        projected: List[Tuple[float, float]] = []
        for point in platform_roi_corners:
            source_point = np.array(point, dtype=np.float64)
            camera_point = translation + rotate_vector_by_quaternion(source_point, quaternion)
            if not np.isfinite(camera_point).all() or camera_point[2] <= 1e-6:
                return None
            u = (camera_point[0] * float(info.k[0]) / camera_point[2]) + float(info.k[2])
            v = (camera_point[1] * float(info.k[4]) / camera_point[2]) + float(info.k[5])
            if not np.isfinite([u, v]).all():
                return None
            projected.append((float(u), float(v)))
        return projected

    def detect_roi_points(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        return inset_polygon(points, self.detect_roi_margin_px)

    def roi_crop(
        self,
        frame: np.ndarray,
        points: List[Tuple[float, float]],
    ) -> Optional[Tuple[np.ndarray, Tuple[int, int, int, int], np.ndarray]]:
        h, w = frame.shape[:2]
        if len(points) < 3:
            return None
        pts = np.asarray(points, dtype=np.float32)
        pts[:, 0] = np.clip(pts[:, 0], 0, max(0, w - 1))
        pts[:, 1] = np.clip(pts[:, 1], 0, max(0, h - 1))
        x0 = int(np.floor(np.min(pts[:, 0])))
        y0 = int(np.floor(np.min(pts[:, 1])))
        x1 = int(np.ceil(np.max(pts[:, 0])))
        y1 = int(np.ceil(np.max(pts[:, 1])))
        if x1 <= x0 or y1 <= y0:
            return None
        crop = frame[y0:y1 + 1, x0:x1 + 1].copy()
        rel = np.round(pts - np.array([x0, y0], dtype=np.float32)).astype(np.int32)
        mask = np.zeros(crop.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [rel], 255)
        crop[mask == 0] = 0
        return crop, (x0, y0, crop.shape[1], crop.shape[0]), mask

    def run_yolo(
        self,
        crop: np.ndarray,
        roi_mask: np.ndarray,
        rect: Tuple[int, int, int, int],
    ) -> List[Detection]:
        if self.pt_model is None or self.active_profile is None:
            return []
        os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
        classes = [int(self.active_profile.class_id)]
        inference_started = time.perf_counter()
        try:
            results = self.pt_model.predict(
                source=crop,
                imgsz=self.yolo_imgsz,
                conf=self.yolo_conf,
                iou=self.yolo_iou,
                classes=classes,
                device=self.pt_device,
                verbose=False,
            )
        finally:
            self.record_inference_timing(time.perf_counter() - inference_started)
        if not results:
            return []
        return self.pt_result_to_detections(results[0], crop.shape[:2], roi_mask, rect)

    def record_inference_timing(self, elapsed_sec: float) -> None:
        elapsed_sec = max(1e-6, float(elapsed_sec))
        duration_ms = elapsed_sec * 1000.0
        fps = 1.0 / elapsed_sec
        alpha = 0.2
        if self.inference_count <= 0:
            self.inference_duration_ema_ms = duration_ms
            self.inference_fps_ema = fps
        else:
            self.inference_duration_ema_ms = (
                alpha * duration_ms + (1.0 - alpha) * self.inference_duration_ema_ms
            )
            self.inference_fps_ema = alpha * fps + (1.0 - alpha) * self.inference_fps_ema
        self.last_inference_duration_ms = duration_ms
        self.inference_count += 1

    def inference_speed_text(self) -> str:
        if self.active_profile is None or self.pt_model is None:
            return "Inference: no model"
        if not self.yolo_enabled:
            return "Inference: YOLO OFF"
        if self.inference_count <= 0:
            return "Inference: waiting"
        return (
            f"Inference: {self.inference_duration_ema_ms:.0f} ms | "
            f"{self.inference_fps_ema:.1f} FPS"
        )

    def pt_result_to_detections(
        self,
        result,
        crop_shape: Tuple[int, int],
        roi_mask: np.ndarray,
        rect: Tuple[int, int, int, int],
    ) -> List[Detection]:
        boxes_obj = getattr(result, "boxes", None)
        if boxes_obj is None or len(boxes_obj) == 0:
            return []
        boxes_xyxy = boxes_obj.xyxy.detach().cpu().numpy()
        scores = boxes_obj.conf.detach().cpu().numpy()
        class_ids = boxes_obj.cls.detach().cpu().numpy().astype(np.int32)
        masks_obj = getattr(result, "masks", None)
        mask_polygons = getattr(masks_obj, "xy", None) if masks_obj is not None else None
        mask_data = None
        if masks_obj is not None and getattr(masks_obj, "data", None) is not None:
            mask_data = masks_obj.data.detach().cpu().numpy()

        detections: List[Detection] = []
        for index, box_xyxy in enumerate(boxes_xyxy):
            class_id = int(class_ids[index])
            if class_id != int(self.active_profile.class_id):
                continue
            x1, y1, x2, y2 = self.clip_xyxy_box(box_xyxy, crop_shape)
            if x2 <= x1 or y2 <= y1:
                continue
            crop_mask = self.pt_mask_for_detection(
                index,
                mask_polygons,
                mask_data,
                crop_shape,
                roi_mask,
                (x1, y1, x2, y2),
            )
            if cv2.countNonZero(crop_mask) < 16:
                continue
            full_mask = crop_mask
            contour = self.largest_contour(full_mask)
            if contour is None:
                continue
            corners = self.contour_corners(contour, rect[0], rect[1])
            if len(corners) != 4:
                continue
            moments = cv2.moments(contour)
            if abs(moments["m00"]) > 1e-6:
                cx = float(moments["m10"] / moments["m00"]) + rect[0]
                cy = float(moments["m01"] / moments["m00"]) + rect[1]
            else:
                cx = float(x1 + (x2 - x1) * 0.5 + rect[0])
                cy = float(y1 + (y2 - y1) * 0.5 + rect[1])
            detections.append(Detection(
                score=float(scores[index]),
                class_id=class_id,
                box=(x1 + rect[0], y1 + rect[1], x2 - x1, y2 - y1),
                mask=self.place_crop_mask(full_mask, rect),
                center=(cx, cy),
                corners=corners,
            ))
        detections.sort(key=lambda detection: detection.score, reverse=True)
        return detections

    def clip_xyxy_box(
        self,
        box_xyxy: np.ndarray,
        image_shape: Tuple[int, int],
    ) -> Tuple[int, int, int, int]:
        height, width = image_shape
        x1, y1, x2, y2 = [float(value) for value in box_xyxy[:4]]
        x1 = int(np.clip(round(x1), 0, width - 1))
        y1 = int(np.clip(round(y1), 0, height - 1))
        x2 = int(np.clip(round(x2), 0, width - 1))
        y2 = int(np.clip(round(y2), 0, height - 1))
        return x1, y1, x2, y2

    def pt_mask_for_detection(
        self,
        index: int,
        mask_polygons,
        mask_data,
        crop_shape: Tuple[int, int],
        roi_mask: np.ndarray,
        box_xyxy: Tuple[int, int, int, int],
    ) -> np.ndarray:
        mask = np.zeros(crop_shape, dtype=np.uint8)
        if mask_polygons is not None and index < len(mask_polygons):
            polygon = np.asarray(mask_polygons[index], dtype=np.float32)
            if polygon.ndim == 2 and len(polygon) >= 3:
                points = np.round(polygon).astype(np.int32)
                points[:, 0] = np.clip(points[:, 0], 0, crop_shape[1] - 1)
                points[:, 1] = np.clip(points[:, 1], 0, crop_shape[0] - 1)
                cv2.fillPoly(mask, [points], 255)

        if cv2.countNonZero(mask) < 16 and mask_data is not None and index < len(mask_data):
            raw_mask = mask_data[index].astype(np.float32)
            if raw_mask.shape[:2] != crop_shape:
                raw_mask = cv2.resize(
                    raw_mask,
                    (crop_shape[1], crop_shape[0]),
                    interpolation=cv2.INTER_LINEAR,
                )
            mask = (raw_mask >= self.mask_threshold).astype(np.uint8) * 255

        if cv2.countNonZero(mask) < 16:
            x1, y1, x2, y2 = box_xyxy
            mask[y1:y2, x1:x2] = 255
        return cv2.bitwise_and(mask, roi_mask)

    def place_crop_mask(self, crop_mask: np.ndarray, rect: Tuple[int, int, int, int]) -> np.ndarray:
        x0, y0, w, h = rect
        full_mask = np.zeros((max(1, y0 + h), max(1, x0 + w)), dtype=np.uint8)
        full_mask[y0:y0 + h, x0:x0 + w] = crop_mask
        return full_mask

    def mask_for_frame(self, mask: np.ndarray, shape: Tuple[int, int]) -> np.ndarray:
        output = np.zeros(shape, dtype=np.uint8)
        h = min(shape[0], mask.shape[0])
        w = min(shape[1], mask.shape[1])
        output[:h, :w] = mask[:h, :w]
        return output

    def largest_contour(self, mask: np.ndarray):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= 16.0]
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)

    def contour_corners(self, contour, offset_x: int, offset_y: int) -> List[Tuple[float, float]]:
        hull = cv2.convexHull(contour)
        if hull is None or len(hull) < 3:
            return []
        rect = cv2.minAreaRect(hull)
        if rect[1][0] < 2.0 or rect[1][1] < 2.0:
            return []
        corners = cv2.boxPoints(rect)
        return [(float(x + offset_x), float(y + offset_y)) for x, y in corners]

    def select_detection_by_confidence(
        self,
        detections: List[Detection],
        depth_m: Optional[np.ndarray],
    ) -> Optional[Detection]:
        if not detections:
            return None
        selected = max(detections, key=lambda d: d.score)
        if depth_m is None:
            self.peak_pixel = None
            return selected
        selected_mask = self.mask_for_frame(selected.mask, depth_m.shape[:2])
        self.peak_pixel = self.find_highest_peak(depth_m, selected_mask)
        return selected

    def find_highest_peak(
        self,
        depth_m: np.ndarray,
        candidate_mask: np.ndarray,
    ) -> Optional[Tuple[int, int]]:
        if cv2.countNonZero(candidate_mask) == 0:
            return None
        ys, xs = np.where(candidate_mask > 0)
        if ys.size == 0:
            return None
        values = depth_m[ys, xs]
        valid = np.isfinite(values) & (values > 0.0)
        if not np.any(valid):
            return None
        xs = xs[valid]
        ys = ys[valid]
        values = values[valid]
        idx = int(np.argmin(values))
        return int(xs[idx]), int(ys[idx])

    def estimate_detection_pose(
        self,
        detection: Detection,
        depth_m: np.ndarray,
        info: CameraInfo,
    ) -> Optional[DetectionPose]:
        pose = self.estimate_pose(detection, depth_m, info)
        if pose is None:
            return None
        pick_pixel = self.camera_point_to_pixel(pose.origin, info, depth_m.shape[:2])
        if pick_pixel is None:
            return None
        return DetectionPose(pose=pose, pick_pixel=pick_pixel)

    def estimate_pose(
        self,
        detection: Detection,
        depth_m: np.ndarray,
        info: CameraInfo,
    ) -> Optional[Pose3D]:
        if len(detection.corners) != 4 or info.k[0] <= 1e-6 or info.k[4] <= 1e-6:
            return None
        mask = self.mask_for_frame(detection.mask, depth_m.shape[:2])
        depth_mask = (mask > 0) & np.isfinite(depth_m) & (depth_m > 0.0)
        if not np.any(depth_mask):
            return None
        pose_depth = depth_m.copy()
        pose_depth[(mask == 0) | ~np.isfinite(pose_depth) | (pose_depth <= 0.0)] = np.nan

        fallback_depth = float(np.nanmedian(pose_depth))
        if not np.isfinite(fallback_depth) or fallback_depth <= 0.0:
            fallback_depth = float(np.nanmedian(depth_m[depth_mask]))
        if not np.isfinite(fallback_depth) or fallback_depth <= 0.0:
            return None

        camera_points = []
        for corner in detection.corners:
            depth = self.average_depth_at(pose_depth, corner, fallback_depth)
            camera_points.append(self.project_pixel(corner, depth, info))
        origin_idx = lower_left_corner_index(detection.corners)
        if origin_idx < 0:
            return None
        prev_idx = (origin_idx + 3) % 4
        next_idx = (origin_idx + 1) % 4
        origin_corner = camera_points[origin_idx]
        dir_a = camera_points[prev_idx] - origin_corner
        dir_b = camera_points[next_idx] - origin_corner
        len_a = float(np.linalg.norm(dir_a))
        len_b = float(np.linalg.norm(dir_b))
        if len_a < 1e-9 or len_b < 1e-9:
            return None
        x_axis = dir_a if len_a >= len_b else dir_b
        y_axis = dir_b if len_a >= len_b else dir_a
        x_axis = normalize(x_axis)
        if x_axis is None:
            return None
        y_axis = y_axis - x_axis * float(np.dot(x_axis, y_axis))
        y_axis = normalize(y_axis)
        if y_axis is None:
            return None
        z_axis = normalize(np.cross(x_axis, y_axis))
        if z_axis is None:
            return None
        if float(np.dot(z_axis, origin_corner)) > 0.0:
            y_axis *= -1.0
            z_axis = normalize(np.cross(x_axis, y_axis))
            if z_axis is None:
                return None
        pick_pixel = corner_center(detection.corners)
        pick_depth = self.average_depth_at(pose_depth, pick_pixel, fallback_depth)
        pose_origin = self.project_pixel(pick_pixel, pick_depth, info)
        rotation = np.array([
            [x_axis[0], y_axis[0], z_axis[0]],
            [x_axis[1], y_axis[1], z_axis[1]],
            [x_axis[2], y_axis[2], z_axis[2]],
        ], dtype=np.float64)
        return Pose3D(origin=pose_origin, rotation=rotation)

    def average_depth_at(self, depth_m: np.ndarray, point: Tuple[float, float], fallback: float) -> float:
        x = int(round(point[0]))
        y = int(round(point[1]))
        x0 = max(0, x - 4)
        y0 = max(0, y - 4)
        x1 = min(depth_m.shape[1], x + 5)
        y1 = min(depth_m.shape[0], y + 5)
        patch = depth_m[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size:
            return float(np.median(valid))
        return fallback

    def mask_center_3d(
        self,
        depth_m: np.ndarray,
        mask: np.ndarray,
        info: CameraInfo,
        fallback_depth: float,
    ) -> np.ndarray:
        valid = (mask > 0) & np.isfinite(depth_m) & (depth_m > 0.0)
        ys, xs = np.where(valid)
        if xs.size:
            depths = depth_m[ys, xs]
            return np.array([
                np.median((xs.astype(np.float64) - info.k[2]) * depths / info.k[0]),
                np.median((ys.astype(np.float64) - info.k[5]) * depths / info.k[4]),
                np.median(depths),
            ], dtype=np.float64)
        moments = cv2.moments(mask)
        if abs(moments["m00"]) > 1e-6:
            center = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        else:
            center = (0.0, 0.0)
        return self.project_pixel(center, fallback_depth, info)

    def project_pixel(self, point: Tuple[float, float], depth: float, info: CameraInfo) -> np.ndarray:
        x = (float(point[0]) - info.k[2]) * depth / info.k[0]
        y = (float(point[1]) - info.k[5]) * depth / info.k[4]
        return np.array([x, y, depth], dtype=np.float64)

    def camera_point_to_pixel(
        self,
        point: np.ndarray,
        info: CameraInfo,
        shape: Tuple[int, int],
    ) -> Optional[Tuple[float, float]]:
        if info.k[0] <= 1e-6 or info.k[4] <= 1e-6:
            return None
        z = float(point[2])
        if not np.isfinite(z) or z <= 1e-9:
            return None
        x = float(point[0]) * info.k[0] / z + info.k[2]
        y = float(point[1]) * info.k[4] / z + info.k[5]
        if not np.isfinite(x) or not np.isfinite(y):
            return None
        return (
            float(np.clip(x, 0.0, max(0, shape[1] - 1))),
            float(np.clip(y, 0.0, max(0, shape[0] - 1))),
        )

    def clear_seek_debug_capture(self) -> None:
        self.last_seek_debug_frame = None
        self.last_seek_raw_frame = None
        self.last_seek_overlay_frame = None
        self.last_seek_debug_stamp_ns = 0
        self.last_seek_debug_frame_id = ""
        self.last_seek_debug_score = None

    @staticmethod
    def stamp_to_nanoseconds(stamp) -> int:
        try:
            return int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))
        except Exception:
            return 0

    def update_seek_debug_capture(
        self,
        stamp,
        frame_id: str,
        raw_frame: Optional[np.ndarray],
        overlay_frame: Optional[np.ndarray],
    ) -> None:
        if not self.seek_mode_active or self.selected_pose is None:
            return
        if overlay_frame is None or overlay_frame.size == 0:
            return
        self.last_seek_debug_frame = overlay_frame.copy()
        self.last_seek_overlay_frame = overlay_frame.copy()
        if raw_frame is not None and raw_frame.size > 0:
            self.last_seek_raw_frame = raw_frame.copy()
        self.last_seek_debug_stamp_ns = self.stamp_to_nanoseconds(stamp)
        self.last_seek_debug_frame_id = frame_id
        self.last_seek_debug_score = (
            float(self.selected_detection.score)
            if self.selected_detection is not None else None
        )

    @staticmethod
    def draw_seek_diagnostic_label(image: np.ndarray, lines: List[str]) -> None:
        if image is None or image.size == 0:
            return
        pad = 10
        line_h = 22
        width = max(160, min(image.shape[1] - 2 * pad, 560))
        height = max(36, pad + line_h * len(lines) + 8)
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (width, height), (18, 20, 24), -1)
        cv2.addWeighted(overlay, 0.72, image, 0.28, 0.0, image)
        for index, line in enumerate(lines):
            cv2.putText(
                image,
                str(line),
                (pad, pad + 18 + index * line_h),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (245, 248, 252),
                1,
                cv2.LINE_AA,
            )

    def seek_diagnostic_text(self, pose: Pose3D, frame_id: str) -> List[str]:
        profile = self.active_profile
        score = self.last_seek_debug_score
        item_name = profile.item_name if profile is not None else "item"
        first = f"{item_name}"
        if score is not None:
            first += f" {score * 100.0:.0f}%"
        return [
            first,
            f"frame {frame_id or self.last_seek_debug_frame_id or '<none>'}",
            (
                f"x={pose.origin[0] * METERS_TO_MM:.1f} "
                f"y={pose.origin[1] * METERS_TO_MM:.1f} "
                f"z={pose.origin[2] * METERS_TO_MM:.1f} mm"
            ),
        ]

    def build_seek_diagnostic_image(self, pose: Pose3D, frame_id: str) -> Optional[np.ndarray]:
        overlay_frame = self.last_seek_overlay_frame
        if overlay_frame is None:
            overlay_frame = self.last_seek_debug_frame
        if overlay_frame is None or overlay_frame.size == 0:
            return None
        raw_frame = self.last_seek_raw_frame
        if raw_frame is None or raw_frame.size == 0:
            raw_frame = overlay_frame
        raw = np.asarray(raw_frame).copy()
        overlay = np.asarray(overlay_frame).copy()
        if raw.ndim == 2:
            raw = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
        if overlay.ndim == 2:
            overlay = cv2.cvtColor(overlay, cv2.COLOR_GRAY2BGR)
        if raw.shape[:2] != overlay.shape[:2]:
            overlay = cv2.resize(overlay, (raw.shape[1], raw.shape[0]), interpolation=cv2.INTER_LINEAR)
        self.draw_seek_diagnostic_label(raw, ["Raw RGB"])
        self.draw_seek_diagnostic_label(overlay, self.seek_diagnostic_text(pose, frame_id))
        divider = np.full((raw.shape[0], 8, 3), (20, 22, 26), dtype=np.uint8)
        return np.hstack((raw, divider, overlay))

    def write_exact_seek_diagnostic_image(self, pose: Pose3D, frame_id: str) -> Tuple[bool, str]:
        if not (self.seek_diagnostic_image_enabled or self.debug_images_enabled):
            return True, ""
        image = self.build_seek_diagnostic_image(pose, frame_id)
        if image is None:
            return False, "diagnostic frame missing"
        try:
            self.seek_diagnostic_image_path.parent.mkdir(parents=True, exist_ok=True)
            tmp_path = self.seek_diagnostic_image_path.with_name(
                f".{self.seek_diagnostic_image_path.stem}.tmp{self.seek_diagnostic_image_path.suffix or '.png'}"
            )
            if not cv2.imwrite(str(tmp_path), image):
                return False, f"image write failed: {self.seek_diagnostic_image_path}"
            tmp_path.replace(self.seek_diagnostic_image_path)
            self.get_logger().info(
                f"Saved exact item seek diagnostic image: {self.seek_diagnostic_image_path}"
            )
            return True, str(self.seek_diagnostic_image_path)
        except Exception as exc:
            self.get_logger().warn(f"Exact item seek diagnostic save failed: {exc}")
            return False, str(exc)

    def write_seek_debug_outputs(
        self,
        stamp,
        frame_id: str,
        pose: Pose3D,
    ) -> Tuple[bool, str]:
        if not self.debug_images_enabled:
            return True, ""
        frame = self.last_seek_debug_frame
        if frame is None or frame.size == 0:
            return False, "debug frame missing"
        stamp_ns = self.last_seek_debug_stamp_ns or self.stamp_to_nanoseconds(stamp) or time.time_ns()
        image_path = self.seek_snapshots_dir / f"seek_{stamp_ns}_last.png"
        pose_path = self.seek_snapshots_dir / f"seek_{stamp_ns}_pose.yaml"
        try:
            self.seek_snapshots_dir.mkdir(parents=True, exist_ok=True)
            wrote_image = cv2.imwrite(str(image_path), frame)
            qx, qy, qz, qw = rotation_to_quaternion(pose.rotation)
            profile = self.active_profile
            pose_data = {
                "timestamp_ns": int(stamp_ns),
                "frame_id": frame_id or self.last_seek_debug_frame_id,
                "debug_frame_path": str(image_path),
                "seek_window_sec": float(self.seek_window_sec),
                "seek_decay_sec": float(self.seek_decay_sec),
                "item_name": profile.item_name if profile is not None else "",
                "profile_path": str(profile.path) if profile is not None else "",
                "model_path": str(profile.model_path) if profile is not None else "",
                "position_mm": {
                    "x": float(pose.origin[0] * METERS_TO_MM),
                    "y": float(pose.origin[1] * METERS_TO_MM),
                    "z": float(pose.origin[2] * METERS_TO_MM),
                },
                "orientation": {
                    "x": float(qx),
                    "y": float(qy),
                    "z": float(qz),
                    "w": float(qw),
                },
                "rotation_matrix": [
                    [float(pose.rotation[row, col]) for col in range(3)]
                    for row in range(3)
                ],
                "selected_detection": {
                    "score": self.last_seek_debug_score,
                },
            }
            pose_path.write_text(yaml.safe_dump(pose_data, sort_keys=False), encoding="utf-8")
            if not wrote_image:
                return False, f"image write failed: {image_path}"
            self.get_logger().info(
                f"Seek debug saved:\n  image: {image_path}\n  pose:  {pose_path}"
            )
            return True, str(image_path)
        except Exception as exc:
            self.get_logger().warn(f"Seek debug save failed: {exc}")
            return False, str(exc)

    def publish_pose_outputs(
        self,
        stamp,
        frame_id: str,
        raw_frame: Optional[np.ndarray] = None,
        overlay_frame: Optional[np.ndarray] = None,
    ) -> None:
        now = time.monotonic()
        if (
            self.seek_mode_active and
            self.seek_started_time > 0.0 and
            now - self.seek_started_time > max(0.1, self.seek_window_sec)
        ):
            self.seek_started_time = now
            self.last_seek_pose = None
            self.last_seek_pose_time = 0.0
            self.clear_seek_debug_capture()
            self.status = f"Seek still ON: no valid YOLO pose in {self.seek_window_sec:.1f}s window; reacquiring"

        pose_array = PoseArray()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = frame_id
        for detection_pose in self.latest_detection_poses:
            if detection_pose is not None:
                pose_array.poses.append(pose_to_msg(detection_pose.pose))
        self.item_pose_array_pub.publish(pose_array)

        self.update_seek_debug_capture(stamp, frame_id, raw_frame, overlay_frame)
        seek_pose = self.selected_pose if self.seek_mode_active else None
        if self.seek_mode_active:
            if seek_pose is not None:
                self.last_seek_pose = seek_pose
                self.last_seek_pose_time = now
            elif (
                self.last_seek_pose is not None and
                now - self.last_seek_pose_time <= max(0.0, self.seek_decay_sec)
            ):
                seek_pose = self.last_seek_pose

        if self.seek_mode_active and seek_pose is not None:
            msg = PoseStamped()
            msg.header = pose_array.header
            msg.pose = pose_to_msg(seek_pose)
            self.seek_pose_pub.publish(msg)
            self.publish_marker(stamp, frame_id, seek_pose)
            self.seek_mode_active = False
            self.seek_result_latched = True
            self.seek_started_time = 0.0
            self.last_seek_pose = seek_pose
            self.last_seek_pose_time = now
            status = "Seek done, handed off YOLO item target | YOLO OFF"
            exact_saved, exact_detail = self.write_exact_seek_diagnostic_image(seek_pose, frame_id)
            debug_saved, debug_detail = self.write_seek_debug_outputs(stamp, frame_id, seek_pose)
            if self.seek_diagnostic_image_enabled or self.debug_images_enabled:
                if exact_saved and exact_detail:
                    status += f" | latest {Path(exact_detail).name}"
                elif not exact_saved:
                    status += f" | latest save failed: {exact_detail}"
            if self.debug_images_enabled:
                if debug_saved:
                    status += f" | debug {Path(debug_detail).name}"
                else:
                    status += f" | debug save failed: {debug_detail}"
            status += " | waiting for item pick release"
            self.turn_off_seek_yolo(status)

    def publish_marker(self, stamp, frame_id: str, pose: Pose3D) -> None:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = "item_detect"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose_to_msg(pose)
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.015
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.45
        self.item_marker_pub.publish(marker)

    def display_frame(self, frame: np.ndarray, depth_m: Optional[np.ndarray]) -> np.ndarray:
        if self.view_mode == "Depth":
            return self.depth_visualization(depth_m, frame.shape)
        if self.view_mode == "Binarized":
            return self.binarized_visualization(frame.shape)
        return frame.copy()

    def binarized_visualization(self, frame_shape: Tuple[int, int, int]) -> np.ndarray:
        output = np.zeros(frame_shape, dtype=np.uint8)
        for index, detection in enumerate(self.latest_detections):
            mask = self.mask_for_frame(detection.mask, frame_shape[:2])
            color = (230, 230, 230) if index != 0 else (255, 255, 255)
            output[mask > 0] = color
        return output

    def depth_visualization(self, depth_m: Optional[np.ndarray], frame_shape: Tuple[int, int, int]) -> np.ndarray:
        output = np.zeros(frame_shape, dtype=np.uint8)
        if depth_m is None:
            cv2.putText(output, "No depth image", (18, 34),
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (225, 230, 236), 1, cv2.LINE_AA)
            return output
        depth = depth_m.astype(np.float32, copy=False)
        if depth.shape[:2] != frame_shape[:2]:
            depth = cv2.resize(depth, (frame_shape[1], frame_shape[0]), interpolation=cv2.INTER_NEAREST)
        valid = np.isfinite(depth) & (depth > 0.0)
        if not np.any(valid):
            cv2.putText(output, "No valid depth", (18, 34),
                        cv2.FONT_HERSHEY_DUPLEX, 0.8, (225, 230, 236), 1, cv2.LINE_AA)
            return output
        values = depth[valid]
        near = float(np.percentile(values, 2.0))
        far = float(np.percentile(values, 98.0))
        if not np.isfinite(near) or not np.isfinite(far) or far <= near:
            near = float(np.min(values))
            far = float(np.max(values))
        if far <= near:
            far = near + 1e-3
        normalized = np.zeros(depth.shape[:2], dtype=np.uint8)
        normalized[valid] = np.clip((depth[valid] - near) * 255.0 / (far - near), 0.0, 255.0).astype(np.uint8)
        color_map = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
        output = cv2.applyColorMap(normalized, color_map)
        output[~valid] = (0, 0, 0)
        return output

    def draw_detection_axes(
        self,
        image: np.ndarray,
        detection: Detection,
        index: int,
        detection_pose: Optional[DetectionPose],
    ) -> None:
        if len(detection.corners) != 4:
            return
        origin_idx = lower_left_corner_index(detection.corners)
        if origin_idx < 0:
            return
        corners = [np.array(point, dtype=np.float32) for point in detection.corners]
        origin = corners[origin_idx]
        prev_corner = corners[(origin_idx + 3) % 4]
        next_corner = corners[(origin_idx + 1) % 4]
        dir_a = prev_corner - origin
        dir_b = next_corner - origin
        len_a = float(np.linalg.norm(dir_a))
        len_b = float(np.linalg.norm(dir_b))
        if len_a < 1e-3 or len_b < 1e-3:
            return
        if len_a >= len_b:
            x_dir = dir_a / len_a
            y_dir = dir_b / len_b
            x_half_len = max(8.0, len_a * 0.5)
            y_half_len = max(8.0, len_b * 0.5)
        else:
            x_dir = dir_b / len_b
            y_dir = dir_a / len_a
            x_half_len = max(8.0, len_b * 0.5)
            y_half_len = max(8.0, len_a * 0.5)
        hull_center = np.array(corner_center(detection.corners), dtype=np.float32)
        if detection_pose is not None:
            axis_center = np.array(detection_pose.pick_pixel, dtype=np.float32)
        else:
            axis_center = hull_center
        x_start = axis_center - x_dir * x_half_len
        x_end = axis_center + x_dir * x_half_len
        y_start = axis_center - y_dir * y_half_len
        y_end = axis_center + y_dir * y_half_len

        corner_points = np.asarray(
            [[int(round(point[0])), int(round(point[1]))] for point in corners],
            dtype=np.int32,
        )
        cv2.polylines(image, [corner_points], True, (255, 220, 40), 2, cv2.LINE_AA)
        for corner in corners:
            cv2.circle(image, (int(round(corner[0])), int(round(corner[1]))), 4, (255, 255, 255), -1, cv2.LINE_AA)

        center_pt = (int(round(axis_center[0])), int(round(axis_center[1])))
        x_start_pt = (int(round(x_start[0])), int(round(x_start[1])))
        x_end_pt = (int(round(x_end[0])), int(round(x_end[1])))
        y_start_pt = (int(round(y_start[0])), int(round(y_start[1])))
        y_end_pt = (int(round(y_end[0])), int(round(y_end[1])))
        cv2.line(image, x_start_pt, x_end_pt, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.line(image, y_start_pt, y_end_pt, (255, 120, 0), 2, cv2.LINE_AA)
        cv2.arrowedLine(image, center_pt, x_end_pt, (0, 0, 255), 3, cv2.LINE_AA, 0, 0.15)
        cv2.arrowedLine(image, center_pt, y_end_pt, (255, 120, 0), 3, cv2.LINE_AA, 0, 0.15)
        if detection_pose is not None:
            cv2.circle(image, center_pt, 6, (40, 255, 255), -1, cv2.LINE_AA)
        cv2.putText(image, "X", (x_end_pt[0] + 4, x_end_pt[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(image, "Y", (y_end_pt[0] + 4, y_end_pt[1] - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 120, 0), 2, cv2.LINE_AA)
        if detection_pose is not None:
            cv2.putText(image, str(index + 1), (center_pt[0] + 8, center_pt[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.56, (40, 255, 255), 2, cv2.LINE_AA)

    def build_no_camera_topics_placeholder(self) -> np.ndarray:
        placeholder = np.zeros((PREVIEW_CANVAS_HEIGHT, PREVIEW_CANVAS_WIDTH, 3), dtype=np.uint8)
        placeholder[:] = (18, 18, 18)
        cv2.rectangle(
            placeholder,
            (0, 0),
            (placeholder.shape[1] - 1, placeholder.shape[0] - 1),
            (34, 34, 34),
            2,
        )

        lines = [
            "no camera topics...",
            f"color: {self.color_topic}  publishers={self.count_publishers(self.color_topic)}",
            f"depth: {self.depth_topic}  publishers={self.count_publishers(self.depth_topic)}",
            f"info:  {self.camera_info_topic}  publishers={self.count_publishers(self.camera_info_topic)}",
        ]
        scales = [1.35, 0.68, 0.68, 0.68]
        thicknesses = [3, 1, 1, 1]
        colors = [
            (80, 220, 255),
            (220, 220, 220),
            (220, 220, 220),
            (220, 220, 220),
        ]
        y = (placeholder.shape[0] // 2) - 55
        for index, line in enumerate(lines):
            text_size, _ = cv2.getTextSize(
                line,
                cv2.FONT_HERSHEY_SIMPLEX,
                scales[index],
                thicknesses[index],
            )
            x = max(20, (placeholder.shape[1] - text_size[0]) // 2)
            cv2.putText(
                placeholder,
                line,
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                scales[index],
                colors[index],
                thicknesses[index],
                cv2.LINE_AA,
            )
            y += 52 if index == 0 else 32
        return placeholder

    def render_no_camera_topics_overlay(self) -> None:
        now = time.monotonic()
        if self.last_camera_render_time > 0.0 and now - self.last_camera_render_time < 1.0:
            return

        output = self.build_ui(self.build_no_camera_topics_placeholder())
        if self.publish_overlay:
            msg = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.camera_frame or "camera_color_optical_frame"
            self.overlay_pub.publish(msg)
        if self.start_visualization:
            cv2.imshow(WINDOW_NAME, output)
            cv2.waitKey(1)
        self.last_camera_render_time = now

    def render_overlay(self, frame: np.ndarray, depth_m: Optional[np.ndarray]) -> np.ndarray:
        output = frame.copy()
        profile = self.active_profile
        if len(self.bin_teach_roi_points) >= 3:
            pts = np.asarray(self.bin_teach_roi_points, dtype=np.int32)
            cv2.polylines(output, [pts], True, (80, 220, 255), 2)
        for index, detection in enumerate(self.latest_detections):
            mask = self.mask_for_frame(detection.mask, output.shape[:2])
            color = (90, 180, 255)
            overlay = output.copy()
            overlay[mask > 0] = color
            output = cv2.addWeighted(overlay, 0.28, output, 0.72, 0.0)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(output, contours, -1, color, 2)
            detection_pose = (
                self.latest_detection_poses[index]
                if index < len(self.latest_detection_poses)
                else None
            )
            self.draw_detection_axes(output, detection, index, detection_pose)
            label = f"{detection.score * 100.0:.0f}%"
            cv2.putText(output, label, (int(round(detection.center[0])) + 8, int(round(detection.center[1])) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(output, label, (int(round(detection.center[0])) + 8, int(round(detection.center[1])) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (245, 245, 245), 1, cv2.LINE_AA)
        if self.seek_is_on() and self.selected_detection is not None:
            mask = self.mask_for_frame(self.selected_detection.mask, output.shape[:2])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(output, contours, -1, (0, 255, 0), 3)
        if self.peak_pixel is not None:
            cv2.circle(output, self.peak_pixel, 12, (0, 0, 255), 2)
            cv2.circle(output, self.peak_pixel, 3, (255, 255, 255), -1)
        title = profile.item_name if profile else "No YOLO profile"
        if self.selected_detection is not None:
            title = f"{title} {self.selected_detection.score * 100.0:.0f}%"
        cv2.putText(output, title, (18, 32),
                    cv2.FONT_HERSHEY_DUPLEX, 0.8, (240, 245, 250), 1, cv2.LINE_AA)
        return output

    def build_ui(self, preview_image: np.ndarray) -> np.ndarray:
        canvas_w = PREVIEW_CANVAS_WIDTH
        canvas_h = TOP_BAR_HEIGHT + PREVIEW_CANVAS_HEIGHT
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        canvas[:] = (30, 32, 36)
        self.buttons.clear()
        self.slider_rects.clear()

        self.draw_top_controls(canvas)

        preview, scale = self.fit_preview(preview_image)
        self.preview_scale = scale
        x = 0
        y = TOP_BAR_HEIGHT
        canvas[y:y + PREVIEW_CANVAS_HEIGHT, x:x + PREVIEW_CANVAS_WIDTH] = (8, 10, 12)
        canvas[y:y + preview.shape[0], x:x + preview.shape[1]] = preview
        self.preview_rect = (x, y, preview.shape[1], preview.shape[0])

        self.draw_delete_confirmation_overlay(canvas)
        if self.start_visualization:
            self.resize_window_if_needed(canvas_w, canvas_h)
        return canvas

    def draw_top_controls(self, canvas: np.ndarray) -> None:
        width = canvas.shape[1]
        bar = canvas[:TOP_BAR_HEIGHT, :]
        bar[:] = (28, 30, 34)
        cv2.line(canvas, (0, 58), (width, 58), (52, 56, 62), 1)

        margin = 16
        gap = 10
        y = 14
        h = 38
        button_count = 8
        button_w = max(112, min(132, (width - 2 * margin - (button_count - 1) * gap) // button_count))
        x = margin
        self.draw_button(
            canvas,
            "view_mode",
            (x, y, button_w, h),
            f"View: {self.view_mode}",
            True,
            True,
            fill_color=(72, 128, 68),
            border_color=(132, 215, 150),
        )
        x += button_w + gap
        self.draw_button(
            canvas,
            "overlay",
            (x, y, button_w, h),
            "Overlay: ON" if self.publish_overlay else "Overlay: OFF",
            True,
            self.publish_overlay,
            fill_color=(48, 62, 72),
            active_fill_color=(68, 124, 154),
            border_color=(102, 106, 112),
            active_border_color=(132, 205, 236),
        )
        x += button_w + gap
        self.draw_button(
            canvas,
            "seek",
            (x, y, button_w, h),
            "Seek: ON" if self.seek_is_on() else "Seek: OFF",
            True,
            self.seek_is_on(),
            fill_color=(48, 62, 72),
            active_fill_color=(70, 126, 186),
            border_color=(102, 106, 112),
            active_border_color=(126, 202, 255),
        )
        x += button_w + gap
        self.draw_button(
            canvas,
            "debug_images",
            (x, y, button_w, h),
            "Debug Img",
            True,
            self.debug_images_enabled,
            fill_color=(58, 64, 72),
            active_fill_color=(70, 126, 186),
            border_color=(112, 120, 130),
            active_border_color=(126, 202, 255),
        )
        x += button_w + gap
        self.draw_button(
            canvas,
            "go_to_teach",
            (x, y, button_w, h),
            "Go Teach..." if self.go_to_teach_in_progress else "Go To Teach",
            self.can_go_to_teach(),
            self.go_to_teach_in_progress,
            fill_color=(70, 140, 94),
            active_fill_color=(70, 126, 186),
            border_color=(134, 232, 165),
            active_border_color=(126, 202, 255),
        )
        x += button_w + gap
        self.draw_button(
            canvas,
            "open_model",
            (x, y, button_w, h),
            "Open Model",
            True,
            False,
            fill_color=(61, 78, 96),
            border_color=(130, 166, 198),
        )
        x += button_w + gap
        self.draw_button(
            canvas,
            "delete_profile",
            (x, y, button_w, h),
            "Delete Item",
            self.can_delete_profile(),
            False,
            fill_color=(86, 76, 148),
            border_color=(160, 146, 246),
        )
        x += button_w + gap
        self.draw_button(
            canvas,
            "yolo_toggle",
            (x, y, button_w, h),
            "YOLO: ON" if self.yolo_enabled else "YOLO: OFF",
            True,
            self.yolo_enabled,
            fill_color=(68, 58, 58),
            active_fill_color=(70, 140, 94),
            border_color=(138, 106, 106),
            active_border_color=(134, 232, 165),
        )

        panel_y = y + h + 12
        panel_gap = 10
        panel_h = 92
        panel_total_w = max(360, width - 2 * margin)
        panel_w = max(120, (panel_total_w - 2 * panel_gap) // 3)
        summary_rect = (margin, panel_y, panel_w, panel_h)
        seek_rect = (margin + panel_w + panel_gap, panel_y, panel_w, panel_h)
        quality_rect = (
            seek_rect[0] + panel_w + panel_gap,
            panel_y,
            max(120, width - margin - (seek_rect[0] + panel_w + panel_gap)),
            panel_h,
        )
        status_rect = (margin, panel_y + panel_h + 8, max(120, width - 2 * margin), 28)

        self.draw_panel_box(canvas, summary_rect, "Item Summary")
        self.draw_panel_box(canvas, seek_rect, "Seek Controls")
        self.draw_panel_box(canvas, quality_rect, "Detection Quality")

        x, y, w, h = summary_rect
        count = len(self.latest_detections)
        cv2.putText(canvas, f"Detected masks: {count}", (x + 12, y + 42),
                    cv2.FONT_HERSHEY_DUPLEX, 0.45, (164, 238, 144), 1, cv2.LINE_AA)
        best_text = "Best: n/a"
        if self.selected_detection is not None:
            best_text = f"Best confidence: {self.selected_detection.score * 100.0:.0f}%"
        cv2.putText(canvas, best_text, (x + 12, y + 64),
                    cv2.FONT_HERSHEY_DUPLEX, 0.43, (205, 212, 220), 1, cv2.LINE_AA)
        if self.seek_is_on():
            if self.selected_pose is not None:
                xyz = self.selected_pose.origin * METERS_TO_MM
                pose_text = f"Best candidate XYZ: {xyz[0]:+.1f} {xyz[1]:+.1f} {xyz[2]:+.1f} mm"
            else:
                pose_text = "Need valid depth on selected mask"
        else:
            pose_text = "Best = highest-confidence YOLO mask"
        cv2.putText(canvas, fit_text(pose_text, max(20, w // 9)), (x + 12, y + 82),
                    cv2.FONT_HERSHEY_DUPLEX, 0.38, (165, 170, 176), 1, cv2.LINE_AA)

        x, y, w, h = seek_rect
        self.draw_slider(
            canvas,
            "seek_window",
            (x + 12, y + 37, w - 24, 26),
            f"Window  {self.seek_window_sec:.1f}s",
            self.seek_window_sec,
            SEEK_WINDOW_MIN_SEC,
            SEEK_WINDOW_MAX_SEC,
            (140, 210, 250),
        )
        self.draw_slider(
            canvas,
            "seek_decay",
            (x + 12, y + 67, w - 24, 26),
            f"Decay  {self.seek_decay_sec:.1f}s",
            self.seek_decay_sec,
            SEEK_DECAY_MIN_SEC,
            SEEK_DECAY_MAX_SEC,
            (154, 230, 170),
        )

        x, y, w, h = quality_rect
        confidence_percent = int(round(np.clip(self.yolo_conf, 0.0, 1.0) * 100.0))
        margin_px = int(round(self.detect_roi_margin_px))
        cv2.putText(canvas, fit_text(self.inference_speed_text(), max(20, w // 8)), (x + 12, y + 36),
                    cv2.FONT_HERSHEY_DUPLEX, 0.43, (205, 212, 220), 1, cv2.LINE_AA)
        self.draw_slider(
            canvas,
            "confidence",
            (x + 12, y + 45, w - 24, 22),
            f"Confidence  {confidence_percent}%",
            self.yolo_conf,
            0.0,
            1.0,
            (85, 225, 255),
        )
        self.draw_slider(
            canvas,
            "detect_roi_margin",
            (x + 12, y + 68, w - 24, 22),
            f"ROI margin  {margin_px}px",
            self.detect_roi_margin_px,
            DETECT_ROI_MARGIN_MIN_PX,
            DETECT_ROI_MARGIN_MAX_PX,
            (255, 210, 80),
        )

        x, y, w, h = status_rect
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (34, 36, 40), -1)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (72, 77, 84), 1)
        default_status = (
            f"Ready | {yolo_version_label(self.active_profile.yolo_version)} Model: "
            f"{Path(self.active_profile.model_path).name}"
            if self.active_profile is not None else
            f"Ready | Open Model to choose {supported_yolo_label()} .pt"
        )
        status_text = self.status or default_status
        cv2.putText(canvas, "Status", (x + 10, y + 19),
                    cv2.FONT_HERSHEY_DUPLEX, 0.43, (202, 208, 214), 1, cv2.LINE_AA)
        cv2.putText(canvas, fit_text(status_text, max(20, (w - 88) // 8)), (x + 70, y + 19),
                    cv2.FONT_HERSHEY_DUPLEX, 0.44, (194, 199, 206), 1, cv2.LINE_AA)

    def draw_panel_box(self, canvas: np.ndarray, rect: Tuple[int, int, int, int], title: str) -> None:
        x, y, w, h = rect
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (38, 41, 46), -1)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (72, 77, 84), 1)
        cv2.rectangle(canvas, (x, y), (x + w, y + 22), (46, 50, 56), -1)
        cv2.putText(canvas, title, (x + 12, y + 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (220, 224, 230), 1, cv2.LINE_AA)

    def draw_slider(
        self,
        canvas: np.ndarray,
        name: str,
        rect: Tuple[int, int, int, int],
        label: str,
        value: float,
        min_value: float,
        max_value: float,
        color: Tuple[int, int, int],
    ) -> None:
        x, y, w, h = rect
        cv2.putText(canvas, label, (x, y + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (225, 230, 236), 1, cv2.LINE_AA)
        track_x = x
        track_y = y + h - 8
        track_w = max(1, w)
        norm = 0.0
        if max_value > min_value:
            norm = (float(value) - min_value) / (max_value - min_value)
        norm = float(np.clip(norm, 0.0, 1.0))
        cv2.rectangle(canvas, (track_x, track_y - 3), (track_x + track_w, track_y + 3), (68, 73, 79), -1)
        cv2.rectangle(canvas, (track_x, track_y - 3), (track_x + track_w, track_y + 3), (93, 99, 106), 1)
        cv2.rectangle(canvas, (track_x, track_y - 2), (track_x + int(round(track_w * norm)), track_y + 2),
                      color, -1)
        knob_x = track_x + int(round(track_w * norm))
        cv2.circle(canvas, (knob_x, track_y), 8, (235, 235, 235), -1)
        cv2.circle(canvas, (knob_x, track_y), 8, (96, 100, 106), 1)
        self.slider_rects[name] = (track_x, track_y - 12, track_w, 24)

    def draw_dropdown_arrow(self, canvas: np.ndarray, rect: Tuple[int, int, int, int], enabled: bool) -> None:
        x, y, w, h = rect
        cx = x + w - 16
        cy = y + h // 2 + 2
        color = (245, 245, 245) if enabled else (150, 150, 150)
        pts = np.array([[cx - 6, cy - 4], [cx + 6, cy - 4], [cx, cy + 4]], dtype=np.int32)
        cv2.fillPoly(canvas, [pts], color)

    def fit_preview(self, image: np.ndarray) -> Tuple[np.ndarray, float]:
        h, w = image.shape[:2]
        scale = min(PREVIEW_CANVAS_WIDTH / float(w), PREVIEW_CANVAS_HEIGHT / float(h))
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        return cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR), scale

    def resize_window_if_needed(self, width: int, height: int) -> None:
        window_size = (int(width), int(height))
        if window_size == self.rendered_window_size:
            return
        try:
            cv2.resizeWindow(WINDOW_NAME, window_size[0], window_size[1])
            self.rendered_window_size = window_size
        except cv2.error as exc:
            self.get_logger().warn(f"Could not resize YOLO detect window: {exc}")

    def draw_button(
        self,
        canvas: np.ndarray,
        name: str,
        rect: Tuple[int, int, int, int],
        label: str,
        enabled: bool = True,
        active: bool = False,
        fill_color: Optional[Tuple[int, int, int]] = None,
        border_color: Optional[Tuple[int, int, int]] = None,
        active_fill_color: Optional[Tuple[int, int, int]] = None,
        active_border_color: Optional[Tuple[int, int, int]] = None,
    ) -> None:
        x, y, w, h = rect
        base_fill = fill_color if fill_color is not None else (48, 62, 72)
        base_border = border_color if border_color is not None else (126, 202, 255)
        active_fill = active_fill_color if active_fill_color is not None else (62, 98, 130)
        active_border = active_border_color if active_border_color is not None else base_border
        fill = active_fill if active else (base_fill if enabled else (54, 54, 54))
        border = active_border if enabled else (100, 100, 100)
        text = (238, 242, 245) if enabled else (150, 150, 150)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), fill, -1)
        cv2.rectangle(canvas, (x, y), (x + w, y + h), border, 2)
        cv2.putText(canvas, fit_text(label, max(8, w // 9)), (x + 12, y + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, text, 1, cv2.LINE_AA)
        self.buttons[name] = Button(name, rect, enabled)

    def profile_label(self) -> str:
        if self.active_profile is not None:
            return f"{yolo_version_label(self.active_profile.yolo_version)} {self.active_profile.label}"
        if not self.profiles:
            return f"No {supported_yolo_label()} model"
        return "Open Model"

    @staticmethod
    def rect_contains(rect: Tuple[int, int, int, int], x: int, y: int) -> bool:
        rx, ry, rw, rh = rect
        return rx <= x <= rx + rw and ry <= y <= ry + rh

    def layout_delete_confirmation(self, frame_shape: Tuple[int, int, int]) -> None:
        height, width = frame_shape[:2]
        dialog_w = int(np.clip(width - 120, 420, 640))
        dialog_h = 170
        dialog_x = max(20, (width - dialog_w) // 2)
        dialog_y = max(20, (height - dialog_h) // 2)
        button_w = 126
        button_h = 36
        button_gap = 12
        button_y = dialog_y + dialog_h - button_h - 16
        cancel_x = dialog_x + dialog_w - (2 * button_w + button_gap + 16)
        self.delete_confirm_dialog_rect = (dialog_x, dialog_y, dialog_w, dialog_h)
        self.delete_confirm_cancel_rect = (cancel_x, button_y, button_w, button_h)
        self.delete_confirm_accept_rect = (cancel_x + button_w + button_gap, button_y, button_w, button_h)

    def draw_delete_confirmation_overlay(self, frame: np.ndarray) -> None:
        if not self.delete_confirm_active:
            return
        self.layout_delete_confirmation(frame.shape)
        shaded = frame.copy()
        shaded[:] = (0, 0, 0)
        cv2.addWeighted(shaded, 0.38, frame, 0.62, 0.0, frame)

        x, y, w, h = self.delete_confirm_dialog_rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (42, 45, 50), -1)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (122, 126, 132), 2)
        target = Path(self.active_profile.path).name if self.active_profile is not None else "selected model"
        cv2.putText(frame, "Confirm Item Delete", (x + 16, y + 34),
                    cv2.FONT_HERSHEY_DUPLEX, 0.66, (242, 242, 242), 1, cv2.LINE_AA)
        cv2.putText(frame, fit_text(f"Delete model: {target}", 60), (x + 16, y + 74),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.56, (214, 218, 224), 1, cv2.LINE_AA)
        cv2.putText(frame, "This action cannot be undone.", (x + 16, y + 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (175, 182, 190), 1, cv2.LINE_AA)

        cx, cy, cw, ch = self.delete_confirm_cancel_rect
        cv2.rectangle(frame, (cx, cy), (cx + cw, cy + ch), (74, 78, 84), -1)
        cv2.rectangle(frame, (cx, cy), (cx + cw, cy + ch), (140, 144, 150), 2)
        cv2.putText(frame, "Cancel", (cx + 30, cy + 24),
                    cv2.FONT_HERSHEY_DUPLEX, 0.52, (245, 245, 245), 1, cv2.LINE_AA)

        dx, dy, dw, dh = self.delete_confirm_accept_rect
        cv2.rectangle(frame, (dx, dy), (dx + dw, dy + dh), (90, 76, 152), -1)
        cv2.rectangle(frame, (dx, dy), (dx + dw, dy + dh), (170, 156, 245), 2)
        cv2.putText(frame, "Delete", (dx + 30, dy + 24),
                    cv2.FONT_HERSHEY_DUPLEX, 0.52, (245, 245, 245), 1, cv2.LINE_AA)

    def mouse_callback(self, event, x: int, y: int, flags, param) -> None:
        del flags, param
        if self.delete_confirm_active:
            if event == cv2.EVENT_LBUTTONDOWN:
                if self.rect_contains(self.delete_confirm_accept_rect, x, y):
                    self.confirm_delete_profile()
                    return
                if (
                    self.rect_contains(self.delete_confirm_cancel_rect, x, y) or
                    not self.rect_contains(self.delete_confirm_dialog_rect, x, y)
                ):
                    self.delete_confirm_active = False
                    self.status = "Delete cancelled"
                    return
            if event == cv2.EVENT_LBUTTONUP:
                self.active_slider = None
            return

        if event == cv2.EVENT_LBUTTONUP:
            if self.active_slider is not None:
                self.save_runtime_ui_settings()
            self.active_slider = None
            return
        if event == cv2.EVENT_MOUSEMOVE and self.active_slider is not None:
            self.update_slider_from_x(self.active_slider, x)
            return
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        for name, rect in list(self.slider_rects.items()):
            if self.rect_contains(rect, x, y):
                self.active_slider = name
                self.update_slider_from_x(name, x)
                return

        for name, button in list(self.buttons.items()):
            if self.rect_contains(button.rect, x, y):
                if not button.enabled:
                    return
                if name == "view_mode":
                    self.advance_view_mode()
                    self.status = f"View: {self.view_mode}"
                elif name == "seek":
                    self.set_seek_mode(not self.seek_is_on())
                elif name == "overlay":
                    self.publish_overlay = not self.publish_overlay
                    self.status = "Overlay toggled"
                elif name == "debug_images":
                    self.debug_images_enabled = not self.debug_images_enabled
                    self.save_runtime_ui_settings()
                    self.status = "Debug images enabled" if self.debug_images_enabled else "Debug images disabled"
                elif name == "yolo_toggle":
                    self.set_yolo_enabled(not self.yolo_enabled)
                elif name == "open_model":
                    self.request_open_model()
                elif name == "go_to_teach":
                    self.request_go_to_teach()
                elif name == "delete_profile":
                    self.request_delete_profile()
                self.save_runtime_ui_settings()
                return

    def advance_view_mode(self) -> None:
        if self.view_mode == "RGB":
            self.view_mode = "Binarized"
        elif self.view_mode == "Binarized":
            self.view_mode = "Depth"
        else:
            self.view_mode = "RGB"

    def set_yolo_enabled(self, enabled: bool, message: Optional[str] = None) -> None:
        enabled = bool(enabled)
        if self.yolo_enabled == enabled:
            if message:
                self.status = message
            return
        self.yolo_enabled = enabled
        if enabled:
            self.last_inference_time = 0.0
            self.status = message or "YOLO ON | live inference enabled"
            return

        self.seek_auto_yolo_active = False
        self.latest_detections = []
        self.latest_detection_poses = []
        self.selected_detection = None
        self.selected_pose = None
        self.peak_pixel = None
        if self.seek_mode_active:
            self.seek_mode_active = False
            self.seek_result_latched = False
            self.seek_started_time = 0.0
            self.last_seek_pose = None
            self.last_seek_pose_time = 0.0
        self.status = message or "YOLO OFF | live inference paused"

    def update_slider_from_x(self, name: str, x: int) -> None:
        rect = self.slider_rects.get(name)
        if rect is None:
            return
        rx, _, rw, _ = rect
        norm = 0.0 if rw <= 0 else (float(x) - float(rx)) / float(rw)
        norm = float(np.clip(norm, 0.0, 1.0))
        if name == "confidence":
            self.yolo_conf = norm
            self.last_inference_time = 0.0
            self.status = f"Confidence threshold: {int(round(self.yolo_conf * 100.0))}%"
        elif name == "detect_roi_margin":
            margin = DETECT_ROI_MARGIN_MIN_PX + norm * (
                DETECT_ROI_MARGIN_MAX_PX - DETECT_ROI_MARGIN_MIN_PX
            )
            self.detect_roi_margin_px = float(round(margin))
            self.last_inference_time = 0.0
            self.status = f"ROI margin: {int(round(self.detect_roi_margin_px))} px"
        elif name == "seek_window":
            self.seek_window_sec = SEEK_WINDOW_MIN_SEC + norm * (SEEK_WINDOW_MAX_SEC - SEEK_WINDOW_MIN_SEC)
            self.status = f"Seek window: {self.seek_window_sec:.1f}s"
        elif name == "seek_decay":
            self.seek_decay_sec = SEEK_DECAY_MIN_SEC + norm * (SEEK_DECAY_MAX_SEC - SEEK_DECAY_MIN_SEC)
            self.status = f"Seek decay: {self.seek_decay_sec:.1f}s"

    def load_runtime_ui_settings(self) -> None:
        if not self.runtime_settings_path.exists():
            return
        try:
            root = yaml.safe_load(self.runtime_settings_path.read_text(encoding="utf-8")) or {}
            if not isinstance(root, dict):
                return
            view_mode = str(root.get("view_mode", self.view_mode)).strip()
            if view_mode in ("RGB", "Binarized", "Depth"):
                self.view_mode = view_mode
            elif view_mode.lower() == "rgb":
                self.view_mode = "RGB"
            elif view_mode.lower() in ("binarized", "binary", "bw"):
                self.view_mode = "Binarized"
            elif view_mode.lower() == "depth":
                self.view_mode = "Depth"
            if "overlay_enabled" in root:
                self.publish_overlay = as_bool(root["overlay_enabled"])
            if "debug_images_enabled" in root:
                self.debug_images_enabled = as_bool(root["debug_images_enabled"])
            if "yolo_conf" in root:
                self.yolo_conf = float(np.clip(float(root["yolo_conf"]), 0.0, 1.0))
            if "detect_roi_margin_px" in root:
                self.detect_roi_margin_px = float(np.clip(
                    float(root["detect_roi_margin_px"]),
                    DETECT_ROI_MARGIN_MIN_PX,
                    DETECT_ROI_MARGIN_MAX_PX,
                ))
            if "seek_window_sec" in root:
                self.seek_window_sec = float(np.clip(
                    float(root["seek_window_sec"]), SEEK_WINDOW_MIN_SEC, SEEK_WINDOW_MAX_SEC))
            if "seek_decay_sec" in root:
                self.seek_decay_sec = float(np.clip(
                    float(root["seek_decay_sec"]), SEEK_DECAY_MIN_SEC, SEEK_DECAY_MAX_SEC))
        except Exception as exc:
            self.get_logger().warn(f"Failed to load YOLO detect runtime UI settings: {exc}")

    def save_runtime_ui_settings(self) -> None:
        try:
            self.runtime_settings_path.parent.mkdir(parents=True, exist_ok=True)
            data = {
                "view_mode": self.view_mode,
                "overlay_enabled": bool(self.publish_overlay),
                "debug_images_enabled": bool(self.debug_images_enabled),
                "yolo_conf": float(self.yolo_conf),
                "detect_roi_margin_px": float(self.detect_roi_margin_px),
                "seek_window_sec": float(self.seek_window_sec),
                "seek_decay_sec": float(self.seek_decay_sec),
            }
            tmp_path = self.runtime_settings_path.with_suffix(".tmp")
            tmp_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
            tmp_path.replace(self.runtime_settings_path)
        except Exception as exc:
            self.get_logger().warn(f"Failed to save YOLO detect runtime UI settings: {exc}")

    def save_selected_model_export_file(self) -> None:
        try:
            self.selected_model_export_path.parent.mkdir(parents=True, exist_ok=True)
            model_path = ""
            if self.active_profile is not None and self.active_profile.model_path:
                model_path = self.active_profile.model_path
            self.selected_model_export_path.write_text(model_path + "\n", encoding="utf-8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to save selected YOLO model file: {exc}")

    def selected_profile_path_text(self) -> str:
        if self.active_profile is not None:
            return str(self.active_profile.path)
        if self.selected_profile_path is not None:
            return str(self.selected_profile_path)
        return ""

    def publish_selected_profile(self) -> None:
        msg = String()
        msg.data = self.selected_profile_path_text()
        self.selected_profile_pub.publish(msg)

    def save_selected_profile_export_file(self) -> None:
        try:
            self.selected_profile_export_path.parent.mkdir(parents=True, exist_ok=True)
            self.selected_profile_export_path.write_text(
                self.selected_profile_path_text() + "\n",
                encoding="utf-8",
            )
        except Exception as exc:
            self.get_logger().warn(f"Failed to save selected item detect profile file: {exc}")

    def seek_is_on(self) -> bool:
        return bool(self.seek_mode_active or self.seek_result_latched)

    def handle_seek(self, request, response):
        del request
        self.set_seek_mode(not self.seek_is_on())
        response.success = True
        response.message = "Seek armed" if self.seek_mode_active else "Seek cancelled"
        return response

    def handle_seek_complete(self, request, response):
        del request
        self.set_seek_mode(False, "Seek released by item pick")
        response.success = True
        response.message = "Seek released by item pick"
        return response

    def handle_repick(self, request, response):
        del request
        if self.seek_mode_active:
            response.success = False
            response.message = "Repick rejected: seek is already acquiring"
            return response
        if not self.seek_result_latched:
            response.success = False
            response.message = "Repick rejected: no latched seek result"
            return response

        self.seek_auto_yolo_active = True
        self.set_yolo_enabled(True)
        self.seek_mode_active = True
        self.seek_result_latched = False
        self.seek_started_time = time.monotonic()
        self.last_seek_pose = None
        self.last_seek_pose_time = 0.0
        self.clear_seek_debug_capture()
        self.last_inference_time = 0.0
        self.status = "Repick requested: YOLO ON momentarily to reacquire item pose"
        response.success = True
        response.message = self.status
        return response

    def turn_off_seek_yolo(self, message: str) -> None:
        self.seek_auto_yolo_active = False
        if self.yolo_enabled:
            self.set_yolo_enabled(False, message)
        else:
            self.status = message

    def handle_seek_status(self, request, response):
        del request
        active = self.seek_is_on()
        response.success = active
        response.message = "Seek: ON" if active else "Seek: OFF"
        return response

    def handle_go_to_teach(self, request, response):
        del request
        response.success = self.request_go_to_teach()
        response.message = self.status
        return response

    def set_seek_mode(self, active: bool, message: Optional[str] = None) -> None:
        active = bool(active)
        if active:
            self.seek_auto_yolo_active = True
            self.set_yolo_enabled(True)
        self.seek_mode_active = active
        self.seek_result_latched = False
        if active:
            self.seek_started_time = time.monotonic()
            self.last_seek_pose = None
            self.last_seek_pose_time = 0.0
            self.clear_seek_debug_capture()
            self.last_inference_time = 0.0
            self.status = message or "YOLO ON | Seek armed; choosing highest-confidence mask"
        else:
            self.seek_started_time = 0.0
            self.last_seek_pose = None
            self.last_seek_pose_time = 0.0
            self.clear_seek_debug_capture()
            self.selected_pose = None
            self.peak_pixel = None
            self.turn_off_seek_yolo(message or "Seek cancelled | YOLO OFF")

    def can_go_to_teach(self) -> bool:
        profile = self.active_profile
        return (
            profile is not None and
            profile.has_teach_joints and
            len(profile.teach_joints_deg) >= 6 and
            not self.go_to_teach_in_progress
        )

    def can_delete_profile(self) -> bool:
        return self.active_profile is not None and not self.go_to_teach_in_progress

    def delete_pending(self) -> bool:
        profile = self.active_profile
        return (
            profile is not None and
            self.pending_delete_profile == profile.path and
            time.monotonic() < self.pending_delete_deadline
        )

    def delete_profile_label(self) -> str:
        return "Confirm Delete" if self.delete_pending() else "Delete Item"

    def request_delete_profile(self) -> bool:
        profile = self.active_profile
        if profile is None:
            self.status = "Delete Item: select a YOLO model"
            return False
        self.delete_confirm_active = True
        self.status = f"Confirm delete {profile.item_name} model"
        return False

    def confirm_delete_profile(self) -> bool:
        profile = self.active_profile
        if profile is None:
            self.delete_confirm_active = False
            self.status = "Delete Item: select a YOLO model"
            return False
        try:
            deleted_name = profile.path.name
            self.pt_model = None
            self.pt_class_names = {}
            if profile.path.exists() and profile.path.is_file() and self.is_safe_model_path(profile.path):
                profile.path.unlink()
            deleted_models = self.delete_model_artifacts(profile)
            if deleted_models:
                deleted_message = f"Deleted profile {deleted_name} and model folder"
            else:
                deleted_message = f"Deleted profile {deleted_name}; model folder missing or outside model root"
        except Exception as exc:
            self.status = f"Delete Item failed: {exc}"
            self.get_logger().warn(self.status)
            self.pending_delete_profile = None
            self.pending_delete_deadline = 0.0
            self.delete_confirm_active = False
            return False
        self.pending_delete_profile = None
        self.pending_delete_deadline = 0.0
        self.delete_confirm_active = False
        self.active_profile = None
        self.selected_profile_index = -1
        self.selected_model_path = None
        self.selected_profile_path = None
        self.pt_model = None
        self.pt_class_names = {}
        self.latest_detections = []
        self.latest_detection_poses = []
        self.selected_detection = None
        self.selected_pose = None
        self.peak_pixel = None
        self.refresh_profiles()
        self.save_selected_model_export_file()
        self.save_selected_profile_export_file()
        self.publish_selected_profile()
        self.status = deleted_message if self.profiles else f"{deleted_message}; no profiles remaining"
        return True

    def delete_model_artifacts(self, profile: ItemProfile) -> List[Path]:
        deleted: List[Path] = []
        candidates: List[Path] = []
        for path_text in [profile.model_dir]:
            if path_text:
                candidates.append(resolve_path(path_text))
        for path_text in [profile.model_path, profile.model_pt_path]:
            if not path_text:
                continue
            path = resolve_path(path_text)
            candidates.append(path.parent if path.suffix else path)

        seen = set()
        for candidate in candidates:
            try:
                resolved = candidate.resolve()
            except Exception:
                continue
            if resolved in seen:
                continue
            seen.add(resolved)
            if not self.is_safe_model_path(resolved):
                continue
            try:
                if resolved.is_dir():
                    shutil.rmtree(resolved)
                    deleted.append(resolved)
                elif resolved.is_file():
                    resolved.unlink()
                    deleted.append(resolved)
            except Exception as exc:
                self.get_logger().warn(f"Delete Item: could not delete model path {resolved}: {exc}")
        return deleted

    def is_safe_model_path(self, path: Path) -> bool:
        try:
            resolved_root = self.model_root.resolve()
            resolved_path = path.resolve()
            if resolved_path == resolved_root:
                return False
            resolved_path.relative_to(resolved_root)
            return True
        except Exception:
            return False

    def request_go_to_teach(self) -> bool:
        profile = self.active_profile
        if profile is None:
            self.status = "Go to Teach: select an item profile"
            return False
        if not profile.has_teach_joints or len(profile.teach_joints_deg) < 6:
            self.status = "Go to Teach: selected profile has no teach joints"
            return False
        if self.go_to_teach_in_progress:
            self.status = "Go to Teach: command in progress"
            return False
        if not self.movj_client.service_is_ready():
            self.status = "Go to Teach: MovJ service not ready"
            return False

        request = MovJ.Request()
        request.mode = True
        request.a = float(profile.teach_joints_deg[0])
        request.b = float(profile.teach_joints_deg[1])
        request.c = float(profile.teach_joints_deg[2])
        request.d = float(profile.teach_joints_deg[3])
        request.e = float(profile.teach_joints_deg[4])
        request.f = float(profile.teach_joints_deg[5])
        request.param_value = []

        self.go_to_teach_in_progress = True
        self.status = "Go to Teach: sending MovJ"
        self.get_logger().info(
            f"Go to Teach MovJ -> {self.movj_service_name} with joints (deg): "
            f"[{request.a:.3f}, {request.b:.3f}, {request.c:.3f}, "
            f"{request.d:.3f}, {request.e:.3f}, {request.f:.3f}]"
        )
        future = self.movj_client.call_async(request)
        future.add_done_callback(self.handle_go_to_teach_done)
        return True

    def handle_go_to_teach_done(self, future) -> None:
        try:
            response = future.result()
            ok = response is not None and response.res != -1
            if ok:
                self.status = "Go to Teach: MovJ accepted"
                self.get_logger().info(
                    f"Go to Teach: MovJ accepted "
                    f"(res={response.res}, robot_return={response.robot_return})"
                )
            else:
                self.status = "Go to Teach: MovJ failed"
                res = response.res if response is not None else -999
                robot_return = response.robot_return if response is not None else "null"
                self.get_logger().warn(
                    f"Go to Teach: MovJ failed (res={res}, robot_return={robot_return})"
                )
        except Exception as exc:
            self.status = "Go to Teach: MovJ error"
            self.get_logger().warn(f"Go to Teach: MovJ call failed: {exc}")
        finally:
            self.go_to_teach_in_progress = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ItemDetectYoloNode()
    try:
        rclpy.spin(node)
    finally:
        if node.start_visualization:
            cv2.destroyWindow(WINDOW_NAME)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
