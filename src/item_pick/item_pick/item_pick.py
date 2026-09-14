import json
import math
import os
import re
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field, replace
from enum import Enum
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import CP, DO, InverseKin, MovJ, MovJIO, MovL, MovLIO, SpeedFactor, Stop, TrayInterceptStart
try:
    from dobot_msgs_v4.srv import GetCurrentCommandId
except ImportError:  # pragma: no cover - only hit when tests use stale generated ROS msgs
    GetCurrentCommandId = None
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from edge_station import function_timing
from edge_station.motion_profiles import (
    FAST_SETTLE_SEC,
    FIXED_HOME_JOINTS_DEG,
    FIXED_HOME_MOVJ_PARAM,
    MAX_SAFE_ACCJ_PERCENT,
    MAX_SAFE_ACCL_PERCENT,
    MAX_SAFE_CP_PERCENT,
    MAX_SAFE_SPEED_FACTOR_PERCENT,
    MAX_SAFE_SPEEDJ_PERCENT,
    MAX_SAFE_SPEEDL_PERCENT,
    PICK_SETTLE_TIME_SCALE_MAX_SAFE_DEFAULT,
)
from edge_station.motion_validation import validate_joint_degrees, validate_tcp_pose

try:
    import yaml
except Exception:
    yaml = None

from .teach_io import (
    LayoutError as TeachLayoutError,
    SchemaError as TeachSchemaError,
    load_subject_yaml,
    read_subject_params,
    read_subject_section,
    resolve_single_teach_file,
    write_subject_section,
)


SERVICE_ROOT_DEFAULT = '/dobot_bringup_ros2/srv'
GRIPPER_DO_SERVICE_DEFAULT = f'{SERVICE_ROOT_DEFAULT}/DO'
INVERSE_KIN_SERVICE_DEFAULT = f'{SERVICE_ROOT_DEFAULT}/InverseKin'
# Short wait for InverseKin availability: if the service is down we
# proceed rather than blocking the pick worker for 10 s per pose.
INVERSE_KIN_SERVICE_TIMEOUT_SEC = 2.0


class IkCheckResult(Enum):
    REACHABLE = 'reachable'
    UNREACHABLE = 'unreachable'
    UNAVAILABLE = 'unavailable'


DEFAULT_ACC_PERCENT = MAX_SAFE_ACCL_PERCENT
PICK_DESCENT_SPEED_PERCENT = 5
PICK_DI_TRIGGER_RETRACT_SPEED_PERCENT = 70
PICK_DI_TRIGGER_RETRACT_Z_UP_MM = 80.0
PICK_NORMAL_SPEED_FACTOR_PERCENT = MAX_SAFE_SPEED_FACTOR_PERCENT
FINAL_Z_UP_SPEED_PERCENT = MAX_SAFE_SPEEDL_PERCENT
PICK_DI_FINAL_Z_UP_ACCEL_PERCENT = 5
PICK_CP_PERCENT = MAX_SAFE_CP_PERCENT
TCP_FIELDS = ('x', 'y', 'z', 'rx', 'ry', 'rz')
ITEM_POSE_TOPIC = 'bin_seek_pose'
ITEM_POSE_ARRAY_TOPIC_DEFAULT = 'bin_item_poses'
ROBOT_JOINT_TOPIC_DEFAULT = 'joint_states_robot'
DI_STATUS_TOPIC_DEFAULT = '/dobot_bringup_ros2/DIStatus_200mS'
HELD_ITEM_DI_INDEX_DEFAULT = 1
HELD_ITEM_DI_ACTIVE_HIGH_DEFAULT = True
HELD_ITEM_DI_STALE_SEC_DEFAULT = 2.0
LATEST_ITEM_QA_SNAPSHOT_MARKER = 'latest_item_detection_snapshot.json'
PICK_SETTLE_TIME_SCALE_DEFAULT = PICK_SETTLE_TIME_SCALE_MAX_SAFE_DEFAULT
PICK_SETTLE_TIME_SCALE_MIN = 0.0
PICK_SETTLE_TIME_SCALE_MAX = 1.0
PICK_RUNTIME_SETTLING_TIME_SEC = 0.0
PICK_DI_RETRY_COUNT_DEFAULT = 1
PICK_QA_CONFIRM_TIMEOUT_SEC = 20.0
PICK_QA_CONFIRM_POLL_SEC = 0.05
PICK_QA_FINAL_Z_REACH_TIMEOUT_SEC = 12.0
PICK_QA_FINAL_Z_REACH_TOLERANCE_MM = 8.0
PICK_QA_STABLE_SEC = FAST_SETTLE_SEC
PICK_COMMAND_ID_WAIT_TIMEOUT_SEC = 45.0
PICK_COMMAND_ID_POLL_SEC = 0.05
PICK_MONITORED_DESCENT_TIMEOUT_SEC = 45.0
PICK_DI_POLL_SEC = 1.0 / 60.0
PICK_DI_STOP_SETTLE_SEC = 0.10
PICK_DI_BOTTOM_LEEWAY_SEC = 0.30
COMMAND_ID_WRAP_LOW_WATERMARK = 3
COMMAND_ID_WRAP_NEAR_TARGET_WINDOW = 8
CAMERA_BIN_VALID_POSE_ATTEMPTS_DEFAULT = 15
ITEM_CAMERA_TF_READY_WAIT_SEC = 2.0
ITEM_CAMERA_TF_RETRY_SLEEP_SEC = 0.05
ROBOT_GOAL_FRAME_DEFAULT = 'base_link'
ROBOT_GRIPPER_FRAME_DEFAULT = 'Link6'
CALIBRATED_CAMERA_FRAME_DEFAULT = 'calibrated_camera_link'
POST_STOP_MOVL_GOAL_DEBUG_FRAME_DEFAULT = 'item_goal_tcp'
POST_STOP_MOVL_GOAL_NOMINAL_DEBUG_FRAME_DEFAULT = 'item_movel_goal_nominal_tcp'
POST_STOP_MOVL_GOAL_TOOL_OFFSET_DEBUG_FRAME_DEFAULT = 'item_movel_goal_tool_offset'
POST_STOP_MOVL_GOAL_TOOL_AXIS_X_TIP_FRAME_DEFAULT = 'item_movel_goal_tool_axis_x_tip'
POST_STOP_MOVL_GOAL_TOOL_AXIS_Y_TIP_FRAME_DEFAULT = 'item_movel_goal_tool_axis_y_tip'
POST_STOP_MOVL_GOAL_TOOL_AXIS_Z_TIP_FRAME_DEFAULT = 'item_movel_goal_tool_axis_z_tip'
# Frame published at Link6 location *implied* by the goal TCP after the
# Dobot controller subtracts its internal Tool N (movl ``tool`` arg).
# RViz models the robot only up to Link6, so this lets the operator see
# where the physical flange will land even though the MovL goal pose
# refers to the controller's TCP. Default name matches what the lost
# TF-fix work used.
POST_STOP_MOVL_GOAL_FLANGE_TCP_DEBUG_FRAME_DEFAULT = 'item_movel_goal_flange_tcp'
ITEM_POSE_WATCH_TIMEOUT_SEC = 120.0
ITEM_POSE_WATCH_TIMEOUT_MIN = 120.0
ITEM_POSE_WATCH_TIMEOUT_MAX = 180.0
# The detector may replay a short-lived fixed-camera prefetch, but its header
# remains the sensor acquisition time. Reject older or unclocked coordinates at
# the motion boundary even if a publisher or mixed-version detector replays
# them after this node arms.
ITEM_POSE_MAX_ACQUISITION_AGE_SEC_DEFAULT = 3.0
ITEM_POSE_MAX_ACQUISITION_AGE_SEC_MIN = 0.1
ITEM_POSE_MAX_ACQUISITION_AGE_SEC_MAX = 10.0
ITEM_POSE_FUTURE_STAMP_TOLERANCE_SEC = 0.25
ITEM_POSE_MOTION_NOISE_FLOOR_MM_S = 5.0
LEGACY_EE_INTERCEPT_SPEED_COMPAT = float(MAX_SAFE_SPEEDL_PERCENT)
POST_STOP_X_OFFSET_MIN = -50.0
POST_STOP_X_OFFSET_MAX = 400.0
POST_STOP_Y_OFFSET_MIN = -50.0
POST_STOP_Y_OFFSET_MAX = 300.0
POST_STOP_Z_OFFSET_MIN = 50.0
POST_STOP_Z_OFFSET_MAX = 200.0
TRAY_PLACE_ADDITIONAL_Z_MM_DEFAULT = 50.0
TRAY_PLACE_ADDITIONAL_Z_MM_MIN = 0.0
TRAY_PLACE_ADDITIONAL_Z_MM_MAX = 150.0
FINAL_Z_UP_MIN = 50.0
FINAL_Z_UP_MAX = 300.0
FINAL_Z_UP_DEFAULT = 200.0
SETTLING_TIME_MIN_SEC = 0.0
SETTLING_TIME_MAX_SEC = 2.0
SETTLING_TIME_LEGACY_DEFAULT_SEC = PICK_RUNTIME_SETTLING_TIME_SEC
PRE_PICK_SETTLING_LEGACY_DEFAULT_SEC = PICK_RUNTIME_SETTLING_TIME_SEC
SETTLING_TIME_DEFAULT_SEC = PICK_RUNTIME_SETTLING_TIME_SEC
PRE_PICK_SETTLING_DEFAULT_SEC = PICK_RUNTIME_SETTLING_TIME_SEC
TOOL_OFFSET_TRANSLATION_MIN_MM = -500.0
TOOL_OFFSET_TRANSLATION_MAX_MM = 500.0
TOOL_OFFSET_ROTATION_MIN_DEG = -180.0
TOOL_OFFSET_ROTATION_MAX_DEG = 180.0
COMMAND_HYSTERESIS_MIN_SEC = 0.1
COMMAND_HYSTERESIS_MAX_SEC = 1.0
COMMAND_HYSTERESIS_DEFAULT_SEC = 0.1
LOCKED_MAX_SPEED_MM_S = LEGACY_EE_INTERCEPT_SPEED_COMPAT
TCP_GOAL_REACHED_TOLERANCE_MM = 5.0
TCP_GOAL_WAIT_TIMEOUT_SEC = 20.0
MANUAL_RELEASE_PULSE_MS = 300
# Bounded per-move reach timeout for the drop-last-item return path so the
# drop_last_item service can never hang the executor waiting on a TCP goal
# that is never reached. Kept generous enough for bounded stop cleanup.
DROP_MOVE_REACH_TIMEOUT_SEC = 12.0
DROP_RETURN_RETRACT_Z_MM = 60.0
DROP_RELEASE_ABOVE_PICK_Z_MM = 50.0
POST_PICK_DROP_RELEASE_ABOVE_PICK_Z_MM = 50.0
DROP_SAFE_HOVER_ABOVE_PICK_Z_MM = 150.0
DROP_RELEASE_IO_DISTANCE_PERCENT = 100
# During post-pick IO-drop recovery, release before the +50 mm endpoint so the
# item is already venting/open while the robot finishes the descent.
POST_PICK_DROP_RELEASE_IO_DISTANCE_PERCENT = 80
DROP_RETRACT_NEUTRAL_IO_DISTANCE_PERCENT = 0
# When Stop/Return races an active item pick, drop_last_item waits briefly
# for the pick worker to finish its own retract/final-Z-up path before it
# attempts the cached held-item return. If the worker does not become idle
# inside this bounded window, the bridge must not home blindly.
DROP_LAST_BUSY_WAIT_TIMEOUT_SEC = 20.0
DROP_LAST_BUSY_WAIT_POLL_SEC = 0.05
GOAL_TF_LOOKUP_TIMEOUT_SEC_DEFAULT = 0.2
CAMERA_BIN_SAFE_MARGIN_MM_DEFAULT = 0.0
START_SEQUENCE_SERVICE_DEFAULT = 'item_pick/start_sequence'
TRACK_SERVICE_DEFAULT = 'item_pick/track'
TRACK_STATUS_SERVICE_DEFAULT = 'item_pick/track_status'
TRACK_CANCEL_SERVICE_DEFAULT = 'item_pick/cancel_track'
LIFECYCLE_CANCEL_SERVICE_DEFAULT = 'item_pick/lifecycle_cancel'
LIFECYCLE_RESUME_SERVICE_DEFAULT = 'item_pick/lifecycle_resume'
RETRY_CACHED_CANDIDATE_SERVICE_DEFAULT = 'item_pick/retry_cached_candidate'
POST_PICK_DROP_RECOVER_CACHED_SERVICE_DEFAULT = 'item_pick/post_pick_drop_recover_cached'
CLEAR_CANDIDATE_CACHE_SERVICE_DEFAULT = 'item_pick/clear_candidate_cache'
ITEM_SEEK_COMPLETE_SERVICE_DEFAULT = 'item_detect/seek_complete'
ITEM_SEEK_STATUS_SERVICE_DEFAULT = 'item_detect/seek_status'
ITEM_REPICK_SERVICE_DEFAULT = 'item_detect/repick'
ITEM_REPICK_RESPONSE_TIMEOUT_SEC = 1.0
ITEM_REPICK_STATUS_RESPONSE_TIMEOUT_SEC = 0.5
ITEM_REPICK_STATUS_POLL_SEC = 0.05
ITEM_REPICK_STATUS_LOG_INTERVAL_SEC = 2.0
ITEM_GO_TO_TEACH_SERVICE_DEFAULT = 'item_detect/go_to_teach'
ITEM_GO_TO_TEACH_STATUS_SERVICE_DEFAULT = 'item_detect/go_to_teach_status'
ITEM_GO_TO_TEACH_RESPONSE_TIMEOUT_SEC = 2.0
ITEM_GO_TO_TEACH_STATUS_TIMEOUT_SEC = 5.0
ITEM_GO_TO_TEACH_STABILITY_SEC = FAST_SETTLE_SEC
ITEM_GO_TO_TEACH_STABILITY_TIMEOUT_SEC = 30.0
ITEM_GO_TO_TEACH_STABILITY_PREWAIT_SEC = FAST_SETTLE_SEC
ROBOT_LINEAR_MOVE_EPS_MM = 1.0
ROBOT_ROT_MOVE_EPS_DEG = 1.0
ROBOT_TCP_STALE_SEC = 1.0
# A camera seek is only safe when every robot joint is proven at the fixed
# home target from recent feedback.  Keep these values aligned with the
# pick-cycle fixed-home verifier.
ROBOT_JOINT_STALE_SEC = 1.0
FIXED_HOME_JOINT_TOLERANCE_DEG = 0.75
FIXED_HOME_JOINT_VERIFY_TIMEOUT_SEC = 2.0
FIXED_HOME_JOINT_VERIFY_POLL_SEC = 0.05
# Gripper-relax Trigger service used by the conveyor-driven lifecycle
# preflight to drop the gripper to a neutral state (DO1, DO2, DO3 all 0)
# before motion, e.g. when an operator clicks Start Run / Stop Run on
# the dashboard. The pick FSM uses _send_do() for the same outputs;
# this service exists so headless lifecycle callers (the edge bridge)
# can request a relax without re-implementing DO bookkeeping.
GRIPPER_RELAX_SERVICE_DEFAULT = 'item_pick/relax_gripper'
ACTIVE_ITEM_PROFILE_MARKER = '.active_item_profile.json'
# Drop-last-item Trigger service used by conveyor-driven Stop/Reset cleanup
# to return a held item to its cached base-frame pick pose before release.
# Clear-last-target lets the lifecycle invalidate that cached goal at safe
# boundaries (prepare_start, after a successful place).
DROP_LAST_ITEM_SERVICE_DEFAULT = 'item_pick/drop_last_item'
CLEAR_LAST_TARGET_SERVICE_DEFAULT = 'item_pick/clear_last_target'
TOOL_OFFSET_PREVIEW_PARENT_FRAME_DEFAULT = 'Link6'
TOOL_OFFSET_PREVIEW_FRAME_DEFAULT = 'item_pick_tool_offset_preview'
TOOL_OFFSET_PREVIEW_AXIS_X_TIP_FRAME_DEFAULT = 'item_pick_tool_offset_preview_axis_x_tip'
TOOL_OFFSET_PREVIEW_AXIS_Y_TIP_FRAME_DEFAULT = 'item_pick_tool_offset_preview_axis_y_tip'
TOOL_OFFSET_PREVIEW_AXIS_Z_TIP_FRAME_DEFAULT = 'item_pick_tool_offset_preview_axis_z_tip'
# All per-item persistence (runtime UI state, tool offsets, "active profile"
# pointer) now lives in the pick:/ros__parameters: section of the single
# items/<id>.yaml that the node finds in profiles_dir. There is no separate
# ~/.ros/item_pick_runtime_settings.json, no item_detect_selected_profile.txt,
# and no <item>_tool.yaml sidecar.
TEACH_FILES_ROOT = Path.home() / 'CATARM' / 'apps' / 'edge-station-node' / 'teach_files'
DEFAULT_ITEMS_DIR = TEACH_FILES_ROOT / 'items'
DEFAULT_BINS_DIR = TEACH_FILES_ROOT / 'bins'
DEFAULT_QA_SNAPSHOT_ROOT = Path(
    os.environ.get(
        'DETECTION_SNAPSHOT_ROOT',
        '/root/CATARM/apps/edge-station-node/detection_snapshots',
    )
) / 'qa_logs'
PLATFORM_CALIBRATION_FILE_DEFAULT = str(
    TEACH_FILES_ROOT / 'platform' / 'platform_calibration_robot_platform_1.yaml'
)
# Eye-in-hand calibration (parent Link6 -> child camera). Its translation is
# the camera origin expressed in the Link6/flange frame, i.e. the camera
# offset used to predict where the wrist-mounted camera lands for a candidate
# pick orientation. Sourced from the calibration file so the camera-inside-bin
# check does not depend on a runtime TF that a fixed-camera tree may not carry.
EYE_IN_HAND_CALIBRATION_FILE_DEFAULT = str(
    TEACH_FILES_ROOT / 'calibration' / 'eye_in_hand.yaml'
)
FIXED_CAMERA_CALIBRATION_FILE_DEFAULT = str(
    TEACH_FILES_ROOT / 'calibration' / 'cr10_orbbec335.yaml'
)
EYE_IN_HAND_CALIBRATION_LOAD_RETRY_COUNT = 3
EYE_IN_HAND_CALIBRATION_LOAD_RETRY_SLEEP_SEC = 0.05
PICK_SECTION_KEY = 'pick'
TEACH_SECTION_KEY = 'teach'
ITEM_SUBJECT_KIND = 'item'
BIN_SUBJECT_KIND = 'bin'
RUNTIME_SETTINGS_SAVE_DEBOUNCE_MS = 250
GOAL_TF_DIAG_AXIS_LENGTH_MM = 60.0
GRIPPER_DO_CLOSE_INDEX = 1
GRIPPER_DO_OPEN_INDEX = 2
GRIPPER_DO_SUCTION_INDEX = 3
GRIPPER_DO_PURGE_INDEX = 4
FIXED_HOME_COMMAND_ID_MISSING_MARKER = 'fixed_home_command_id_missing=1'
MOVLIO_DO_MODE_PERCENT = 0
MOVJIO_APPROACH_OPEN_DISTANCE_PERCENT = 50
MOVLIO_PICKUP_START_DISTANCE_PERCENT = 0
MOVLIO_RETRACT_CLOSE_DISTANCE_PERCENT = 90
MOVLIO_NO_DI_PURGE_OFF_DISTANCE_PERCENT = 50
# A single fixed-camera acquisition may expose many masks, but production
# recovery is deliberately bounded: try the three best distinct candidates
# from that acquisition, then get the arm to fixed home before asking the
# camera for a fresh scene.
PICK_MAX_YOLO_CANDIDATES_PER_FRAME = 3
FAILED_PICK_AREA_TOPIC_DEFAULT = 'item_detect/failed_pick_area'
# Pick approach height is intentionally fixed here.
PICK_FIXED_APPROACH_Z_UP_MM = 50.0
# Optional canary-only split of the final pick descent.  The legacy path keeps
# the full +50 mm -> pick motion at 5%; the opt-in path traverses only the
# upper, observed-clear part faster and preserves the final monitored segment
# at the existing 5% speed.  Production templates deliberately leave this
# disabled until per-arm braking/DI logs have been reviewed.
PICK_TWO_STAGE_DESCENT_ENABLED_ENV = 'EDGE_PICK_TWO_STAGE_DESCENT_ENABLED'
PICK_FAST_DESCENT_SWITCH_Z_UP_MM_ENV = 'EDGE_PICK_FAST_DESCENT_SWITCH_Z_UP_MM'
PICK_FAST_DESCENT_SPEED_PERCENT_ENV = 'EDGE_PICK_FAST_DESCENT_SPEED_PERCENT'
PICK_TWO_STAGE_DESCENT_ENABLED_DEFAULT = False
PICK_FAST_DESCENT_SWITCH_Z_UP_MM_DEFAULT = 30.0
PICK_FAST_DESCENT_SWITCH_Z_UP_MM_MIN = 20.0
PICK_FAST_DESCENT_SWITCH_Z_UP_MM_MAX = 45.0
PICK_FAST_DESCENT_SPEED_PERCENT_DEFAULT = 50
USE_FINGERS_DEFAULT = True
GRAB_ON_PICK_DEFAULT = False
RELAX_FINGERS_ON_PICK_DEFAULT = False


def display_name_for_item_block(item_block: object) -> str:
    """Return a human-readable label from a loaded ``item:`` block.

    The single items yaml carries ``item: {id, display_name?}``; the
    GUI/log surfaces prefer ``display_name`` and fall back to ``id``.
    Returns the empty-state placeholder when no usable identity is
    present so callers do not need to special-case None.
    """
    if not isinstance(item_block, dict):
        return 'No active item teach'
    display = str(item_block.get('display_name', '') or '').strip()
    if display:
        return display
    item_id = str(item_block.get('id', '') or '').strip()
    return item_id or 'No active item teach'


def clamp_settling_time_sec(value: float) -> float:
    return max(SETTLING_TIME_MIN_SEC, min(SETTLING_TIME_MAX_SEC, float(value)))


def clamp_item_pose_watch_timeout_sec(value: float) -> float:
    """Keep item_pick alive beyond the cycle's 90 s detector-off owner."""
    return max(
        ITEM_POSE_WATCH_TIMEOUT_MIN,
        min(ITEM_POSE_WATCH_TIMEOUT_MAX, float(value)),
    )


def clamp_pick_settle_time_scale(value: float) -> float:
    return max(
        PICK_SETTLE_TIME_SCALE_MIN,
        min(PICK_SETTLE_TIME_SCALE_MAX, float(value)),
    )


def scaled_pick_settling_time_sec(
    value: float,
    scale: float = PICK_SETTLE_TIME_SCALE_DEFAULT,
) -> float:
    return clamp_settling_time_sec(
        clamp_settling_time_sec(value) * clamp_pick_settle_time_scale(scale)
    )


def pick_two_stage_descent_enabled_from_env() -> bool:
    raw = os.environ.get(
        PICK_TWO_STAGE_DESCENT_ENABLED_ENV,
        '1' if PICK_TWO_STAGE_DESCENT_ENABLED_DEFAULT else '0',
    )
    return str(raw or '').strip().lower() in {'1', 'true', 'yes', 'on'}


def _pick_two_stage_float_from_env(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or not str(raw).strip():
        return float(default)
    try:
        return float(raw)
    except (TypeError, ValueError):
        # NaN is rejected by ``validate_pick_two_stage_descent_config``.  This
        # is fail-closed: an opt-in with a malformed tuning value can never
        # silently fall back to an enabled motion profile.
        return float('nan')


def validate_pick_two_stage_descent_config(
    enabled: bool,
    switch_z_up_mm: float,
    fast_speed_percent: float,
) -> tuple[bool, str]:
    """Validate the canary descent envelope without clamping unsafe input."""
    if not enabled:
        return True, 'disabled'
    if not math.isfinite(float(switch_z_up_mm)):
        return False, 'fast descent switch height is not a finite number'
    if not (
        PICK_FAST_DESCENT_SWITCH_Z_UP_MM_MIN
        <= float(switch_z_up_mm)
        <= PICK_FAST_DESCENT_SWITCH_Z_UP_MM_MAX
    ):
        return False, (
            'fast descent switch height must be within '
            f'{PICK_FAST_DESCENT_SWITCH_Z_UP_MM_MIN:.0f}..'
            f'{PICK_FAST_DESCENT_SWITCH_Z_UP_MM_MAX:.0f} mm above pick'
        )
    if float(switch_z_up_mm) >= PICK_FIXED_APPROACH_Z_UP_MM:
        return False, 'fast descent switch must stay below the fixed approach height'
    if not math.isfinite(float(fast_speed_percent)):
        return False, 'fast descent speed is not a finite number'
    if not (
        PICK_DESCENT_SPEED_PERCENT
        < float(fast_speed_percent)
        <= 100.0
    ):
        return False, (
            'fast descent speed must be greater than the legacy descent '
            f'({PICK_DESCENT_SPEED_PERCENT}%) and at most 100%'
        )
    return True, 'valid'


@dataclass
class ItemPickSnapshot:
    tcp_values: dict[str, float] = field(default_factory=lambda: {name: 0.0 for name in TCP_FIELDS})
    tcp_stamp: float | None = None
    busy: bool = False
    armed: bool = False
    action_text: str = 'Ready'
    item_pose_seq: int = 0
    has_last_item: bool = False


@dataclass(frozen=True)
class ItemPoseTarget:
    position_mm: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]
    frame_id: str
    stamp_sec: float
    # Preserve the exact ROS builtin_interfaces/Time identity.  Float seconds
    # are still kept for acquisition-age calculations, but they cannot safely
    # identify one camera frame at epoch-sized values because sub-microsecond
    # bits may be rounded away.
    acquisition_stamp_ns: int | None = None


@dataclass(frozen=True)
class PredictedGoal:
    x_mm: float
    y_mm: float
    z_mm: float
    rx_deg: float
    ry_deg: float
    rz_deg: float
    source_frame_id: str
    lead_time_sec: float
    item_age_sec: float
    item_speed_base_mmps: float
    nominal_x_mm: float = 0.0
    nominal_y_mm: float = 0.0
    nominal_z_mm: float = 0.0
    nominal_rx_deg: float = 0.0
    nominal_ry_deg: float = 0.0
    nominal_rz_deg: float = 0.0
    orientation_choice: str = 'preferred'
    camera_safety_message: str = ''


@dataclass(frozen=True)
class CachedPickAttempt:
    candidates: tuple[ItemPoseTarget, ...]
    next_candidate_index: int
    post_speed_mm_s: float
    x_offset_mm: float
    y_offset_mm: float
    z_offset_mm: float
    use_fingers: bool
    grab_on_pick: bool
    final_z_up_mm: float
    pre_pick_settling_time_sec: float
    pick_settling_time_sec: float
    tool_offset_x_mm: float
    tool_offset_y_mm: float
    tool_offset_z_mm: float
    tool_offset_rx_deg: float
    tool_offset_ry_deg: float
    tool_offset_rz_deg: float
    relax_fingers_on_pick: bool = False
    successful_candidate_index: int | None = None


@dataclass(frozen=True)
class BinCameraSafetyArea:
    profile_path: Path
    bin_teach_path: Path
    bin_frame_id: str
    base_to_bin_translation_m: tuple[float, float, float]
    base_to_bin_rotation_xyzw: tuple[float, float, float, float]
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    margin_m: float


@dataclass(frozen=True)
class CameraBinCandidateChoice:
    selected_index: int
    message: str
    valid: bool = True


class ItemPickNode(Node):
    def __init__(self) -> None:
        super().__init__('item_pick')
        self._lock = threading.Lock()
        self._snapshot = ItemPickSnapshot()
        self._joint_lock = threading.Lock()
        self._latest_joint_names: tuple[str, ...] = ()
        self._latest_joint_positions_rad: tuple[float, ...] | None = None
        self._last_joint_receive_time_monotonic = 0.0
        self._joint_feedback_seq = 0
        self._di_lock = threading.Lock()
        self._latest_di_bits: int | None = None
        self._last_di_receive_time = 0.0
        self._last_di_message = 'no DI status received'
        self._pick_di_session_lock = threading.Lock()
        self._active_pick_di_session: dict[str, object] | None = None
        self._item_pose_seq = 0
        self._item_pose_watch_armed = False
        self._item_pose_watch_seq_floor = 0
        self._item_pose_watch_deadline_monotonic = 0.0
        self._item_pose_watch_stop_dispatched = False
        self._item_pose_watch_generation = 0
        self._item_pose_skip_warned = False
        self._camera_bin_pose_reject_count = 0
        self._last_camera_bin_reject_reason = ''
        # Default TF-only ("troubleshoot") mode comes from the
        # ``tf_only_default`` ROS parameter so the production launch can
        # ship `False` (arm actually moves on a detected pose) while the
        # lab GUI launch can ship `True` (preview TF only). Until this
        # parameter existed the constructor hardcoded ``True``, which
        # made every Trigger-driven `item_pick/track` call -- the path
        # used by ``pick_cycle`` -- arm in preview-only mode and silently
        # refuse to move the arm even when the detection succeeded.
        self._item_pose_watch_tf_only_mode = bool(
            self.declare_parameter('tf_only_default', False).value
        )
        self._item_pose_watch_timeout_sec = ITEM_POSE_WATCH_TIMEOUT_SEC
        item_pose_max_age_sec = float(
            self.declare_parameter(
                'item_pose_max_acquisition_age_sec',
                ITEM_POSE_MAX_ACQUISITION_AGE_SEC_DEFAULT,
            ).value
        )
        if not math.isfinite(item_pose_max_age_sec):
            item_pose_max_age_sec = ITEM_POSE_MAX_ACQUISITION_AGE_SEC_DEFAULT
        self._item_pose_max_acquisition_age_sec = max(
            ITEM_POSE_MAX_ACQUISITION_AGE_SEC_MIN,
            min(ITEM_POSE_MAX_ACQUISITION_AGE_SEC_MAX, item_pose_max_age_sec),
        )
        self._item_pose_array_topic = str(
            self.declare_parameter(
                'item_pose_array_topic',
                ITEM_POSE_ARRAY_TOPIC_DEFAULT,
            ).value
        ).strip() or ITEM_POSE_ARRAY_TOPIC_DEFAULT
        self._failed_pick_area_topic = str(
            self.declare_parameter(
                'failed_pick_area_topic',
                FAILED_PICK_AREA_TOPIC_DEFAULT,
            ).value
        ).strip() or FAILED_PICK_AREA_TOPIC_DEFAULT
        self._latest_item_pose_candidates: tuple[ItemPoseTarget, ...] = ()
        self._latest_item_pose_candidates_seq = 0
        self._active_pick_workers = 0
        self._cancel_requested = False
        # Persistent gate used only by Stop/Reset/operator lifecycle cleanup.
        # Ordinary in-run cancellation keeps using ``_cancel_requested`` so
        # same-session camera/DI retries remain available.  Once latched, no
        # new pick worker may start until the bridge explicitly resumes after
        # a successful, verified-home prepare/recover handshake.
        self._lifecycle_stop_latched = False
        self._lifecycle_epoch = 0
        self._manual_stop_inflight = False
        self._manual_release_inflight = False
        self._goal_tf_diagnose_inflight = False
        self._post_stop_movel_speed_mm_s = LOCKED_MAX_SPEED_MM_S
        self._post_stop_x_offset_mm = 0.0
        self._post_stop_y_offset_mm = 0.0
        self._post_stop_z_offset_mm = 100.0
        self._use_fingers = USE_FINGERS_DEFAULT
        self._grab_on_pick = GRAB_ON_PICK_DEFAULT
        self._relax_fingers_on_pick = RELAX_FINGERS_ON_PICK_DEFAULT
        self._active_profile_pick_error = ''
        self._final_z_up_mm = FINAL_Z_UP_DEFAULT
        self._tool_offset_x_mm = 0.0
        self._tool_offset_y_mm = 0.0
        self._tool_offset_z_mm = 0.0
        self._tool_offset_rx_deg = 0.0
        self._tool_offset_ry_deg = 0.0
        self._tool_offset_rz_deg = 0.0
        self._pick_settle_time_scale = clamp_pick_settle_time_scale(
            float(self.declare_parameter(
                'pick_settle_time_scale',
                PICK_SETTLE_TIME_SCALE_DEFAULT,
            ).value)
        )
        legacy_settling_time_raw_sec = clamp_settling_time_sec(
            float(self.declare_parameter(
                'settling_time_sec',
                SETTLING_TIME_LEGACY_DEFAULT_SEC,
            ).value)
        )
        self.declare_parameter(
            'pre_pick_settling_time_sec',
            PRE_PICK_SETTLING_LEGACY_DEFAULT_SEC,
        )
        self.declare_parameter(
            'pick_settling_time_sec',
            legacy_settling_time_raw_sec,
        )
        self._pre_pick_settling_time_sec = PICK_RUNTIME_SETTLING_TIME_SEC
        self._pick_settling_time_sec = PICK_RUNTIME_SETTLING_TIME_SEC
        requested_two_stage_descent = bool(
            self.declare_parameter(
                'pick_two_stage_descent_enabled',
                pick_two_stage_descent_enabled_from_env(),
            ).value
        )
        try:
            requested_fast_switch_z_up_mm = float(
                self.declare_parameter(
                    'pick_fast_descent_switch_z_up_mm',
                    _pick_two_stage_float_from_env(
                        PICK_FAST_DESCENT_SWITCH_Z_UP_MM_ENV,
                        PICK_FAST_DESCENT_SWITCH_Z_UP_MM_DEFAULT,
                    ),
                ).value
            )
        except (TypeError, ValueError):
            requested_fast_switch_z_up_mm = float('nan')
        try:
            requested_fast_descent_speed_percent = float(
                self.declare_parameter(
                    'pick_fast_descent_speed_percent',
                    _pick_two_stage_float_from_env(
                        PICK_FAST_DESCENT_SPEED_PERCENT_ENV,
                        PICK_FAST_DESCENT_SPEED_PERCENT_DEFAULT,
                    ),
                ).value
            )
        except (TypeError, ValueError):
            requested_fast_descent_speed_percent = float('nan')
        two_stage_valid, two_stage_reason = validate_pick_two_stage_descent_config(
            requested_two_stage_descent,
            requested_fast_switch_z_up_mm,
            requested_fast_descent_speed_percent,
        )
        self._pick_two_stage_descent_enabled = bool(
            requested_two_stage_descent and two_stage_valid
        )
        self._pick_fast_descent_switch_z_up_mm = (
            requested_fast_switch_z_up_mm
            if math.isfinite(requested_fast_switch_z_up_mm)
            else PICK_FAST_DESCENT_SWITCH_Z_UP_MM_DEFAULT
        )
        self._pick_fast_descent_speed_percent = (
            requested_fast_descent_speed_percent
            if math.isfinite(requested_fast_descent_speed_percent)
            else float(PICK_FAST_DESCENT_SPEED_PERCENT_DEFAULT)
        )
        if requested_two_stage_descent and not two_stage_valid:
            self.get_logger().error(
                'Two-stage pick descent requested with invalid configuration; '
                f'using legacy {PICK_DESCENT_SPEED_PERCENT}% descent: '
                f'{two_stage_reason}'
            )
        self._command_hysteresis_sec = max(
            COMMAND_HYSTERESIS_MIN_SEC,
            min(
                COMMAND_HYSTERESIS_MAX_SEC,
                float(
                    self.declare_parameter(
                        'command_hysteresis_sec',
                        COMMAND_HYSTERESIS_DEFAULT_SEC,
                    ).value
                ),
            ),
        )
        self._publish_goal_debug_tf = bool(
            self.declare_parameter('publish_goal_debug_tf', True).value
        )
        self._robot_goal_frame_id = str(
            self.declare_parameter('robot_goal_frame_id', ROBOT_GOAL_FRAME_DEFAULT).value
        ).strip() or ROBOT_GOAL_FRAME_DEFAULT
        self._robot_gripper_frame_id = str(
            self.declare_parameter('robot_gripper_frame_id', ROBOT_GRIPPER_FRAME_DEFAULT).value
        ).strip() or ROBOT_GRIPPER_FRAME_DEFAULT
        self._camera_safety_frame_id = str(
            self.declare_parameter('camera_safety_frame_id', CALIBRATED_CAMERA_FRAME_DEFAULT).value
        ).strip() or CALIBRATED_CAMERA_FRAME_DEFAULT
        self._prefer_camera_inside_bin = bool(
            self.declare_parameter('prefer_camera_inside_bin', True).value
        )
        # When True, pick-pose orientation selection always lands on the
        # candidate that keeps the wrist camera inside the bin ROI; if neither
        # candidate is fully inside, the one closest to the ROI is used.
        self._require_camera_inside_bin = bool(
            self.declare_parameter('require_camera_inside_bin', True).value
        )
        self._camera_bin_safe_margin_mm = max(
            0.0,
            float(self.declare_parameter(
                'camera_bin_safe_margin_mm',
                CAMERA_BIN_SAFE_MARGIN_MM_DEFAULT,
            ).value),
        )
        try:
            self._camera_bin_valid_pose_max_attempts = max(
                1,
                int(
                    self.declare_parameter(
                        'camera_bin_valid_pose_attempts',
                        CAMERA_BIN_VALID_POSE_ATTEMPTS_DEFAULT,
                    ).value
                ),
            )
        except (TypeError, ValueError):
            self._camera_bin_valid_pose_max_attempts = CAMERA_BIN_VALID_POSE_ATTEMPTS_DEFAULT
        self._eye_in_hand_calibration_file = str(
            self.declare_parameter(
                'eye_in_hand_calibration_file',
                EYE_IN_HAND_CALIBRATION_FILE_DEFAULT,
            ).value
        ).strip() or EYE_IN_HAND_CALIBRATION_FILE_DEFAULT
        self._fixed_camera_calibration_file = str(
            self.declare_parameter(
                'fixed_camera_calibration_file',
                FIXED_CAMERA_CALIBRATION_FILE_DEFAULT,
            ).value
        ).strip() or FIXED_CAMERA_CALIBRATION_FILE_DEFAULT
        # Camera origin in the Link6/flange frame, parsed from the eye-in-hand
        # calibration. If unavailable at startup it is retried when the
        # camera-bin safety check runs; runtime TF is intentionally not used.
        self._camera_offset_gripper_m_calibration = (
            self._load_eye_in_hand_camera_offset_m()
        )
        self._post_stop_movel_goal_debug_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_debug_frame_id',
                POST_STOP_MOVL_GOAL_DEBUG_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_DEBUG_FRAME_DEFAULT
        self._post_stop_movel_goal_nominal_debug_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_nominal_debug_frame_id',
                POST_STOP_MOVL_GOAL_NOMINAL_DEBUG_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_NOMINAL_DEBUG_FRAME_DEFAULT
        self._post_stop_movel_goal_tool_offset_debug_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_tool_offset_debug_frame_id',
                POST_STOP_MOVL_GOAL_TOOL_OFFSET_DEBUG_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_TOOL_OFFSET_DEBUG_FRAME_DEFAULT
        self._post_stop_movel_goal_tool_axis_x_tip_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_tool_axis_x_tip_frame_id',
                POST_STOP_MOVL_GOAL_TOOL_AXIS_X_TIP_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_TOOL_AXIS_X_TIP_FRAME_DEFAULT
        self._post_stop_movel_goal_tool_axis_y_tip_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_tool_axis_y_tip_frame_id',
                POST_STOP_MOVL_GOAL_TOOL_AXIS_Y_TIP_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_TOOL_AXIS_Y_TIP_FRAME_DEFAULT
        self._post_stop_movel_goal_tool_axis_z_tip_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_tool_axis_z_tip_frame_id',
                POST_STOP_MOVL_GOAL_TOOL_AXIS_Z_TIP_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_TOOL_AXIS_Z_TIP_FRAME_DEFAULT
        self._tool_offset_preview_parent_frame_id = str(
            self.declare_parameter(
                'tool_offset_preview_parent_frame_id',
                TOOL_OFFSET_PREVIEW_PARENT_FRAME_DEFAULT,
            ).value
        ).strip() or TOOL_OFFSET_PREVIEW_PARENT_FRAME_DEFAULT
        self._tool_offset_preview_frame_id = str(
            self.declare_parameter(
                'tool_offset_preview_frame_id',
                TOOL_OFFSET_PREVIEW_FRAME_DEFAULT,
            ).value
        ).strip() or TOOL_OFFSET_PREVIEW_FRAME_DEFAULT
        self._tool_offset_preview_axis_x_tip_frame_id = str(
            self.declare_parameter(
                'tool_offset_preview_axis_x_tip_frame_id',
                TOOL_OFFSET_PREVIEW_AXIS_X_TIP_FRAME_DEFAULT,
            ).value
        ).strip() or TOOL_OFFSET_PREVIEW_AXIS_X_TIP_FRAME_DEFAULT
        self._tool_offset_preview_axis_y_tip_frame_id = str(
            self.declare_parameter(
                'tool_offset_preview_axis_y_tip_frame_id',
                TOOL_OFFSET_PREVIEW_AXIS_Y_TIP_FRAME_DEFAULT,
            ).value
        ).strip() or TOOL_OFFSET_PREVIEW_AXIS_Y_TIP_FRAME_DEFAULT
        self._tool_offset_preview_axis_z_tip_frame_id = str(
            self.declare_parameter(
                'tool_offset_preview_axis_z_tip_frame_id',
                TOOL_OFFSET_PREVIEW_AXIS_Z_TIP_FRAME_DEFAULT,
            ).value
        ).strip() or TOOL_OFFSET_PREVIEW_AXIS_Z_TIP_FRAME_DEFAULT
        self._tool_offset_x_mm = self._clamp_tool_offset_translation_mm(
            float(self.declare_parameter('tool_offset_x_mm', 0.0).value),
        )
        self._tool_offset_y_mm = self._clamp_tool_offset_translation_mm(
            float(self.declare_parameter('tool_offset_y_mm', 0.0).value),
        )
        self._tool_offset_z_mm = self._clamp_tool_offset_translation_mm(
            float(self.declare_parameter('tool_offset_z_mm', 0.0).value),
        )
        self._tool_offset_rx_deg = self._clamp_tool_offset_rotation_deg(
            float(self.declare_parameter('tool_offset_rx_deg', 0.0).value),
        )
        self._tool_offset_ry_deg = self._clamp_tool_offset_rotation_deg(
            float(self.declare_parameter('tool_offset_ry_deg', 0.0).value),
        )
        self._tool_offset_rz_deg = self._clamp_tool_offset_rotation_deg(
            float(self.declare_parameter('tool_offset_rz_deg', 0.0).value),
        )
        self._goal_tf_lookup_timeout_sec = max(
            0.01,
            float(self.declare_parameter(
                'goal_tf_lookup_timeout_sec',
                GOAL_TF_LOOKUP_TIMEOUT_SEC_DEFAULT,
            ).value),
        )
        self._start_sequence_service_name = str(
            self.declare_parameter(
                'start_sequence_service',
                START_SEQUENCE_SERVICE_DEFAULT,
            ).value
        ).strip() or START_SEQUENCE_SERVICE_DEFAULT
        self._track_service_name = str(
            self.declare_parameter(
                'track_service',
                TRACK_SERVICE_DEFAULT,
            ).value
        ).strip() or TRACK_SERVICE_DEFAULT
        self._track_status_service_name = str(
            self.declare_parameter(
                'track_status_service',
                TRACK_STATUS_SERVICE_DEFAULT,
            ).value
        ).strip() or TRACK_STATUS_SERVICE_DEFAULT
        self._track_cancel_service_name = str(
            self.declare_parameter(
                'track_cancel_service',
                TRACK_CANCEL_SERVICE_DEFAULT,
            ).value
        ).strip() or TRACK_CANCEL_SERVICE_DEFAULT
        self._lifecycle_cancel_service_name = str(
            self.declare_parameter(
                'lifecycle_cancel_service',
                LIFECYCLE_CANCEL_SERVICE_DEFAULT,
            ).value
        ).strip() or LIFECYCLE_CANCEL_SERVICE_DEFAULT
        self._lifecycle_resume_service_name = str(
            self.declare_parameter(
                'lifecycle_resume_service',
                LIFECYCLE_RESUME_SERVICE_DEFAULT,
            ).value
        ).strip() or LIFECYCLE_RESUME_SERVICE_DEFAULT
        self._retry_cached_candidate_service_name = str(
            self.declare_parameter(
                'retry_cached_candidate_service',
                RETRY_CACHED_CANDIDATE_SERVICE_DEFAULT,
            ).value
        ).strip() or RETRY_CACHED_CANDIDATE_SERVICE_DEFAULT
        self._post_pick_drop_recover_cached_service_name = str(
            self.declare_parameter(
                'post_pick_drop_recover_cached_service',
                POST_PICK_DROP_RECOVER_CACHED_SERVICE_DEFAULT,
            ).value
        ).strip() or POST_PICK_DROP_RECOVER_CACHED_SERVICE_DEFAULT
        self._clear_candidate_cache_service_name = str(
            self.declare_parameter(
                'clear_candidate_cache_service',
                CLEAR_CANDIDATE_CACHE_SERVICE_DEFAULT,
            ).value
        ).strip() or CLEAR_CANDIDATE_CACHE_SERVICE_DEFAULT
        self._item_seek_complete_service_name = str(
            self.declare_parameter(
                'item_seek_complete_service',
                ITEM_SEEK_COMPLETE_SERVICE_DEFAULT,
            ).value
        ).strip() or ITEM_SEEK_COMPLETE_SERVICE_DEFAULT
        self._item_seek_status_service_name = str(
            self.declare_parameter(
                'item_seek_status_service',
                ITEM_SEEK_STATUS_SERVICE_DEFAULT,
            ).value
        ).strip() or ITEM_SEEK_STATUS_SERVICE_DEFAULT
        self._item_repick_service_name = str(
            self.declare_parameter(
                'item_repick_service',
                ITEM_REPICK_SERVICE_DEFAULT,
            ).value
        ).strip() or ITEM_REPICK_SERVICE_DEFAULT
        self._item_go_to_teach_service_name = str(
            self.declare_parameter(
                'item_go_to_teach_service',
                ITEM_GO_TO_TEACH_SERVICE_DEFAULT,
            ).value
        ).strip() or ITEM_GO_TO_TEACH_SERVICE_DEFAULT
        self._item_go_to_teach_status_service_name = str(
            self.declare_parameter(
                'item_go_to_teach_status_service',
                ITEM_GO_TO_TEACH_STATUS_SERVICE_DEFAULT,
            ).value
        ).strip() or ITEM_GO_TO_TEACH_STATUS_SERVICE_DEFAULT
        self._gripper_do_service_name = str(
            self.declare_parameter(
                'gripper_do_service',
                GRIPPER_DO_SERVICE_DEFAULT,
            ).value
        ).strip() or GRIPPER_DO_SERVICE_DEFAULT
        self._robot_joint_topic = str(
            self.declare_parameter(
                'robot_joint_topic',
                ROBOT_JOINT_TOPIC_DEFAULT,
            ).value
        ).strip() or ROBOT_JOINT_TOPIC_DEFAULT
        self._di_status_topic = str(
            self.declare_parameter(
                'di_status_topic',
                DI_STATUS_TOPIC_DEFAULT,
            ).value
        ).strip() or DI_STATUS_TOPIC_DEFAULT
        try:
            self._held_item_di_index = max(
                1,
                int(
                    self.declare_parameter(
                        'held_item_di_index',
                        HELD_ITEM_DI_INDEX_DEFAULT,
                    ).value
                ),
            )
        except (TypeError, ValueError):
            self._held_item_di_index = HELD_ITEM_DI_INDEX_DEFAULT
        held_active_raw = self.declare_parameter(
            'held_item_di_active_high',
            HELD_ITEM_DI_ACTIVE_HIGH_DEFAULT,
        ).value
        if isinstance(held_active_raw, bool):
            self._held_item_di_active_high = held_active_raw
        else:
            self._held_item_di_active_high = (
                str(held_active_raw).strip().lower() in {'1', 'true', 'yes', 'on'}
            )
        try:
            self._held_item_di_stale_sec = max(
                0.1,
                float(
                    self.declare_parameter(
                        'held_item_di_stale_sec',
                        HELD_ITEM_DI_STALE_SEC_DEFAULT,
                    ).value
                ),
            )
        except (TypeError, ValueError):
            self._held_item_di_stale_sec = HELD_ITEM_DI_STALE_SEC_DEFAULT
        try:
            self._pick_di_retry_count = max(
                0,
                int(
                    self.declare_parameter(
                        'pick_di_retry_count',
                        PICK_DI_RETRY_COUNT_DEFAULT,
                    ).value
                ),
            )
        except (TypeError, ValueError):
            self._pick_di_retry_count = PICK_DI_RETRY_COUNT_DEFAULT
        self._gripper_relax_service_name = str(
            self.declare_parameter(
                'gripper_relax_service',
                GRIPPER_RELAX_SERVICE_DEFAULT,
            ).value
        ).strip() or GRIPPER_RELAX_SERVICE_DEFAULT
        self._drop_last_item_service_name = str(
            self.declare_parameter(
                'drop_last_item_service',
                DROP_LAST_ITEM_SERVICE_DEFAULT,
            ).value
        ).strip() or DROP_LAST_ITEM_SERVICE_DEFAULT
        self._clear_last_target_service_name = str(
            self.declare_parameter(
                'clear_last_target_service',
                CLEAR_LAST_TARGET_SERVICE_DEFAULT,
            ).value
        ).strip() or CLEAR_LAST_TARGET_SERVICE_DEFAULT

        self._mov_j_client = self.create_client(MovJ, f'{SERVICE_ROOT_DEFAULT}/MovJ')
        self._mov_jio_client = self.create_client(MovJIO, f'{SERVICE_ROOT_DEFAULT}/MovJIO')
        self._mov_l_client = self.create_client(MovL, f'{SERVICE_ROOT_DEFAULT}/MovL')
        self._mov_lio_client = self.create_client(MovLIO, f'{SERVICE_ROOT_DEFAULT}/MovLIO')
        if GetCurrentCommandId is not None:
            self._get_current_command_id_client = self.create_client(
                GetCurrentCommandId,
                f'{SERVICE_ROOT_DEFAULT}/GetCurrentCommandId',
            )
        else:
            self._get_current_command_id_client = None
        self._inverse_kin_client = self.create_client(InverseKin, INVERSE_KIN_SERVICE_DEFAULT)
        self._stop_client = self.create_client(Stop, f'{SERVICE_ROOT_DEFAULT}/Stop')
        self._cp_client = self.create_client(CP, f'{SERVICE_ROOT_DEFAULT}/CP')
        self._speed_factor_client = self.create_client(SpeedFactor, f'{SERVICE_ROOT_DEFAULT}/SpeedFactor')
        self._do_client = self.create_client(DO, self._gripper_do_service_name)
        self._failed_pick_area_pub = self.create_publisher(
            PoseStamped,
            self._failed_pick_area_topic,
            10,
        )
        self._item_seek_complete_client = self.create_client(
            Trigger,
            self._item_seek_complete_service_name,
        )
        self._item_seek_status_client = self.create_client(
            Trigger,
            self._item_seek_status_service_name,
        )
        self._item_repick_client = self.create_client(
            Trigger,
            self._item_repick_service_name,
        )
        self._item_go_to_teach_client = self.create_client(
            Trigger,
            self._item_go_to_teach_service_name,
        )
        self._item_go_to_teach_status_client = self.create_client(
            Trigger,
            self._item_go_to_teach_status_service_name,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._goal_tf_static_broadcaster = StaticTransformBroadcaster(self)
        self._goal_static_tf_by_child: dict[str, TransformStamped] = {}
        self._fixed_camera_tf_warned_keys: set[str] = set()
        self._publish_fixed_camera_calibration_tf(log_success=True)
        # Single-file teach layout: profiles_dir holds exactly one
        # ``items/<id>.yaml`` and bin_teach_dir holds exactly one
        # ``bins/<id>.yaml`` at runtime. CATARM swaps them atomically;
        # restart the node to pick up a new file. The legacy
        # item_detect_selected_profile.txt / runtime/sidecar files are
        # gone, see teach_io.py for the single-file resolution helper.
        profiles_dir_text = str(
            self.declare_parameter('profiles_dir', str(DEFAULT_ITEMS_DIR)).value
        ).strip() or str(DEFAULT_ITEMS_DIR)
        bin_teach_dir_text = str(
            self.declare_parameter('bin_teach_dir', str(DEFAULT_BINS_DIR)).value
        ).strip() or str(DEFAULT_BINS_DIR)
        self._platform_calibration_file = str(
            self.declare_parameter(
                'platform_calibration_file',
                PLATFORM_CALIBRATION_FILE_DEFAULT,
            ).value
            or PLATFORM_CALIBRATION_FILE_DEFAULT
        ).strip()
        self._items_dir = Path(profiles_dir_text).expanduser()
        self._bins_dir = Path(bin_teach_dir_text).expanduser()
        self._qa_snapshot_root = Path(
            str(
                self.declare_parameter(
                    'qa_snapshot_root',
                    str(DEFAULT_QA_SNAPSHOT_ROOT),
                ).value
            )
        ).expanduser()
        # Mirror the Dobot controller's internal Tool N TCP (the value
        # ``tool=N`` selects in MovL/MovLIO commands). RViz only models
        # the robot up to Link6 (the flange), so when the controller
        # applies its internal Tool TCP the physical flange lands at
        # ``goal_tcp * (dobot_tool_offset)^-1``. The
        # ``item_movel_goal_flange_tcp`` debug TF visualises that flange
        # pose so the operator can compare it to the URDF without
        # guessing where ``Link6`` ends up. Launch defaults seed the
        # values; the active item yaml ``pick:/ros__parameters:``
        # overrides them on a per-item basis.
        self._dobot_tool_offset_x_mm = self._clamp_tool_offset_translation_mm(
            float(self.declare_parameter('dobot_tool_offset_x_mm', 0.0).value),
        )
        self._dobot_tool_offset_y_mm = self._clamp_tool_offset_translation_mm(
            float(self.declare_parameter('dobot_tool_offset_y_mm', 0.0).value),
        )
        self._dobot_tool_offset_z_mm = self._clamp_tool_offset_translation_mm(
            float(self.declare_parameter('dobot_tool_offset_z_mm', 0.0).value),
        )
        self._dobot_tool_offset_rx_deg = self._clamp_tool_offset_rotation_deg(
            float(self.declare_parameter('dobot_tool_offset_rx_deg', 0.0).value),
        )
        self._dobot_tool_offset_ry_deg = self._clamp_tool_offset_rotation_deg(
            float(self.declare_parameter('dobot_tool_offset_ry_deg', 0.0).value),
        )
        self._dobot_tool_offset_rz_deg = self._clamp_tool_offset_rotation_deg(
            float(self.declare_parameter('dobot_tool_offset_rz_deg', 0.0).value),
        )
        self._post_stop_movel_goal_flange_tcp_debug_frame_id = str(
            self.declare_parameter(
                'post_stop_movel_goal_flange_tcp_debug_frame_id',
                POST_STOP_MOVL_GOAL_FLANGE_TCP_DEBUG_FRAME_DEFAULT,
            ).value
        ).strip() or POST_STOP_MOVL_GOAL_FLANGE_TCP_DEBUG_FRAME_DEFAULT
        self._active_item_file: Path | None = None
        self._active_item_file_mtime_ns: int | None = None
        self._active_item_id: str | None = None
        self._active_item_display_name: str | None = None
        self._active_item_dimensions_mm: dict[str, float] | None = None
        self._active_profile_saved_tool_offsets: dict[str, object] | None = None
        self._track_trigger_handler = None
        self._last_item_target: ItemPoseTarget | None = None
        self._cached_pick_attempt: CachedPickAttempt | None = None
        # Stable metadata exposed by item_pick/track_status. The phase starts
        # at one for the original fixed-camera acquisition and advances when
        # the detector acknowledges a fresh post-home repick. The DI counter
        # is monotonic so consumers can distinguish a new confirmation from a
        # stale status poll.
        self._item_redetection_phase = 0
        # Physical attempts are deliberately distinct from detector phases.
        # They advance only after a monitored pick descent is accepted.
        self._physical_pick_attempt = 0
        self._physical_repick_attempt = 0
        self._last_pick_motion_started = False
        self._last_pick_candidate_number = 0
        self._item_di_confirmed_for_current_track = False
        self._item_di_confirmed_generation = 0
        self._last_fixed_home_movj_command_id: int | None = None
        self._last_fixed_home_joint_feedback_seq_baseline: int | None = None
        self._inverse_kin_unavailable_warning_keys: set[str] = set()
        # Resolved base-frame pick goal from the most recent successful
        # pick goal computation. Stop/Reset uses this safety-critical cache to
        # return a held item to its original pick location before retracting.
        # Guarded by ``self._lock``.
        self._last_base_drop_release_goal: PredictedGoal | None = None
        self._drop_last_inflight = False
        self.create_subscription(ToolVectorActual, 'dobot_msgs_v4/msg/ToolVectorActual', self._tcp_callback, 10)
        self.create_subscription(JointState, self._robot_joint_topic, self._joint_state_callback, 10)
        self.create_subscription(String, self._di_status_topic, self._di_status_callback, 10)
        self.create_subscription(PoseArray, self._item_pose_array_topic, self._item_pose_array_callback, 10)
        self.create_subscription(PoseStamped, ITEM_POSE_TOPIC, self._item_pose_callback, 10)
        # Blocking return/recovery services run in their own reentrant group so
        # motion-service futures can still resolve on another executor thread.
        self._drop_cb_group = ReentrantCallbackGroup()
        self._start_sequence_service = self.create_service(
            TrayInterceptStart,
            self._start_sequence_service_name,
            self._start_sequence_service_callback,
        )
        self._track_service = self.create_service(
            Trigger,
            self._track_service_name,
            self._track_service_callback,
        )
        self._track_status_service = self.create_service(
            Trigger,
            self._track_status_service_name,
            self._track_status_service_callback,
        )
        self._track_cancel_service = self.create_service(
            Trigger,
            self._track_cancel_service_name,
            self._track_cancel_service_callback,
            callback_group=self._drop_cb_group,
        )
        self._lifecycle_cancel_service = self.create_service(
            Trigger,
            self._lifecycle_cancel_service_name,
            self._lifecycle_cancel_service_callback,
            callback_group=self._drop_cb_group,
        )
        self._lifecycle_resume_service = self.create_service(
            Trigger,
            self._lifecycle_resume_service_name,
            self._lifecycle_resume_service_callback,
            callback_group=self._drop_cb_group,
        )
        self._retry_cached_candidate_service = self.create_service(
            Trigger,
            self._retry_cached_candidate_service_name,
            self._retry_cached_candidate_service_callback,
        )
        self._post_pick_drop_recover_cached_service = self.create_service(
            Trigger,
            self._post_pick_drop_recover_cached_service_name,
            self._post_pick_drop_recover_cached_service_callback,
            callback_group=self._drop_cb_group,
        )
        self._clear_candidate_cache_service = self.create_service(
            Trigger,
            self._clear_candidate_cache_service_name,
            self._clear_candidate_cache_service_callback,
        )
        self._gripper_relax_service = self.create_service(
            Trigger,
            self._gripper_relax_service_name,
            self._gripper_relax_service_callback,
        )
        # The drop-last-item callback blocks until its held-item
        # return/retract motion finishes, which means it waits on MovJ/DO service
        # futures. Those futures are resolved by the executor; with the
        # node now spun on a MultiThreadedExecutor, placing this service
        # in its own ReentrantCallbackGroup lets the executor keep
        # resolving the motion client responses (default group) on
        # another thread while this callback is parked, avoiding a
        # single-threaded-executor deadlock.
        self._drop_last_item_service = self.create_service(
            Trigger,
            self._drop_last_item_service_name,
            self._drop_last_item_service_callback,
            callback_group=self._drop_cb_group,
        )
        self._clear_last_target_service = self.create_service(
            Trigger,
            self._clear_last_target_service_name,
            self._clear_last_target_service_callback,
        )
        self.get_logger().info(
            'Item pick mode configured: MovJIO pre-Z-up with gripper open and suction off at 50 percent, '
            'MovL approach, '
            f'MovLIO monitored descent at {PICK_DESCENT_SPEED_PERCENT} percent with suction at 0 percent, '
            f'DI trigger Stop on first held sample at poll {PICK_DI_POLL_SEC:.4f}s, '
            f'post-Stop settle {PICK_DI_STOP_SETTLE_SEC:.2f}s, '
            f'pick-pose late DI leeway {PICK_DI_BOTTOM_LEEWAY_SEC:.2f}s, '
            f'DI-trigger retract fixed {PICK_DI_TRIGGER_RETRACT_Z_UP_MM:.0f} mm '
            f'at {PICK_DI_TRIGGER_RETRACT_SPEED_PERCENT} percent, '
            'finger timing selected per item profile, '
            f'no-DI MovLIO final Z-up at {FINAL_Z_UP_SPEED_PERCENT} percent '
            f'with profile-aware finger state + purge at 0 percent and relax + purge off at '
            f'{MOVLIO_NO_DI_PURGE_OFF_DISTANCE_PERCENT} percent, '
            'then next cached same-seek candidate from final Z-up; fixed-home/fresh-seek after cache exhaust.'
        )
        self.get_logger().info(
            'Two-stage pick descent canary: '
            f'enabled={int(self._pick_two_stage_descent_enabled)}, '
            f'switch_z_up={self._pick_fast_descent_switch_z_up_mm:.1f} mm, '
            f'fast_speed={self._pick_fast_descent_speed_percent:.0f}%, '
            f'final_speed={PICK_DESCENT_SPEED_PERCENT}% '
            '(production default is disabled)'
        )
        self.get_logger().info(f'Start item pick sequence service: {self._start_sequence_service_name}')
        self.get_logger().info(f'Track virtual-click service: {self._track_service_name}')
        self.get_logger().info(f'Track armed status service: {self._track_status_service_name}')
        self.get_logger().info(f'Track cancel service: {self._track_cancel_service_name}')
        self.get_logger().info(f'Lifecycle cancel service: {self._lifecycle_cancel_service_name}')
        self.get_logger().info(f'Lifecycle resume service: {self._lifecycle_resume_service_name}')
        self.get_logger().info(f'Retry cached candidate service: {self._retry_cached_candidate_service_name}')
        self.get_logger().info(
            f'Post-pick drop cached recovery service: {self._post_pick_drop_recover_cached_service_name}'
        )
        self.get_logger().info(f'Clear cached candidates service: {self._clear_candidate_cache_service_name}')
        self.get_logger().info(f'Failed pick area topic: {self._failed_pick_area_topic}')
        self.get_logger().info(f'Item detect seek-complete service: {self._item_seek_complete_service_name}')
        self.get_logger().info(f'Item detect seek-status service: {self._item_seek_status_service_name}')
        self.get_logger().info(f'Item detect repick service: {self._item_repick_service_name}')
        self.get_logger().info(f'Item detect go-to-teach service: {self._item_go_to_teach_service_name}')
        self.get_logger().info(
            f'Item detect go-to-teach status service: {self._item_go_to_teach_status_service_name}'
        )
        self.get_logger().info(f'Gripper DO service: {self._gripper_do_service_name}')
        self.get_logger().info(f'Robot joint feedback: {self._robot_joint_topic}')
        self.get_logger().info(
            f'Held-item DI feedback: topic={self._di_status_topic}, '
            f'DI{self._held_item_di_index}, '
            f'active_high={int(self._held_item_di_active_high)}, '
            f'stale_sec={self._held_item_di_stale_sec:.1f}, '
            f'pick_retries={self._pick_di_retry_count}'
        )
        self.get_logger().info(f'Gripper relax service: {self._gripper_relax_service_name}')
        self.get_logger().info(f'Drop last item service: {self._drop_last_item_service_name}')
        self.get_logger().info(f'Clear last target service: {self._clear_last_target_service_name}')
        self.get_logger().info(
            f'Single-file teach: items_dir="{self._items_dir}", bins_dir="{self._bins_dir}", '
            f'platform_calibration_file="{self._platform_calibration_file}", '
            f'fixed_camera_calibration_file="{self._fixed_camera_calibration_file}"'
        )
        self.get_logger().info(
            'Startup defaults: '
            f'wait={self._item_pose_watch_timeout_sec:.0f}s, '
            f'pose_max_acquisition_age={self._item_pose_max_acquisition_age_sec:.3f}s, '
            f'motion_profile=CP{MAX_SAFE_CP_PERCENT}/SF{MAX_SAFE_SPEED_FACTOR_PERCENT}/'
            f'SpeedJ{MAX_SAFE_SPEEDJ_PERCENT}/AccJ{MAX_SAFE_ACCJ_PERCENT}/'
            f'SpeedL{MAX_SAFE_SPEEDL_PERCENT}/AccL{MAX_SAFE_ACCL_PERCENT}, '
            f'legacy_speed_field={self._post_stop_movel_speed_mm_s:.0f}, '
            f'offsets(x={self._post_stop_x_offset_mm:.0f},'
            f'y={self._post_stop_y_offset_mm:.0f},'
            f'z={self._post_stop_z_offset_mm:.0f}) mm, '
            f'tool_offset(x={self._tool_offset_x_mm:.1f},'
            f'y={self._tool_offset_y_mm:.1f},'
            f'z={self._tool_offset_z_mm:.1f},'
            f'rx={self._tool_offset_rx_deg:.1f},'
            f'ry={self._tool_offset_ry_deg:.1f},'
            f'rz={self._tool_offset_rz_deg:.1f}), '
            f'dobot_tool_offset(x={self._dobot_tool_offset_x_mm:.1f},'
            f'y={self._dobot_tool_offset_y_mm:.1f},'
            f'z={self._dobot_tool_offset_z_mm:.1f},'
            f'rx={self._dobot_tool_offset_rx_deg:.1f},'
            f'ry={self._dobot_tool_offset_ry_deg:.1f},'
            f'rz={self._dobot_tool_offset_rz_deg:.1f}), '
            f'fixed_approach_z={PICK_FIXED_APPROACH_Z_UP_MM:.0f} mm, '
            f'two_stage_descent={int(self._pick_two_stage_descent_enabled)}, '
            f'fast_switch_z_up={self._pick_fast_descent_switch_z_up_mm:.1f} mm, '
            f'fast_descent_speed={self._pick_fast_descent_speed_percent:.0f}%, '
            f'use_fingers={int(self._use_fingers)}, '
            f'grab_on_pick={int(self._grab_on_pick)}, '
            f'relax_fingers_on_pick={int(self._relax_fingers_on_pick)}, '
            f'final_z_up={self._final_z_up_mm:.0f} mm, '
            f'settle_scale={self._pick_settle_time_scale:.2f}, '
            f'pre_pick_settle={self._pre_pick_settling_time_sec:.1f}s, '
            f'pick_settle={self._pick_settling_time_sec:.1f}s, '
            f'camera_bin_pose_attempts={self._camera_bin_valid_pose_max_attempts}'
        )
        self._sync_profile_tool_offsets_from_state(force=True)

    def _reset_runtime_state_locked(self, reason: str) -> None:
        self._item_pose_watch_generation += 1
        self._item_pose_watch_armed = False
        self._item_pose_watch_stop_dispatched = False
        self._item_pose_watch_seq_floor = self._item_pose_seq
        self._item_pose_watch_deadline_monotonic = 0.0
        active_workers = int(getattr(self, '_active_pick_workers', 0))
        # Never revive a worker that is still unwinding from Stop/Reset.
        self._cancel_requested = active_workers > 0
        self._cached_pick_attempt = None
        self._snapshot.busy = active_workers > 0
        self._snapshot.action_text = reason

    def _reset_runtime_state(self, reason: str) -> None:
        with self._lock:
            self._reset_runtime_state_locked(reason)

    def snapshot(self) -> ItemPickSnapshot:
        with self._lock:
            return ItemPickSnapshot(
                tcp_values=dict(self._snapshot.tcp_values),
                tcp_stamp=self._snapshot.tcp_stamp,
                busy=self._snapshot.busy,
                armed=self._item_pose_watch_armed,
                action_text=self._snapshot.action_text,
                item_pose_seq=self._item_pose_seq,
                has_last_item=self._last_item_target is not None,
            )

    def _tcp_callback(self, msg: ToolVectorActual) -> None:
        with self._lock:
            self._snapshot.tcp_values['x'] = float(msg.x)
            self._snapshot.tcp_values['y'] = float(msg.y)
            self._snapshot.tcp_values['z'] = float(msg.z)
            self._snapshot.tcp_values['rx'] = float(msg.rx)
            self._snapshot.tcp_values['ry'] = float(msg.ry)
            self._snapshot.tcp_values['rz'] = float(msg.rz)
            self._snapshot.tcp_stamp = time.time()

    def _joint_state_callback(self, msg: JointState) -> None:
        """Cache robot joint feedback with a monotonic receive timestamp.

        The Dobot ``joint_states_robot`` publisher reports radians.  The
        receive timestamp is used instead of the message clock so a stale or
        misconfigured ROS timestamp can never certify fixed home.
        """
        try:
            names = tuple(str(name).strip() for name in msg.name)
            positions = tuple(float(position) for position in msg.position)
        except (TypeError, ValueError):
            names = ()
            positions = ()
        with self._joint_lock:
            self._latest_joint_names = names
            self._latest_joint_positions_rad = positions
            self._last_joint_receive_time_monotonic = time.monotonic()
            self._joint_feedback_seq += 1

    def _joint_feedback_snapshot(
        self,
    ) -> tuple[tuple[str, ...], tuple[float, ...] | None, int, float]:
        joint_lock = getattr(self, '_joint_lock', None)
        if joint_lock is None:
            return (), None, 0, 0.0
        with joint_lock:
            return (
                tuple(getattr(self, '_latest_joint_names', ())),
                getattr(self, '_latest_joint_positions_rad', None),
                int(getattr(self, '_joint_feedback_seq', 0)),
                float(getattr(self, '_last_joint_receive_time_monotonic', 0.0)),
            )

    def _current_joint_degrees_j1_to_j6(
        self,
    ) -> tuple[float, float, float, float, float, float] | None:
        """Return a fail-closed J1..J6 degree mapping from latest feedback."""
        names, positions, _seq, _received = self._joint_feedback_snapshot()
        if positions is None:
            return None

        ordered_rad: tuple[float, ...] | None = None
        if names:
            if len(names) != len(positions):
                return None
            # A named message must carry each expected robot joint exactly
            # once.  Do not fall back to positional order for a partial or
            # reordered named message.
            if any(names.count(f'joint{index}') != 1 for index in range(1, 7)):
                return None
            by_name = {name: position for name, position in zip(names, positions)}
            ordered_rad = tuple(by_name[f'joint{index}'] for index in range(1, 7))
        elif len(positions) == 6:
            # Some JointState publishers omit names.  Six positions are the
            # documented Dobot J1..J6 order; any other length is ambiguous.
            ordered_rad = tuple(positions)

        if ordered_rad is None or any(not math.isfinite(value) for value in ordered_rad):
            return None
        return (
            math.degrees(ordered_rad[0]),
            math.degrees(ordered_rad[1]),
            math.degrees(ordered_rad[2]),
            math.degrees(ordered_rad[3]),
            math.degrees(ordered_rad[4]),
            math.degrees(ordered_rad[5]),
        )

    def _fixed_home_joint_feedback_status(
        self,
        *,
        newer_than_seq: int | None = None,
    ) -> tuple[bool, str]:
        """Check that one recent, complete joint sample proves fixed home."""
        _names, _positions, seq, received = self._joint_feedback_snapshot()
        if received <= 0.0:
            return False, 'fixed home unproven: no robot joint feedback received'
        if newer_than_seq is not None and seq <= int(newer_than_seq):
            return False, (
                'fixed home unproven: joint feedback did not advance after home dispatch '
                f'(seq={seq}, baseline={int(newer_than_seq)})'
            )
        age_sec = max(0.0, time.monotonic() - received)
        if age_sec > ROBOT_JOINT_STALE_SEC:
            return False, (
                'fixed home unproven: robot joint feedback is stale '
                f'(seq={seq}, age={age_sec:.2f}s > {ROBOT_JOINT_STALE_SEC:.2f}s)'
            )

        current_deg = self._current_joint_degrees_j1_to_j6()
        if current_deg is None:
            return False, (
                'fixed home unproven: joint feedback does not contain one finite '
                'J1..J6 value for every robot joint'
            )

        mismatches: list[str] = []
        for index, (current, target) in enumerate(
            zip(current_deg, FIXED_HOME_JOINTS_DEG),
            start=1,
        ):
            delta = self._angle_delta_deg(current, target)
            if delta > FIXED_HOME_JOINT_TOLERANCE_DEG:
                mismatches.append(
                    f'J{index}={current:.3f}deg target={target:.3f}deg delta={delta:.3f}deg'
                )
        if mismatches:
            return False, (
                'fixed home unproven: joint target mismatch '
                f'(tolerance={FIXED_HOME_JOINT_TOLERANCE_DEG:.2f}deg; '
                + '; '.join(mismatches)
                + ')'
            )
        return True, (
            f'fresh fixed-home joint feedback seq={seq}, age={age_sec:.2f}s, '
            f'all six joints within {FIXED_HOME_JOINT_TOLERANCE_DEG:.2f}deg'
        )

    def _wait_for_fresh_fixed_home_joint_feedback(
        self,
        *,
        newer_than_seq: int | None = None,
        timeout_sec: float = FIXED_HOME_JOINT_VERIFY_TIMEOUT_SEC,
    ) -> tuple[bool, str]:
        """Wait briefly for a fresh fixed-home joint proof, then fail closed."""
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        last_message = 'fixed home unproven: no robot joint feedback received'
        while rclpy.ok():
            if self._is_cancel_requested():
                return False, 'fixed home unproven: sequence cancelled during joint verification'
            verified, last_message = self._fixed_home_joint_feedback_status(
                newer_than_seq=newer_than_seq,
            )
            if verified:
                return True, last_message
            if time.monotonic() >= deadline:
                return False, last_message
            time.sleep(FIXED_HOME_JOINT_VERIFY_POLL_SEC)
        return False, 'fixed home unproven: ROS shutdown during joint verification'

    def _require_fixed_home_before_item_detect_repick(self, context: str) -> bool:
        """Final camera interlock immediately before a repick service call."""
        verified, message = self._wait_for_fresh_fixed_home_joint_feedback()
        if not verified:
            failure = f'{context}: refusing item-detect repick; {message}'
            self.get_logger().error(failure)
            self._set_action_text(failure)
            return False
        self.get_logger().info(f'{context}: item-detect repick home interlock passed; {message}')
        return True

    def _di_status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(str(msg.data or '{}'))
            raw_bits = payload.get('digital_input_bits')
            if raw_bits is None:
                raise ValueError('digital_input_bits missing')
            if isinstance(raw_bits, str):
                bits = int(raw_bits.strip(), 0)
            else:
                bits = int(raw_bits)
        except Exception as exc:
            with self._di_lock:
                self._last_di_message = f'bad DI status payload: {exc}'
            return

        with self._di_lock:
            self._latest_di_bits = bits
            self._last_di_receive_time = time.monotonic()
            self._last_di_message = f'{self._di_status_topic}: digital_input_bits={bits}'
            bit_index = max(0, int(getattr(self, '_held_item_di_index', HELD_ITEM_DI_INDEX_DEFAULT)) - 1)
            raw_active = bool(bits & (1 << bit_index))
            active_high = bool(getattr(self, '_held_item_di_active_high', HELD_ITEM_DI_ACTIVE_HIGH_DEFAULT))
            held = raw_active if active_high else not raw_active
            held_message = (
                f'DI{getattr(self, "_held_item_di_index", HELD_ITEM_DI_INDEX_DEFAULT)} '
                f'raw={int(raw_active)} active_high={int(active_high)} '
                f'held={int(held)} bits={bits} age=0.00s'
            )

        self._update_pick_di_session(True, held, held_message)

    def _held_item_state(self) -> tuple[bool, bool | None, str]:
        """Return held-item state from DI feedback.

        Returns ``(available, held, message)``. Unavailable is intentionally
        fail-safe for the pick worker: it may retry, but it must not close and
        leave the bin path as if the item were definitely held.
        """
        with self._di_lock:
            bits = self._latest_di_bits
            stamp = self._last_di_receive_time
            last_message = self._last_di_message

        if bits is None:
            return False, None, f'Held-item DI feedback unavailable: {last_message}'

        age_sec = time.monotonic() - stamp
        if age_sec > self._held_item_di_stale_sec:
            return (
                False,
                None,
                f'Held-item DI feedback stale: age={age_sec:.2f}s '
                f'> {self._held_item_di_stale_sec:.2f}s; last={last_message}',
            )

        bit_index = max(0, int(self._held_item_di_index) - 1)
        raw_active = bool(bits & (1 << bit_index))
        held = raw_active if self._held_item_di_active_high else not raw_active
        return (
            True,
            held,
            f'DI{self._held_item_di_index} raw={int(raw_active)} '
            f'active_high={int(self._held_item_di_active_high)} '
            f'held={int(held)} bits={bits} age={age_sec:.2f}s',
        )

    def _start_pick_di_session(
        self,
        *,
        candidate_number: int,
        candidate_count: int,
        baseline_held: bool | None,
        baseline_message: str,
    ) -> None:
        """Latch held-item DI transitions for one queued pick attempt."""
        if not hasattr(self, '_pick_di_session_lock'):
            self._pick_di_session_lock = threading.Lock()
        with self._pick_di_session_lock:
            self._active_pick_di_session = {
                'candidate_number': int(candidate_number),
                'candidate_count': int(candidate_count),
                'baseline_held': baseline_held,
                'baseline_message': str(baseline_message),
                'saw_not_held': bool(baseline_held is False),
                'triggered': False,
                'trigger_consumed': False,
                'picked': False,
                'message': '',
            }

    def _update_pick_di_session(
        self,
        available: bool,
        held: bool | None,
        message: str,
    ) -> tuple[bool, str]:
        session_lock = getattr(self, '_pick_di_session_lock', None)
        if session_lock is None:
            return False, str(message)
        with session_lock:
            session = self._active_pick_di_session
            if not session:
                return False, str(message)
            if bool(session.get('picked')):
                return True, str(session.get('message') or message)
            if bool(session.get('triggered')):
                return False, str(session.get('message') or message)
            if not available:
                return False, str(message)
            if held is False:
                session['saw_not_held'] = True
                session['message'] = str(message)
                return False, str(message)
            if held is True and bool(session.get('saw_not_held')):
                session['triggered'] = True
                session['message'] = f'{message}; DI held trigger detected'
                candidate_number = int(session.get('candidate_number') or 0)
                candidate_count = int(session.get('candidate_count') or 0)
                self.get_logger().info(
                    f'DI held trigger detected for candidate {candidate_number}/{candidate_count}: '
                    f'{session["message"]}'
                )
                return False, str(session['message'])
            session['message'] = (
                f'{message}; waiting for fresh false->true transition '
                f'(baseline held={session.get("baseline_held")})'
            )
            return False, str(session['message'])

    def _poll_pick_di_session(self) -> tuple[bool, str]:
        available, held, message = self._held_item_state()
        self._update_pick_di_session(available, held, message)
        session_lock = getattr(self, '_pick_di_session_lock', None)
        if session_lock is None:
            return False, str(message)
        with session_lock:
            session = self._active_pick_di_session
            if not session:
                return False, str(message)
            return bool(session.get('picked')), str(session.get('message') or message)

    def _poll_pick_di_trigger_session(self) -> tuple[bool, str]:
        available, held, message = self._held_item_state()
        self._update_pick_di_session(available, held, message)
        session_lock = getattr(self, '_pick_di_session_lock', None)
        if session_lock is None:
            return False, str(message)
        with session_lock:
            session = self._active_pick_di_session
            if not session:
                return False, str(message)
            if bool(session.get('triggered')) and not bool(session.get('trigger_consumed')):
                session['trigger_consumed'] = True
                return True, str(session.get('message') or message)
            return False, str(session.get('message') or message)

    def _finalize_pick_di_session_after_stop_settle(self, message: str) -> tuple[bool, str]:
        session_lock = getattr(self, '_pick_di_session_lock', None)
        if session_lock is None:
            return False, str(message)
        with session_lock:
            session = self._active_pick_di_session
            if not session or not bool(session.get('triggered')):
                return False, str(message)
            if not bool(session.get('picked')):
                session['picked'] = True
                session['message'] = f'{message}; post-Stop settle complete; DI held latched'
                candidate_number = int(session.get('candidate_number') or 0)
                candidate_count = int(session.get('candidate_count') or 0)
                self.get_logger().info(
                    f'DI held latched for candidate {candidate_number}/{candidate_count} '
                    f'after post-Stop settle: {session["message"]}'
                )
            return True, str(session.get('message') or message)

    def _finish_pick_di_session(self) -> tuple[bool, str]:
        session_lock = getattr(self, '_pick_di_session_lock', None)
        if session_lock is None:
            return False, 'No active pick DI session.'
        with session_lock:
            session = self._active_pick_di_session
            self._active_pick_di_session = None
        if not session:
            return False, 'No active pick DI session.'
        picked = bool(session.get('picked'))
        message = str(session.get('message') or session.get('baseline_message') or '')
        if not message:
            message = 'No held-item DI trigger observed.'
        return picked, message

    def _clear_pick_di_session(self) -> None:
        session_lock = getattr(self, '_pick_di_session_lock', None)
        if session_lock is None:
            self._active_pick_di_session = None
            return
        with session_lock:
            self._active_pick_di_session = None

    def _latest_qa_snapshot_marker_path(self) -> Path:
        return self._qa_snapshot_root / LATEST_ITEM_QA_SNAPSHOT_MARKER

    def _resolve_latest_qa_detection_yaml_path(self) -> Path | None:
        marker_path = self._latest_qa_snapshot_marker_path()
        try:
            text = marker_path.read_text(encoding='utf-8')
            payload = json.loads(text)
        except FileNotFoundError:
            self.get_logger().warn(
                f'QA snapshot marker missing; cannot confirm picked result: {marker_path}'
            )
            return None
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to read QA snapshot marker "{marker_path}": {exc}'
            )
            return None

        detection_yaml_text = ''
        if isinstance(payload, dict):
            detection_yaml_text = str(payload.get('detection_yaml', '') or '').strip()
        elif isinstance(payload, str):
            detection_yaml_text = payload.strip()
        if not detection_yaml_text:
            self.get_logger().warn(
                f'QA snapshot marker "{marker_path}" has no detection_yaml path'
            )
            return None
        detection_yaml_path = Path(detection_yaml_text).expanduser()
        if not detection_yaml_path.exists():
            self.get_logger().warn(
                f'QA snapshot YAML missing; cannot confirm picked result: {detection_yaml_path}'
            )
            return None
        return detection_yaml_path

    def _write_pick_result_to_detection_yaml(
        self,
        detection_yaml_path: Path,
        *,
        picked: bool,
        message: str,
    ) -> bool:
        if yaml is None:
            self.get_logger().warn(
                f'PyYAML unavailable; cannot update QA snapshot pick result: {detection_yaml_path}'
            )
            return False
        try:
            root = yaml.safe_load(detection_yaml_path.read_text(encoding='utf-8')) or {}
            if not isinstance(root, dict):
                root = {}
            root['picked'] = bool(picked)
            root['picked_result_source'] = 'item_pick_di_feedback'
            root['picked_result_message'] = str(message)
            tmp_path = detection_yaml_path.with_name(detection_yaml_path.name + '.tmp')
            tmp_path.write_text(
                yaml.safe_dump(root, sort_keys=False),
                encoding='utf-8',
            )
            tmp_path.replace(detection_yaml_path)
            return True
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to update QA snapshot "{detection_yaml_path}" with pick result: {exc}'
            )
            return False

    def _tcp_xyz_distance_mm(
        self,
        goal_xyz_mm: tuple[float, float, float],
    ) -> tuple[float | None, ItemPickSnapshot]:
        snapshot = self.snapshot()
        if snapshot.tcp_stamp is None:
            return None, snapshot
        dx = float(snapshot.tcp_values.get('x', 0.0)) - float(goal_xyz_mm[0])
        dy = float(snapshot.tcp_values.get('y', 0.0)) - float(goal_xyz_mm[1])
        dz = float(snapshot.tcp_values.get('z', 0.0)) - float(goal_xyz_mm[2])
        distance_mm = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
        return distance_mm, snapshot

    def _wait_for_tcp_xyz_goal_passive(
        self,
        goal_xyz_mm: tuple[float, float, float],
        tolerance_mm: float = PICK_QA_FINAL_Z_REACH_TOLERANCE_MM,
        timeout_sec: float = PICK_QA_FINAL_Z_REACH_TIMEOUT_SEC,
    ) -> tuple[bool, str]:
        tolerance = max(0.1, float(tolerance_mm))
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while rclpy.ok():
            if time.monotonic() >= deadline:
                break
            distance_mm, snapshot = self._tcp_xyz_distance_mm(goal_xyz_mm)
            if distance_mm is None:
                time.sleep(PICK_QA_CONFIRM_POLL_SEC)
                continue
            if snapshot.tcp_stamp is not None:
                tcp_age = time.time() - float(snapshot.tcp_stamp)
                if tcp_age > ROBOT_TCP_STALE_SEC:
                    time.sleep(PICK_QA_CONFIRM_POLL_SEC)
                    continue
            if distance_mm <= tolerance:
                return True, (
                    f'reached final Z-up within {distance_mm:.2f} mm '
                    f'(tol={tolerance:.1f} mm)'
                )
            time.sleep(PICK_QA_CONFIRM_POLL_SEC)

        if not rclpy.ok():
            return False, 'ROS shutdown while waiting for final Z-up during QA pick confirmation'
        distance_mm, snapshot = self._tcp_xyz_distance_mm(goal_xyz_mm)
        if distance_mm is None:
            return False, 'No TCP feedback while waiting for final Z-up during QA pick confirmation'
        return False, (
            f'timed out waiting for final Z-up reach during QA pick confirmation '
            f'(distance={distance_mm:.2f} mm, tol={tolerance:.1f} mm, timeout={float(timeout_sec):.1f}s)'
        )

    def _wait_for_tcp_motion_away_from_goal_passive(
        self,
        goal_xyz_mm: tuple[float, float, float],
        threshold_mm: float = PICK_QA_FINAL_Z_REACH_TOLERANCE_MM,
        timeout_sec: float = PICK_QA_FINAL_Z_REACH_TIMEOUT_SEC,
    ) -> tuple[bool, str]:
        threshold = max(0.1, float(threshold_mm))
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while rclpy.ok():
            if time.monotonic() >= deadline:
                break
            distance_mm, snapshot = self._tcp_xyz_distance_mm(goal_xyz_mm)
            if distance_mm is None:
                time.sleep(PICK_QA_CONFIRM_POLL_SEC)
                continue
            if snapshot.tcp_stamp is not None:
                tcp_age = time.time() - float(snapshot.tcp_stamp)
                if tcp_age > ROBOT_TCP_STALE_SEC:
                    time.sleep(PICK_QA_CONFIRM_POLL_SEC)
                    continue
            if distance_mm > threshold:
                return True, (
                    f'robot left final Z-up by {distance_mm:.2f} mm '
                    f'(threshold={threshold:.1f} mm)'
                )
            time.sleep(PICK_QA_CONFIRM_POLL_SEC)

        if not rclpy.ok():
            return False, 'ROS shutdown while waiting for fixed-home motion during QA pick confirmation'
        distance_mm, snapshot = self._tcp_xyz_distance_mm(goal_xyz_mm)
        if distance_mm is None:
            return False, 'No TCP feedback while waiting for fixed-home motion during QA pick confirmation'
        return False, (
            f'timed out waiting for robot to leave final Z-up during QA pick confirmation '
            f'(distance={distance_mm:.2f} mm, threshold={threshold:.1f} mm, timeout={float(timeout_sec):.1f}s)'
        )

    def _confirm_pick_result_async(
        self,
        detection_yaml_path: Path | None,
        *,
        final_z_goal_xyz_mm: tuple[float, float, float] | None = None,
        baseline_held: bool | None = None,
    ) -> None:
        if detection_yaml_path is None:
            return

        def worker() -> None:
            if final_z_goal_xyz_mm is not None:
                reached_final_z, reached_message = self._wait_for_tcp_xyz_goal_passive(
                    final_z_goal_xyz_mm,
                )
                if not reached_final_z:
                    self.get_logger().warn(
                        f'QA pick confirmation aborted before fixed-home DI check: {reached_message}'
                    )
                    return
                moved_from_final_z, moved_message = self._wait_for_tcp_motion_away_from_goal_passive(
                    final_z_goal_xyz_mm,
                )
                if not moved_from_final_z:
                    self.get_logger().warn(
                        f'QA pick confirmation did not observe fixed-home motion after final Z-up: {moved_message}'
                    )
                    return

            deadline = time.monotonic() + PICK_QA_CONFIRM_TIMEOUT_SEC
            stable_anchor_pose: tuple[float, float, float, float, float, float] | None = None
            stable_since: float | None = None
            stable_elapsed = 0.0
            last_tcp_stamp: float | None = None
            last_linear_delta = 0.0
            last_rot_delta = 0.0
            initial_available, initial_held, initial_message = self._held_item_state()
            allow_immediate_confirm = (baseline_held is not True)
            saw_not_held = bool(baseline_held is False)
            if baseline_held is True:
                self.get_logger().warn(
                    'QA pick confirmation started from a pre-pick held-item DI=true baseline; '
                    'waiting for a fresh false->true transition before marking picked.'
                )
            elif initial_available and initial_held:
                self.get_logger().info(
                    'QA pick confirmation observed held-item DI=true as fixed-home motion began; '
                    f'accepting immediate confirmation because pre-pick baseline was not held: {initial_message}'
                )

            while rclpy.ok() and time.monotonic() < deadline:
                available, held, message = self._held_item_state()
                if available and held is False:
                    saw_not_held = True
                if available and held and (allow_immediate_confirm or saw_not_held):
                    if self._write_pick_result_to_detection_yaml(
                        detection_yaml_path,
                        picked=True,
                        message=message,
                    ):
                        self.get_logger().info(
                            f'Confirmed held-item DI and updated QA snapshot: '
                            f'{detection_yaml_path} ({message})'
                        )
                    return

                now = time.monotonic()
                snapshot = self.snapshot()
                if snapshot.tcp_stamp is not None and (time.time() - float(snapshot.tcp_stamp)) <= ROBOT_TCP_STALE_SEC:
                    pose = (
                        float(snapshot.tcp_values.get('x', 0.0)),
                        float(snapshot.tcp_values.get('y', 0.0)),
                        float(snapshot.tcp_values.get('z', 0.0)),
                        float(snapshot.tcp_values.get('rx', 0.0)),
                        float(snapshot.tcp_values.get('ry', 0.0)),
                        float(snapshot.tcp_values.get('rz', 0.0)),
                    )
                    if stable_anchor_pose is None:
                        stable_anchor_pose = pose
                        stable_since = now
                        last_tcp_stamp = snapshot.tcp_stamp
                    elif snapshot.tcp_stamp != last_tcp_stamp:
                        linear_delta, rot_delta = self._tcp_pose_delta(pose, stable_anchor_pose)
                        last_linear_delta = linear_delta
                        last_rot_delta = rot_delta
                        if linear_delta > ROBOT_LINEAR_MOVE_EPS_MM or rot_delta > ROBOT_ROT_MOVE_EPS_DEG:
                            stable_anchor_pose = pose
                            stable_since = now
                            last_linear_delta = 0.0
                            last_rot_delta = 0.0
                        last_tcp_stamp = snapshot.tcp_stamp

                    if stable_since is not None:
                        stable_elapsed = max(0.0, now - stable_since)
                    if stable_since is not None and stable_elapsed >= PICK_QA_STABLE_SEC:
                        stable_message = (
                            'Robot TCP stable after fixed-home move for '
                            f'{stable_elapsed:.2f}s '
                            f'(window delta {last_linear_delta:.2f}mm, {last_rot_delta:.2f}deg)'
                        )
                        self.get_logger().warn(
                            f'QA pick confirmation ended after fixed-home motion without held-item DI success; '
                            f'left picked=false in {detection_yaml_path} ({stable_message})'
                        )
                        return
                elif snapshot.tcp_stamp is not None:
                    last_tcp_stamp = snapshot.tcp_stamp

                if now >= deadline:
                    timeout_message = (
                        f'QA pick confirmation timed out without held-item DI success; '
                        f'left picked=false in {detection_yaml_path}'
                    )
                    if snapshot.tcp_stamp is None:
                        timeout_message += '; no TCP feedback received during fixed-home DI check'
                    else:
                        tcp_age = time.time() - float(snapshot.tcp_stamp)
                        if tcp_age > ROBOT_TCP_STALE_SEC:
                            timeout_message += f'; TCP feedback stale ({tcp_age:.2f}s old)'
                    self.get_logger().warn(
                        timeout_message
                    )
                    return
                time.sleep(PICK_QA_CONFIRM_POLL_SEC)

        threading.Thread(
            target=worker,
            name='item-pick-qa-confirm',
            daemon=True,
        ).start()

    @staticmethod
    def _rpy_deg_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[float, float, float, float]:
        roll = math.radians(float(roll_deg))
        pitch = math.radians(float(pitch_deg))
        yaw = math.radians(float(yaw_deg))

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    @staticmethod
    def _quat_conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        return (-q[0], -q[1], -q[2], q[3])

    @staticmethod
    def _quat_multiply(
        left: tuple[float, float, float, float],
        right: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        lx, ly, lz, lw = left
        rx, ry, rz, rw = right
        return (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )

    @staticmethod
    def _quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        norm = math.sqrt((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]))
        if norm <= 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)

    @classmethod
    def _quat_angular_distance_deg(
        cls,
        left: tuple[float, float, float, float],
        right: tuple[float, float, float, float],
    ) -> float:
        l = cls._quat_normalize(left)
        r = cls._quat_normalize(right)
        dot = (l[0] * r[0]) + (l[1] * r[1]) + (l[2] * r[2]) + (l[3] * r[3])
        # q and -q represent the same rotation; use absolute dot for shortest distance.
        dot = max(-1.0, min(1.0, abs(dot)))
        return math.degrees(2.0 * math.acos(dot))

    @classmethod
    def _rotate_vector_by_quaternion(
        cls,
        vector_xyz: tuple[float, float, float],
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        q = cls._quat_normalize(quaternion_xyzw)
        pure = (float(vector_xyz[0]), float(vector_xyz[1]), float(vector_xyz[2]), 0.0)
        rotated = cls._quat_multiply(cls._quat_multiply(q, pure), cls._quat_conjugate(q))
        return (rotated[0], rotated[1], rotated[2])

    @classmethod
    def _yaw_deg_from_quaternion_xy_axis(
        cls,
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> float:
        x_axis = cls._rotate_vector_by_quaternion((1.0, 0.0, 0.0), quaternion_xyzw)
        return math.degrees(math.atan2(x_axis[1], x_axis[0]))

    @classmethod
    def _sterilize_quaternion_to_world_normal(
        cls,
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        # Keep yaw but remove roll/pitch so tool Z becomes world-normal (up/down).
        yaw_deg = cls._yaw_deg_from_quaternion_xy_axis(quaternion_xyzw)
        q_up = cls._quat_normalize(cls._rpy_deg_to_quaternion(0.0, 0.0, yaw_deg))
        q_down = cls._quat_normalize(cls._rpy_deg_to_quaternion(180.0, 0.0, yaw_deg))
        q_input = cls._quat_normalize(quaternion_xyzw)
        return min(
            (q_up, q_down),
            key=lambda q_candidate: cls._quat_angular_distance_deg(q_input, q_candidate),
        )

    @staticmethod
    def _quaternion_to_rpy_deg(
        quaternion_xyzw: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        qx, qy, qz, qw = quaternion_xyzw
        sinr_cosp = 2.0 * ((qw * qx) + (qy * qz))
        cosr_cosp = 1.0 - (2.0 * ((qx * qx) + (qy * qy)))
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * ((qw * qy) - (qz * qx))
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * ((qw * qz) + (qx * qy))
        cosy_cosp = 1.0 - (2.0 * ((qy * qy) + (qz * qz)))
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def _choose_min_rotation_candidate_index(
        self,
        candidates: tuple[tuple[float, float, float, float], ...],
    ) -> int:
        if len(candidates) <= 1:
            return 0

        snapshot = self.snapshot()
        if snapshot.tcp_stamp is None:
            return 0

        q_current_tcp = self._quat_normalize(
            self._rpy_deg_to_quaternion(
                float(snapshot.tcp_values.get('rx', 0.0)),
                float(snapshot.tcp_values.get('ry', 0.0)),
                float(snapshot.tcp_values.get('rz', 0.0)),
            )
        )
        return min(
            range(len(candidates)),
            key=lambda idx: self._quat_angular_distance_deg(q_current_tcp, candidates[idx]),
        )

    @classmethod
    def _compose_transform_m(
        cls,
        parent_translation_m: tuple[float, float, float],
        parent_rotation_xyzw: tuple[float, float, float, float],
        child_translation_m: tuple[float, float, float],
        child_rotation_xyzw: tuple[float, float, float, float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        rotated_child_translation = cls._rotate_vector_by_quaternion(
            child_translation_m,
            parent_rotation_xyzw,
        )
        composed_translation = (
            parent_translation_m[0] + rotated_child_translation[0],
            parent_translation_m[1] + rotated_child_translation[1],
            parent_translation_m[2] + rotated_child_translation[2],
        )
        composed_rotation = cls._quat_normalize(
            cls._quat_multiply(parent_rotation_xyzw, child_rotation_xyzw)
        )
        return composed_translation, composed_rotation

    @classmethod
    def _transform_point_inverse_m(
        cls,
        point_m: tuple[float, float, float],
        parent_to_child_translation_m: tuple[float, float, float],
        parent_to_child_rotation_xyzw: tuple[float, float, float, float],
    ) -> tuple[float, float, float]:
        relative = (
            point_m[0] - parent_to_child_translation_m[0],
            point_m[1] - parent_to_child_translation_m[1],
            point_m[2] - parent_to_child_translation_m[2],
        )
        return cls._rotate_vector_by_quaternion(
            relative,
            cls._quat_conjugate(parent_to_child_rotation_xyzw),
        )

    @staticmethod
    def _builtin_time_to_sec(stamp) -> float:
        sec = float(getattr(stamp, 'sec', 0))
        nanosec = float(getattr(stamp, 'nanosec', 0))
        return sec + (nanosec * 1e-9)

    @staticmethod
    def _builtin_time_to_nanoseconds(stamp) -> int:
        sec = int(getattr(stamp, 'sec', 0))
        nanosec = int(getattr(stamp, 'nanosec', 0))
        return (sec * 1_000_000_000) + nanosec

    @staticmethod
    def _item_pose_stamp_freshness(
        stamp_sec: float,
        now_sec: float,
        max_age_sec: float,
    ) -> tuple[bool, float, str]:
        """Validate the sensor acquisition time used to authorize motion.

        The message receive time is deliberately irrelevant: replaying an old
        message does not reacquire the bin scene. The small future tolerance
        only absorbs normal clock quantization/transport skew.
        """
        stamp_sec = float(stamp_sec)
        now_sec = float(now_sec)
        max_age_sec = float(max_age_sec)
        if not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
            return False, math.inf, 'acquisition timestamp is missing or non-positive'
        if not math.isfinite(now_sec) or now_sec <= 0.0:
            return False, math.inf, 'current ROS time is unavailable or non-positive'
        if not math.isfinite(max_age_sec) or max_age_sec <= 0.0:
            return False, math.inf, 'pose freshness limit is invalid'

        age_sec = now_sec - stamp_sec
        if age_sec < -ITEM_POSE_FUTURE_STAMP_TOLERANCE_SEC:
            return (
                False,
                age_sec,
                f'acquisition timestamp is {-age_sec:.3f}s in the future',
            )
        if age_sec > max_age_sec:
            return (
                False,
                age_sec,
                f'acquisition age {age_sec:.3f}s exceeds {max_age_sec:.3f}s limit',
            )
        return True, max(0.0, age_sec), 'fresh acquisition timestamp'

    def _item_pose_stamp_is_fresh_for_motion(
        self,
        stamp_sec: float,
        source_topic: str,
    ) -> bool:
        try:
            now_sec = float(self.get_clock().now().nanoseconds) * 1e-9
        except Exception as exc:
            now_sec = math.nan
            clock_failure = f'could not read current ROS time: {exc}'
        else:
            clock_failure = ''
        max_age_sec = float(
            getattr(
                self,
                '_item_pose_max_acquisition_age_sec',
                ITEM_POSE_MAX_ACQUISITION_AGE_SEC_DEFAULT,
            )
        )
        fresh, age_sec, reason = self._item_pose_stamp_freshness(
            stamp_sec,
            now_sec,
            max_age_sec,
        )
        if fresh:
            return True
        if clock_failure:
            reason = clock_failure

        with self._lock:
            should_log = not bool(getattr(self, '_item_pose_skip_warned', False))
            self._item_pose_skip_warned = True
        if should_log:
            age_text = f'{age_sec:.3f}s' if math.isfinite(age_sec) else 'unavailable'
            self.get_logger().warn(
                f'Dropping {source_topic}: {reason} '
                f'(stamp={float(stamp_sec):.9f}, age={age_text}); '
                'message receive/replay time cannot certify scene freshness'
            )
        return False

    @classmethod
    def _item_pose_target_from_pose(
        cls,
        pose: Pose,
        frame_id: str,
        stamp_sec: float,
        acquisition_stamp_ns: int | None = None,
    ) -> ItemPoseTarget:
        orientation = (
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        return ItemPoseTarget(
            position_mm=(
                float(pose.position.x) * 1000.0,
                float(pose.position.y) * 1000.0,
                float(pose.position.z) * 1000.0,
            ),
            rpy_deg=cls._quaternion_to_rpy_deg(orientation),
            frame_id=str(frame_id),
            stamp_sec=float(stamp_sec),
            acquisition_stamp_ns=(
                int(acquisition_stamp_ns)
                if acquisition_stamp_ns is not None
                else None
            ),
        )

    @staticmethod
    def _angle_delta_deg(left_deg: float, right_deg: float) -> float:
        return abs(((float(left_deg) - float(right_deg) + 180.0) % 360.0) - 180.0)

    @classmethod
    def _item_pose_targets_equivalent(
        cls,
        left: ItemPoseTarget,
        right: ItemPoseTarget,
    ) -> bool:
        if left.frame_id != right.frame_id:
            return False
        position_close = all(
            abs(float(a) - float(b)) <= 1e-3
            for a, b in zip(left.position_mm, right.position_mm)
        )
        rpy_close = all(
            cls._angle_delta_deg(a, b) <= 1e-3
            for a, b in zip(left.rpy_deg, right.rpy_deg)
        )
        return position_close and rpy_close

    @staticmethod
    def _item_pose_candidate_stamp_matches(
        primary: ItemPoseTarget,
        candidate: ItemPoseTarget,
    ) -> bool:
        primary_sec = float(primary.stamp_sec)
        candidate_sec = float(candidate.stamp_sec)
        if (
            not math.isfinite(primary_sec)
            or not math.isfinite(candidate_sec)
            or primary_sec <= 0.0
            or candidate_sec <= 0.0
        ):
            return False

        primary_ns = getattr(primary, 'acquisition_stamp_ns', None)
        candidate_ns = getattr(candidate, 'acquisition_stamp_ns', None)
        if primary_ns is None:
            primary_ns = int(round(primary_sec * 1_000_000_000))
        if candidate_ns is None:
            candidate_ns = int(round(candidate_sec * 1_000_000_000))
        return int(primary_ns) > 0 and int(primary_ns) == int(candidate_ns)

    @classmethod
    def _merge_item_pose_candidates(
        cls,
        primary: ItemPoseTarget,
        candidates: tuple[ItemPoseTarget, ...],
    ) -> tuple[ItemPoseTarget, ...]:
        merged: list[ItemPoseTarget] = [primary]
        for candidate in candidates:
            if candidate.frame_id != primary.frame_id:
                continue
            if not cls._item_pose_candidate_stamp_matches(primary, candidate):
                continue
            if any(cls._item_pose_targets_equivalent(candidate, existing) for existing in merged):
                continue
            merged.append(candidate)
        return tuple(merged)

    def _latest_item_pose_array_seq(self) -> int:
        with self._lock:
            return int(getattr(self, '_latest_item_pose_candidates_seq', 0) or 0)

    def _publish_failed_pick_area(
        self,
        candidate_target: ItemPoseTarget,
        *,
        reason: str,
    ) -> bool:
        publisher = getattr(self, '_failed_pick_area_pub', None)
        if publisher is None:
            self.get_logger().warn(
                f'Cannot publish failed pick area for {reason}: publisher unavailable'
            )
            return False
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = candidate_target.frame_id
        msg.pose.position.x = float(candidate_target.position_mm[0]) * 0.001
        msg.pose.position.y = float(candidate_target.position_mm[1]) * 0.001
        msg.pose.position.z = float(candidate_target.position_mm[2]) * 0.001
        qx, qy, qz, qw = self._rpy_deg_to_quaternion(*candidate_target.rpy_deg)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        try:
            publisher.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'Failed to publish failed pick area for {reason}: {exc}')
            return False
        self.get_logger().info(
            'DATALOG item_pick_failed_area_published: '
            f'reason={reason} frame_id="{candidate_target.frame_id}" '
            f'pos_mm=({candidate_target.position_mm[0]:.1f},'
            f'{candidate_target.position_mm[1]:.1f},{candidate_target.position_mm[2]:.1f})'
        )
        return True

    def _cached_attempt_with_candidates(
        self,
        attempt: CachedPickAttempt,
        candidates: tuple[ItemPoseTarget, ...],
        *,
        next_candidate_index: int = 0,
        successful_candidate_index: int | None = None,
    ) -> CachedPickAttempt:
        return replace(
            attempt,
            candidates=tuple(candidates),
            next_candidate_index=max(0, min(len(candidates), int(next_candidate_index))),
            successful_candidate_index=successful_candidate_index,
        )

    def _replace_cached_pick_candidates(
        self,
        candidates: tuple[ItemPoseTarget, ...],
        *,
        next_candidate_index: int = 0,
        successful_candidate_index: int | None = None,
    ) -> None:
        with self._lock:
            attempt = self._cached_pick_attempt
            if attempt is None:
                return
            self._cached_pick_attempt = self._cached_attempt_with_candidates(
                attempt,
                tuple(candidates),
                next_candidate_index=next_candidate_index,
                successful_candidate_index=successful_candidate_index,
            )

    @staticmethod
    def _vector_norm3(vector_xyz: tuple[float, float, float]) -> float:
        return math.sqrt((vector_xyz[0] * vector_xyz[0]) + (vector_xyz[1] * vector_xyz[1]) + (vector_xyz[2] * vector_xyz[2]))

    @staticmethod
    def _vector_dot3(left_xyz: tuple[float, float, float], right_xyz: tuple[float, float, float]) -> float:
        return (left_xyz[0] * right_xyz[0]) + (left_xyz[1] * right_xyz[1]) + (left_xyz[2] * right_xyz[2])

    @staticmethod
    def _normalize_vector3(vector_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
        norm = ItemPickNode._vector_norm3(vector_xyz)
        if norm <= 1e-12:
            return (0.0, 0.0, 0.0)
        return (vector_xyz[0] / norm, vector_xyz[1] / norm, vector_xyz[2] / norm)

    @staticmethod
    def _clamp_tool_offset_translation_mm(value_mm: float) -> float:
        return max(
            TOOL_OFFSET_TRANSLATION_MIN_MM,
            min(TOOL_OFFSET_TRANSLATION_MAX_MM, float(value_mm)),
        )

    @staticmethod
    def _clamp_tool_offset_rotation_deg(value_deg: float) -> float:
        return max(
            TOOL_OFFSET_ROTATION_MIN_DEG,
            min(TOOL_OFFSET_ROTATION_MAX_DEG, float(value_deg)),
        )

    def _profile_display_name(self, fallback_id: str | None = None) -> str:
        """Return the human-readable label for the active item teach.

        Prefers the cached ``item:/display_name`` resolved from the
        single items yaml; ``fallback_id`` is used (or the placeholder
        is returned) when no item is loaded yet so callers never have
        to special-case None during log/UI formatting.
        """
        if self._active_item_display_name:
            return self._active_item_display_name
        if self._active_item_id:
            return self._active_item_id
        if fallback_id:
            return str(fallback_id)
        return 'No active item teach'

    def _resolve_marked_active_item_file(self) -> Path | None:
        marker = self._items_dir / ACTIVE_ITEM_PROFILE_MARKER
        try:
            payload = json.loads(marker.read_text(encoding='utf-8'))
        except FileNotFoundError:
            return None
        except Exception as exc:
            self.get_logger().warn(
                f'Ignoring unreadable active item marker "{marker}": {exc}'
            )
            return None
        if not isinstance(payload, dict):
            self.get_logger().warn(f'Ignoring malformed active item marker "{marker}"')
            return None
        filename = str(payload.get('filename') or '').strip()
        path_text = str(payload.get('path') or '').strip()
        candidate = (self._items_dir / filename) if filename else Path(path_text)
        if not candidate.is_absolute():
            candidate = self._items_dir / candidate
        try:
            items_dir = self._items_dir.resolve()
            resolved = candidate.resolve()
        except OSError as exc:
            self.get_logger().warn(
                f'Ignoring active item marker path "{candidate}": {exc}'
            )
            return None
        try:
            resolved.relative_to(items_dir)
        except ValueError:
            self.get_logger().warn(
                f'Ignoring active item marker outside items dir: "{resolved}"'
            )
            return None
        if resolved.suffix.lower() not in {'.yaml', '.yml'} or not resolved.is_file():
            self.get_logger().warn(
                f'Ignoring active item marker for missing/non-yaml file: "{resolved}"'
            )
            return None
        return resolved

    def _resolve_active_item_file(self) -> Path | None:
        """Resolve the active item profile from marker, then single-file fallback.

        The marker is written atomically by ``cmd.sync_teach`` and lets the
        node ignore a stale duplicate YAML during profile swaps. If no marker
        exists, the historical single-file rule is kept.
        """
        marked = self._resolve_marked_active_item_file()
        if marked is not None:
            return marked
        try:
            return resolve_single_teach_file(self._items_dir, ITEM_SUBJECT_KIND)
        except TeachLayoutError as exc:
            self.get_logger().error(
                f'item_pick refuses to resolve active item: {exc}. '
                'Restart the node once exactly one yaml remains in '
                f'"{self._items_dir}".'
            )
            return None
        except OSError as exc:
            self.get_logger().warn(
                f'Failed to inspect items dir "{self._items_dir}": {exc}'
            )
            return None

    def _load_active_item_root(self, item_file: Path) -> dict | None:
        try:
            return load_subject_yaml(item_file, ITEM_SUBJECT_KIND)
        except (TeachLayoutError, TeachSchemaError) as exc:
            self.get_logger().warn(
                f'Could not load item yaml "{item_file}": {exc}'
            )
            return None
        except Exception as exc:
            self.get_logger().warn(
                f'Unexpected error loading item yaml "{item_file}": {exc}'
            )
            return None

    @staticmethod
    def _safe_yaml_load(path: Path) -> dict[str, object] | None:
        if yaml is None:
            return None
        try:
            with path.open('r', encoding='utf-8') as infile:
                payload = yaml.safe_load(infile)
        except Exception:
            return None
        return payload if isinstance(payload, dict) else None

    @staticmethod
    def _yaml_map(payload: object, key: str) -> dict[str, object]:
        if not isinstance(payload, dict):
            return {}
        value = payload.get(key)
        return value if isinstance(value, dict) else {}

    @staticmethod
    def _yaml_str(payload: object, key: str, default: str = '') -> str:
        if not isinstance(payload, dict):
            return default
        value = payload.get(key, default)
        return str(value).strip() if value is not None else default

    def _platform_calibration_path(self) -> Path | None:
        path_text = str(getattr(self, '_platform_calibration_file', '') or '').strip()
        if not path_text:
            return None
        return Path(path_text).expanduser()

    def _platform_calibration_source(self) -> str:
        return 'item_pick platform_calibration_file parameter'


    @staticmethod
    def _yaml_xyz_m(payload: object) -> tuple[float, float, float] | None:
        if not isinstance(payload, dict):
            return None
        try:
            return (
                float(payload['x']),
                float(payload['y']),
                float(payload['z']),
            )
        except Exception:
            return None

    @classmethod
    def _yaml_xyzw_quaternion(cls, payload: object) -> tuple[float, float, float, float] | None:
        if not isinstance(payload, dict):
            return None
        try:
            return cls._quat_normalize((
                float(payload['x']),
                float(payload['y']),
                float(payload['z']),
                float(payload['w']),
            ))
        except Exception:
            return None

    def _warn_fixed_camera_tf_once(self, key: str, message: str) -> None:
        warned = getattr(self, '_fixed_camera_tf_warned_keys', set())
        if key in warned:
            return
        warned.add(key)
        self._fixed_camera_tf_warned_keys = warned
        self.get_logger().warn(message)

    def _fixed_camera_calibration_path(self) -> Path | None:
        path_text = str(getattr(self, '_fixed_camera_calibration_file', '') or '').strip()
        if not path_text:
            return None
        return Path(path_text).expanduser()

    @staticmethod
    def _normalize_tf_frame(frame_id: object) -> str:
        return str(frame_id or '').strip().strip('/')

    def _load_fixed_camera_calibration_tf(
        self,
        requested_child_frame: str = '',
        *,
        warn_on_failure: bool = False,
    ) -> TransformStamped | None:
        path = self._fixed_camera_calibration_path()
        requested_child = self._normalize_tf_frame(requested_child_frame)
        if path is None:
            if warn_on_failure:
                self._warn_fixed_camera_tf_once(
                    'fixed_camera_calibration_file:empty',
                    'Fixed-camera calibration TF unavailable: fixed_camera_calibration_file is empty.',
                )
            return None

        root = self._safe_yaml_load(path)
        if root is None:
            if warn_on_failure:
                self._warn_fixed_camera_tf_once(
                    f'fixed_camera_calibration_file:read:{path}',
                    f'Fixed-camera calibration TF unavailable: could not read "{path}".',
                )
            return None

        metadata = self._yaml_map(root, 'metadata')
        params = self._yaml_map(root, 'parameters')
        transform = self._yaml_map(root, 'calibration_transform')
        if not transform:
            transform = self._yaml_map(root, 'transform')
        if not transform:
            if warn_on_failure:
                self._warn_fixed_camera_tf_once(
                    f'fixed_camera_calibration_file:transform:{path}',
                    f'Fixed-camera calibration TF unavailable: "{path}" has no calibration transform.',
                )
            return None

        translation = self._yaml_xyz_m(self._yaml_map(transform, 'translation'))
        rotation = self._yaml_xyzw_quaternion(self._yaml_map(transform, 'rotation'))
        if translation is None or rotation is None:
            if warn_on_failure:
                self._warn_fixed_camera_tf_once(
                    f'fixed_camera_calibration_file:values:{path}',
                    f'Fixed-camera calibration TF unavailable: "{path}" has invalid translation/rotation.',
                )
            return None

        calibration_type = self._normalize_tf_frame(
            params.get('calibration_type', 'eye_on_base')
        ).lower()
        parent_frame = self._normalize_tf_frame(
            metadata.get('transform_parent_frame')
            or params.get('transform_parent_frame')
            or (
                params.get('robot_effector_frame')
                if calibration_type in {'eye_on_hand', 'eye_in_hand'}
                else params.get('robot_base_frame')
            )
            or self._robot_goal_frame_id
        )
        child_frame = self._normalize_tf_frame(
            metadata.get('transform_child_frame')
            or params.get('transform_child_frame')
            or params.get('tracking_base_frame')
            or requested_child
        )
        if not parent_frame or not child_frame:
            if warn_on_failure:
                self._warn_fixed_camera_tf_once(
                    f'fixed_camera_calibration_file:frames:{path}',
                    f'Fixed-camera calibration TF unavailable: "{path}" has missing parent/child frame.',
                )
            return None
        if requested_child and child_frame != requested_child:
            if warn_on_failure:
                self._warn_fixed_camera_tf_once(
                    f'fixed_camera_calibration_file:child_mismatch:{path}:{requested_child}',
                    f'Fixed-camera calibration TF "{path}" child frame is "{child_frame}", '
                    f'but bin_seek_pose uses "{requested_child}". Refusing to alias frames.',
                )
            return None

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = translation[0]
        tf_msg.transform.translation.y = translation[1]
        tf_msg.transform.translation.z = translation[2]
        tf_msg.transform.rotation.x = rotation[0]
        tf_msg.transform.rotation.y = rotation[1]
        tf_msg.transform.rotation.z = rotation[2]
        tf_msg.transform.rotation.w = rotation[3]
        return tf_msg

    def _send_static_transform_cached(self, tf_msg: TransformStamped) -> None:
        child = self._normalize_tf_frame(getattr(tf_msg, 'child_frame_id', ''))
        if not child:
            return
        tf_msg.child_frame_id = child
        tf_msg.header.frame_id = self._normalize_tf_frame(tf_msg.header.frame_id)
        self._goal_static_tf_by_child[child] = tf_msg
        self._goal_tf_static_broadcaster.sendTransform(list(self._goal_static_tf_by_child.values()))

    def _publish_fixed_camera_calibration_tf(
        self,
        source_frame: str = '',
        *,
        log_success: bool = False,
        warn_on_failure: bool = False,
    ) -> bool:
        tf_msg = self._load_fixed_camera_calibration_tf(
            source_frame,
            warn_on_failure=warn_on_failure,
        )
        if tf_msg is None:
            return False
        self._send_static_transform_cached(tf_msg)
        published = getattr(self, '_fixed_camera_tf_published_children', set())
        child = str(tf_msg.child_frame_id)
        first_publish = child not in published
        published.add(child)
        self._fixed_camera_tf_published_children = published
        if log_success or first_publish:
            self.get_logger().info(
                f'Published fixed-camera calibration TF from "{self._fixed_camera_calibration_file}": '
                f'{tf_msg.header.frame_id}->{tf_msg.child_frame_id}'
            )
        return True

    def _load_active_bin_camera_safety_area(self) -> tuple[BinCameraSafetyArea | None, str]:
        if not (self._prefer_camera_inside_bin or self._require_camera_inside_bin):
            return None, 'camera-bin preference disabled'
        if yaml is None:
            return None, 'PyYAML unavailable; camera-bin preference skipped'

        profile_path = self._active_item_file
        if profile_path is None:
            return None, 'no active item profile; camera-bin preference skipped'

        # Sanity-check the bin id wired into the item yaml against the
        # single bin yaml CATARM placed in ``bins/``. The bin geometry
        # itself comes from the bin yaml -- the items file only carries
        # the id so a mismatch is surfaced as a warning rather than a
        # silent fallback to whatever path used to be embedded there.
        item_root = self._load_active_item_root(profile_path)
        item_teach_params = read_subject_params(item_root, TEACH_SECTION_KEY) if item_root else None
        expected_bin_id = ''
        if isinstance(item_teach_params, dict):
            raw_bin = item_teach_params.get('bin_id')
            if raw_bin is None:
                raw_bin = item_teach_params.get('bin_teach_id', '')
            expected_bin_id = str(raw_bin or '').strip()

        try:
            bin_file = resolve_single_teach_file(self._bins_dir, BIN_SUBJECT_KIND)
        except TeachLayoutError as exc:
            return None, f'bin teach layout error: {exc}'
        except OSError as exc:
            return None, f'failed to inspect bins dir "{self._bins_dir}": {exc}'
        if bin_file is None:
            return None, f'no bin yaml in "{self._bins_dir}"; camera-bin preference skipped'

        try:
            bin_root = load_subject_yaml(bin_file, BIN_SUBJECT_KIND)
        except (TeachLayoutError, TeachSchemaError) as exc:
            return None, f'could not read bin yaml "{bin_file}": {exc}'

        bin_data = read_subject_params(bin_root, TEACH_SECTION_KEY)
        if not isinstance(bin_data, dict):
            return None, f'bin yaml "{bin_file.name}" has no teach:/ros__parameters: section'

        actual_bin_id = str(bin_root.get(BIN_SUBJECT_KIND, {}).get('id', '') or '').strip()
        if expected_bin_id and actual_bin_id and expected_bin_id != actual_bin_id:
            self.get_logger().warn(
                f'Active item references bin id "{expected_bin_id}" but '
                f'bins/ holds "{actual_bin_id}". Using the bin yaml on disk.'
            )

        bin_teach_path = bin_file

        parent_frame = self._yaml_str(bin_data, 'parent_frame')
        bin_frame_id = self._yaml_str(bin_data, 'bin_frame', 'bin_frame')
        transform = self._yaml_map(bin_data, 'transform')
        parent_to_bin_t = self._yaml_xyz_m(self._yaml_map(transform, 'translation'))
        parent_to_bin_q = self._yaml_xyzw_quaternion(self._yaml_map(transform, 'rotation'))
        if parent_to_bin_t is None or parent_to_bin_q is None:
            return None, f'bin teach file "{bin_teach_path.name}" has no usable parent->bin transform'

        base_to_parent_t = (0.0, 0.0, 0.0)
        base_to_parent_q = (0.0, 0.0, 0.0, 1.0)
        if parent_frame and parent_frame != self._robot_goal_frame_id:
            platform_path = self._platform_calibration_path()
            platform_source = self._platform_calibration_source()
            if platform_path is None:
                return None, (
                    f'bin parent "{parent_frame}" is not "{self._robot_goal_frame_id}" '
                    'and platform_calibration_file is not configured'
                )

            platform_root = self._safe_yaml_load(platform_path)
            platform_tf = self._yaml_map(platform_root, 'transform')
            if not platform_tf:
                platform_tf = self._yaml_map(platform_root, 'calibration_transform')
            metadata = self._yaml_map(platform_root, 'metadata')
            metadata_parent = self._yaml_str(metadata, 'transform_parent_frame')
            metadata_child = self._yaml_str(metadata, 'transform_child_frame')
            if metadata_parent and metadata_parent != self._robot_goal_frame_id:
                return None, (
                    f'platform calibration parent "{metadata_parent}" is not '
                    f'"{self._robot_goal_frame_id}"'
                )
            if metadata_child and metadata_child != parent_frame:
                return None, (
                    f'platform calibration child "{metadata_child}" is not bin parent '
                    f'"{parent_frame}"'
                )
            base_to_parent_t = self._yaml_xyz_m(self._yaml_map(platform_tf, 'translation'))
            base_to_parent_q = self._yaml_xyzw_quaternion(self._yaml_map(platform_tf, 'rotation'))
            if base_to_parent_t is None or base_to_parent_q is None:
                return None, (
                    f'could not read platform calibration "{platform_path}" '
                    f'from {platform_source}'
                )
            self.get_logger().info(
                f'Camera-bin safety using {platform_source}: {platform_path}'
            )

        base_to_bin_t, base_to_bin_q = self._compose_transform_m(
            base_to_parent_t,
            base_to_parent_q,
            parent_to_bin_t,
            parent_to_bin_q,
        )

        marker_positions = self._yaml_map(bin_data, 'marker_positions')
        marker_points_bin: list[tuple[float, float, float]] = []
        for marker_pose in marker_positions.values():
            marker_parent = self._yaml_xyz_m(marker_pose)
            if marker_parent is None:
                continue
            marker_points_bin.append(
                self._transform_point_inverse_m(marker_parent, parent_to_bin_t, parent_to_bin_q)
            )
        if len(marker_points_bin) < 3:
            return None, f'bin teach file "{bin_teach_path.name}" has fewer than 3 marker positions'

        x_values = [point[0] for point in marker_points_bin]
        y_values = [point[1] for point in marker_points_bin]
        x_min = min(x_values)
        x_max = max(x_values)
        y_min = min(y_values)
        y_max = max(y_values)
        margin_m = max(0.0, float(self._camera_bin_safe_margin_mm) * 0.001)
        max_margin_m = max(0.0, min(x_max - x_min, y_max - y_min) * 0.5 - 1e-6)
        margin_m = min(margin_m, max_margin_m)
        return BinCameraSafetyArea(
            profile_path=profile_path,
            bin_teach_path=bin_teach_path,
            bin_frame_id=bin_frame_id,
            base_to_bin_translation_m=base_to_bin_t,
            base_to_bin_rotation_xyzw=base_to_bin_q,
            x_min_m=x_min + margin_m,
            x_max_m=x_max - margin_m,
            y_min_m=y_min + margin_m,
            y_max_m=y_max - margin_m,
            margin_m=margin_m,
        ), f'camera-bin preference loaded from "{bin_teach_path.name}"'

    def _normalize_pick_params(self, pick_params: object) -> dict[str, object] | None:
        """Validate and normalize ``pick:/ros__parameters:`` values.

        Returns None when the section is absent / malformed so callers
        can distinguish "no saved tool teach" from "all zeros". Reads
        run against the live items yaml on disk; pick is the only
        writer of this section so legacy fallbacks are intentionally
        absent (see plan: no sidecar / runtime / registry reads).

        Non-tool fields (``tf_only_mode``, ``item_pose_wait_timeout_sec``)
        are passed through unclamped so the GUI's
        ``_load_runtime_settings`` hook can restore them; the per-field
        clamping happens there.
        """
        if not isinstance(pick_params, dict):
            self._active_profile_pick_error = (
                'pick:/ros__parameters: section is missing or malformed.'
            )
            return None
        for key in ('use_fingers', 'grab_on_pick'):
            if key not in pick_params:
                self._active_profile_pick_error = (
                    f'pick:/ros__parameters: is missing required boolean "{key}".'
                )
                return None
            if not isinstance(pick_params[key], bool):
                self._active_profile_pick_error = (
                    f'pick:/ros__parameters: "{key}" must be a YAML boolean.'
                )
                return None
        optional_relax_key = 'relax_fingers_on_pick'
        if (
            optional_relax_key in pick_params
            and not isinstance(pick_params[optional_relax_key], bool)
        ):
            self._active_profile_pick_error = (
                f'pick:/ros__parameters: "{optional_relax_key}" must be a YAML boolean.'
            )
            return None
        self._active_profile_pick_error = ''
        result: dict[str, object] = {
            'use_fingers': bool(pick_params['use_fingers']),
            'grab_on_pick': bool(pick_params['grab_on_pick']),
            'relax_fingers_on_pick': bool(
                pick_params.get(
                    'relax_fingers_on_pick',
                    RELAX_FINGERS_ON_PICK_DEFAULT,
                )
            ),
            'item_standoff_z_mm': max(
                POST_STOP_Z_OFFSET_MIN,
                min(
                    POST_STOP_Z_OFFSET_MAX,
                    float(pick_params.get('item_standoff_z_mm', self._post_stop_z_offset_mm)),
                ),
            ),
            'tray_place_additional_z_mm': max(
                TRAY_PLACE_ADDITIONAL_Z_MM_MIN,
                min(
                    TRAY_PLACE_ADDITIONAL_Z_MM_MAX,
                    float(pick_params.get(
                        'tray_place_additional_z_mm',
                        TRAY_PLACE_ADDITIONAL_Z_MM_DEFAULT,
                    )),
                ),
            ),
            'final_z_up_mm': max(
                FINAL_Z_UP_MIN,
                min(
                    FINAL_Z_UP_MAX,
                    float(pick_params.get('final_z_up_mm', self._final_z_up_mm)),
                ),
            ),
            'pre_pick_settling_time_sec': PICK_RUNTIME_SETTLING_TIME_SEC,
            'pick_settling_time_sec': PICK_RUNTIME_SETTLING_TIME_SEC,
            'tool_offset_x_mm': self._clamp_tool_offset_translation_mm(
                pick_params.get('tool_offset_x_mm', 0.0)
            ),
            'tool_offset_y_mm': self._clamp_tool_offset_translation_mm(
                pick_params.get('tool_offset_y_mm', 0.0)
            ),
            'tool_offset_z_mm': self._clamp_tool_offset_translation_mm(
                pick_params.get('tool_offset_z_mm', 0.0)
            ),
            'tool_offset_rx_deg': self._clamp_tool_offset_rotation_deg(
                pick_params.get('tool_offset_rx_deg', 0.0)
            ),
            'tool_offset_ry_deg': self._clamp_tool_offset_rotation_deg(
                pick_params.get('tool_offset_ry_deg', 0.0)
            ),
            'tool_offset_rz_deg': self._clamp_tool_offset_rotation_deg(
                pick_params.get('tool_offset_rz_deg', 0.0)
            ),
            'dobot_tool_offset_x_mm': self._clamp_tool_offset_translation_mm(
                pick_params.get('dobot_tool_offset_x_mm', self._dobot_tool_offset_x_mm)
            ),
            'dobot_tool_offset_y_mm': self._clamp_tool_offset_translation_mm(
                pick_params.get('dobot_tool_offset_y_mm', self._dobot_tool_offset_y_mm)
            ),
            'dobot_tool_offset_z_mm': self._clamp_tool_offset_translation_mm(
                pick_params.get('dobot_tool_offset_z_mm', self._dobot_tool_offset_z_mm)
            ),
            'dobot_tool_offset_rx_deg': self._clamp_tool_offset_rotation_deg(
                pick_params.get('dobot_tool_offset_rx_deg', self._dobot_tool_offset_rx_deg)
            ),
            'dobot_tool_offset_ry_deg': self._clamp_tool_offset_rotation_deg(
                pick_params.get('dobot_tool_offset_ry_deg', self._dobot_tool_offset_ry_deg)
            ),
            'dobot_tool_offset_rz_deg': self._clamp_tool_offset_rotation_deg(
                pick_params.get('dobot_tool_offset_rz_deg', self._dobot_tool_offset_rz_deg)
            ),
        }
        if 'tf_only_mode' in pick_params:
            result['tf_only_mode'] = bool(pick_params.get('tf_only_mode', False))
        if 'item_pose_wait_timeout_sec' in pick_params:
            result['item_pose_wait_timeout_sec'] = float(
                pick_params.get('item_pose_wait_timeout_sec', 0.0)
            )
        for key in ('item_length', 'item_width', 'size_tolerance'):
            if key in pick_params:
                result[key] = pick_params[key]
        return result

    @staticmethod
    def _normalize_item_dimensions(teach_params: object) -> dict[str, float] | None:
        if not isinstance(teach_params, dict):
            return None
        try:
            length_mm = float(teach_params.get('item_length_mm', 0.0) or 0.0)
            width_mm = float(teach_params.get('item_width_mm', 0.0) or 0.0)
            height_mm = float(teach_params.get('item_height_mm', 0.0) or 0.0)
        except (TypeError, ValueError):
            return None
        if (
            not math.isfinite(length_mm)
            or not math.isfinite(width_mm)
            or not math.isfinite(height_mm)
            or length_mm <= 0.0
            or width_mm <= 0.0
            or height_mm <= 0.0
        ):
            return None
        return {
            'item_length_mm': length_mm,
            'item_width_mm': width_mm,
            'item_height_mm': height_mm,
        }

    def _apply_saved_tool_offsets_locked(self, profile_offsets: dict[str, object]) -> None:
        self._use_fingers = bool(profile_offsets['use_fingers'])
        self._grab_on_pick = bool(profile_offsets['grab_on_pick'])
        self._relax_fingers_on_pick = bool(
            profile_offsets.get(
                'relax_fingers_on_pick',
                RELAX_FINGERS_ON_PICK_DEFAULT,
            )
        )
        self._post_stop_z_offset_mm = float(profile_offsets['item_standoff_z_mm'])
        self._final_z_up_mm = float(profile_offsets['final_z_up_mm'])
        self._pre_pick_settling_time_sec = PICK_RUNTIME_SETTLING_TIME_SEC
        self._pick_settling_time_sec = PICK_RUNTIME_SETTLING_TIME_SEC
        self._tool_offset_x_mm = float(profile_offsets['tool_offset_x_mm'])
        self._tool_offset_y_mm = float(profile_offsets['tool_offset_y_mm'])
        self._tool_offset_z_mm = float(profile_offsets['tool_offset_z_mm'])
        self._tool_offset_rx_deg = float(profile_offsets['tool_offset_rx_deg'])
        self._tool_offset_ry_deg = float(profile_offsets['tool_offset_ry_deg'])
        self._tool_offset_rz_deg = float(profile_offsets['tool_offset_rz_deg'])
        self._dobot_tool_offset_x_mm = float(profile_offsets['dobot_tool_offset_x_mm'])
        self._dobot_tool_offset_y_mm = float(profile_offsets['dobot_tool_offset_y_mm'])
        self._dobot_tool_offset_z_mm = float(profile_offsets['dobot_tool_offset_z_mm'])
        self._dobot_tool_offset_rx_deg = float(profile_offsets['dobot_tool_offset_rx_deg'])
        self._dobot_tool_offset_ry_deg = float(profile_offsets['dobot_tool_offset_ry_deg'])
        self._dobot_tool_offset_rz_deg = float(profile_offsets['dobot_tool_offset_rz_deg'])

    def _sync_profile_tool_offsets_from_state(self, force: bool = False) -> tuple[str | None, dict[str, object] | None]:
        """Resolve the single items yaml and refresh cached pick state.

        ``force=False`` short-circuits when the items yaml's mtime is
        unchanged so the periodic GUI refresh does not re-parse every
        100 ms. CATARM swaps the file atomically (tmp+rename) so the
        mtime check is a reliable swap signal -- nodes do not hot-load
        new files, but mtime catches any in-place edit the operator may
        do (e.g. ``Save Tool Teach``) without reaching for the disk.
        """
        item_file = self._resolve_active_item_file()
        if item_file is None:
            with self._lock:
                self._active_item_file = None
                self._active_item_id = None
                self._active_item_display_name = None
                self._active_item_file_mtime_ns = None
                self._active_item_dimensions_mm = None
                self._active_profile_saved_tool_offsets = None
                self._active_profile_pick_error = ''
            return None, None

        try:
            current_mtime_ns = item_file.stat().st_mtime_ns
        except FileNotFoundError:
            current_mtime_ns = None
        except OSError as exc:
            self.get_logger().warn(
                f'Failed to stat active item yaml "{item_file}": {exc}'
            )
            current_mtime_ns = None

        if (
            not force
            and item_file == self._active_item_file
            and current_mtime_ns == self._active_item_file_mtime_ns
        ):
            with self._lock:
                saved_offsets = (
                    dict(self._active_profile_saved_tool_offsets)
                    if self._active_profile_saved_tool_offsets is not None
                    else None
                )
                return self._active_item_id, saved_offsets

        root = self._load_active_item_root(item_file)
        item_block = root.get(ITEM_SUBJECT_KIND) if isinstance(root, dict) else None
        teach_params = read_subject_params(root, TEACH_SECTION_KEY) if isinstance(root, dict) else None
        pick_params = read_subject_params(root, PICK_SECTION_KEY) if isinstance(root, dict) else None
        item_dimensions = self._normalize_item_dimensions(teach_params)
        saved_offsets = self._normalize_pick_params(pick_params)
        if saved_offsets is None and self._active_profile_pick_error:
            self.get_logger().error(
                f'Invalid pick profile in "{item_file}": {self._active_profile_pick_error}'
            )

        with self._lock:
            self._active_item_file = item_file
            self._active_item_file_mtime_ns = current_mtime_ns
            self._active_item_id = (
                str(item_block.get('id', '') or '').strip() or None
                if isinstance(item_block, dict)
                else None
            )
            self._active_item_display_name = (
                display_name_for_item_block(item_block)
                if isinstance(item_block, dict)
                else None
            )
            self._active_profile_saved_tool_offsets = (
                dict(saved_offsets) if saved_offsets is not None else None
            )
            self._active_item_dimensions_mm = (
                dict(item_dimensions) if item_dimensions is not None else None
            )
            if saved_offsets is not None:
                self._apply_saved_tool_offsets_locked(saved_offsets)
        return self._active_item_id, dict(saved_offsets) if saved_offsets is not None else None

    def get_active_profile_tool_offset_state(self, force: bool = False) -> tuple[str | None, dict[str, object] | None]:
        return self._sync_profile_tool_offsets_from_state(force=force)

    def get_active_item_metadata(self) -> tuple[Path | None, str | None, str | None]:
        """Return ``(items_yaml_path, item_id, display_name)`` snapshot.

        GUI/save paths need both the path (to call ``write_subject_section``)
        and the id+display name (to round-trip the ``item:`` block when
        the section writer reseeds an empty file). Cached on the node so
        the GUI side does not have to re-resolve the single file on every
        save click.
        """
        with self._lock:
            return (
                self._active_item_file,
                self._active_item_id,
                self._active_item_display_name,
            )

    @staticmethod
    def _solve_intercept_time_sec(
        relative_position_mm: tuple[float, float, float],
        target_velocity_mmps: tuple[float, float, float],
        interceptor_speed_mmps: float,
    ) -> float | None:
        s = max(1e-6, float(interceptor_speed_mmps))
        r = (
            float(relative_position_mm[0]),
            float(relative_position_mm[1]),
            float(relative_position_mm[2]),
        )
        v = (
            float(target_velocity_mmps[0]),
            float(target_velocity_mmps[1]),
            float(target_velocity_mmps[2]),
        )

        c = ItemPickNode._vector_dot3(r, r)
        if c <= 1e-9:
            return 0.0

        a = ItemPickNode._vector_dot3(v, v) - (s * s)
        b = 2.0 * ItemPickNode._vector_dot3(r, v)
        eps = 1e-9

        if abs(a) <= eps:
            if abs(b) <= eps:
                return None
            t_linear = -c / b
            return t_linear if t_linear >= 0.0 else None

        disc = (b * b) - (4.0 * a * c)
        if disc < 0.0:
            return None

        sqrt_disc = math.sqrt(max(0.0, disc))
        t1 = (-b - sqrt_disc) / (2.0 * a)
        t2 = (-b + sqrt_disc) / (2.0 * a)
        candidates = [t for t in (t1, t2) if t >= 0.0]
        if not candidates:
            return None
        return min(candidates)

    def _item_pose_camera_to_base(
        self,
        item_x_mm: float,
        item_y_mm: float,
        item_z_mm: float,
        item_rx_deg: float,
        item_ry_deg: float,
        item_rz_deg: float,
        camera_frame_id: str,
    ) -> tuple[
        float,
        float,
        float,
        float,
        float,
        float,
        tuple[float, float, float, float],
        tuple[float, float, float, float],
    ] | None:
        source_frame = str(camera_frame_id).strip() or 'camera_color_optical_frame'
        target_frame = self._robot_goal_frame_id
        tf_msg = self._lookup_item_camera_transform(target_frame, source_frame)
        if tf_msg is None:
            # Without this WARN the operator sees only "arm not moving"
            # because every downstream caller silently returns None on
            # this path (no service response, no GUI text in headless
            # mode, no arm motion).
            detail = getattr(self, '_last_item_tf_error', 'unknown TF error')
            self.get_logger().warn(
                f'TF lookup failed {target_frame}<-{source_frame}: {detail}. '
                'Item pick will abort this dispatch rather than applying the '
                'pose in a different camera frame. Check that item_detect is '
                'publishing the exact fixed-camera TF named by bin_seek_pose.'
            )
            self._set_action_text(f'TF lookup failed {target_frame}<-{source_frame}: {detail}')
            return None

        q_base_camera = self._quat_normalize((
            float(tf_msg.transform.rotation.x),
            float(tf_msg.transform.rotation.y),
            float(tf_msg.transform.rotation.z),
            float(tf_msg.transform.rotation.w),
        ))
        t_base_camera_m = (
            float(tf_msg.transform.translation.x),
            float(tf_msg.transform.translation.y),
            float(tf_msg.transform.translation.z),
        )
        p_camera_item_m = (
            float(item_x_mm) * 0.001,
            float(item_y_mm) * 0.001,
            float(item_z_mm) * 0.001,
        )
        p_base_item_offset_m = self._rotate_vector_by_quaternion(p_camera_item_m, q_base_camera)
        p_base_item_m = (
            t_base_camera_m[0] + p_base_item_offset_m[0],
            t_base_camera_m[1] + p_base_item_offset_m[1],
            t_base_camera_m[2] + p_base_item_offset_m[2],
        )

        q_camera_item = self._quat_normalize(
            self._rpy_deg_to_quaternion(item_rx_deg, item_ry_deg, item_rz_deg),
        )
        q_base_item = self._quat_normalize(self._quat_multiply(q_base_camera, q_camera_item))
        item_rpy_base_deg = self._quaternion_to_rpy_deg(q_base_item)

        return (
            p_base_item_m[0] * 1000.0,
            p_base_item_m[1] * 1000.0,
            p_base_item_m[2] * 1000.0,
            item_rpy_base_deg[0],
            item_rpy_base_deg[1],
            item_rpy_base_deg[2],
            q_base_item,
            q_base_camera,
        )

    def _lookup_item_camera_transform(
        self,
        target_frame: str,
        source_frame: str,
    ) -> TransformStamped | None:
        # The detector also publishes this static TF, but the pick node is the
        # safety-critical consumer. Re-publish the exact fixed-camera calibration
        # before lookup so a missed /tf_static delivery cannot brick picking.
        self._publish_fixed_camera_calibration_tf(source_frame)
        deadline = time.monotonic() + ITEM_CAMERA_TF_READY_WAIT_SEC
        errors: list[str] = []
        while True:
            errors.clear()
            try:
                return self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self._goal_tf_lookup_timeout_sec),
                )
            except TransformException as exc:
                errors.append(f'{target_frame}<-{source_frame}: {exc}')
            if time.monotonic() >= deadline:
                break
            time.sleep(ITEM_CAMERA_TF_RETRY_SLEEP_SEC)

        self._last_item_tf_error = '; '.join(errors) if errors else 'no TF lookup checked'
        fallback_tf = self._load_fixed_camera_calibration_tf(source_frame, warn_on_failure=True)
        if fallback_tf is not None:
            fallback_parent = self._normalize_tf_frame(fallback_tf.header.frame_id)
            requested_parent = self._normalize_tf_frame(target_frame)
            if fallback_parent == requested_parent:
                self._send_static_transform_cached(fallback_tf)
                self._warn_fixed_camera_tf_once(
                    f'fixed_camera_calibration_file:direct_fallback:{requested_parent}:{source_frame}',
                    'Fixed-camera TF lookup missed /tf_static; using exact calibration file '
                    f'directly for {requested_parent}->{source_frame}.',
                )
                return fallback_tf
            self._warn_fixed_camera_tf_once(
                f'fixed_camera_calibration_file:parent_mismatch:{fallback_parent}:{requested_parent}',
                'Fixed-camera calibration TF parent frame is '
                f'"{fallback_parent}", but item_pick requested "{requested_parent}". '
                'Refusing to alias frames.',
            )
        return None

    def _load_eye_in_hand_camera_offset_m(self) -> tuple[float, float, float] | None:
        """Parse the camera origin in the Link6 frame from eye-in-hand YAML.

        The eye-in-hand calibration stores ``transform.translation`` as the
        camera frame origin expressed in ``transform_parent_frame`` (the robot
        effector, Link6). That vector is exactly the camera offset in the
        gripper frame used to predict the wrist camera position for a candidate
        pick orientation. Returns None when the file is missing/unreadable;
        callers retry the file read and skip camera-bin preference if it is
        still unavailable. Runtime TF is intentionally not used for this value.
        """
        path_text = self._eye_in_hand_calibration_file
        if not path_text:
            return None
        path = Path(path_text).expanduser()
        root = self._safe_yaml_load(path)
        if root is None:
            self.get_logger().warn(
                f'Eye-in-hand calibration not loaded from "{path}"; '
                'camera-bin preference will retry the calibration file.'
            )
            return None

        calibration_type = self._yaml_str(root, 'calibration_type')
        if calibration_type and calibration_type != 'eye_in_hand':
            self.get_logger().warn(
                f'Calibration "{path}" has calibration_type "{calibration_type}", '
                'expected "eye_in_hand"; using its transform translation anyway.'
            )

        parent_frame = (
            self._yaml_str(root, 'transform_parent_frame')
            or self._yaml_str(root, 'robot_effector_frame')
        )
        transform = self._yaml_map(root, 'transform')
        translation = self._yaml_xyz_m(self._yaml_map(transform, 'translation'))
        if translation is None:
            self.get_logger().warn(
                f'Eye-in-hand calibration "{path}" has no usable '
                'transform.translation; camera-bin preference will retry the calibration file.'
            )
            return None

        if parent_frame and parent_frame != self._robot_gripper_frame_id:
            self.get_logger().warn(
                f'Eye-in-hand calibration parent frame "{parent_frame}" differs '
                f'from gripper frame "{self._robot_gripper_frame_id}"; using the '
                'translation as the camera offset in the gripper frame anyway.'
            )

        self.get_logger().info(
            'Loaded eye-in-hand camera offset from '
            f'"{path}": camera origin in {self._robot_gripper_frame_id} = '
            f'({translation[0] * 1000.0:.1f}, {translation[1] * 1000.0:.1f}, '
            f'{translation[2] * 1000.0:.1f}) mm.'
        )
        return translation

    def _lookup_camera_offset_in_gripper_m(self) -> tuple[float, float, float] | None:
        if self._camera_offset_gripper_m_calibration is not None:
            return self._camera_offset_gripper_m_calibration
        for attempt in range(1, EYE_IN_HAND_CALIBRATION_LOAD_RETRY_COUNT + 1):
            offset = self._load_eye_in_hand_camera_offset_m()
            if offset is not None:
                self._camera_offset_gripper_m_calibration = offset
                return offset
            if attempt < EYE_IN_HAND_CALIBRATION_LOAD_RETRY_COUNT:
                time.sleep(EYE_IN_HAND_CALIBRATION_LOAD_RETRY_SLEEP_SEC)
        self.get_logger().warn(
            'Camera-bin preference skipped: eye-in-hand calibration file unavailable; '
            'runtime TF fallback is disabled.'
        )
        return None

    def _camera_position_for_goal_m(
        self,
        goal_xyz_mm: tuple[float, float, float],
        goal_rotation_xyzw: tuple[float, float, float, float],
        camera_offset_gripper_m: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        camera_offset_base_m = self._rotate_vector_by_quaternion(
            camera_offset_gripper_m,
            goal_rotation_xyzw,
        )
        return (
            (float(goal_xyz_mm[0]) * 0.001) + camera_offset_base_m[0],
            (float(goal_xyz_mm[1]) * 0.001) + camera_offset_base_m[1],
            (float(goal_xyz_mm[2]) * 0.001) + camera_offset_base_m[2],
        )

    def _camera_inside_bin_area(
        self,
        camera_base_m: tuple[float, float, float],
        safety_area: BinCameraSafetyArea,
    ) -> tuple[bool, tuple[float, float, float]]:
        camera_bin_m = self._transform_point_inverse_m(
            camera_base_m,
            safety_area.base_to_bin_translation_m,
            safety_area.base_to_bin_rotation_xyzw,
        )
        inside = (
            safety_area.x_min_m <= camera_bin_m[0] <= safety_area.x_max_m and
            safety_area.y_min_m <= camera_bin_m[1] <= safety_area.y_max_m
        )
        return inside, camera_bin_m

    @staticmethod
    def _roi_violation_m(
        camera_bin_m: tuple[float, float, float],
        safety_area: BinCameraSafetyArea,
    ) -> float:
        """Planar distance the camera sits outside the bin ROI (0.0 if inside)."""
        dx = max(
            safety_area.x_min_m - camera_bin_m[0],
            0.0,
            camera_bin_m[0] - safety_area.x_max_m,
        )
        dy = max(
            safety_area.y_min_m - camera_bin_m[1],
            0.0,
            camera_bin_m[1] - safety_area.y_max_m,
        )
        return math.hypot(dx, dy)

    def _choose_camera_preferred_candidate_index(
        self,
        preferred_index: int,
        q_base_goal_candidates: tuple[tuple[float, float, float, float], ...],
        candidate_goal_xyz_mm: tuple[tuple[float, float, float], ...],
    ) -> CameraBinCandidateChoice:
        camera_check_enabled = (
            self._prefer_camera_inside_bin or self._require_camera_inside_bin
        )
        if not camera_check_enabled or len(q_base_goal_candidates) <= 1:
            return CameraBinCandidateChoice(preferred_index, '')

        safety_area, safety_reason = self._load_active_bin_camera_safety_area()
        if safety_area is None:
            if self._require_camera_inside_bin:
                return CameraBinCandidateChoice(
                    preferred_index,
                    f'Camera-bin requirement rejected pose: {safety_reason}',
                    valid=False,
                )
            return CameraBinCandidateChoice(preferred_index, safety_reason)

        camera_offset_gripper_m = self._lookup_camera_offset_in_gripper_m()
        if camera_offset_gripper_m is None:
            message = 'camera-bin preference skipped: eye-in-hand calibration unavailable'
            if self._require_camera_inside_bin:
                return CameraBinCandidateChoice(
                    preferred_index,
                    f'Camera-bin requirement rejected pose: {message}',
                    valid=False,
                )
            return CameraBinCandidateChoice(preferred_index, message)

        checks: list[tuple[bool, tuple[float, float, float]]] = []
        for idx, q_goal in enumerate(q_base_goal_candidates):
            camera_base_m = self._camera_position_for_goal_m(
                candidate_goal_xyz_mm[idx],
                q_goal,
                camera_offset_gripper_m,
            )
            checks.append(self._camera_inside_bin_area(camera_base_m, safety_area))

        preferred_inside, preferred_bin_m = checks[preferred_index]
        if preferred_inside:
            return CameraBinCandidateChoice(preferred_index, (
                f'Camera-bin preference: preferred pose keeps {self._camera_safety_frame_id} '
                f'inside {safety_area.bin_frame_id} '
                f'(x={preferred_bin_m[0] * 1000.0:.1f}, y={preferred_bin_m[1] * 1000.0:.1f} mm).'
            ))

        for idx, (inside, camera_bin_m) in enumerate(checks):
            if idx == preferred_index or not inside:
                continue
            message = (
                f'Camera-bin preference selected 180deg opposite item-X pose: '
                f'preferred camera would be outside {safety_area.bin_frame_id} '
                f'(x={preferred_bin_m[0] * 1000.0:.1f}, y={preferred_bin_m[1] * 1000.0:.1f} mm), '
                f'flipped camera is inside '
                f'(x={camera_bin_m[0] * 1000.0:.1f}, y={camera_bin_m[1] * 1000.0:.1f} mm).'
            )
            self.get_logger().info(message)
            return CameraBinCandidateChoice(idx, message)

        # Neither candidate is fully inside. In hard-required mode this pose is
        # rejected so seek can publish a fresh position instead of falling back
        # to an unsafe orientation.
        if self._require_camera_inside_bin:
            best_idx = min(
                range(len(checks)),
                key=lambda idx: self._roi_violation_m(checks[idx][1], safety_area),
            )
            best_bin_m = checks[best_idx][1]
            best_violation_mm = self._roi_violation_m(best_bin_m, safety_area) * 1000.0
            message = (
                f'Camera-bin requirement rejected pose: no orientation keeps '
                f'{self._camera_safety_frame_id} fully inside {safety_area.bin_frame_id}; '
                f'closest candidate was '
                f'(x={best_bin_m[0] * 1000.0:.1f}, y={best_bin_m[1] * 1000.0:.1f} mm, '
                f'outside by {best_violation_mm:.1f} mm).'
            )
            self.get_logger().warn(message)
            return CameraBinCandidateChoice(best_idx, message, valid=False)

        message = (
            f'Camera-bin preference warning: both item-X pose options put '
            f'{self._camera_safety_frame_id} outside {safety_area.bin_frame_id}; '
            f'continuing with preferred pick anyway '
            f'(x={preferred_bin_m[0] * 1000.0:.1f}, y={preferred_bin_m[1] * 1000.0:.1f} mm).'
        )
        self.get_logger().warn(message)
        return CameraBinCandidateChoice(preferred_index, message)

    def _publish_goal_debug_transform(
        self,
        parent_frame: str,
        child_frame: str,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        rx_deg: float,
        ry_deg: float,
        rz_deg: float,
    ) -> None:
        if not self._publish_goal_debug_tf:
            return

        frame_id = str(parent_frame).strip() or self._robot_goal_frame_id
        child = str(child_frame).strip() or self._post_stop_movel_goal_debug_frame_id

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = frame_id
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = float(x_mm) * 0.001
        tf_msg.transform.translation.y = float(y_mm) * 0.001
        tf_msg.transform.translation.z = float(z_mm) * 0.001
        qx, qy, qz, qw = self._rpy_deg_to_quaternion(
            float(rx_deg),
            float(ry_deg),
            float(rz_deg),
        )
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._send_static_transform_cached(tf_msg)

    def _publish_goal_debug_transforms(
        self,
        approach_goal: tuple[float, float, float, float, float, float],
        nominal_approach_goal: tuple[float, float, float, float, float, float],
        tool_offset: tuple[float, float, float, float, float, float],
    ) -> None:
        self._publish_goal_debug_transform(
            self._robot_goal_frame_id,
            self._post_stop_movel_goal_debug_frame_id,
            approach_goal[0],
            approach_goal[1],
            approach_goal[2],
            approach_goal[3],
            approach_goal[4],
            approach_goal[5],
        )
        self._publish_goal_debug_transform(
            self._robot_goal_frame_id,
            self._post_stop_movel_goal_nominal_debug_frame_id,
            nominal_approach_goal[0],
            nominal_approach_goal[1],
            nominal_approach_goal[2],
            nominal_approach_goal[3],
            nominal_approach_goal[4],
            nominal_approach_goal[5],
        )
        self._publish_goal_debug_transform(
            self._post_stop_movel_goal_nominal_debug_frame_id,
            self._post_stop_movel_goal_tool_offset_debug_frame_id,
            tool_offset[0],
            tool_offset[1],
            tool_offset[2],
            tool_offset[3],
            tool_offset[4],
            tool_offset[5],
        )
        self._publish_goal_flange_tcp_debug_transform()

    def _publish_goal_flange_tcp_debug_transform(self) -> None:
        """Publish the predicted Link6 (flange) pose at the goal TCP.

        The MovL goal pose tells the controller where to place its
        internal Tool N origin; the physical flange (Link6 in the URDF)
        then lands at ``goal_tcp * inverse(dobot_tool_offset)``. RViz
        models the URDF only up to Link6, so without this frame the
        operator cannot tell where the flange will end up when the
        controller subtracts its own Tool N TCP from the commanded
        pose. Publishing as a child of the primary goal_tcp frame keeps
        the frame anchored to the most recently dispatched pose.
        """
        offset_rx, offset_ry, offset_rz = (
            float(self._dobot_tool_offset_rx_deg),
            float(self._dobot_tool_offset_ry_deg),
            float(self._dobot_tool_offset_rz_deg),
        )
        offset_t_mm = (
            float(self._dobot_tool_offset_x_mm),
            float(self._dobot_tool_offset_y_mm),
            float(self._dobot_tool_offset_z_mm),
        )
        offset_q = self._rpy_deg_to_quaternion(offset_rx, offset_ry, offset_rz)
        offset_q_norm = self._quat_normalize(offset_q)
        inv_q = (-offset_q_norm[0], -offset_q_norm[1], -offset_q_norm[2], offset_q_norm[3])
        rotated_t_mm = self._rotate_vector_by_quaternion(offset_t_mm, inv_q)
        inv_t_mm = (-rotated_t_mm[0], -rotated_t_mm[1], -rotated_t_mm[2])
        inv_rpy_deg = self._quaternion_to_rpy_deg(inv_q)
        self._publish_goal_debug_transform(
            self._post_stop_movel_goal_debug_frame_id,
            self._post_stop_movel_goal_flange_tcp_debug_frame_id,
            inv_t_mm[0],
            inv_t_mm[1],
            inv_t_mm[2],
            inv_rpy_deg[0],
            inv_rpy_deg[1],
            inv_rpy_deg[2],
        )

    def _publish_goal_tool_axis_tips(self, axis_length_mm: float = GOAL_TF_DIAG_AXIS_LENGTH_MM) -> None:
        axis_len = max(1.0, float(axis_length_mm))
        self._publish_goal_debug_transform(
            self._post_stop_movel_goal_debug_frame_id,
            self._post_stop_movel_goal_tool_axis_x_tip_frame_id,
            axis_len,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self._publish_goal_debug_transform(
            self._post_stop_movel_goal_debug_frame_id,
            self._post_stop_movel_goal_tool_axis_y_tip_frame_id,
            0.0,
            axis_len,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self._publish_goal_debug_transform(
            self._post_stop_movel_goal_debug_frame_id,
            self._post_stop_movel_goal_tool_axis_z_tip_frame_id,
            0.0,
            0.0,
            axis_len,
            0.0,
            0.0,
            0.0,
        )

    def _publish_primary_goal_debug_transform(
        self,
        goal: tuple[float, float, float, float, float, float],
    ) -> None:
        self._publish_goal_debug_transform(
            self._robot_goal_frame_id,
            self._post_stop_movel_goal_debug_frame_id,
            goal[0],
            goal[1],
            goal[2],
            goal[3],
            goal[4],
            goal[5],
        )
        self._publish_goal_flange_tcp_debug_transform()

    def _item_pose_array_callback(self, msg: PoseArray) -> None:
        header_stamp_sec = self._builtin_time_to_sec(msg.header.stamp)
        header_stamp_ns = self._builtin_time_to_nanoseconds(msg.header.stamp)
        if not self._item_pose_stamp_is_fresh_for_motion(
            header_stamp_sec,
            str(getattr(self, '_item_pose_array_topic', ITEM_POSE_ARRAY_TOPIC_DEFAULT)),
        ):
            return
        frame_id = str(msg.header.frame_id)
        candidates = tuple(
            self._item_pose_target_from_pose(
                pose,
                frame_id,
                header_stamp_sec,
                acquisition_stamp_ns=header_stamp_ns,
            )
            for pose in msg.poses
        )
        with self._lock:
            self._latest_item_pose_candidates = candidates
            self._latest_item_pose_candidates_seq += 1
            candidate_seq = self._latest_item_pose_candidates_seq
            watch_armed = bool(self._item_pose_watch_armed)
            watch_generation = int(self._item_pose_watch_generation)
            watch_floor = int(self._item_pose_watch_seq_floor)
        if watch_armed and candidates:
            self.get_logger().info(
                "DATALOG item_pick_pose_array_cached: "
                f"candidates={len(candidates)} candidate_seq={candidate_seq} "
                f"watch_generation={watch_generation} pose_seq_floor={watch_floor} "
                f"frame={frame_id}"
            )

    def _item_pose_callback(self, msg: PoseStamped) -> None:
        header_stamp_sec = self._builtin_time_to_sec(msg.header.stamp)
        header_stamp_ns = self._builtin_time_to_nanoseconds(msg.header.stamp)
        if not self._item_pose_stamp_is_fresh_for_motion(
            header_stamp_sec,
            ITEM_POSE_TOPIC,
        ):
            return
        item_target = self._item_pose_target_from_pose(
            msg.pose,
            str(msg.header.frame_id),
            header_stamp_sec,
            acquisition_stamp_ns=header_stamp_ns,
        )

        should_send_stop = False
        tf_only_mode = False
        dispatch_target: ItemPoseTarget | None = None
        dispatch_candidates: tuple[ItemPoseTarget, ...] = (item_target,)
        post_speed_mm_s = 0.0
        x_offset_mm = 0.0
        y_offset_mm = 0.0
        z_offset_mm = 0.0
        use_fingers = USE_FINGERS_DEFAULT
        grab_on_pick = GRAB_ON_PICK_DEFAULT
        relax_fingers_on_pick = RELAX_FINGERS_ON_PICK_DEFAULT
        final_z_up_mm = 0.0
        pre_pick_settling_time_sec = 0.0
        pick_settling_time_sec = 0.0
        tool_offset_x_mm = 0.0
        tool_offset_y_mm = 0.0
        tool_offset_z_mm = 0.0
        tool_offset_rx_deg = 0.0
        tool_offset_ry_deg = 0.0
        tool_offset_rz_deg = 0.0
        watch_timeout_sec = ITEM_POSE_WATCH_TIMEOUT_SEC
        # Diagnostic note: every silent ``return`` below used to swallow
        # the pose without any log line, which made "arm not moving" bugs
        # untraceable from the docker logs. The skip_reason / log-once
        # plumbing here turns each path into an INFO/WARN line so the
        # operator can see WHY a published bin_seek_pose did not produce
        # a MovL.
        skip_reason: str | None = None
        with self._lock:
            self._item_pose_seq += 1
            self._last_item_target = item_target
            current_seq = self._item_pose_seq
            current_floor = self._item_pose_watch_seq_floor
            current_deadline = self._item_pose_watch_deadline_monotonic
            if bool(getattr(self, '_lifecycle_stop_latched', False)):
                skip_reason = 'lifecycle stop latch active'
            elif not self._item_pose_watch_armed:
                skip_reason = 'not armed'
            elif self._item_pose_seq <= self._item_pose_watch_seq_floor:
                skip_reason = (
                    f'seq {self._item_pose_seq} <= floor '
                    f'{self._item_pose_watch_seq_floor} (stale pose from '
                    'before arm)'
                )
            elif self._item_pose_watch_stop_dispatched:
                skip_reason = 'stop already dispatched for this arming cycle'
            elif time.monotonic() > self._item_pose_watch_deadline_monotonic:
                watch_timeout_sec = float(self._item_pose_watch_timeout_sec)
                self._reset_runtime_state_locked(
                    f'No item pose within {watch_timeout_sec:.0f}s. Node reset.'
                )
                self.get_logger().warn(
                    f'Item pose received but watch deadline exceeded '
                    f'(timeout={watch_timeout_sec:.1f}s); resetting watch.'
                )
                return

            if skip_reason is not None:
                # Don't log every dropped pose -- the camera spams at 30Hz
                # once detection is happy. The not-armed case is expected
                # during place-to-next-pick prefetch; stale/deadline/order
                # violations remain WARN so real sequencing bugs stand out.
                if not getattr(self, '_item_pose_skip_warned', False):
                    message = (
                        'Dropping bin_seek_pose: '
                        f'{skip_reason} '
                        f'(seq={current_seq}, floor={current_floor}, '
                        f'frame_id="{item_target.frame_id}", '
                        f'pos_mm={item_target.position_mm})'
                    )
                    if skip_reason == 'not armed':
                        self.get_logger().info(message)
                    else:
                        self.get_logger().warn(message)
                    self._item_pose_skip_warned = True
                return

            self._item_pose_watch_armed = False
            self._item_pose_watch_stop_dispatched = True
            tf_only_mode = bool(self._item_pose_watch_tf_only_mode)
            watch_timeout_sec = float(self._item_pose_watch_timeout_sec)
            should_send_stop = True
            dispatch_target = item_target
            dispatch_candidates = self._merge_item_pose_candidates(
                item_target,
                self._latest_item_pose_candidates,
            )
            post_speed_mm_s = float(self._post_stop_movel_speed_mm_s)
            x_offset_mm = float(self._post_stop_x_offset_mm)
            y_offset_mm = float(self._post_stop_y_offset_mm)
            z_offset_mm = float(self._post_stop_z_offset_mm)
            use_fingers = bool(self._use_fingers)
            grab_on_pick = bool(self._grab_on_pick)
            relax_fingers_on_pick = bool(self._relax_fingers_on_pick)
            final_z_up_mm = float(self._final_z_up_mm)
            pre_pick_settling_time_sec = float(self._pre_pick_settling_time_sec)
            pick_settling_time_sec = float(self._pick_settling_time_sec)
            tool_offset_x_mm = float(self._tool_offset_x_mm)
            tool_offset_y_mm = float(self._tool_offset_y_mm)
            tool_offset_z_mm = float(self._tool_offset_z_mm)
            tool_offset_rx_deg = float(self._tool_offset_rx_deg)
            tool_offset_ry_deg = float(self._tool_offset_ry_deg)
            tool_offset_rz_deg = float(self._tool_offset_rz_deg)

        if tf_only_mode:
            if should_send_stop and dispatch_target is not None:
                self.get_logger().info(
                    f'Consumed bin_seek_pose seq={current_seq} '
                    f'(frame_id="{dispatch_target.frame_id}", '
                    f'pos_mm={dispatch_target.position_mm}); '
                    'tf_only mode -> dispatching TF preview worker.'
                )
                self._set_action_text(
                    'Item pose update detected. Troubleshoot mode: goal TF preview only...'
                )
                worker = threading.Thread(
                    target=self._preview_goal_only_request,
                    args=(
                        dispatch_target,
                        x_offset_mm,
                        y_offset_mm,
                        z_offset_mm,
                        use_fingers,
                        grab_on_pick,
                        relax_fingers_on_pick,
                        final_z_up_mm,
                        pre_pick_settling_time_sec,
                        pick_settling_time_sec,
                        tool_offset_x_mm,
                        tool_offset_y_mm,
                        tool_offset_z_mm,
                        tool_offset_rx_deg,
                        tool_offset_ry_deg,
                        tool_offset_rz_deg,
                    ),
                    daemon=True,
                )
                worker.start()
            return

        if should_send_stop and dispatch_target is not None:
            self.get_logger().info(
                f'Consumed bin_seek_pose seq={current_seq} '
                f'(frame_id="{dispatch_target.frame_id}", '
                f'pos_mm={dispatch_target.position_mm}, '
                f'rpy_deg={dispatch_target.rpy_deg}); '
                f'offsets=(x={x_offset_mm:.1f},y={y_offset_mm:.1f},z={z_offset_mm:.1f})mm '
                f'approach_z={PICK_FIXED_APPROACH_Z_UP_MM:.1f}mm fixed '
                f'use_fingers={int(use_fingers)} grab_on_pick={int(grab_on_pick)} '
                f'relax_fingers_on_pick={int(relax_fingers_on_pick)} '
                f'final_z_up={final_z_up_mm:.1f}mm; '
                f'ranked_candidates={len(dispatch_candidates)}; '
                'dispatching pick MovL worker.'
            )
            self._set_action_text('Item pose update detected. Starting pick sequence...')
            worker = threading.Thread(
                target=self._send_movel_request,
                args=(
                    dispatch_target,
                    post_speed_mm_s,
                    x_offset_mm,
                    y_offset_mm,
                    z_offset_mm,
                    use_fingers,
                    grab_on_pick,
                    relax_fingers_on_pick,
                    final_z_up_mm,
                    pre_pick_settling_time_sec,
                    pick_settling_time_sec,
                    tool_offset_x_mm,
                    tool_offset_y_mm,
                    tool_offset_z_mm,
                    tool_offset_rx_deg,
                    tool_offset_ry_deg,
                    tool_offset_rz_deg,
                    dispatch_candidates,
                ),
                daemon=True,
            )
            worker.start()

    def _compute_base_goal_from_item_target(
        self,
        item_target: ItemPoseTarget,
        x_offset_mm: float,
        y_offset_mm: float,
        z_offset_mm: float,
        tool_offset_x_mm: float,
        tool_offset_y_mm: float,
        tool_offset_z_mm: float,
        tool_offset_rx_deg: float,
        tool_offset_ry_deg: float,
        tool_offset_rz_deg: float,
        ee_speed_mmps: float,
        predict_target_motion: bool = True,
    ) -> PredictedGoal | None:
        _ = ee_speed_mmps
        _ = predict_target_motion
        target_x, target_y, target_z = item_target.position_mm
        target_rx, target_ry, target_rz = item_target.rpy_deg
        frame_id = item_target.frame_id
        item_base_pose = self._item_pose_camera_to_base(
            target_x,
            target_y,
            target_z,
            target_rx,
            target_ry,
            target_rz,
            frame_id,
        )
        if item_base_pose is None:
            return None

        item_base_x, item_base_y, item_base_z, _, _, _, q_base_item, _ = item_base_pose
        # Sterilize item frame orientation first so generated goals are world-normal.
        item_yaw_deg = self._yaw_deg_from_quaternion_xy_axis(q_base_item)
        q_base_item_sterilized = self._quat_normalize(
            self._rpy_deg_to_quaternion(0.0, 0.0, item_yaw_deg)
        )
        item_local_x_in_base = self._rotate_vector_by_quaternion((1.0, 0.0, 0.0), q_base_item_sterilized)
        item_local_y_in_base = self._rotate_vector_by_quaternion((0.0, 1.0, 0.0), q_base_item_sterilized)
        item_local_z_in_base = self._rotate_vector_by_quaternion((0.0, 0.0, 1.0), q_base_item_sterilized)
        item_age_sec = 0.0
        item_now_x = item_base_x
        item_now_y = item_base_y
        item_now_z = item_base_z

        stand_off_vec_base_mm = (
            (item_local_x_in_base[0] * x_offset_mm)
            + (item_local_y_in_base[0] * y_offset_mm)
            + (item_local_z_in_base[0] * z_offset_mm),
            (item_local_x_in_base[1] * x_offset_mm)
            + (item_local_y_in_base[1] * y_offset_mm)
            + (item_local_z_in_base[1] * z_offset_mm),
            (item_local_x_in_base[2] * x_offset_mm)
            + (item_local_y_in_base[2] * y_offset_mm)
            + (item_local_z_in_base[2] * z_offset_mm),
        )
        desired_now_goal_mm = (
            item_now_x + stand_off_vec_base_mm[0],
            item_now_y + stand_off_vec_base_mm[1],
            item_now_z + stand_off_vec_base_mm[2],
        )

        lead_time_sec = 0.0
        nominal_x_goal = desired_now_goal_mm[0]
        nominal_y_goal = desired_now_goal_mm[1]
        nominal_z_goal = desired_now_goal_mm[2]

        # Build two valid item-aligned EE orientation candidates (+/- direction),
        # then choose the one with minimal rotation from current TCP.
        # No fixed 90-degree Z offset is baked here; use tool_offset_rz_deg for that.
        q_align_options = (
            self._rpy_deg_to_quaternion(180.0, 0.0, 0.0),
            self._rpy_deg_to_quaternion(180.0, 0.0, 180.0),
        )
        q_tool_offset = self._quat_normalize(
            self._rpy_deg_to_quaternion(tool_offset_rx_deg, tool_offset_ry_deg, tool_offset_rz_deg)
        )
        q_base_nominal_candidates = tuple(
            self._quat_normalize(self._quat_multiply(q_base_item_sterilized, q_align))
            for q_align in q_align_options
        )
        q_base_goal_raw_candidates = tuple(
            self._quat_normalize(self._quat_multiply(q_nominal, q_tool_offset))
            for q_nominal in q_base_nominal_candidates
        )
        # Final sterilization: all generated goal orientations keep Z world-normal.
        q_base_goal_candidates = tuple(
            self._sterilize_quaternion_to_world_normal(q_raw)
            for q_raw in q_base_goal_raw_candidates
        )
        preferred_candidate_idx = self._choose_min_rotation_candidate_index(q_base_goal_candidates)
        candidate_tool_offsets_base_mm = tuple(
            self._rotate_vector_by_quaternion(
                (tool_offset_x_mm, tool_offset_y_mm, tool_offset_z_mm),
                q_nominal,
            )
            for q_nominal in q_base_nominal_candidates
        )
        candidate_goal_xyz_mm = tuple(
            (
                nominal_x_goal + tool_offset_base_mm[0],
                nominal_y_goal + tool_offset_base_mm[1],
                nominal_z_goal + tool_offset_base_mm[2],
            )
            for tool_offset_base_mm in candidate_tool_offsets_base_mm
        )
        # Keep the proven pre-repick behavior: choose orientation from the
        # current TCP and camera-bin checks. Do not require a parseable
        # home-referenced joint solution before trying a cached candidate.
        self._last_camera_bin_reject_reason = ''
        camera_choice = self._choose_camera_preferred_candidate_index(
            preferred_candidate_idx,
            q_base_goal_candidates,
            candidate_goal_xyz_mm,
        )
        if not camera_choice.valid:
            self._last_camera_bin_reject_reason = camera_choice.message
            self._set_action_text(camera_choice.message)
            return None
        selected_candidate_idx = camera_choice.selected_index
        camera_safety_message = camera_choice.message
        q_base_nominal_goal = q_base_nominal_candidates[selected_candidate_idx]
        q_base_goal = q_base_goal_candidates[selected_candidate_idx]
        target_x_goal, target_y_goal, target_z_goal = candidate_goal_xyz_mm[selected_candidate_idx]

        nominal_rx_deg, nominal_ry_deg, nominal_rz_deg = self._quaternion_to_rpy_deg(q_base_nominal_goal)
        goal_rx_deg, goal_ry_deg, goal_rz_deg = self._quaternion_to_rpy_deg(q_base_goal)
        orientation_choice = 'preferred'
        if selected_candidate_idx != preferred_candidate_idx:
            orientation_choice = 'flipped_180'

        return PredictedGoal(
            x_mm=target_x_goal,
            y_mm=target_y_goal,
            z_mm=target_z_goal,
            rx_deg=goal_rx_deg,
            ry_deg=goal_ry_deg,
            rz_deg=goal_rz_deg,
            source_frame_id=frame_id,
            lead_time_sec=lead_time_sec,
            item_age_sec=item_age_sec,
            item_speed_base_mmps=0.0,
            nominal_x_mm=nominal_x_goal,
            nominal_y_mm=nominal_y_goal,
            nominal_z_mm=nominal_z_goal,
            nominal_rx_deg=nominal_rx_deg,
            nominal_ry_deg=nominal_ry_deg,
            nominal_rz_deg=nominal_rz_deg,
            orientation_choice=orientation_choice,
            camera_safety_message=camera_safety_message,
        )

    def _validate_motion_goal(
        self,
        goal: tuple[float, float, float, float, float, float],
        label: str,
    ) -> bool:
        result = validate_tcp_pose(goal, label=label)
        if result.ok:
            return True
        message = f'Motion validation failed before robot command: {result.message}'
        self.get_logger().error(message)
        self._set_action_text(message)
        return False

    def _send_movel_goal(
        self,
        goal: tuple[float, float, float, float, float, float],
        reference_pose: tuple[float, float, float, float, float, float] | None,
        speed_mm_s: float,
        label_prefix: str,
        forced_v_percent: int | None = None,
        forced_a_percent: int | None = None,
    ) -> tuple[bool, int, str]:
        label_lower = str(label_prefix or '').lower()
        if 'final z-up' in label_lower or 'final z' in label_lower:
            trace_name = 'item_pick_final_z_up'
        elif 'retract' in label_lower:
            trace_name = 'item_pick_retract_motion'
        else:
            trace_name = 'item_pick_movl'
        token = function_timing.record_function_start(None, trace_name)
        try:
            return self._send_movel_goal_traced(
                goal,
                reference_pose,
                speed_mm_s,
                label_prefix,
                forced_v_percent,
                forced_a_percent,
            )
        finally:
            function_timing.record_function_return(token)

    def _send_movel_goal_traced(
        self,
        goal: tuple[float, float, float, float, float, float],
        reference_pose: tuple[float, float, float, float, float, float] | None,
        speed_mm_s: float,
        label_prefix: str,
        forced_v_percent: int | None = None,
        forced_a_percent: int | None = None,
    ) -> tuple[bool, int, str]:
        _ = (reference_pose, speed_mm_s)
        if forced_v_percent is not None:
            v_percent = max(1, min(100, int(forced_v_percent)))
            mapping_source = 'forced'
        else:
            v_percent = 100
            mapping_source = 'locked_max'
        a_percent = DEFAULT_ACC_PERCENT
        if forced_a_percent is not None:
            a_percent = max(1, min(100, int(forced_a_percent)))
        movl_request = MovL.Request()
        movl_request.mode = False
        movl_request.a = float(goal[0])
        movl_request.b = float(goal[1])
        movl_request.c = float(goal[2])
        movl_request.d = float(goal[3])
        movl_request.e = float(goal[4])
        movl_request.f = float(goal[5])
        movl_request.param_value = self._build_motion_param_value(v_percent, a_percent)
        movl_label = (
            f'{label_prefix}('
            f'{movl_request.a:.1f},{movl_request.b:.1f},{movl_request.c:.1f},'
            f'{movl_request.d:.2f},{movl_request.e:.2f},{movl_request.f:.2f},'
            f'v={v_percent},a={a_percent})'
        )
        if not self._validate_motion_goal(
            (movl_request.a, movl_request.b, movl_request.c, movl_request.d, movl_request.e, movl_request.f),
            movl_label,
        ):
            return False, v_percent, 'validation_failed'
        movl_response = self._call_service(self._mov_l_client, movl_request, movl_label)
        if movl_response is None:
            return False, v_percent, mapping_source
        if int(getattr(movl_response, 'res', -1)) < 0:
            return False, v_percent, mapping_source
        return True, v_percent, mapping_source

    def _send_movj_goal(
        self,
        goal: tuple[float, float, float, float, float, float],
        reference_pose: tuple[float, float, float, float, float, float] | None,
        speed_mm_s: float,
        label_prefix: str,
        forced_v_percent: int | None = None,
        forced_a_percent: int | None = None,
    ) -> tuple[bool, int, str]:
        trace_name = (
            'item_pick_approach_motion'
            if 'approach' in str(label_prefix or '').lower()
            else 'item_pick_movj'
        )
        token = function_timing.record_function_start(None, trace_name)
        try:
            return self._send_movj_goal_traced(
                goal,
                reference_pose,
                speed_mm_s,
                label_prefix,
                forced_v_percent,
                forced_a_percent,
            )
        finally:
            function_timing.record_function_return(token)

    def _send_movj_goal_traced(
        self,
        goal: tuple[float, float, float, float, float, float],
        reference_pose: tuple[float, float, float, float, float, float] | None,
        speed_mm_s: float,
        label_prefix: str,
        forced_v_percent: int | None = None,
        forced_a_percent: int | None = None,
    ) -> tuple[bool, int, str]:
        _ = (reference_pose, speed_mm_s)
        if forced_v_percent is not None:
            v_percent = max(1, min(100, int(forced_v_percent)))
            mapping_source = 'forced'
        else:
            v_percent = 100
            mapping_source = 'locked_max'
        a_percent = DEFAULT_ACC_PERCENT
        if forced_a_percent is not None:
            a_percent = max(1, min(100, int(forced_a_percent)))

        movj_request = MovJ.Request()
        movj_request.mode = False
        movj_request.a = float(goal[0])
        movj_request.b = float(goal[1])
        movj_request.c = float(goal[2])
        movj_request.d = float(goal[3])
        movj_request.e = float(goal[4])
        movj_request.f = float(goal[5])
        movj_request.param_value = self._build_motion_param_value(v_percent, a_percent)
        movj_label = (
            f'{label_prefix}('
            f'{movj_request.a:.1f},{movj_request.b:.1f},{movj_request.c:.1f},'
            f'{movj_request.d:.2f},{movj_request.e:.2f},{movj_request.f:.2f},'
            f'v={v_percent},a={a_percent})'
        )
        if not self._validate_motion_goal(
            (movj_request.a, movj_request.b, movj_request.c, movj_request.d, movj_request.e, movj_request.f),
            movj_label,
        ):
            return False, v_percent, 'validation_failed'
        movj_response = self._call_service(self._mov_j_client, movj_request, movj_label)
        if movj_response is None:
            return False, v_percent, mapping_source
        if int(getattr(movj_response, 'res', -1)) < 0:
            return False, v_percent, mapping_source
        return True, v_percent, mapping_source

    def _send_movjio_goal(
        self,
        goal: tuple[float, float, float, float, float, float],
        reference_pose: tuple[float, float, float, float, float, float] | None,
        speed_mm_s: float,
        label_prefix: str,
        mdis: list[str] | None = None,
        forced_v_percent: int | None = None,
        forced_a_percent: int | None = None,
    ) -> tuple[bool, int, str]:
        trace_name = (
            'item_pick_approach_motion'
            if 'approach' in str(label_prefix or '').lower()
            else 'item_pick_movjio'
        )
        token = function_timing.record_function_start(None, trace_name)
        try:
            return self._send_movjio_goal_traced(
                goal,
                reference_pose,
                speed_mm_s,
                label_prefix,
                mdis,
                forced_v_percent,
                forced_a_percent,
            )
        finally:
            function_timing.record_function_return(token)

    def _send_movjio_goal_traced(
        self,
        goal: tuple[float, float, float, float, float, float],
        reference_pose: tuple[float, float, float, float, float, float] | None,
        speed_mm_s: float,
        label_prefix: str,
        mdis: list[str] | None = None,
        forced_v_percent: int | None = None,
        forced_a_percent: int | None = None,
    ) -> tuple[bool, int, str]:
        _ = (reference_pose, speed_mm_s)
        if not self._wait_for_service(self._mov_jio_client, 'MovJIO'):
            return False, 0, 'service_unavailable'
        if forced_v_percent is not None:
            v_percent = max(1, min(100, int(forced_v_percent)))
            mapping_source = 'forced'
        else:
            v_percent = 100
            mapping_source = 'locked_max'
        a_percent = DEFAULT_ACC_PERCENT
        if forced_a_percent is not None:
            a_percent = max(1, min(100, int(forced_a_percent)))

        movjio_request = MovJIO.Request()
        movjio_request.mode = False
        movjio_request.a = float(goal[0])
        movjio_request.b = float(goal[1])
        movjio_request.c = float(goal[2])
        movjio_request.d = float(goal[3])
        movjio_request.e = float(goal[4])
        movjio_request.f = float(goal[5])
        movjio_request.mdis = list(mdis) if mdis is not None else []
        movjio_request.param_value = self._build_motion_param_value(v_percent, a_percent)

        mdis_label = ''
        if movjio_request.mdis:
            mdis_label = f',mdis={";".join(movjio_request.mdis)}'
        movjio_label = (
            f'{label_prefix}('
            f'{movjio_request.a:.1f},{movjio_request.b:.1f},{movjio_request.c:.1f},'
            f'{movjio_request.d:.2f},{movjio_request.e:.2f},{movjio_request.f:.2f},'
            f'v={v_percent},a={a_percent}{mdis_label})'
        )
        if not self._validate_motion_goal(
            (
                movjio_request.a,
                movjio_request.b,
                movjio_request.c,
                movjio_request.d,
                movjio_request.e,
                movjio_request.f,
            ),
            movjio_label,
        ):
            return False, v_percent, 'validation_failed'
        movjio_response = self._call_service(self._mov_jio_client, movjio_request, movjio_label)
        if movjio_response is None:
            return False, v_percent, mapping_source
        if int(getattr(movjio_response, 'res', -1)) < 0:
            return False, v_percent, mapping_source
        return True, v_percent, mapping_source

    def _send_fixed_home_movj_goal(self, label_prefix: str) -> tuple[bool, int, str]:
        token = function_timing.record_function_start(None, 'item_pick_movj')
        try:
            return self._send_fixed_home_movj_goal_traced(label_prefix)
        finally:
            function_timing.record_function_return(token)

    def _send_fixed_home_movj_goal_traced(self, label_prefix: str) -> tuple[bool, int, str]:
        self._last_fixed_home_movj_command_id = None
        self._last_fixed_home_joint_feedback_seq_baseline = None
        v_percent = 100
        validation = validate_joint_degrees(
            FIXED_HOME_JOINTS_DEG,
            label=f'{label_prefix} joint MovJ',
        )
        if not validation.ok:
            message = f'Motion validation failed before robot command: {validation.message}'
            self.get_logger().error(message)
            self._set_action_text(message)
            return False, v_percent, 'validation_failed'

        movj_request = MovJ.Request()
        movj_request.mode = True
        movj_request.a = float(FIXED_HOME_JOINTS_DEG[0])
        movj_request.b = float(FIXED_HOME_JOINTS_DEG[1])
        movj_request.c = float(FIXED_HOME_JOINTS_DEG[2])
        movj_request.d = float(FIXED_HOME_JOINTS_DEG[3])
        movj_request.e = float(FIXED_HOME_JOINTS_DEG[4])
        movj_request.f = float(FIXED_HOME_JOINTS_DEG[5])
        movj_request.param_value = [FIXED_HOME_MOVJ_PARAM]
        movj_label = (
            f'{label_prefix}('
            f'j1={movj_request.a:.2f},j2={movj_request.b:.2f},j3={movj_request.c:.2f},'
            f'j4={movj_request.d:.2f},j5={movj_request.e:.2f},j6={movj_request.f:.2f},'
            f'{FIXED_HOME_MOVJ_PARAM})'
        )
        movj_response = self._call_service(self._mov_j_client, movj_request, movj_label)
        if movj_response is None:
            return False, v_percent, 'forced'
        if int(getattr(movj_response, 'res', -1)) < 0:
            return False, v_percent, 'forced'
        robot_return = getattr(movj_response, 'robot_return', '')
        command_id = self._parse_controller_command_id(robot_return)
        self._last_fixed_home_movj_command_id = command_id
        _names, _positions, joint_seq, _received = self._joint_feedback_snapshot()
        # The service response is the earliest proof that the controller
        # accepted this command. Samples received while the request was in
        # flight predate that boundary and must not certify home completion.
        self._last_fixed_home_joint_feedback_seq_baseline = joint_seq
        if command_id is None:
            self.get_logger().warn(
                f'No controller command id returned for queued {label_prefix}: {robot_return!r}'
            )
        else:
            self.get_logger().info(
                f'DATALOG item_pick_fixed_home_command_id: label="{label_prefix}" command_id={command_id}'
            )
        return True, v_percent, 'forced'

    @staticmethod
    def _build_movelio_do_token(
        mode: int,
        distance: int,
        index: int,
        status: int,
    ) -> str:
        return f'{{{int(mode)},{int(distance)},{int(index)},{int(status)}}}'

    def _build_pick_approach_mdis(
        self,
        *,
        relax_fingers_on_pick: bool = RELAX_FINGERS_ON_PICK_DEFAULT,
    ) -> list[str]:
        open_status = 0 if relax_fingers_on_pick else 1
        return [
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                MOVJIO_APPROACH_OPEN_DISTANCE_PERCENT,
                GRIPPER_DO_CLOSE_INDEX,
                0,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                MOVJIO_APPROACH_OPEN_DISTANCE_PERCENT,
                GRIPPER_DO_OPEN_INDEX,
                open_status,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                MOVJIO_APPROACH_OPEN_DISTANCE_PERCENT,
                GRIPPER_DO_SUCTION_INDEX,
                0,
            ),
        ]

    def _build_pick_descent_mdis(self) -> list[str]:
        return [
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                MOVLIO_PICKUP_START_DISTANCE_PERCENT,
                GRIPPER_DO_SUCTION_INDEX,
                1,
            )
        ]

    def _build_pick_retract_mdis(
        self,
        *,
        use_fingers: bool,
        grab_on_pick: bool,
        relax_fingers_on_pick: bool = RELAX_FINGERS_ON_PICK_DEFAULT,
    ) -> list[str]:
        if use_fingers and grab_on_pick:
            return [
                self._build_movelio_do_token(
                    MOVLIO_DO_MODE_PERCENT,
                    MOVLIO_PICKUP_START_DISTANCE_PERCENT,
                    GRIPPER_DO_SUCTION_INDEX,
                    1,
                )
            ]
        open_status = 0 if use_fingers or relax_fingers_on_pick else 1
        close_status = 1 if use_fingers else 0
        return [
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                MOVLIO_RETRACT_CLOSE_DISTANCE_PERCENT,
                GRIPPER_DO_OPEN_INDEX,
                open_status,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                MOVLIO_RETRACT_CLOSE_DISTANCE_PERCENT,
                GRIPPER_DO_CLOSE_INDEX,
                close_status,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                MOVLIO_RETRACT_CLOSE_DISTANCE_PERCENT,
                GRIPPER_DO_SUCTION_INDEX,
                1,
            ),
        ]

    def _build_no_di_final_z_up_mdis(
        self,
        *,
        use_fingers: bool,
        relax_fingers_on_pick: bool = RELAX_FINGERS_ON_PICK_DEFAULT,
    ) -> list[str]:
        open_at_start = 0 if use_fingers or relax_fingers_on_pick else 1
        close_at_start = 1 if use_fingers and not relax_fingers_on_pick else 0
        open_after_purge = 0 if use_fingers or relax_fingers_on_pick else 1
        return [
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                0,
                GRIPPER_DO_OPEN_INDEX,
                open_at_start,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                0,
                GRIPPER_DO_CLOSE_INDEX,
                close_at_start,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                0,
                GRIPPER_DO_SUCTION_INDEX,
                0,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                0,
                GRIPPER_DO_PURGE_INDEX,
                1,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                MOVLIO_NO_DI_PURGE_OFF_DISTANCE_PERCENT,
                GRIPPER_DO_CLOSE_INDEX,
                0,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                MOVLIO_NO_DI_PURGE_OFF_DISTANCE_PERCENT,
                GRIPPER_DO_OPEN_INDEX,
                open_after_purge,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                MOVLIO_NO_DI_PURGE_OFF_DISTANCE_PERCENT,
                GRIPPER_DO_PURGE_INDEX,
                0,
            ),
        ]

    def _build_drop_return_release_mdis(
        self,
        *,
        io_distance_percent: int = DROP_RELEASE_IO_DISTANCE_PERCENT,
    ) -> list[str]:
        io_distance_percent = max(0, min(100, int(io_distance_percent)))
        return [
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                io_distance_percent,
                GRIPPER_DO_CLOSE_INDEX,
                0,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                io_distance_percent,
                GRIPPER_DO_SUCTION_INDEX,
                0,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                io_distance_percent,
                GRIPPER_DO_OPEN_INDEX,
                1,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                io_distance_percent,
                GRIPPER_DO_PURGE_INDEX,
                1,
            ),
        ]

    def _build_drop_return_retract_mdis(self) -> list[str]:
        return [
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                DROP_RETRACT_NEUTRAL_IO_DISTANCE_PERCENT,
                GRIPPER_DO_CLOSE_INDEX,
                0,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                DROP_RETRACT_NEUTRAL_IO_DISTANCE_PERCENT,
                GRIPPER_DO_OPEN_INDEX,
                0,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                DROP_RETRACT_NEUTRAL_IO_DISTANCE_PERCENT,
                GRIPPER_DO_SUCTION_INDEX,
                0,
            ),
            self._build_movelio_do_token(
                MOVLIO_DO_MODE_PERCENT,
                DROP_RETRACT_NEUTRAL_IO_DISTANCE_PERCENT,
                GRIPPER_DO_PURGE_INDEX,
                0,
            ),
        ]

    def _send_movelio_goal(
        self,
        goal: tuple[float, float, float, float, float, float],
        reference_pose: tuple[float, float, float, float, float, float] | None,
        speed_mm_s: float,
        label_prefix: str,
        mdis: list[str] | None = None,
        forced_v_percent: int | None = None,
        forced_a_percent: int | None = None,
    ) -> tuple[bool, int, str]:
        label_lower = str(label_prefix or '').lower()
        if 'descent' in label_lower:
            trace_name = 'item_pick_descent_motion'
        elif 'retract' in label_lower:
            trace_name = 'item_pick_retract_motion'
        else:
            trace_name = 'item_pick_movelio'
        token = function_timing.record_function_start(None, trace_name)
        try:
            return self._send_movelio_goal_traced(
                goal,
                reference_pose,
                speed_mm_s,
                label_prefix,
                mdis,
                forced_v_percent,
                forced_a_percent,
            )
        finally:
            function_timing.record_function_return(token)

    def _send_movelio_goal_traced(
        self,
        goal: tuple[float, float, float, float, float, float],
        reference_pose: tuple[float, float, float, float, float, float] | None,
        speed_mm_s: float,
        label_prefix: str,
        mdis: list[str] | None = None,
        forced_v_percent: int | None = None,
        forced_a_percent: int | None = None,
    ) -> tuple[bool, int, str]:
        _ = (reference_pose, speed_mm_s)
        if not self._wait_for_service(self._mov_lio_client, 'MovLIO'):
            return False, 0, 'service_unavailable'
        if forced_v_percent is not None:
            v_percent = max(1, min(100, int(forced_v_percent)))
            mapping_source = 'forced'
        else:
            v_percent = 100
            mapping_source = 'locked_max'
        a_percent = DEFAULT_ACC_PERCENT
        if forced_a_percent is not None:
            a_percent = max(1, min(100, int(forced_a_percent)))

        movlio_request = MovLIO.Request()
        movlio_request.mode = False
        movlio_request.a = float(goal[0])
        movlio_request.b = float(goal[1])
        movlio_request.c = float(goal[2])
        movlio_request.d = float(goal[3])
        movlio_request.e = float(goal[4])
        movlio_request.f = float(goal[5])
        movlio_request.mdis = list(mdis) if mdis is not None else []
        movlio_request.param_value = self._build_motion_param_value(v_percent, a_percent)

        mdis_label = ''
        if movlio_request.mdis:
            mdis_label = f',mdis={";".join(movlio_request.mdis)}'
        movlio_label = (
            f'{label_prefix}('
            f'{movlio_request.a:.1f},{movlio_request.b:.1f},{movlio_request.c:.1f},'
            f'{movlio_request.d:.2f},{movlio_request.e:.2f},{movlio_request.f:.2f},'
            f'v={v_percent},a={a_percent}{mdis_label})'
        )
        if not self._validate_motion_goal(
            (movlio_request.a, movlio_request.b, movlio_request.c, movlio_request.d, movlio_request.e, movlio_request.f),
            movlio_label,
        ):
            return False, v_percent, 'validation_failed'
        movlio_response = self._call_service(self._mov_lio_client, movlio_request, movlio_label)
        if movlio_response is None:
            return False, v_percent, mapping_source
        if int(getattr(movlio_response, 'res', -1)) < 0:
            return False, v_percent, mapping_source
        return True, v_percent, mapping_source

    def _log_pick_queue_command_rejected(
        self,
        command_label: str,
        v_percent: int,
        status: str,
    ) -> None:
        self.get_logger().warn(
            'DATALOG item_pick_queue_command_rejected: '
            f'command="{command_label}" effective_v={int(v_percent)} status={status}'
        )

    def _log_pick_queue_all_sent(
        self,
        command_statuses: tuple[tuple[str, int, str], ...],
    ) -> None:
        commands = '; '.join(
            f'{label} v={int(v_percent)}/{status}'
            for label, v_percent, status in command_statuses
        )
        self.get_logger().info(
            'DATALOG item_pick_queue_all_sent: '
            f'all queued pick commands accepted by controller; commands={commands}'
        )

    def _send_stop_command(self, label: str) -> tuple[bool, str]:
        if not self._wait_for_service(self._stop_client, 'Stop', timeout_sec=2.0):
            return False, 'Stop service unavailable'
        response = self._call_service(
            self._stop_client,
            Stop.Request(),
            label,
            timeout_sec=4.0,
        )
        if response is None:
            return False, 'Stop returned no response'
        res = int(getattr(response, 'res', -1))
        robot_return = str(getattr(response, 'robot_return', '')).strip()
        detail = f'res={res}'
        if robot_return:
            detail += f' robot_return="{robot_return}"'
        return res >= 0, detail

    def _stop_if_cancelled_after_motion_dispatch(self, context: str) -> bool:
        """Close the race between an in-flight service call and Stop.

        The lifecycle cancel service may send ``Stop`` while a MovJ/MovL
        request is still awaiting its acknowledgement.  If that request is
        accepted just after Stop, the queued command would otherwise survive.
        Re-check cancellation after every accepted motion call and issue a
        final Stop before the worker exits.
        """
        if not self._is_cancel_requested():
            return False
        stop_ok, stop_message = self._send_stop_command(
            f'Stop() [cancel after {context}]'
        )
        message = (
            f'Sequence cancelled after {context}; '
            f'Stop {"accepted" if stop_ok else "failed"} ({stop_message}).'
        )
        self.get_logger().warn(message)
        self._set_action_text(message)
        return True

    def _send_do(self, index: int, status: int, time_ms: int = 0) -> bool:
        trace_name = (
            'item_pick_suction_do'
            if int(index) == GRIPPER_DO_SUCTION_INDEX
            else 'item_pick_do'
        )
        token = function_timing.record_function_start(None, trace_name)
        try:
            if not self._wait_for_service(self._do_client, 'DO'):
                return False

            request = DO.Request()
            request.index = int(index)
            request.status = int(status)
            request.time = int(time_ms)
            label = f'DO(index={request.index},status={request.status},time={request.time})'
            response = self._call_service(
                self._do_client,
                request,
                label,
                timeout_sec=4.0,
            )
            return response is not None and int(getattr(response, 'res', -1)) >= 0
        finally:
            function_timing.record_function_return(token)

    def _set_cp(self, percent: int, label: str) -> bool:
        percent = max(1, min(100, int(percent)))
        if not self._wait_for_service(self._cp_client, 'CP', timeout_sec=2.0):
            return False

        request = CP.Request()
        request.r = percent
        response = self._call_service(
            self._cp_client,
            request,
            f'CP {label} ({percent}%)',
            timeout_sec=4.0,
        )
        return response is not None and int(getattr(response, 'res', -1)) >= 0

    def _set_speed_factor(self, percent: int, label: str) -> bool:
        percent = max(1, min(100, int(percent)))
        if not self._wait_for_service(self._speed_factor_client, 'SpeedFactor', timeout_sec=2.0):
            return False

        request = SpeedFactor.Request()
        request.ratio = percent
        response = self._call_service(
            self._speed_factor_client,
            request,
            f'SpeedFactor {label} ({percent}%)',
            timeout_sec=4.0,
        )
        return response is not None and int(getattr(response, 'res', -1)) >= 0

    def relax_gripper(self) -> tuple[bool, str]:
        """Drive the gripper to a fully neutral state (DO1=DO2=DO3=DO4=0).

        All four DO commands are fired via call_async without waiting
        for acknowledgements (fire-and-forget). The Dobot DO service
        applies outputs on hardware regardless of the res code it returns,
        so blocking on each ack is unnecessary and risks holding the
        single-threaded executor for too long when a negative-res or
        slow response arrives.
        """
        self._set_action_text('Gripper relax: firing DO1, DO2, DO3, DO4 to OFF...')
        if not self._do_client.service_is_ready():
            msg = 'Gripper relax skipped: DO service not ready'
            self._set_action_text(msg)
            return True, msg
        for index in (
            GRIPPER_DO_CLOSE_INDEX,
            GRIPPER_DO_OPEN_INDEX,
            GRIPPER_DO_SUCTION_INDEX,
            GRIPPER_DO_PURGE_INDEX,
        ):
            req = DO.Request()
            req.index = int(index)
            req.status = 0
            req.time = 0
            self._do_client.call_async(req)
        self._set_action_text('Gripper relaxed (DO1=DO2=DO3=DO4=OFF).')
        return True, 'Gripper relaxed (DO1=DO2=DO3=DO4=OFF)'

    @function_timing.traced("item_pick_gripper_relax")
    def _gripper_relax_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """ROS Trigger entrypoint for ``item_pick/relax_gripper``.

        Used by the edge lifecycle bridge during ``cmd.prepare_start``
        / ``cmd.prepare_stop`` to neutralise the gripper before motion.
        Mirrors the contract of the existing ``Trigger`` services on
        this node (``success`` boolean + human-readable ``message``).
        """
        del request
        try:
            success, message = self.relax_gripper()
        except Exception as exc:  # pragma: no cover - defensive logging path
            self.get_logger().error(f'Gripper relax service crashed: {exc}')
            response.success = False
            response.message = f'Gripper relax raised: {exc}'
            return response
        response.success = bool(success)
        response.message = message
        return response

    def _cached_attempt_with_progress(
        self,
        attempt: CachedPickAttempt,
        *,
        next_candidate_index: int | None = None,
        successful_candidate_index: int | None = None,
    ) -> CachedPickAttempt:
        candidate_count = len(attempt.candidates)
        if next_candidate_index is None:
            next_index = int(attempt.next_candidate_index)
        else:
            next_index = int(next_candidate_index)
        next_index = max(0, min(candidate_count, next_index))
        if successful_candidate_index is None:
            success_index = attempt.successful_candidate_index
        else:
            success_index = int(successful_candidate_index)
        return replace(
            attempt,
            next_candidate_index=next_index,
            successful_candidate_index=success_index,
        )

    def _set_cached_pick_progress(
        self,
        *,
        next_candidate_index: int,
        successful_candidate_index: int | None = None,
    ) -> None:
        with self._lock:
            attempt = self._cached_pick_attempt
            if attempt is None:
                return
            self._cached_pick_attempt = self._cached_attempt_with_progress(
                attempt,
                next_candidate_index=next_candidate_index,
                successful_candidate_index=successful_candidate_index,
            )

    def _lifecycle_start_blocked_locked(self, context: str) -> bool:
        """Reject new pick-side motion while lifecycle Stop/Reset is latched.

        Callers must hold ``self._lock``.  The gate is intentionally separate
        from the transient worker-cancel flag: normal in-run candidate/repick
        recovery remains automatic, while lifecycle cleanup gets a persistent
        barrier against callbacks that were already queued when Stop arrived.
        """
        if not bool(getattr(self, '_lifecycle_stop_latched', False)):
            return False
        self._snapshot.action_text = (
            f'{context} blocked: lifecycle stop is latched; '
            'wait for successful prepare/resume after verified home.'
        )
        return True

    @function_timing.traced("item_pick_retry_cached_candidate")
    def _retry_cached_candidate_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Start the next same-frame cached YOLO candidate, if one remains."""
        del request
        with self._lock:
            if self._lifecycle_start_blocked_locked('Cached candidate retry'):
                response.success = False
                response.message = str(self._snapshot.action_text)
                return response
            attempt = self._cached_pick_attempt
            if attempt is None:
                response.success = False
                response.message = 'No cached item-pick candidates available'
                return response
            candidate_count = len(attempt.candidates)
            start_index = max(0, min(candidate_count, int(attempt.next_candidate_index)))
            if start_index >= candidate_count:
                response.success = False
                response.message = (
                    f'No remaining cached item-pick candidates '
                    f'({start_index}/{candidate_count} already consumed)'
                )
                return response
            if self._snapshot.busy or int(getattr(self, '_active_pick_workers', 0)) > 0:
                response.success = False
                response.message = 'Cannot retry cached candidate while item pick is busy'
                return response
            if self._drop_last_inflight:
                response.success = False
                response.message = 'Cannot retry cached candidate while item return/recovery is active'
                return response
            if self._last_base_drop_release_goal is not None:
                # A failed local repick resolves and caches its drop goal before
                # motion/DI confirmation.  If that worker later times out, the
                # goal can survive even though no item was acquired.  Fresh DI
                # no-held is the only safe authority to self-heal that stale
                # cache; held or unknown remains fail-closed for Stop/Return.
                try:
                    held_available, held, held_message = self._held_item_state()
                except Exception as exc:  # pragma: no cover - defensive safety path
                    held_available, held = False, None
                    held_message = f'held-item DI check raised: {exc}'
                if held_available and held is False:
                    self._last_base_drop_release_goal = None
                    self._last_item_target = None
                    self.get_logger().warn(
                        'Cleared stale cached pick/drop target before cached-candidate retry '
                        f'because fresh DI confirms no item held: {held_message}'
                    )
                else:
                    response.success = False
                    response.message = (
                        'Cannot retry cached candidate while a cached last pick/drop target is still present; '
                        'post-pick drop recovery has not released the held-item target; '
                        f'{held_message}'
                    )
                    return response

            self._snapshot.busy = True
            self._snapshot.action_text = (
                f'Retrying cached item candidate {start_index + 1}/{candidate_count}...'
            )
            self._cancel_requested = False
            self._last_base_drop_release_goal = None
            self._last_item_target = None

        worker = threading.Thread(
            target=self._send_movel_request,
            args=(
                attempt.candidates[start_index],
                attempt.post_speed_mm_s,
                attempt.x_offset_mm,
                attempt.y_offset_mm,
                attempt.z_offset_mm,
                attempt.use_fingers,
                attempt.grab_on_pick,
                attempt.relax_fingers_on_pick,
                attempt.final_z_up_mm,
                attempt.pre_pick_settling_time_sec,
                attempt.pick_settling_time_sec,
                attempt.tool_offset_x_mm,
                attempt.tool_offset_y_mm,
                attempt.tool_offset_z_mm,
                attempt.tool_offset_rx_deg,
                attempt.tool_offset_ry_deg,
                attempt.tool_offset_rz_deg,
                attempt.candidates,
            ),
            kwargs={
                'pick_candidates_override': attempt.candidates,
                'start_candidate_index': start_index,
            },
            daemon=True,
        )
        worker.start()
        response.success = True
        response.message = (
            f'Cached item-pick retry started at candidate '
            f'{start_index + 1}/{candidate_count}'
        )
        return response

    @function_timing.traced("item_pick_clear_candidate_cache")
    def _clear_candidate_cache_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Clear same-frame YOLO retry candidates after tray target acquisition."""
        del request
        with self._lock:
            had_cache = self._cached_pick_attempt is not None
            self._cached_pick_attempt = None
        response.success = True
        response.message = (
            'Cleared cached same-frame item-pick candidates'
            if had_cache
            else 'No cached same-frame item-pick candidates to clear'
        )
        return response

    @function_timing.traced("item_pick_post_pick_drop_recover_cached")
    def _post_pick_drop_recover_cached_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Return a suspected dropped item to the bin using the cached pick pose."""
        del request
        busy_wait_started = False
        with self._lock:
            if self._lifecycle_start_blocked_locked('Post-pick drop recovery'):
                response.success = False
                response.message = f'{self._snapshot.action_text} [home_unsafe=1]'
                return response
            if self._drop_last_inflight:
                response.success = False
                response.message = 'Post-pick drop recovery already in progress [home_unsafe=1]'
                return response
            if self._snapshot.busy or int(getattr(self, '_active_pick_workers', 0)) > 0:
                busy_wait_started = True

        if busy_wait_started:
            self._set_action_text(
                'Post-pick drop recovery: waiting for active pick sequence to become idle...'
            )
            if not self._wait_for_drop_last_item_idle():
                response.success = False
                response.message = (
                    'Cannot recover post-pick drop while item pick sequence is active; '
                    'state unknown after bounded wait [home_unsafe=1]'
                )
                return response

        with self._lock:
            if self._lifecycle_start_blocked_locked('Post-pick drop recovery'):
                response.success = False
                response.message = f'{self._snapshot.action_text} [home_unsafe=1]'
                return response
            attempt = self._cached_pick_attempt
            release_goal = self._last_base_drop_release_goal
            if self._drop_last_inflight:
                response.success = False
                response.message = 'Post-pick drop recovery already in progress [home_unsafe=1]'
                return response
            if self._snapshot.busy or int(getattr(self, '_active_pick_workers', 0)) > 0:
                response.success = False
                response.message = (
                    'Cannot recover post-pick drop while item pick sequence is active [home_unsafe=1]'
                )
                return response
            if attempt is None or not attempt.candidates:
                response.success = False
                response.message = 'No cached item candidates for post-pick drop recovery [home_unsafe=1]'
                return response
            if release_goal is None:
                response.success = False
                response.message = 'No cached pick/drop target for post-pick drop recovery [home_unsafe=1]'
                return response
            self._drop_last_inflight = True
            self._cancel_requested = False
            post_speed_mm_s = float(attempt.post_speed_mm_s)
            final_z_up_mm = float(attempt.final_z_up_mm)
            next_candidate_index = max(
                0,
                min(len(attempt.candidates), int(attempt.next_candidate_index)),
            )
            failed_candidate_index = (
                int(attempt.successful_candidate_index)
                if attempt.successful_candidate_index is not None
                else max(0, next_candidate_index - 1)
            )
            failed_candidate_index = max(
                0,
                min(len(attempt.candidates) - 1, failed_candidate_index),
            )
            failed_pick_target = attempt.candidates[failed_candidate_index]

        result_holder: dict = {
            'success': False,
            'message': 'Post-pick drop recovery did not run',
            'home_unsafe': False,
        }
        self._set_action_text(
            'Post-pick drop recovery: returning directly above cached pick pose...'
        )
        try:
            worker = threading.Thread(
                target=self._post_pick_drop_recover_cached_worker,
                args=(
                    release_goal,
                    post_speed_mm_s,
                    final_z_up_mm,
                    next_candidate_index,
                    len(attempt.candidates),
                    result_holder,
                ),
                daemon=True,
            )
            worker.start()
            worker.join()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f'Post-pick drop recovery dispatch crashed: {exc}')
            result_holder = {
                'success': False,
                'message': f'Post-pick drop recovery dispatch raised: {exc}',
                'home_unsafe': True,
            }
        finally:
            if result_holder.get('success'):
                failed_area_published = self._publish_failed_pick_area(
                    failed_pick_target,
                    reason='post_pick_drop_recovery',
                )
                self._set_cached_pick_progress(
                    next_candidate_index=next_candidate_index,
                )
                if next_candidate_index < len(attempt.candidates):
                    cache_message = (
                        'Post-pick drop recovery preserved same-seek cached candidates: '
                        f'next_candidate={next_candidate_index + 1}/{len(attempt.candidates)} '
                        f'failed_area_published={int(failed_area_published)}'
                    )
                else:
                    cache_message = (
                        'Post-pick drop recovery exhausted same-seek cached candidates: '
                        f'next_candidate={next_candidate_index}/{len(attempt.candidates)} '
                        f'failed_area_published={int(failed_area_published)}'
                    )
                self.get_logger().info(cache_message)
            with self._lock:
                self._drop_last_inflight = False
                if result_holder.get('success'):
                    self._last_base_drop_release_goal = None
                    self._last_item_target = None

        message = str(result_holder.get('message', ''))
        if result_holder.get('home_unsafe'):
            message = f'{message} [home_unsafe=1]'
        response.success = bool(result_holder.get('success'))
        response.message = message
        return response

    @function_timing.traced("item_pick_clear_last_target")
    def _clear_last_target_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Invalidate the cached last pick/drop target.

        Called by the lifecycle bridge at safe boundaries (prepare_start,
        DI-confirmed no-held stop/reset, and non-overlapped place completion)
        so a later stop does not drop a phantom item. Always succeeds; reports
        whether a goal was present.
        """
        del request
        with self._lock:
            had_goal = self._last_base_drop_release_goal is not None
            self._last_base_drop_release_goal = None
            self._last_item_target = None
        response.success = True
        response.message = (
            'Cleared cached last pick/drop target'
            if had_goal
            else 'No cached last pick/drop target to clear'
        )
        return response

    @function_timing.traced("item_pick_drop_last_item")
    def _drop_last_item_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """ROS Trigger entrypoint for ``item_pick/drop_last_item``.

        Returns the most recently picked item to its cached base-frame pick
        pose, releases it there, retracts, returns to fixed home, and only
        then reports success. When no target is cached, success is allowed
        only if DI confirms no item is held; a held/unknown item without a
        cache is home-unsafe.

        This callback runs in a ReentrantCallbackGroup; while it joins the
        motion worker, the MultiThreadedExecutor keeps resolving the
        worker's MovJ/DO client responses on another thread.
        """
        del request
        busy_wait_started = False
        with self._lock:
            if self._drop_last_inflight:
                response.success = False
                response.message = 'Drop last item already in progress [home_unsafe=1]'
                return response
            if self._snapshot.busy or int(getattr(self, '_active_pick_workers', 0)) > 0:
                busy_wait_started = True

        if busy_wait_started:
            self._set_action_text(
                'Drop last item: waiting for active pick sequence to finish before stop cleanup...'
            )
            if not self._wait_for_drop_last_item_idle():
                response.success = False
                response.message = (
                    'Cannot drop last item while a pick sequence is active; '
                    'state unknown after bounded wait [home_unsafe=1]'
                )
                return response

        with self._lock:
            release_goal = self._last_base_drop_release_goal
            if self._drop_last_inflight:
                response.success = False
                response.message = 'Drop last item already in progress [home_unsafe=1]'
                return response
            if self._snapshot.busy or int(getattr(self, '_active_pick_workers', 0)) > 0:
                response.success = False
                response.message = (
                    'Cannot drop last item while a pick sequence is active [home_unsafe=1]'
                )
                return response
            if release_goal is not None:
                self._drop_last_inflight = True
                # Lifecycle cancellation deliberately remains latched against
                # new picks, but this synchronous return/drop worker is the
                # approved cleanup motion.  The active-pick barrier above has
                # proved the cancelled worker exited, so clearing its transient
                # flag cannot revive it.
                self._cancel_requested = False
                post_speed_mm_s = float(self._post_stop_movel_speed_mm_s)

        if release_goal is None:
            # Held-item state may read hardware/IO; keep it outside the node
            # lock so a slow DI sample cannot block item_pick state updates.
            try:
                available, held, held_message = self._held_item_state()
            except Exception as exc:  # pragma: no cover - defensive safety path
                available, held, held_message = False, None, f'held-item check raised: {exc}'
            with self._lock:
                # If an active worker finished and populated the cache while
                # the DI check was in flight, use the cache instead of failing
                # with a stale "no target" result.
                release_goal = self._last_base_drop_release_goal
                if release_goal is not None:
                    if self._drop_last_inflight:
                        response.success = False
                        response.message = 'Drop last item already in progress [home_unsafe=1]'
                        return response
                    if self._snapshot.busy or int(getattr(self, '_active_pick_workers', 0)) > 0:
                        response.success = False
                        response.message = (
                            'Cannot drop last item while a pick sequence is active [home_unsafe=1]'
                        )
                        return response
                    self._drop_last_inflight = True
                    self._cancel_requested = False
                    post_speed_mm_s = float(self._post_stop_movel_speed_mm_s)
                else:
                    if bool(held):
                        response.success = False
                        response.message = (
                            'holding item but no cached last pick target available '
                            f'({held_message}) [home_unsafe=1]'
                        )
                        return response
                    if not available:
                        response.success = False
                        response.message = (
                            'held item state unknown and no cached last pick target available '
                            f'({held_message}) [home_unsafe=1]'
                        )
                        return response
                    response.success = True
                    response.message = 'nothing to drop: no cached last pick target'
                    return response

        result_holder: dict = {
            'success': False,
            'message': 'Drop did not run',
            'home_unsafe': False,
        }
        self._set_action_text('Drop last item: returning through fixed home to cached last pick target...')
        try:
            worker = threading.Thread(
                target=self._drop_last_item_worker,
                args=(release_goal, post_speed_mm_s, result_holder),
                daemon=True,
            )
            worker.start()
            worker.join()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f'Drop last item dispatch crashed: {exc}')
            result_holder = {
                'success': False,
                'message': f'Drop dispatch raised: {exc}',
                'home_unsafe': True,
            }
        finally:
            with self._lock:
                self._drop_last_inflight = False
                if result_holder.get('success'):
                    self._last_base_drop_release_goal = None
                    self._last_item_target = None

        message = str(result_holder.get('message', ''))
        if result_holder.get('home_unsafe'):
            message = f'{message} [home_unsafe=1]'
        response.success = bool(result_holder.get('success'))
        response.message = message
        return response

    def _wait_for_drop_last_item_idle(
        self,
        timeout_sec: float | None = None,
    ) -> bool:
        """Wait until an active pick sequence is idle enough for drop-last.

        The stop cleanup must not dispatch a cached held-item return while
        the item-pick worker may still be approaching, descending, retracting,
        or final-Z-up. This wait is deliberately bounded so a stuck worker
        turns into a home-unsafe stop failure instead of hanging the service.
        """
        if timeout_sec is None:
            timeout_sec = DROP_LAST_BUSY_WAIT_TIMEOUT_SEC
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok():
            with self._lock:
                busy = bool(self._snapshot.busy)
            if not busy:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(DROP_LAST_BUSY_WAIT_POLL_SEC)
        return False

    @staticmethod
    def _drop_return_poses(
        release_goal: PredictedGoal,
    ) -> tuple[
        tuple[float, float, float, float, float, float],
        tuple[float, float, float, float, float, float],
    ]:
        release_z_mm = release_goal.z_mm + DROP_RELEASE_ABOVE_PICK_Z_MM
        safe_hover_z_mm = release_goal.z_mm + DROP_SAFE_HOVER_ABOVE_PICK_Z_MM
        release_pose = (
            release_goal.x_mm,
            release_goal.y_mm,
            release_z_mm,
            release_goal.rx_deg,
            release_goal.ry_deg,
            release_goal.rz_deg,
        )
        safe_hover_pose = (
            release_goal.x_mm,
            release_goal.y_mm,
            safe_hover_z_mm,
            release_goal.rx_deg,
            release_goal.ry_deg,
            release_goal.rz_deg,
        )
        return safe_hover_pose, release_pose

    def _queue_cached_pick_release_and_home(
        self,
        release_goal: PredictedGoal,
        post_speed_mm_s: float,
        *,
        context_label: str,
        initial_home_label: str | None,
        release_io_distance_percent: int = DROP_RELEASE_IO_DISTANCE_PERCENT,
        queue_final_home: bool = True,
    ) -> tuple[bool, str]:
        safe_hover_pose, release_pose = self._drop_return_poses(release_goal)
        self.get_logger().info(
            f'{context_label}: queued safe bin return '
            f'safe_hover=({safe_hover_pose[0]:.1f},{safe_hover_pose[1]:.1f},{safe_hover_pose[2]:.1f}) '
            f'release_pose=({release_pose[0]:.1f},{release_pose[1]:.1f},{release_pose[2]:.1f}) '
            f'hover_above_pick={DROP_SAFE_HOVER_ABOVE_PICK_Z_MM:.1f}mm '
            f'release_above_pick={DROP_RELEASE_ABOVE_PICK_Z_MM:.1f}mm'
        )

        if self._is_cancel_requested():
            return False, f'{context_label}: cancelled before motion'
        if not self._wait_for_service(self._mov_j_client, 'MovJ'):
            return False, f'{context_label}: MovJ unavailable'
        self._publish_primary_goal_debug_transform(release_pose)

        command_statuses: list[tuple[str, int, str]] = []
        if initial_home_label is not None:
            self._set_action_text(f'{context_label}: queueing fixed-home via before bin return...')
            via_ok, via_v, via_map = self._send_fixed_home_movj_goal(initial_home_label)
            command_statuses.append((initial_home_label, via_v, via_map))
            if not via_ok:
                self._log_pick_queue_command_rejected(initial_home_label, via_v, via_map)
                return False, f'{context_label}: fixed-home via MovJ failed before bin return'

        hover_label = f'{context_label}: MovJ safe hover 150 mm above cached pick'
        self._set_action_text(f'{context_label}: queueing safe hover 150 mm above cached pick...')
        hover_ok, hover_v, hover_map = self._send_movj_goal(
            safe_hover_pose,
            None,
            post_speed_mm_s,
            hover_label,
            forced_v_percent=100,
            forced_a_percent=100,
        )
        command_statuses.append((hover_label, hover_v, hover_map))
        if not hover_ok:
            self._log_pick_queue_command_rejected(hover_label, hover_v, hover_map)
            return False, f'{context_label}: safe-hover MovJ failed'

        release_label = f'{context_label}: MovLIO descend to 50 mm above pick with release/purge'
        self._set_action_text(f'{context_label}: queueing descent to 50 mm with release/purge...')
        release_ok, release_v, release_map = self._send_movelio_goal(
            release_pose,
            safe_hover_pose,
            post_speed_mm_s,
            release_label,
            mdis=self._build_drop_return_release_mdis(
                io_distance_percent=release_io_distance_percent,
            ),
            forced_v_percent=100,
            forced_a_percent=100,
        )
        command_statuses.append((release_label, release_v, release_map))
        if not release_ok:
            self._log_pick_queue_command_rejected(release_label, release_v, release_map)
            return False, f'{context_label}: release descent MovLIO failed'

        retract_label = f'{context_label}: MovLIO neutralize gripper at drop pose then retract to 150 mm'
        self._set_action_text(
            f'{context_label}: queueing gripper neutral at drop pose, then retract to 150 mm...'
        )
        retract_ok, retract_v, retract_map = self._send_movelio_goal(
            safe_hover_pose,
            release_pose,
            post_speed_mm_s,
            retract_label,
            mdis=self._build_drop_return_retract_mdis(),
            forced_v_percent=100,
            forced_a_percent=100,
        )
        command_statuses.append((retract_label, retract_v, retract_map))
        if not retract_ok:
            self._log_pick_queue_command_rejected(retract_label, retract_v, retract_map)
            return False, f'{context_label}: safe-hover retract MovLIO failed'

        if not queue_final_home:
            self._log_pick_queue_all_sent(tuple(command_statuses))
            self._set_action_text(f'{context_label}: waiting for +150 mm hover after release/retract...')
            if not self._wait_for_tcp_xyz_goal(safe_hover_pose[:3]):
                return False, f'{context_label}: +150 mm hover after release/retract was not proven'
            return (
                True,
                (
                    f'{context_label}: queued +{DROP_SAFE_HOVER_ABOVE_PICK_Z_MM:.0f} -> '
                    f'+{DROP_RELEASE_ABOVE_PICK_Z_MM:.0f} release/purge -> '
                    f'+{DROP_SAFE_HOVER_ABOVE_PICK_Z_MM:.0f} complete; ready for cached retry'
                ),
            )

        final_home_label = f'{context_label}: MovJ final fixed home after queued release/purge'
        self._set_action_text(f'{context_label}: queueing final fixed home after bin release...')
        home_ok, home_v, home_map = self._send_fixed_home_movj_goal(final_home_label)
        command_statuses.append((final_home_label, home_v, home_map))
        if not home_ok:
            self._log_pick_queue_command_rejected(final_home_label, home_v, home_map)
            return False, f'{context_label}: final fixed-home MovJ failed'

        self._log_pick_queue_all_sent(tuple(command_statuses))
        home_command_id = self._last_fixed_home_movj_command_id
        wait_ok, wait_message = self._wait_for_queued_fixed_home_completion(
            home_command_id,
            context_label=context_label,
        )
        if not wait_ok:
            return False, wait_message
        return (
            True,
            (
                f'{context_label}: queued +{DROP_SAFE_HOVER_ABOVE_PICK_Z_MM:.0f} -> '
                f'+{DROP_RELEASE_ABOVE_PICK_Z_MM:.0f} release/purge -> '
                f'+{DROP_SAFE_HOVER_ABOVE_PICK_Z_MM:.0f} -> fixed home complete; '
                f'{wait_message}'
            ),
        )

    @function_timing.traced("item_pick_drop_last_worker")
    def _drop_last_item_worker(
        self,
        release_goal: PredictedGoal,
        post_speed_mm_s: float,
        result_holder: dict,
    ) -> None:
        """Queue fixed-home via, safe bin release/retract with 80% IO, and final home."""
        try:
            queued_ok, queued_message = self._queue_cached_pick_release_and_home(
                release_goal,
                post_speed_mm_s,
                context_label='Stop/Return',
                initial_home_label='MovJ fixed home via before drop last item return',
                release_io_distance_percent=POST_PICK_DROP_RELEASE_IO_DISTANCE_PERCENT,
            )
            if not queued_ok:
                result_holder.update(success=False, message=queued_message, home_unsafe=True)
                return
            result_holder.update(
                success=True,
                message=(
                    'Returned held item via queued '
                    f'+{DROP_SAFE_HOVER_ABOVE_PICK_Z_MM:.0f} -> '
                    f'+{DROP_RELEASE_ABOVE_PICK_Z_MM:.0f} release/purge -> '
                    f'+{DROP_SAFE_HOVER_ABOVE_PICK_Z_MM:.0f} -> fixed home'
                ),
                home_unsafe=False,
            )
            self._set_action_text(queued_message)
        except Exception as exc:
            self.get_logger().error(f'Drop-last-item worker crashed: {exc}')
            result_holder.update(success=False, message=f'Drop raised: {exc}', home_unsafe=True)

    @function_timing.traced("item_pick_post_pick_drop_recover_worker")
    def _post_pick_drop_recover_cached_worker(
        self,
        release_goal: PredictedGoal,
        post_speed_mm_s: float,
        final_z_up_mm: float,
        next_candidate_index: int,
        candidate_count: int,
        result_holder: dict,
    ) -> None:
        """Queue safe bin release/retract after a post-pick DI drop."""
        try:
            _ = final_z_up_mm
            next_candidate_text = (
                f'{next_candidate_index + 1}/{candidate_count}'
                if next_candidate_index < candidate_count
                else 'none remaining'
            )
            queued_ok, queued_message = self._queue_cached_pick_release_and_home(
                release_goal,
                post_speed_mm_s,
                context_label='Post-pick drop recovery',
                initial_home_label=None,
                release_io_distance_percent=POST_PICK_DROP_RELEASE_IO_DISTANCE_PERCENT,
                # Post-pick DI-drop recovery must leave the arm at +150 mm
                # over the cached bin pick point so retry_cached_candidate can
                # try the next same-seek YOLO pose directly, without homing.
                queue_final_home=False,
            )
            if not queued_ok:
                result_holder.update(
                    success=False,
                    message=queued_message,
                    home_unsafe=True,
                )
                return

            result_holder.update(
                success=True,
                message=(
                    'Post-pick drop recovery complete: '
                    f'returned/released via queued +{DROP_SAFE_HOVER_ABOVE_PICK_Z_MM:.0f} -> '
                    f'+{DROP_RELEASE_ABOVE_PICK_Z_MM:.0f} release/purge -> '
                    f'+{DROP_SAFE_HOVER_ABOVE_PICK_Z_MM:.0f}; '
                    f'next cached candidate {next_candidate_text}'
                ),
                home_unsafe=False,
            )
            self._set_action_text(queued_message)
        except Exception as exc:
            self.get_logger().error(f'Post-pick drop cached recovery worker crashed: {exc}')
            result_holder.update(
                success=False,
                message=f'Post-pick drop recovery raised: {exc}',
                home_unsafe=True,
            )

    def _gripper_set_open_hold(self) -> bool:
        self._set_action_text('Gripper open-hold: disable close (DO1 OFF), enable open (DO2 ON)...')
        if not self._send_do(1, 0):
            return False
        return self._send_do(2, 1)

    def _gripper_set_open_no_suction(self) -> bool:
        self._set_action_text(
            'Gripper pre-pick state: disable close (DO1 OFF), disable suction (DO3 OFF), enable open (DO2 ON)...'
        )
        if not self._send_do(GRIPPER_DO_CLOSE_INDEX, 0):
            return False
        if not self._send_do(GRIPPER_DO_SUCTION_INDEX, 0):
            return False
        return self._send_do(GRIPPER_DO_OPEN_INDEX, 1)

    def _gripper_set_close_hold(self) -> bool:
        self._set_action_text(
            'Gripper close-hold: disable open (DO2 OFF), enable close (DO1 ON), enable suction (DO3 ON)...'
        )
        if not self._send_do(GRIPPER_DO_OPEN_INDEX, 0):
            return False
        if not self._send_do(GRIPPER_DO_CLOSE_INDEX, 1):
            return False
        return self._send_do(GRIPPER_DO_SUCTION_INDEX, 1)

    def _wait_for_tcp_xyz_goal(
        self,
        goal_xyz_mm: tuple[float, float, float],
        tolerance_mm: float = TCP_GOAL_REACHED_TOLERANCE_MM,
        timeout_sec: float | None = TCP_GOAL_WAIT_TIMEOUT_SEC,
        update_action_text: bool = True,
    ) -> bool:
        tolerance = max(0.1, float(tolerance_mm))
        deadline = None
        if timeout_sec is not None:
            deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while rclpy.ok():
            if self._is_cancel_requested():
                if update_action_text:
                    self._set_action_text('Sequence cancelled while waiting for pick position.')
                return False
            if deadline is not None and time.monotonic() >= deadline:
                break
            snapshot = self.snapshot()
            if snapshot.tcp_stamp is None:
                time.sleep(0.02)
                continue
            dx = float(snapshot.tcp_values.get('x', 0.0)) - float(goal_xyz_mm[0])
            dy = float(snapshot.tcp_values.get('y', 0.0)) - float(goal_xyz_mm[1])
            dz = float(snapshot.tcp_values.get('z', 0.0)) - float(goal_xyz_mm[2])
            distance_mm = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
            if distance_mm <= tolerance:
                return True
            time.sleep(0.02)
        if not rclpy.ok():
            if update_action_text:
                self._set_action_text('ROS shutdown while waiting for pick pose reach.')
            return False
        snapshot = self.snapshot()
        current_text = 'unknown'
        if snapshot.tcp_stamp is not None:
            current_text = (
                f'x={snapshot.tcp_values.get("x", 0.0):.1f}, '
                f'y={snapshot.tcp_values.get("y", 0.0):.1f}, '
                f'z={snapshot.tcp_values.get("z", 0.0):.1f}'
            )
        timeout_text = (
            'Timeout waiting for pick pose reach '
            f'(goal x={goal_xyz_mm[0]:.1f}, y={goal_xyz_mm[1]:.1f}, '
            f'z={goal_xyz_mm[2]:.1f}; current {current_text}; '
            f'tol={tolerance:.1f} mm; timeout={float(timeout_sec or 0.0):.1f}s).'
        )
        self.get_logger().warn(timeout_text)
        self._set_action_text(timeout_text)
        return False

    @staticmethod
    def _parse_controller_command_id(robot_return: object) -> int | None:
        text = str(robot_return or '').strip()
        if not text:
            return None
        match = re.search(r'\{\s*(-?\d+)\s*\}', text) or re.search(r'-?\d+', text)
        if match is None:
            return None
        try:
            command_id = int(match.group(1) if match.lastindex else match.group(0))
        except ValueError:
            return None
        if command_id < 0:
            return None
        return command_id

    def _read_current_controller_command_id(self) -> int | None:
        if GetCurrentCommandId is None or self._get_current_command_id_client is None:
            return None
        if not self._get_current_command_id_client.service_is_ready():
            if not self._wait_for_service(
                self._get_current_command_id_client,
                'GetCurrentCommandId',
                timeout_sec=1.0,
            ):
                return None
        response = self._call_service(
            self._get_current_command_id_client,
            GetCurrentCommandId.Request(),
            'GetCurrentCommandId()',
            timeout_sec=1.0,
        )
        if response is None or int(getattr(response, 'res', -1)) < 0:
            return None
        return self._parse_controller_command_id(getattr(response, 'robot_return', ''))

    @staticmethod
    def _controller_command_id_reached(
        current_command_id: int,
        target_command_id: int,
        highest_command_id_seen: int | None,
    ) -> tuple[bool, str]:
        if int(current_command_id) >= int(target_command_id):
            return True, (
                f'controller command id {current_command_id} reached target {target_command_id}'
            )
        near_target_floor = max(
            int(target_command_id) - COMMAND_ID_WRAP_NEAR_TARGET_WINDOW,
            int(target_command_id) // 2,
        )
        if (
            int(target_command_id) > COMMAND_ID_WRAP_NEAR_TARGET_WINDOW
            and highest_command_id_seen is not None
            and int(highest_command_id_seen) >= near_target_floor
            and int(current_command_id) <= COMMAND_ID_WRAP_LOW_WATERMARK
        ):
            return True, (
                'controller command id wrapped/reset after queued target range '
                f'(target={target_command_id}, high_water={highest_command_id_seen}, '
                f'current={current_command_id})'
            )
        return False, ''

    def _wait_for_queued_fixed_home_completion(
        self,
        target_command_id: int | None,
        *,
        context_label: str,
    ) -> tuple[bool, str]:
        if target_command_id is None:
            message = (
                f'{context_label}: no controller command id returned for final fixed-home MovJ; '
                'cannot prove queued motion completion.'
            )
            self.get_logger().error(message)
            self._set_action_text(message)
            return False, message

        if GetCurrentCommandId is None or self._get_current_command_id_client is None:
            message = (
                f'{context_label}: GetCurrentCommandId service type is unavailable; '
                'cannot prove queued fixed-home completion.'
            )
            self.get_logger().error(message)
            self._set_action_text(message)
            return False, message

        timeout = max(0.1, float(PICK_COMMAND_ID_WAIT_TIMEOUT_SEC))
        deadline = time.monotonic() + timeout
        last_command_id: int | None = None
        highest_command_id_seen: int | None = None
        command_id_reached_message = ''
        self._set_action_text(
            f'{context_label}: waiting for final fixed-home command id {target_command_id}...'
        )
        while rclpy.ok():
            if self._is_cancel_requested():
                message = f'{context_label}: cancelled while waiting for queued fixed home.'
                self._set_action_text(message)
                return False, message
            current_command_id = self._read_current_controller_command_id()
            if current_command_id is not None:
                last_command_id = current_command_id
                if highest_command_id_seen is None or current_command_id > highest_command_id_seen:
                    highest_command_id_seen = current_command_id
                reached, reached_message = self._controller_command_id_reached(
                    current_command_id,
                    int(target_command_id),
                    highest_command_id_seen,
                )
                if reached:
                    command_id_reached_message = reached_message
                    break
            if time.monotonic() >= deadline:
                message = (
                    f'{context_label}: timeout waiting for final fixed-home command id '
                    f'{target_command_id} (last={last_command_id}).'
                )
                self.get_logger().error(message)
                self._set_action_text(message)
                return False, message
            time.sleep(PICK_COMMAND_ID_POLL_SEC)

        if not rclpy.ok():
            message = f'{context_label}: ROS shutdown while waiting for queued fixed home.'
            self._set_action_text(message)
            return False, message

        stable, stable_message = self._wait_for_tcp_stable_after_pick_home()
        if not stable:
            message = (
                f'{context_label}: final fixed-home command reached but robot did not settle. '
                f'{stable_message}'
            )
            self.get_logger().error(message)
            self._set_action_text(message)
            return False, message

        baseline_seq = getattr(
            self,
            '_last_fixed_home_joint_feedback_seq_baseline',
            None,
        )
        home_verified, home_message = self._wait_for_fresh_fixed_home_joint_feedback(
            newer_than_seq=baseline_seq,
        )
        if not home_verified:
            message = (
                f'{context_label}: final fixed-home command reached and TCP settled, '
                f'but joint target is unproven. {home_message}'
            )
            self.get_logger().error(message)
            self._set_action_text(message)
            return False, message

        message = (
            f'{context_label}: final fixed-home queue drained; '
            f'{command_id_reached_message}; {stable_message}; {home_message}'
        )
        self._set_action_text(message)
        return True, message

    def _monitor_pick_descent_to_goal(
        self,
        pick_goal: tuple[float, float, float, float, float, float],
        *,
        candidate_number: int,
        candidate_count: int,
        close_fingers_on_di: bool = False,
        timeout_sec: float = PICK_MONITORED_DESCENT_TIMEOUT_SEC,
    ) -> tuple[bool, bool, str]:
        """Watch TCP reach and held-item DI during the slow pick descent."""
        di_stop_sent = False
        di_stop_ok = False
        di_stop_message = ''
        di_stop_context = 'during pick descent'
        bottom_leeway_deadline: float | None = None
        bottom_leeway_distance_mm: float | None = None

        def _send_di_trigger_stop_once(
            picked_message: str,
            *,
            context: str,
        ) -> None:
            nonlocal di_stop_sent, di_stop_ok, di_stop_message, di_stop_context
            if di_stop_sent:
                return
            di_stop_context = context
            di_stop_ok, di_stop_message = self._send_stop_command('Stop() [pick DI trigger]')
            self.get_logger().info(
                'DATALOG item_pick_descent_di_stop: '
                f'candidate={candidate_number}/{candidate_count} '
                f'context="{context}" stop_ok={int(di_stop_ok)} '
                f'stop="{di_stop_message}" '
                f'poll_sec={PICK_DI_POLL_SEC:.4f} '
                f'di="{picked_message}"'
            )
            self._set_action_text(
                f'DI trigger seen for candidate {candidate_number}/{candidate_count}; '
                f'Stop {"accepted" if di_stop_ok else "failed"} ({di_stop_message}); '
                'treating item as picked.'
            )
            di_stop_sent = True

        def _finish_picked_after_stop(picked_message: str) -> tuple[bool, bool, str]:
            if not di_stop_sent:
                _send_di_trigger_stop_once(picked_message, context='DI held sample')
            if not di_stop_ok:
                self._clear_pick_di_session()
                message = (
                    f'Pick DI Stop failed for candidate {candidate_number}/{candidate_count}; '
                    f'aborting before settle/retract: {di_stop_message}'
                )
                self.get_logger().error(message)
                self._set_action_text(message)
                return False, False, message
            if close_fingers_on_di:
                if not self._gripper_set_close_hold():
                    self._clear_pick_di_session()
                    message = (
                        f'Immediate finger close failed for candidate '
                        f'{candidate_number}/{candidate_count}; aborting before settle/retract.'
                    )
                    self.get_logger().error(message)
                    self._set_action_text(message)
                    return False, False, message
                self.get_logger().info(
                    'DATALOG item_pick_di_immediate_finger_close: '
                    f'candidate={candidate_number}/{candidate_count} success=1'
                )
            if not self._wait_settling_time(
                PICK_DI_STOP_SETTLE_SEC,
                f'candidate {candidate_number}/{candidate_count} DI trigger stop',
            ):
                self._clear_pick_di_session()
                message = (
                    f'Sequence cancelled during post-Stop pick DI settle for candidate '
                    f'{candidate_number}/{candidate_count}.'
                )
                self._set_action_text(message)
                return False, False, message
            finalized, picked_message = self._finalize_pick_di_session_after_stop_settle(
                picked_message
            )
            if not finalized:
                self._clear_pick_di_session()
                message = (
                    f'Pick DI trigger was not finalized after post-Stop settle for candidate '
                    f'{candidate_number}/{candidate_count}; {picked_message}'
                )
                self._set_action_text(message)
                return False, False, message
            _, picked_message = self._finish_pick_di_session()
            message = (
                f'DI held latched after post-Stop settle during pick descent for candidate '
                f'{candidate_number}/{candidate_count}; '
                f'Stop {"accepted" if di_stop_ok else "failed"} ({di_stop_message}); '
                f'context={di_stop_context}; '
                f'{picked_message}'
            )
            state_lock = getattr(self, '_lock', None)
            if state_lock is None:
                self._item_di_confirmed_for_current_track = True
                self._item_di_confirmed_generation = int(
                    getattr(self, '_item_di_confirmed_generation', 0) or 0
                ) + 1
                di_confirmed_generation = int(self._item_di_confirmed_generation)
            else:
                with state_lock:
                    self._item_di_confirmed_for_current_track = True
                    self._item_di_confirmed_generation = int(
                        getattr(self, '_item_di_confirmed_generation', 0) or 0
                    ) + 1
                    di_confirmed_generation = int(self._item_di_confirmed_generation)
            self.get_logger().info(
                'DATALOG item_pick_di_confirmed: '
                f'success=1 candidate={candidate_number}/{candidate_count} '
                f'generation={di_confirmed_generation} '
                f'context="{di_stop_context}" di="{picked_message}"'
            )
            self.get_logger().info(
                'DATALOG item_pick_descent_di_trigger: '
                f'candidate={candidate_number}/{candidate_count} '
                f'stop_ok={int(di_stop_ok)} stop="{di_stop_message}" '
                f'context="{di_stop_context}" '
                f'poll_sec={PICK_DI_POLL_SEC:.4f} '
                f'settle_sec={PICK_DI_STOP_SETTLE_SEC:.2f} '
                f'di="{picked_message}"'
            )
            self._set_action_text(message)
            return True, True, message

        def _finish_without_di(reason: str, picked_message: str) -> tuple[bool, bool, str]:
            picked, finish_message = self._finish_pick_di_session()
            if picked:
                return _finish_picked_after_stop(finish_message)
            detail_message = finish_message or picked_message
            message = (
                f'no DI trigger for candidate {candidate_number}/{candidate_count}; '
                f'{reason}; without held DI latch; {detail_message}'
            )
            distance_text = (
                f'{bottom_leeway_distance_mm:.2f}'
                if bottom_leeway_distance_mm is not None
                else 'unknown'
            )
            self.get_logger().info(
                'DATALOG item_pick_descent_reached_without_di: '
                f'candidate={candidate_number}/{candidate_count} '
                f'distance_mm={distance_text} '
                f'poll_sec={PICK_DI_POLL_SEC:.4f} '
                f'bottom_leeway_sec={PICK_DI_BOTTOM_LEEWAY_SEC:.2f} '
                f'di_stop_sent={int(di_stop_sent)} '
                f'di="{detail_message}"'
            )
            self._set_action_text(message)
            return True, False, message

        timeout = max(0.1, float(timeout_sec))
        tolerance = max(0.1, float(TCP_GOAL_REACHED_TOLERANCE_MM))
        deadline = time.monotonic() + timeout
        last_distance_text = 'unknown'
        self._set_action_text(
            f'Monitoring pick descent candidate {candidate_number}/{candidate_count} '
            f'for DI trigger or pick pose reach...'
        )
        self.get_logger().info(
            'DATALOG item_pick_descent_monitor_start: '
            f'candidate={candidate_number}/{candidate_count} '
            f'timeout_sec={timeout:.1f} tolerance_mm={tolerance:.1f}'
        )

        while rclpy.ok():
            if self._is_cancel_requested():
                stop_ok, stop_message = self._send_stop_command('Stop() [pick descent cancel]')
                self._clear_pick_di_session()
                message = (
                    f'Sequence cancelled during monitored pick descent; '
                    f'Stop {"accepted" if stop_ok else "failed"} ({stop_message}).'
                )
                self.get_logger().warn(
                    'DATALOG item_pick_descent_safety_stop: '
                    f'reason="cancel" candidate={candidate_number}/{candidate_count} '
                    f'stop_ok={int(stop_ok)} stop="{stop_message}"'
                )
                self._set_action_text(message)
                return False, False, message

            trigger_now, picked_message = self._poll_pick_di_trigger_session()
            if trigger_now:
                return _finish_picked_after_stop(picked_message)

            distance_mm, snapshot = self._tcp_xyz_distance_mm(pick_goal[:3])
            if distance_mm is not None:
                last_distance_text = f'{distance_mm:.2f}mm'
                tcp_fresh = True
                if snapshot.tcp_stamp is not None:
                    tcp_fresh = (time.time() - float(snapshot.tcp_stamp)) <= ROBOT_TCP_STALE_SEC
                if tcp_fresh and distance_mm <= tolerance:
                    now = time.monotonic()
                    trigger_now, picked_message = self._poll_pick_di_trigger_session()
                    if trigger_now:
                        return _finish_picked_after_stop(picked_message)

                    if bottom_leeway_deadline is None:
                        bottom_leeway_deadline = now + PICK_DI_BOTTOM_LEEWAY_SEC
                        bottom_leeway_distance_mm = distance_mm
                        self.get_logger().info(
                            'DATALOG item_pick_descent_bottom_leeway_start: '
                            f'candidate={candidate_number}/{candidate_count} '
                            f'distance_mm={distance_mm:.2f} '
                            f'leeway_sec={PICK_DI_BOTTOM_LEEWAY_SEC:.2f} '
                            f'poll_sec={PICK_DI_POLL_SEC:.4f} '
                            f'di="{picked_message}"'
                        )
                        self._set_action_text(
                            f'candidate {candidate_number}/{candidate_count}: '
                            f'reached pick pose; waiting {PICK_DI_BOTTOM_LEEWAY_SEC:.2f}s '
                            'for late DI latch...'
                        )
                    elif now >= bottom_leeway_deadline:
                        return _finish_without_di(
                            f'reached pick pose within {distance_mm:.2f} mm '
                            f'(tol={tolerance:.1f} mm) and bottom DI leeway expired',
                            picked_message,
                        )
                    else:
                        self._set_action_text(
                            f'candidate {candidate_number}/{candidate_count}: '
                            'at pick pose; waiting for late DI latch '
                            f'({max(0.0, bottom_leeway_deadline - now):.2f}s left)...'
                        )

            if (
                bottom_leeway_deadline is not None
                and not di_stop_sent
                and time.monotonic() >= bottom_leeway_deadline
            ):
                trigger_now, picked_message = self._poll_pick_di_trigger_session()
                if trigger_now:
                    return _finish_picked_after_stop(picked_message)
                return _finish_without_di(
                    f'bottom DI leeway expired after {PICK_DI_BOTTOM_LEEWAY_SEC:.2f}s',
                    picked_message,
                )

            if not di_stop_sent and bottom_leeway_deadline is None:
                if time.monotonic() >= deadline:
                    stop_ok, stop_message = self._send_stop_command('Stop() [pick descent timeout]')
                    self._clear_pick_di_session()
                    message = (
                        f'Pick descent timeout for candidate {candidate_number}/{candidate_count}; '
                        f'last_distance={last_distance_text}; '
                        f'Stop {"accepted" if stop_ok else "failed"} ({stop_message}).'
                    )
                    self.get_logger().warn(
                        'DATALOG item_pick_descent_safety_stop: '
                        f'reason="timeout" candidate={candidate_number}/{candidate_count} '
                        f'last_distance="{last_distance_text}" stop_ok={int(stop_ok)} '
                        f'stop="{stop_message}"'
                    )
                    self._set_action_text(message)
                    return False, False, message

            time.sleep(PICK_DI_POLL_SEC)

        self._clear_pick_di_session()
        message = 'ROS shutdown while monitoring pick descent.'
        self._set_action_text(message)
        return False, False, message

    def _wait_for_tcp_stable_after_pick_home(self) -> tuple[bool, str]:
        stability_sec = max(0.0, float(FAST_SETTLE_SEC))
        deadline = time.monotonic() + max(0.1, float(ITEM_GO_TO_TEACH_STABILITY_TIMEOUT_SEC))
        stable_anchor_pose: tuple[float, float, float, float, float, float] | None = None
        stable_since: float | None = None
        stable_elapsed = 0.0
        last_stamp: float | None = None
        last_linear_delta = 0.0
        last_rot_delta = 0.0

        while rclpy.ok():
            if self._is_cancel_requested():
                return False, 'Sequence cancelled while waiting for fixed-home stability after pick'

            now = time.monotonic()
            snapshot = self.snapshot()
            if snapshot.tcp_stamp is not None and (time.time() - float(snapshot.tcp_stamp)) <= ROBOT_TCP_STALE_SEC:
                pose = (
                    float(snapshot.tcp_values.get('x', 0.0)),
                    float(snapshot.tcp_values.get('y', 0.0)),
                    float(snapshot.tcp_values.get('z', 0.0)),
                    float(snapshot.tcp_values.get('rx', 0.0)),
                    float(snapshot.tcp_values.get('ry', 0.0)),
                    float(snapshot.tcp_values.get('rz', 0.0)),
                )
                if stable_anchor_pose is None:
                    stable_anchor_pose = pose
                    stable_since = now
                    last_stamp = snapshot.tcp_stamp
                elif snapshot.tcp_stamp != last_stamp:
                    linear_delta, rot_delta = self._tcp_pose_delta(pose, stable_anchor_pose)
                    last_linear_delta = linear_delta
                    last_rot_delta = rot_delta
                    if linear_delta > ROBOT_LINEAR_MOVE_EPS_MM or rot_delta > ROBOT_ROT_MOVE_EPS_DEG:
                        stable_anchor_pose = pose
                        stable_since = now
                        last_linear_delta = 0.0
                        last_rot_delta = 0.0
                    last_stamp = snapshot.tcp_stamp

                if stable_since is not None:
                    stable_elapsed = max(0.0, now - stable_since)
                if stable_since is not None and stable_elapsed >= stability_sec:
                    return (
                        True,
                        'Robot TCP stable for '
                        f'{stable_elapsed:.2f}s after queued fixed home '
                        f'(window delta {last_linear_delta:.2f}mm, {last_rot_delta:.2f}deg)',
                    )

            if now >= deadline:
                if snapshot.tcp_stamp is None:
                    return False, 'No TCP feedback received after queued fixed home'
                tcp_age = time.time() - float(snapshot.tcp_stamp)
                if tcp_age > ROBOT_TCP_STALE_SEC:
                    return False, (
                        'TCP feedback stale after queued fixed home: '
                        f'last update {tcp_age:.2f}s ago'
                    )
                return False, (
                    'Robot did not become stable after queued fixed home '
                    f'(stable time {stable_elapsed:.2f}/{stability_sec:.2f}s, '
                    f'window delta {last_linear_delta:.2f}mm, {last_rot_delta:.2f}deg)'
                )
            time.sleep(0.05)
        return False, 'ROS shutdown while monitoring fixed-home stability after pick'

    def _wait_for_pick_attempt_completion(
        self,
        target_command_id: int | None,
        *,
        candidate_number: int,
        candidate_count: int,
        monitor_di: bool = True,
    ) -> tuple[bool, bool, str]:
        """Return on picked DI; otherwise prove fixed-home before retry/fail."""
        def _picked_message(picked_message: str) -> str:
            return (
                f'pick confirmed by DI; ready for tray '
                f'(candidate {candidate_number}/{candidate_count}; {picked_message}; '
                'fixed-home return remains queued)'
            )

        if target_command_id is None:
            self._clear_pick_di_session()
            message = (
                f'Pick failed: no controller command id returned for fixed-home MovJ '
                f'after candidate {candidate_number}/{candidate_count}; cannot prove queued motion completion.'
            )
            self.get_logger().error(message)
            self._set_action_text(message)
            return False, False, message

        if monitor_di:
            picked_now, picked_message = self._poll_pick_di_session()
            if picked_now:
                _, picked_message = self._finish_pick_di_session()
                message = _picked_message(picked_message)
                self._set_action_text(message)
                return True, True, message
        if GetCurrentCommandId is None or self._get_current_command_id_client is None:
            self._clear_pick_di_session()
            message = (
                f'Pick failed: GetCurrentCommandId service type is unavailable after '
                f'candidate {candidate_number}/{candidate_count}; cannot prove queued motion completion.'
            )
            self.get_logger().error(message)
            self._set_action_text(message)
            return False, False, message

        timeout = max(0.1, float(PICK_COMMAND_ID_WAIT_TIMEOUT_SEC))
        deadline = time.monotonic() + timeout
        last_command_id: int | None = None
        highest_command_id_seen: int | None = None
        command_id_reached_message = ''
        wait_message = (
            f'Waiting for candidate {candidate_number}/{candidate_count} fixed-home command id '
            f'{target_command_id} while monitoring held-item DI...'
        )
        if not monitor_di:
            wait_message = (
                f'Waiting for candidate {candidate_number}/{candidate_count} fixed-home command id '
                f'{target_command_id} after no-DI pick pose reach...'
            )
        self._set_action_text(wait_message)
        while rclpy.ok():
            if self._is_cancel_requested():
                self._clear_pick_di_session()
                message = 'Sequence cancelled while waiting for queued pick fixed home.'
                self._set_action_text(message)
                return False, False, message

            if monitor_di:
                picked_now, picked_message = self._poll_pick_di_session()
                if picked_now:
                    _, picked_message = self._finish_pick_di_session()
                    message = _picked_message(picked_message)
                    self._set_action_text(message)
                    return True, True, message
            current_command_id = self._read_current_controller_command_id()
            if current_command_id is not None:
                last_command_id = current_command_id
                if highest_command_id_seen is None or current_command_id > highest_command_id_seen:
                    highest_command_id_seen = current_command_id
                reached, reached_message = self._controller_command_id_reached(
                    current_command_id,
                    int(target_command_id),
                    highest_command_id_seen,
                )
                if reached:
                    command_id_reached_message = reached_message
                    break
            if time.monotonic() >= deadline:
                self._clear_pick_di_session()
                message = (
                    f'Pick failed: timeout waiting for fixed-home command id {target_command_id} '
                    f'after candidate {candidate_number}/{candidate_count} (last={last_command_id}).'
                )
                self.get_logger().error(message)
                self._set_action_text(message)
                return False, False, message
            time.sleep(PICK_COMMAND_ID_POLL_SEC)

        if not rclpy.ok():
            self._clear_pick_di_session()
            message = 'Pick failed: ROS shutdown while waiting for queued fixed-home command id.'
            self._set_action_text(message)
            return False, False, message
        if command_id_reached_message:
            self._set_action_text(
                f'Candidate {candidate_number}/{candidate_count} fixed-home queue drained: '
                f'{command_id_reached_message}.'
            )

        if monitor_di:
            picked_now, picked_message = self._poll_pick_di_session()
            if picked_now:
                _, picked_message = self._finish_pick_di_session()
                message = _picked_message(picked_message)
                self._set_action_text(message)
                return True, True, message

        stable, stable_message = self._wait_for_tcp_stable_after_pick_home()
        if not stable:
            message = f'Pick failed: fixed-home command reached but robot did not settle. {stable_message}'
            self.get_logger().error(message)
            self._set_action_text(message)
            return False, False, message

        baseline_seq = getattr(
            self,
            '_last_fixed_home_joint_feedback_seq_baseline',
            None,
        )
        home_verified, home_message = self._wait_for_fresh_fixed_home_joint_feedback(
            newer_than_seq=baseline_seq,
        )
        if not home_verified:
            message = (
                'Pick failed: fixed-home command reached and TCP settled, but joint target '
                f'is unproven. {home_message}'
            )
            self.get_logger().error(message)
            self._set_action_text(message)
            return False, False, message

        if monitor_di:
            self._poll_pick_di_session()
            picked, picked_message = self._finish_pick_di_session()
            if picked:
                message = _picked_message(
                    f'{picked_message}; {stable_message}; {home_message}'
                )
                self._set_action_text(message)
                return True, True, message
        else:
            picked_message = 'late DI ignored after pick pose reached without trigger'

        message = (
            f'no DI trigger for candidate {candidate_number}/{candidate_count}; '
            f'{picked_message}; {stable_message}; {home_message}'
        )
        self._set_action_text(message)
        return True, False, message

    def _check_inverse_kin_goal(
        self,
        goal: tuple[float, float, float, float, float, float],
        label: str,
    ) -> IkCheckResult:
        if not self._wait_for_service(
            self._inverse_kin_client,
            'InverseKin',
            timeout_sec=INVERSE_KIN_SERVICE_TIMEOUT_SEC,
        ):
            self._warn_inverse_kin_unavailable_once(
                'service_unavailable',
                'InverseKin service unavailable for reachability checks; '
                'soft-limit validation remains active.'
            )
            return IkCheckResult.UNAVAILABLE

        request = InverseKin.Request()
        request.x = float(goal[0])
        request.y = float(goal[1])
        request.z = float(goal[2])
        request.rx = float(goal[3])
        request.ry = float(goal[4])
        request.rz = float(goal[5])
        request.use_joint_near = '0'
        # The bridge always serialises jointNear= into the TCP command string even
        # when useJointNear=0. An empty string produces "jointNear=" which the
        # controller rejects with -20000 (parameter number error), and the Dobot
        # parser requires the six-joint vector itself to be brace-wrapped. The
        # controller ignores the values because useJointNear=0.
        request.joint_near = '{0,0,0,0,0,0}'
        request.user = '0'
        request.tool = '1'

        response = self._call_service(
            self._inverse_kin_client,
            request,
            (
                f'InverseKin {label}('
                f'{request.x:.1f},{request.y:.1f},{request.z:.1f},'
                f'{request.rx:.2f},{request.ry:.2f},{request.rz:.2f})'
            ),
            log_rejections=False,
        )
        if response is None:
            self._warn_inverse_kin_unavailable_once(
                'call_failed',
                'InverseKin call timed out or failed; soft-limit validation remains active.'
            )
            return IkCheckResult.UNAVAILABLE
        res = int(getattr(response, 'res', -1))
        # Dobot error codes <= -10000 are protocol/format errors
        # (-10000 unknown command, -20000 param count, -30xxx/-40xxx/-50xxx/-60xxx
        # param type/range). These indicate a bad request, not an unreachable pose.
        if res <= -10000:
            self._warn_inverse_kin_unavailable_once(
                f'protocol_{res}',
                f'InverseKin returned protocol error {res}; treating IK as unavailable. '
                'Soft-limit validation remains active.'
            )
            return IkCheckResult.UNAVAILABLE
        if res < 0:
            return IkCheckResult.UNREACHABLE
        # The older working path uses the controller result code only. The
        # returned payload is not required to contain a parseable joint vector.
        return IkCheckResult.REACHABLE

    def _warn_inverse_kin_unavailable_once(self, key: str, message: str) -> None:
        warning_keys = getattr(self, '_inverse_kin_unavailable_warning_keys', None)
        if warning_keys is None:
            warning_keys = set()
            self._inverse_kin_unavailable_warning_keys = warning_keys
        if key in warning_keys:
            return
        warning_keys.add(key)
        self.get_logger().warn(f'{message} Further identical IK-unavailable warnings are suppressed.')

    def _rearm_item_pose_watch_after_rejected_pose(self, reason: str) -> int | None:
        with self._lock:
            if self._cancel_requested or bool(
                getattr(self, '_lifecycle_stop_latched', False)
            ):
                self._snapshot.action_text = 'Sequence cancelled after IK pose rejection.'
                return None
            self._camera_bin_pose_reject_count += 1
            reject_count = int(self._camera_bin_pose_reject_count)
            max_attempts = max(1, int(self._camera_bin_valid_pose_max_attempts))
            if reject_count >= max_attempts:
                message = (
                    f'No valid item pose after {reject_count}/{max_attempts} '
                    f'IK/soft-limit checks. {reason}'
                )
                self.get_logger().error(message)
                self._reset_runtime_state_locked(message)
                return None
            remaining_sec = self._item_pose_watch_deadline_monotonic - time.monotonic()
            if remaining_sec <= 0.0:
                self._reset_runtime_state_locked(
                    f'IK rejected pose and seek timeout expired. {reason}'
                )
                return None
            self._item_pose_watch_generation += 1
            self._item_pose_watch_armed = True
            self._item_pose_watch_seq_floor = self._item_pose_seq
            self._item_pose_watch_stop_dispatched = False
            self._item_pose_skip_warned = False
            self._snapshot.action_text = (
                f'Pose validation rejected detection {reject_count}/{max_attempts}; '
                f'waiting for next "{ITEM_POSE_TOPIC}" '
                f'({remaining_sec:.1f}s left). {reason}'
            )
            return self._item_pose_watch_generation

    def _rearm_item_pose_watch_after_camera_bin_reject(self, reason: str) -> int | None:
        with self._lock:
            if self._cancel_requested or bool(
                getattr(self, '_lifecycle_stop_latched', False)
            ):
                self._snapshot.action_text = 'Sequence cancelled after camera-bin pose rejection.'
                return None

            self._camera_bin_pose_reject_count += 1
            reject_count = int(self._camera_bin_pose_reject_count)
            max_attempts = max(1, int(self._camera_bin_valid_pose_max_attempts))
            if reject_count >= max_attempts:
                message = (
                    f'No valid item pose after {reject_count}/{max_attempts} '
                    f'camera-bin ROI checks. {reason}'
                )
                self.get_logger().error(message)
                self._reset_runtime_state_locked(message)
                return None

            remaining_sec = self._item_pose_watch_deadline_monotonic - time.monotonic()
            if remaining_sec <= 0.0:
                message = (
                    f'Camera-bin rejected pose and seek timeout expired after '
                    f'{reject_count}/{max_attempts} ROI checks. {reason}'
                )
                self.get_logger().error(message)
                self._reset_runtime_state_locked(message)
                return None

            self._item_pose_watch_generation += 1
            self._item_pose_watch_armed = True
            self._item_pose_watch_seq_floor = self._item_pose_seq
            self._item_pose_watch_stop_dispatched = False
            self._item_pose_skip_warned = False
            self._snapshot.action_text = (
                f'Camera-bin rejected pose {reject_count}/{max_attempts}; '
                f'waiting for next "{ITEM_POSE_TOPIC}" '
                f'({remaining_sec:.1f}s left). {reason}'
            )
            return self._item_pose_watch_generation

    def _rearm_item_pose_watch_after_pick_di_failure(self, reason: str) -> int | None:
        with self._lock:
            if self._cancel_requested or bool(
                getattr(self, '_lifecycle_stop_latched', False)
            ):
                self._snapshot.action_text = 'Sequence cancelled after pick DI failure.'
                return None

            remaining_sec = self._item_pose_watch_deadline_monotonic - time.monotonic()
            if remaining_sec <= 0.0:
                message = f'Pick failed: all candidates missed and seek timeout expired. {reason}'
                self.get_logger().error(message)
                self._reset_runtime_state_locked(message)
                return None

            self._item_pose_watch_generation += 1
            self._item_pose_watch_armed = True
            self._item_pose_watch_seq_floor = self._item_pose_seq
            self._item_pose_watch_stop_dispatched = False
            self._item_pose_skip_warned = False
            self._snapshot.action_text = (
                f'all candidates failed; requesting new item seek from home '
                f'({remaining_sec:.1f}s left). {reason}'
            )
            return self._item_pose_watch_generation

    def _call_trigger_client_once(
        self,
        client,
        service_name: str,
        label: str,
        timeout_sec: float,
        *,
        wait_for_service_sec: float = 0.2,
    ) -> tuple[bool, bool, str]:
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=max(0.0, float(wait_for_service_sec))):
                return False, False, f'{label} service not ready: {service_name}'

        try:
            future = client.call_async(Trigger.Request())
        except Exception as exc:
            return False, False, f'Failed to call {service_name}: {exc}'

        started = time.monotonic()
        deadline = started + max(0.1, float(timeout_sec))
        while rclpy.ok() and not future.done():
            if self._is_cancel_requested():
                return False, False, f'{label} cancelled while waiting for {service_name}'
            if time.monotonic() >= deadline:
                return False, False, f'Timed out waiting for {service_name} response'
            time.sleep(0.02)

        exception = future.exception()
        if exception is not None:
            return False, False, f'{service_name} response failed: {exception}'
        response = future.result()
        if response is None:
            return False, False, f'{service_name} returned no response'
        return True, bool(getattr(response, 'success', False)), str(getattr(response, 'message', '') or '')

    def _wait_item_go_to_teach_idle_before_repick(self) -> bool:
        deadline = time.monotonic() + max(0.1, float(ITEM_GO_TO_TEACH_STATUS_TIMEOUT_SEC))
        last_message = ''
        while rclpy.ok():
            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled while moving to item-detect teach for repick.')
                return False
            available, active, message = self._call_trigger_client_once(
                self._item_go_to_teach_status_client,
                self._item_go_to_teach_status_service_name,
                'Item detect go-to-teach status',
                timeout_sec=0.5,
            )
            last_message = message
            if not available:
                self.get_logger().warn(message)
                self._set_action_text(message)
                return False
            if not active:
                text = message.lower()
                if any(marker in text for marker in ('fail', 'error', 'not ready')):
                    self.get_logger().warn(
                        'Item detect go-to-teach did not finish cleanly before repick: '
                        f'{message}'
                    )
                    self._set_action_text(
                        'Camera-bin rejected pose, but item detect go-to-teach failed before repick: '
                        f'{message}'
                    )
                    return False
                return True
            if time.monotonic() >= deadline:
                timeout_message = (
                    'Timed out waiting for item detect go-to-teach to finish before repick: '
                    f'{last_message or self._item_go_to_teach_status_service_name}'
                )
                self.get_logger().warn(timeout_message)
                self._set_action_text(timeout_message)
                return False
            time.sleep(0.1)
        self._set_action_text('ROS shutdown while waiting for item detect go-to-teach before repick.')
        return False

    @staticmethod
    def _angle_delta_deg(lhs: float, rhs: float) -> float:
        return abs((float(lhs) - float(rhs) + 180.0) % 360.0 - 180.0)

    @staticmethod
    def _tcp_pose_delta(
        lhs: tuple[float, float, float, float, float, float],
        rhs: tuple[float, float, float, float, float, float],
    ) -> tuple[float, float]:
        linear_delta = math.sqrt(
            ((lhs[0] - rhs[0]) ** 2)
            + ((lhs[1] - rhs[1]) ** 2)
            + ((lhs[2] - rhs[2]) ** 2)
        )
        rot_delta = max(
            ItemPickNode._angle_delta_deg(lhs[3], rhs[3]),
            ItemPickNode._angle_delta_deg(lhs[4], rhs[4]),
            ItemPickNode._angle_delta_deg(lhs[5], rhs[5]),
        )
        return linear_delta, rot_delta

    def _wait_for_tcp_stable_before_repick(self) -> tuple[bool, str]:
        stability_sec = max(0.0, float(ITEM_GO_TO_TEACH_STABILITY_SEC))
        deadline = time.monotonic() + max(0.1, float(ITEM_GO_TO_TEACH_STABILITY_TIMEOUT_SEC))
        stable_anchor_pose: tuple[float, float, float, float, float, float] | None = None
        stable_since: float | None = None
        stable_elapsed = 0.0
        last_stamp: float | None = None
        last_linear_delta = 0.0
        last_rot_delta = 0.0

        while rclpy.ok():
            if self._is_cancel_requested():
                return False, 'Sequence cancelled while waiting for robot stability before repick'

            now = time.monotonic()
            snapshot = self.snapshot()
            if snapshot.tcp_stamp is not None and (time.time() - float(snapshot.tcp_stamp)) <= ROBOT_TCP_STALE_SEC:
                pose = (
                    float(snapshot.tcp_values.get('x', 0.0)),
                    float(snapshot.tcp_values.get('y', 0.0)),
                    float(snapshot.tcp_values.get('z', 0.0)),
                    float(snapshot.tcp_values.get('rx', 0.0)),
                    float(snapshot.tcp_values.get('ry', 0.0)),
                    float(snapshot.tcp_values.get('rz', 0.0)),
                )
                if stable_anchor_pose is None:
                    stable_anchor_pose = pose
                    stable_since = now
                    last_stamp = snapshot.tcp_stamp
                elif snapshot.tcp_stamp != last_stamp:
                    linear_delta, rot_delta = self._tcp_pose_delta(pose, stable_anchor_pose)
                    last_linear_delta = linear_delta
                    last_rot_delta = rot_delta
                    if linear_delta > ROBOT_LINEAR_MOVE_EPS_MM or rot_delta > ROBOT_ROT_MOVE_EPS_DEG:
                        stable_anchor_pose = pose
                        stable_since = now
                        last_linear_delta = 0.0
                        last_rot_delta = 0.0
                    last_stamp = snapshot.tcp_stamp

                if stable_since is not None:
                    stable_elapsed = max(0.0, now - stable_since)
                if stable_since is not None and stable_elapsed >= stability_sec:
                    return (
                        True,
                        'Robot TCP stable for '
                        f'{stable_elapsed:.2f}s before item detect repick '
                        f'(window delta {last_linear_delta:.2f}mm, {last_rot_delta:.2f}deg)',
                    )

            if now >= deadline:
                if snapshot.tcp_stamp is None:
                    return False, 'No TCP feedback received before item detect repick'
                tcp_age = time.time() - float(snapshot.tcp_stamp)
                if tcp_age > ROBOT_TCP_STALE_SEC:
                    return False, (
                        'TCP feedback stale before item detect repick: '
                        f'last update {tcp_age:.2f}s ago'
                    )
                return False, (
                    'Robot did not become stable before item detect repick '
                    f'(stable time {stable_elapsed:.2f}/{stability_sec:.2f}s, '
                    f'window delta {last_linear_delta:.2f}mm, {last_rot_delta:.2f}deg)'
                )
            time.sleep(0.05)
        return False, 'ROS shutdown while monitoring robot stability before item detect repick'

    def _move_robot_to_fixed_home_before_repick(
        self,
        reason: str,
        *,
        context: str = 'Camera-bin rejected pose recovery',
        home_label: str = 'MovJ fixed home before camera-bin repick',
    ) -> bool:
        """Return to the canonical fixed home and prove it before reacquiring."""
        already_home, already_home_message = self._fixed_home_joint_feedback_status()
        if already_home:
            self.get_logger().info(
                f'{context}: robot already outside the bin at verified fixed home; '
                f'{already_home_message}'
            )
            return True

        self._set_action_text(
            f'{context}; returning robot to fixed home before requesting a fresh pose...'
        )
        home_ok, _home_v, _home_map = self._send_fixed_home_movj_goal(
            home_label,
        )
        if not home_ok:
            message = (
                f'{context}: fixed-home MovJ was rejected; not requesting item-detect repick. '
                f'{reason}'
            )
            self.get_logger().error(message)
            self._set_action_text(message)
            return False
        if self._stop_if_cancelled_after_motion_dispatch('camera-bin repick fixed-home dispatch'):
            return False

        home_command_id = self._last_fixed_home_movj_command_id
        completed, completion_message = self._wait_for_queued_fixed_home_completion(
            home_command_id,
            context_label=context,
        )
        if not completed:
            message = (
                f'{context}: fixed home could not be proven; not requesting item-detect repick. '
                f'{completion_message}'
            )
            self.get_logger().error(message)
            self._set_action_text(message)
            return False
        self.get_logger().info(
            f'{context}: robot cleared the fixed-camera view at verified fixed home; '
            f'{completion_message}'
        )
        return True

    @staticmethod
    def _item_redetection_phase_from_message(message: str) -> int | None:
        text = str(message or '')
        match = re.search(
            r'\bdetection_phase\s*[=:]\s*(\d+)\b',
            text,
            re.IGNORECASE,
        )
        if match is None:
            match = re.search(
                r'\bredetection\s+phase\s+(\d+)\b',
                text,
                re.IGNORECASE,
            )
        if match is None:
            return None
        phase = int(match.group(1))
        return phase if phase >= 1 else None

    def _record_item_redetection_phase_from_message(self, message: str) -> int | None:
        phase = self._item_redetection_phase_from_message(message)
        if phase is None:
            return None
        with self._lock:
            previous = int(getattr(self, '_item_redetection_phase', 0) or 0)
            self._item_redetection_phase = max(previous, phase)
            recorded = int(self._item_redetection_phase)
        if recorded > previous:
            self.get_logger().info(
                'DATALOG item_pick_redetection_phase_observed: '
                f'phase={recorded} previous={previous}'
            )
        return recorded

    @function_timing.traced("item_pick_repick")
    def _request_item_detect_repick_after_camera_bin_reject(self, reason: str) -> bool:
        if not self._move_robot_to_fixed_home_before_repick(reason):
            return False

        client = self._item_repick_client
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.2):
                message = (
                    f'Camera-bin rejected pose but item detect repick service is not ready: '
                    f'{self._item_repick_service_name}. Waiting for a fresh pose may time out. {reason}'
                )
                self.get_logger().warn(message)
                self._set_action_text(message)
                return False

        if not self._require_fixed_home_before_item_detect_repick(
            'Camera-bin rejected pose recovery',
        ):
            return False
        try:
            future = client.call_async(Trigger.Request())
        except Exception as exc:
            message = (
                f'Camera-bin rejected pose but failed to call item detect repick '
                f'{self._item_repick_service_name}: {exc}. Waiting for a fresh pose may time out.'
            )
            self.get_logger().warn(message)
            self._set_action_text(message)
            return False

        started = time.monotonic()
        timeout_sec = max(0.1, float(ITEM_REPICK_RESPONSE_TIMEOUT_SEC))
        while rclpy.ok() and not future.done():
            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled during item detect repick request.')
                return False
            if (time.monotonic() - started) >= timeout_sec:
                message = (
                    f'Timed out waiting for item detect repick response: '
                    f'{self._item_repick_service_name}. Waiting for a fresh pose may time out.'
                )
                self.get_logger().warn(message)
                self._set_action_text(message)
                return False
            time.sleep(0.02)

        exception = future.exception()
        if exception is not None:
            message = (
                f'Item detect repick call failed: {exception}. '
                'Waiting for a fresh pose may time out.'
            )
            self.get_logger().warn(message)
            self._set_action_text(message)
            return False

        response = future.result()
        if response is None:
            message = (
                f'Item detect repick returned no response from {self._item_repick_service_name}. '
                'Waiting for a fresh pose may time out.'
            )
            self.get_logger().warn(message)
            self._set_action_text(message)
            return False

        response_message = str(getattr(response, 'message', '') or '').strip()
        if not bool(getattr(response, 'success', False)):
            if 'already acquiring' in response_message.lower():
                self._record_item_redetection_phase_from_message(response_message)
                self.get_logger().info(
                    'Item detect repick already acquiring after camera-bin rejection: '
                    f'{response_message}'
                )
                return True
            message = (
                f'Item detect repick rejected after camera-bin pose rejection: '
                f'{response_message or "no message"}. Waiting for a fresh pose may time out.'
            )
            self.get_logger().warn(message)
            self._set_action_text(message)
            return False

        self._record_item_redetection_phase_from_message(response_message)
        self.get_logger().info(
            'Requested fresh item detect pose after camera-bin rejection: '
            f'{response_message or self._item_repick_service_name}'
        )
        return True

    @function_timing.traced("item_pick_repick")
    def _request_item_detect_repick_from_home(self, reason: str) -> bool:
        # This path is also reached after a mixed same-frame attempt: an early
        # candidate may have moved into the bin and retracted only to its local
        # final-Z pose, while every remaining candidate was rejected before
        # motion.  Merely checking that the arm is home would fail safely but
        # leave it parked in the camera view.  Always *ensure* fixed home first
        # (the helper skips a duplicate MovJ when recent all-joint feedback
        # already proves home), and only then allow the repick RPC to arm fresh
        # fixed-camera inference/snapshot capture.
        if not self._move_robot_to_fixed_home_before_repick(
            reason,
            context='All item-pick candidates failed',
            home_label='MovJ fixed home before exhausted-candidate repick',
        ):
            return False

        client = self._item_repick_client
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.2):
                message = (
                    f'all candidates failed; requesting new item seek from home but item detect repick '
                    f'service is not ready: {self._item_repick_service_name}. {reason}'
                )
                self.get_logger().warn(message)
                self._set_action_text(message)
                return False

        if not self._require_fixed_home_before_item_detect_repick(
            'All item-pick candidates failed',
        ):
            return False
        try:
            future = client.call_async(Trigger.Request())
        except Exception as exc:
            message = (
                f'all candidates failed; requesting new item seek from home but repick call failed: '
                f'{exc}. {reason}'
            )
            self.get_logger().warn(message)
            self._set_action_text(message)
            return False

        started = time.monotonic()
        timeout_sec = max(0.1, float(ITEM_REPICK_RESPONSE_TIMEOUT_SEC))
        while rclpy.ok() and not future.done():
            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled during item detect repick request from home.')
                return False
            if (time.monotonic() - started) >= timeout_sec:
                message = (
                    f'Timed out waiting for item detect repick response from home: '
                    f'{self._item_repick_service_name}. {reason}'
                )
                self.get_logger().warn(message)
                self._set_action_text(message)
                return False
            time.sleep(0.02)

        exception = future.exception()
        if exception is not None:
            message = f'Item detect repick from home failed: {exception}. {reason}'
            self.get_logger().warn(message)
            self._set_action_text(message)
            return False

        response = future.result()
        if response is None:
            message = f'Item detect repick from home returned no response. {reason}'
            self.get_logger().warn(message)
            self._set_action_text(message)
            return False

        response_message = str(getattr(response, 'message', '') or '').strip()
        if not bool(getattr(response, 'success', False)):
            if 'already acquiring' in response_message.lower():
                self._record_item_redetection_phase_from_message(response_message)
                self.get_logger().info(
                    'Item detect repick already acquiring from home after all candidates failed: '
                    f'{response_message}'
                )
                return True
            message = (
                f'Item detect repick from home rejected after all candidates failed: '
                f'{response_message or "no message"}. {reason}'
            )
            self.get_logger().warn(message)
            self._set_action_text(message)
            return False

        self._record_item_redetection_phase_from_message(response_message)
        self.get_logger().info(
            'all candidates failed; requesting new item seek from home: '
            f'{response_message or self._item_repick_service_name}'
        )
        return True

    def _read_item_detect_seek_status_for_repick(
        self,
    ) -> tuple[bool, bool, str]:
        client = getattr(self, '_item_seek_status_client', None)
        service_name = str(
            getattr(
                self,
                '_item_seek_status_service_name',
                ITEM_SEEK_STATUS_SERVICE_DEFAULT,
            )
        )
        if client is None:
            return False, False, f'Item detect seek-status client unavailable: {service_name}'
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.1):
                return False, False, f'Item detect seek-status service not ready: {service_name}'

        try:
            future = client.call_async(Trigger.Request())
        except Exception as exc:
            return False, False, f'Failed to call item detect seek-status {service_name}: {exc}'

        started = time.monotonic()
        timeout_sec = max(0.1, float(ITEM_REPICK_STATUS_RESPONSE_TIMEOUT_SEC))
        while rclpy.ok() and not future.done():
            if self._is_cancel_requested():
                return False, False, 'Sequence cancelled while polling item detect seek status.'
            if (time.monotonic() - started) >= timeout_sec:
                return False, False, f'Timed out waiting for item detect seek-status response: {service_name}'
            time.sleep(0.01)

        if not future.done():
            return False, False, f'ROS shutdown while waiting for item detect seek-status response: {service_name}'

        exception = future.exception()
        if exception is not None:
            return False, False, f'Item detect seek-status call failed: {exception}'

        response = future.result()
        if response is None:
            return False, False, 'Item detect seek-status returned no response'

        return (
            True,
            bool(getattr(response, 'success', False)),
            str(getattr(response, 'message', '') or '').strip(),
        )

    def _reset_item_pose_watch_after_repick_failure(
        self,
        generation: int,
        message: str,
    ) -> bool:
        did_reset = False
        with self._lock:
            if (
                self._item_pose_watch_generation == int(generation)
                and self._item_pose_watch_armed
            ):
                self._reset_runtime_state_locked(message)
                did_reset = True

        if did_reset:
            self.get_logger().warn(message)
            self._notify_item_detect_seek_complete()
        return did_reset

    def _reset_item_pose_watch_after_repick_start_failure(
        self,
        generation: int,
        reason: str,
    ) -> bool:
        with self._lock:
            action_text = str(self._snapshot.action_text or '').strip()
        if action_text:
            message = action_text
        else:
            message = f'Item detect repick failed to start. {reason}'
        if 'fail' not in message.lower():
            message = f'Item detect repick failed to start: {message}'
        return self._reset_item_pose_watch_after_repick_failure(generation, message)

    def _start_item_repick_seek_status_monitor(
        self,
        generation: int,
        reason: str,
    ) -> None:
        if getattr(self, '_item_seek_status_client', None) is None:
            self.get_logger().warn(
                'Item detect seek-status client unavailable; repick recovery will rely on pose-watch timeout.'
            )
            return
        worker = threading.Thread(
            target=self._item_repick_seek_status_monitor_worker,
            args=(int(generation), str(reason)),
            daemon=True,
        )
        worker.start()

    def _item_repick_seek_status_monitor_worker(
        self,
        generation: int,
        reason: str,
    ) -> None:
        last_log_time = 0.0
        while rclpy.ok():
            with self._lock:
                if self._item_pose_watch_generation != int(generation):
                    return
                if not self._item_pose_watch_armed:
                    return
                remaining_sec = self._item_pose_watch_deadline_monotonic - time.monotonic()
                if remaining_sec <= 0.0:
                    return

            available, active, message = self._read_item_detect_seek_status_for_repick()
            if not available:
                now = time.monotonic()
                if now - last_log_time >= ITEM_REPICK_STATUS_LOG_INTERVAL_SEC:
                    self.get_logger().warn(
                        f'Repick seek-status monitor delayed: {message}'
                    )
                    last_log_time = now
                time.sleep(ITEM_REPICK_STATUS_POLL_SEC)
                continue

            if active:
                self._record_item_redetection_phase_from_message(message)
                now = time.monotonic()
                if now - last_log_time >= ITEM_REPICK_STATUS_LOG_INTERVAL_SEC:
                    self.get_logger().info(
                        f'Repick seek-status monitor active: {message}'
                    )
                    last_log_time = now
                time.sleep(ITEM_REPICK_STATUS_POLL_SEC)
                continue

            failure_message = (
                f'Item detect repick failed before fresh "{ITEM_POSE_TOPIC}" arrived: '
                f'{message or "seek became idle"}. {reason}'
            )
            self._reset_item_pose_watch_after_repick_failure(generation, failure_message)
            return

    @function_timing.traced("item_pick_settle")
    def _wait_settling_time(self, settling_time_sec: float, label: str) -> bool:
        wait_sec = max(0.0, float(settling_time_sec))
        if wait_sec <= 1e-6:
            return True
        self._set_action_text(f'{label}: settling for {wait_sec:.1f}s...')
        deadline = time.monotonic() + wait_sec
        while rclpy.ok():
            if self._is_cancel_requested():
                self._set_action_text('Sequence cancelled during settling wait.')
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            time.sleep(min(0.02, remaining))
        self._set_action_text('ROS shutdown during settling wait.')
        return False

    def _preview_goal_only_request(
        self,
        item_target: ItemPoseTarget,
        x_offset_mm: float,
        y_offset_mm: float,
        z_offset_mm: float,
        use_fingers: bool,
        grab_on_pick: bool,
        relax_fingers_on_pick: bool,
        final_z_up_mm: float,
        pre_pick_settling_time_sec: float,
        pick_settling_time_sec: float,
        tool_offset_x_mm: float,
        tool_offset_y_mm: float,
        tool_offset_z_mm: float,
        tool_offset_rx_deg: float,
        tool_offset_ry_deg: float,
        tool_offset_rz_deg: float,
    ) -> None:
        try:
            self._set_action_text('Computing item goal preview in base frame...')
            base_goal = self._compute_base_goal_from_item_target(
                item_target,
                x_offset_mm,
                y_offset_mm,
                z_offset_mm,
                tool_offset_x_mm,
                tool_offset_y_mm,
                tool_offset_z_mm,
                tool_offset_rx_deg,
                tool_offset_ry_deg,
                tool_offset_rz_deg,
                self._post_stop_movel_speed_mm_s,
                predict_target_motion=False,
            )
            if base_goal is None:
                return

            approach_goal = (
                base_goal.x_mm,
                base_goal.y_mm,
                base_goal.z_mm + PICK_FIXED_APPROACH_Z_UP_MM,
                base_goal.rx_deg,
                base_goal.ry_deg,
                base_goal.rz_deg,
            )
            self._publish_primary_goal_debug_transform(approach_goal)
            self._set_action_text(
                f'Previewed approach goal from {base_goal.source_frame_id}: '
                f'pick pose + Z stand-off ({z_offset_mm:.1f} mm), with Approach Z '
                f'{PICK_FIXED_APPROACH_Z_UP_MM:.1f} mm fixed, '
                f'use_fingers={int(use_fingers)}, grab_on_pick={int(grab_on_pick)}, '
                f'relax_fingers_on_pick={int(relax_fingers_on_pick)}, '
                f'final Z-up ({final_z_up_mm:.1f} mm), '
                f'pre-pick settle {pre_pick_settling_time_sec:.1f}s, '
                f'pick settle {pick_settling_time_sec:.1f}s, '
                f'tool offset=({tool_offset_x_mm:.1f},{tool_offset_y_mm:.1f},{tool_offset_z_mm:.1f},'
                f'{tool_offset_rx_deg:.1f},{tool_offset_ry_deg:.1f},{tool_offset_rz_deg:.1f}). '
                f'TF-only frame="{self._post_stop_movel_goal_debug_frame_id}".'
            )
        except Exception as exc:
            self.get_logger().error(f'Preview goal computation failed: {exc}')
            self._set_action_text(f'Preview goal computation failed: {exc}')
        finally:
            self._set_busy(False)

    def _arm_item_pose_watch_locked(self) -> int:
        self._item_pose_watch_generation += 1
        self._item_pose_watch_armed = True
        self._item_pose_watch_seq_floor = self._item_pose_seq
        self._item_pose_watch_deadline_monotonic = (
            time.monotonic()
            + clamp_item_pose_watch_timeout_sec(self._item_pose_watch_timeout_sec)
        )
        self._item_pose_watch_stop_dispatched = False
        self._camera_bin_pose_reject_count = 0
        self._last_camera_bin_reject_reason = ''
        self.get_logger().info(
            "DATALOG item_pick_pose_watch_armed: "
            f"generation={self._item_pose_watch_generation} "
            f"seq_floor={self._item_pose_watch_seq_floor} "
            f"timeout_sec={float(self._item_pose_watch_timeout_sec):.1f}"
        )
        # Allow the next "first dropped pose" log to fire again so each
        # arming cycle produces at most one diagnostic warning if the
        # camera spam keeps getting rejected.
        self._item_pose_skip_warned = False
        return self._item_pose_watch_generation

    def _item_pose_watchdog_worker(self, generation: int) -> None:
        while rclpy.ok():
            if self.count_publishers(ITEM_POSE_TOPIC) <= 0:
                self._reset_runtime_state(
                    f'No item pose publisher on "{ITEM_POSE_TOPIC}". Node reset.'
                )
                return
            with self._lock:
                if self._item_pose_watch_generation != generation:
                    return
                if not self._item_pose_watch_armed:
                    return
                remaining_sec = self._item_pose_watch_deadline_monotonic - time.monotonic()
                if remaining_sec <= 0.0:
                    watch_timeout_sec = clamp_item_pose_watch_timeout_sec(
                        self._item_pose_watch_timeout_sec
                    )
                    self._reset_runtime_state_locked(
                        f'No item pose within {watch_timeout_sec:.0f}s. Node reset.'
                    )
                    return
            time.sleep(min(0.1, max(0.02, remaining_sec)))

    @function_timing.traced("item_pick_worker")
    def _send_movel_request(
        self,
        item_target: ItemPoseTarget,
        post_speed_mm_s: float,
        x_offset_mm: float,
        y_offset_mm: float,
        z_offset_mm: float,
        use_fingers: bool,
        grab_on_pick: bool,
        relax_fingers_on_pick: bool,
        final_z_up_mm: float,
        pre_pick_settling_time_sec: float,
        pick_settling_time_sec: float,
        tool_offset_x_mm: float,
        tool_offset_y_mm: float,
        tool_offset_z_mm: float,
        tool_offset_rx_deg: float,
        tool_offset_ry_deg: float,
        tool_offset_rz_deg: float,
        candidate_targets: tuple[ItemPoseTarget, ...] | None = None,
        *,
        pick_candidates_override: tuple[ItemPoseTarget, ...] | None = None,
        start_candidate_index: int = 0,
    ) -> None:
        seek_complete_notified = False
        busy_released_after_queue = False
        waiting_for_next_pose_after_reject = False
        with self._lock:
            self._active_pick_workers = int(
                getattr(self, '_active_pick_workers', 0)
            ) + 1
        try:
            requested_two_stage_descent = bool(
                getattr(self, '_pick_two_stage_descent_enabled', False)
            )
            fast_switch_z_up_mm = float(
                getattr(
                    self,
                    '_pick_fast_descent_switch_z_up_mm',
                    PICK_FAST_DESCENT_SWITCH_Z_UP_MM_DEFAULT,
                )
            )
            fast_descent_speed_percent = float(
                getattr(
                    self,
                    '_pick_fast_descent_speed_percent',
                    PICK_FAST_DESCENT_SPEED_PERCENT_DEFAULT,
                )
            )
            two_stage_valid, two_stage_reason = (
                validate_pick_two_stage_descent_config(
                    requested_two_stage_descent,
                    fast_switch_z_up_mm,
                    fast_descent_speed_percent,
                )
            )
            two_stage_descent_enabled = bool(
                requested_two_stage_descent and two_stage_valid
            )
            if requested_two_stage_descent and not two_stage_valid:
                self.get_logger().error(
                    'Two-stage descent was invalidated at the command boundary; '
                    f'using legacy descent: {two_stage_reason}'
                )
            if pick_candidates_override is not None:
                pick_candidates = tuple(pick_candidates_override)
            else:
                pick_candidates = self._merge_item_pose_candidates(
                    item_target,
                    tuple(candidate_targets or ()),
                )
            available_candidate_count = len(pick_candidates)
            if available_candidate_count > PICK_MAX_YOLO_CANDIDATES_PER_FRAME:
                pick_candidates = pick_candidates[:PICK_MAX_YOLO_CANDIDATES_PER_FRAME]
            candidate_count = len(pick_candidates)
            start_candidate_index = max(
                0,
                min(candidate_count, int(start_candidate_index)),
            )
            self.get_logger().info(
                'Pick worker started: target frame_id='
                f'"{item_target.frame_id}" pos_mm={item_target.position_mm} '
                f'rpy_deg={item_target.rpy_deg}; '
                f'ranked_candidates={candidate_count}'
                + (
                    f'/{available_candidate_count} capped'
                    if available_candidate_count > candidate_count else ''
                )
            )
            if available_candidate_count > candidate_count:
                self.get_logger().info(
                    f'Pick worker limiting same-frame ranked candidates to '
                    f'{candidate_count}/{available_candidate_count}; requesting a fresh seek after these fail.'
                )
            if candidate_count <= 0 or start_candidate_index >= candidate_count:
                message = (
                    f'No cached item-pick candidates remaining '
                    f'(start={start_candidate_index}, count={candidate_count}).'
                )
                self.get_logger().warn(message)
                self._set_action_text(message)
                return
            with self._lock:
                self._cached_pick_attempt = CachedPickAttempt(
                    candidates=pick_candidates,
                    next_candidate_index=start_candidate_index,
                    post_speed_mm_s=float(post_speed_mm_s),
                    x_offset_mm=float(x_offset_mm),
                    y_offset_mm=float(y_offset_mm),
                    z_offset_mm=float(z_offset_mm),
                    use_fingers=bool(use_fingers),
                    grab_on_pick=bool(grab_on_pick),
                    relax_fingers_on_pick=bool(relax_fingers_on_pick),
                    final_z_up_mm=float(final_z_up_mm),
                    pre_pick_settling_time_sec=float(pre_pick_settling_time_sec),
                    pick_settling_time_sec=float(pick_settling_time_sec),
                    tool_offset_x_mm=float(tool_offset_x_mm),
                    tool_offset_y_mm=float(tool_offset_y_mm),
                    tool_offset_z_mm=float(tool_offset_z_mm),
                    tool_offset_rx_deg=float(tool_offset_rx_deg),
                    tool_offset_ry_deg=float(tool_offset_ry_deg),
                    tool_offset_rz_deg=float(tool_offset_rz_deg),
                )
            if self._is_cancel_requested():
                self.get_logger().warn(
                    'Pick worker aborted before dispatch: cancel requested.'
                )
                self._set_action_text('Sequence cancelled before dispatch.')
                return
            self._set_action_text('Computing item goal in base frame...')

            if self._is_cancel_requested():
                self.get_logger().warn(
                    'Pick worker aborted before goal computation: cancel requested.'
                )
                self._set_action_text('Sequence cancelled before goal computation.')
                return
            if not self._wait_for_service(self._mov_jio_client, 'MovJIO'):
                self.get_logger().warn(
                    'Pick worker aborted: MovJIO service did not become available '
                    '(check that cr_robot_ros2 is up and healthy).'
                )
                return
            if not self._wait_for_service(self._mov_l_client, 'MovL'):
                self.get_logger().warn(
                    'Pick worker aborted: MovL service did not become available '
                    '(check that cr_robot_ros2 is up and healthy).'
                )
                return

            last_reject_kind = ''
            last_camera_bin_reject_reason = ''
            last_ik_reject_reason = ''
            last_validation_reject_reason = ''
            motion_attempts_used = 0
            last_no_pick_reason = ''
            qa_detection_yaml_path = self._resolve_latest_qa_detection_yaml_path()

            def _record_candidate_motion_failure(
                candidate_target: ItemPoseTarget,
                *,
                reason: str,
                command_label: str,
                v_percent: int = 0,
                status: str = '',
            ) -> None:
                if command_label:
                    self._log_pick_queue_command_rejected(
                        command_label,
                        v_percent,
                        status,
                    )
                failed_area_published = self._publish_failed_pick_area(
                    candidate_target,
                    reason=reason,
                )
                self.get_logger().warn(
                    'DATALOG item_pick_motion_failed_area: '
                    f'reason={reason} command="{command_label}" '
                    f'failed_area_published={int(failed_area_published)}'
                )

            if candidate_count > 1:
                self.get_logger().info(
                    f'Pick worker evaluating {candidate_count} ranked candidates '
                    'from the current detection frame before requesting reacquisition.'
                )

            candidate_index = start_candidate_index
            while candidate_index < candidate_count:
                candidate_target = pick_candidates[candidate_index]
                if self._is_cancel_requested():
                    self._set_action_text('Sequence cancelled during goal computation.')
                    return

                candidate_number = candidate_index + 1
                candidate_base_goal = self._compute_base_goal_from_item_target(
                    candidate_target,
                    x_offset_mm,
                    y_offset_mm,
                    z_offset_mm,
                    tool_offset_x_mm,
                    tool_offset_y_mm,
                    tool_offset_z_mm,
                    tool_offset_rx_deg,
                    tool_offset_ry_deg,
                    tool_offset_rz_deg,
                    post_speed_mm_s,
                )
                if candidate_base_goal is None:
                    camera_bin_reject_reason = str(
                        getattr(self, '_last_camera_bin_reject_reason', '') or ''
                    ).strip()
                    if camera_bin_reject_reason:
                        last_reject_kind = 'camera_bin'
                        last_camera_bin_reject_reason = camera_bin_reject_reason
                        failed_area_published = self._publish_failed_pick_area(
                            candidate_target,
                            reason='pre_motion_camera_bin_reject',
                        )
                        retry_text = (
                            '; trying next ranked candidate without reacquisition'
                            if candidate_number < candidate_count else ''
                        )
                        self.get_logger().warn(
                            f'Pick ranked candidate {candidate_number}/{candidate_count} '
                            f'rejected before motion: {camera_bin_reject_reason}{retry_text} '
                            f'failed_area_published={int(failed_area_published)}'
                        )
                        candidate_index += 1
                        continue

                    last_reject_kind = 'base_goal'
                    retry_text = (
                        '; trying next ranked candidate without reacquisition'
                        if candidate_number < candidate_count else ''
                    )
                    self.get_logger().warn(
                        f'Pick ranked candidate {candidate_number}/{candidate_count} '
                        'failed base-goal computation (see TF lookup warning above)'
                        f'{retry_text}'
                    )
                    candidate_index += 1
                    continue

                candidate_pick_goal = (
                    candidate_base_goal.x_mm,
                    candidate_base_goal.y_mm,
                    candidate_base_goal.z_mm,
                    candidate_base_goal.rx_deg,
                    candidate_base_goal.ry_deg,
                    candidate_base_goal.rz_deg,
                )
                candidate_approach_goal = (
                    candidate_pick_goal[0],
                    candidate_pick_goal[1],
                    candidate_pick_goal[2] + PICK_FIXED_APPROACH_Z_UP_MM,
                    candidate_pick_goal[3],
                    candidate_pick_goal[4],
                    candidate_pick_goal[5],
                )
                candidate_fast_descent_goal = (
                    candidate_pick_goal[0],
                    candidate_pick_goal[1],
                    candidate_pick_goal[2] + fast_switch_z_up_mm,
                    candidate_pick_goal[3],
                    candidate_pick_goal[4],
                    candidate_pick_goal[5],
                )
                candidate_final_z_goal = (
                    candidate_approach_goal[0],
                    candidate_approach_goal[1],
                    candidate_approach_goal[2] + float(final_z_up_mm),
                    candidate_approach_goal[3],
                    candidate_approach_goal[4],
                    candidate_approach_goal[5],
                )
                candidate_di_retract_goal = (
                    candidate_pick_goal[0],
                    candidate_pick_goal[1],
                    candidate_pick_goal[2] + PICK_DI_TRIGGER_RETRACT_Z_UP_MM,
                    candidate_pick_goal[3],
                    candidate_pick_goal[4],
                    candidate_pick_goal[5],
                )
                candidate_di_final_z_goal = (
                    candidate_di_retract_goal[0],
                    candidate_di_retract_goal[1],
                    candidate_di_retract_goal[2] + float(final_z_up_mm),
                    candidate_di_retract_goal[3],
                    candidate_di_retract_goal[4],
                    candidate_di_retract_goal[5],
                )
                candidate_pre_z_up_goal = candidate_final_z_goal

                self._set_action_text('Checking IK reachability before motion...')
                ik_checks = [
                    ('pre_z_up', candidate_pre_z_up_goal),
                    ('approach', candidate_approach_goal),
                    ('pick', candidate_pick_goal),
                    ('di_retract', candidate_di_retract_goal),
                    ('di_final_z_up', candidate_di_final_z_goal),
                    ('final_z_up', candidate_final_z_goal),
                ]
                if two_stage_descent_enabled:
                    ik_checks.insert(
                        2,
                        ('fast_descent_switch', candidate_fast_descent_goal),
                    )
                # Several safety legs intentionally share the exact same 6D
                # pose (for example approach/DI retract and the pre/final
                # Z-up goals).  Ask the controller once per unique pose for
                # this candidate, while keeping the cache strictly local to
                # the candidate evaluation.  UNAVAILABLE is deliberately not
                # cached: a later duplicate remains a fresh opportunity for
                # the service to recover, preserving the soft-limit fallback
                # behaviour when it does not.
                candidate_ik_results: dict[
                    tuple[float, float, float, float, float, float],
                    IkCheckResult,
                ] = {}
                candidate_ik_reject_reason = ''
                for ik_label, ik_goal in ik_checks:
                    if self._is_cancel_requested():
                        self._set_action_text('Sequence cancelled during IK check.')
                        return
                    validation = validate_tcp_pose(
                        ik_goal,
                        label=f'candidate {candidate_number}/{candidate_count} {ik_label}',
                    )
                    if not validation.ok:
                        candidate_ik_reject_reason = validation.message
                        last_validation_reject_reason = validation.message
                        break
                    ik_result = candidate_ik_results.get(ik_goal)
                    if ik_result is None:
                        ik_result = self._check_inverse_kin_goal(ik_goal, ik_label)
                        if ik_result != IkCheckResult.UNAVAILABLE:
                            candidate_ik_results[ik_goal] = ik_result
                    if ik_result == IkCheckResult.REACHABLE:
                        continue
                    if ik_result == IkCheckResult.UNAVAILABLE:
                        continue
                    candidate_ik_reject_reason = (
                        f'IK unreachable for {ik_label} goal '
                        f'({ik_goal[0]:.1f},{ik_goal[1]:.1f},{ik_goal[2]:.1f},'
                        f'{ik_goal[3]:.2f},{ik_goal[4]:.2f},{ik_goal[5]:.2f}).'
                    )
                    break

                if candidate_ik_reject_reason:
                    last_reject_kind = 'ik'
                    last_ik_reject_reason = candidate_ik_reject_reason
                    failed_area_published = self._publish_failed_pick_area(
                        candidate_target,
                        reason='pre_motion_ik_reject',
                    )
                    retry_text = (
                        '; trying next ranked candidate without reacquisition'
                        if candidate_number < candidate_count else ''
                    )
                    self.get_logger().warn(
                        f'Pick ranked candidate {candidate_number}/{candidate_count} '
                        f'rejected by IK: {candidate_ik_reject_reason}{retry_text} '
                        f'failed_area_published={int(failed_area_published)}'
                    )
                    candidate_index += 1
                    continue

                item_target = candidate_target
                base_goal = candidate_base_goal
                pick_goal = candidate_pick_goal
                approach_goal = candidate_approach_goal
                final_z_goal = candidate_final_z_goal
                pre_z_up_goal = candidate_pre_z_up_goal
                snapshot = self.snapshot()
                current_pose = (
                    float(snapshot.tcp_values.get('x', 0.0)),
                    float(snapshot.tcp_values.get('y', 0.0)),
                    float(snapshot.tcp_values.get('z', 0.0)),
                    float(snapshot.tcp_values.get('rx', 0.0)),
                    float(snapshot.tcp_values.get('ry', 0.0)),
                    float(snapshot.tcp_values.get('rz', 0.0)),
                )

                if candidate_index > 0:
                    self.get_logger().info(
                        f'Using ranked candidate {candidate_number}/{candidate_count} '
                        'after rejecting or missing earlier candidates without pose reacquisition.'
                    )
                # Cache the exact resolved base-frame pick goal. Stop/Reset
                # returns a held item to this pose, releases it, then retracts;
                # no camera-frame TF is needed after the arm leaves the bin.
                with self._lock:
                    self._last_item_target = item_target
                    self._last_base_drop_release_goal = replace(base_goal)
                self.get_logger().info(
                    'Pick target/goal resolved: '
                    f'candidate={candidate_number}/{candidate_count} '
                    f'source_frame="{base_goal.source_frame_id}" '
                    f'item_camera_pos_mm=({item_target.position_mm[0]:.3f}, {item_target.position_mm[1]:.3f}, {item_target.position_mm[2]:.3f}) '
                    f'item_camera_rpy_deg=({item_target.rpy_deg[0]:.3f}, {item_target.rpy_deg[1]:.3f}, {item_target.rpy_deg[2]:.3f}) '
                    f'current_tcp=({current_pose[0]:.3f}, {current_pose[1]:.3f}, {current_pose[2]:.3f}, '
                    f'{current_pose[3]:.3f}, {current_pose[4]:.3f}, {current_pose[5]:.3f}) '
                    f'pick_goal=({pick_goal[0]:.3f}, {pick_goal[1]:.3f}, {pick_goal[2]:.3f}, '
                    f'{pick_goal[3]:.3f}, {pick_goal[4]:.3f}, {pick_goal[5]:.3f}) '
                    f'approach_goal=({approach_goal[0]:.3f}, {approach_goal[1]:.3f}, {approach_goal[2]:.3f}, '
                    f'{approach_goal[3]:.3f}, {approach_goal[4]:.3f}, {approach_goal[5]:.3f}) '
                    f'fast_descent_switch_goal=({candidate_fast_descent_goal[0]:.3f}, '
                    f'{candidate_fast_descent_goal[1]:.3f}, {candidate_fast_descent_goal[2]:.3f}, '
                    f'{candidate_fast_descent_goal[3]:.3f}, {candidate_fast_descent_goal[4]:.3f}, '
                    f'{candidate_fast_descent_goal[5]:.3f}) '
                    f'di_retract_goal_nominal=({candidate_di_retract_goal[0]:.3f}, {candidate_di_retract_goal[1]:.3f}, {candidate_di_retract_goal[2]:.3f}, '
                    f'{candidate_di_retract_goal[3]:.3f}, {candidate_di_retract_goal[4]:.3f}, {candidate_di_retract_goal[5]:.3f}) '
                    f'pre_z_up_goal=({pre_z_up_goal[0]:.3f}, {pre_z_up_goal[1]:.3f}, {pre_z_up_goal[2]:.3f}, '
                    f'{pre_z_up_goal[3]:.3f}, {pre_z_up_goal[4]:.3f}, {pre_z_up_goal[5]:.3f}) '
                    f'final_z_goal=({final_z_goal[0]:.3f}, {final_z_goal[1]:.3f}, {final_z_goal[2]:.3f}, '
                    f'{final_z_goal[3]:.3f}, {final_z_goal[4]:.3f}, {final_z_goal[5]:.3f}) '
                    f'di_final_z_goal_nominal=({candidate_di_final_z_goal[0]:.3f}, {candidate_di_final_z_goal[1]:.3f}, {candidate_di_final_z_goal[2]:.3f}, '
                    f'{candidate_di_final_z_goal[3]:.3f}, {candidate_di_final_z_goal[4]:.3f}, {candidate_di_final_z_goal[5]:.3f}) '
                    f'nominal_goal=({base_goal.nominal_x_mm:.3f}, {base_goal.nominal_y_mm:.3f}, {base_goal.nominal_z_mm:.3f}, '
                    f'{base_goal.nominal_rx_deg:.3f}, {base_goal.nominal_ry_deg:.3f}, {base_goal.nominal_rz_deg:.3f}) '
                    f'orientation_choice={base_goal.orientation_choice} '
                    f'tool_offset=({tool_offset_x_mm:.1f}, {tool_offset_y_mm:.1f}, {tool_offset_z_mm:.1f}, '
                    f'{tool_offset_rx_deg:.1f}, {tool_offset_ry_deg:.1f}, {tool_offset_rz_deg:.1f}) '
                    f'camera_safety="{base_goal.camera_safety_message}"'
                )
                self._publish_primary_goal_debug_transform(approach_goal)
                self.get_logger().info(
                    'DATALOG item_pick_two_stage_descent_plan: '
                    f'candidate={candidate_number}/{candidate_count} '
                    f'enabled={int(two_stage_descent_enabled)} '
                    f'approach_z_up_mm={PICK_FIXED_APPROACH_Z_UP_MM:.1f} '
                    f'switch_z_up_mm={fast_switch_z_up_mm:.1f} '
                    f'fast_speed_percent={fast_descent_speed_percent:.0f} '
                    f'final_speed_percent={PICK_DESCENT_SPEED_PERCENT} '
                    'monitor_hz=60.0 stop_on_di=1'
                )

                if self._is_cancel_requested():
                    self._set_action_text('Sequence cancelled before queued motion dispatch.')
                    return

                self._set_action_text('Applying CP100 and SpeedFactor100 before monitored pick motion...')
                if not self._set_cp(PICK_CP_PERCENT, 'before monitored pick motion'):
                    return
                if self._is_cancel_requested():
                    self._set_action_text('Sequence cancelled after CP setup.')
                    return
                if not self._set_speed_factor(
                    PICK_NORMAL_SPEED_FACTOR_PERCENT,
                    'before monitored pick motion',
                ):
                    return
                if self._is_cancel_requested():
                    self._set_action_text('Sequence cancelled after speed-factor setup.')
                    return

                upcoming_physical_attempt = int(
                    getattr(self, '_physical_pick_attempt', 0) or 0
                ) + 1
                upcoming_repick_attempt = max(0, upcoming_physical_attempt - 1)
                attempt_text = (
                    f'physical repick {upcoming_repick_attempt}'
                    if upcoming_repick_attempt > 0
                    else 'physical pick 1'
                )
                self._set_action_text(
                    f'candidate {candidate_number}/{candidate_count}: '
                    f'queueing {attempt_text} pre-Z-up, approach, and monitored descent...'
                )
                relax_fingers_on_pick = bool(relax_fingers_on_pick)
                finger_approach_text = (
                    'fingers relaxed'
                    if relax_fingers_on_pick
                    else 'gripper open'
                )
                pre_z_up_label = (
                    f'MovJIO pre-Z-up from {base_goal.source_frame_id} '
                    f'with {finger_approach_text} and suction off at 50 percent'
                )
                pre_z_up_ok, pre_z_up_v, pre_z_up_map = self._send_movjio_goal(
                    pre_z_up_goal,
                    current_pose,
                    post_speed_mm_s,
                    pre_z_up_label,
                    mdis=self._build_pick_approach_mdis(
                        relax_fingers_on_pick=relax_fingers_on_pick,
                    ),
                    forced_v_percent=100,
                    forced_a_percent=100,
                )
                if not pre_z_up_ok:
                    _record_candidate_motion_failure(
                        candidate_target,
                        reason='pre_z_up_command_rejected',
                        command_label=pre_z_up_label,
                        v_percent=pre_z_up_v,
                        status=pre_z_up_map,
                    )
                    return
                if self._stop_if_cancelled_after_motion_dispatch('pre-Z-up dispatch'):
                    return

                pick_v = 0
                pick_map = 'not_run'
                fast_descent_v = 0
                fast_descent_map = 'disabled'
                fast_descent_label = ''
                retract_v = 0
                retract_map = 'not_run'
                final_v = 0
                final_map = 'not_run'
                home_v = 0
                home_map = 'not_run'
                use_fingers = bool(use_fingers)
                grab_on_pick = bool(grab_on_pick)
                relax_fingers_on_pick = bool(relax_fingers_on_pick)
                close_fingers_on_di = use_fingers and grab_on_pick
                self.get_logger().info(
                    'DATALOG item_pick_gripper_mode: '
                    f'candidate={candidate_number}/{candidate_count} '
                    f'use_fingers={int(use_fingers)} '
                    f'grab_on_pick={int(grab_on_pick)} '
                    f'relax_fingers_on_pick={int(relax_fingers_on_pick)} '
                    f'close_fingers_on_di={int(close_fingers_on_di)}'
                )
                approach_label = 'MovL approach from pre-Z-up to pick approach'
                approach_ok, approach_v, approach_map = self._send_movel_goal(
                    approach_goal,
                    pre_z_up_goal,
                    LOCKED_MAX_SPEED_MM_S,
                    approach_label,
                    forced_v_percent=100,
                    forced_a_percent=100,
                )
                if not approach_ok:
                    _record_candidate_motion_failure(
                        candidate_target,
                        reason='approach_command_rejected',
                        command_label=approach_label,
                        v_percent=approach_v,
                        status=approach_map,
                    )
                    return
                if self._stop_if_cancelled_after_motion_dispatch('approach dispatch'):
                    return

                held_message = 'held-item DI gate active after queued descent command accepted'
                if not use_fingers:
                    held_message += '; fingers disabled by item profile'
                if relax_fingers_on_pick:
                    held_message += '; DO1/DO2 relaxed at pick'

                if two_stage_descent_enabled:
                    baseline_di_available, baseline_di_held, baseline_di_message = (
                        self._held_item_state()
                    )
                    self._start_pick_di_session(
                        candidate_number=candidate_number,
                        candidate_count=candidate_count,
                        baseline_held=(
                            baseline_di_held if baseline_di_available else None
                        ),
                        baseline_message=baseline_di_message,
                    )
                    fast_descent_label = (
                        'MovLIO canary fast descent to monitored switch '
                        f'+{fast_switch_z_up_mm:.0f} mm with suction at 0 percent'
                    )
                    (
                        fast_descent_ok,
                        fast_descent_v,
                        fast_descent_map,
                    ) = self._send_movelio_goal(
                        candidate_fast_descent_goal,
                        approach_goal,
                        post_speed_mm_s,
                        fast_descent_label,
                        mdis=self._build_pick_descent_mdis(),
                        forced_v_percent=int(round(fast_descent_speed_percent)),
                        forced_a_percent=100,
                    )
                    if not fast_descent_ok:
                        self._clear_pick_di_session()
                        _record_candidate_motion_failure(
                            candidate_target,
                            reason='pick_fast_descent_command_rejected',
                            command_label=fast_descent_label,
                            v_percent=fast_descent_v,
                            status=fast_descent_map,
                        )
                        return
                    if self._stop_if_cancelled_after_motion_dispatch(
                        'canary fast pick descent dispatch'
                    ):
                        self._clear_pick_di_session()
                        return
                    pick_label = (
                        'MovLIO monitored final descent from canary switch '
                        'to pick goal at legacy speed'
                    )
                    pick_reference_goal = candidate_fast_descent_goal
                else:
                    pick_label = (
                        'MovLIO descent to pick goal with suction at 0 percent'
                    )
                    pick_reference_goal = approach_goal

                pick_ok, pick_v, pick_map = self._send_movelio_goal(
                    pick_goal,
                    pick_reference_goal,
                    post_speed_mm_s,
                    pick_label,
                    mdis=self._build_pick_descent_mdis(),
                    forced_v_percent=PICK_DESCENT_SPEED_PERCENT,
                    forced_a_percent=100,
                )
                if not pick_ok:
                    if two_stage_descent_enabled:
                        stop_ok, stop_message = self._send_stop_command(
                            'Stop() [two-stage final descent enqueue rejected]'
                        )
                        self.get_logger().warn(
                            'DATALOG item_pick_two_stage_descent_abort: '
                            'reason="final_enqueue_rejected" '
                            f'stop_ok={int(stop_ok)} stop="{stop_message}"'
                        )
                    self._clear_pick_di_session()
                    _record_candidate_motion_failure(
                        candidate_target,
                        reason='pick_descent_command_rejected',
                        command_label=pick_label,
                        v_percent=pick_v,
                        status=pick_map,
                    )
                    return
                # This is the sole physical-attempt boundary. Validation,
                # approach/recovery, and rejected commands never advance it.
                with self._lock:
                    self._physical_pick_attempt = int(
                        getattr(self, '_physical_pick_attempt', 0) or 0
                    ) + 1
                    self._physical_repick_attempt = max(
                        0,
                        self._physical_pick_attempt - 1,
                    )
                    self._last_pick_motion_started = True
                    self._last_pick_candidate_number = candidate_number
                    physical_pick_attempt = self._physical_pick_attempt
                    physical_repick_attempt = self._physical_repick_attempt
                motion_attempts_used += 1
                self.get_logger().info(
                    'DATALOG item_pick_physical_attempt_started: '
                    f'detection_round={int(getattr(self, "_item_redetection_phase", 0) or 0)} '
                    f'candidate={candidate_number}/{candidate_count} '
                    f'physical_pick_attempt={physical_pick_attempt} '
                    f'physical_repick_attempt={physical_repick_attempt} '
                    'motion_started=1'
                )
                if self._stop_if_cancelled_after_motion_dispatch('pick descent dispatch'):
                    self._clear_pick_di_session()
                    return

                if not two_stage_descent_enabled:
                    baseline_di_available, baseline_di_held, baseline_di_message = (
                        self._held_item_state()
                    )
                    self._start_pick_di_session(
                        candidate_number=candidate_number,
                        candidate_count=candidate_count,
                        baseline_held=(
                            baseline_di_held if baseline_di_available else None
                        ),
                        baseline_message=baseline_di_message,
                    )
                if baseline_di_available:
                    self.get_logger().info(
                        f'Pick DI baseline for monitored descent candidate '
                        f'{candidate_number}/{candidate_count}: {baseline_di_message}'
                    )
                else:
                    self.get_logger().warn(
                        f'Pick DI baseline unavailable after queued descent command accepted for candidate '
                        f'{candidate_number}/{candidate_count}: {baseline_di_message}'
                    )
                if not use_fingers:
                    retract_mode_text = (
                        'fingers relaxed + suction kept on'
                        if relax_fingers_on_pick
                        else 'gripper open + suction kept on'
                    )
                elif grab_on_pick:
                    retract_mode_text = 'gripper already closed at DI stop + suction kept on'
                else:
                    retract_mode_text = (
                        f'gripper close at {MOVLIO_RETRACT_CLOSE_DISTANCE_PERCENT} percent'
                    )
                fast_command_log = (
                    f'{fast_descent_label} v={fast_descent_v}/{fast_descent_map}; '
                    if two_stage_descent_enabled
                    else ''
                )
                self.get_logger().info(
                    'DATALOG item_pick_monitored_sequence_accepted: '
                    f'candidate={candidate_number}/{candidate_count}, '
                    f'commands="{pre_z_up_label} v={pre_z_up_v}/{pre_z_up_map}; '
                    f'{approach_label} v={approach_v}/{approach_map}; '
                    f'{fast_command_log}'
                    f'{pick_label} v={pick_v}/{pick_map}" '
                    f'di="{held_message}"'
                )
                self.get_logger().info(
                    'Monitored pick sequence (pre-Z-up + approach + '
                    f'{"canary fast/slow descent" if two_stage_descent_enabled else "slow descent"}): '
                    f'candidate={candidate_number}/{candidate_count}, '
                    f'pick stand-off offsets (X {x_offset_mm:.0f}, Y {y_offset_mm:.0f}, Z {z_offset_mm:.0f} mm). '
                    f'tool offset=({tool_offset_x_mm:.1f},{tool_offset_y_mm:.1f},{tool_offset_z_mm:.1f},'
                    f'{tool_offset_rx_deg:.1f},{tool_offset_ry_deg:.1f},{tool_offset_rz_deg:.1f}). '
                    f'approach_z={PICK_FIXED_APPROACH_Z_UP_MM:.0f} mm fixed, '
                    f'use_fingers={int(use_fingers)}, grab_on_pick={int(grab_on_pick)}, '
                    f'relax_fingers_on_pick={int(relax_fingers_on_pick)}, '
                    f'final_z_up={final_z_up_mm:.0f} mm, '
                    f'di="{held_message}". '
                    f'(v: pre-z-up={pre_z_up_v}/{pre_z_up_map}, '
                    f'approach={approach_v}/{approach_map}, '
                    f'fast-down={fast_descent_v}/{fast_descent_map}, '
                    f'final-down={pick_v}/{pick_map}).'
                )

                completion_known, picked, completion_message = self._monitor_pick_descent_to_goal(
                    pick_goal,
                    candidate_number=candidate_number,
                    candidate_count=candidate_count,
                    close_fingers_on_di=close_fingers_on_di,
                )
                if not completion_known:
                    if qa_detection_yaml_path is not None:
                        self._write_pick_result_to_detection_yaml(
                            qa_detection_yaml_path,
                            picked=False,
                            message=completion_message,
                        )
                    _record_candidate_motion_failure(
                        candidate_target,
                        reason='pick_motion_completion_unknown',
                        command_label=completion_message,
                    )
                    return
                if self._stop_if_cancelled_after_motion_dispatch('pick descent monitor'):
                    return
                if picked:
                    self._set_action_text(
                        f'candidate {candidate_number}/{candidate_count}: '
                        f'DI triggered; retracting fixed {PICK_DI_TRIGGER_RETRACT_Z_UP_MM:.0f} mm '
                        f'at {PICK_DI_TRIGGER_RETRACT_SPEED_PERCENT} percent...'
                    )
                    retract_label = (
                        f'MovLIO {PICK_DI_TRIGGER_RETRACT_SPEED_PERCENT} percent retract after DI trigger '
                        f'with {retract_mode_text}'
                    )
                    stop_snapshot = self.snapshot()
                    stopped_pose = (
                        float(stop_snapshot.tcp_values.get('x', pick_goal[0])),
                        float(stop_snapshot.tcp_values.get('y', pick_goal[1])),
                        float(stop_snapshot.tcp_values.get('z', pick_goal[2])),
                        float(stop_snapshot.tcp_values.get('rx', pick_goal[3])),
                        float(stop_snapshot.tcp_values.get('ry', pick_goal[4])),
                        float(stop_snapshot.tcp_values.get('rz', pick_goal[5])),
                    )
                    if two_stage_descent_enabled:
                        trigger_z_up_mm = stopped_pose[2] - pick_goal[2]
                        final_zone_travel_mm = max(
                            0.0,
                            fast_switch_z_up_mm - trigger_z_up_mm,
                        )
                        self.get_logger().info(
                            'DATALOG item_pick_two_stage_descent_stop_envelope: '
                            f'candidate={candidate_number}/{candidate_count} '
                            f'switch_z_up_mm={fast_switch_z_up_mm:.2f} '
                            f'trigger_stop_z_up_mm={trigger_z_up_mm:.2f} '
                            f'final_zone_travel_mm={final_zone_travel_mm:.2f} '
                            f'fast_speed_percent={fast_descent_speed_percent:.0f} '
                            f'final_speed_percent={PICK_DESCENT_SPEED_PERCENT}'
                        )
                    retract_goal = (
                        stopped_pose[0],
                        stopped_pose[1],
                        stopped_pose[2] + PICK_DI_TRIGGER_RETRACT_Z_UP_MM,
                        stopped_pose[3],
                        stopped_pose[4],
                        stopped_pose[5],
                    )
                    post_pick_final_z_goal = (
                        retract_goal[0],
                        retract_goal[1],
                        retract_goal[2] + float(final_z_up_mm),
                        retract_goal[3],
                        retract_goal[4],
                        retract_goal[5],
                    )
                    retract_ok, retract_v, retract_map = self._send_movelio_goal(
                        retract_goal,
                        stopped_pose,
                        LOCKED_MAX_SPEED_MM_S,
                        retract_label,
                        mdis=self._build_pick_retract_mdis(
                            use_fingers=use_fingers,
                            grab_on_pick=grab_on_pick,
                            relax_fingers_on_pick=relax_fingers_on_pick,
                        ),
                        forced_v_percent=PICK_DI_TRIGGER_RETRACT_SPEED_PERCENT,
                        forced_a_percent=100,
                    )
                    if not retract_ok:
                        _record_candidate_motion_failure(
                            candidate_target,
                            reason='di_retract_command_rejected',
                            command_label=retract_label,
                            v_percent=retract_v,
                            status=retract_map,
                        )
                        return
                    if self._stop_if_cancelled_after_motion_dispatch('DI retract dispatch'):
                        return
                    self.get_logger().info(
                        'DATALOG item_pick_di_trigger_retract: '
                        f'candidate={candidate_number}/{candidate_count} '
                        f'command="{retract_label}" '
                        f'retract_goal=({retract_goal[0]:.3f}, {retract_goal[1]:.3f}, {retract_goal[2]:.3f}, '
                        f'{retract_goal[3]:.3f}, {retract_goal[4]:.3f}, {retract_goal[5]:.3f}) '
                        f'effective_v={retract_v} status={retract_map}'
                    )

                    final_label = 'MovL max-speed final Z-up'
                    final_ok, final_v, final_map = self._send_movel_goal(
                        post_pick_final_z_goal,
                        retract_goal,
                        LOCKED_MAX_SPEED_MM_S,
                        final_label,
                        forced_v_percent=FINAL_Z_UP_SPEED_PERCENT,
                        forced_a_percent=PICK_DI_FINAL_Z_UP_ACCEL_PERCENT,
                    )
                    if not final_ok:
                        _record_candidate_motion_failure(
                            candidate_target,
                            reason='di_final_z_command_rejected',
                            command_label=final_label,
                            v_percent=final_v,
                            status=final_map,
                        )
                        return
                    if self._stop_if_cancelled_after_motion_dispatch('final Z-up dispatch'):
                        return

                    home_label = 'MovJ fixed home after DI pick'
                    home_ok, home_v, home_map = self._send_fixed_home_movj_goal(home_label)
                    if not home_ok:
                        _record_candidate_motion_failure(
                            candidate_target,
                            reason='di_home_command_rejected',
                            command_label=home_label,
                            v_percent=home_v,
                            status=home_map,
                        )
                        return
                    if self._stop_if_cancelled_after_motion_dispatch('fixed-home dispatch'):
                        return
                    home_command_id = self._last_fixed_home_movj_command_id
                    home_command_id_missing = home_command_id is None
                    self.get_logger().info(
                        'DATALOG item_pick_di_trigger_return_home_queued: '
                        f'candidate={candidate_number}/{candidate_count} '
                        f'commands="{retract_label} v={retract_v}/{retract_map}; '
                        f'{final_label} v={final_v}/{final_map}; '
                        f'{home_label} v={home_v}/{home_map}" '
                        f'fixed_home_command_id={home_command_id}'
                    )
                    if home_command_id_missing:
                        self.get_logger().warn(
                            'DI-success fixed-home MovJ was accepted without a controller '
                            'command id; pick_cycle must verify fixed home before tray teach.'
                        )
                    self._set_cached_pick_progress(
                        next_candidate_index=candidate_index + 1,
                        successful_candidate_index=candidate_index,
                    )
                    queue_message = (
                        f'{completion_message}; '
                        f'fixed-home return queued command_id={home_command_id}; '
                    )
                    if home_command_id_missing:
                        queue_message += (
                            f'{FIXED_HOME_COMMAND_ID_MISSING_MARKER}; '
                            'fixed-home verification required before tray'
                        )
                    else:
                        queue_message += 'ready for tray'

                    if qa_detection_yaml_path is not None:
                        self._write_pick_result_to_detection_yaml(
                            qa_detection_yaml_path,
                            picked=True,
                            message=queue_message,
                        )
                    seek_complete_notified = self._notify_item_detect_seek_complete()
                    self.get_logger().info(
                        f'pick confirmed by DI; fixed-home return queued; ready for tray: '
                        f'{queue_message}'
                    )
                    busy_released_after_queue = True
                    self._set_busy(False)
                    if home_command_id_missing:
                        self._set_action_text(
                            'pick confirmed by DI; fixed-home return queued without command id; '
                            f'{FIXED_HOME_COMMAND_ID_MISSING_MARKER}; '
                            'fixed-home verification required before tray'
                        )
                    else:
                        self._set_action_text(
                            'pick confirmed by DI; fixed-home return queued; ready for tray'
                        )
                    return

                last_no_pick_reason = completion_message
                if qa_detection_yaml_path is not None:
                    self._write_pick_result_to_detection_yaml(
                        qa_detection_yaml_path,
                        picked=False,
                        message=completion_message,
                    )
                failed_area_published = self._publish_failed_pick_area(
                    candidate_target,
                    reason='no_di_at_pick_depth',
                )
                self.get_logger().info(
                    'DATALOG item_pick_no_di_final_z_up: '
                    f'candidate={candidate_number}/{candidate_count} '
                    f'final_z_up_speed={FINAL_Z_UP_SPEED_PERCENT} '
                    'final_z_up_movelio=1 '
                    f'close_at_zero_percent={int(use_fingers and not relax_fingers_on_pick)} '
                    f'relax_fingers_on_pick={int(relax_fingers_on_pick)} '
                    'purge_at_zero_percent=1 '
                    f'relax_at_{MOVLIO_NO_DI_PURGE_OFF_DISTANCE_PERCENT}_percent=1 '
                    f'failed_area_published={int(failed_area_published)} '
                    f'message="{completion_message}"'
                )
                if relax_fingers_on_pick:
                    final_label = (
                        'MovLIO max-speed final Z-up after no DI with fingers relaxed '
                        'and suction purge at 0 percent, purge off at 50 percent'
                    )
                elif use_fingers:
                    final_label = (
                        'MovLIO max-speed final Z-up after no DI with gripper close '
                        'and suction purge at 0 percent, relax at 50 percent'
                    )
                else:
                    final_label = (
                        'MovLIO max-speed final Z-up after no DI with gripper open '
                        'and suction purge at 0 percent, purge off at 50 percent'
                    )
                final_ok, final_v, final_map = self._send_movelio_goal(
                    final_z_goal,
                    pick_goal,
                    LOCKED_MAX_SPEED_MM_S,
                    final_label,
                    mdis=self._build_no_di_final_z_up_mdis(
                        use_fingers=use_fingers,
                        relax_fingers_on_pick=relax_fingers_on_pick,
                    ),
                    forced_v_percent=FINAL_Z_UP_SPEED_PERCENT,
                    forced_a_percent=100,
                )
                if not final_ok:
                    self._log_pick_queue_command_rejected(
                        final_label,
                        final_v,
                        final_map,
                    )
                    return
                if self._stop_if_cancelled_after_motion_dispatch('no-DI final Z-up dispatch'):
                    return
                if not self._wait_for_tcp_xyz_goal(final_z_goal[:3]):
                    return

                next_candidate_index = candidate_index + 1
                self._set_cached_pick_progress(
                    next_candidate_index=next_candidate_index,
                )
                if next_candidate_index < candidate_count:
                    last_no_pick_reason = (
                        f'{completion_message}; '
                        'gripper close/purge then relax queued during final Z-up; '
                        'trying next cached same-seek candidate directly from final Z-up'
                    )
                    retry_text = (
                        f'no DI trigger for candidate {candidate_number}/{candidate_count}; '
                        f'trying cached candidate {next_candidate_index + 1}/{candidate_count} '
                        'from final Z-up'
                    )
                    self.get_logger().warn(
                        f'{retry_text}. Gripper close/purge then relax queued during no-DI final Z-up. '
                        f'failed_area_published={int(failed_area_published)}. {completion_message}'
                    )
                    self._set_action_text(retry_text)
                    candidate_index = next_candidate_index
                    continue

                self._set_action_text(
                    f'candidate {candidate_number}/{candidate_count}: '
                    'queueing fixed-home MovJ after no-DI final Z-up; cached candidates exhausted...'
                )
                home_label = 'MovJ fixed home after no-DI pick'
                home_ok, home_v, home_map = self._send_fixed_home_movj_goal(home_label)
                if not home_ok:
                    self._log_pick_queue_command_rejected(
                        home_label,
                        home_v,
                        home_map,
                    )
                    return
                if self._stop_if_cancelled_after_motion_dispatch('no-DI fixed-home dispatch'):
                    return

                home_command_id = self._last_fixed_home_movj_command_id
                self.get_logger().info(
                    f'candidate {candidate_number}/{candidate_count} no-DI exhausted candidates; '
                    f'fixed_home_command_id={home_command_id}'
                )

                completion_known, _, home_completion_message = self._wait_for_pick_attempt_completion(
                    home_command_id,
                    candidate_number=candidate_number,
                    candidate_count=candidate_count,
                    monitor_di=False,
                )
                if not completion_known:
                    if qa_detection_yaml_path is not None:
                        self._write_pick_result_to_detection_yaml(
                            qa_detection_yaml_path,
                            picked=False,
                            message=home_completion_message,
                        )
                    return
                last_no_pick_reason = (
                    f'{completion_message}; '
                    'gripper close/purge then relax queued during final Z-up; '
                    f'{home_completion_message}'
                )
                retry_text = (
                    f'no DI trigger for candidate {candidate_number}/{candidate_count}; '
                    'no more current candidates'
                )
                self.get_logger().warn(
                    f'{retry_text}. Gripper close/purge then relax queued during no-DI final Z-up. '
                    f'{completion_message}; {home_completion_message}'
                )
                self._set_action_text(retry_text)
                break

            if motion_attempts_used <= 0:
                if last_reject_kind == 'ik' or last_validation_reject_reason or (
                    last_ik_reject_reason and not last_camera_bin_reject_reason
                ):
                    reject_reason = (
                        last_validation_reject_reason
                        or last_ik_reject_reason
                        or 'IK rejected every ranked candidate.'
                    )
                    self.get_logger().warn(
                        f'Pick worker rejected all {candidate_count} ranked candidates by IK: '
                        f'{reject_reason}'
                    )
                    generation = self._rearm_item_pose_watch_after_rejected_pose(
                        reject_reason
                    )
                    if generation is not None:
                        repick_started = self._request_item_detect_repick_from_home(
                            reject_reason
                        )
                        if repick_started:
                            waiting_for_next_pose_after_reject = True
                            self._start_item_repick_seek_status_monitor(
                                generation,
                                reject_reason,
                            )
                            watchdog = threading.Thread(
                                target=self._item_pose_watchdog_worker,
                                args=(generation,),
                                daemon=True,
                            )
                            watchdog.start()
                        else:
                            self._reset_item_pose_watch_after_repick_start_failure(
                                generation,
                                reject_reason,
                            )
                    return

                if last_reject_kind == 'camera_bin' or last_camera_bin_reject_reason:
                    reject_reason = (
                        last_camera_bin_reject_reason
                        or 'Camera-bin rejected every ranked candidate.'
                    )
                    self.get_logger().warn(
                        f'Pick worker rejected all {candidate_count} ranked candidates before motion: '
                        f'{reject_reason}'
                    )
                    generation = self._rearm_item_pose_watch_after_camera_bin_reject(
                        reject_reason
                    )
                    if generation is not None:
                        repick_started = self._request_item_detect_repick_after_camera_bin_reject(
                            reject_reason
                        )
                        if repick_started:
                            waiting_for_next_pose_after_reject = True
                            self._start_item_repick_seek_status_monitor(generation, reject_reason)
                            watchdog = threading.Thread(
                                target=self._item_pose_watchdog_worker,
                                args=(generation,),
                                daemon=True,
                            )
                            watchdog.start()
                        else:
                            self._reset_item_pose_watch_after_repick_start_failure(
                                generation,
                                reject_reason,
                            )
                    return

                # _compute_base_goal_from_item_target / its TF helper has
                # already logged the root cause; we re-state here so the
                # operator sees the dispatch outcome too.
                self.get_logger().warn(
                    f'Pick worker aborted: failed to compute base goal for '
                    f'{candidate_count} ranked candidate(s) (see TF lookup warning above).'
                )
                return

            reject_reason = (
                f'all candidates failed; requesting new item seek from home. '
                f'Attempts={motion_attempts_used}/{candidate_count}. '
                f'{last_no_pick_reason or "held-item DI did not trigger"}'
            )
            self.get_logger().warn(reject_reason)
            generation = self._rearm_item_pose_watch_after_pick_di_failure(reject_reason)
            if generation is not None:
                repick_started = self._request_item_detect_repick_from_home(reject_reason)
                if repick_started:
                    waiting_for_next_pose_after_reject = True
                    self._start_item_repick_seek_status_monitor(generation, reject_reason)
                    watchdog = threading.Thread(
                        target=self._item_pose_watchdog_worker,
                        args=(generation,),
                        daemon=True,
                    )
                    watchdog.start()
                else:
                    self._reset_item_pose_watch_after_repick_start_failure(
                        generation,
                        reject_reason,
                    )
            return
        except Exception as exc:
            self.get_logger().error(f'MovL predicted-goal flow failed: {exc}')
            self._set_action_text(f'MovL predicted-goal flow failed: {exc}')
        finally:
            self._clear_pick_di_session()
            if not seek_complete_notified and not waiting_for_next_pose_after_reject:
                self._notify_item_detect_seek_complete()
            if not busy_released_after_queue and not waiting_for_next_pose_after_reject:
                self._set_busy(False)
            with self._lock:
                self._active_pick_workers = max(
                    0,
                    int(getattr(self, '_active_pick_workers', 1)) - 1,
                )

    @function_timing.traced("item_pick_notify_seek_complete")
    def _notify_item_detect_seek_complete(self) -> bool:
        if not self._item_seek_complete_client.service_is_ready():
            if not self._item_seek_complete_client.wait_for_service(timeout_sec=0.2):
                self.get_logger().warn(
                    f'Item detect seek-complete service not ready: {self._item_seek_complete_service_name}'
                )
                return False

        future = self._item_seek_complete_client.call_async(Trigger.Request())
        started = time.time()
        while rclpy.ok() and not future.done():
            if (time.time() - started) >= 1.0:
                self.get_logger().warn(
                    f'Timed out notifying item detect seek completion: {self._item_seek_complete_service_name}'
                )
                return False
            time.sleep(0.02)

        exception = future.exception()
        if exception is not None:
            self.get_logger().warn(f'Item detect seek-complete call failed: {exception}')
            return False

        response = future.result()
        if response is not None and not bool(response.success):
            self.get_logger().warn(f'Item detect seek-complete rejected: {response.message}')
            return False
        return response is not None

    def _set_action_text(self, text: str) -> None:
        with self._lock:
            self._snapshot.action_text = text

    def _set_busy(self, busy: bool) -> None:
        with self._lock:
            self._snapshot.busy = busy

    @function_timing.traced("item_pick_start_sequence_service")
    def _start_sequence_service_callback(
        self,
        request: TrayInterceptStart.Request,
        response: TrayInterceptStart.Response,
    ) -> TrayInterceptStart.Response:
        started = self.run_item_sequence(
            float(request.tray_vector_wait_timeout_sec),
            float(request.ee_intercept_speed_mm_s),
            float(request.tray_intercept_x_offset_mm),
            float(request.tray_intercept_y_offset_mm),
            float(request.tray_standoff_z_mm),
            float(request.follow_distance_mm),
            self.get_pre_pick_settling_time_sec(),
            self.get_pick_settling_time_sec(),
            bool(request.troubleshoot_tf_only),
        )
        with self._lock:
            response.started = bool(started)
            response.message = str(self._snapshot.action_text)
            response.applied_tray_vector_wait_timeout_sec = float(self._item_pose_watch_timeout_sec)
            response.applied_ee_intercept_speed_mm_s = float(self._post_stop_movel_speed_mm_s)
            response.applied_tray_intercept_x_offset_mm = float(self._post_stop_x_offset_mm)
            response.applied_tray_intercept_y_offset_mm = float(self._post_stop_y_offset_mm)
            response.applied_ee_final_pose_angle_deg = 0.0
            response.applied_tray_standoff_z_mm = float(self._post_stop_z_offset_mm)
            response.applied_follow_distance_mm = 0.0
            response.applied_post_follow_z_up_mm = float(PICK_FIXED_APPROACH_Z_UP_MM)
            response.applied_troubleshoot_tf_only = bool(self._item_pose_watch_tf_only_mode)
        return response

    def set_track_trigger_handler(self, handler) -> None:
        self._track_trigger_handler = handler

    @function_timing.traced("item_pick_track_service")
    def _track_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        with self._lock:
            if self._lifecycle_start_blocked_locked('Item track'):
                response.success = False
                response.message = str(self._snapshot.action_text)
                return response
        handler = self._track_trigger_handler
        if handler is not None:
            try:
                started, message = handler()
            except Exception as exc:
                started = False
                message = f'Track virtual-click failed: {exc}'
                self._set_action_text(message)
        else:
            started = self.run_track_from_current_settings()
            with self._lock:
                message = str(self._snapshot.action_text)

        response.success = bool(started)
        response.message = str(message)
        return response

    @function_timing.traced("item_pick_track_status_service")
    def _track_status_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        with self._lock:
            armed = bool(self._item_pose_watch_armed)
            busy = bool(self._snapshot.busy)
            active_workers = int(getattr(self, '_active_pick_workers', 0))
            cleanup_active = bool(getattr(self, '_drop_last_inflight', False))
            lifecycle_latched = bool(
                getattr(self, '_lifecycle_stop_latched', False)
            )
            cancelling = bool(self._cancel_requested) and (
                active_workers > 0 or cleanup_active
            )
            action_text = str(self._snapshot.action_text)
            detection_phase = int(
                getattr(self, '_item_redetection_phase', 0) or 0
            )
            di_confirmed = bool(
                getattr(self, '_item_di_confirmed_for_current_track', False)
            )
            di_confirmed_generation = int(
                getattr(self, '_item_di_confirmed_generation', 0) or 0
            )
            physical_pick_attempt = int(
                getattr(self, '_physical_pick_attempt', 0) or 0
            )
            physical_repick_attempt = int(
                getattr(self, '_physical_repick_attempt', 0) or 0
            )
            motion_started = bool(
                getattr(self, '_last_pick_motion_started', False)
            )
            candidate_number = int(
                getattr(self, '_last_pick_candidate_number', 0) or 0
            )

        # ``Trigger.success`` is consumed by pick_cycle as the active flag.
        # A pose callback clears ``armed`` before its motion worker starts, so
        # reporting only ``armed`` makes an active pick look idle.  Stop/Reset
        # may then home while the worker is still dispatching candidates.
        response.success = armed or busy or active_workers > 0 or cleanup_active
        if armed:
            response.message = f'Track armed: waiting for "{ITEM_POSE_TOPIC}". {action_text}'
        elif cancelling:
            response.message = (
                f'Track cancelling/draining {active_workers} active motion worker(s). '
                f'{action_text}'
            )
        elif busy or active_workers > 0:
            response.message = f'Track busy but not armed. {action_text}'
        elif cleanup_active:
            response.message = f'Item return/drop cleanup active. {action_text}'
        elif lifecycle_latched:
            response.message = f'Track idle; lifecycle stop latched. {action_text}'
        else:
            response.message = f'Track not armed. {action_text}'
        response.message += (
            ' [item_pick_progress '
            f'detection_phase={detection_phase} '
            f'detection_round={detection_phase} '
            f'di_confirmed={int(di_confirmed)} '
            f'di_confirmed_generation={di_confirmed_generation} '
            f'candidate_number={candidate_number} '
            f'physical_pick_attempt={physical_pick_attempt} '
            f'physical_repick_attempt={physical_repick_attempt} '
            f'motion_started={int(motion_started)}]'
        )
        return response

    @function_timing.traced("item_pick_lifecycle_cancel")
    def _lifecycle_cancel_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Persistently close the pick start gate, then cancel active motion."""
        with self._lock:
            self._lifecycle_stop_latched = True
            self._lifecycle_epoch = int(
                getattr(self, '_lifecycle_epoch', 0)
            ) + 1
            self._cancel_requested = True
        result = self._track_cancel_service_callback(request, response)
        result.message = f'Lifecycle stop latched; {result.message}'
        return result

    def resume_lifecycle_start_gate(self, reason: str) -> tuple[bool, str]:
        """Open the lifecycle start gate only from an explicit safe handshake."""
        with self._lock:
            active_workers = int(getattr(self, '_active_pick_workers', 0))
            active = (
                bool(self._item_pose_watch_armed)
                or bool(self._snapshot.busy)
                or active_workers > 0
                or bool(getattr(self, '_drop_last_inflight', False))
                or bool(getattr(self, '_manual_stop_inflight', False))
            )
            if active:
                message = (
                    f'Lifecycle resume rejected ({reason}): item-pick activity '
                    'is not proven idle.'
                )
                self._snapshot.action_text = message
                return False, message
            was_latched = bool(
                getattr(self, '_lifecycle_stop_latched', False)
            )
            self._lifecycle_stop_latched = False
            self._cancel_requested = False
            self._lifecycle_epoch = int(
                getattr(self, '_lifecycle_epoch', 0)
            ) + 1
            message = (
                f'Lifecycle start gate ready ({reason}); '
                f'previously_latched={int(was_latched)}'
            )
            self._snapshot.action_text = message
            return True, message

    @function_timing.traced("item_pick_lifecycle_resume")
    def _lifecycle_resume_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        success, message = self.resume_lifecycle_start_gate(
            'bridge verified-home handshake'
        )
        response.success = success
        response.message = message
        return response

    @function_timing.traced("item_pick_cancel_track")
    def _track_cancel_service_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Cancel an armed or moving item-pick sequence.

        Stop/Reset is allowed to arrive after the pose callback has handed
        commands to the Dobot queue.  In that state merely disarming the pose
        watch is insufficient: the worker can continue with the remaining
        ranked candidates after the arm has been homed.  Latch the worker's
        cancellation flag and synchronously send ``Stop`` before acknowledging
        an active-motion cancellation.  ``busy`` remains true until the worker
        exits so lifecycle cleanup can prove it is settled before homing.
        """
        del request
        with self._lock:
            was_armed = bool(self._item_pose_watch_armed)
            was_busy = bool(self._snapshot.busy)
            active_workers = int(getattr(self, '_active_pick_workers', 0))
            cleanup_active = bool(getattr(self, '_drop_last_inflight', False))
            stop_dispatched = bool(self._item_pose_watch_stop_dispatched)
            motion_active = cleanup_active or active_workers > 0 or (
                was_busy and (stop_dispatched or not was_armed)
            )

            if was_armed or was_busy or active_workers > 0 or cleanup_active:
                self._cancel_requested = True
                self._item_pose_watch_generation += 1
                self._item_pose_watch_armed = False
                self._item_pose_watch_deadline_monotonic = 0.0
                self._item_pose_watch_seq_floor = self._item_pose_seq
                if not motion_active:
                    self._item_pose_watch_stop_dispatched = False
                    self._snapshot.busy = False

        if motion_active:
            stop_ok, stop_message = self._send_stop_command(
                'Stop() [item track cancel]'
            )
            seek_complete_notified = self._notify_item_detect_seek_complete()
            stale_busy_cleared = False
            if stop_ok:
                with self._lock:
                    remaining_workers = int(
                        getattr(self, '_active_pick_workers', 0)
                    )
                    cleanup_still_active = bool(
                        getattr(self, '_drop_last_inflight', False)
                    )
                    if remaining_workers <= 0 and not cleanup_still_active:
                        # A failed multi-candidate pick may leave ``busy`` set
                        # while it waits for another pose even though its worker
                        # has already exited. Stop has now closed any queued
                        # motion, so no nonexistent worker remains to clear it.
                        self._item_pose_watch_stop_dispatched = False
                        self._snapshot.busy = False
                        stale_busy_cleared = True
            worker_state_message = (
                'no active motion worker remained; busy cleared; '
                if stale_busy_cleared
                else 'waiting for motion worker to exit; '
            )
            message = (
                'Cancelled active item-pick motion; '
                f'Stop {"accepted" if stop_ok else "failed"} ({stop_message}); '
                f'{worker_state_message}'
                f'seek_complete_notified={int(bool(seek_complete_notified))}'
            )
            self._set_action_text(message)
            response.success = bool(stop_ok)
            response.message = message
            return response

        if was_armed or was_busy or active_workers > 0 or cleanup_active:
            seek_complete_notified = self._notify_item_detect_seek_complete()
            message = (
                'Cancelled item track: armed-only state cleared (no motion); '
                f'seek_complete_notified={int(bool(seek_complete_notified))}'
            )
            self._set_action_text(message)
        else:
            message = 'No active item track to cancel'
            self._set_action_text(message)

        response.success = True
        response.message = message
        return response

    @function_timing.traced("item_pick_track_service")
    def run_track_from_current_settings(self) -> bool:
        with self._lock:
            item_pose_watch_timeout_sec = float(self._item_pose_watch_timeout_sec)
            post_stop_z_offset_mm = float(self._post_stop_z_offset_mm)
            pre_pick_settling_time_sec = float(self._pre_pick_settling_time_sec)
            pick_settling_time_sec = float(self._pick_settling_time_sec)
            tf_only_mode = bool(self._item_pose_watch_tf_only_mode)
            tool_offset_x_mm = float(self._tool_offset_x_mm)
            tool_offset_y_mm = float(self._tool_offset_y_mm)
            tool_offset_z_mm = float(self._tool_offset_z_mm)
            tool_offset_rx_deg = float(self._tool_offset_rx_deg)
            tool_offset_ry_deg = float(self._tool_offset_ry_deg)
            tool_offset_rz_deg = float(self._tool_offset_rz_deg)

        return self.run_item_sequence(
            item_pose_watch_timeout_sec,
            LOCKED_MAX_SPEED_MM_S,
            0.0,
            0.0,
            post_stop_z_offset_mm,
            0.0,
            pre_pick_settling_time_sec,
            pick_settling_time_sec,
            tf_only_mode,
            tool_offset_x_mm,
            tool_offset_y_mm,
            tool_offset_z_mm,
            tool_offset_rx_deg,
            tool_offset_ry_deg,
            tool_offset_rz_deg,
        )

    def _is_cancel_requested(self) -> bool:
        with self._lock:
            return bool(self._cancel_requested)

    def is_manual_release_inflight(self) -> bool:
        with self._lock:
            return bool(self._manual_release_inflight)

    def is_manual_stop_inflight(self) -> bool:
        with self._lock:
            return bool(self._manual_stop_inflight)

    def is_goal_tf_diagnose_inflight(self) -> bool:
        with self._lock:
            return bool(self._goal_tf_diagnose_inflight)

    def request_publish_tool_offset_preview(
        self,
        tool_offset_x_mm: float,
        tool_offset_y_mm: float,
        tool_offset_z_mm: float,
        tool_offset_rx_deg: float,
        tool_offset_ry_deg: float,
        tool_offset_rz_deg: float,
    ) -> bool:
        if not self._publish_goal_debug_tf:
            self._set_action_text('TF publishing is disabled (publish_goal_debug_tf=false).')
            return False

        try:
            tx_mm = self._clamp_tool_offset_translation_mm(tool_offset_x_mm)
            ty_mm = self._clamp_tool_offset_translation_mm(tool_offset_y_mm)
            tz_mm = self._clamp_tool_offset_translation_mm(tool_offset_z_mm)
            rx_deg = self._clamp_tool_offset_rotation_deg(tool_offset_rx_deg)
            ry_deg = self._clamp_tool_offset_rotation_deg(tool_offset_ry_deg)
            rz_deg = self._clamp_tool_offset_rotation_deg(tool_offset_rz_deg)

            with self._lock:
                self._tool_offset_x_mm = tx_mm
                self._tool_offset_y_mm = ty_mm
                self._tool_offset_z_mm = tz_mm
                self._tool_offset_rx_deg = rx_deg
                self._tool_offset_ry_deg = ry_deg
                self._tool_offset_rz_deg = rz_deg

            self._publish_goal_debug_transform(
                self._tool_offset_preview_parent_frame_id,
                self._tool_offset_preview_frame_id,
                tx_mm,
                ty_mm,
                tz_mm,
                rx_deg,
                ry_deg,
                rz_deg,
            )
            self._publish_goal_debug_transform(
                self._tool_offset_preview_frame_id,
                self._tool_offset_preview_axis_x_tip_frame_id,
                GOAL_TF_DIAG_AXIS_LENGTH_MM,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            self._publish_goal_debug_transform(
                self._tool_offset_preview_frame_id,
                self._tool_offset_preview_axis_y_tip_frame_id,
                0.0,
                GOAL_TF_DIAG_AXIS_LENGTH_MM,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            self._publish_goal_debug_transform(
                self._tool_offset_preview_frame_id,
                self._tool_offset_preview_axis_z_tip_frame_id,
                0.0,
                0.0,
                GOAL_TF_DIAG_AXIS_LENGTH_MM,
                0.0,
                0.0,
                0.0,
            )
            self._set_action_text(
                'Published tool-offset TF preview in RViz: '
                f'"{self._tool_offset_preview_parent_frame_id}" -> "{self._tool_offset_preview_frame_id}".'
            )
            return True
        except Exception as exc:
            self._set_action_text(f'Failed to publish tool-offset TF preview: {exc}')
            return False

    def request_goal_tf_diagnose(
        self,
        post_stop_z_offset_mm: float,
        tool_offset_x_mm: float,
        tool_offset_y_mm: float,
        tool_offset_z_mm: float,
        tool_offset_rx_deg: float,
        tool_offset_ry_deg: float,
        tool_offset_rz_deg: float,
    ) -> bool:
        with self._lock:
            if self._goal_tf_diagnose_inflight:
                self._snapshot.action_text = 'TF diagnose already in progress.'
                return False
            item_target = self._last_item_target
            if item_target is None:
                self._snapshot.action_text = 'No item pose received yet. Publish item pose, then retry TF diagnose.'
                return False
            self._goal_tf_diagnose_inflight = True
            self._snapshot.action_text = (
                f'TF diagnose started: computing goal and publishing "{self._post_stop_movel_goal_debug_frame_id}"...'
            )

        worker = threading.Thread(
            target=self._goal_tf_diagnose_worker,
            args=(
                item_target,
                post_stop_z_offset_mm,
                tool_offset_x_mm,
                tool_offset_y_mm,
                tool_offset_z_mm,
                tool_offset_rx_deg,
                tool_offset_ry_deg,
                tool_offset_rz_deg,
            ),
            daemon=True,
        )
        worker.start()
        return True

    def _goal_tf_diagnose_worker(
        self,
        item_target: ItemPoseTarget,
        post_stop_z_offset_mm: float,
        tool_offset_x_mm: float,
        tool_offset_y_mm: float,
        tool_offset_z_mm: float,
        tool_offset_rx_deg: float,
        tool_offset_ry_deg: float,
        tool_offset_rz_deg: float,
    ) -> None:
        try:
            z_offset_mm = max(
                POST_STOP_Z_OFFSET_MIN,
                min(POST_STOP_Z_OFFSET_MAX, float(post_stop_z_offset_mm)),
            )
            tx_mm = self._clamp_tool_offset_translation_mm(tool_offset_x_mm)
            ty_mm = self._clamp_tool_offset_translation_mm(tool_offset_y_mm)
            tz_mm = self._clamp_tool_offset_translation_mm(tool_offset_z_mm)
            rx_deg = self._clamp_tool_offset_rotation_deg(tool_offset_rx_deg)
            ry_deg = self._clamp_tool_offset_rotation_deg(tool_offset_ry_deg)
            rz_deg = self._clamp_tool_offset_rotation_deg(tool_offset_rz_deg)

            self._set_action_text('TF diagnose: computing goal from latest item pose...')
            base_goal = self._compute_base_goal_from_item_target(
                item_target,
                0.0,
                0.0,
                z_offset_mm,
                tx_mm,
                ty_mm,
                tz_mm,
                rx_deg,
                ry_deg,
                rz_deg,
                self._post_stop_movel_speed_mm_s,
                predict_target_motion=False,
            )
            if base_goal is None:
                return

            approach_goal = (
                base_goal.x_mm,
                base_goal.y_mm,
                base_goal.z_mm + PICK_FIXED_APPROACH_Z_UP_MM,
                base_goal.rx_deg,
                base_goal.ry_deg,
                base_goal.rz_deg,
            )
            self._publish_primary_goal_debug_transform(approach_goal)
            self._set_action_text(
                'TF diagnose published. RViz TF frame: '
                f'"{self._post_stop_movel_goal_debug_frame_id}" '
                f'(fixed approach Z {PICK_FIXED_APPROACH_Z_UP_MM:.0f} mm).'
            )
        finally:
            with self._lock:
                self._goal_tf_diagnose_inflight = False

    def request_release_pulse(self) -> bool:
        with self._lock:
            if self._manual_release_inflight:
                self._snapshot.action_text = 'Release pulse already in progress.'
                return False
            if self._snapshot.busy or int(getattr(self, '_active_pick_workers', 0)) > 0:
                self._snapshot.action_text = 'Cannot run release pulse while item pick sequence is active.'
                return False
            self._manual_release_inflight = True
            self._snapshot.action_text = (
                f'Release pulse started: DO1 OFF + DO3 OFF (vent) + DO2 pulse {int(MANUAL_RELEASE_PULSE_MS)} ms.'
            )

        worker = threading.Thread(target=self._manual_release_pulse_worker, daemon=True)
        worker.start()
        return True

    def _release_gripper_pulse(self) -> bool:
        """Run the release DO sequence synchronously.

        DO1 OFF -> DO3 OFF (vent) -> DO2 ON -> hold ``MANUAL_RELEASE_PULSE_MS``
        -> DO2 OFF, leaving the gripper in a neutral state. Returns True
        only when every DO command was accepted. Shared by the manual
        release pulse and the drop-last-item worker so both paths fire an
        identical, single-sourced release.
        """
        if not self._send_do(1, 0):
            return False
        if not self._send_do(3, 0):
            return False
        if not self._send_do(2, 1):
            return False
        pulse_sec = float(MANUAL_RELEASE_PULSE_MS) * 0.001
        wait_started = time.monotonic()
        while (time.monotonic() - wait_started) < pulse_sec:
            time.sleep(0.01)
        return self._send_do(2, 0)

    def _manual_release_pulse_worker(self) -> None:
        try:
            if not self._release_gripper_pulse():
                return
            self._set_action_text('Release pulse complete: neutral state (DO1 OFF, DO2 OFF, DO3 OFF/vent).')
        finally:
            with self._lock:
                self._manual_release_inflight = False

    def request_manual_stop(self) -> bool:
        with self._lock:
            if self._manual_stop_inflight:
                self._snapshot.action_text = 'Manual Stop already in progress.'
                return False
            self._manual_stop_inflight = True
            self._cancel_requested = True
            self._lifecycle_stop_latched = True
            self._lifecycle_epoch = int(
                getattr(self, '_lifecycle_epoch', 0)
            ) + 1
            self._item_pose_watch_generation += 1
            self._item_pose_watch_armed = False
            self._item_pose_watch_stop_dispatched = False
            self._item_pose_watch_deadline_monotonic = 0.0
            if int(getattr(self, '_active_pick_workers', 0)) <= 0:
                self._snapshot.busy = False
            self._snapshot.action_text = 'Manual Stop requested. Sending robot Stop...'

        worker = threading.Thread(target=self._manual_stop_worker, daemon=True)
        worker.start()
        return True

    def _manual_stop_worker(self) -> None:
        try:
            if not self._wait_for_service(self._stop_client, 'Stop'):
                return
            stop_response = self._call_service(self._stop_client, Stop.Request(), 'Stop() [manual]')
            if stop_response is None:
                return
            if int(getattr(stop_response, 'res', -1)) < 0:
                return
            self._set_action_text('Manual Stop sent. Sequence halted.')
        finally:
            self._notify_item_detect_seek_complete()
            with self._lock:
                if int(getattr(self, '_active_pick_workers', 0)) <= 0:
                    self._snapshot.busy = False
                self._manual_stop_inflight = False

    def _build_motion_param_value(self, v_percent: int, a_percent: int, include_tool: bool = True) -> list[str]:
        args = [f'v={int(v_percent)}', f'a={int(a_percent)}']
        if include_tool:
            args.append('tool=1')
        return [','.join(args)]

    def get_command_hysteresis_sec(self) -> float:
        with self._lock:
            return float(self._command_hysteresis_sec)

    def get_pre_pick_settling_time_sec(self) -> float:
        with self._lock:
            return float(self._pre_pick_settling_time_sec)

    def get_pick_settling_time_sec(self) -> float:
        with self._lock:
            return float(self._pick_settling_time_sec)

    def get_settling_time_sec(self) -> float:
        return self.get_pick_settling_time_sec()

    def set_command_hysteresis_sec(self, command_hysteresis_sec: float) -> float:
        with self._lock:
            self._command_hysteresis_sec = max(
                COMMAND_HYSTERESIS_MIN_SEC,
                min(COMMAND_HYSTERESIS_MAX_SEC, float(command_hysteresis_sec)),
            )
            return float(self._command_hysteresis_sec)

    @function_timing.traced("item_pick_run_sequence")
    def run_item_sequence(
        self,
        item_pose_watch_timeout_sec: float,
        post_stop_movel_speed_mm_s: float,
        post_stop_x_offset_mm: float,
        post_stop_y_offset_mm: float,
        post_stop_z_offset_mm: float,
        follow_distance_mm: float,
        pre_pick_settling_time_sec: float,
        pick_settling_time_sec: float,
        tf_only_mode: bool,
        tool_offset_x_mm: float | None = None,
        tool_offset_y_mm: float | None = None,
        tool_offset_z_mm: float | None = None,
        tool_offset_rx_deg: float | None = None,
        tool_offset_ry_deg: float | None = None,
        tool_offset_rz_deg: float | None = None,
    ) -> bool:
        # Capture a generation before profile/filesystem work.  If Stop/Reset
        # races that work and a later resume opens the boolean gate again, the
        # changed epoch still prevents this older request from arming.
        with self._lock:
            if self._lifecycle_start_blocked_locked('Item sequence start'):
                return False
            request_lifecycle_epoch = int(
                getattr(self, '_lifecycle_epoch', 0)
            )
        _ = (
            post_stop_movel_speed_mm_s,
            follow_distance_mm,
            pre_pick_settling_time_sec,
            pick_settling_time_sec,
        )
        active_item_id, saved_offsets = self._sync_profile_tool_offsets_from_state(force=False)
        if self._active_item_file is None:
            self._set_action_text(
                f'No item yaml in "{self._items_dir}". Drop the active item teach there and restart.'
            )
            return False
        if saved_offsets is None:
            # No sidecar fallback: pick: section must exist in the
            # active items yaml. Seeding a zero-offset section here is
            # rejected by the new contract (only item_teach_node may
            # create the first file), so surface the gap to the
            # operator instead of silently arming on defaults.
            profile_error = self._active_profile_pick_error or 'pick: section missing'
            self._set_action_text(
                f'Invalid pick profile in "{self._active_item_file.name}": '
                f'{profile_error} Save it in item pick before arming.'
            )
            return False
        saved_use_fingers = bool(saved_offsets['use_fingers'])
        saved_grab_on_pick = bool(saved_offsets['grab_on_pick'])
        saved_relax_fingers_on_pick = bool(
            saved_offsets.get(
                'relax_fingers_on_pick',
                RELAX_FINGERS_ON_PICK_DEFAULT,
            )
        )
        saved_post_stop_z_offset_mm = float(saved_offsets['item_standoff_z_mm'])
        saved_final_z_up_mm = float(saved_offsets['final_z_up_mm'])
        raw_pre_pick_settling_time_sec = float(saved_offsets['pre_pick_settling_time_sec'])
        raw_pick_settling_time_sec = float(saved_offsets['pick_settling_time_sec'])
        saved_pre_pick_settling_time_sec = PICK_RUNTIME_SETTLING_TIME_SEC
        saved_pick_settling_time_sec = PICK_RUNTIME_SETTLING_TIME_SEC
        saved_tool_offset_x_mm = float(saved_offsets['tool_offset_x_mm'])
        saved_tool_offset_y_mm = float(saved_offsets['tool_offset_y_mm'])
        saved_tool_offset_z_mm = float(saved_offsets['tool_offset_z_mm'])
        saved_tool_offset_rx_deg = float(saved_offsets['tool_offset_rx_deg'])
        saved_tool_offset_ry_deg = float(saved_offsets['tool_offset_ry_deg'])
        saved_tool_offset_rz_deg = float(saved_offsets['tool_offset_rz_deg'])
        if self.count_publishers(ITEM_POSE_TOPIC) <= 0:
            self._reset_runtime_state(
                f'No item pose publisher on "{ITEM_POSE_TOPIC}". Node reset.'
            )
            return False

        with self._lock:
            if (
                self._lifecycle_start_blocked_locked('Item sequence start')
                or int(getattr(self, '_lifecycle_epoch', 0))
                != request_lifecycle_epoch
            ):
                if not bool(getattr(self, '_lifecycle_stop_latched', False)):
                    self._snapshot.action_text = (
                        'Item sequence start rejected: lifecycle generation '
                        'changed while the request was being prepared.'
                    )
                return False
            if self._snapshot.busy or int(getattr(self, '_active_pick_workers', 0)) > 0:
                self._snapshot.action_text = 'Busy running previous item pick sequence.'
                return False
            self._snapshot.busy = True
            self._item_redetection_phase = 1
            self._physical_pick_attempt = 0
            self._physical_repick_attempt = 0
            self._last_pick_motion_started = False
            self._last_pick_candidate_number = 0
            self._item_di_confirmed_for_current_track = False
            self._last_base_drop_release_goal = None
            self._last_item_target = None
            self._cancel_requested = False
            self._item_pose_watch_timeout_sec = clamp_item_pose_watch_timeout_sec(
                item_pose_watch_timeout_sec
            )
            self._post_stop_movel_speed_mm_s = LOCKED_MAX_SPEED_MM_S
            # Always pick on the detected item location (no XY operator offset).
            self._post_stop_x_offset_mm = 0.0
            self._post_stop_y_offset_mm = 0.0
            self._post_stop_z_offset_mm = saved_post_stop_z_offset_mm
            self._use_fingers = saved_use_fingers
            self._grab_on_pick = saved_grab_on_pick
            self._relax_fingers_on_pick = saved_relax_fingers_on_pick
            self._final_z_up_mm = saved_final_z_up_mm
            self._pre_pick_settling_time_sec = saved_pre_pick_settling_time_sec
            self._pick_settling_time_sec = saved_pick_settling_time_sec
            self._tool_offset_x_mm = saved_tool_offset_x_mm
            self._tool_offset_y_mm = saved_tool_offset_y_mm
            self._tool_offset_z_mm = saved_tool_offset_z_mm
            self._tool_offset_rx_deg = saved_tool_offset_rx_deg
            self._tool_offset_ry_deg = saved_tool_offset_ry_deg
            self._tool_offset_rz_deg = saved_tool_offset_rz_deg
            self._item_pose_watch_tf_only_mode = bool(tf_only_mode)
            generation = self._arm_item_pose_watch_locked()
            watch_timeout_sec = float(self._item_pose_watch_timeout_sec)
            mode_name = 'tf_only' if tf_only_mode else 'normal'
            self.get_logger().info(
                'Run settings: '
                f'mode={mode_name} '
                f'wait={watch_timeout_sec:.0f}s '
                f'motion_profile=CP{MAX_SAFE_CP_PERCENT}/SF{MAX_SAFE_SPEED_FACTOR_PERCENT}/'
                f'SpeedJ{MAX_SAFE_SPEEDJ_PERCENT}/AccJ{MAX_SAFE_ACCJ_PERCENT}/'
                f'SpeedL{MAX_SAFE_SPEEDL_PERCENT}/AccL{MAX_SAFE_ACCL_PERCENT} '
                f'legacy_speed_field={self._post_stop_movel_speed_mm_s:.0f} '
                f'offsets(x=0,y=0,z={self._post_stop_z_offset_mm:.0f}) mm '
                f'tool_offset(x={self._tool_offset_x_mm:.1f},y={self._tool_offset_y_mm:.1f},'
                f'z={self._tool_offset_z_mm:.1f},rx={self._tool_offset_rx_deg:.1f},'
                f'ry={self._tool_offset_ry_deg:.1f},rz={self._tool_offset_rz_deg:.1f}) '
                f'fixed_approach_z={PICK_FIXED_APPROACH_Z_UP_MM:.0f} mm '
                f'use_fingers={int(self._use_fingers)} '
                f'grab_on_pick={int(self._grab_on_pick)} '
                f'relax_fingers_on_pick={int(self._relax_fingers_on_pick)} '
                f'final_z_up={self._final_z_up_mm:.0f} mm '
                f'pre_pick_settle={self._pre_pick_settling_time_sec:.1f}s '
                f'pick_settle={self._pick_settling_time_sec:.1f}s '
                f'camera_bin_pose_attempts={self._camera_bin_valid_pose_max_attempts}'
            )
            self.get_logger().info(
                'Pick timing: settle reduced from '
                f'pre_pick={clamp_settling_time_sec(raw_pre_pick_settling_time_sec):.2f}s '
                f'pick={clamp_settling_time_sec(raw_pick_settling_time_sec):.2f}s '
                'to '
                f'pre_pick={self._pre_pick_settling_time_sec:.2f}s '
                f'pick={self._pick_settling_time_sec:.2f}s '
                '(teach-file settle ignored for runtime max-speed pick)'
            )
            self.get_logger().info(
                'Pick timing config: '
                f'pre_pick_settle={self._pre_pick_settling_time_sec:.2f}s '
                f'pick_settle={self._pick_settling_time_sec:.2f}s'
            )
            if tf_only_mode:
                self._snapshot.action_text = (
                    f'Troubleshoot mode armed... waiting for "{ITEM_POSE_TOPIC}" '
                    f'for {watch_timeout_sec:.0f}s (TF preview only).'
                )
            else:
                self._snapshot.action_text = (
                    f'Item pick sequence armed... waiting for fresh "{ITEM_POSE_TOPIC}" '
                    f'for {watch_timeout_sec:.0f}s.'
                )

        watchdog = threading.Thread(
            target=self._item_pose_watchdog_worker,
            args=(generation,),
            daemon=True,
        )
        watchdog.start()
        return True

    def _wait_for_service(self, client, label: str, timeout_sec: float = 10.0) -> bool:
        started = time.time()
        while rclpy.ok():
            if client.wait_for_service(timeout_sec=0.3):
                return True
            if (time.time() - started) >= timeout_sec:
                break
        self._set_action_text(f'{label} service not ready.')
        return False

    def _call_service(
        self,
        client,
        request,
        label: str,
        timeout_sec: float = 8.0,
        *,
        log_rejections: bool = True,
    ):
        self._set_action_text(f'SEND {label}')
        future = client.call_async(request)
        started = time.time()
        while rclpy.ok() and not future.done():
            if (time.time() - started) >= timeout_sec:
                self._set_action_text(f'Timeout: {label}')
                return None
            time.sleep(0.02)

        exception = future.exception()
        if exception is not None:
            self._set_action_text(f'Exception: {label}: {exception}')
            return None

        response = future.result()
        if response is None:
            self._set_action_text(f'No response: {label}')
            return None

        res = int(getattr(response, 'res', -1))
        robot_return = str(getattr(response, 'robot_return', '')).strip()
        if res < 0:
            if log_rejections:
                if robot_return:
                    self.get_logger().warn(
                        'DATALOG robot_command_rejected: '
                        f'label="{label}" res={res} robot_return="{robot_return}"'
                    )
                else:
                    self.get_logger().warn(
                        'DATALOG robot_command_rejected: '
                        f'label="{label}" res={res}'
                    )
            if robot_return:
                self._set_action_text(f'FAIL {label}: res={res}, return={robot_return}')
            else:
                self._set_action_text(f'FAIL {label}: res={res}')
            return response

        if robot_return:
            self._set_action_text(f'OK {label}: {robot_return}')
        else:
            self._set_action_text(f'OK {label}')
        return response

class ItemPickGui:
    def __init__(self, node: ItemPickNode) -> None:
        self.node = node
        self._gui_thread_id = threading.get_ident()
        self.root = tk.Tk()
        self.root.title('Item Pick Operator Console')
        fixed_width = 960
        fixed_height = 520
        self.root.geometry(f'{fixed_width}x{fixed_height}')
        self.root.minsize(fixed_width, fixed_height)
        self.root.maxsize(fixed_width, fixed_height)
        self.root.resizable(False, False)
        self._closed = False
        # The GUI no longer owns a runtime settings file -- the single
        # items/<id>.yaml owned by the node carries pick:/ros__parameters:
        # with both the tool teach offsets and the UI runtime state.
        # The debounce timer below queues a section write that goes
        # through teach_io.write_subject_section so concurrent writes
        # from item_teach / item_detect / pick never stomp each other.
        self._runtime_settings_save_after_id: str | None = None
        self._suspend_runtime_settings_events = False
        self._active_item_id: str | None = None
        self._active_profile_has_saved_tool_teach = False
        self._saved_tool_teach_values: tuple[float, ...] | None = None

        outer = tk.Frame(self.root, padx=12, pady=12)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1, uniform='maincols')
        outer.columnconfigure(1, weight=1, uniform='maincols')
        outer.rowconfigure(0, weight=1)

        modes_frame = tk.LabelFrame(outer, text='Operating Modes', padx=10, pady=8)
        modes_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 6))
        modes_frame.columnconfigure(0, weight=1)
        slider_length = 250

        self.run_button = tk.Button(
            modes_frame,
            text='Arm Track Item',
            command=self._run_clicked,
            width=20,
        )
        self.run_button.grid(row=0, column=0, sticky='ew')

        self.stop_button = tk.Button(
            modes_frame,
            text='Stop',
            command=self._stop_clicked,
            width=20,
        )
        self.stop_button.grid(row=1, column=0, sticky='ew', pady=(8, 0))
        self._stop_default_bg = self.stop_button.cget('bg')
        self._stop_default_fg = self.stop_button.cget('fg')
        self._stop_default_active_bg = self.stop_button.cget('activebackground')
        self._stop_default_active_fg = self.stop_button.cget('activeforeground')
        self._set_stop_button_enabled(False)

        self.release_button = tk.Button(
            modes_frame,
            text=f'Release {int(MANUAL_RELEASE_PULSE_MS)}ms',
            command=self._release_clicked,
            width=20,
            bg='#1565c0',
            fg='white',
            activebackground='#0d47a1',
            activeforeground='white',
        )
        self.release_button.grid(row=2, column=0, sticky='ew', pady=(8, 0))

        # Mirror the node's current tf_only state so the GUI checkbox
        # reflects what the headless service path will actually use.
        # (The node's value is seeded by the `tf_only_default` ROS
        # parameter; production launches set it to False.)
        self.tf_only_var = tk.BooleanVar(value=bool(node._item_pose_watch_tf_only_mode))
        self.tf_only_button = tk.Button(
            modes_frame,
            command=self._toggle_tf_only_clicked,
            width=24,
        )
        self.tf_only_button.grid(row=3, column=0, sticky='ew', pady=(8, 0))
        self._tf_only_default_bg = self.tf_only_button.cget('bg')
        self._tf_only_default_fg = self.tf_only_button.cget('fg')
        self._tf_only_default_active_bg = self.tf_only_button.cget('activebackground')
        self._tf_only_default_active_fg = self.tf_only_button.cget('activeforeground')
        self._sync_tf_only_button(is_busy=False)

        tk.Label(modes_frame, text='Item pose wait timeout (sec)').grid(row=4, column=0, sticky='w', pady=(10, 0))
        self.item_pose_watch_timeout_var = tk.DoubleVar(value=ITEM_POSE_WATCH_TIMEOUT_SEC)
        self.item_pose_watch_timeout_scale = tk.Scale(
            modes_frame,
            from_=ITEM_POSE_WATCH_TIMEOUT_MIN,
            to=ITEM_POSE_WATCH_TIMEOUT_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=slider_length,
            variable=self.item_pose_watch_timeout_var,
            showvalue=True,
        )
        self.item_pose_watch_timeout_scale.grid(row=5, column=0, sticky='ew')

        mode_hint = (
            'Press Arm Track Item, then wait for item pose. '
            'Normal mode queues approach, descent IO, retract IO, final Z-up, and fixed home without intermediate waits.'
        )
        tk.Label(
            modes_frame,
            text=mode_hint,
            anchor='w',
            justify=tk.LEFT,
            wraplength=430,
        ).grid(row=6, column=0, sticky='w', pady=(8, 0))
        self.action_var = tk.StringVar(value='Ready')
        tk.Label(
            modes_frame,
            textvariable=self.action_var,
            anchor='w',
            justify=tk.LEFT,
            wraplength=430,
        ).grid(row=7, column=0, sticky='ew', pady=(8, 0))

        ee_settings_frame = tk.LabelFrame(outer, text='EE Position Settings', padx=10, pady=8)
        ee_settings_frame.grid(row=0, column=1, sticky='nsew')
        ee_settings_frame.columnconfigure(0, weight=1)

        tk.Label(ee_settings_frame, text='Item stand-off (+item Z, mm)').grid(row=0, column=0, sticky='w')
        self.post_stop_z_offset_var = tk.DoubleVar(value=100.0)
        self.post_stop_z_offset_scale = tk.Scale(
            ee_settings_frame,
            from_=POST_STOP_Z_OFFSET_MIN,
            to=POST_STOP_Z_OFFSET_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=slider_length,
            variable=self.post_stop_z_offset_var,
            showvalue=True,
        )
        self.post_stop_z_offset_scale.grid(row=1, column=0, sticky='ew')

        finger_mode_frame = tk.Frame(ee_settings_frame)
        finger_mode_frame.grid(row=2, column=0, sticky='ew', pady=(10, 0))
        finger_mode_frame.columnconfigure(0, weight=1)
        finger_mode_frame.columnconfigure(1, weight=1)
        finger_mode_frame.columnconfigure(2, weight=1)
        self.use_fingers_var = tk.BooleanVar(value=USE_FINGERS_DEFAULT)
        self.grab_on_pick_var = tk.BooleanVar(value=GRAB_ON_PICK_DEFAULT)
        self.relax_fingers_on_pick_var = tk.BooleanVar(
            value=RELAX_FINGERS_ON_PICK_DEFAULT
        )
        self.use_fingers_check = tk.Checkbutton(
            finger_mode_frame,
            text='Use fingers',
            variable=self.use_fingers_var,
        )
        self.use_fingers_check.grid(row=0, column=0, sticky='w')
        self.grab_on_pick_check = tk.Checkbutton(
            finger_mode_frame,
            text='Grab on pick',
            variable=self.grab_on_pick_var,
        )
        self.grab_on_pick_check.grid(row=0, column=1, sticky='w')
        self.relax_fingers_on_pick_check = tk.Checkbutton(
            finger_mode_frame,
            text='Relax fingers at pick',
            variable=self.relax_fingers_on_pick_var,
        )
        self.relax_fingers_on_pick_check.grid(row=1, column=0, columnspan=3, sticky='w')

        tk.Label(ee_settings_frame, text='Final Z-up (mm)').grid(
            row=3,
            column=0,
            sticky='w',
            pady=(10, 0),
        )
        self.final_z_up_var = tk.DoubleVar(value=FINAL_Z_UP_DEFAULT)
        self.final_z_up_scale = tk.Scale(
            ee_settings_frame,
            from_=FINAL_Z_UP_MIN,
            to=FINAL_Z_UP_MAX,
            orient=tk.HORIZONTAL,
            resolution=1.0,
            length=slider_length,
            variable=self.final_z_up_var,
            showvalue=True,
        )
        self.final_z_up_scale.grid(row=4, column=0, sticky='ew')

        settling_frame = tk.Frame(ee_settings_frame)
        settling_frame.grid(row=5, column=0, sticky='ew', pady=(10, 0))
        settling_frame.columnconfigure(0, weight=1, uniform='settle_cols')
        settling_frame.columnconfigure(1, weight=1, uniform='settle_cols')
        tk.Label(settling_frame, text='Pre-pick settle (sec)').grid(row=0, column=0, sticky='w', padx=(0, 4))
        tk.Label(settling_frame, text='Pick settle (sec)').grid(row=0, column=1, sticky='w', padx=(4, 0))
        self.pre_pick_settling_time_var = tk.DoubleVar(value=SETTLING_TIME_DEFAULT_SEC)
        self.pick_settling_time_var = tk.DoubleVar(value=SETTLING_TIME_DEFAULT_SEC)
        self.pre_pick_settling_time_scale = tk.Scale(
            settling_frame,
            from_=SETTLING_TIME_MIN_SEC,
            to=SETTLING_TIME_MAX_SEC,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=120,
            variable=self.pre_pick_settling_time_var,
            showvalue=True,
        )
        self.pre_pick_settling_time_scale.grid(row=1, column=0, sticky='ew', padx=(0, 4))
        self.pick_settling_time_scale = tk.Scale(
            settling_frame,
            from_=SETTLING_TIME_MIN_SEC,
            to=SETTLING_TIME_MAX_SEC,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=120,
            variable=self.pick_settling_time_var,
            showvalue=True,
        )
        self.pick_settling_time_scale.grid(row=1, column=1, sticky='ew', padx=(4, 0))

        tk.Label(
            ee_settings_frame,
            text='Tool offset (x/y/z mm, rx/ry/rz deg) | Use button to preview TF wrt Link6 in RViz',
        ).grid(row=8, column=0, sticky='w', pady=(10, 0))
        tool_offset_frame = tk.Frame(ee_settings_frame)
        tool_offset_frame.grid(row=9, column=0, sticky='ew')
        for col in range(3):
            tool_offset_frame.columnconfigure(col, weight=1, uniform='tool_offset_col')
        self.active_profile_var = tk.StringVar(value='Active item teach: waiting for item_detect...')
        tk.Label(
            tool_offset_frame,
            textvariable=self.active_profile_var,
            anchor='w',
            justify=tk.LEFT,
            wraplength=430,
        ).grid(row=0, column=0, columnspan=3, sticky='ew', pady=(0, 6))

        self.tool_offset_x_var = tk.DoubleVar(value=0.0)
        self.tool_offset_y_var = tk.DoubleVar(value=0.0)
        self.tool_offset_z_var = tk.DoubleVar(value=0.0)
        self.tool_offset_rx_var = tk.DoubleVar(value=0.0)
        self.tool_offset_ry_var = tk.DoubleVar(value=0.0)
        self.tool_offset_rz_var = tk.DoubleVar(value=0.0)
        # Per-item Dobot internal Tool N TCP. Mirrors the controller's
        # ``Tool(N)`` definition so the ``item_movel_goal_flange_tcp``
        # debug TF can show where Link6 (URDF flange) actually lands
        # after the controller subtracts its own Tool TCP from the
        # commanded pose. Seeded from the node's launch params and
        # overridden by the pick: section of the active items yaml.
        self.dobot_tool_offset_x_var = tk.DoubleVar(value=float(node._dobot_tool_offset_x_mm))
        self.dobot_tool_offset_y_var = tk.DoubleVar(value=float(node._dobot_tool_offset_y_mm))
        self.dobot_tool_offset_z_var = tk.DoubleVar(value=float(node._dobot_tool_offset_z_mm))
        self.dobot_tool_offset_rx_var = tk.DoubleVar(value=float(node._dobot_tool_offset_rx_deg))
        self.dobot_tool_offset_ry_var = tk.DoubleVar(value=float(node._dobot_tool_offset_ry_deg))
        self.dobot_tool_offset_rz_var = tk.DoubleVar(value=float(node._dobot_tool_offset_rz_deg))

        tk.Label(tool_offset_frame, text='X').grid(row=1, column=0, sticky='w')
        tk.Label(tool_offset_frame, text='Y').grid(row=1, column=1, sticky='w')
        tk.Label(tool_offset_frame, text='Z').grid(row=1, column=2, sticky='w')
        self.tool_offset_x_spinbox = tk.Spinbox(
            tool_offset_frame,
            from_=TOOL_OFFSET_TRANSLATION_MIN_MM,
            to=TOOL_OFFSET_TRANSLATION_MAX_MM,
            increment=1.0,
            textvariable=self.tool_offset_x_var,
            width=9,
            format='%.1f',
        )
        self.tool_offset_x_spinbox.grid(row=2, column=0, sticky='ew', padx=(0, 4))
        self.tool_offset_y_spinbox = tk.Spinbox(
            tool_offset_frame,
            from_=TOOL_OFFSET_TRANSLATION_MIN_MM,
            to=TOOL_OFFSET_TRANSLATION_MAX_MM,
            increment=1.0,
            textvariable=self.tool_offset_y_var,
            width=9,
            format='%.1f',
        )
        self.tool_offset_y_spinbox.grid(row=2, column=1, sticky='ew', padx=(0, 4))
        self.tool_offset_z_spinbox = tk.Spinbox(
            tool_offset_frame,
            from_=TOOL_OFFSET_TRANSLATION_MIN_MM,
            to=TOOL_OFFSET_TRANSLATION_MAX_MM,
            increment=1.0,
            textvariable=self.tool_offset_z_var,
            width=9,
            format='%.1f',
        )
        self.tool_offset_z_spinbox.grid(row=2, column=2, sticky='ew')

        tk.Label(tool_offset_frame, text='Rx').grid(row=3, column=0, sticky='w', pady=(8, 0))
        tk.Label(tool_offset_frame, text='Ry').grid(row=3, column=1, sticky='w', pady=(8, 0))
        tk.Label(tool_offset_frame, text='Rz').grid(row=3, column=2, sticky='w', pady=(8, 0))
        self.tool_offset_rx_spinbox = tk.Spinbox(
            tool_offset_frame,
            from_=TOOL_OFFSET_ROTATION_MIN_DEG,
            to=TOOL_OFFSET_ROTATION_MAX_DEG,
            increment=1.0,
            textvariable=self.tool_offset_rx_var,
            width=9,
            format='%.1f',
        )
        self.tool_offset_rx_spinbox.grid(row=4, column=0, sticky='ew', padx=(0, 4))
        self.tool_offset_ry_spinbox = tk.Spinbox(
            tool_offset_frame,
            from_=TOOL_OFFSET_ROTATION_MIN_DEG,
            to=TOOL_OFFSET_ROTATION_MAX_DEG,
            increment=1.0,
            textvariable=self.tool_offset_ry_var,
            width=9,
            format='%.1f',
        )
        self.tool_offset_ry_spinbox.grid(row=4, column=1, sticky='ew', padx=(0, 4))
        self.tool_offset_rz_spinbox = tk.Spinbox(
            tool_offset_frame,
            from_=TOOL_OFFSET_ROTATION_MIN_DEG,
            to=TOOL_OFFSET_ROTATION_MAX_DEG,
            increment=1.0,
            textvariable=self.tool_offset_rz_var,
            width=9,
            format='%.1f',
        )
        self.tool_offset_rz_spinbox.grid(row=4, column=2, sticky='ew')

        tool_offset_button_frame = tk.Frame(tool_offset_frame)
        tool_offset_button_frame.grid(row=5, column=0, columnspan=3, sticky='ew', pady=(10, 0))
        tool_offset_button_frame.columnconfigure(0, weight=1)
        tool_offset_button_frame.columnconfigure(1, weight=1)
        self.save_tool_offset_button = tk.Button(
            tool_offset_button_frame,
            text='Save Tool Teach',
            command=self._save_tool_offset_profile_clicked,
            width=24,
            bg='#6a1b9a',
            fg='white',
            activebackground='#4a148c',
            activeforeground='white',
        )
        self.save_tool_offset_button.grid(row=0, column=0, sticky='ew', padx=(0, 4))
        self.show_tool_tf_button = tk.Button(
            tool_offset_button_frame,
            text='Show Tool TF (Link6 Ref)',
            command=self._show_tool_tf_preview_clicked,
            width=24,
            bg='#2e7d32',
            fg='white',
            activebackground='#1b5e20',
            activeforeground='white',
        )
        self.show_tool_tf_button.grid(row=0, column=1, sticky='ew', padx=(4, 0))
        self.tool_offset_profile_status_var = tk.StringVar(
            value='Tool teach must be saved for the active item teach before arming.'
        )
        tk.Label(
            tool_offset_frame,
            textvariable=self.tool_offset_profile_status_var,
            anchor='w',
            justify=tk.LEFT,
            wraplength=430,
        ).grid(row=6, column=0, columnspan=3, sticky='ew', pady=(8, 0))

        self._arm_locked_setting_controls = [
            self.item_pose_watch_timeout_scale,
            self.post_stop_z_offset_scale,
            self.use_fingers_check,
            self.grab_on_pick_check,
            self.relax_fingers_on_pick_check,
            self.final_z_up_scale,
            self.pre_pick_settling_time_scale,
            self.pick_settling_time_scale,
            self.tool_offset_x_spinbox,
            self.tool_offset_y_spinbox,
            self.tool_offset_z_spinbox,
            self.tool_offset_rx_spinbox,
            self.tool_offset_ry_spinbox,
            self.tool_offset_rz_spinbox,
        ]

        self._register_runtime_setting_traces()
        self._load_runtime_settings()
        self._sync_profile_tool_offsets_from_state(force=True)
        self._sync_tf_only_button(is_busy=False)
        self._set_arm_locked_setting_controls_enabled(True)
        self.node.set_track_trigger_handler(self._track_clicked_from_service)
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._refresh()

    def _track_clicked_from_service(self) -> tuple[bool, str]:
        if self._closed:
            return False, 'Item pick GUI is closed.'

        if threading.get_ident() == self._gui_thread_id:
            started = bool(self._run_clicked(resume_lifecycle_gate=False))
            snapshot = self.node.snapshot()
            return started, str(snapshot.action_text)

        done = threading.Event()
        result: dict[str, object] = {
            'started': False,
            'message': 'Track virtual-click did not run.',
        }

        def run_on_gui() -> None:
            try:
                started = bool(self._run_clicked(resume_lifecycle_gate=False))
                snapshot = self.node.snapshot()
                result['started'] = started
                result['message'] = str(snapshot.action_text)
            except Exception as exc:
                message = f'Track virtual-click failed: {exc}'
                self.node._set_action_text(message)
                self.action_var.set(message)
                result['started'] = False
                result['message'] = message
            finally:
                done.set()

        try:
            self.root.after(0, run_on_gui)
        except Exception as exc:
            return False, f'Track virtual-click could not reach GUI thread: {exc}'

        if not done.wait(timeout=5.0):
            return False, 'Track virtual-click timed out waiting for GUI thread.'

        return bool(result['started']), str(result['message'])

    def _run_clicked(self, *, resume_lifecycle_gate: bool = True) -> bool:
        # A physical GUI click is an explicit local resume.  A ROS track
        # request must never open the gate itself because it may have been
        # queued before lifecycle Stop; the bridge opens it only after its
        # verified-home prepare/recover handshake.
        if resume_lifecycle_gate:
            resumed, message = self.node.resume_lifecycle_start_gate(
                'explicit item-pick GUI Run'
            )
            if not resumed:
                self.action_var.set(message)
                return False
        self._sync_profile_tool_offsets_from_state()
        if not self._can_arm_sequence():
            reason = self._arm_block_reason()
            self.node._set_action_text(reason)
            self.action_var.set(reason)
            return False
        item_pose_watch_timeout_value = float(self.item_pose_watch_timeout_var.get())
        post_stop_z_offset_value = float(self.post_stop_z_offset_var.get())
        pre_pick_settling_time_value = float(self.pre_pick_settling_time_var.get())
        pick_settling_time_value = float(self.pick_settling_time_var.get())
        tool_offset_x_value = float(self.tool_offset_x_var.get())
        tool_offset_y_value = float(self.tool_offset_y_var.get())
        tool_offset_z_value = float(self.tool_offset_z_var.get())
        tool_offset_rx_value = float(self.tool_offset_rx_var.get())
        tool_offset_ry_value = float(self.tool_offset_ry_var.get())
        tool_offset_rz_value = float(self.tool_offset_rz_var.get())
        tf_only_mode = bool(self.tf_only_var.get())

        started = self.node.run_item_sequence(
            item_pose_watch_timeout_value,
            LOCKED_MAX_SPEED_MM_S,
            0.0,
            0.0,
            post_stop_z_offset_value,
            0.0,
            pre_pick_settling_time_value,
            pick_settling_time_value,
            tf_only_mode,
            tool_offset_x_value,
            tool_offset_y_value,
            tool_offset_z_value,
            tool_offset_rx_value,
            tool_offset_ry_value,
            tool_offset_rz_value,
        )
        if not started:
            snapshot = self.node.snapshot()
            self.action_var.set(snapshot.action_text)
            return False
        else:
            self._set_stop_button_enabled(True)
            self.run_button.configure(state=tk.DISABLED)
            self.release_button.configure(state=tk.DISABLED)
            self.show_tool_tf_button.configure(state=tk.DISABLED)
            self.save_tool_offset_button.configure(state=tk.DISABLED)
            self._sync_tf_only_button(is_busy=True)
            self._set_arm_locked_setting_controls_enabled(False)
            return True

    def _stop_clicked(self) -> None:
        started = self.node.request_manual_stop()
        if not started:
            snapshot = self.node.snapshot()
            self.action_var.set(snapshot.action_text)

    def _release_clicked(self) -> None:
        started = self.node.request_release_pulse()
        if not started:
            snapshot = self.node.snapshot()
            self.action_var.set(snapshot.action_text)

    def _toggle_tf_only_clicked(self) -> None:
        current = bool(self.tf_only_var.get())
        self.tf_only_var.set(not current)
        self._sync_tf_only_button(is_busy=False)

    def _show_tool_tf_preview_clicked(self) -> None:
        tool_offset_x_value = float(self.tool_offset_x_var.get())
        tool_offset_y_value = float(self.tool_offset_y_var.get())
        tool_offset_z_value = float(self.tool_offset_z_var.get())
        tool_offset_rx_value = float(self.tool_offset_rx_var.get())
        tool_offset_ry_value = float(self.tool_offset_ry_var.get())
        tool_offset_rz_value = float(self.tool_offset_rz_var.get())

        started = self.node.request_publish_tool_offset_preview(
            tool_offset_x_value,
            tool_offset_y_value,
            tool_offset_z_value,
            tool_offset_rx_value,
            tool_offset_ry_value,
            tool_offset_rz_value,
        )
        if not started:
            snapshot = self.node.snapshot()
            self.action_var.set(snapshot.action_text)

    def _save_tool_offset_profile_clicked(self) -> None:
        saved = self._save_profile_tool_offsets_for_active_item()
        if saved:
            message = f'Saved tool teach for "{self._profile_display_name()}".'
            self.node._set_action_text(message)
            self.action_var.set(message)

    def _goal_tf_diagnose_clicked(self) -> None:
        post_stop_z_offset_value = float(self.post_stop_z_offset_var.get())
        tool_offset_x_value = float(self.tool_offset_x_var.get())
        tool_offset_y_value = float(self.tool_offset_y_var.get())
        tool_offset_z_value = float(self.tool_offset_z_var.get())
        tool_offset_rx_value = float(self.tool_offset_rx_var.get())
        tool_offset_ry_value = float(self.tool_offset_ry_var.get())
        tool_offset_rz_value = float(self.tool_offset_rz_var.get())

        started = self.node.request_goal_tf_diagnose(
            post_stop_z_offset_value,
            tool_offset_x_value,
            tool_offset_y_value,
            tool_offset_z_value,
            tool_offset_rx_value,
            tool_offset_ry_value,
            tool_offset_rz_value,
        )
        if not started:
            snapshot = self.node.snapshot()
            self.action_var.set(snapshot.action_text)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, float(value)))

    def _profile_display_name(self) -> str:
        return self.node._profile_display_name()

    @staticmethod
    def _tool_offset_signature_from_profile(
        profile_offsets: dict[str, object] | None,
    ) -> tuple[float, ...] | None:
        if profile_offsets is None:
            return None
        return (
            float(bool(profile_offsets.get('use_fingers', USE_FINGERS_DEFAULT))),
            float(bool(profile_offsets.get('grab_on_pick', GRAB_ON_PICK_DEFAULT))),
            float(bool(profile_offsets.get(
                'relax_fingers_on_pick',
                RELAX_FINGERS_ON_PICK_DEFAULT,
            ))),
            round(float(profile_offsets.get('item_standoff_z_mm', 0.0)), 4),
            round(float(profile_offsets.get('final_z_up_mm', FINAL_Z_UP_DEFAULT)), 4),
            round(float(profile_offsets.get('pre_pick_settling_time_sec', 0.0)), 4),
            round(float(profile_offsets.get('pick_settling_time_sec', 0.0)), 4),
            round(float(profile_offsets.get('tool_offset_x_mm', 0.0)), 4),
            round(float(profile_offsets.get('tool_offset_y_mm', 0.0)), 4),
            round(float(profile_offsets.get('tool_offset_z_mm', 0.0)), 4),
            round(float(profile_offsets.get('tool_offset_rx_deg', 0.0)), 4),
            round(float(profile_offsets.get('tool_offset_ry_deg', 0.0)), 4),
            round(float(profile_offsets.get('tool_offset_rz_deg', 0.0)), 4),
            round(float(profile_offsets.get('dobot_tool_offset_x_mm', 0.0)), 4),
            round(float(profile_offsets.get('dobot_tool_offset_y_mm', 0.0)), 4),
            round(float(profile_offsets.get('dobot_tool_offset_z_mm', 0.0)), 4),
            round(float(profile_offsets.get('dobot_tool_offset_rx_deg', 0.0)), 4),
            round(float(profile_offsets.get('dobot_tool_offset_ry_deg', 0.0)), 4),
            round(float(profile_offsets.get('dobot_tool_offset_rz_deg', 0.0)), 4),
        )

    def _current_tool_offset_signature(self) -> tuple[float, ...]:
        return (
            float(bool(self.use_fingers_var.get())),
            float(bool(self.grab_on_pick_var.get())),
            float(bool(self.relax_fingers_on_pick_var.get())),
            round(float(self.post_stop_z_offset_var.get()), 4),
            round(float(self.final_z_up_var.get()), 4),
            round(float(self.pre_pick_settling_time_var.get()), 4),
            round(float(self.pick_settling_time_var.get()), 4),
            round(float(self.tool_offset_x_var.get()), 4),
            round(float(self.tool_offset_y_var.get()), 4),
            round(float(self.tool_offset_z_var.get()), 4),
            round(float(self.tool_offset_rx_var.get()), 4),
            round(float(self.tool_offset_ry_var.get()), 4),
            round(float(self.tool_offset_rz_var.get()), 4),
            round(float(self.dobot_tool_offset_x_var.get()), 4),
            round(float(self.dobot_tool_offset_y_var.get()), 4),
            round(float(self.dobot_tool_offset_z_var.get()), 4),
            round(float(self.dobot_tool_offset_rx_var.get()), 4),
            round(float(self.dobot_tool_offset_ry_var.get()), 4),
            round(float(self.dobot_tool_offset_rz_var.get()), 4),
        )

    def _has_unsaved_tool_offset_changes(self) -> bool:
        if self._saved_tool_teach_values is None:
            return True
        return self._current_tool_offset_signature() != self._saved_tool_teach_values

    def _arm_block_reason(self) -> str:
        if self._active_item_id is None:
            return (
                'No active item teach in items/. Drop a single yaml there and restart item_pick.'
            )
        if not self._active_profile_has_saved_tool_teach or self._saved_tool_teach_values is None:
            if self.node._active_profile_pick_error:
                return (
                    f'Invalid pick profile for "{self._profile_display_name()}": '
                    f'{self.node._active_profile_pick_error}'
                )
            return (
                'No saved tool teach for '
                f'"{self._profile_display_name()}". Save it before arming.'
            )
        if self._has_unsaved_tool_offset_changes():
            return (
                'Tool teach changes for '
                f'"{self._profile_display_name()}" are not saved. '
                'Save them before arming.'
            )
        return ''

    def _can_arm_sequence(self) -> bool:
        return not bool(self._arm_block_reason())

    def _update_profile_tool_offset_status(self) -> None:
        profile_name = self._profile_display_name()
        self.active_profile_var.set(f'Active item teach: {profile_name}')
        if self._active_item_id is None:
            self.tool_offset_profile_status_var.set(
                f'No yaml in items/ (waiting for CATARM swap into {self.node._items_dir}).'
            )
            return
        if not self._active_profile_has_saved_tool_teach or self._saved_tool_teach_values is None:
            profile_error = self.node._active_profile_pick_error
            if profile_error:
                self.tool_offset_profile_status_var.set(f'Invalid pick profile: {profile_error}')
            else:
                self.tool_offset_profile_status_var.set(
                    'No pick:/ros__parameters: section in active item yaml. '
                    'Current UI values are unsaved.'
                )
            return
        if self._has_unsaved_tool_offset_changes():
            self.tool_offset_profile_status_var.set(
                'Saved tool teach loaded, but current UI values have unsaved changes.'
            )
            return
        self.tool_offset_profile_status_var.set(
            'Saved tool teach loaded and ready for arming.'
        )

    def _register_runtime_setting_traces(self) -> None:
        tracked_vars = [
            self.tf_only_var,
            self.item_pose_watch_timeout_var,
            self.use_fingers_var,
            self.grab_on_pick_var,
            self.relax_fingers_on_pick_var,
            self.post_stop_z_offset_var,
            self.final_z_up_var,
            self.pre_pick_settling_time_var,
            self.pick_settling_time_var,
            self.tool_offset_x_var,
            self.tool_offset_y_var,
            self.tool_offset_z_var,
            self.tool_offset_rx_var,
            self.tool_offset_ry_var,
            self.tool_offset_rz_var,
            self.dobot_tool_offset_x_var,
            self.dobot_tool_offset_y_var,
            self.dobot_tool_offset_z_var,
            self.dobot_tool_offset_rx_var,
            self.dobot_tool_offset_ry_var,
            self.dobot_tool_offset_rz_var,
        ]
        for var in tracked_vars:
            var.trace_add('write', self._on_runtime_setting_changed)

    def _sync_profile_tool_offsets_from_state(self, force: bool = False) -> None:
        item_id, profile_offsets = self.node.get_active_profile_tool_offset_state(force=force)
        profile_signature = self._tool_offset_signature_from_profile(profile_offsets)
        profile_changed = item_id != self._active_item_id
        saved_changed = profile_signature != self._saved_tool_teach_values
        self._active_item_id = item_id
        self._active_profile_has_saved_tool_teach = profile_offsets is not None
        if profile_offsets is not None and (force or profile_changed or saved_changed):
            self._suspend_runtime_settings_events = True
            try:
                self.use_fingers_var.set(bool(profile_offsets['use_fingers']))
                self.grab_on_pick_var.set(bool(profile_offsets['grab_on_pick']))
                self.relax_fingers_on_pick_var.set(bool(profile_offsets.get(
                    'relax_fingers_on_pick',
                    RELAX_FINGERS_ON_PICK_DEFAULT,
                )))
                self.post_stop_z_offset_var.set(self._clamp(
                    profile_offsets.get('item_standoff_z_mm', self.post_stop_z_offset_var.get()),
                    POST_STOP_Z_OFFSET_MIN,
                    POST_STOP_Z_OFFSET_MAX,
                ))
                self.final_z_up_var.set(self._clamp(
                    profile_offsets.get('final_z_up_mm', self.final_z_up_var.get()),
                    FINAL_Z_UP_MIN,
                    FINAL_Z_UP_MAX,
                ))
                self.pre_pick_settling_time_var.set(self._clamp(
                    profile_offsets.get('pre_pick_settling_time_sec', self.pre_pick_settling_time_var.get()),
                    SETTLING_TIME_MIN_SEC,
                    SETTLING_TIME_MAX_SEC,
                ))
                self.pick_settling_time_var.set(self._clamp(
                    profile_offsets.get('pick_settling_time_sec', self.pick_settling_time_var.get()),
                    SETTLING_TIME_MIN_SEC,
                    SETTLING_TIME_MAX_SEC,
                ))
                self.tool_offset_x_var.set(self._clamp(
                    profile_offsets.get('tool_offset_x_mm', self.tool_offset_x_var.get()),
                    TOOL_OFFSET_TRANSLATION_MIN_MM,
                    TOOL_OFFSET_TRANSLATION_MAX_MM,
                ))
                self.tool_offset_y_var.set(self._clamp(
                    profile_offsets.get('tool_offset_y_mm', self.tool_offset_y_var.get()),
                    TOOL_OFFSET_TRANSLATION_MIN_MM,
                    TOOL_OFFSET_TRANSLATION_MAX_MM,
                ))
                self.tool_offset_z_var.set(self._clamp(
                    profile_offsets.get('tool_offset_z_mm', self.tool_offset_z_var.get()),
                    TOOL_OFFSET_TRANSLATION_MIN_MM,
                    TOOL_OFFSET_TRANSLATION_MAX_MM,
                ))
                self.tool_offset_rx_var.set(self._clamp(
                    profile_offsets.get('tool_offset_rx_deg', self.tool_offset_rx_var.get()),
                    TOOL_OFFSET_ROTATION_MIN_DEG,
                    TOOL_OFFSET_ROTATION_MAX_DEG,
                ))
                self.tool_offset_ry_var.set(self._clamp(
                    profile_offsets.get('tool_offset_ry_deg', self.tool_offset_ry_var.get()),
                    TOOL_OFFSET_ROTATION_MIN_DEG,
                    TOOL_OFFSET_ROTATION_MAX_DEG,
                ))
                self.tool_offset_rz_var.set(self._clamp(
                    profile_offsets.get('tool_offset_rz_deg', self.tool_offset_rz_var.get()),
                    TOOL_OFFSET_ROTATION_MIN_DEG,
                    TOOL_OFFSET_ROTATION_MAX_DEG,
                ))
                self.dobot_tool_offset_x_var.set(self._clamp(
                    profile_offsets.get('dobot_tool_offset_x_mm', self.dobot_tool_offset_x_var.get()),
                    TOOL_OFFSET_TRANSLATION_MIN_MM,
                    TOOL_OFFSET_TRANSLATION_MAX_MM,
                ))
                self.dobot_tool_offset_y_var.set(self._clamp(
                    profile_offsets.get('dobot_tool_offset_y_mm', self.dobot_tool_offset_y_var.get()),
                    TOOL_OFFSET_TRANSLATION_MIN_MM,
                    TOOL_OFFSET_TRANSLATION_MAX_MM,
                ))
                self.dobot_tool_offset_z_var.set(self._clamp(
                    profile_offsets.get('dobot_tool_offset_z_mm', self.dobot_tool_offset_z_var.get()),
                    TOOL_OFFSET_TRANSLATION_MIN_MM,
                    TOOL_OFFSET_TRANSLATION_MAX_MM,
                ))
                self.dobot_tool_offset_rx_var.set(self._clamp(
                    profile_offsets.get('dobot_tool_offset_rx_deg', self.dobot_tool_offset_rx_var.get()),
                    TOOL_OFFSET_ROTATION_MIN_DEG,
                    TOOL_OFFSET_ROTATION_MAX_DEG,
                ))
                self.dobot_tool_offset_ry_var.set(self._clamp(
                    profile_offsets.get('dobot_tool_offset_ry_deg', self.dobot_tool_offset_ry_var.get()),
                    TOOL_OFFSET_ROTATION_MIN_DEG,
                    TOOL_OFFSET_ROTATION_MAX_DEG,
                ))
                self.dobot_tool_offset_rz_var.set(self._clamp(
                    profile_offsets.get('dobot_tool_offset_rz_deg', self.dobot_tool_offset_rz_var.get()),
                    TOOL_OFFSET_ROTATION_MIN_DEG,
                    TOOL_OFFSET_ROTATION_MAX_DEG,
                ))
            finally:
                self._suspend_runtime_settings_events = False
        self._saved_tool_teach_values = profile_signature
        self._update_profile_tool_offset_status()

    def _collect_pick_section_params(self) -> dict:
        """Build the ``pick:/ros__parameters:`` mapping from current UI vars.

        Carries both the runtime UI state (tf_only, watch timeout, pick
        timings) and the tool teach offsets (operator-tunable
        ``tool_offset_*``) plus the per-item Dobot internal tool TCP
        (``dobot_tool_offset_*``) that the flange-tcp debug TF and the
        controller's ``tool=N`` argument depend on. Optional manually
        entered item-size filter fields are round-tripped unchanged.
        """
        params = {
            'tf_only_mode': bool(self.tf_only_var.get()),
            'use_fingers': bool(self.use_fingers_var.get()),
            'grab_on_pick': bool(self.grab_on_pick_var.get()),
            'relax_fingers_on_pick': bool(self.relax_fingers_on_pick_var.get()),
            'item_pose_wait_timeout_sec': float(self.item_pose_watch_timeout_var.get()),
            'item_standoff_z_mm': float(self.post_stop_z_offset_var.get()),
            'tray_place_additional_z_mm': TRAY_PLACE_ADDITIONAL_Z_MM_DEFAULT,
            'final_z_up_mm': float(self.final_z_up_var.get()),
            'pre_pick_settling_time_sec': float(self.pre_pick_settling_time_var.get()),
            'pick_settling_time_sec': float(self.pick_settling_time_var.get()),
            'tool_offset_x_mm': float(self.tool_offset_x_var.get()),
            'tool_offset_y_mm': float(self.tool_offset_y_var.get()),
            'tool_offset_z_mm': float(self.tool_offset_z_var.get()),
            'tool_offset_rx_deg': float(self.tool_offset_rx_var.get()),
            'tool_offset_ry_deg': float(self.tool_offset_ry_var.get()),
            'tool_offset_rz_deg': float(self.tool_offset_rz_var.get()),
            'dobot_tool_offset_x_mm': float(self.dobot_tool_offset_x_var.get()),
            'dobot_tool_offset_y_mm': float(self.dobot_tool_offset_y_var.get()),
            'dobot_tool_offset_z_mm': float(self.dobot_tool_offset_z_var.get()),
            'dobot_tool_offset_rx_deg': float(self.dobot_tool_offset_rx_var.get()),
            'dobot_tool_offset_ry_deg': float(self.dobot_tool_offset_ry_var.get()),
            'dobot_tool_offset_rz_deg': float(self.dobot_tool_offset_rz_var.get()),
        }
        _, profile_offsets = self.node.get_active_profile_tool_offset_state()
        if profile_offsets is not None:
            for key in (
                'tray_place_additional_z_mm',
                'item_length',
                'item_width',
                'size_tolerance',
            ):
                if key in profile_offsets:
                    params[key] = profile_offsets[key]
        return params

    def _write_pick_section_to_active_item(self) -> bool:
        """Persist the pick:/ros__parameters: section atomically.

        Returns False when there is no active item yaml so callers can
        surface a clear action-text. Sibling sections (``item:``,
        ``teach:``, ``detect:``) are round-tripped by the helper so the
        teach/detect writers' state is never stomped here.
        """
        item_file, item_id, display_name = self.node.get_active_item_metadata()
        if item_file is None or not item_id:
            return False
        params = self._collect_pick_section_params()
        try:
            write_subject_section(
                item_file,
                PICK_SECTION_KEY,
                {'ros__parameters': params},
                subject_kind=ITEM_SUBJECT_KIND,
                subject_id=item_id,
                display_name=display_name,
            )
        except (TeachLayoutError, TeachSchemaError) as exc:
            self.node.get_logger().warn(
                f'Refused to write pick: section to "{item_file}": {exc}'
            )
            return False
        except OSError as exc:
            self.node.get_logger().warn(
                f'Failed to write pick: section to "{item_file}": {exc}'
            )
            return False
        return True

    def _save_profile_tool_offsets_for_active_item(self) -> bool:
        if self._active_item_id is None:
            message = (
                f'No item yaml in items/. Drop one in '
                f'{self.node._items_dir} and restart item_pick.'
            )
            self.node._set_action_text(message)
            self.action_var.set(message)
            return False

        if not self._write_pick_section_to_active_item():
            message = (
                f'Failed to save tool teach for "{self._profile_display_name()}". '
                'See logs.'
            )
            self.node._set_action_text(message)
            self.action_var.set(message)
            return False
        self.node.get_active_profile_tool_offset_state(force=True)
        self._sync_profile_tool_offsets_from_state(force=True)
        return True

    def _schedule_runtime_settings_save(self) -> None:
        if self._runtime_settings_save_after_id is not None:
            self.root.after_cancel(self._runtime_settings_save_after_id)
            self._runtime_settings_save_after_id = None
        self._runtime_settings_save_after_id = self.root.after(
            RUNTIME_SETTINGS_SAVE_DEBOUNCE_MS,
            self._save_runtime_settings,
        )

    def _on_runtime_setting_changed(self, *_args) -> None:
        if self._suspend_runtime_settings_events:
            return
        snapshot = self.node.snapshot()
        ui_locked = bool(snapshot.armed or snapshot.busy or self.node.is_manual_stop_inflight())
        self._sync_tf_only_button(is_busy=ui_locked)
        self._set_arm_locked_setting_controls_enabled(not ui_locked)
        self._update_profile_tool_offset_status()
        # Auto-save is intentionally disabled here: the pick: section
        # write must be an explicit operator action because it touches
        # the shared items yaml that CATARM may swap underneath us.
        # ``Save Tool Teach`` performs the section write; the debounce
        # plumbing is kept so a future deploy can re-enable autosave
        # without re-introducing a separate runtime-settings file.

    def _load_runtime_settings(self) -> None:
        """Pull tf_only + watch timeout out of the active items yaml.

        Tool offsets are loaded by ``_sync_profile_tool_offsets_from_state``;
        this hook covers the two extra UI vars (``tf_only_mode``,
        ``item_pose_wait_timeout_sec``) so the legacy
        ``item_pick_runtime_settings.json`` is gone end-to-end.
        """
        _, profile_offsets = self.node.get_active_profile_tool_offset_state(force=True)
        if profile_offsets is None:
            return
        self._suspend_runtime_settings_events = True
        try:
            if 'tf_only_mode' in profile_offsets:
                self.tf_only_var.set(bool(profile_offsets.get('tf_only_mode', False)))
            if 'item_pose_wait_timeout_sec' in profile_offsets:
                self.item_pose_watch_timeout_var.set(self._clamp(
                    profile_offsets.get(
                        'item_pose_wait_timeout_sec',
                        ITEM_POSE_WATCH_TIMEOUT_SEC,
                    ),
                    ITEM_POSE_WATCH_TIMEOUT_MIN,
                    ITEM_POSE_WATCH_TIMEOUT_MAX,
                ))
        finally:
            self._suspend_runtime_settings_events = False

    def _save_runtime_settings(self) -> None:
        # Kept as a no-op so the legacy after-timer call paths stay
        # green; the consolidated layout requires explicit
        # ``Save Tool Teach`` actions instead of debounced background
        # writes. Resetting the after-id mirrors the original behaviour
        # so ``_schedule_runtime_settings_save`` stays cycle-safe.
        self._runtime_settings_save_after_id = None

    def _set_arm_locked_setting_controls_enabled(self, enabled: bool) -> None:
        state = tk.NORMAL if enabled else tk.DISABLED
        for control in self._arm_locked_setting_controls:
            try:
                control.configure(state=state)
            except tk.TclError:
                pass
        self.grab_on_pick_check.configure(
            state=(
                tk.NORMAL
                if enabled and bool(self.use_fingers_var.get())
                else tk.DISABLED
            )
        )

    def _refresh(self) -> None:
        snapshot = self.node.snapshot()
        stop_inflight = self.node.is_manual_stop_inflight()
        release_inflight = self.node.is_manual_release_inflight()
        ui_locked = bool(snapshot.armed or snapshot.busy or stop_inflight)
        if not ui_locked:
            self._sync_profile_tool_offsets_from_state()
        self._sync_tf_only_button(is_busy=ui_locked)
        self._set_arm_locked_setting_controls_enabled(not ui_locked)
        can_arm = self._can_arm_sequence()
        can_save_tool_offset = (
            self._active_item_id is not None
            and not ui_locked
        )

        self.action_var.set(snapshot.action_text)
        self.run_button.configure(
            state=tk.NORMAL if (not ui_locked and can_arm) else tk.DISABLED
        )
        self._set_stop_button_enabled(bool(snapshot.busy) and not stop_inflight)
        self.release_button.configure(
            state=tk.DISABLED if (ui_locked or release_inflight) else tk.NORMAL
        )
        self.show_tool_tf_button.configure(
            state=tk.DISABLED if ui_locked else tk.NORMAL
        )
        self.save_tool_offset_button.configure(
            state=tk.NORMAL if can_save_tool_offset else tk.DISABLED
        )

        if not self._closed:
            self.root.after(100, self._refresh)

    def _set_stop_button_enabled(self, enabled: bool) -> None:
        if enabled:
            self.stop_button.configure(
                state=tk.NORMAL,
                bg='#d32f2f',
                fg='white',
                activebackground='#b71c1c',
                activeforeground='white',
            )
            return
        self.stop_button.configure(
            state=tk.DISABLED,
            bg=self._stop_default_bg,
            fg=self._stop_default_fg,
            activebackground=self._stop_default_active_bg,
            activeforeground=self._stop_default_active_fg,
        )

    def _sync_tf_only_button(self, is_busy: bool) -> None:
        tf_only_enabled = bool(self.tf_only_var.get())
        if tf_only_enabled:
            label = 'Troubleshoot TF-only: ON'
            bg = '#ef6c00'
            fg = 'white'
            active_bg = '#e65100'
            active_fg = 'white'
        else:
            label = 'Troubleshoot TF-only: OFF'
            bg = self._tf_only_default_bg
            fg = self._tf_only_default_fg
            active_bg = self._tf_only_default_active_bg
            active_fg = self._tf_only_default_active_fg

        self.tf_only_button.configure(
            text=label,
            state=tk.DISABLED if is_busy else tk.NORMAL,
            bg=bg,
            fg=fg,
            activebackground=active_bg,
            activeforeground=active_fg,
        )

    def _on_close(self) -> None:
        if self._runtime_settings_save_after_id is not None:
            self.root.after_cancel(self._runtime_settings_save_after_id)
            self._runtime_settings_save_after_id = None
        self._closed = True
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ItemPickNode()
    # MultiThreadedExecutor (not SingleThreaded) so the drop_last_item
    # service callback can block on its held-item return/retract motion while the
    # executor keeps resolving the MovJ/DO client responses that the
    # motion depends on. With a single-threaded executor that wait would
    # deadlock; the drop service lives in a ReentrantCallbackGroup so it
    # never blocks the default group used by the motion client responses.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    stop_event = threading.Event()

    def spin() -> None:
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    gui = ItemPickGui(node)
    try:
        gui.run()
    finally:
        stop_event.set()
        spin_thread.join(timeout=1.0)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
