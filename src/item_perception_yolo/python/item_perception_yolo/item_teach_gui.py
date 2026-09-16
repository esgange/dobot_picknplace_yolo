"""Read-only item-profile editor, detector and teaching pose preview."""

import copy
import math
import os
from pathlib import Path
import signal
import threading
import time
import queue

import numpy as np
from geometry_msgs.msg import TransformStamped
from python_qt_binding import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from .item_teach_core import (
    GRIPPER_FIELDS, MODEL_TASKS, MOTION_FIELDS,
    item_directory, load_item_profile, record_home, save_item_profile,
    settings_from_profile, item_save_target, file_sha256,
    GEOMETRY_FIELDS, DEFAULT_PICKDEPTH_DIAMETER_MM, QUALITY_DEFAULTS,
    NEW_PROFILE_IMAGE_SIZE, SPEED_FIELDS, NEW_PROFILE_SPEED, NEW_PROFILE_ACCELERATION,
)
from .platform_teach_core import (
    _parse_env_file, load_robot_lan1_ip, ui_state_path, workspace_root,
    rotation_matrix_to_quaternion,
)
from .ui_state import load_package_ui_state, write_item_ui_state, write_item_preview_state
from .item_preview import validate_prefix
from .item_detector import ItemDetectNode, INITIAL_PREVIEW_YOLO, transform_matrix
from .ui_state import write_item_station_state
from .item_teach_recovery import recover_item_fields
from .bin_teach_core import bin_platform_warning
from .station_calibration import latest_station_calibration


class DetectionImage(QtWidgets.QLabel):
    clicked = QtCore.Signal(QtCore.QPointF)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.clicked.emit(QtCore.QPointF(event.pos()))
        super().mousePressEvent(event)


def image_click(point, label, width, height):
    """Map a click through the actual centered, letterboxed pixmap; reject margins."""
    pixmap = label.pixmap()
    if pixmap is None or pixmap.isNull():
        return None
    area = label.contentsRect()
    left = area.x() + (area.width() - pixmap.width()) / 2
    top = area.y() + (area.height() - pixmap.height()) / 2
    x, y = point.x() - left, point.y() - top
    if not 0 <= x < pixmap.width() or not 0 <= y < pixmap.height():
        return None
    return QtCore.QPointF(x * width / pixmap.width(), y * height / pixmap.height())


def draw_sampling_circle(painter, points):
    """Native-projected source pixels are scaled/letterboxed with the RGB image."""
    if points is not None:
        painter.save()
        painter.setPen(QtGui.QPen(QtGui.QColor("cyan"), 3))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(*p) for p in points]))
        painter.restore()


class ItemTeachNode(ItemDetectNode):
    def __init__(self):
        super().__init__("item_teach")
        self.robot_ip = load_robot_lan1_ip()
        values = _parse_env_file(workspace_root() / ".env")
        name = values["DOBOT_ROBOT_NODE_NAME"]
        if not name or "/" in name:
            raise ValueError("DOBOT_ROBOT_NODE_NAME must name one root-namespace node")
        self.publisher_node = "/" + name
        self._joints = None
        self._receipt = None
        self.selection_lock = threading.RLock()
        self.selected_pose = None
        self.selected_pose_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.1, self._broadcast_selected_pose)
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, qos_profile_sensor_data,
        )
        self.events.record("INFO", "node_started", "Item editor started; no command clients")

    def clear_selected_pose(self):
        """Clear both clicked and simulated teaching previews; never retain old batch frames."""
        with self.selection_lock:
            self.selected_pose = None

    def show_selected_pose(self, candidate, stamp_ns, epoch):
        self._validate_sources()
        if epoch != self.arm_epoch or not self.yolo_enabled or self.native.failed:
            raise ValueError("Selected pose invalidated before TF publication")
        transform = build_selected_pose_transform(
            self.applied.platform.base_from_platform, candidate, self.get_clock().now().to_msg())
        with self.selection_lock:
            if epoch != self.arm_epoch or not self.yolo_enabled or self.native.failed:
                raise ValueError("Selected pose invalidated before TF publication")
            self.selected_pose = (epoch, (transform,), None)
        self.events.record("INFO", "item_teach_pose_selected",
                           "Frozen teaching-only TF; not a robot target or service response",
                           frame=transform.child_frame_id, source_stamp_ns=stamp_ns,
                           candidate=candidate)

    def show_simulated_poses(self, response, view):
        """Install only the successful returned batch, not raw detections."""
        self.clear_selected_pose()
        self.validate_simulation_view(view)
        epoch = view["simulation_epoch"]
        stamp_ns = response.header.stamp.sec * 1_000_000_000 + response.header.stamp.nanosec
        now = self.get_clock().now()
        if stamp_ns != view["stamp_ns"] or stamp_ns <= 0 or stamp_ns > now.nanoseconds:
            raise ValueError("Simulated snapshot timestamp is invalid or mismatched")
        transforms = build_simulated_pose_transforms(
            self.applied.platform.base_from_platform, response, now.to_msg())
        # Keep only identity evidence in the timer state, never another image/depth snapshot.
        binding = {"simulation_epoch": epoch,
                   "simulation_profile": tuple(view["simulation_profile"])}
        with self.selection_lock:
            if (epoch != self.arm_epoch or not self.yolo_enabled
                    or self.native.failed or self.fatal_error):
                raise ValueError("Simulated batch invalidated before TF publication")
            if transforms:
                self.selected_pose = (epoch, transforms, binding)
        self.events.record(
            "INFO", "item_teach_batch_tf", "Frozen simulated teaching TFs; no robot commands",
            batch_id=response.batch_id, source_stamp_ns=stamp_ns,
            frames=[transform.child_frame_id for transform in transforms],
            candidate_ids=[candidate.id for candidate in response.candidates])

    def _broadcast_selected_pose(self):
        with self.selection_lock:
            if self.selected_pose is None:
                return
            epoch, transforms, binding = self.selected_pose
            if (epoch != self.arm_epoch or not self.yolo_enabled
                    or self.native.failed or self.fatal_error):
                self.selected_pose = None
                return
            try:
                if binding is None:
                    self._validate_sources()
                else:
                    self.validate_simulation_view(binding)
            except (ValueError, OSError) as exc:
                self.selected_pose = None
                self.events.record("WARNING", "item_teach_pose_cleared", str(exc))
                return
            if epoch != self.arm_epoch or not self.yolo_enabled or self.native.failed:
                self.selected_pose = None
                return
            stamp = self.get_clock().now().to_msg()
            for transform in transforms:
                transform.header.stamp = stamp
            self.selected_pose_broadcaster.sendTransform(list(transforms))

    def close_runtime(self):
        self.clear_selected_pose()
        super().close_runtime()

    def _on_joints(self, message):
        with self._feedback_lock:
            self._joints = message
            self._receipt = time.monotonic()

    def capture_home(self):
        endpoints = self.get_publishers_info_by_topic("/joint_states")
        publishers = [
            (entry.node_namespace.rstrip("/") + "/" + entry.node_name) for entry in endpoints
        ]
        if publishers != [self.publisher_node]:
            raise ValueError(
                f"Home requires sole publisher {self.publisher_node}; found {publishers}"
            )
        with self._feedback_lock:
            message, receipt = self._joints, self._receipt
        if message is None or receipt is None or time.monotonic() - receipt > 1.0:
            raise ValueError("No fresh /joint_states feedback; home was not captured")
        home = record_home(
            list(message.name), list(message.position), int(message.header.stamp.sec),
            int(message.header.stamp.nanosec), now_ns=self.get_clock().now().nanoseconds,
            robot_ip=self.robot_ip, publisher=self.publisher_node,
        )
        self.events.record("INFO", "home_feedback_read", "Read six actual home joints", home=home)
        return home


def build_selected_pose_transform(base_from_platform, candidate, stamp,
                                  child_frame_id="item_teach_selected_item"):
    """Publish directly under base to avoid competing platform_reference authorities."""
    transform = TransformStamped()
    transform.transform.translation.x, transform.transform.translation.y, \
        transform.transform.translation.z = map(float, candidate["position"])
    q = transform.transform.rotation
    q.x, q.y, q.z, q.w = map(float, candidate["quaternion"])
    matrix = np.asarray(base_from_platform) @ transform_matrix(transform)
    transform.header.frame_id = "base_link"
    transform.header.stamp = stamp
    transform.child_frame_id = child_frame_id
    t = transform.transform.translation
    t.x, t.y, t.z = map(float, matrix[:3, 3])
    q.x, q.y, q.z, q.w = rotation_matrix_to_quaternion(matrix[:3, :3])
    return transform


def build_simulated_pose_transforms(base_from_platform, response, stamp):
    """P1..Pn are response priorities, composed exactly like the clicked-item preview."""
    if not response.success or response.header.frame_id != "platform_reference":
        raise ValueError("Simulated teaching TF requires a successful platform-relative response")
    transforms, seen = [], set()
    for priority, candidate in enumerate(response.candidates, 1):
        if candidate.priority != priority or not candidate.id or candidate.id in seen:
            raise ValueError("Simulated teaching TF requires distinct, priority-ordered candidates")
        seen.add(candidate.id)
        point, rotation = candidate.pose.position, candidate.pose.orientation
        transform = build_selected_pose_transform(
            base_from_platform,
            {"position": [point.x, point.y, point.z],
             "quaternion": [rotation.x, rotation.y, rotation.z, rotation.w]},
            stamp, child_frame_id=f"item_teach_candidate_{priority}")
        transforms.append(transform)
    return tuple(transforms)


class ItemTeachWindow(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.home = None
        self.saved_path = None
        self.save_target = None
        self.recovered_draft = False
        self.profile_image_size = NEW_PROFILE_IMAGE_SIZE
        self.job_results = queue.Queue(maxsize=1)
        self.job_busy = False
        self.model_load_reserved = False
        self.pending_model_path = None
        self.model_requested_pair = None
        self.closing = False
        self.preview_revision = self.job_revision = 0
        self.preview_update_due = None
        self.preview_settings_paused = False
        self.pending_pose = None
        self.pending_simulation = None
        self.simulation_busy = False
        self.simulation_response = None
        self.selected_pose_result = None
        self.selected_pose_status = ""
        self.last_preview_sequence = None
        self.preview_status = "YOLO OFF"
        self.preview_error = ""
        self.displayed_view = self.frozen_view = self.selected_detection = None
        self.setWindowTitle("Item Teach — visual pose inspection (no motion)")
        self.resize(1560, 960)
        self.setStyleSheet("""
            QGroupBox { font-weight: 600; border: 1px solid #cbd2da;
                        border-radius: 6px; margin-top: 12px; padding: 12px 8px 8px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QLineEdit, QComboBox { min-height: 24px; }
            QPushButton { min-height: 28px; padding: 2px 8px; }
            QPushButton:checked { background: #d9eafa; color: #123b60;
                                  border: 1px solid #4783b5; border-radius: 4px; }
            QSplitter::handle { background: #cbd2da; }
            QLabel#viewHeading { color: #e1e7ed; background: #202a35;
                                  padding: 7px 10px; font-weight: 600; }
        """)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(12, 10, 12, 10)
        outer.setSpacing(8)
        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("Item Teach")
        title.setFont(QtGui.QFont("Sans", 18, QtGui.QFont.Bold))
        header.addWidget(title)
        notice = QtWidgets.QLabel(
            "Visual inspection & teach files\nRead-only poses · No robot motion"
        )
        header.addWidget(notice)
        header.addStretch(1)
        load = self.load_teach_button = QtWidgets.QPushButton("Load Item Teach…")
        save = self.save_button = QtWidgets.QPushButton("Save Item Teach…")
        save.setToolTip("Update the loaded pair; change Item name to create a new pair.")
        load.clicked.connect(self._load_dialog)
        save.clicked.connect(self._save)
        header.addWidget(load)
        header.addWidget(save)
        outer.addLayout(header)
        self.recovery_notice = QtWidgets.QLabel()
        self.recovery_notice.setWordWrap(True)
        self.recovery_notice.setTextFormat(QtCore.Qt.PlainText)
        self.recovery_notice.hide()
        outer.addWidget(self.recovery_notice)
        scroll = self.settings_scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setMinimumWidth(340)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        form_host = QtWidgets.QWidget()
        form_column = self.settings_column = QtWidgets.QVBoxLayout(form_host)
        form_column.setContentsMargins(0, 0, 8, 0)
        form_column.setSpacing(10)
        form_column.setAlignment(QtCore.Qt.AlignTop)
        sections = {}

        def group(title, order):
            box = QtWidgets.QGroupBox(title)
            layout = QtWidgets.QFormLayout(box)
            layout.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
            layout.setRowWrapPolicy(QtWidgets.QFormLayout.WrapLongRows)
            layout.setVerticalSpacing(7)
            sections[order] = box
            return layout

        station = group("1  Camera / station", 0)
        scroll.setWidget(form_host)
        split = self.workspace_split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        split.setChildrenCollapsible(False)
        split.setHandleWidth(6)
        split.addWidget(scroll)
        view_host = QtWidgets.QWidget()
        view_layout = QtWidgets.QVBoxLayout(view_host)
        view_layout.setContentsMargins(4, 0, 0, 0)
        view_layout.setSpacing(6)
        camera_row = QtWidgets.QHBoxLayout()
        self.camera_prefix = QtWidgets.QLineEdit()
        self.camera_prefix.setPlaceholderText("Camera prefix, e.g. bin_camera")
        connect_camera = QtWidgets.QPushButton("Connect RGB")
        connect_camera.clicked.connect(self._connect_camera)
        camera_row.addWidget(self.camera_prefix)
        camera_row.addWidget(connect_camera)
        station.addRow(camera_row)
        self.platform_path = QtWidgets.QLineEdit()
        self.calibration_camera_path = QtWidgets.QLineEdit()
        self.bin_path = QtWidgets.QLineEdit()
        for label, field in (("Latest platform", self.platform_path),
                             ("Camera calibration", self.calibration_camera_path)):
            field.setReadOnly(True)
            field.setPlaceholderText("Automatically selected from calibration/")
            station.addRow(label, field)
        reload_station = QtWidgets.QPushButton("Reload Latest Calibration")
        reload_station.clicked.connect(self._update_station_preview)
        station.addRow(reload_station)
        station_fields = (("Bin teach", self.bin_path,
                           workspace_root() / "offline_teach/bin_teach"),)
        for label, field, directory in station_fields:
            row = QtWidgets.QHBoxLayout()
            field.setReadOnly(True)
            field.setPlaceholderText(label + " YAML")
            choose = QtWidgets.QPushButton(label + "…")
            choose.clicked.connect(lambda _checked=False, f=field, d=directory:
                                   self._choose_station_file(f, d))
            row.addWidget(field)
            row.addWidget(choose)
            station.addRow(row)
        self.bin_platform_warning = QtWidgets.QLabel()
        self.bin_platform_warning.setTextFormat(QtCore.Qt.PlainText)
        self.bin_platform_warning.setWordWrap(True)
        self.bin_platform_warning.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.bin_platform_warning.setSizePolicy(
            QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        self.bin_platform_warning.setStyleSheet(
            "background-color: #fff3cd; color: #664d03; border: 1px solid #e0b74a; "
            "border-radius: 4px; padding: 6px;")
        self.bin_platform_warning.hide()
        station.addRow(self.bin_platform_warning)
        self.station_status = QtWidgets.QLabel(
            "Station calibration loads automatically; select a bin teach for its ROI."
        )
        self.station_status.setWordWrap(True)
        station.addRow(self.station_status)
        toggle_row = QtWidgets.QHBoxLayout()
        self.yolo_toggle = QtWidgets.QPushButton("YOLO Detect: OFF")
        self.yolo_toggle.setCheckable(True)
        self.yolo_toggle.toggled.connect(self._toggle_yolo)
        toggle_row.addWidget(self.yolo_toggle)
        self.simulate_button = QtWidgets.QPushButton("Simulate Trigger")
        self.simulate_button.setToolTip(
            "Freeze a new RGB/depth pair with only the ranked service candidates. "
            "Requires a saved profile and YOLO ON; Armed may be OFF. No robot commands.")
        self.simulate_button.clicked.connect(self._simulate_trigger)
        toggle_row.addWidget(self.simulate_button)
        self.armed_toggle = QtWidgets.QPushButton("Armed: OFF")
        self.armed_toggle.setCheckable(True)
        self.armed_toggle.setToolTip(
            "Requires saved profile, loaded model, YOLO ON and applied station with fresh inputs."
        )
        self.armed_toggle.toggled.connect(self._style_armed)
        self.armed_toggle.toggled.connect(self._toggle_armed)
        toggle_row.addWidget(self.armed_toggle)
        view_layout.addLayout(toggle_row)
        self.preview_help = QtWidgets.QLabel(
            "All model detections; live confidence, IoU and detection cap.\n"
            "Size border: GREEN within tolerance / RED outside / GRAY not checked.\n"
            "Click an item for RGB/depth pose + teaching TF; click image again to resume."
        )
        self.preview_help.setWordWrap(True)
        self.preview_help.hide()  # Details live inside each image pane, not a duplicate panel.
        self.video = DetectionImage("Connect RGB to see the camera")
        self.video.clicked.connect(self._select_detection)
        self.video.setAlignment(QtCore.Qt.AlignCenter)
        self.video.setMinimumSize(400, 200)
        self.video.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
        self.video.setStyleSheet("background: #111; color: white;")
        images = self.image_split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        images.setChildrenCollapsible(False)
        images.setHandleWidth(6)

        def image_panel(title, image):
            panel = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(panel)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(0)
            heading = QtWidgets.QLabel(title)
            heading.setObjectName("viewHeading")
            layout.addWidget(heading)
            feedback = QtWidgets.QLabel()
            feedback.setTextFormat(QtCore.Qt.PlainText)
            feedback.setWordWrap(True)
            feedback.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)
            feedback.setStyleSheet(
                "background: #111; color: white; padding: 8px 12px; font-weight: bold;")
            feedback.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Minimum)
            feedback.hide()
            layout.addWidget(feedback)
            layout.addWidget(image, 1)
            images.addWidget(panel)
            return feedback

        self.rgb_feedback = image_panel(
            "RGB  /  Bin ROI · item size · click to inspect pose", self.video)
        self.depth_video = QtWidgets.QLabel("Depth sampling: select station files and enable YOLO")
        self.depth_video.setAlignment(QtCore.Qt.AlignCenter)
        self.depth_video.setMinimumSize(400, 140)
        self.depth_video.setWordWrap(True)
        self.depth_video.setStyleSheet("background: #171e26; color: #e1e7ed; padding: 4px;")
        self.depth_video.setSizePolicy(QtWidgets.QSizePolicy.Ignored,
                                       QtWidgets.QSizePolicy.Expanding)
        self.depth_feedback = image_panel(
            "DEPTH  /  Native pixels · accepted BLACK · rejected RED", self.depth_video)
        images.setStretchFactor(0, 1)
        images.setStretchFactor(1, 1)
        images.setSizes([560, 560])
        view_layout.addWidget(images, 1)
        self.video_status = QtWidgets.QLabel("YOLO OFF — raw RGB")
        self.video_status.setWordWrap(True)
        view_layout.addWidget(self.video_status)
        split.addWidget(view_host)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([390, 1170])
        outer.addWidget(split, 1)
        self.inputs = {}

        identity = group("2  Item / model", 1)
        self.name = QtWidgets.QLineEdit()
        self.name.setPlaceholderText("Item name")
        self.model = QtWidgets.QLineEdit()
        self.model.setReadOnly(True)
        self.model.setPlaceholderText("Select .pt anywhere on this PC")
        browse = QtWidgets.QPushButton("Browse .pt…")
        browse.clicked.connect(self._browse_model)
        model_row = QtWidgets.QHBoxLayout()
        model_row.addWidget(self.model)
        model_row.addWidget(browse)
        self.load_model_button = QtWidgets.QPushButton("Load Model / Read Classes")
        self.load_model_button.clicked.connect(self._load_model)
        self.task = QtWidgets.QComboBox()
        self.task.addItem("Select model task", "")
        for task in MODEL_TASKS:
            self.task.addItem(task, task)
        self.task.setEnabled(False)
        identity.addRow("Item name", self.name)
        identity.addRow("Model source", model_row)
        identity.addRow(self.load_model_button)
        identity.addRow("Verified task", self.task)
        self.classes = QtWidgets.QListWidget()
        self.classes.setMaximumHeight(100)
        self.classes.setMinimumHeight(60)
        self.classes.itemChanged.connect(self._detection_settings_changed)
        identity.addRow("Pick classes", self.classes)
        self.geometry_source = QtWidgets.QComboBox()
        self.geometry_source.addItem("No geometry (preview only)", "none")
        self.geometry_source.currentIndexChanged.connect(self._detection_settings_changed)
        identity.addRow("Geometry output", self.geometry_source)

        home_group = group("Home — routine start / end", 5)
        self.home_label = QtWidgets.QLabel("Not recorded. No zero-joint default.")
        self.home_label.setWordWrap(True)
        home_group.addRow(self.home_label)
        self.home_button = QtWidgets.QPushButton("Record Current Joints as Home")
        self.home_button.clicked.connect(self._capture_home)
        home_group.addRow(self.home_button)
        home_help = QtWidgets.QLabel("Reads feedback only; never moves the robot.")
        home_help.setWordWrap(True)
        home_group.addRow(home_help)

        motion = group("Vertical motion — mm", 6)
        descriptions = {
            "standoff_height": "Gripper compensation at final Link6 pick position",
            "prepick_height": "Distance above Link6 pick Z before final approach",
            "retract_height": "Extra clearance above pre-pick Z (pick + pre-pick + retract)",
        }
        for key in MOTION_FIELDS:
            field = QtWidgets.QLineEdit()
            field.setPlaceholderText("Required; millimetres")
            field.setToolTip(descriptions[key])
            self.inputs[key] = field
            motion.addRow(key, field)

        speed = group("Motion speeds — %", 7)
        speed_labels = {"travel_percent": "Travel / Home", "approach_percent": "Final approach",
                        "retract_percent": "Pick to pre-pick retract"}
        for key in SPEED_FIELDS:
            field = QtWidgets.QLineEdit(str(NEW_PROFILE_SPEED[key]))
            field.setPlaceholderText("Required; integer 1–100%")
            field.setToolTip("Per-motion speed percentage; global SpeedFactor remains 100%")
            self.inputs[key] = field
            speed.addRow(speed_labels[key], field)

        acceleration = group("Motion acceleration — %", 8)
        for key in SPEED_FIELDS:
            field = QtWidgets.QLineEdit(str(NEW_PROFILE_ACCELERATION[key]))
            field.setPlaceholderText("Required; integer 1–100%")
            field.setToolTip("Per-motion acceleration percentage (MovLIO a=)")
            self.inputs[f"acceleration_{key}"] = field
            acceleration.addRow(speed_labels[key], field)

        grip = group("Gripper / timing", 9)
        for key in GRIPPER_FIELDS:
            field = QtWidgets.QCheckBox()
            self.inputs[key] = field
            grip.addRow(key, field)
        self.inputs["use_grip"].stateChanged.connect(
            lambda state: self._grip_enabled(state == QtCore.Qt.Checked))
        self.inputs["grip_onpick"].setEnabled(False)
        self.inputs["grip_onpick"].setToolTip("No effect when use_grip is false")
        settle = QtWidgets.QLineEdit()
        settle.setPlaceholderText("Required; seconds to wait at pick if DI1 has not triggered")
        self.inputs["pick_settling"] = settle
        grip.addRow("pick_settling [s]", settle)
        grip_help = QtWidgets.QLabel(
            "DO1 exhaust · DO2 close · DO13 suction · DO14 open\n"
            "DI1 suction · DI12 fully open\nI/O is not actuated in this version."
        )
        grip_help.setWordWrap(True)
        grip.addRow(grip_help)

        yolo = group("3  YOLO — live settings", 2)
        for key, text in (
            ("confidence", "Required; 0..1"), ("iou", "Required; 0..1 (NMS-capable model)"),
            ("max_detections", "Required; detections per image, e.g. 20"),
        ):
            field = QtWidgets.QLineEdit()
            field.setPlaceholderText(text)
            field.setText(str(INITIAL_PREVIEW_YOLO[key]))
            self.inputs[key] = field
            label = {"confidence": "Confidence (0–1)", "iou": "Overlap IoU (0–1)"}
            yolo.addRow(label.get(key, key), field)
        retry = group("Pose candidates", 10)
        limit = QtWidgets.QLineEdit()
        limit.setPlaceholderText("Required; e.g. 3 = up to three ranked poses")
        self.inputs["pose_candidates"] = limit
        retry.addRow("pose_candidates", limit)
        explanation = QtWidgets.QLabel(
            "Maximum ranked poses requested for the controller to use for retries.\n"
            "YOLO detection cap is separate. Robot retry execution is not implemented."
        )
        explanation.setWordWrap(True)
        retry.addRow(explanation)
        geometry = group("4  Item size / pick depth — mm", 3)
        for key in GEOMETRY_FIELDS:
            field = QtWidgets.QLineEdit()
            field.setPlaceholderText("Required; millimetres")
            if key == "pickdepth_radius":
                field.setText(str(DEFAULT_PICKDEPTH_DIAMETER_MM))
                field.setToolTip("Sampling circle DIAMETER in mm, despite the variable name")
            self.inputs[key] = field
            label = {"height": "Length X / height", "width": "Width Y",
                     "tolerance": "Size tolerance ±"}
            geometry.addRow(label.get(key, key), field)
        geometry_help = QtWidgets.QLabel(
            "Item X: long axis; item Y: short axis.\n"
            "pickdepth_radius is the circle DIAMETER (default 30 mm).\n"
            "Length/width use platform Z=0; depth supplies the pick point."
        )
        geometry_help.setWordWrap(True)
        geometry.addRow(geometry_help)
        quality = group("Data-quality limits", 4)
        for key, value in QUALITY_DEFAULTS.items():
            field = QtWidgets.QLineEdit(str(value))
            self.inputs[key] = field
            quality.addRow(key, field)

        for order in sorted(sections):
            form_column.addWidget(sections[order])
        footer = QtWidgets.QHBoxLayout()
        self.activity_toggle = QtWidgets.QToolButton()
        self.activity_toggle.setText("Activity log")
        self.activity_toggle.setCheckable(True)
        self.activity_toggle.setArrowType(QtCore.Qt.RightArrow)
        self.activity_toggle.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        footer.addWidget(self.activity_toggle)
        self.activity_summary = QtWidgets.QLabel("Ready")
        self.activity_summary.setSizePolicy(QtWidgets.QSizePolicy.Ignored,
                                            QtWidgets.QSizePolicy.Preferred)
        footer.addWidget(self.activity_summary, 1)
        outer.addLayout(footer)
        self.status = QtWidgets.QPlainTextEdit()
        self.status.setReadOnly(True)
        self.status.setMaximumBlockCount(1000)
        self.status.setMaximumHeight(100)
        self.status.hide()
        self.activity_toggle.toggled.connect(self.status.setVisible)
        self.activity_toggle.toggled.connect(lambda checked: self.activity_toggle.setArrowType(
            QtCore.Qt.DownArrow if checked else QtCore.Qt.RightArrow))
        outer.addWidget(self.status)
        live_fields = {"confidence", "iou", "max_detections", *GEOMETRY_FIELDS, *QUALITY_DEFAULTS}
        for key in live_fields:
            self.inputs[key].textChanged.connect(self._detection_settings_changed)
        text_fields = [v for key, v in self.inputs.items()
                       if isinstance(v, QtWidgets.QLineEdit) and key not in live_fields]
        for field in (self.name, self.model, *text_fields):
            field.textChanged.connect(self._dirty)
        self.task.currentIndexChanged.connect(self._dirty)
        for key in GRIPPER_FIELDS:
            self.inputs[key].stateChanged.connect(self._dirty)
        state = load_package_ui_state(ui_state_path())
        if state is not None and state.item_preview_camera_prefix is not None:
            self.camera_prefix.setText(state.item_preview_camera_prefix)
        if state is not None and state.item_platform_filename is not None:
            self.bin_path.setText(
                str(workspace_root() / "offline_teach/bin_teach" / state.item_bin_filename))
        if state is not None and state.item_profile_filename is not None:
            self._load(item_directory() / state.item_profile_filename, prefill=True)
        else:
            self._message("Load model + Connect RGB, then enable YOLO Detect. "
                          "Dimensions may be blank for detection; pose checks require them.")
        # Restore the bin before latest-station discovery. This is the read-only station-preview
        # exception to unapplied prefill; model execution and arming stay explicit.
        self.bin_path.textChanged.connect(self._update_station_preview)
        self._update_station_preview()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh_video)
        self.timer.start(100)

    def _connect_camera(self):
        try:
            prefix = validate_prefix(self.camera_prefix.text().strip())
            self.yolo_toggle.setChecked(False)
            write_item_preview_state(ui_state_path(), prefix)
            self.node.connect_camera(prefix)
            if self.node.applied is None and self.platform_path.text() and self.bin_path.text():
                self.bin_platform_warning.clear()
                self.bin_platform_warning.setToolTip("")
                self.bin_platform_warning.hide()
                self.station_status.setText(
                    "Bin ROI hidden: RGB camera is not bound to the selected station. "
                    "Select matching platform/bin teach files."
                )
        except (ValueError, OSError, RuntimeError) as exc:
            self._error("Camera connection failed", exc)

    def _toggle_yolo(self, enabled):
        self._resume_live()
        self.preview_error = ""
        self.preview_update_due = None
        self.preview_settings_paused = False
        if not enabled:
            self.preview_revision += 1
            self.node.yolo_enabled = False
            self.node.last_view = None
            self.node.disarm()
            self.armed_toggle.setChecked(False)
            self.preview_status = "YOLO OFF"
            self.yolo_toggle.setText("YOLO Detect: OFF")
            return
        try:
            self._configure_preview()
            self.last_preview_sequence = None
            self.preview_status = "YOLO ON"
            self.yolo_toggle.setText("YOLO Detect: ON")
        except (ValueError, OSError, RuntimeError) as exc:
            self.yolo_toggle.setChecked(False)
            self._error("YOLO not enabled", exc)

    def _configure_preview(self):
        if self.model_load_reserved:
            raise ValueError("Model loading is pending; wait for Load Model to finish")
        if self.node.camera_prefix != self.camera_prefix.text().strip():
            raise ValueError("Connect the exact camera prefix before enabling YOLO")
        if not self.node.camera_prefix:
            raise ValueError("Connect RGB first")
        if self.node.model_config is None or self.node.model_config["path"] != str(
                Path(self.model.text()).expanduser().resolve()):
            raise ValueError("Load the selected model first")
        # Incomplete dimensions are explicitly unchecked, not guessed or reused.
        size_fields = ("height", "width", "tolerance")
        geometry = ({key: self._number(key) for key in GEOMETRY_FIELDS}
                    if all(self.inputs[key].text().strip() for key in size_fields) else None)
        quality = self._quality_settings()
        self.node.enable_preview(self.geometry_source.currentData(), self._yolo_settings(),
                                 geometry=geometry, quality=quality,
                                 diameter_mm=self._number("pickdepth_radius"))
        yolo = self.node.preview_yolo
        self.preview_help.setText(
            f"Active: confidence {yolo['confidence']:g}, IoU {yolo['iou']:g}, "
            f"{yolo['image_size']} px, cap {yolo['max_detections']}; all model classes.\n"
            "Size border: GREEN within tolerance / RED outside / GRAY not checked.\n"
            "Click for RGB/depth pose + teaching TF; click image again to resume live.\n"
            "Edits update automatically, disarm and require saving before re-arming."
        )

    def _resume_live(self):
        if (self.frozen_view is not None or self.pending_pose is not None
                or self.pending_simulation is not None or self.simulation_busy):
            self.preview_revision += 1
        self.pending_simulation = self.simulation_response = None
        if not self.simulation_busy and not self.model_load_reserved:
            self.simulate_button.setEnabled(True)
            self.simulate_button.setText("Simulate Trigger")
        self.pending_pose = self.selected_pose_result = None
        self.selected_pose_status = ""
        self.node.clear_selected_pose()
        self.displayed_view = self.frozen_view = self.selected_detection = None
        self.last_preview_sequence = None

    def _select_detection(self, point):
        view = self.displayed_view
        if view is None:
            return
        pixel = image_click(point, self.video, view["width"], view["height"])
        if pixel is None:
            return
        if self.pending_simulation is not None or self.simulation_busy:
            self._resume_live()  # Cancel display/request eligibility, never interrupt native work.
            return
        if self.frozen_view is not None:
            self._resume_live()
            return
        if view.get("preview_mode") != "all":
            return
        matches = []
        for item in view["metadata"]["detections"]:
            polygon = QtGui.QPolygonF([QtCore.QPointF(*p) for p in item["polygon"]])
            if polygon.containsPoint(pixel, QtCore.Qt.OddEvenFill):
                rect = item["rectangle"]
                area = abs(sum(rect[i][0] * rect[(i+1) % 4][1]
                               - rect[(i+1) % 4][0] * rect[i][1] for i in range(4)))
                matches.append((area, -item["confidence"], item["source_index"], item))
        if not matches:
            return
        self.selected_detection = min(matches, key=lambda value: value[:3])[3]
        self.frozen_view = view  # Exact displayed frame, not the worker's newer result/index.
        self.node.clear_selected_pose()
        self.selected_pose_result = None
        try:
            settings = self._inference_settings()
            if self.selected_detection.get("size_valid") is False:
                raise ValueError("Size outside tolerance; no pose TF")
            self.pending_pose = (view, self.selected_detection, settings)
            self.selected_pose_status = "Calculating selected RGB/depth pose…"
        except ValueError as exc:
            self.selected_pose_status = "Pose unavailable: " + str(exc)
            self._message(self.selected_pose_status)
        self.node.events.record("INFO", "item_preview_selected",
                                "Frozen frame-local selection; calculating only the clicked item",
                                stamp_ns=view["stamp_ns"],
                                source_index=self.selected_detection["source_index"],
                                measurement=self.selected_detection["measurement"],
                                reason=self.selected_detection["measurement_error"])

    def _simulate_trigger(self):
        if (self.model_load_reserved or self.pending_simulation is not None
                or self.simulation_busy or self.closing):
            return
        try:
            if self.saved_path is None:
                raise ValueError("Save/load a complete current item profile before simulating")
            if not self.yolo_toggle.isChecked() or self.preview_settings_paused:
                raise ValueError("Enable YOLO with valid settings before simulating")
            self.node.enable_yolo(self._inference_settings())
            # Validate without temporarily arming/exposing a ROS service.
            _profile, digest = self.node._validate_pose_profile(self.saved_path)
            self._resume_live()
            self.preview_revision += 1  # Discard any all-detection preview already in flight.
            self.pending_simulation = (
                self.saved_path, digest,
                (self.node.get_clock().now().nanoseconds, time.monotonic()))
            self.simulate_button.setEnabled(False)
            self.simulate_button.setText("Trigger queued…")
            self.preview_status = "SIMULATE TRIGGER — waiting for a new RGB/depth pair"
            self.preview_error = ""
            self._message(self.preview_status)
        except (ValueError, OSError, RuntimeError) as exc:
            self._error("Cannot simulate trigger", exc)

    def _job(self, kind, action):
        if self.job_busy:
            raise ValueError("A model/preview operation is already running")
        self.job_busy = True
        self.job_revision = self.preview_revision

        def run():
            try:
                self.job_results.put((kind, action(), None))
            except Exception as exc:
                self.job_results.put((kind, None, exc))
        threading.Thread(target=run, daemon=True).start()

    def _load_model(self):
        if self.model_load_reserved or self.closing or self.node.native.failed:
            return
        # Reserve before the modal question: Qt timers also run inside its event
        # loop and must not keep taking the worker for automatic ROI previews.
        self._reserve_model_load()
        path = self.model.text()
        if QtWidgets.QMessageBox.question(self, "Load trusted model?",
                                          "A .pt can execute code. Load only a model you trust.\n"
                                          "Load this model locally and read its classes?",
                                          QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                          QtWidgets.QMessageBox.No) != QtWidgets.QMessageBox.Yes:
            self._finish_model_load()
            return
        if self.closing or self.node.native.failed:
            self._finish_model_load()
            return
        self._queue_model_load(path)

    def _reserve_model_load(self):
        self.model_load_reserved = True
        self.load_model_button.setEnabled(False)
        self.load_teach_button.setEnabled(False)
        self.save_button.setEnabled(False)
        self.yolo_toggle.setEnabled(False)
        self.armed_toggle.setEnabled(False)
        self.simulate_button.setEnabled(False)

    def _queue_model_load(self, path, *, pair=None):
        """Queue exactly one confirmed model; a paired load is bound to its YAML hash."""
        self.yolo_toggle.setChecked(False)
        self.node.disarm()
        self.armed_toggle.setChecked(False)
        self.node.model_config = None
        self.node.model_metadata = None
        self.model_requested_path = path
        self.model_requested_pair = pair
        self.pending_model_path = path
        self.load_model_button.setText("Model queued — waiting for current preview…")
        self.preview_status = "Model load queued; automatic previews paused"
        self._message(self.preview_status)
        self.node.events.record("INFO", "item_model_load_queued", self.preview_status,
                                path=path, waiting_for_preview=self.job_busy)
        self._start_pending_model_load()

    def _finish_model_load(self):
        self.pending_model_path = None
        self.model_requested_pair = None
        self.model_load_reserved = False
        self.load_model_button.setText("Load Model / Read Classes")
        self.load_model_button.setEnabled(True)
        self.load_teach_button.setEnabled(True)
        self.save_button.setEnabled(True)
        self.yolo_toggle.setEnabled(True)
        self.armed_toggle.setEnabled(True)
        self.simulate_button.setEnabled(not self.simulation_busy
                                        and self.pending_simulation is None)

    def _start_pending_model_load(self):
        if self.pending_model_path is None or self.job_busy or self.closing:
            return
        path = self.pending_model_path
        if self.model.text() != path:
            self._finish_model_load()
            self._message("Model selection changed while queued; load cancelled. "
                          "Select Load Model again.")
            return
        self.pending_model_path = None
        self.load_model_button.setText("Loading model / reading classes…")
        self.preview_status = "Loading model; automatic previews paused"
        pair = self.model_requested_pair

        def inspect():
            if pair is None:
                return self.node.inspect_model(path)
            profile = self._validate_model_pair(pair)
            return self.node.inspect_model(path, expected_sha256=profile["model"]["sha256"])
        self._job("model", inspect)

    @staticmethod
    def _validate_model_pair(pair, metadata=None):
        path, expected_digest = pair[:2]
        recovery = len(pair) == 3 and pair[2] == "recovery"
        if recovery:
            draft = recover_item_fields(path)
            if draft.model_path is None:
                raise ValueError("Recovered paired model is missing or changed")
            profile, digest = {"model": {"sha256": draft.model_sha256}}, draft.digest
        else:
            profile, digest = load_item_profile(path)
        if digest != expected_digest:
            raise ValueError("Item teach changed while its paired model was loading")
        if recovery:
            if metadata is not None and metadata["sha256"] != draft.model_sha256:
                raise ValueError("Recovered paired model hash changed")
            return profile  # Actual model metadata is authoritative for a recovery draft.
        if metadata is not None:
            if (metadata["sha256"] != profile["model"]["sha256"]
                    or metadata["task"] != profile["model"]["declared_task"]):
                raise ValueError("Paired model hash/task does not match the item teach")
            if set(profile["yolo"]["class_ids"]) - {int(k) for k in metadata["classes"]}:
                raise ValueError("Paired model is missing saved item class IDs")
            source = profile["geometry_source"]
            if source != "none" and source not in metadata["geometry_sources"]:
                raise ValueError("Paired model does not provide the saved geometry output")
        return profile

    def _populate_classes(self, names, selected):
        self.classes.blockSignals(True)
        self.classes.clear()
        for key, name in sorted(names.items(), key=lambda pair: int(pair[0])):
            entry = QtWidgets.QListWidgetItem(f"{key}: {name}")
            entry.setData(QtCore.Qt.UserRole, int(key))
            entry.setFlags(entry.flags() | QtCore.Qt.ItemIsUserCheckable)
            entry.setCheckState(QtCore.Qt.Checked if int(key) in selected else QtCore.Qt.Unchecked)
            self.classes.addItem(entry)
        self.classes.blockSignals(False)

    def _selected_classes(self):
        return [self.classes.item(i).data(QtCore.Qt.UserRole) for i in range(self.classes.count())
                if self.classes.item(i).checkState() == QtCore.Qt.Checked]

    def _populate_sources(self, sources, selected):
        self.geometry_source.blockSignals(True)
        self.geometry_source.clear()
        self.geometry_source.addItem("No geometry (preview only)", "none")
        for source in sources:
            self.geometry_source.addItem(source, source)
        index = self.geometry_source.findData(selected)
        self.geometry_source.setCurrentIndex(max(0, index))
        self.geometry_source.blockSignals(False)

    def _choose_station_file(self, field, directory):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Select teaching artifact",
                                                        str(directory), "YAML (*.yaml)")
        if path:
            if field.text() == path:
                # Explicit reselection may revalidate a corrected file; no timer
                # repeatedly reloads a failed or externally changed artifact.
                self._update_station_preview()
            else:
                field.setText(path)

    def _update_station_preview(self, *_):
        """Select latest station once, validate the bin and subscribe without Apply."""
        self.yolo_toggle.setChecked(False)
        self.node.disarm()
        self.armed_toggle.setChecked(False)
        self.node.applied = self.node.bin_artifact = None
        self.node.last_view = None
        self.last_preview_sequence = None
        self._resume_live()
        self.bin_platform_warning.clear()
        self.bin_platform_warning.setToolTip("")
        self.bin_platform_warning.hide()
        platform, bin_path = "", self.bin_path.text()
        self.platform_path.clear()
        self.calibration_camera_path.clear()
        self.platform_path.setToolTip("")
        self.calibration_camera_path.setToolTip("")
        try:
            latest = latest_station_calibration(root=workspace_root())
            platform = str(latest.platform.path)
            self.platform_path.setText(platform)
            self.calibration_camera_path.setText(str(latest.camera.path))
            self.platform_path.setToolTip(platform)
            self.calibration_camera_path.setToolTip(str(latest.camera.path))
            self.node.events.record(
                "INFO", "item_latest_station_selected", "Selected newest hash-bound calibration",
                platform=platform, platform_sha256=latest.platform.sha256,
                camera=str(latest.camera.path), camera_sha256=latest.camera.sha256)
            if not bin_path:
                self.station_status.setText(
                    "Latest platform/camera loaded. Select a bin teach to display its ROI.")
                return
            self.node.apply_station(platform, bin_path, expected_station=latest)
            self.camera_prefix.setText(self.node.camera_prefix)
            write_item_station_state(ui_state_path(), Path(platform).name, Path(bin_path).name)
            write_item_preview_state(ui_state_path(), self.node.camera_prefix)
            message = (f"Bin ROI ready for /{self.node.camera_prefix}: automatically displayed "
                       "when valid RGB, CameraInfo and TF are available. "
                       "YOLO detection and arming remain manual.")
            self.station_status.setText(message)
            self._message(message)
            self.node.events.record("INFO", "item_station_preview_auto_loaded", message,
                                    platform=platform, bin=bin_path)
            template, selected_platform = self.node.bin_artifact, self.node.applied.platform
            warning = bin_platform_warning(template, selected_platform)
            if warning:
                # Timestamped filenames otherwise clip inside the narrow setup column.
                self.bin_platform_warning.setText(warning.replace("_", "_\u200b"))
                self.bin_platform_warning.setToolTip(
                    f"{warning}\n\n"
                    f"Recorded platform SHA-256: {template.source_platform_calibration_sha256}\n"
                    f"Selected platform SHA-256: {selected_platform.sha256}")
                self.bin_platform_warning.show()
                self._message(warning)
                self.node.events.record(
                    "WARNING", "item_bin_platform_mismatch", warning,
                    bin_filename=template.path.name,
                    source_platform_filename=template.source_platform_calibration_filename,
                    source_platform_sha256=template.source_platform_calibration_sha256,
                    selected_platform_filename=selected_platform.path.name,
                    selected_platform_sha256=selected_platform.sha256)
        except (ValueError, OSError, RuntimeError) as exc:
            self.node.disarm()
            self.node.applied = self.node.bin_artifact = self.node.last_view = None
            self.bin_platform_warning.clear()
            self.bin_platform_warning.setToolTip("")
            self.bin_platform_warning.hide()
            message = f"Bin ROI hidden: {exc}. Check latest station calibration and bin teach."
            self.station_status.setText(message)
            self._message(message)
            self.node.events.record("WARNING", "item_station_preview_invalid", message,
                                    platform=platform, bin=bin_path)

    def _style_armed(self, enabled):
        self.armed_toggle.setStyleSheet(
            "background:#b51f24;color:white;font-weight:700;border:2px solid #7c1115;"
            if enabled else "")

    def _toggle_armed(self, enabled):
        self._resume_live()
        if not enabled:
            self.node.disarm()
            self.armed_toggle.setText("Armed: OFF")
            return
        try:
            if self.model_load_reserved:
                raise ValueError("Wait for model loading to finish before arming")
            if self.saved_path is None:
                raise ValueError("Explicitly save/load the current profile before arming")
            if not self.yolo_toggle.isChecked():
                raise ValueError("Enable YOLO before arming")
            self.node.enable_yolo(self._inference_settings())
            self.node.arm(self.saved_path)
            self.armed_toggle.setText("Armed: ON")
            self._message("Pose service ON: /item_detect/get_item_poses. No robot motion.")
        except (ValueError, OSError, RuntimeError) as exc:
            self.armed_toggle.setChecked(False)
            self._error("Not armed", exc)

    def _refresh_video(self):
        if self.closing:
            return
        try:
            kind, value, error = self.job_results.get_nowait()
        except queue.Empty:
            pass
        else:
            self.job_busy = False
            if kind == "simulate":
                self.simulation_busy = False
                self.simulate_button.setEnabled(not self.model_load_reserved)
                self.simulate_button.setText("Simulate Trigger")
            if kind != "model" and self.job_revision != self.preview_revision:
                # Even already-completed, queued replies may belong to settings
                # edited after inference finished. Never resurrect their overlays.
                value = error = None
                self.node.last_view = None
            pair = self.model_requested_pair if kind == "model" else None
            if kind == "model" and error is None and pair is not None:
                try:
                    self._validate_model_pair(pair, value)
                except (ValueError, OSError) as exc:
                    error = exc
            if kind == "model":
                self._finish_model_load()
            if error is not None:
                self.preview_status = str(error)
                self.preview_error = str(error)
                if kind == "simulate":
                    self.node.clear_selected_pose()
                    self._message("Simulate Trigger failed: " + str(error))
                if kind == "pose":
                    self.node.clear_selected_pose()
                    self.selected_pose_status = "Pose unavailable: " + str(error)
                    self._message(self.selected_pose_status)
                if kind == "model":
                    self.node.model_config = self.node.model_metadata = None
                    if pair is not None:
                        self.saved_path = None
                    self._error("Model load failed", error)
            elif kind == "model":
                if self.model.text() != self.model_requested_path:
                    self.node.model_config = None
                    self._message("Model selection changed during load; result discarded.")
                    return
                selected = self._selected_classes()
                missing = set(selected) - {int(key) for key in value["classes"]}
                if missing:
                    self._error("Saved class selection is invalid",
                                f"Loaded model has no class IDs {sorted(missing)}. "
                                "Review checkboxes and save the corrected profile.")
                self.task.setCurrentIndex(self.task.findData(value["task"]))
                self._populate_classes(value["classes"], selected)
                source = self.geometry_source.currentData()
                if pair is None and source == "none" and len(value["geometry_sources"]) == 1:
                    source = value["geometry_sources"][0]
                self._populate_sources(value["geometry_sources"], source)
                recovery = pair is not None and len(pair) == 3
                if recovery and source not in ("none", *value["geometry_sources"]):
                    self.geometry_source.setCurrentIndex(-1)
                    self._message("Recovered geometry output is unknown/incompatible; select it.")
                self.preview_error = ""
                self.preview_status = "Model loaded — enable YOLO Detect to preview"
                self._message("Model loaded. Preview displays all model classes. "
                              "Select geometry when both mask and OBB are available.")
                if pair is not None:
                    self._message(
                        "Recovered draft and verified paired model loaded; review fields and "
                        "save the corrected profile before arming." if recovery else
                        "Item teach and paired model loaded. Saved classes/settings "
                        "retained; YOLO Detect and Armed remain OFF.")
                    self.node.events.record("INFO", "item_pair_model_loaded",
                                            "Verified paired model loaded with item teach",
                                            profile=str(pair[0]), profile_sha256=pair[1])
            elif kind == "simulate" and value is not None:
                response = value["response"]
                self.node.clear_selected_pose()
                if response.success:
                    try:
                        self.node.show_simulated_poses(response, value["view"])
                        self.simulation_response = response
                        self.frozen_view = {**value["view"], "preview_mode": "simulated"}
                        self.preview_error = ""
                        self.preview_status = f"SIMULATED {response.status}: {response.message}"
                        for candidate in response.candidates:
                            self._message(self._batch_candidate_text(candidate))
                    except (ValueError, OSError) as exc:
                        self._resume_live()
                        self.node.last_view = None
                        self.preview_status = "Simulate Trigger preview rejected: " + str(exc)
                        self.preview_error = str(exc)
                else:
                    self.preview_status = "Simulate Trigger failed: " + response.message
                    self.preview_error = response.message
                self._message(self.preview_status)
            elif kind == "pose" and value is not None and self.frozen_view is not None:
                self.frozen_view = {**self.frozen_view, "depth_rgb": value["depth_rgb"]}
                candidate = value["candidate"]
                try:
                    if candidate is not None:
                        self.node.show_selected_pose(candidate, value["stamp_ns"], value["epoch"])
                        self.selected_pose_result = candidate
                        self.selected_pose_status = (
                            "Teaching TF: base_link → item_teach_selected_item")
                    else:
                        self.node.clear_selected_pose()
                        self.selected_pose_status = "Pose rejected: " + value["reason"]
                except (ValueError, OSError) as exc:
                    self.node.clear_selected_pose()
                    self.selected_pose_status = "Pose unavailable: " + str(exc)
                self.node.events.record("INFO", "item_teach_pose_result",
                                        self.selected_pose_status, result=value["candidate"],
                                        source_stamp_ns=value["stamp_ns"])
                self._message(self.selected_pose_status)
            elif value is not None:
                self.preview_error = ""
                metadata = value["metadata"]
                if value["preview_mode"] == "roi":
                    self.preview_status = "YOLO OFF — loaded bin ROI"
                elif value["preview_mode"] == "all":
                    self.preview_status = f"DETECTIONS: {metadata['count']} | click for pose"
                else:
                    self.preview_status = (f"FILTERED: {len(metadata['candidates'])} valid picks "
                                           f"/ {metadata['count']} YOLO detections")
                if value["preview_mode"] != "roi":
                    self._populate_sources(metadata["geometry_sources"],
                                           self.geometry_source.currentData())
        if self.node.native.failed:
            self.node.clear_selected_pose()
            self._finish_model_load()
            self.node.fatal_error = "Item native worker failed; no restart or fallback"
            self.node.disarm()
            self.node.get_logger().fatal(self.node.fatal_error)
            QtWidgets.QApplication.instance().quit()
            return
        # The explicitly trusted model takes the next free slot exactly once.
        # Never interrupt/restart the lifetime worker or retry a failed request.
        self._start_pending_model_load()
        if (self.preview_update_due is not None and not self.model_load_reserved
                and time.monotonic() >= self.preview_update_due):
            self._apply_live_detection_settings()
        if (self.pending_pose is not None and not self.job_busy and not self.model_load_reserved
                and self.frozen_view is not None and self.node.yolo_enabled):
            view, detection, settings = self.pending_pose
            self.pending_pose = None
            self._job("pose", lambda: self.node.clicked_pose(view, detection, settings))
        if (self.pending_simulation is not None and not self.job_busy
                and not self.model_load_reserved):
            path, digest, requested_at = self.pending_simulation
            self.pending_simulation = None
            self.simulation_busy = True
            revision = self.preview_revision
            self.simulate_button.setText("Simulating…")
            self._job("simulate", lambda: self.node.simulate_trigger(
                path, expected_digest=digest, requested_at=requested_at,
                cancelled=lambda: self.closing or revision != self.preview_revision))
        if self.node.service is None and self.armed_toggle.isChecked():
            self.armed_toggle.setChecked(False)
        if self.simulation_response is not None:
            try:
                self.node.validate_simulation_view(self.frozen_view)
            except (ValueError, OSError) as exc:
                self._resume_live()
                self.node.last_view = None
                self.preview_error = "Simulated batch cleared: " + str(exc)
                self._message(self.preview_error)
        frame, camera_status = self.node.camera_snapshot()
        if ((self.node.yolo_enabled or self.node.applied is not None)
                and not self.preview_settings_paused
                and not self.model_load_reserved and not self.job_busy and frame is not None
                and self.pending_simulation is None and not self.simulation_busy
                and self.frozen_view is None
                and frame["sequence"] != self.last_preview_sequence):
            self.last_preview_sequence = frame["sequence"]
            self._job("preview" if self.node.yolo_enabled else "roi",
                      self.node.preview_once if self.node.yolo_enabled else self.node.roi_once)
        view = self.node.last_view if self.node.last_view is not None else frame
        roi_note = ""
        if view is not None and self.frozen_view is None:
            age = (self.node.get_clock().now().nanoseconds - view["stamp_ns"]) / 1e9
            result_snapshot = view.get("preview_mode") in ("all", "filtered")
            if age < 0 or (age > 0.5 and not result_snapshot):
                # Live ROI-only projections must stay fresh. Completed inference is
                # instead a labelled snapshot: keep its annotations AND source RGB
                # together even when CPU inference itself takes over 0.5 seconds.
                # This display-only choice never supplies a pose-service response.
                view = frame
                roi_note = "Bin ROI hidden: overlay frame is stale; waiting for fresh projection"
        if self.frozen_view is not None:
            view = self.frozen_view
        if view is not None and view.get("metadata", {}).get("roi_overlay") is not None:
            roi = view["metadata"]["roi_overlay"]
            roi_note = "Loaded Bin ROI" if roi["visible"] else "Bin ROI hidden: " + roi["reason"]
        self.displayed_view = view
        self.video_status.setText(
            f"{self.preview_status}\n{camera_status} | {self.armed_toggle.text()}\n{roi_note}"
        )
        if view is None:
            self.rgb_feedback.clear()
            self.rgb_feedback.hide()
            self.depth_feedback.clear()
            self.depth_feedback.hide()
            self.video.clear()
            self.video.setText(camera_status)
            self.depth_video.clear()
            self.depth_video.setText("No valid RGB/depth view: " + camera_status)
            return
        age = max(0.0, (self.node.get_clock().now().nanoseconds - view["stamp_ns"]) / 1e9)
        image = QtGui.QImage(
            view["rgb"], view["width"], view["height"], view["width"] * 3,
            QtGui.QImage.Format_RGB888,
        ).copy()
        selected = self.selected_detection
        mode = view.get("preview_mode")
        metadata = view.get("metadata", {})
        if self.preview_settings_paused:
            title = "YOLO PREVIEW PAUSED — updating/checking settings"
        elif selected:
            title = "FROZEN SELECTION — click image to resume"
        elif mode == "simulated":
            batch = self.simulation_response
            title = f"SIMULATED {batch.status} — {batch.message} | click RGB to resume"
        elif self.pending_simulation is not None or self.simulation_busy:
            title = "SIMULATE TRIGGER — acquiring/processing | click RGB to cancel"
        elif mode == "all":
            title = f"DETECTIONS: {len(metadata['detections'])} | click for pose"
        elif mode == "filtered":
            title = (f"FILTERED: {len(metadata['candidates'])} valid picks "
                     f"/ {metadata['count']} YOLO detections")
        elif mode == "roi":
            title = "YOLO OFF — loaded bin ROI"
        else:
            title = ("YOLO ON — waiting for annotated result" if self.node.yolo_enabled
                     else "YOLO OFF — RGB view")
        if not selected and mode in ("all", "filtered") and age > 0.5:
            title = "RESULT SNAPSHOT — " + title
            if roi_note == "Loaded Bin ROI":
                roi_note += " (same result snapshot, not a live projection)"
        timing = view.get("metadata", {}).get("inference_ms")
        suffix = "" if timing is None else f" | inference {timing:.1f}ms"
        frame_note = f"{'STALE ' if age > 0.5 else ''}Frame age {age:.2f}s{suffix}"
        rgb_lines = [title, frame_note]
        batch_lines = []
        if mode == "simulated":
            batch_lines = [f"Frozen teaching batch | {batch.valid_count} valid / "
                           f"{batch.detected_count} detections | NO ROBOT COMMANDS"]
            count = len(batch.candidates)
            batch_lines.append(
                "RViz TF: base_link → item_teach_candidate_1"
                + (f"…{count}" if count > 1 else "") + " (frozen)" if count else
                "RViz TF: no candidate frames (empty batch)")
            for candidate in batch.candidates[:3]:
                batch_lines.append(self._batch_candidate_text(candidate))
            if len(batch.candidates) > 3:
                batch_lines.append(f"{len(batch.candidates)-3} more poses labelled on image; "
                                   "full details in Activity log")
            batch_lines.append("Poses in platform_reference; "
                               "Armed service uses independent new frames.")
            rgb_lines.extend(batch_lines)
        settings_note = ""
        if mode == "all" and not self.preview_settings_paused:
            try:
                current_yolo = self._yolo_settings()
                settings_note = (f"conf {current_yolo['confidence']:g} / "
                                 f"IoU {current_yolo['iou']:g}"
                                 f" / cap {current_yolo['max_detections']} | "
                                 "size: green OK, red outside, gray unchecked")
            except ValueError:
                pass  # Invalid edits pause inference; never show guessed settings.
        self.video_status.setText(
            f"{title}\n{frame_note} | {camera_status} | {self.armed_toggle.text()}\n{roi_note}"
            + (f"\nPreview blocked: {self.preview_error}" if self.preview_error else "")
        )
        if selected:
            item_note = (f"#{selected['source_index']} {selected['class_name']} "
                         f"| confidence {selected['confidence']:.2f}")
            rgb_lines.append(item_note)
            measurement = selected["measurement"]
            if measurement:
                dimension_note = (f"X / height: {measurement['length_mm']:.2f} mm   "
                                  f"Y / width: {measurement['width_mm']:.2f} mm")
                rgb_lines.extend([dimension_note, selected.get(
                    "size_reason", "Platform Z=0 projected size")])
            else:
                rgb_lines.append("Measurement unavailable — see status below")
            pose = self.selected_pose_result
            pose_text = self.selected_pose_status
            if pose is not None:
                xyz = ", ".join(f"{p*1000:+.2f}" for p in pose["position"])
                q = pose["quaternion"]
                yaw = math.degrees(2 * math.atan2(q[2], q[3]))
                rgb_lines.extend([f"platform_reference XYZ [mm]: {xyz}",
                                  f"Yaw: {yaw:+.2f}° | Teaching snapshot TF — no motion"])
                pose_text += f"\nplatform_reference XYZ [mm]: {xyz}; yaw {yaw:+.2f}°"
            else:
                rgb_lines.append("Pose: " + self.selected_pose_status)
            self.video_status.setText(
                f"Frozen frame, age {age:.2f}s. "
                + ("Reference-plane dimensions, not depth-corrected physical size."
                   if measurement else selected["measurement_error"])
                + "\n" + pose_text
                + ("\nSampling circle unavailable: " + selected["sampling_circle_error"]
                   if selected.get("sampling_circle_error") else "")
                + f"\n{camera_status} | {self.armed_toggle.text()}")
            painter = QtGui.QPainter(image)
            draw_sampling_circle(painter, selected.get("sampling_circle"))
            painter.end()
        if settings_note:
            rgb_lines.append(settings_note)
        self.rgb_feedback.setText("\n".join(rgb_lines))
        self.rgb_feedback.show()
        self.video.setPixmap(QtGui.QPixmap.fromImage(image).scaled(
            self.video.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation,
        ))
        depth_pixels = view.get("depth_rgb")
        if depth_pixels:
            depth = QtGui.QImage(depth_pixels, view["width"], view["height"], view["width"] * 3,
                                 QtGui.QImage.Format_RGB888).copy()
            depth_age = (self.node.get_clock().now().nanoseconds - view["depth_stamp_ns"]) / 1e9
            depth_lines = [title, f"Depth age: {depth_age:.2f}s | Accepted BLACK / Rejected RED"]
            depth_lines.extend(batch_lines)
            if selected:
                depth_lines.append(item_note)
                if selected["measurement"]:
                    depth_lines.append(dimension_note)
                if self.selected_pose_result is not None:
                    depth_lines.extend([f"platform_reference XYZ [mm]: {xyz}",
                                        f"Yaw: {yaw:+.2f}° | Teaching snapshot TF"])
                    pose = self.selected_pose_result
                    depth_lines.append(
                        f"Depth {pose['filtered_camera_depth']*1000:.1f} mm | "
                        f"{pose['accepted_depth_count']} accepted / "
                        f"{pose['rejected_depth_count']} rejected")
                else:
                    depth_lines.append("Pose: " + self.selected_pose_status)
                painter = QtGui.QPainter(depth)
                draw_sampling_circle(painter, selected.get("depth_sampling_circle"))
                painter.end()
            if settings_note:
                depth_lines.append(settings_note)
            self.depth_feedback.setText("\n".join(depth_lines))
            self.depth_feedback.show()
            self.depth_video.setPixmap(QtGui.QPixmap.fromImage(depth).scaled(
                self.depth_video.size(), QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.SmoothTransformation))
        else:
            self.depth_feedback.clear()
            self.depth_feedback.hide()
            self.depth_video.clear()
            self.depth_video.setText(
                "Registered depth unavailable for this RGB snapshot.\n"
                + view.get("depth_error", "Waiting for synchronized RGB/depth and CameraInfo."))

    def closeEvent(self, event):
        self.closing = True
        self._resume_live()
        self.pending_model_path = None
        self.timer.stop()
        self.node.close_runtime()
        super().closeEvent(event)

    @staticmethod
    def _batch_candidate_text(candidate):
        p, q = candidate.pose.position, candidate.pose.orientation
        yaw = math.degrees(2 * math.atan2(q.z, q.w))
        return (f"P{candidate.priority} {candidate.class_name} | "
                f"XYZ [mm] {p.x*1000:+.1f}, {p.y*1000:+.1f}, {p.z*1000:+.1f} "
                f"| yaw {yaw:+.1f}° | size {candidate.length*1000:.1f} × "
                f"{candidate.width*1000:.1f} mm | depth "
                f"{candidate.filtered_camera_depth*1000:.1f} mm | "
                f"{candidate.accepted_depth_count} accepted / "
                f"{candidate.rejected_depth_count} rejected")

    def _message(self, message):
        self.status.appendPlainText(message)
        self.activity_summary.setText(message.replace("\n", " "))
        self.activity_summary.setToolTip(message)

    def _dirty(self, *_):
        self._resume_live()
        source = self.geometry_source.currentData()
        if self.node.yolo_enabled and source != self.node.preview_source:
            self._resume_live()
            self.node.last_view = None
            self.node.preview_source = source
        self.node.disarm()
        self.armed_toggle.setChecked(False)
        self.saved_path = None

    def _detection_settings_changed(self, *_):
        self.node.disarm()
        self.armed_toggle.setChecked(False)
        self.saved_path = None
        if not self.yolo_toggle.isChecked():
            return  # Editing a form never starts YOLO automatically.
        self.preview_revision += 1
        self.node.yolo_enabled = False
        self.node.last_view = None
        self._resume_live()
        self.preview_settings_paused = True
        self.preview_update_due = time.monotonic() + 0.3
        self.preview_error = ""
        self.preview_status = "Updating YOLO settings — preview paused while typing"
        self.preview_help.setText(self.preview_status)

    def _apply_live_detection_settings(self):
        self.preview_update_due = None
        if not self.yolo_toggle.isChecked() or self.closing:
            return
        try:
            self._configure_preview()
        except (ValueError, OSError, RuntimeError) as exc:
            self.node.yolo_enabled = False
            self.preview_settings_paused = True
            self.preview_error = str(exc)
            self.preview_status = f"Preview paused — correct settings: {exc}"
            self.preview_help.setText(self.preview_status)
            return  # No old-value fallback, modal dialog or repeated timer retries.
        self.preview_settings_paused = False
        self.last_preview_sequence = None
        self.preview_error = ""
        self.preview_status = "YOLO settings updated — waiting for a new result"
        self.node.events.record("INFO", "item_preview_settings_updated", self.preview_status,
                                yolo=self._yolo_settings())

    def _grip_enabled(self, enabled):
        self.inputs["grip_onpick"].setEnabled(
            enabled or self.inputs["grip_onpick"].checkState() == QtCore.Qt.PartiallyChecked)

    def _browse_model(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select pretrained YOLO .pt — any folder", str(Path.home()),
            "PyTorch model (*.pt)",
        )
        if path:
            self.yolo_toggle.setChecked(False)
            self.profile_image_size = NEW_PROFILE_IMAGE_SIZE
            self.model.setText(path)
            self.node.model_config = None
            self._populate_classes({}, [])
            self._populate_sources([], "none")
            self._message("Model selected. Save will copy/hash it; no weights are executed here.")

    def _show_home(self):
        self.home_label.setText(
            "Recorded on robot: " + self.home["robot_lan1_ip"] + "\n" + "  ".join(
                f"J{i + 1}: {math.degrees(value):+.3f}°"
                for i, value in enumerate(self.home["positions_rad"])
            ) + "\nSaved in radians, joint1 through joint6."
        )

    def _capture_home(self):
        try:
            home = self.node.capture_home()
        except (ValueError, RuntimeError) as exc:
            self._error("Home not recorded", exc)
            return
        if self.home is not None and QtWidgets.QMessageBox.question(
            self, "Replace recorded home?", "Replace all six saved home joints with this reading?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No,
        ) != QtWidgets.QMessageBox.Yes:
            return
        self.home = home
        self.node.events.record("INFO", "home_recorded", "Accepted six home joints", home=home)
        self._show_home()
        self._dirty()
        self._message("Recorded home joints. No robot movement was requested.")

    def _yolo_settings(self):
        if self.profile_image_size is None:
            raise ValueError("Recovered image_size is unknown; Browse a model to start at 640 px")
        return {
            "confidence": self._number("confidence"),
            "iou": self._number("iou"),
            "image_size": self.profile_image_size,
            "max_detections": self._number("max_detections", int),
            "class_ids": self._selected_classes(),
        }

    def _number(self, key, numeric_type=float):
        value = self.inputs[key].text().strip()
        if not value:
            raise ValueError(f"Setting '{key}' is required")
        try:
            parsed = numeric_type(value)
        except ValueError as exc:
            raise ValueError(f"Setting '{key}' must be a valid number") from exc
        if not math.isfinite(parsed):
            raise ValueError(f"Setting '{key}' must be finite")
        return parsed

    def _inference_settings(self):
        geometry = {key: self._number(key) for key in GEOMETRY_FIELDS}
        return {"model_task": self.task.currentData(), "yolo": self._yolo_settings(),
                "geometry": geometry, "geometry_source": self.geometry_source.currentData(),
                "quality": self._quality_settings()}

    def _quality_settings(self):
        return {key: self._number(key, int if key == "minimum_depth_samples" else float)
                for key in QUALITY_DEFAULTS}

    def _settings(self):
        for key in GRIPPER_FIELDS:
            if self.inputs[key].checkState() == QtCore.Qt.PartiallyChecked:
                raise ValueError(f"Recovered {key} is unknown; explicitly choose on or off")
        return {
            "item": {"name": self.name.text().strip()}, "model_task": self.task.currentData(),
            "motion": {key: self._number(key) for key in MOTION_FIELDS},
            "speed": {key: self._number(key, int) for key in SPEED_FIELDS},
            "acceleration": {key: self._number(f"acceleration_{key}", int) for key in SPEED_FIELDS},
            "timing": {"pick_settling": self._number("pick_settling")},
            "gripper": {key: self.inputs[key].isChecked() for key in GRIPPER_FIELDS},
            "retry": {"pose_candidates": self._number("pose_candidates", int)},
            **self._inference_settings(),
        }

    def _save(self):
        if self.model_load_reserved or self.closing or self.node.native.failed:
            return
        if self.home is None:
            self._error("Missing home", "Record actual home joints before saving.")
            return
        overwrite = (self.save_target is not None
                     and self.name.text().strip() == self.save_target.item_name)
        action = (f"Overwrite loaded item teach:\n{self.save_target.path.name}\n"
                  "Its paired .pt is updated only if the selected model changed.\n"
                  "One hidden previous-version ZIP backup will be kept."
                  if overwrite else
                  "Save a NEW YAML and .pt pair in offline_teach/item_teach/?\n"
                  "The item name changed or there is no known loaded item name.")
        if QtWidgets.QMessageBox.question(
            self, "Save item teach?",
            action + "\nNo robot movement will occur.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No,
        ) != QtWidgets.QMessageBox.Yes:
            return
        self._dirty()  # Invalidate service/frozen requests before publishing a changed pair.
        try:
            output, profile = save_item_profile(
                self._settings(), self.home, Path(self.model.text()),
                root=workspace_root(), save_target=self.save_target,
            )
            target = item_save_target(
                output, profile["item"]["name"], file_sha256(output),
                created_at_utc=profile["created_at_utc"], root=workspace_root())
        except (ValueError, OSError) as exc:
            self._error("Save failed", exc)
            return
        self.saved_path = output
        self.save_target = target
        # Retain the explicitly loaded source path and native fingerprint. Saving
        # a new copy is not permission to switch/reload the active model.
        self.recovered_draft = False
        self.recovery_notice.clear()
        self.recovery_notice.hide()
        self.node.events.record(
            "INFO", "item_pair_saved", "Updated loaded pair" if overwrite else "Created pair",
            path=str(output), overwritten=overwrite,
        )
        try:
            write_item_ui_state(ui_state_path(), output.name)
        except (ValueError, OSError) as exc:
            self._error("Pair saved; prefill update failed", exc)
        QtWidgets.QMessageBox.information(
            self, "Item teach saved",
            f"Saved successfully:\n{output}\n{output.with_suffix('.pt')}",
        )
        self._message(f"Saved: {output.name}. Saving does not start inference or motion.")

    def _load_dialog(self):
        if self.model_load_reserved or self.closing or self.node.native.failed:
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load item teach", str(item_directory()), "Item teach (*.yaml)",
        )
        if not path:
            return
        self._reserve_model_load()
        if QtWidgets.QMessageBox.question(
            self, "Load item teach and paired model?",
            "Replace the form and recorded home, then load this teach file's paired .pt?\n"
            "A .pt can execute code; continue only if you trust this pair.\n"
            "Old/invalid files open as recovery drafts with unclear fields empty.\n"
            "YOLO Detect and Armed stay OFF. No robot movement.\n" + Path(path).name,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No,
        ) != QtWidgets.QMessageBox.Yes:
            self._finish_model_load()
            return
        if self.closing or self.node.native.failed:
            self._finish_model_load()
            return
        queued = False
        try:
            _profile, digest = self._load(Path(path), prefill=False)
            if self.model.text():
                pair = ((Path(path), digest, "recovery") if self.recovered_draft
                        else (Path(path), digest))
                self._queue_model_load(self.model.text(), pair=pair)
                queued = True
            else:
                self._finish_model_load()
            write_item_ui_state(ui_state_path(), Path(path).name)
        except (ValueError, OSError) as exc:
            if not queued:
                self._finish_model_load()
            self._error("Load failed", exc)

    def _load(self, path, *, prefill):
        try:
            profile, _digest = load_item_profile(path)
        except (ValueError, OSError) as exc:
            return self._load_recovery(path, exc, prefill=prefill)
        target = item_save_target(
            path, profile["item"]["name"], _digest,
            created_at_utc=profile["created_at_utc"], root=workspace_root())
        if target.model_sha256 != profile["model"]["sha256"]:
            raise ValueError("Paired model changed while loading; load it again")
        self.recovered_draft = False
        self.recovery_notice.clear()
        self.recovery_notice.hide()
        self.yolo_toggle.setChecked(False)
        settings = settings_from_profile(profile)
        self.profile_image_size = settings["yolo"]["image_size"]
        self.name.setText(settings["item"]["name"])
        self.model.setText(str(path.parent / profile["model"]["filename"]))
        self.task.setCurrentIndex(self.task.findData(settings["model_task"]))
        self._populate_classes({key: "saved selection (load model to verify)"
                                for key in settings["yolo"]["class_ids"]},
                               settings["yolo"]["class_ids"])
        source = settings["geometry_source"]
        self._populate_sources([source] if source != "none" else [], source)
        for section in ("motion", "speed", "timing", "retry", "yolo", "geometry", "quality"):
            for key, value in settings[section].items():
                if key in ("class_ids", "image_size"):
                    continue
                self.inputs[key].setText(
                    ",".join(map(str, value)) if isinstance(value, list) else str(value)
                )
        for key, value in settings["acceleration"].items():
            self.inputs[f"acceleration_{key}"].setText(str(value))
        for key, value in settings["gripper"].items():
            self.inputs[key].setTristate(False)
            self.inputs[key].setChecked(value)
        self._grip_enabled(self.inputs["use_grip"].isChecked())
        self.home = copy.deepcopy(profile["home"])
        self._show_home()
        self.saved_path = target.path
        self.save_target = target
        action = "Restored saved item teach" if prefill else "Loaded saved item teach"
        self._message(
            f"{action}: {path.name}. "
            "No extra Save required. Model trust/loading, YOLO and Armed stay explicit; "
            "no controller request or motion was sent."
        )
        return profile, _digest

    def _load_recovery(self, path, error, *, prefill):
        draft = recover_item_fields(path)
        self.save_target = (item_save_target(
            path, draft.values.get("name"), draft.digest, root=workspace_root())
            if draft.digest is not None else None)
        self.yolo_toggle.setChecked(False)
        self.node.disarm()
        self.node.model_config = self.node.model_metadata = self.node.last_view = None
        self._resume_live()
        self.armed_toggle.setChecked(False)
        self.recovered_draft = True
        self.saved_path = None
        self.profile_image_size = draft.values.get("image_size")
        self.name.setText(draft.values.get("name") or "")
        self.model.setText(str(draft.model_path) if draft.model_path is not None else "")
        self.task.setCurrentIndex(self.task.findData(draft.values.get("model_task")))
        ids = draft.values.get("class_ids") or []
        self._populate_classes({key: "recovered (verify model)" for key in ids}, ids)
        source = draft.values.get("geometry_source")
        self._populate_sources([source] if source in ("mask", "obb") else [], source)
        if source is None:
            self.geometry_source.setCurrentIndex(-1)
        for key, widget in self.inputs.items():
            value = draft.values.get(key)
            if isinstance(widget, QtWidgets.QLineEdit):
                widget.setText("" if value is None else str(value))
            elif isinstance(widget, QtWidgets.QCheckBox):
                widget.setTristate(value is None)
                widget.setCheckState(QtCore.Qt.PartiallyChecked if value is None else
                                     QtCore.Qt.Checked if value else QtCore.Qt.Unchecked)
        self._grip_enabled(self.inputs["use_grip"].checkState() == QtCore.Qt.Checked)
        self.home = draft.home
        if self.home is not None:
            self._show_home()
        else:
            self.home_label.setText("Home unavailable — record all six actual joints again.")
        self.recovery_notice.setText(
            "RECOVERY DRAFT — review cleared fields/unknown checkboxes in Activity log. "
            "Save corrected fields before simulating/arming. Same item name updates the loaded "
            "file with a previous-version backup; a changed/unknown original name creates a pair.")
        self.recovery_notice.show()
        self._message(f"Recovered {'prefill' if prefill else 'form'}: {path.name}. {error}")
        for issue in draft.issues:
            self._message(issue)
        self.node.events.record("WARNING", "item_teach_recovered",
                                "GUI draft only; no valid profile or execution", path=str(path),
                                reason=str(error), cleared_fields=draft.issues)
        return None, draft.digest

    def _error(self, title, error):
        self._message(f"{title}: {error}")
        self.node.events.record("ERROR", "operator_action_failed", str(error), action=title)
        QtWidgets.QMessageBox.warning(self, title, str(error))


def main(args=None):
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required"
        )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    rclpy.init(args=args)
    node = None
    window = None
    executor = None
    worker = None
    previous = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    try:
        node = ItemTeachNode()
        window = ItemTeachWindow(node)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        worker = threading.Thread(target=executor.spin, daemon=True)
        worker.start()
        window.show()
        app.exec_()
        if node.fatal_error:
            raise RuntimeError(node.fatal_error)
    except Exception as exc:
        if node is not None:
            node.events.record("FATAL", "node_failed", str(exc))
            node.get_logger().fatal(str(exc))
        raise
    finally:
        if window is not None:
            window.close()
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if worker is not None:
            worker.join(timeout=2.0)
        if node is not None:
            node.events.record("INFO", "node_stopped", "Item editor stopped")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous)
