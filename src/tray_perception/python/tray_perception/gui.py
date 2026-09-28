"""Visual tray teaching with one bounded background job and read-only ROS feedback."""

from concurrent.futures import ThreadPoolExecutor
import copy
import math
import os
from pathlib import Path
import signal
import threading
import time

from python_qt_binding import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from .core import (
    read_session, tray_directory, validate_settings,
    validate_geometry, validate_preview, write_session)
from .documents import detection_profile, open_document, save_document, validate_name
from item_perception_yolo.item_preview import validate_prefix
from item_perception_yolo.item_teach_core import file_sha256
from .node import TrayTeachNode


class TrayCanvas(QtWidgets.QWidget):
    clicked = QtCore.Signal(float, float)

    def __init__(self):
        super().__init__()
        self.image = None
        self.points = []
        self.selecting = False
        self.highlight = []
        self.empty_text = "Connect RGB to begin"
        self.setMinimumSize(400, 200)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

    def show_frame(self, rgb, width, height):
        self.image = QtGui.QImage(rgb, width, height, width * 3, QtGui.QImage.Format_RGB888).copy()
        self.update()

    def image_rect(self):
        if self.image is None:
            return QtCore.QRectF()
        scale = min(self.width() / self.image.width(), self.height() / self.image.height())
        width, height = self.image.width() * scale, self.image.height() * scale
        return QtCore.QRectF(
            (self.width() - width) / 2, (self.height() - height) / 2, width, height)

    def image_point(self, point):
        rect = self.image_rect()
        if self.image is None or not rect.contains(point):
            return None
        x = (point.x() - rect.left()) * self.image.width() / rect.width()
        y = (point.y() - rect.top()) * self.image.height() / rect.height()
        if 0 <= x < self.image.width() and 0 <= y < self.image.height():
            return x, y
        return None

    def mousePressEvent(self, event):
        point = self.image_point(event.localPos())
        if self.selecting and event.button() == QtCore.Qt.LeftButton and point is not None:
            self.clicked.emit(*point)

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#151b22"))
        if self.image is None:
            painter.setPen(QtCore.Qt.white)
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter,
                             self.empty_text)
            return
        rect = self.image_rect()
        painter.drawImage(rect, self.image)
        painter.setPen(QtGui.QPen(QtGui.QColor("cyan"), 2))
        if self.highlight:
            painter.drawPolygon(QtGui.QPolygonF([
                QtCore.QPointF(rect.left() + x * rect.width() / self.image.width(),
                               rect.top() + y * rect.height() / self.image.height())
                for x, y in self.highlight]))
        if len(self.points) > 1:
            polygon = QtGui.QPolygonF([
                QtCore.QPointF(rect.left() + x * rect.width() / self.image.width(),
                               rect.top() + y * rect.height() / self.image.height())
                for x, y in self.points])
            if len(self.points) == 4:
                painter.drawPolygon(polygon)
            else:
                painter.drawPolyline(polygon)
        for index, (x, y) in enumerate(self.points, 1):
            point = QtCore.QPointF(
                rect.left() + x * rect.width() / self.image.width(),
                rect.top() + y * rect.height() / self.image.height())
            painter.drawEllipse(point, 7, 7)
            painter.drawText(point + QtCore.QPointF(10, -8), str(index))


class TrayTeachWindow(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="tray-teach")
        self.future = self.completion = None
        self.pending_job = None
        self.preview_cancelled = False
        self.closing = False
        self.job_kind = ""
        self.settings = self.plane_view = None
        self.detail_sample = None
        self.preview_settings = self.last_view = None
        self.preview_due = None
        self.preview_error = self.geometry_error = ""
        self.filling = False
        self.profile_path = None
        self.profile_digest = ""
        self.profile_filename = ""
        self.save_target = None
        self.draft_reason = ""
        self.auto_source_attempts = {}
        self.auto_source_notices = {}
        self.saved_plane = None
        self.session_ready = False
        self.session_due = None
        self.last_session = None
        self.buttons = []
        self.pending_ids = []
        self.image_size = 640
        self.points = []
        self.next_preview = 0.
        self.setWindowTitle("Tray Teach — live inspection and teach files")
        self.resize(1560, 960)
        self.setStyleSheet("""
            QGroupBox { font-weight: 600; border: 1px solid #cbd2da;
                        border-radius: 6px; margin-top: 12px; padding: 12px 8px 8px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QLineEdit { min-height: 24px; }
            QPushButton { min-height: 28px; padding: 2px 8px; }
            QPushButton:checked { background: #d9eafa; color: #123b60;
                                  border: 1px solid #4783b5; border-radius: 4px; }
            QSplitter::handle { background: #cbd2da; }
            QLabel#viewHeading { color: #e1e7ed; background: #202a35;
                                padding: 7px 10px; font-weight: 600; }
        """)
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)
        title = QtWidgets.QLabel("Tray Teach")
        title.setFont(QtGui.QFont("Sans", 18, QtGui.QFont.Bold))
        header = QtWidgets.QHBoxLayout()
        header.addWidget(title)
        header.addWidget(QtWidgets.QLabel(
            "Live inspection & teach files\nRead-only poses · No motion"))
        header.addStretch()
        self.load_teach_button = self._button("Load Tray Teach…", self._load_tray)
        self.load_teach_button.setToolTip("Reopen a saved tray profile for preview or teaching")
        self.save_button = self._button("Save Tray Teach…", self._save)
        self.save_button.setToolTip(
            "Save edits to the loaded tray file; a new or renamed tray creates a new file")
        header.addWidget(self.load_teach_button)
        header.addWidget(self.save_button)
        root.addLayout(header)
        toggle_row = QtWidgets.QHBoxLayout()
        self.request_status = QtWidgets.QLabel("Disarmed — no tray pose service")
        self.request_status.setWordWrap(True)
        splitter = self.workspace_split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(6)
        root.addWidget(splitter, 1)
        scroll = self.settings_scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setMinimumWidth(340)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        sidebar = QtWidgets.QWidget()
        controls = QtWidgets.QVBoxLayout(sidebar)
        controls.setContentsMargins(0, 0, 8, 0)
        controls.setSpacing(10)
        self.sections = {}

        def group(title):
            box = QtWidgets.QGroupBox(title)
            form = QtWidgets.QFormLayout(box)
            form.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
            form.setRowWrapPolicy(QtWidgets.QFormLayout.WrapLongRows)
            form.setVerticalSpacing(7)
            self.sections[title] = box
            controls.addWidget(box)
            return form

        camera_form = group("1  Camera / calibration")
        identity = group("2  Tray / model")
        detection = group("3  Detection settings")
        geometry = group("4  Tray size filter — mm")
        position_form = group("5  Tray Detect Pose")
        plane_form = group("6  Reference plane — teach file")
        self.camera_prefix = QtWidgets.QLineEdit()
        self.camera_prefix.setPlaceholderText("Camera prefix, e.g. robot_camera")
        self.camera_prefix.textChanged.connect(self._prefix_edited)
        camera_row = QtWidgets.QHBoxLayout()
        camera_row.addWidget(self.camera_prefix, 1)
        camera_row.addWidget(self._button("Connect RGB", self._connect_camera))
        camera_form.addRow(camera_row)
        self.camera_status = QtWidgets.QLabel("No camera connected")
        self.camera_status.setWordWrap(True)
        camera_form.addRow(self.camera_status)
        self.camera_path = QtWidgets.QLineEdit()
        self.model_path = QtWidgets.QLineEdit()
        self._path_row(camera_form, "Calibration", self.camera_path, self._load_camera)
        self.name = QtWidgets.QLineEdit()
        self.name.setPlaceholderText("Tray name")
        identity.addRow("Tray name", self.name)
        self._path_row(identity, "YOLO model", self.model_path, self._load_model)
        self.model_status = QtWidgets.QLabel("No model loaded")
        identity.addRow("Verified task", self.model_status)
        self.position_label = QtWidgets.QLabel("Not recorded. No zero-joint default.")
        self.position_label.setWordWrap(True)
        position_form.addRow(self.position_label)
        self.position_button = self._button(
            "Record Current Joints as Tray Detect Pose", self._record_position)
        position_form.addRow(self.position_button)
        note = QtWidgets.QLabel(
            "Robot observation position for tray detection. Reads feedback only; never "
            "moves the robot. Save Tray Teach to store it. Optional for arming and pose "
            "requests; controller motion is separate.")
        note.setWordWrap(True)
        position_form.addRow(note)
        self.dimensions = {}
        for key, label in (("length_mm", "Length (mm)"), ("width_mm", "Width (mm)"),
                           ("tolerance_mm", "Tolerance ± (mm)")):
            widget = QtWidgets.QLineEdit()
            widget.setPlaceholderText("Required for size acceptance")
            geometry.addRow(label, widget)
            self.dimensions[key] = widget
            widget.textChanged.connect(self._edited)
        measurement_hint = QtWidgets.QLabel(
            "Leave these fields blank to measure first. Create the reference plane, then "
            "click a tray to read its width/X and length/Y. Enter these values and a "
            "tolerance when ready to filter detections.")
        measurement_hint.setWordWrap(True)
        geometry.addRow(measurement_hint)
        self.classes = QtWidgets.QListWidget()
        self.classes.setMinimumHeight(60)
        self.classes.setMaximumHeight(100)
        identity.addRow("Tray classes", self.classes)
        self.confidence, self.iou = QtWidgets.QLineEdit(), QtWidgets.QLineEdit()
        for widget, initial in ((self.confidence, .25), (self.iou, .7)):
            widget.setText(str(initial))
            widget.textChanged.connect(self._edited)
        detection.addRow("Confidence (0–1)", self.confidence)
        detection.addRow("Overlap IoU (0–1)", self.iou)
        self.maximum = QtWidgets.QLineEdit("100")
        detection.addRow("Max detections", self.maximum)
        self.maximum.textChanged.connect(self._edited)
        self.name.textChanged.connect(self._edited)
        self.classes.itemChanged.connect(self._edited)
        self.preview_toggle = QtWidgets.QPushButton("YOLO Detect: OFF")
        self.preview_toggle.setCheckable(True)
        self.preview_toggle.toggled.connect(self._toggle_yolo)
        toggle_row.addWidget(self.preview_toggle, 1)
        self.simulate_button = self._button("Simulate Trigger", self._simulate_trigger)
        self.simulate_button.setToolTip(
            "Run the real fresh-frame pose pipeline locally. Requires a saved profile and "
            "YOLO ON; Armed may be OFF. No robot commands.")
        toggle_row.addWidget(self.simulate_button, 1)
        self.armed_toggle = QtWidgets.QPushButton("Armed: OFF")
        self.armed_toggle.setCheckable(True)
        self.armed_toggle.setToolTip(
            "Expose /tray_detect/get_tray_pose for controller requests. No robot motion.")
        self.armed_toggle.toggled.connect(self._toggle_armed)
        toggle_row.addWidget(self.armed_toggle, 1)
        self.form_fields = [self.camera_prefix, self.name, self.classes,
                            self.confidence, self.iou, self.maximum,
                            self.preview_toggle, self.armed_toggle, *self.dimensions.values()]
        self.plane_label = QtWidgets.QLabel("Reference plane: not taught")
        self.plane_label.setWordWrap(True)
        plane_form.addRow(self.plane_label)
        self.snapshot_button = self._button("Capture 4-corner snapshot…", self._snapshot_plane)
        self.undo_button = self._button("Undo corner", self._undo)
        self.plane_button = self._button("Create reference plane", self._capture)
        self.cancel_plane_button = self._button("Cancel corner capture", self._discard_plane_draft)
        plane_form.addRow(self.snapshot_button)
        plane_form.addRow(self.undo_button)
        plane_form.addRow(self.plane_button)
        plane_form.addRow(self.cancel_plane_button)
        self.corner_status = QtWidgets.QLabel("No corner snapshot captured")
        self.corner_status.setWordWrap(True)
        plane_form.addRow(self.corner_status)
        instructions = QtWidgets.QLabel(
            "Capture the uncovered tray, then select four corners in the RGB pane. "
            "The panes hold that observation until Create or Cancel; streams keep running. "
            "Each corner uses the valid depth samples available; zero valid samples blocks it. "
            "Create the plane to enable measurements; dimensions and Item Teach position "
            "are not needed yet. Save the complete Tray Teach profile to keep the plane. "
            "A green four-corner outline marks the created or loaded plane.\n\n"
            "Origin: corner nearest base_link. X = short edge; Y = long edge, both inward. "
            "Re-teach after changing tray support height or tilt.")
        instructions.setWordWrap(True)
        plane_form.addRow(instructions)
        controls.addStretch()
        scroll.setWidget(sidebar)
        splitter.addWidget(scroll)
        visual = QtWidgets.QWidget()
        column = QtWidgets.QVBoxLayout(visual)
        column.setContentsMargins(4, 0, 0, 0)
        column.setSpacing(6)
        column.addLayout(toggle_row)
        column.addWidget(self.request_status)
        self.canvas = TrayCanvas()
        self.canvas.selecting = True
        self.canvas.clicked.connect(self._click)
        self.depth_canvas = TrayCanvas()
        self.depth_canvas.empty_text = "Waiting for synchronized registered depth"
        images = self.image_split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        images.setChildrenCollapsible(False)
        images.setHandleWidth(6)
        for title, canvas in (
                ("RGB — LIVE", self.canvas), ("Registered depth — LIVE", self.depth_canvas)):
            panel = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(panel)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(0)
            label = QtWidgets.QLabel(title)
            label.setWordWrap(True)
            label.setObjectName("viewHeading")
            layout.addWidget(label)
            layout.addWidget(canvas, 1)
            images.addWidget(panel)
            if canvas is self.canvas:
                self.rgb_status = label
            else:
                self.depth_status = label
        images.setStretchFactor(0, 1)
        images.setStretchFactor(1, 1)
        images.setSizes([560, 560])
        images.handle(1).setToolTip("Drag to resize the RGB and depth views")
        column.addWidget(images, 1)
        self.result_label = QtWidgets.QLabel("No tray pose selected")
        self.result_label.setWordWrap(True)
        column.addWidget(self.result_label)
        self.detail_label = QtWidgets.QLabel(
            "Click a tray to inspect its size; cameras stay live.")
        self.detail_label.setWordWrap(True)
        column.addWidget(self.detail_label)
        splitter.addWidget(visual)
        splitter.setSizes([390, 1170])
        splitter.setStretchFactor(1, 1)
        self.status = QtWidgets.QLabel(
            "Connect RGB and browse a model to preview. Selected files load automatically.")
        self.status.setWordWrap(True)
        root.addWidget(self.status)
        state = read_session(node.root)
        if state is not None:
            self.camera_path.setText(state["camera_filename"])
            self.profile_filename = state["profile_filename"]
            self.model_path.setText(state["model_path"])
            if state["schema_version"] == 1:
                self._fill_settings(state["settings"])
            else:
                self.filling = True
                for key, widget in self._draft_fields().items():
                    widget.setText(state["draft"][key])
                self.image_size = state["draft"]["image_size"]
                self.pending_ids = state["draft"]["class_ids"][:]
                self.filling = False
            self.classes.blockSignals(True)
            for identifier in self.pending_ids:
                entry = QtWidgets.QListWidgetItem(
                    f"{identifier}: saved selection (load model to verify)")
                entry.setData(QtCore.Qt.UserRole, identifier)
                entry.setFlags(entry.flags() | QtCore.Qt.ItemIsUserCheckable)
                entry.setCheckState(QtCore.Qt.Checked)
                self.classes.addItem(entry)
            self.classes.blockSignals(False)
            self.status.setText("Previous draft restored. Available calibration/model files "
                                "load automatically; Armed stays OFF.")
        self.last_session = state
        self.session_ready = True
        for widget in (self.camera_path, self.model_path):
            widget.textChanged.connect(self._schedule_remember)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(100)

    def _button(self, title, callback):
        button = QtWidgets.QPushButton(title)
        button.clicked.connect(callback)
        self.buttons.append(button)
        return button

    def _path_row(self, form, label, field, callback, *, button_text="Browse…"):
        field.setReadOnly(True)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(field, 1)
        row.addWidget(self._button(button_text, callback))
        form.addRow(label, row)

    def _message(self, text, *, error=False):
        self.status.setText(text)
        self.node.events.record("WARNING" if error else "INFO", "gui", text)

    def _edited(self, *_args):
        if self.filling:
            return
        self._schedule_remember()
        self.settings = None
        self.preview_settings = None
        self.node.yolo_enabled = False
        self._invalidate_preview()
        self._reset_inspection()
        self.preview_due = time.monotonic() + .3
        if hasattr(self, "result_label"):
            self.result_label.setText("Updating detection settings after typing…")

    def _prefix_edited(self):
        self._edited()

    def _connect_camera(self):
        try:
            prefix = validate_prefix(self.camera_prefix.text().strip())
        except ValueError as exc:
            self._message(str(exc), error=True)
            return
        self._edited()
        self._job(lambda: self.node.connect_camera(prefix),
                  lambda value: self._message(f"Connected /{value}"), "connect")

    def _toggle_yolo(self, enabled):
        self._edited()
        self.preview_toggle.setText("YOLO Detect: ON" if enabled else "YOLO Detect: OFF")
        self._refresh_preview_settings()

    def _invalidate_preview(self, *_args):
        if self.future is not None and self.job_kind == "preview":
            self.preview_cancelled = True
        self.node.invalidate()

    def _update_controls(self):
        exclusive = self.pending_job is not None or (
            self.future is not None and self.job_kind != "preview")
        for widget in self.buttons + self.form_fields:
            widget.setEnabled(not exclusive)
        self.plane_button.setEnabled(
            not exclusive and self.plane_view is not None and len(self.points) == 4)
        self.undo_button.setEnabled(not exclusive and bool(self.points))
        self.cancel_plane_button.setEnabled(not exclusive and self.plane_view is not None)
        self.canvas.selecting = not exclusive
        self.simulate_button.setEnabled(not exclusive and self.plane_view is None)
        self.armed_toggle.setEnabled(not exclusive and self.plane_view is None)
        self.save_button.setEnabled(
            not exclusive and self._save_ready())

    def _save_ready(self):
        try:
            validate_name(self.name.text().strip())
            updating = self.save_target is not None and (
                self.name.text().strip() == self.save_target.name)
            self.save_button.setToolTip(
                (f"Update {self.save_target.path.name}" if updating else
                 "Save a new named Tray Teach file in offline_teach/tray_teach/") +
                "; incomplete fields are saved as a draft. Create the plane to save corner edits.")
            return True
        except (ValueError, TypeError) as exc:
            self.save_button.setToolTip(str(exc))
            return False

    def _job(self, function, callback, kind):
        if self.closing or self.node.fatal_error:
            return
        if self.future is not None:
            if self.job_kind == "preview" and kind != "preview" and self.pending_job is None:
                self.pending_job = (function, callback, kind)
                if kind in ("simulate", "arm", "snapshot", "corners", "plane", "read_profile",
                            "save"):
                    self.preview_cancelled = True
                else:
                    self._invalidate_preview()
                self._update_controls()
                self._message("Finishing the current preview before the requested action…")
            return
        self.job_kind, self.completion = kind, callback
        self.preview_cancelled = False

        def run():
            with self.node.work_lock:
                return function()
        self.future = self.pool.submit(run)
        self._update_controls()

    def _choose(self, title, directory, pattern, remembered=""):
        initial = directory / remembered if remembered else directory
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, title, str(initial), pattern)
        return Path(path) if path else None

    def _load_camera(self):
        path = self._choose("Camera calibration", self.node.root / "calibration", "YAML (*.yaml)",
                            self.camera_path.text())
        if path is not None:
            if path.resolve().parent != (self.node.root / "calibration").resolve():
                self._message("Select a camera YAML inside calibration/", error=True)
                return
            self.camera_path.setText(path.name)
            self._start_camera_load(path)

    def _start_camera_load(self, path):
        self.auto_source_attempts["camera"] = str(path.absolute())
        self._edited()
        self.node.camera = self.node.plane = None

        def loaded(camera):
            self.camera_path.setText(camera.path.name)
            self.camera_prefix.setText(camera.settings.camera_prefix)
            self._message(f"Loaded camera calibration: {camera.path.name}")
        self._job(lambda: self.node.apply_camera(path), loaded, "camera")

    def _load_model(self):
        path = self._choose("YOLO model", self.node.root, "PyTorch (*.pt)", self.model_path.text())
        if path is not None:
            if str(path) != self.model_path.text():
                self.classes.clear()
                self.pending_ids = []
            self.model_path.setText(str(path))
            self._start_model_load(path)

    def _start_model_load(self, path):
        self.auto_source_attempts["model"] = str(path.absolute())
        self.model_path.setText(str(path.absolute()))
        self._edited()
        self.node.model = self.node.model_metadata = None
        self.model_status.setText("Loading model / reading classes…")
        self._job(lambda: self.node.inspect_model(path), self._model_loaded, "model")

    def _autoload_sources(self):
        if (self.closing or self.node.fatal_error or self.node.native.failed or
                self.future is not None or self.pending_job is not None
                or self.node.requests.busy):
            return
        for kind, field, directory, start in (
                ("camera", self.camera_path, self.node.root / "calibration",
                 self._start_camera_load),
                ("model", self.model_path, self.node.root, self._start_model_load)):
            value = field.text().strip()
            if not value:
                continue
            path = Path(value).expanduser()
            path = path if path.is_absolute() else directory / path
            key = str(path.absolute())
            loaded = (getattr(self.node.camera, "path", None) if kind == "camera" else
                      self.node.model.get("path") if self.node.model is not None else None)
            if loaded is not None and Path(loaded) == path:
                self.auto_source_attempts[kind] = key
            if self.auto_source_attempts.get(kind) == key:
                continue
            if not path.is_file():
                if self.auto_source_notices.get(kind) != key:
                    self.auto_source_notices[kind] = key
                    self._message(f"Selected {kind} file is unavailable: {path}", error=True)
                continue
            start(path)
            return  # One explicit operation; the next source uses the next free worker slot.

    def _model_loaded(self, metadata):
        restored_ids = (self._class_ids() if self.model_path.text() == self.node.model["path"]
                        else [])
        self.model_path.setText(self.node.model["path"])
        self.model_status.setText(metadata["task"])
        self.classes.blockSignals(True)
        self.classes.clear()
        for identifier, name in metadata["classes"].items():
            entry = QtWidgets.QListWidgetItem(f"{identifier}: {name}")
            entry.setData(QtCore.Qt.UserRole, int(identifier))
            entry.setFlags(entry.flags() | QtCore.Qt.ItemIsUserCheckable)
            entry.setCheckState(QtCore.Qt.Checked if int(identifier) in restored_ids
                                else QtCore.Qt.Unchecked)
            self.classes.addItem(entry)
        self.classes.blockSignals(False)
        self._schedule_remember()
        self.preview_toggle.setChecked(True)
        self._refresh_preview_settings()
        self._message(f"Loaded {metadata['task']} model. Preview shows all classes; "
                      "check classes to accept for tray poses.")

    def _record_position(self):
        try:
            position = self.node.capture_detect_pose()
        except (ValueError, RuntimeError) as exc:
            self._message(f"Tray Detect Pose not recorded: {exc}", error=True)
            return
        if self.node.position is not None and QtWidgets.QMessageBox.question(
            self, "Replace Tray Detect Pose?",
            "Replace all six saved Tray Detect Pose joints with this reading?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No,
        ) != QtWidgets.QMessageBox.Yes:
            return
        self._invalidate_preview()
        self.node.position = position
        self._show_position()
        self.node.events.record("INFO", "detect_pose_recorded", "Accepted six joints",
                                position=position)
        self._message("Recorded Tray Detect Pose. Save Tray Teach to store it. "
                      "No robot movement was requested.")

    def _show_position(self):
        if self.node.position is not None:
            joints = "  ".join(f"J{i + 1}: {math.degrees(q):+.3f}°"
                               for i, q in enumerate(self.node.position["positions_rad"]))
            self.position_label.setText(
                "Recorded on robot: " + self.node.position["robot_lan1_ip"] + "\n" + joints
                + "\nSaved in radians, joint1 through joint6.")
        else:
            self.position_label.setText("Not recorded. No zero-joint default.")

    def _load_tray(self):
        path = self._choose("Load Tray Teach", tray_directory(self.node.root), "YAML (*.yaml)",
                            self.profile_filename)
        if path is not None:
            self._job(lambda: open_document(path, self.node.root),
                      lambda result: self._open_tray(path, *result), "read_profile")

    def _open_tray(self, path, document, target):
        self._edited()
        draft = document["artifact_type"] == "tray_teach_draft"

        def loaded(profile):
            self.preview_toggle.setChecked(False)
            if draft:
                self._fill_draft(profile["form"])
                if self.node.model is not None:
                    self.model_path.setText(self.node.model["path"])
                    self._model_loaded(self.node.model_metadata)
                else:
                    self.model_status.setText("No verified model loaded")
                if self.node.camera is not None:
                    self.camera_path.setText(self.node.camera.path.name)
                    self.camera_prefix.setText(self.node.camera_prefix)
                try:
                    task = self.node.model["task"] if self.node.model else None
                    detection_profile(profile, task)
                    self.draft_reason = ""
                except ValueError as exc:
                    self.draft_reason = str(exc)
            else:
                self._model_loaded(self.node.model_metadata)
                self._fill_settings(profile["settings"])
                self.camera_path.setText(profile["camera_calibration"]["filename"])
                self.camera_prefix.setText(self.node.camera_prefix)
                self.draft_reason = ""
            self.profile_filename, self.profile_path = path.name, path
            self.profile_digest, self.save_target = target.yaml_sha256, target
            self.saved_plane = copy.deepcopy(profile["reference_plane"])
            self._show_position()
            self._refresh_preview_settings()
            self._remember()
            self._message(f"Loaded {'draft' if draft else 'complete profile'}: {path.name}. "
                          + (f"Still needed for detection: {self.draft_reason}. "
                             if self.draft_reason else
                             "Detection data ready; Simulate or Arm without another Save. ")
                          + "Save updates this file; renaming creates a new file.")

        def load():
            if file_sha256(path) != target.yaml_sha256:
                raise ValueError("Tray Teach file changed; load it again")
            result = (self.node.load_draft(path, target.yaml_sha256) if draft
                      else self.node.load_saved(path))
            if file_sha256(path) != target.yaml_sha256:
                raise ValueError("Tray Teach file changed while loading; reload it")
            return result
        self._job(load, loaded, "profile")

    def _fill_draft(self, form):
        self.filling = True
        self.camera_path.setText(form["camera_filename"])
        self.model_path.setText(form["model_path"])
        for key, widget in self._draft_fields().items():
            widget.setText(form["draft"][key])
        self.image_size = form["draft"]["image_size"]
        self.pending_ids = form["draft"]["class_ids"][:]
        self.classes.clear()
        for identifier in self.pending_ids:
            entry = QtWidgets.QListWidgetItem(
                f"{identifier}: saved selection (load model to verify)")
            entry.setData(QtCore.Qt.UserRole, identifier)
            entry.setFlags(entry.flags() | QtCore.Qt.ItemIsUserCheckable)
            entry.setCheckState(QtCore.Qt.Checked)
            self.classes.addItem(entry)
        self.filling = False
        self._edited()

    def _fill_settings(self, settings):
        self.filling = True
        self.name.setText(settings["name"])
        for key, value in settings["geometry"].items():
            self.dimensions[key].setText(str(value))
        self.confidence.setText(str(settings["yolo"]["confidence"]))
        self.iou.setText(str(settings["yolo"]["iou"]))
        self.maximum.setText(str(settings["yolo"]["max_detections"]))
        self.image_size = settings["yolo"]["image_size"]
        self.pending_ids = settings["yolo"]["class_ids"][:]
        for index in range(self.classes.count()):
            item = self.classes.item(index)
            item.setCheckState(QtCore.Qt.Checked if item.data(QtCore.Qt.UserRole) in
                               self.pending_ids else QtCore.Qt.Unchecked)
        self.filling = False
        self._edited()

    def _geometry(self):
        try:
            geometry = {key: float(widget.text()) for key, widget in self.dimensions.items()}
            validate_geometry(geometry)
            return geometry
        except ValueError as exc:
            raise ValueError("Enter valid length, width and tolerance in mm") from exc

    def _yolo(self):
        try:
            return {"confidence": float(self.confidence.text()), "iou": float(self.iou.text()),
                    "max_detections": int(self.maximum.text()), "image_size": self.image_size,
                    "class_ids": self._class_ids()}
        except ValueError as exc:
            raise ValueError("Enter valid confidence, IoU and integer detection cap") from exc

    def _form_settings(self):
        if self.node.model is None:
            raise ValueError("Select an available YOLO model; it loads automatically")
        task = self.node.model["task"]
        settings = {"name": self.name.text().strip(), "model_task": task,
                    "geometry_source": {"segment": "mask", "obb": "obb", "detect": "none"}[task],
                    "geometry": self._geometry(), "yolo": self._yolo()}
        validate_settings(settings)
        return settings

    def _class_ids(self):
        return [self.classes.item(i).data(QtCore.Qt.UserRole)
                for i in range(self.classes.count())
                if self.classes.item(i).checkState() == QtCore.Qt.Checked]

    def _draft_fields(self):
        return {"camera_prefix": self.camera_prefix, "name": self.name,
                **self.dimensions, "confidence": self.confidence, "iou": self.iou,
                "max_detections": self.maximum}

    def _schedule_remember(self, *_args):
        if self.session_ready and not self.filling:
            self.session_due = time.monotonic() + .3

    def _remember(self):
        if not self.session_ready:
            return
        self.session_due = None
        state = self._form_state()
        if state != self.last_session:
            write_session(self.node.root, state)
            self.last_session = state

    def _form_state(self):
        state = {"schema_version": 2, "profile_filename": self.profile_filename,
                 "camera_filename": self.camera_path.text(),
                 "item_filename": "",  # Reserved field in existing session/draft schemas.
                 "model_path": self.model_path.text(),
                 "draft": {key: widget.text() for key, widget in self._draft_fields().items()}}
        state["draft"].update(image_size=self.image_size, class_ids=self._class_ids())
        return state

    def _persist_session(self):
        try:
            self._remember()
        except (ValueError, OSError) as exc:
            self._message(f"Could not remember Tray Teach draft: {exc}", error=True)

    def _refresh_preview_settings(self):
        self.preview_due = None
        self.preview_settings = None
        self.node.yolo_enabled = False
        self.preview_error = self.geometry_error = ""
        if not self.preview_toggle.isChecked():
            return
        try:
            if self.node.model is None:
                raise ValueError("Waiting for the selected YOLO model to load")
            task = self.node.model["task"]
            yolo = self._yolo()
            accepted = yolo["class_ids"]
            yolo["class_ids"] = sorted(int(i) for i in self.node.model_metadata["classes"])
            try:
                geometry = self._geometry()
            except ValueError as exc:
                geometry, self.geometry_error = None, str(exc)
            settings = {"model_task": task,
                        "geometry_source": {"segment": "mask", "obb": "obb", "detect": "none"}[
                            task], "yolo": yolo, "accepted_class_ids": accepted,
                        "geometry": geometry}
            validate_preview(settings)
            self.preview_settings = settings
            self.node.yolo_enabled = True
        except (ValueError, TypeError) as exc:
            self.preview_error = str(exc)
        try:
            self.settings = self._form_settings()
        except ValueError:
            self.settings = None  # Session drafts never bypass complete-profile validation.

    def _trigger_settings(self):
        if self.profile_path is None or not self.profile_digest:
            raise ValueError("Save or load a complete Tray Teach profile first")
        if not self.preview_toggle.isChecked() or self.preview_settings is None:
            raise ValueError("Enable YOLO with valid settings before triggering or arming")
        return self._form_settings()

    def _style_armed(self, enabled):
        self.armed_toggle.blockSignals(True)
        self.armed_toggle.setChecked(enabled)
        self.armed_toggle.setText("Armed: ON" if enabled else "Armed: OFF")
        self.armed_toggle.setStyleSheet(
            "background:#b51f24;color:white;font-weight:bold;" if enabled else "")
        self.armed_toggle.blockSignals(False)

    def _toggle_armed(self, enabled):
        self._style_armed(enabled)
        if not enabled:
            self.node.requests.disarm()
            return
        try:
            settings = self._trigger_settings()
            path, digest = self.profile_path, self.profile_digest
            self._job(lambda: self.node.requests.arm(path, settings, digest),
                      lambda _: self._message("Armed: /tray_detect/get_tray_pose"), "arm")
        except (ValueError, OSError) as exc:
            self._style_armed(False)
            self._message(str(exc), error=True)

    def _simulate_trigger(self):
        try:
            settings = self._trigger_settings()
            path, digest = self.profile_path, self.profile_digest
            requested_at = (self.node.get_clock().now().nanoseconds, time.monotonic())

            def shown(value):
                response, view = value["response"], value["view"]
                if not response.success:
                    self._message(f"Simulate Trigger: {response.message}", error=True)
                    return
                self._show_view(view)
                self._show_detail(view, f"Last simulated request: {response.status} — "
                                  f"{response.message}", trigger=True)
                self._message(f"{response.status}: {response.message}. Preview continues live.")
            self._job(lambda: self.node.requests.simulate(path, settings, digest, requested_at),
                      shown, "simulate")
        except (ValueError, OSError) as exc:
            self._message(str(exc), error=True)

    def _snapshot_plane(self):
        self._discard_plane_draft()
        self._invalidate_preview()

        def captured(view):
            if view["generation"] != self.node.generation:
                raise ValueError("Corner snapshot was invalidated; capture it again")
            self.plane_view = view
            self.points.clear()
            self.detail_sample = None
            self._show_plane_view(view)
            self._message("Select four corners in the captured RGB pane, then Create or Cancel.")
        self._job(self.node.freeze_for_plane, captured, "snapshot")

    def _plane_click(self, x, y):
        if self.pending_job is not None or (
                self.future is not None and self.job_kind != "preview"):
            return
        if self.plane_view is not None and len(self.points) < 4:
            if self.plane_view["generation"] != self.node.generation:
                self._discard_plane_draft()
                return
            if any(math.hypot(x - px, y - py) < 3 for px, py in self.points):
                self._message("Choose a different corner", error=True)
                return
            self.points.append([x, y])
            self.canvas.update()
            self.corner_status.setText(f"{len(self.points)}/4 corners on captured observation")
            self._corner_evidence()

    def _click(self, x, y):
        if self.plane_view is not None:
            self._plane_click(x, y)
            return
        if self.last_view is None or self.last_view["generation"] != self.node.generation:
            return
        hits = []
        for item in self.last_view["result"].get("detections", []):
            polygon = QtGui.QPolygonF([QtCore.QPointF(*p) for p in item["polygon"]])
            if polygon.containsPoint(QtCore.QPointF(x, y), QtCore.Qt.WindingFill):
                rect = polygon.boundingRect()
                hits.append((rect.width() * rect.height(), -item["confidence"],
                             item["source_index"], item))
        if hits:
            item = min(hits, key=lambda hit: hit[:3])[3]
            self.canvas.highlight = item["polygon"]
            self.canvas.update()
            measured = (f"Reference plane: X/width {item['width_mm']:.1f} mm × "
                        f"Y/length {item['length_mm']:.1f} mm"
                        if "length_mm" in item else
                        "2D outline only; metric dimensions unavailable")
            origin = (" | base_link corner XYZ: " + ", ".join(
                f"{value * 1000:.1f}" for value in item["position"]) + " mm"
                if "position" in item else "")
            self._show_detail(self.last_view,
                              f"Last clicked tray: {item['class_name']} | {measured} | "
                              f"{item['reason']}{origin}")

    def _show_detail(self, view, summary, *, trigger=False):
        sample = view if trigger else {
            "generation": view["generation"], "rgb": {"stamp_ns": view["rgb"]["stamp_ns"]}}
        self.detail_sample = sample, summary
        self._update_detail()

    def _update_detail(self):
        if self.detail_sample is None or self.plane_view is not None:
            return
        view, summary = self.detail_sample
        if view["generation"] != self.node.generation:
            self.detail_sample = None
            self.detail_label.setText("Inspection invalidated; click a live tray again.")
            return
        if "trigger_binding" in view:
            try:
                self.node.requests.validate_view(view)
            except (ValueError, OSError, RuntimeError) as exc:
                self.detail_sample = None
                self.node.invalidate(str(exc))
                self.detail_label.setText(f"Simulated result invalidated: {exc}")
                return
        age = max(0., (self.node.get_clock().now().nanoseconds - view["rgb"]["stamp_ns"]) / 1e9)
        self.detail_label.setText(f"{summary} | observation {age:.1f} s ago | cameras LIVE")

    def _corner_evidence(self):
        view, pixels = self.plane_view, copy.deepcopy(self.points)

        def shown(result):
            if self.plane_view is not view or view["generation"] != self.node.generation:
                return
            details = [f"{s['index']}: {s['accepted']}/49 valid, "
                       f"{s['median_mm']} mm — {s['reason']}" for s in result["samples"]]
            text = "Corners: " + "; ".join(details)
            self._show_plane_view({**view, **result}, text)
        self._job(lambda: self.node.corner_preview(view, pixels), shown, "corners")

    def _undo(self):
        if self.points:
            self.points.pop()
            self.canvas.update()
            self.corner_status.setText(f"{len(self.points)}/4 corners on captured observation")
            self._corner_evidence()

    def _capture(self):
        if self.plane_view is None or len(self.points) != 4:
            self._message("Capture a plane snapshot and select exactly four corners first",
                          error=True)
            return
        view, points = self.plane_view, copy.deepcopy(self.points)

        def captured(plane):
            self._discard_plane_draft()
            self._message(
                f"Reference plane created; fit error {plane['max_error_mm']:.2f} mm. "
                "Click a live tray to measure it, then enter your size filters. "
                "Save Tray Teach to store it in the teach file.")

        def create():
            if self.plane_view is not view:
                raise ValueError("Corner snapshot was discarded; capture it again")
            return self.node.capture_plane(view, points)
        self._job(create, captured, "plane")

    def _discard_plane_draft(self, *_args):
        had_draft = self.plane_view is not None
        self.plane_view = None
        self.points.clear()
        self.canvas.points = []
        self.canvas.highlight = []
        self.corner_status.setText("No corner snapshot captured")
        if had_draft:
            self.detail_label.setText("Click a live tray to inspect its size.")
            if self.last_view is not None and self.last_view["generation"] == self.node.generation:
                self._display_view(self.last_view)
            else:
                for canvas in (self.canvas, self.depth_canvas):
                    canvas.image = None
                    canvas.empty_text = "Waiting for the next live observation"
                    canvas.update()
            self.next_preview = 0.

    def _show_plane_view(self, view, evidence=""):
        rgb = view["rgb"]
        self.canvas.points = self.points
        self.canvas.highlight = []
        self.canvas.show_frame(view.get("corner_overlay", rgb["rgb"]), rgb["width"], rgb["height"])
        self._show_depth(view)
        self.result_label.setText(f"Reference plane draft: {len(self.points)}/4 corners — "
                                  "select in RGB, then Create or Cancel")
        self.corner_status.setText(f"{len(self.points)}/4 corners on captured observation")
        self.detail_label.setText(
            evidence or "Each corner uses all valid depth samples available.")

    def _reset_inspection(self):
        self._discard_plane_draft()
        self.detail_sample = None
        self.detail_label.setText("Click a tray to inspect its size; cameras stay live.")
        self.last_view = None
        self.canvas.highlight = []
        self.canvas.update()

    def _save(self):
        try:
            validate_name(self.name.text().strip())
            form, target = self._form_state(), self.save_target
            try:
                settings = self._form_settings()
            except ValueError:
                settings = None
            position, plane = copy.deepcopy(self.node.position), copy.deepcopy(self.node.plane)
            camera, model = self.node.camera, copy.deepcopy(self.node.model)
            self.node.requests.disarm("Saving Tray Teach")

            def saved(result):
                path, document, self.save_target, self.draft_reason = result
                self.settings, self.profile_path = settings, path
                self.profile_digest = self.save_target.yaml_sha256
                self.saved_plane = copy.deepcopy(plane)
                self.profile_filename = path.name
                self._remember()
                state = "draft" if self.draft_reason else "complete profile"
                detail = f" Still needed: {self.draft_reason}." if self.draft_reason else ""
                self._message(f"Saved {state}: {path}.{detail} "
                              "Save updates this file; renaming creates a new file.")

            def save():
                reason = ""
                try:
                    self.node.validate_sources()
                    if form["draft"]["camera_prefix"].strip() != camera.settings.camera_prefix:
                        raise ValueError("Connect the calibrated camera before detection")
                except ValueError as exc:
                    reason = str(exc)
                return save_document(form, settings, position, plane, camera, model,
                                     self.node.root, target=target, readiness_error=reason)
            self._job(save, saved, "save")
        except (ValueError, OSError, RuntimeError) as exc:
            self._message(str(exc), error=True)

    def _show_view(self, view):
        if view["generation"] != self.node.generation:
            return
        if "trigger_binding" in view:
            self.node.requests.validate_view(view)
        self.node.accept_view(view)
        self.last_view = view
        if self.plane_view is None:
            self._display_view(view)

    def _display_view(self, view):
        self.canvas.highlight = []
        rgb, result = view["rgb"], view["result"]
        self.canvas.show_frame(view["overlay"], rgb["width"], rgb["height"])
        self._show_depth(view)
        age = max(0., (self.node.get_clock().now().nanoseconds - rgb["stamp_ns"]) / 1e9)
        selected = result.get("selected")
        if selected is None:
            self.result_label.setText(
                result.get("reason", result.get("error", "YOLO preview OFF")))
        else:
            xyz = ", ".join(f"{v * 1000:.1f}" for v in selected["position"])
            self.result_label.setText(
                f"Selected tray: {selected['length_mm']:.1f} × {selected['width_mm']:.1f} mm | "
                f"base_link origin XYZ: {xyz} mm | snapshot age {age:.2f} s")
        rejected = [f"{d['source_index']}: {d['reason']}" for d in result.get("detections", [])
                    if not d["valid"]]
        self.result_label.setToolTip("\n".join(rejected))

    def _show_depth(self, view):
        rgb = view["rgb"]
        if view.get("depth_overlay"):
            self.depth_canvas.show_frame(view["depth_overlay"], rgb["width"], rgb["height"])
        else:
            self.depth_canvas.image = None
            self.depth_canvas.update()
        self.depth_status.setText("Registered depth — LIVE | " + (
            view.get("depth_error", "") or "200–1000 mm; black = outside range"))

    def _tick(self):
        if self.closing:
            return
        if self.session_due is not None and time.monotonic() >= self.session_due:
            self._persist_session()
        if self.preview_due is not None and time.monotonic() >= self.preview_due:
            self._refresh_preview_settings()
        if self.future is not None and self.future.done():
            future, callback, kind = self.future, self.completion, self.job_kind
            obsolete = kind == "preview" and self.preview_cancelled
            self.future = self.completion = None
            if kind == "preview":
                self.next_preview = time.monotonic() + 1.
            try:
                result = future.result()
                if not obsolete:
                    callback(result)
            except Exception as exc:
                terminal = self.node.native.failed or isinstance(exc, RuntimeError)
                if terminal or not obsolete:
                    self.node.selected = None
                    self._message(str(exc), error=True)
                    if kind == "preview":
                        self.result_label.setText(f"No current tray pose: {exc}")
                if terminal:
                    self.node.fatal_error = str(exc)
        if self.node.fatal_error:
            self.timer.stop()
            QtWidgets.QMessageBox.critical(self, "Tray Teach stopped", self.node.fatal_error)
            self.close()
            return
        self._update_detail()
        if self.plane_view is not None and self.plane_view["generation"] != self.node.generation:
            self._discard_plane_draft()
            self.corner_status.setText("Snapshot invalidated — capture the corners again")
        if self.future is None and self.pending_job is not None:
            job, self.pending_job = self.pending_job, None
            self._job(*job)
        self._autoload_sources()
        busy = self.future is not None
        requests = self.node.requests
        arming = ((self.future is not None and self.job_kind == "arm") or
                  (self.pending_job is not None and self.pending_job[2] == "arm"))
        if not arming:
            self._style_armed(requests.service is not None)
        self.request_status.setText(
            ("Request running | " if requests.busy else "") + requests.status)
        self._update_controls()
        prefix = self.node.camera_prefix
        self.camera_status.setText(self.node.camera_status + (
            f"\n/{prefix}/color/image_raw\n/{prefix}/depth/image_raw" if prefix else ""))
        self.camera_status.setToolTip("\n".join(f"/{prefix}/{topic}" for topic in (
            "color/image_raw", "depth/image_raw", "color/camera_info", "depth/camera_info"))
            if prefix else "Connect RGB first")
        error = self.preview_error or (
            "Size filter inactive: " + self.geometry_error if self.geometry_error else "")
        mode = ("YOLO paused: " + self.preview_error if self.preview_error else
                "YOLO ON" if self.preview_toggle.isChecked() else "YOLO OFF — raw RGB")
        metric = "" if self.last_view is None else self.last_view["metric_error"]
        cloud = self.node.rviz.status()
        detail = "\n".join(text for text in (metric, error) if text)
        self.rgb_status.setText(
            f"RGB /{prefix or 'not connected'} — LIVE | {mode}\n{detail}\n"
            f"RViz: {cloud['status']} — {cloud['point_count']} voxels {cloud['reason']}")
        if self.plane_view is not None:
            age = max(0., (self.node.get_clock().now().nanoseconds -
                           self.plane_view["rgb"]["stamp_ns"]) / 1e9)
            self.rgb_status.setText(
                f"RGB — CAPTURED {age:.1f} s ago | select four corners\n"
                "Create or Cancel returns to live view; camera streams keep running")
            self.depth_status.setText("Depth — CAPTURED | matching corner sample evidence")
        if self.node.plane is not None:
            stored = "saved in teach file" if self.node.plane == self.saved_plane else \
                "not saved — Save Tray Teach"
            self.plane_label.setText(
                f"Reference plane: GREEN / ready | {stored}\n"
                f"base_link | fit error {self.node.plane['max_error_mm']:.2f} mm")
            self.plane_label.setStyleSheet("color: #187a29")
        else:
            self.plane_label.setText("Reference plane: not taught")
            self.plane_label.setStyleSheet("")
        if (busy or requests.busy or prefix is None
                or prefix != self.camera_prefix.text().strip()):
            return
        if time.monotonic() < self.next_preview:
            return
        self.next_preview = time.monotonic() + 1.
        settings = (copy.deepcopy(self.preview_settings)
                    if self.preview_toggle.isChecked() else None)
        generation = self.node.generation
        self._job(lambda: self.node.preview(settings, generation=generation),
                  self._show_view, "preview")

    def closeEvent(self, event):
        if self.closing:
            event.accept()
            return
        self._persist_session()
        self.closing = True
        self._discard_plane_draft()
        self.timer.stop()
        self.pending_job = None
        self.node.close()
        self.pool.shutdown(wait=True, cancel_futures=True)
        event.accept()


def main(args=None):
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = executor = thread = window = None
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous:
        signal.signal(sig, lambda *_: app.quit())
    try:
        node = TrayTeachNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        window = TrayTeachWindow(node)
        window.show()
        app.exec_()
        if node.fatal_error:
            raise RuntimeError(node.fatal_error)
    finally:
        if window is not None:
            window.close()
        elif node is not None:
            node.close()
        if executor is not None:
            executor.shutdown(timeout_sec=2)
        if thread is not None:
            thread.join(timeout=2)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
