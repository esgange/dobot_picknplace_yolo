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
    copy_teach_position, read_session, save_profile, tray_directory, validate_settings,
    write_session)
from .node import TrayTeachNode


class TrayCanvas(QtWidgets.QWidget):
    clicked = QtCore.Signal(float, float)

    def __init__(self):
        super().__init__()
        self.image = None
        self.points = []
        self.selecting = False
        self.setMinimumSize(480, 360)
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
                             "Load camera calibration to begin")
            return
        rect = self.image_rect()
        painter.drawImage(rect, self.image)
        painter.setPen(QtGui.QPen(QtGui.QColor("cyan"), 2))
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
        self.job_kind = ""
        self.settings = self.frozen = None
        self.profile_path = None
        self.profile_filename = ""
        self.buttons = []
        self.pending_ids = []
        self.image_size = 640
        self.points = []
        self.next_preview = 0.
        self.setWindowTitle("Tray Teach")
        self.resize(1500, 900)
        root = QtWidgets.QVBoxLayout(self)
        title = QtWidgets.QLabel("Tray Teach")
        title.setFont(QtGui.QFont("Sans", 18, QtGui.QFont.Bold))
        header = QtWidgets.QHBoxLayout()
        header.addWidget(title)
        header.addStretch()
        self.load_teach_button = self._button("Load Tray Teach…", self._load_tray)
        self.load_teach_button.setToolTip("Reopen a saved tray profile for preview or teaching")
        self.save_button = self._button("Save Tray Teach…", self._save)
        self.save_button.setToolTip("Save a new timestamped tray YAML and paired model")
        header.addWidget(self.load_teach_button)
        header.addWidget(self.save_button)
        root.addLayout(header)
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        root.addWidget(splitter, 1)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(390)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        sidebar = QtWidgets.QWidget()
        controls = QtWidgets.QVBoxLayout(sidebar)
        form = QtWidgets.QFormLayout()
        controls.addLayout(form)
        self.camera_path = QtWidgets.QLineEdit()
        self.model_path = QtWidgets.QLineEdit()
        self.item_path = QtWidgets.QLineEdit()
        self._path_row(form, "Calibration", self.camera_path, self._load_camera)
        self._path_row(form, "YOLO model", self.model_path, self._load_model)
        self._path_row(form, "Item Home", self.item_path, self._copy_position)
        self.position_label = QtWidgets.QLabel("Tray Teach Position: not copied")
        self.position_label.setWordWrap(True)
        controls.addWidget(self.position_label)
        note = QtWidgets.QLabel(
            "Tray Teach Position is a copied teaching reference. This node sends no robot "
            "commands. Controller Home remains in Item Teach.")
        note.setWordWrap(True)
        controls.addWidget(note)
        self.name = QtWidgets.QLineEdit()
        form.addRow("Tray name", self.name)
        self.dimensions = {}
        for key, label in (("length_mm", "Length (mm)"), ("width_mm", "Width (mm)"),
                           ("tolerance_mm", "Tolerance ± (mm)")):
            widget = QtWidgets.QDoubleSpinBox()
            widget.setRange(0, 10000)
            widget.setDecimals(2)
            form.addRow(label, widget)
            self.dimensions[key] = widget
            widget.valueChanged.connect(self._edited)
        self.classes = QtWidgets.QListWidget()
        self.classes.setMaximumHeight(120)
        form.addRow("Tray classes", self.classes)
        self.confidence, self.iou = QtWidgets.QDoubleSpinBox(), QtWidgets.QDoubleSpinBox()
        for widget, initial in ((self.confidence, .25), (self.iou, .7)):
            widget.setRange(.01, 1.)
            widget.setSingleStep(.05)
            widget.setValue(initial)
            widget.valueChanged.connect(self._edited)
        form.addRow("Confidence", self.confidence)
        form.addRow("IoU", self.iou)
        self.maximum = QtWidgets.QSpinBox()
        self.maximum.setRange(1, 1000)
        self.maximum.setValue(100)
        form.addRow("Max detections", self.maximum)
        self.maximum.valueChanged.connect(self._edited)
        self.name.textChanged.connect(self._edited)
        self.classes.itemChanged.connect(self._edited)
        self.apply_button = self._button("Apply && Preview", self._apply)
        controls.addWidget(self.apply_button)
        self.preview_toggle = QtWidgets.QCheckBox("YOLO preview (up to 1 Hz)")
        self.preview_toggle.toggled.connect(lambda _checked: self.node.invalidate())
        controls.addWidget(self.preview_toggle)
        self.form_fields = [self.name, self.classes, self.confidence, self.iou, self.maximum,
                            self.preview_toggle, *self.dimensions.values()]
        self.plane_label = QtWidgets.QLabel("Reference plane: not taught")
        self.plane_label.setWordWrap(True)
        controls.addWidget(self.plane_label)
        instructions = QtWidgets.QLabel(
            "Keep the robot and uncovered tray still, then freeze a fresh view. Click four "
            "distinct corners on its "
            "reference surface in any order, then create the plane. Depth is used only here.\n\n"
            "Detection measures on that saved plane. The corner nearest base_link is the "
            "origin; red +X and green +Y follow adjacent edges into the tray. The valid tray "
            "nearest the image center wins.")
        instructions.setWordWrap(True)
        controls.addWidget(instructions)
        controls.addStretch()
        scroll.setWidget(sidebar)
        splitter.addWidget(scroll)
        visual = QtWidgets.QWidget()
        column = QtWidgets.QVBoxLayout(visual)
        actions = QtWidgets.QHBoxLayout()
        column.addLayout(actions)
        self.freeze_button = self._button("Freeze for 4 corners", self._freeze)
        self.undo_button = self._button("Undo corner", self._undo)
        self.plane_button = self._button("Create reference plane", self._capture)
        self.resume_button = self._button("Resume live", self._resume)
        for button in (self.freeze_button, self.undo_button,
                       self.plane_button, self.resume_button):
            actions.addWidget(button)
        self.canvas = TrayCanvas()
        self.canvas.clicked.connect(self._click)
        column.addWidget(self.canvas, 1)
        self.result_label = QtWidgets.QLabel("No tray pose selected")
        self.result_label.setWordWrap(True)
        column.addWidget(self.result_label)
        splitter.addWidget(visual)
        splitter.setSizes([400, 1100])
        splitter.setStretchFactor(1, 1)
        self.status = QtWidgets.QLabel(
            "Load a camera calibration, a trusted model and Item Teach.")
        self.status.setWordWrap(True)
        root.addWidget(self.status)
        state = read_session(node.root)
        if state is not None:
            self.camera_path.setText(state["camera_filename"])
            self.item_path.setText(state["item_filename"])
            self.profile_filename = state["profile_filename"]
            self.model_path.setText(state["model_path"])
            self._fill_settings(state["settings"])
            self.status.setText("Previous values restored as prefill. Explicitly load inputs.")
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(100)

    def _button(self, title, callback):
        button = QtWidgets.QPushButton(title)
        button.clicked.connect(callback)
        self.buttons.append(button)
        return button

    def _path_row(self, form, label, field, callback):
        field.setReadOnly(True)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(field, 1)
        row.addWidget(self._button("Load…", callback))
        form.addRow(label, row)

    def _message(self, text, *, error=False):
        self.status.setText(text)
        self.node.events.record("WARNING" if error else "INFO", "gui", text)

    def _edited(self, *_args):
        self.settings = None
        self.node.invalidate()
        if hasattr(self, "preview_toggle"):
            self.preview_toggle.setChecked(False)
        if hasattr(self, "result_label"):
            self.result_label.setText("Settings changed; Apply & Preview for a new tray pose")

    def _job(self, function, callback, kind):
        if self.future is not None:
            return
        self.job_kind, self.completion = kind, callback
        for widget in self.buttons + self.form_fields:
            widget.setEnabled(False)
        self.future = self.pool.submit(function)

    def _choose(self, title, directory, pattern):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, title, str(directory), pattern)
        return Path(path) if path else None

    def _trust(self):
        return QtWidgets.QMessageBox.question(
            self, "Load trusted model", "PyTorch model loading executes serialized model code. "
            "Do you trust the selected local model?", QtWidgets.QMessageBox.Yes |
            QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No) == QtWidgets.QMessageBox.Yes

    def _load_camera(self):
        path = self._choose("Camera calibration", self.node.root / "calibration", "YAML (*.yaml)")
        if path is not None:
            self._edited()
            self._resume()
            self._job(lambda: self.node.apply_camera(path),
                      lambda camera: self.camera_path.setText(camera.path.name), "camera")

    def _load_model(self):
        path = self._choose("YOLO model", self.node.root, "PyTorch (*.pt)")
        if path is not None and self._trust():
            self._edited()
            self._job(lambda: self.node.inspect_model(path), self._model_loaded, "model")

    def _model_loaded(self, metadata):
        restored_ids = (self.pending_ids if self.model_path.text() == self.node.model["path"]
                        else [])
        self.model_path.setText(self.node.model["path"])
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
        self._message(f"Loaded {metadata['task']} model. Select tray classes and Apply & Preview.")

    def _copy_position(self):
        path = self._choose("Copy Item Teach Home", self.node.root / "offline_teach/item_teach",
                            "YAML (*.yaml)")
        if path is not None:
            def loaded(position):
                self.node.position = position
                self.node.invalidate()
                self.item_path.setText(path.name)
                self._show_position()
                self._message("Copied Tray Teach Position. Item Teach is no longer needed.")
            self._job(lambda: copy_teach_position(path, self.node.root), loaded, "position")

    def _show_position(self):
        if self.node.position is not None:
            joints = ", ".join(f"{math.degrees(q):.2f}°"
                               for q in self.node.position["positions_rad"])
            self.position_label.setText(f"Tray Teach Position — J1…J6:\n{joints}")

    def _load_tray(self):
        path = self._choose("Load Tray Teach", tray_directory(self.node.root), "YAML (*.yaml)")
        if path is not None and self._trust():
            self._edited()
            self._resume()

            def loaded(profile):
                self._model_loaded(self.node.model_metadata)
                self._fill_settings(profile["settings"])
                self.camera_path.setText(profile["camera_calibration"]["filename"])
                self.item_path.clear()
                self.profile_filename = path.name
                self.profile_path = path
                self._show_position()
                self._apply()
                self._message("Tray loaded independently of Item Teach; preview enabled.")
            self._job(lambda: self.node.load_saved(path), loaded, "profile")

    def _fill_settings(self, settings):
        self.name.setText(settings["name"])
        for key, value in settings["geometry"].items():
            self.dimensions[key].setValue(value)
        self.confidence.setValue(settings["yolo"]["confidence"])
        self.iou.setValue(settings["yolo"]["iou"])
        self.maximum.setValue(settings["yolo"]["max_detections"])
        self.image_size = settings["yolo"]["image_size"]
        self.pending_ids = settings["yolo"]["class_ids"][:]
        for index in range(self.classes.count()):
            item = self.classes.item(index)
            item.setCheckState(QtCore.Qt.Checked if item.data(QtCore.Qt.UserRole) in
                               self.pending_ids else QtCore.Qt.Unchecked)

    def _form_settings(self):
        if self.node.model is None:
            raise ValueError("Explicitly load a trusted YOLO model")
        task = self.node.model["task"]
        settings = {"name": self.name.text().strip(), "model_task": task,
                    "geometry_source": {"segment": "mask", "obb": "obb", "detect": "none"}[task],
                    "geometry": {key: widget.value() for key, widget in self.dimensions.items()},
                    "yolo": {"confidence": self.confidence.value(), "iou": self.iou.value(),
                             "max_detections": self.maximum.value(), "image_size": self.image_size,
                             "class_ids": [self.classes.item(i).data(QtCore.Qt.UserRole)
                                           for i in range(self.classes.count())
                                           if self.classes.item(i).checkState() ==
                                           QtCore.Qt.Checked]}}
        validate_settings(settings)
        return settings

    def _remember(self):
        if self.settings is not None:
            self.pending_ids = self.settings["yolo"]["class_ids"][:]
            write_session(self.node.root, {
                "schema_version": 1, "profile_filename": self.profile_filename,
                "camera_filename": self.camera_path.text(), "item_filename": self.item_path.text(),
                "model_path": self.model_path.text(), "settings": self.settings})

    def _apply(self):
        try:
            self.settings = self._form_settings()
            self.node.invalidate()
            self._remember()
            self.preview_toggle.setChecked(True)
            self._message("Preview enabled. Green trays pass dimensions; red trays are rejected.")
        except (ValueError, OSError) as exc:
            self._message(str(exc), error=True)

    def _freeze(self):
        try:
            self.node.invalidate()
            self.frozen = self.node.freeze_for_plane()
            self.points.clear()
            self.canvas.points = self.points
            self.canvas.selecting = True
            rgb = self.frozen["rgb"]
            self.canvas.show_frame(rgb["rgb"], rgb["width"], rgb["height"])
            self._message("Frozen RGB/depth snapshot. Click four surface corners in any order.")
        except Exception as exc:
            self._message(str(exc), error=True)

    def _click(self, x, y):
        if self.frozen is not None and self.future is None and len(self.points) < 4:
            if any(math.hypot(x - px, y - py) < 3 for px, py in self.points):
                self._message("Choose a different corner", error=True)
                return
            self.points.append([x, y])
            self.canvas.update()
            self._message(f"Selected {len(self.points)}/4 corners on the frozen snapshot")

    def _undo(self):
        if self.points:
            self.points.pop()
            self.canvas.update()

    def _capture(self):
        if self.frozen is None or len(self.points) != 4:
            self._message("Freeze the view and click exactly four corners first", error=True)
            return
        view, points = self.frozen, copy.deepcopy(self.points)

        def captured(plane):
            self._resume()
            self._message(
                f"Reference plane created; maximum fit error {plane['max_error_mm']:.2f} mm")
        self._job(lambda: self.node.capture_plane(view, points), captured, "plane")

    def _resume(self):
        self.frozen = None
        self.points.clear()
        self.canvas.points = self.points
        self.canvas.selecting = False
        self.canvas.update()

    def _save(self):
        try:
            settings = self._form_settings()
            self.node.validate_sources()
            if self.node.position is None or self.node.plane is None:
                raise ValueError("Copy a Tray Teach Position and teach its reference plane first")
            position, plane = copy.deepcopy(self.node.position), copy.deepcopy(self.node.plane)
            camera, model = self.node.camera, copy.deepcopy(self.node.model)

            def saved(path):
                self.settings, self.profile_path = settings, path
                self.profile_filename = path.name
                self._remember()
                self._message(f"Saved new tray profile and paired model: {path}")
            self._job(lambda: save_profile(
                settings, position, plane, camera, model["path"], model["sha256"], self.node.root),
                saved, "save")
        except (ValueError, OSError, RuntimeError) as exc:
            self._message(str(exc), error=True)

    def _show_view(self, view):
        if self.frozen is not None or view["generation"] != self.node.generation:
            return
        rgb, result = view["rgb"], view["result"]
        self.canvas.show_frame(view["overlay"], rgb["width"], rgb["height"])
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

    def _tick(self):
        if self.future is not None and self.future.done():
            future, callback, kind = self.future, self.completion, self.job_kind
            self.future = self.completion = None
            if kind == "preview":
                self.next_preview = time.monotonic() + 1.
            try:
                callback(future.result())
            except Exception as exc:
                self.node.selected = None
                self._message(str(exc), error=True)
                if self.node.native.failed or isinstance(exc, RuntimeError):
                    self.node.fatal_error = str(exc)
                if kind == "preview":
                    self.result_label.setText(f"No current tray pose: {exc}")
        if self.node.fatal_error:
            self.timer.stop()
            QtWidgets.QMessageBox.critical(self, "Tray Teach stopped", self.node.fatal_error)
            self.close()
            return
        busy = self.future is not None
        for widget in self.buttons + self.form_fields:
            widget.setEnabled(not busy)
        self.plane_button.setEnabled(
            not busy and self.frozen is not None and len(self.points) == 4)
        self.undo_button.setEnabled(not busy and bool(self.points))
        self.save_button.setEnabled(
            not busy and self.node.plane is not None and self.node.position is not None
            and self.node.model is not None)
        if self.node.plane is not None:
            self.plane_label.setText(
                f"Reference plane: base_link | fit error {self.node.plane['max_error_mm']:.2f} mm")
        else:
            self.plane_label.setText("Reference plane: not taught")
        if busy or self.frozen is not None or self.node.camera is None:
            return
        if time.monotonic() < self.next_preview:
            return
        self.next_preview = time.monotonic() + 1.
        settings = copy.deepcopy(self.settings) if self.preview_toggle.isChecked() else None
        self._job(lambda: self.node.preview(settings), self._show_view, "preview")

    def closeEvent(self, event):
        self.timer.stop()
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
