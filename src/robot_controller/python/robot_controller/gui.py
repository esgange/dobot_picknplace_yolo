"""Qt client for Robot Controller; contains no Dobot command clients."""

from collections import deque
import os
import signal
import sys
import threading
import time

from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from item_perception_yolo.platform_teach_core import workspace_root
from robot_controller_interfaces.action import AutoRun, GoHome, PickItem, PlaceItem
from robot_controller_interfaces.msg import ControllerStatus
from robot_controller_interfaces.srv import Command, Configure, Preview, SetGlobalSpeed

from .ui_state import load_state, save_state
from .feedback import FEEDBACK_MAX_AGE_SEC
from .placement import validate_target
from .operator_ui import acquisition_paused, button_policy, presentation


class GuiNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("robot_controller_gui")
        self.root = workspace_root()
        self.prefill = load_state(self.root / "logs/robot_controller/last_session.json")
        self._status_sample = None
        self.operator_logs = deque(maxlen=1000)
        self.operator_log_lock = threading.Lock()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            ControllerStatus, "/robot_controller/status", self._status, qos)
        log_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/robot_controller/operator_log", self._operator_log, log_qos)
        self.service_clients = {
            "configure": self.create_client(Configure, "/robot_controller/configure"),
            "recover": self.create_client(Command, "/robot_controller/recover"),
            "pause": self.create_client(Command, "/robot_controller/pause"),
            "continue": self.create_client(Command, "/robot_controller/continue"),
            "stop": self.create_client(Command, "/robot_controller/stop"),
            "return_item": self.create_client(Command, "/robot_controller/return_item"),
            "speed": self.create_client(
                SetGlobalSpeed, "/robot_controller/set_global_speed"),
            "preview": self.create_client(Preview, "/robot_controller/preview_v2"),
        }
        self.action_clients = {
            "place": ActionClient(self, PlaceItem, "/robot_controller/place_item"),
            "home": ActionClient(self, GoHome, "/robot_controller/go_home"),
            "pick": ActionClient(self, PickItem, "/robot_controller/pick_item"),
            "auto_run": ActionClient(self, AutoRun, "/robot_controller/auto_run"),
        }

    def _status(self, message):
        self._status_sample = (message, time.monotonic())

    @property
    def status(self):
        sample = self._status_sample
        if sample is None:
            return None
        message, received = sample
        stamp = message.header.stamp
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age = (self.get_clock().now().nanoseconds - source_ns) / 1e9
        if (source_ns <= 0 or not 0 <= age <= FEEDBACK_MAX_AGE_SEC
                or time.monotonic() - received > FEEDBACK_MAX_AGE_SEC):
            return None
        return message

    def _operator_log(self, message):
        with self.operator_log_lock:
            self.operator_logs.append(message.data)

    def take_operator_logs(self):
        with self.operator_log_lock:
            values = tuple(self.operator_logs)
            self.operator_logs.clear()
        return values


class ControllerWindow(QtWidgets.QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.pending = {}
        self.pending_goal = None
        self.goal_handle = None
        self.result_future = None
        self.feedback_message = ""
        self.preview_mode = False
        self.preview_clear_future = None
        self.saved_selection = None
        self.pause_requested_locally = False
        self.return_requested_locally = False
        self.speed_pending_percent = None
        self.speed_syncing = False
        self.setWindowTitle("Robot Controller")
        self.resize(1050, 445)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        header = QtWidgets.QHBoxLayout()
        self.robot_panel, robot_layout = self._status_panel("Robot status")
        self.status = QtWidgets.QLabel("OFFLINE")
        self.status.setTextFormat(QtCore.Qt.PlainText)
        self.status.setAlignment(QtCore.Qt.AlignCenter)
        self.status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.status.setMinimumWidth(0)
        self.status.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        robot_layout.addWidget(self.status, 1)
        self.status_detail = QtWidgets.QLabel()
        self.status_detail.setTextFormat(QtCore.Qt.PlainText)
        self.status_detail.setWordWrap(True)
        self.status_detail.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.status_detail.setStyleSheet("font-size:13px;color:#edf3f8")
        robot_layout.addWidget(self.status_detail)
        self.gripper_panel, gripper_layout = self._status_panel("Gripper status")
        indicators = QtWidgets.QHBoxLayout()
        self.gripper_leds, self.gripper_values = {}, {}
        for channel, name in ((1, "Suction"), (12, "Finger open")):
            column = QtWidgets.QVBoxLayout()
            label = QtWidgets.QLabel(f"DI{channel} · {name}")
            label.setAlignment(QtCore.Qt.AlignCenter)
            led = QtWidgets.QLabel()
            led.setFixedSize(24, 24)
            led.setAccessibleName(f"DI{channel} {name}")
            value = QtWidgets.QLabel("UNKNOWN")
            value.setAlignment(QtCore.Qt.AlignCenter)
            column.addWidget(label)
            column.addWidget(led, 0, QtCore.Qt.AlignCenter)
            column.addWidget(value)
            indicators.addLayout(column, 1)
            self.gripper_leds[channel], self.gripper_values[channel] = led, value
        gripper_layout.addLayout(indicators, 1)
        header.addWidget(self.robot_panel, 3)
        header.addWidget(self.gripper_panel, 3)

        self.teach_panel = QtWidgets.QGroupBox("Teach files")
        self.teach_panel.setMinimumWidth(300)
        self.teach_panel.setMaximumWidth(420)
        self.teach_panel.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                                       QtWidgets.QSizePolicy.Maximum)
        teach = QtWidgets.QGridLayout(self.teach_panel)
        self.teach_browse_buttons = []
        self.item_path = QtWidgets.QLineEdit()
        self.bin_path = QtWidgets.QLineEdit()
        self.tray_path = QtWidgets.QLineEdit()
        if node.prefill:
            self.item_path.setText(str(
                node.root / "offline_teach/item_teach" / node.prefill["item"]))
            if node.prefill["bin"]:
                self.bin_path.setText(str(
                    node.root / "offline_teach/bin_teach" / node.prefill["bin"]))
            if node.prefill["tray"]:
                self.tray_path.setText(str(
                    node.root / "offline_teach/tray_teach" / node.prefill["tray"]))
        for index, (label, edit, directory) in enumerate((
                ("Item Teach", self.item_path, "offline_teach/item_teach"),
                ("Bin Teach", self.bin_path, "offline_teach/bin_teach"),
                ("Tray Teach", self.tray_path, "offline_teach/tray_teach"))):
            teach.addWidget(QtWidgets.QLabel(label), index, 0)
            edit.setMinimumWidth(0)
            edit.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            edit.textChanged.connect(edit.setToolTip)
            edit.setToolTip(edit.text())
            teach.addWidget(edit, index, 1)
            button = QtWidgets.QPushButton("Browse…")
            button.clicked.connect(
                lambda _checked, e=edit, d=directory: self._browse(e, d))
            teach.addWidget(button, index, 2)
            self.teach_browse_buttons.append(button)
        self.configure = QtWidgets.QPushButton("Load Teach Configuration")
        self.configure.clicked.connect(self._configure)
        teach.addWidget(self.configure, 3, 0, 1, 3)
        teach.setColumnStretch(1, 1)
        header.addWidget(self.teach_panel, 4, QtCore.Qt.AlignTop)
        layout.addLayout(header)

        lifecycle = QtWidgets.QHBoxLayout()
        self.recover = QtWidgets.QPushButton("Recover / Clear Error")
        self.pause = QtWidgets.QToolButton()
        self.pause.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        self.managed_primary = "pause"
        self.pause.clicked.connect(lambda: self._managed_command(self.managed_primary))
        self.managed_actions = {}
        self.pause_menu = QtWidgets.QMenu(self.pause)
        for name, title in (("pause", "PAUSE"), ("continue", "CONTINUE"),
                            ("return_item", "RETURN ITEM")):
            action = QtWidgets.QAction(title, self.pause)
            action.triggered.connect(lambda _checked=False, n=name: self._managed_command(n))
            self.managed_actions[name] = action
            if name != "pause":
                self.pause_menu.addAction(action)
        self.stop = QtWidgets.QPushButton("STOP")
        self.stop.setStyleSheet(
            "background:#b51f24;color:white;font-weight:800;font-size:18px")
        self.stop.setToolTip("Stop immediately and preserve the gripper outputs")
        self.recover.clicked.connect(lambda: self._command("recover"))
        self.recover.setToolTip(
            "Cancel the action, preserve grip while lifting and returning Home, "
            "then relax fingers and turn suction/exhaust OFF")
        self.stop.clicked.connect(self._immediate_stop)
        for button in (self.recover, self.pause, self.stop):
            button.setMinimumHeight(58)
            lifecycle.addWidget(button, 1)
        layout.addLayout(lifecycle)

        operations = QtWidgets.QGridLayout()
        self.preview_toggle = QtWidgets.QPushButton("Preview: OFF")
        self.preview_toggle.setCheckable(True)
        self.preview_toggle.setToolTip(
            "ON: Home/Pick/Place publish TFs only. OFF: buttons command real robot motion.")
        self.preview_toggle.setStyleSheet(
            "QPushButton:checked{background:#236fa1;color:white;font-weight:bold}")
        self.home_button = QtWidgets.QPushButton("Home")
        self.pick_item = QtWidgets.QPushButton("Pick Item")
        self.place_item = QtWidgets.QPushButton("Place Item")
        self.place_item.setToolTip(
            "Requires Tray Detect position; observe tray, then place and retract above it")
        self.debug_images = QtWidgets.QCheckBox("Save Pick debug RGB/depth")
        self.preview_toggle.toggled.connect(self._toggle_preview)
        self.home_button.clicked.connect(lambda: self._action("home"))
        self.pick_item.clicked.connect(lambda: self._action("pick"))
        self.place_item.clicked.connect(lambda: self._action("place"))
        for button in (self.home_button, self.preview_toggle,
                       self.pick_item, self.place_item):
            button.setMinimumHeight(52)
        operations.addWidget(self.home_button, 0, 0)
        operations.addWidget(self.preview_toggle, 0, 1)
        operations.addWidget(self.pick_item, 1, 0)
        operations.addWidget(self.place_item, 1, 1)
        self.place_x = QtWidgets.QLineEdit()
        self.place_y = QtWidgets.QLineEdit()
        self.place_x.setPlaceholderText("Positive mm")
        self.place_y.setPlaceholderText("Positive mm")
        self.place_rotation = QtWidgets.QDoubleSpinBox()
        self.place_rotation.setRange(-180., 180.)
        self.place_rotation.setDecimals(1)
        self.place_rotation.setValue(0.)
        self.place_rotation.setSuffix("°")
        self.place_rotation.setToolTip("0° = recorded Tray Detect Pose; rotate about its tool Z")
        target_row = QtWidgets.QHBoxLayout()
        for label, field in (("X (mm)", self.place_x), ("Y (mm)", self.place_y),
                             ("Rotation", self.place_rotation)):
            target_row.addWidget(QtWidgets.QLabel(label))
            target_row.addWidget(field)
        if node.prefill and node.prefill["placement"] is not None:
            x, y, rotation = node.prefill["placement"]
            self.place_x.setText(str(x))
            self.place_y.setText(str(y))
            self.place_rotation.setValue(rotation)
        operations.addLayout(target_row, 2, 0, 1, 2)
        operations.addWidget(self.debug_images, 3, 1)
        auto_row = QtWidgets.QHBoxLayout()
        self.auto_run_button = QtWidgets.QPushButton("Auto Run")
        self.auto_run_button.setMinimumHeight(48)
        self.auto_run_button.clicked.connect(lambda: self._action("auto_run"))
        self.auto_quantity = QtWidgets.QSpinBox()
        self.auto_quantity.setRange(1, 10000)
        self.auto_quantity.setValue(1)
        self.auto_quantity.setAccessibleName("Auto Run quantity")
        self.auto_progress = QtWidgets.QLabel("0 completed")
        auto_row.addWidget(self.auto_run_button, 1)
        auto_row.addWidget(QtWidgets.QLabel("Quantity"))
        auto_row.addWidget(self.auto_quantity)
        auto_row.addWidget(self.auto_progress, 1)
        operations.addLayout(auto_row, 4, 0, 1, 2)
        layout.addLayout(operations)
        self.availability_details = QtWidgets.QLabel()
        self.availability_details.setTextFormat(QtCore.Qt.PlainText)
        self.availability_details.setWordWrap(True)
        self.availability_details.setStyleSheet("color:#785000")
        layout.addWidget(self.availability_details)

        speed = QtWidgets.QHBoxLayout()
        self.speed_label = QtWidgets.QLabel("Global SpeedFactor: unavailable")
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(100)
        self.speed_slider.setTracking(True)
        self.speed_slider.sliderMoved.connect(self._speed_preview)
        self.speed_slider.sliderReleased.connect(self._speed_released)
        self.speed_slider.valueChanged.connect(self._speed_value_changed)
        self.speed_debounce = QtCore.QTimer(self)
        self.speed_debounce.setSingleShot(True)
        self.speed_debounce.setInterval(350)
        self.speed_debounce.timeout.connect(self._speed)
        speed.addWidget(self.speed_label)
        speed.addWidget(self.speed_slider, 1)
        layout.addLayout(speed)

        log_header = QtWidgets.QHBoxLayout()
        self.log_toggle = QtWidgets.QPushButton("Show command log")
        self.log_toggle.setCheckable(True)
        self.log_toggle.toggled.connect(self._toggle_log)
        self.copy_log = QtWidgets.QPushButton("Copy Log")
        self.copy_log.clicked.connect(self._copy_log)
        self.copy_log.hide()
        log_header.addWidget(self.log_toggle)
        log_header.addStretch()
        log_header.addWidget(self.copy_log)
        layout.addLayout(log_header)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.log_view.setPlaceholderText(
            "Controller state, operation phases, and every Dobot service request/response "
            "will appear here.")
        self.log_view.document().setMaximumBlockCount(1000)
        self.log_view.setMinimumHeight(240)
        self.log_view.setStyleSheet(
            "QPlainTextEdit{background:#101820;color:#dce6ef;border:1px solid #465463;"
            "font-family:monospace;font-size:13px;padding:8px;selection-background-color:#315f86}")
        layout.addWidget(self.log_view, 1)
        self.log_view.hide()
        layout.setAlignment(QtCore.Qt.AlignTop)
        self.item_path.textEdited.connect(self._selection_changed)
        self.bin_path.textEdited.connect(self._selection_changed)
        self.tray_path.textEdited.connect(self._selection_changed)
        self.place_x.textEdited.connect(self._clear_preview)
        self.place_y.textEdited.connect(self._clear_preview)
        self.place_rotation.valueChanged.connect(self._clear_preview)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)
        self._refresh()

    @staticmethod
    def _status_panel(title):
        panel = QtWidgets.QFrame()
        panel.setMinimumHeight(170)
        panel.setStyleSheet("QFrame{background:#202a35;border-radius:6px}"
                            "QLabel{color:#edf3f8;border:0;font-size:14px}")
        column = QtWidgets.QVBoxLayout(panel)
        column.setContentsMargins(12, 10, 12, 10)
        column.setSpacing(5)
        heading = QtWidgets.QLabel(title)
        heading.setStyleSheet("font-size:17px;font-weight:700;color:white")
        column.addWidget(heading)
        return panel, column

    def _refresh_status(self, state):
        label, detail = presentation(state, self.preview_mode)
        self.status.setText(label)
        self.status_detail.setText(detail)
        color = ("#61dfa5" if label == "READY" else
                 "#ff9393" if label in ("ATTENTION REQUIRED", "EMERGENCY STOP PRESSED") else
                 "#ffd077" if label in ("PAUSED", "OFFLINE", "HOLDING ITEM") else "#edf3f8")
        size = 18 if len(label) > 14 else 24
        self.status.setWordWrap(True)
        self.status.setStyleSheet(f"font-size:{size}px;font-weight:700;color:{color}")
        if state is None:
            self.status.setToolTip("/robot_controller/status is missing or older than 1 second")
        else:
            details = [state.message,
                       f"Feedback: {'live' if state.feedback_fresh else 'unavailable'}",
                       f"Configuration: {state.configuration_id or 'none'}"]
            if self.feedback_message:
                details.append(self.feedback_message)
            if state.candidate_states:
                details.append("Candidates: " + ", ".join(
                    f"{index}: {value}" for index, value in enumerate(state.candidate_states, 1)))
            self.status.setToolTip("\n".join(details))
        fresh = state is not None and state.feedback_fresh
        for channel, led in self.gripper_leds.items():
            active = bool(state.digital_input_bits & (1 << (channel - 1))) if fresh else None
            value = "Unknown" if active is None else "Detected" if active else "Not detected"
            color = "#e5a52e" if active is None else "#25c97e" if active else "#657789"
            led.setStyleSheet(f"background:{color};border:2px solid #9aa9b7;border-radius:12px")
            led.setAccessibleDescription(value)
            self.gripper_values[channel].setText(value)
            led.setToolTip(f"DI{channel}: {value}")
        self.gripper_panel.setToolTip(
            "Green = Detected · Gray = Not detected · Amber = Unknown\n"
            "Raw DI1 suction detection and DI12 fully-open detection.\n"
            "Feedback updates at 5 Hz; brief pulses may fall between updates.\n"
            "LOW DI12 does not prove fingers closed.")

    def _toggle_log(self, visible):
        if visible:
            self._collapsed_height = self.height()
        target_height = self._collapsed_height + 240 if visible else self._collapsed_height
        self.log_view.setVisible(visible)
        self.copy_log.setVisible(visible)
        self.log_toggle.setText("Hide command log" if visible else "Show command log")
        self.centralWidget().layout().activate()
        self.layout().activate()
        self.resize(self.width(), target_height)

    def _browse(self, edit, directory):
        path, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select Teach YAML", str(self.node.root / directory),
            "Teach YAML (*.yaml)")
        if path:
            edit.setText(path)
            self._selection_changed()

    def _copy_log(self):
        QtWidgets.QApplication.clipboard().setText(self.log_view.toPlainText())

    def _append_operator_logs(self):
        values = self.node.take_operator_logs()
        if not values:
            return
        scroll = self.log_view.verticalScrollBar()
        follow_tail = scroll.value() >= scroll.maximum() - 2
        for value in values:
            self.log_view.appendPlainText(value)
        if follow_tail:
            scroll.setValue(scroll.maximum())

    def _selection_changed(self):
        self.saved_selection = None
        self._clear_preview()

    def _toggle_preview(self, checked):
        if checked and self._availability()["preview_toggle"]:
            self.preview_toggle.blockSignals(True)
            self.preview_toggle.setChecked(False)
            self.preview_toggle.blockSignals(False)
            QtWidgets.QMessageBox.warning(
                self, "Preview unavailable", "Finish or stop the active operation before Preview.")
            return
        self.preview_mode = checked
        self.preview_toggle.setText("Preview: ON — TF only" if checked else "Preview: OFF")
        self.speed_debounce.stop()
        self._clear_preview()
        self.feedback_message = (
            "Preview enabled: motion buttons publish TFs only" if checked else "")
        self._refresh()

    def _clear_preview(self, *_args):
        self.feedback_message = ""
        future = self.pending.pop("preview", None)
        if future is not None:
            future.cancel()
        if self.node.service_clients["preview"].service_is_ready():
            request = Preview.Request()
            request.operation = request.CLEAR
            request.item_teach_file = ""
            request.bin_teach_file = ""
            self.preview_clear_future = self.node.service_clients["preview"].call_async(request)
            self.preview_clear_deadline = time.monotonic() + 5.0

    def closeEvent(self, event):
        if self.preview_mode or "preview" in self.pending:
            self._clear_preview()
        super().closeEvent(event)

    def _call(self, name, request):
        if self.preview_mode and name not in ("preview", "stop"):
            return False
        if name in self.pending:
            return False
        client = self.node.service_clients[name]
        if not client.service_is_ready():
            QtWidgets.QMessageBox.warning(
                self, "Unavailable", f"{client.srv_name} is unavailable")
            return False
        self.pending[name] = client.call_async(request)
        self._refresh_controls()
        return True

    def _configure(self):
        if self._availability()["configure"]:
            return
        request = Configure.Request()
        request.item_teach_file = self.item_path.text().strip()
        request.bin_teach_file = self.bin_path.text().strip()
        request.tray_teach_file = self.tray_path.text().strip()
        self.saved_selection = (request.item_teach_file, request.bin_teach_file,
                                request.tray_teach_file)
        if not self._call("configure", request):
            self.saved_selection = None

    def _command(self, name):
        if name != "stop" and self._availability().get(name, "Unavailable command"):
            return False
        return self._call(name, Command.Request())

    _acquisition_paused = staticmethod(acquisition_paused)

    def _immediate_stop(self):
        if self.preview_mode:
            self._clear_preview()
        # Dispatch the independent Stop before optional action cancellation.
        self._command("stop")
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def _managed_command(self, name):
        if self._availability()[name]:
            return
        if self._command(name):
            if name == "return_item":
                self.return_requested_locally = True
            elif name == "pause":
                self.pause_requested_locally = True
            self._refresh_controls()

    def _preview(self, operation):
        request = Preview.Request()
        request.operation = operation
        request.item_teach_file = self.item_path.text().strip()
        request.bin_teach_file = self.bin_path.text().strip()
        request.tray_teach_file = self.tray_path.text().strip()
        if operation == request.PLACE:
            try:
                request.x_mm, request.y_mm, request.rotation_deg = validate_target(
                    float(self.place_x.text()), float(self.place_y.text()),
                    self.place_rotation.value())
            except ValueError as exc:
                QtWidgets.QMessageBox.warning(self, "Placement target", str(exc))
                return
        self._call("preview", request)

    def _sync_speed_slider(self, percent):
        self.speed_syncing = True
        try:
            self.speed_slider.setValue(percent)
        finally:
            self.speed_syncing = False

    def _speed_preview(self, percent):
        self.speed_label.setText(f"Global SpeedFactor: {int(percent)}% selected")

    def _speed_value_changed(self, percent):
        if self.speed_syncing:
            return
        self._speed_preview(percent)
        if not self.speed_slider.isSliderDown() and self.speed_slider.hasFocus():
            self.speed_debounce.start()

    def _speed_released(self):
        self.speed_debounce.stop()
        self._speed()

    def _speed(self):
        self.speed_debounce.stop()
        if self._availability()["speed"]:
            return
        if "speed" in self.pending:
            return
        percent = int(self.speed_slider.sliderPosition())
        status = self.node.status
        if (status is not None and status.global_speed_percent == percent
                and self.speed_pending_percent is None):
            self.speed_label.setText(f"Global SpeedFactor: {percent}%")
            return
        request = SetGlobalSpeed.Request()
        request.percent = percent
        if self._call("speed", request):
            self.speed_pending_percent = percent
            self.speed_label.setText(f"Global SpeedFactor: requesting {percent}%")

    def _feedback(self, message):
        feedback = message.feedback
        suffix = ""
        if hasattr(feedback, "candidate_total") and feedback.candidate_total:
            suffix = f" · candidate {feedback.candidate_index}/{feedback.candidate_total}"
        self.feedback_message = f"{feedback.phase} · {feedback.message}{suffix}"

    def _action(self, name):
        if self._availability()[name]:
            return
        status = self.node.status
        if not self.preview_mode and name == "place" and self._acquisition_paused(status):
            self._call("continue", Command.Request())
            return
        if self.preview_mode:
            self._preview({"home": Preview.Request.HOME, "pick": Preview.Request.PICK,
                           "place": Preview.Request.PLACE}[name])
            return
        client = self.node.action_clients[name]
        if not client.server_is_ready():
            QtWidgets.QMessageBox.warning(self, "Unavailable", "Action server unavailable")
            return
        kind = {"home": GoHome, "pick": PickItem, "place": PlaceItem, "auto_run": AutoRun}
        goal = kind[name].Goal()
        goal.configuration_id = status.configuration_id
        if name in ("place", "auto_run"):
            try:
                values = validate_target(float(self.place_x.text()), float(self.place_y.text()),
                                         self.place_rotation.value())
                goal.x_mm, goal.y_mm, goal.rotation_deg = values
                previous = load_state(self.node.root / "logs/robot_controller/last_session.json")
                if previous is not None:
                    save_state(self.node.root / "logs/robot_controller/last_session.json",
                               previous["item"], previous["bin"], previous["tray"],
                               placement=values)
            except (ValueError, OSError) as exc:
                QtWidgets.QMessageBox.warning(self, "Placement target", str(exc))
                return
        if name in ("pick", "auto_run"):
            goal.save_debug_images = self.debug_images.isChecked()
        if name == "auto_run":
            goal.quantity = self.auto_quantity.value()
            self.auto_progress.setText(f"0/{goal.quantity} completed")
        self.pending_goal = client.send_goal_async(goal, feedback_callback=self._feedback)
        self._refresh_controls()

    def _collect(self):
        if self.preview_clear_future is not None and (
                self.preview_clear_future.done()
                or time.monotonic() > self.preview_clear_deadline):
            try:
                if not self.preview_clear_future.done():
                    self.preview_clear_future.cancel()
                    raise RuntimeError("Preview clear response timed out")
                cleared = self.preview_clear_future.result()
                if cleared is None or not cleared.success:
                    raise RuntimeError("Preview could not confirm clearing its TFs")
            except Exception as exc:
                QtWidgets.QMessageBox.warning(self, "Preview clear failed", str(exc))
            self.preview_clear_future = None
        for name, future in list(self.pending.items()):
            if not future.done():
                continue
            del self.pending[name]
            accepted = False
            try:
                result = future.result()
                accepted = result is not None and result.success
                if result is None or not result.success:
                    message = "No response" if result is None else result.message
                    if name == "speed":
                        self.speed_pending_percent = None
                        if result is not None and result.confirmed_percent >= 1:
                            self._sync_speed_slider(result.confirmed_percent)
                    QtWidgets.QMessageBox.warning(self, f"{name} rejected", message)
                elif name == "configure" and self.saved_selection is not None:
                    save_state(
                        self.node.root / "logs/robot_controller/last_session.json",
                        *self.saved_selection)
                    self._clear_preview()
                elif name == "speed":
                    self.speed_pending_percent = result.confirmed_percent
                    self._sync_speed_slider(result.confirmed_percent)
                    self.speed_label.setText(
                        f"Global SpeedFactor: {result.confirmed_percent}% confirmed")
                elif name == "preview":
                    self.feedback_message = result.message
                    self.log_view.appendPlainText("PREVIEW: " + result.message)
                elif name == "recover" and result.state == "READY":
                    QtWidgets.QMessageBox.information(
                        self, "Recovered — gripper relaxed",
                        "Recovery completed at Home. Fingers are relaxed, suction and "
                        "exhaust are OFF, and DI1 is LOW. The interrupted action was cancelled.")
            except Exception as exc:
                if name == "speed":
                    self.speed_pending_percent = None
                QtWidgets.QMessageBox.warning(self, f"{name} failed", str(exc))
            finally:
                if name == "pause" and not accepted:
                    self.pause_requested_locally = False
                if name == "return_item" and not accepted:
                    self.return_requested_locally = False
        if self.pending_goal is not None and self.pending_goal.done():
            try:
                handle = self.pending_goal.result()
                if handle is None or not handle.accepted:
                    QtWidgets.QMessageBox.warning(
                        self, "Action rejected",
                        "State/configuration does not permit that hardware action")
                else:
                    self.goal_handle = handle
                    self.result_future = handle.get_result_async()
            except Exception as exc:
                QtWidgets.QMessageBox.warning(self, "Action failed", str(exc))
            self.pending_goal = None
        if self.result_future is not None and self.result_future.done():
            try:
                wrapped = self.result_future.result()
                result = wrapped.result
                self.feedback_message = f"Result {result.outcome}: {result.message}"
                if hasattr(result, "completed_quantity"):
                    self.auto_progress.setText(
                        f"{result.completed_quantity}/{result.requested_quantity} completed")
                if (result.outcome not in (result.SUCCESS, getattr(result, "NO_PICK", -1))
                        or hasattr(result, "completed_quantity")
                        and result.outcome != result.SUCCESS):
                    QtWidgets.QMessageBox.warning(self, "Action ended", result.message)
            except Exception as exc:
                QtWidgets.QMessageBox.warning(self, "Action result failed", str(exc))
            self.result_future = None
            self.goal_handle = None

    def _refresh(self):
        self._append_operator_logs()
        self._collect()
        self._refresh_controls()

    def _availability(self):
        try:
            validate_target(float(self.place_x.text()), float(self.place_y.text()),
                            self.place_rotation.value())
            target_error = ""
        except ValueError:
            target_error = "Enter positive X/Y in mm and a valid rotation"
        reasons = button_policy(
            self.node.status, preview=self.preview_mode,
            pending=bool(self.pending or self.preview_clear_future is not None),
            action_pending=self.pending_goal is not None or self.result_future is not None,
            managed_pending=self.pause_requested_locally or self.return_requested_locally,
            target_error=target_error, item_selected=bool(self.item_path.text().strip()),
            bin_selected=bool(self.bin_path.text().strip()),
            tray_selected=bool(self.tray_path.text().strip()))
        if not self.auto_quantity.hasAcceptableInput():
            reasons["auto_run"] = "Enter a whole quantity from 1 to 10000"
        for name in reasons:
            if reasons[name] or name == "preview_toggle":
                continue
            if name in ("home", "pick", "place", "auto_run"):
                if self.preview_mode:
                    client = self.node.service_clients["preview"]
                    ready = client.service_is_ready()
                elif name == "place" and self._acquisition_paused(self.node.status):
                    ready = self.node.service_clients["continue"].service_is_ready()
                else:
                    client = self.node.action_clients.get(name)
                    ready = client is not None and client.server_is_ready()
            else:
                ready = self.node.service_clients[name].service_is_ready()
            if not ready:
                reasons[name] = "Service unavailable"
        return reasons

    def _refresh_controls(self):
        state = self.node.status
        self._refresh_status(state)
        current = state.state if state else "OFFLINE"
        if current not in ("READY", "HOLDING", "HOMING", "PICKING", "TRAY_POSITIONING", "PLACING"):
            self.pause_requested_locally = False
        if current != "PAUSED":
            self.return_requested_locally = False
        reasons = self._availability()
        paused = current == "PAUSED"
        retry = self._acquisition_paused(state) and not self.preview_mode
        self.configure.setText(
            "Reload Teach Configuration" if state and state.configured
            else "Load Teach Configuration")
        returning = self.return_requested_locally or "return_item" in self.pending
        parking = self.pause_requested_locally or "pause" in self.pending
        if (state is not None and state.feedback_fresh and not self.preview_mode
                and current not in ("FAULT", "HELD_UNKNOWN", "RECOVERY_REQUIRED")
                and (self.pending or self.pending_goal is not None or returning or parking)):
            self.status.setText("BUSY")
            self.status.setStyleSheet("font-size:24px;font-weight:700;color:#edf3f8")
            self.status_detail.setText(
                "Returning item…" if returning else "Pause requested — waiting for confirmation…"
                if parking else "Loading teach files and preparing robot…"
                if "configure" in self.pending else
                "Request sent — waiting for controller confirmation…")
        managed_name = "pause"
        if paused:
            managed_name = ("return_item" if retry or state.can_return_item
                            and not reasons["return_item"] else "continue")
        for name, action in self.managed_actions.items():
            action.setEnabled(not reasons[name])
        self.managed_primary = managed_name
        self.pause.setText(self.managed_actions[managed_name].text())
        menu = self.pause_menu if paused and not retry and state.can_return_item else None
        self.pause.setMenu(menu)
        self.pause.setPopupMode(QtWidgets.QToolButton.MenuButtonPopup if menu is not None
                                else QtWidgets.QToolButton.DelayedPopup)
        if returning or current == "RETURNING_ITEM":
            self.pause.setText("Returning item…")
        elif parking or current == "PAUSING":
            self.pause.setText("Pausing…")
        self.pause.setStyleSheet(
            "QToolButton{background:#d18b00;color:white;font-weight:800;font-size:18px}"
            "QToolButton:disabled{background:#e2e2e2;color:#999}")
        self.place_item.setText("Place Item (Retry)" if retry else "Place Item")
        buttons = {
            "configure": self.configure, "recover": self.recover, managed_name: self.pause,
            "home": self.home_button, "pick": self.pick_item, "place": self.place_item,
            "preview_toggle": self.preview_toggle, "speed": self.speed_slider,
            "auto_run": self.auto_run_button,
        }
        descriptions = {
            "auto_run": "Pick and place the requested quantity, then return Home",
            "configure": "Load teach files, enable and initialize the robot to READY",
            "continue": "Resume the retained operation",
            "recover": "Cancel the action, preserve grip to Home, then reset gripper outputs",
            "pause": "Stop and confirm the operation's paused position",
            "return_item": "Return the held item to its saved bin position, then Home",
            "home": "Move Home while preserving any held item",
            "pick": "Pick an item and carry it to Tray Detect",
            "place": "Retry tray acquisition with up to 3 fresh requests" if retry else
                     "Observe the tray, place and retract above it",
            "preview_toggle": "ON: motion buttons show TFs only. OFF: real robot motion.",
            "speed": "Change the global motion speed factor",
        }
        for name, action in self.managed_actions.items():
            action.setToolTip(reasons[name] or descriptions[name])
        for name, button in buttons.items():
            button.setEnabled(not reasons[name])
            tip = reasons[name] or descriptions[name]
            if self.preview_mode and name in ("home", "pick", "place") and not reasons[name]:
                tip = "Show planned TF targets; no robot motion or gripper output"
            button.setToolTip(tip)
        self.stop.setEnabled(self.node.service_clients["stop"].service_is_ready())
        editable = (state is not None and not state.operation_active and not self.pending
                    and self.pending_goal is None and self.result_future is None)
        for field in (self.place_x, self.place_y, self.place_rotation, self.auto_quantity):
            field.setEnabled(editable)
        if state is not None and getattr(state, "auto_run_active", False):
            self.auto_progress.setText(
                f"{state.auto_run_completed}/{state.auto_run_requested} completed")
        for field in (self.item_path, self.bin_path, self.tray_path, *self.teach_browse_buttons):
            field.setEnabled(editable and current in ("UNCONFIGURED", "INACTIVE", "READY"))
        self.debug_images.setEnabled(not self.preview_mode and editable)
        visible_reasons = []
        if not self.pending and current in ("READY", "HOLDING", "PAUSED", "INACTIVE",
                                            "FAULT", "RECOVERY_REQUIRED", "HELD_UNKNOWN"):
            for name in (("place", "return_item") if retry else
                         ("continue", "return_item") if paused else
                         ("recover",) if current in
                         ("FAULT", "RECOVERY_REQUIRED", "HELD_UNKNOWN") else
                         ("configure",) if current == "INACTIVE" and not self.preview_mode else
                         ("pick", "place")):
                if reasons[name] and reasons[name] != "Unavailable in the current state":
                    label = (self.managed_actions[name].text() if name in self.managed_actions
                             else buttons[name].text())
                    visible_reasons.append(f"{label}: {reasons[name]}")
        self.availability_details.setText("\n".join(visible_reasons))
        self.availability_details.setVisible(bool(visible_reasons))
        if state:
            if (self.speed_pending_percent is not None
                    and "speed" not in self.pending
                    and state.global_speed_percent == self.speed_pending_percent):
                self.speed_pending_percent = None
            editing_speed = (
                self.speed_slider.isSliderDown() or self.speed_debounce.isActive()
                or "speed" in self.pending or self.speed_pending_percent is not None)
            if state.global_speed_percent >= 1 and not editing_speed:
                self._sync_speed_slider(state.global_speed_percent)
            if self.speed_slider.isSliderDown() or self.speed_debounce.isActive():
                speed_text = f"{self.speed_slider.sliderPosition()}% selected"
            elif self.speed_pending_percent is not None:
                suffix = "requesting" if "speed" in self.pending else "confirmed"
                speed_text = f"{self.speed_pending_percent}% {suffix}"
            else:
                speed_text = (f"{state.global_speed_percent}%"
                              if state.global_speed_percent >= 1 else "unknown")
            self.speed_label.setText(f"Global SpeedFactor: {speed_text}")
        else:
            self.speed_label.setText("Global SpeedFactor: unavailable")


def main(args=None):
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous = {
        number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}
    for number in previous:
        signal.signal(number, lambda _number, _frame: app.quit())
    node = executor = thread = window = None
    code = 1
    try:
        node = GuiNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        window = ControllerWindow(node)
        window.show()
        code = app.exec_()
    finally:
        if window is not None:
            window.timer.stop()
            window.close()
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if thread is not None:
            thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for number, handler in previous.items():
            signal.signal(number, handler)
    raise SystemExit(code)
