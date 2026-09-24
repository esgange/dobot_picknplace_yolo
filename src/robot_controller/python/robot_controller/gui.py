"""Qt client for Robot Controller v2; contains no Dobot command clients."""

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
from robot_controller_interfaces.action import GoHome, PickItem
from robot_controller_interfaces.msg import ControllerStatus
from robot_controller_interfaces.srv import Command, Configure, Preview, SetGlobalSpeed

from .ui_state import load_state, save_state
from .feedback import FEEDBACK_MAX_AGE_SEC


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
            "startup": self.create_client(Command, "/robot_controller/startup"),
            "recover": self.create_client(Command, "/robot_controller/recover"),
            "pause": self.create_client(Command, "/robot_controller/pause"),
            "continue": self.create_client(Command, "/robot_controller/continue"),
            "stop": self.create_client(Command, "/robot_controller/stop"),
            "return_item": self.create_client(Command, "/robot_controller/return_item"),
            "speed": self.create_client(
                SetGlobalSpeed, "/robot_controller/set_global_speed"),
            "preview": self.create_client(Preview, "/robot_controller/preview"),
        }
        self.action_clients = {
            "home": ActionClient(self, GoHome, "/robot_controller/go_home"),
            "pick": ActionClient(self, PickItem, "/robot_controller/pick_item"),
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
        self.saved_selection = None
        self.pause_requested_locally = False
        self.return_requested_locally = False
        self.speed_pending_percent = None
        self.speed_syncing = False
        self.setWindowTitle("Robot Controller v2")
        self.resize(1050, 760)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        header = QtWidgets.QHBoxLayout()
        status_column = QtWidgets.QVBoxLayout()
        self.robot_panel, self.status = self._status_panel("Robot status")
        self.gripper_panel, self.gripper_status = self._status_panel("Gripper status / Live I/O")
        status_column.addWidget(self.robot_panel)
        status_column.addWidget(self.gripper_panel)
        header.addLayout(status_column, 3)

        self.teach_panel = QtWidgets.QGroupBox("Teach files")
        self.teach_panel.setMinimumWidth(300)
        self.teach_panel.setMaximumWidth(420)
        self.teach_panel.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                                       QtWidgets.QSizePolicy.Maximum)
        teach = QtWidgets.QGridLayout(self.teach_panel)
        self.item_path = QtWidgets.QLineEdit()
        self.bin_path = QtWidgets.QLineEdit()
        if node.prefill:
            self.item_path.setText(str(
                node.root / "offline_teach/item_teach" / node.prefill["item"]))
            if node.prefill["bin"]:
                self.bin_path.setText(str(
                    node.root / "offline_teach/bin_teach" / node.prefill["bin"]))
        for index, (label, edit, directory) in enumerate((
                ("Item Teach", self.item_path, "offline_teach/item_teach"),
                ("Bin Teach", self.bin_path, "offline_teach/bin_teach"))):
            teach.addWidget(QtWidgets.QLabel(label), index * 2, 0, 1, 2)
            edit.setMinimumWidth(0)
            edit.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            edit.textChanged.connect(edit.setToolTip)
            edit.setToolTip(edit.text())
            teach.addWidget(edit, index * 2 + 1, 0)
            button = QtWidgets.QPushButton("Browse…")
            button.clicked.connect(
                lambda _checked, e=edit, d=directory: self._browse(e, d))
            teach.addWidget(button, index * 2 + 1, 1)
        self.configure = QtWidgets.QPushButton("Load Teach Configuration")
        self.configure.clicked.connect(self._configure)
        teach.addWidget(self.configure, 4, 0, 1, 2)
        teach.setColumnStretch(0, 1)
        header.addWidget(self.teach_panel, 2, QtCore.Qt.AlignTop)
        layout.addLayout(header)

        lifecycle = QtWidgets.QHBoxLayout()
        self.startup = QtWidgets.QPushButton("START")
        self.recover = QtWidgets.QPushButton("Recover / Clear Error")
        self.stop = QtWidgets.QPushButton("PAUSE")
        self.startup.clicked.connect(self._start_or_continue)
        self.recover.clicked.connect(lambda: self._command("recover"))
        self.stop.clicked.connect(self._pause_or_stop)
        for button in (self.startup, self.recover, self.stop):
            button.setMinimumHeight(58)
            lifecycle.addWidget(button)
        layout.addLayout(lifecycle)

        operations = QtWidgets.QGridLayout()
        self.preview_home = QtWidgets.QPushButton("Preview Home TF")
        self.preview_pick = QtWidgets.QPushButton("Preview Pick TFs")
        self.hardware_home = QtWidgets.QPushButton("Hardware Home")
        self.hardware_pick = QtWidgets.QPushButton("Hardware Pick Item")
        self.debug_images = QtWidgets.QCheckBox("Save Pick debug RGB/depth")
        self.preview_home.clicked.connect(lambda: self._preview(Preview.Request.HOME))
        self.preview_pick.clicked.connect(lambda: self._preview(Preview.Request.PICK))
        self.hardware_home.clicked.connect(lambda: self._action("home"))
        self.hardware_pick.clicked.connect(lambda: self._action("pick"))
        for button in (self.preview_home, self.preview_pick,
                       self.hardware_home, self.hardware_pick):
            button.setMinimumHeight(52)
        operations.addWidget(self.preview_home, 0, 0)
        operations.addWidget(self.preview_pick, 0, 1)
        operations.addWidget(self.hardware_home, 1, 0)
        operations.addWidget(self.hardware_pick, 1, 1)
        operations.addWidget(self.debug_images, 2, 1)
        layout.addLayout(operations)

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
        log_title = QtWidgets.QLabel("Controller command log")
        log_title.setStyleSheet("font-size:15px;font-weight:700")
        self.copy_log = QtWidgets.QPushButton("Copy Log")
        self.copy_log.clicked.connect(self._copy_log)
        log_header.addWidget(log_title)
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
        self.log_view.setMinimumHeight(180)
        self.log_view.setStyleSheet(
            "QPlainTextEdit{background:#101820;color:#dce6ef;border:1px solid #465463;"
            "font-family:monospace;font-size:13px;padding:8px;selection-background-color:#315f86}")
        layout.addWidget(self.log_view, 1)
        self.item_path.textEdited.connect(self._selection_changed)
        self.bin_path.textEdited.connect(self._selection_changed)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)
        self._refresh_status(None)

    @staticmethod
    def _status_panel(title):
        panel = QtWidgets.QFrame()
        panel.setStyleSheet("QFrame{background:#202a35;border-radius:6px}"
                            "QLabel{color:#edf3f8;border:0;font-size:14px}")
        column = QtWidgets.QVBoxLayout(panel)
        column.setContentsMargins(12, 10, 12, 10)
        column.setSpacing(5)
        heading = QtWidgets.QLabel(title)
        heading.setStyleSheet("font-size:17px;font-weight:700;color:white")
        text = QtWidgets.QLabel()
        text.setWordWrap(True)
        text.setTextFormat(QtCore.Qt.PlainText)
        text.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        text.setMinimumWidth(0)
        column.addWidget(heading)
        column.addWidget(text)
        return panel, text

    def _refresh_status(self, state):
        if state is None:
            self.status.setText("UNAVAILABLE · Waiting for live controller status")
            self.status.setToolTip("/robot_controller/status is missing or older than 1 second")
            self.gripper_status.setText("I/O unavailable · No live controller status\n"
                                        "Suction, fingers and sensors: UNKNOWN")
            self.gripper_status.setToolTip("")
            return
        robot = f"{state.state} · {state.message}"
        if state.feedback_fresh:
            robot += (f"\n{'ENABLED' if state.robot_enabled else 'DISABLED'} · "
                      f"{'Moving' if state.robot_running else 'Stationary'} · "
                      f"Queue {'active' if state.robot_queue_active else 'idle'} · Feedback LIVE")
            flags = (("Robot error", state.robot_error), ("Collision", state.robot_collision))
            faults = [name for name, active in flags if active]
            if faults:
                robot += "\n" + " · ".join(faults)
        else:
            robot += "\nRobot feedback unavailable · Enable, motion and I/O UNKNOWN"
        robot += f"\nStartup: {'complete' if state.startup_complete else 'required'}"
        if state.candidate_total:
            robot += f" · Candidate {state.candidate_index}/{state.candidate_total}"
        if state.phase:
            robot += f" · {state.phase}"
        if state.waypoint:
            robot += f" · {state.waypoint}"
        self.status.setText(robot)
        details = [f"Configuration: {state.configuration_id or 'none'}"]
        if self.feedback_message:
            details.append(self.feedback_message)
        if state.candidate_states:
            details.append("Candidates: " + ", ".join(
                f"{index}: {value}" for index, value in enumerate(state.candidate_states, 1)))
        self.status.setToolTip("\n".join(details))
        self.gripper_status.setToolTip(
            "Observed I/O from the controller's validated feedback, sampled at 5 Hz.\n"
            "DI1 is raw; held-item decisions use the controller's 50 ms loss debounce.\n"
            "Short pulses may occur between updates. LOW DI12 does not prove fingers closed.")
        held = "YES" if state.holding_item else "NO"
        if not state.feedback_fresh:
            self.gripper_status.setText("I/O unavailable · Robot feedback missing or stale\n"
                                        f"Outputs and sensors: UNKNOWN · Held context: {held}")
            return

        def output(channel):
            return bool(state.digital_outputs & (1 << (channel - 1)))

        def on_off(channel):
            return "ON" if output(channel) else "OFF"

        def high_low(channel):
            return "HIGH" if state.digital_input_bits & (1 << (channel - 1)) else "LOW"

        vacuum = ("CONFLICT" if output(13) and output(1) else
                  "SUCK" if output(13) else "EXHAUST" if output(1) else "NEUTRAL")
        fingers = ("CONFLICT" if output(2) and output(14) else
                   "CLOSE" if output(2) else "OPEN" if output(14) else "NEUTRAL")
        self.gripper_status.setText(
            f"Vacuum: {vacuum} · Finger outputs: {fingers} · Held context: {held}\n"
            f"DO13 suction {on_off(13)} · DO1 exhaust {on_off(1)}\n"
            f"DO2 close {on_off(2)} · DO14 open {on_off(14)}\n"
            f"DI1 suction {high_low(1)} · DI12 fully open {high_low(12)}")

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

    def _clear_preview(self):
        if self.node.service_clients["preview"].service_is_ready():
            request = Preview.Request()
            request.operation = request.CLEAR
            request.item_teach_file = ""
            request.bin_teach_file = ""
            self.node.service_clients["preview"].call_async(request)

    def _call(self, name, request):
        if name in self.pending:
            return False
        client = self.node.service_clients[name]
        if not client.service_is_ready():
            QtWidgets.QMessageBox.warning(
                self, "Unavailable", f"{client.srv_name} is unavailable")
            return False
        self.pending[name] = client.call_async(request)
        return True

    def _configure(self):
        request = Configure.Request()
        request.item_teach_file = self.item_path.text().strip()
        request.bin_teach_file = self.bin_path.text().strip()
        self.saved_selection = (request.item_teach_file, request.bin_teach_file)
        if not self._call("configure", request):
            self.saved_selection = None

    def _command(self, name):
        return self._call(name, Command.Request())

    def _start_or_continue(self):
        state = self.node.status
        operation = "continue" if state is not None and state.state == "PAUSED" else "startup"
        self._command(operation)

    def _immediate_stop(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self._command("stop")

    def _pause_or_stop(self):
        state = self.node.status
        current = state.state if state is not None else "UNREACHABLE"
        pausable = current in ("READY", "HOLDING", "HOMING", "PICKING")
        if (self.pause_requested_locally or self.return_requested_locally
                or "pause" in self.pending or "return_item" in self.pending
                or current in ("PAUSING", "RETURNING_ITEM")):
            self._immediate_stop()
        elif current == "PAUSED" and getattr(state, "can_return_item", False):
            self.return_requested_locally = True
            if not self._command("return_item"):
                self.return_requested_locally = False
        elif pausable:
            self.pause_requested_locally = True
            if not self._command("pause"):
                self.pause_requested_locally = False
        else:
            self._immediate_stop()

    def _preview(self, operation):
        request = Preview.Request()
        request.operation = operation
        request.item_teach_file = self.item_path.text().strip()
        request.bin_teach_file = self.bin_path.text().strip()
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
        if self.pending_goal is not None or self.result_future is not None:
            return
        status = self.node.status
        if status is None or not status.configuration_id:
            QtWidgets.QMessageBox.warning(self, "Not configured", "Load teach files first")
            return
        client = self.node.action_clients[name]
        if not client.server_is_ready():
            QtWidgets.QMessageBox.warning(self, "Unavailable", "Action server unavailable")
            return
        goal = GoHome.Goal() if name == "home" else PickItem.Goal()
        goal.configuration_id = status.configuration_id
        if name == "pick":
            goal.save_debug_images = self.debug_images.isChecked()
        self.pending_goal = client.send_goal_async(goal, feedback_callback=self._feedback)

    def _collect(self):
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
                if result.outcome not in (result.SUCCESS, getattr(result, "NO_PICK", -1)):
                    QtWidgets.QMessageBox.warning(self, "Action ended", result.message)
            except Exception as exc:
                QtWidgets.QMessageBox.warning(self, "Action result failed", str(exc))
            self.result_future = None
            self.goal_handle = None

    def _refresh(self):
        self._append_operator_logs()
        self._collect()
        state = self.node.status
        self._refresh_status(state)
        reachable = state is not None
        active = bool(state and state.operation_active)
        current = state.state if state else "UNREACHABLE"
        if current not in ("READY", "HOLDING", "HOMING", "PICKING"):
            self.pause_requested_locally = False
        if current != "PAUSED":
            self.return_requested_locally = False
        configured = bool(state and state.configured)
        self.configure.setText(
            "Reload Teach Configuration" if configured else "Load Teach Configuration")
        self.configure.setEnabled(
            reachable and current in ("UNCONFIGURED", "INACTIVE", "READY")
            and not active)
        pause_pending = self.pause_requested_locally or "pause" in self.pending
        paused = current == "PAUSED" or pause_pending
        self.startup.setText("CONTINUE" if paused else "START")
        self.startup.setEnabled(
            reachable and ((current == "INACTIVE" and not active)
                           or (current == "PAUSED" and "continue" not in self.pending
                               and "stop" not in self.pending
                               and "return_item" not in self.pending
                               and not self.return_requested_locally)))
        self.recover.setEnabled(reachable and current in ("FAULT", "RECOVERY_REQUIRED")
                                and not active)
        pausable = current in ("READY", "HOLDING", "HOMING", "PICKING")
        if self.return_requested_locally or "return_item" in self.pending:
            self.stop.setText("STOP NOW")
        elif current == "PAUSED" and getattr(state, "can_return_item", False):
            self.stop.setText("RETURN ITEM & STOP")
        elif current in ("PAUSING", "RETURNING_ITEM"):
            self.stop.setText("STOP NOW")
        elif paused:
            self.stop.setText("STOP")
        elif pausable:
            self.stop.setText("PAUSE")
        else:
            self.stop.setText("STOP")
        if self.stop.text() == "PAUSE":
            self.stop.setStyleSheet(
                "background:#d18b00;color:white;font-weight:800;font-size:18px")
            self.stop.setEnabled(self.node.service_clients["pause"].service_is_ready())
        else:
            self.stop.setStyleSheet(
                "background:#b51f24;color:white;font-weight:800;font-size:18px")
            service = "return_item" if self.stop.text() == "RETURN ITEM & STOP" else "stop"
            self.stop.setEnabled(self.node.service_clients[service].service_is_ready()
                                 and (service != "return_item" or service not in self.pending))
        self.hardware_home.setEnabled(reachable and current in ("READY", "HOLDING")
                                      and not active)
        self.hardware_pick.setEnabled(reachable and current == "READY" and not active)
        self.preview_home.setEnabled(
            self.node.service_clients["preview"].service_is_ready())
        self.preview_pick.setEnabled(
            self.node.service_clients["preview"].service_is_ready())
        speed_enabled = reachable and current in ("READY", "HOLDING") and not active
        self.speed_slider.setEnabled(speed_enabled and "speed" not in self.pending)
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
