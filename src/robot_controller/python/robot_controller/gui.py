"""Qt client for Robot Controller v2; contains no Dobot command clients."""

import os
import sys
import threading

from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from item_perception_yolo.platform_teach_core import workspace_root
from robot_controller_interfaces.action import GoHome, PickItem
from robot_controller_interfaces.msg import ControllerStatus
from robot_controller_interfaces.srv import Command, Configure, Preview, SetGlobalSpeed

from .ui_state import load_state, save_state


class GuiNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("robot_controller_gui")
        self.root = workspace_root()
        self.prefill = load_state(self.root / "logs/robot_controller/last_session.json")
        self.status = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            ControllerStatus, "/robot_controller/status", self._status, qos)
        self.service_clients = {
            "configure": self.create_client(Configure, "/robot_controller/configure"),
            "startup": self.create_client(Command, "/robot_controller/startup"),
            "recover": self.create_client(Command, "/robot_controller/recover"),
            "stop": self.create_client(Command, "/robot_controller/stop"),
            "speed": self.create_client(
                SetGlobalSpeed, "/robot_controller/set_global_speed"),
            "preview": self.create_client(Preview, "/robot_controller/preview"),
        }
        self.action_clients = {
            "home": ActionClient(self, GoHome, "/robot_controller/go_home"),
            "pick": ActionClient(self, PickItem, "/robot_controller/pick_item"),
        }

    def _status(self, message):
        self.status = message


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
        self.setWindowTitle("Robot Controller v2")
        self.resize(1050, 570)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        title = QtWidgets.QLabel("Deterministic Home and Pick")
        title.setStyleSheet("font-size:22px;font-weight:700;padding:8px")
        layout.addWidget(title)
        self.item_path = QtWidgets.QLineEdit()
        self.bin_path = QtWidgets.QLineEdit()
        if node.prefill:
            self.item_path.setText(str(
                node.root / "offline_teach/item_teach" / node.prefill["item"]))
            if node.prefill["bin"]:
                self.bin_path.setText(str(
                    node.root / "offline_teach/bin_teach" / node.prefill["bin"]))
        for label, edit, directory in (
                ("Item Teach", self.item_path, "offline_teach/item_teach"),
                ("Bin Teach", self.bin_path, "offline_teach/bin_teach")):
            row = QtWidgets.QHBoxLayout()
            row.addWidget(QtWidgets.QLabel(label))
            row.addWidget(edit, 1)
            button = QtWidgets.QPushButton("Browse…")
            button.clicked.connect(
                lambda _checked, e=edit, d=directory: self._browse(e, d))
            row.addWidget(button)
            layout.addLayout(row)
        self.configure = QtWidgets.QPushButton("Load Teach Configuration")
        self.configure.clicked.connect(self._configure)
        layout.addWidget(self.configure)

        lifecycle = QtWidgets.QHBoxLayout()
        self.startup = QtWidgets.QPushButton("Startup")
        self.recover = QtWidgets.QPushButton("Recover / Clear Error")
        self.stop = QtWidgets.QPushButton("STOP")
        self.stop.setStyleSheet(
            "background:#b51f24;color:white;font-weight:800;font-size:18px")
        self.startup.clicked.connect(lambda: self._command("startup"))
        self.recover.clicked.connect(lambda: self._command("recover"))
        self.stop.clicked.connect(self._stop)
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
        self.speed_slider.setTracking(False)
        self.speed_slider.sliderReleased.connect(self._speed)
        speed.addWidget(self.speed_label)
        speed.addWidget(self.speed_slider, 1)
        layout.addLayout(speed)

        self.status = QtWidgets.QLabel("Waiting for /robot_controller/status")
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(100)
        self.status.setStyleSheet(
            "background:#202a35;color:white;padding:14px;font-size:16px")
        layout.addWidget(self.status)
        note = QtWidgets.QLabel(
            "Launching never enables or moves the robot. Startup is explicit. "
            "Stop and native cancellation preserve suction/finger outputs, discard motion, "
            "and require Recover. Preview is TF-only and cannot command Dobot.")
        note.setWordWrap(True)
        layout.addWidget(note)
        layout.addStretch()
        self.item_path.textEdited.connect(self._selection_changed)
        self.bin_path.textEdited.connect(self._selection_changed)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)

    def _browse(self, edit, directory):
        path, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select Teach YAML", str(self.node.root / directory),
            "Teach YAML (*.yaml)")
        if path:
            edit.setText(path)
            self._selection_changed()

    def _selection_changed(self):
        self.saved_selection = None
        if self.node.service_clients["preview"].service_is_ready():
            request = Preview.Request()
            request.operation = request.CLEAR
            request.item_teach_file = ""
            request.bin_teach_file = ""
            self.node.service_clients["preview"].call_async(request)

    def _call(self, name, request):
        if name in self.pending:
            return
        client = self.node.service_clients[name]
        if not client.service_is_ready():
            QtWidgets.QMessageBox.warning(
                self, "Unavailable", f"{client.srv_name} is unavailable")
            return
        self.pending[name] = client.call_async(request)

    def _configure(self):
        request = Configure.Request()
        request.item_teach_file = self.item_path.text().strip()
        request.bin_teach_file = self.bin_path.text().strip()
        self.saved_selection = (request.item_teach_file, request.bin_teach_file)
        self._call("configure", request)

    def _command(self, name):
        self._call(name, Command.Request())

    def _stop(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self._command("stop")

    def _preview(self, operation):
        request = Preview.Request()
        request.operation = operation
        request.item_teach_file = self.item_path.text().strip()
        request.bin_teach_file = self.bin_path.text().strip()
        self._call("preview", request)

    def _speed(self):
        request = SetGlobalSpeed.Request()
        request.percent = self.speed_slider.value()
        self._call("speed", request)

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
            try:
                result = future.result()
                if result is None or not result.success:
                    message = "No response" if result is None else result.message
                    QtWidgets.QMessageBox.warning(self, f"{name} rejected", message)
                elif name == "configure" and self.saved_selection is not None:
                    save_state(
                        self.node.root / "logs/robot_controller/last_session.json",
                        *self.saved_selection)
                elif name == "speed":
                    self.speed_slider.setValue(result.confirmed_percent)
            except Exception as exc:
                QtWidgets.QMessageBox.warning(self, f"{name} failed", str(exc))
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
        self._collect()
        state = self.node.status
        reachable = state is not None
        active = bool(state and state.operation_active)
        current = state.state if state else "UNREACHABLE"
        self.configure.setEnabled(reachable and current in ("UNCONFIGURED", "INACTIVE")
                                  and not active)
        self.startup.setEnabled(reachable and current == "INACTIVE" and not active)
        self.recover.setEnabled(reachable and current in ("FAULT", "RECOVERY_REQUIRED")
                                and not active)
        self.stop.setEnabled(self.node.service_clients["stop"].service_is_ready())
        self.hardware_home.setEnabled(reachable and current in ("READY", "HOLDING")
                                      and not active)
        self.hardware_pick.setEnabled(reachable and current == "READY" and not active)
        self.preview_home.setEnabled(
            self.node.service_clients["preview"].service_is_ready())
        self.preview_pick.setEnabled(
            self.node.service_clients["preview"].service_is_ready())
        speed_enabled = reachable and current in ("READY", "HOLDING") and not active
        self.speed_slider.setEnabled(speed_enabled)
        if state:
            if state.global_speed_percent >= 1 and not self.speed_slider.isSliderDown():
                self.speed_slider.setValue(state.global_speed_percent)
            speed_text = (f"{state.global_speed_percent}%"
                          if state.global_speed_percent >= 1 else "unknown")
            self.speed_label.setText(f"Global SpeedFactor: {speed_text}")
            progress = f"\n{self.feedback_message}" if self.feedback_message else ""
            self.status.setText(
                f"{state.state} · {state.message}\n"
                f"Configuration: {state.configuration_id[:16] or 'none'} · "
                f"Startup: {'complete' if state.startup_complete else 'required'} · "
                f"Feedback: {'fresh' if state.feedback_fresh else 'unavailable'}{progress}")


def main(args=None):
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required")
    rclpy.init(args=args)
    node = GuiNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = ControllerWindow(node)
    window.show()
    try:
        code = app.exec_()
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)
