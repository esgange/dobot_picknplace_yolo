"""Explicit-action operator window; restored filenames remain unapplied prefill."""

import threading

from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import SetBool, Trigger

from .ui_state import save_state


class ControllerWindow(QtWidgets.QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.service_clients = {
            "home": node.create_client(Trigger, "/robot_controller/go_home"),
            "pick": node.create_client(Trigger, "/robot_controller/pick_item"),
            "stop": node.create_client(Trigger, "/robot_controller/stop"),
            "live": node.create_client(SetBool, "/robot_controller/set_live"),
            "debug_images": node.create_client(
                SetBool, "/robot_controller/set_debug_images"),
        }
        self.pending_calls = {}
        self.state_path = node.root / "logs/robot_controller/last_session.json"
        restored = node.ui_prefill  # Validated before real startup; never auto-applied.
        self.setWindowTitle("Robot Controller")
        self.resize(960, 550)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        self.mode = QtWidgets.QLabel()
        self.mode.setStyleSheet("font-size:20px;font-weight:600;padding:12px;color:#1573b7")
        layout.addWidget(self.mode)
        self.item_path = QtWidgets.QLineEdit()
        self.bin_path = QtWidgets.QLineEdit()
        self.browse_buttons = []
        for label, edit, directory in (("Item Teach", self.item_path, "offline_teach/item_teach"),
                                       ("Bin Teach", self.bin_path, "offline_teach/bin_teach")):
            row = QtWidgets.QHBoxLayout()
            row.addWidget(QtWidgets.QLabel(label))
            row.addWidget(edit, 1)
            browse = QtWidgets.QPushButton("Browse…")
            self.browse_buttons.append(browse)
            browse.clicked.connect(lambda _, e=edit, d=directory: self.browse(e, d))
            row.addWidget(browse)
            layout.addLayout(row)
        if node.profile_path is not None:
            self.item_path.setText(str(node.profile_path))
            if node.selection is not None:
                self.bin_path.setText(str(node.selection.bin.path))
        elif restored is not None:
            self.item_path.setText(str(node.root / "offline_teach/item_teach" / restored["item"]))
            if restored["bin"] is not None:
                self.bin_path.setText(str(node.root / "offline_teach/bin_teach" / restored["bin"]))
        self.apply = QtWidgets.QPushButton("Load Selected Teach Files")
        self.apply.clicked.connect(self.load_selected)
        layout.addWidget(self.apply)
        self.station = QtWidgets.QLabel("Bin required for Pick; Item Teach alone supplies Home.")
        self.station.setWordWrap(True)
        layout.addWidget(self.station)
        row = QtWidgets.QHBoxLayout()
        self.live = QtWidgets.QPushButton("Live: OFF")
        self.live.setCheckable(True)
        self.live.setMinimumHeight(60)
        self.live.setToolTip(
            "OFF publishes TF previews only. ON initializes and permits actual robot commands.")
        self.live.toggled.connect(self.set_live)
        row.addWidget(self.live)
        self.debug_images = QtWidgets.QPushButton("Debug Images: OFF")
        self.debug_images.setCheckable(True)
        self.debug_images.setMinimumHeight(60)
        self.debug_images.setToolTip(
            "Save the exact annotated RGB/depth pair for each requested pick batch.")
        self.debug_images.toggled.connect(self.set_debug_images)
        row.addWidget(self.debug_images)
        self.home = QtWidgets.QPushButton("Preview Home TF")
        self.pick = QtWidgets.QPushButton("Preview Pick TF")
        self.stop = QtWidgets.QPushButton("Clear / Cancel")
        for button, action in ((self.home, "home"), (self.pick, "pick")):
            button.setMinimumHeight(60)
            button.clicked.connect(lambda _, a=action: self.start(a))
            row.addWidget(button)
        self.stop.setMinimumHeight(60)
        self.stop.setStyleSheet("color:#b51f24;font-weight:600")
        self.stop.clicked.connect(self.stop_action)
        row.addWidget(self.stop)
        layout.addLayout(row)
        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        self.status.setStyleSheet("background:#202a35;color:white;padding:14px;font-size:16px")
        layout.addWidget(self.status)
        self.details = QtWidgets.QLabel(
            "Pick always starts at Home. Success returns Home with suction ON; "
            "an exhausted batch returns Home and reports failure.\n"
            "STOP cancels motion; when DI1 is ON it returns to the last pre-pick "
            "pose before releasing.\n"
            "Live canonical feedback is required, including for debug Home.\n"
            "Debug Images saves one requested annotated pair in debug/pick_img; "
            "it does not change poses or motion.\n"
            "Load does not move; preview TFs are not collision validation.")
        self.details.setWordWrap(True)
        layout.addWidget(self.details)
        layout.addStretch()
        self.item_path.textEdited.connect(self.selection_edited)
        self.bin_path.textEdited.connect(self.selection_edited)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.refresh()

    def selection_edited(self):
        if self.node.action_lock.locked():
            self.node.request_stop()
        self.node.clear_preview()
        self.node.selection = None
        self.node.profile_path = None

    def browse(self, edit, directory):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Select " + directory, str(self.node.root / directory), "Teach YAML (*.yaml)")
        if path:
            self.selection_edited()
            edit.setText(path)

    def load_selected(self):
        try:
            self.node.apply_teach(self.item_path.text().strip(), self.bin_path.text().strip())
            save_state(self.state_path, self.item_path.text(), self.bin_path.text().strip())
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Teach load rejected", str(exc))
        self.refresh()

    def start(self, action):
        self._call_service(action, Trigger.Request())

    def stop_action(self):
        self._call_service("stop", Trigger.Request())

    def set_live(self, enabled):
        self._set_live_visual(enabled)
        self._call_service("live", SetBool.Request(data=enabled))

    def set_debug_images(self, enabled):
        self._set_debug_images_visual(enabled)
        self._call_service("debug_images", SetBool.Request(data=enabled))

    def _set_live_visual(self, enabled):
        self.live.setText("Live: ON" if enabled else "Live: OFF")
        self.live.setStyleSheet(
            "background:#b51f24;color:white;font-weight:700;border:2px solid #7c1115;"
            if enabled else "")

    def _set_debug_images_visual(self, enabled):
        self.debug_images.setText("Debug Images: ON" if enabled else "Debug Images: OFF")
        self.debug_images.setStyleSheet(
            "background:#d77b00;color:white;font-weight:700;"
            if enabled else "")

    def _call_service(self, name, request):
        if name in self.pending_calls:
            return
        client = self.service_clients[name]
        if not client.service_is_ready():
            if name in ("live", "debug_images"):
                button = self.live if name == "live" else self.debug_images
                enabled = self.node.live if name == "live" else self.node.debug_images
                with QtCore.QSignalBlocker(button):
                    button.setChecked(enabled)
                (self._set_live_visual if name == "live"
                 else self._set_debug_images_visual)(enabled)
            QtWidgets.QMessageBox.warning(self, "Service unavailable",
                                          f"/robot_controller/{name} is unavailable")
            return
        self.pending_calls[name] = client.call_async(request)
        self.refresh()

    def _collect_service_results(self):
        for name, future in list(self.pending_calls.items()):
            if not future.done():
                continue
            del self.pending_calls[name]
            try:
                result = future.result()
                if result is None or not result.success:
                    message = "No response" if result is None else result.message
                    self.node.execution_message = f"{name} rejected: {message}"
            except Exception as exc:
                self.node.execution_message = f"{name} service failed: {exc}"

    def refresh(self):
        node = self.node
        self._collect_service_results()
        busy = ((node.action_thread is not None and node.action_thread.is_alive())
                or (node.stop_thread is not None and node.stop_thread.is_alive()))
        ready = node.execution_state in ("DEBUG", "READY", "HOLDING", "NO_PICK") and not busy
        actions_pending = any(name in self.pending_calls for name in ("home", "pick", "live"))
        self.apply.setEnabled(not busy)
        self.item_path.setReadOnly(busy)
        self.bin_path.setReadOnly(busy)
        for button in self.browse_buttons:
            button.setEnabled(not busy)
        self.home.setEnabled(ready and not actions_pending and node.profile_path is not None)
        self.pick.setEnabled(ready and not actions_pending and node.selection is not None
                             and not node.holding_item)
        self.stop.setEnabled("stop" not in self.pending_calls)
        self.live.setEnabled(
            not busy and not node.holding_item and "live" not in self.pending_calls)
        self.debug_images.setEnabled("debug_images" not in self.pending_calls)
        with QtCore.QSignalBlocker(self.live):
            self.live.setChecked(node.live)
        self._set_live_visual(node.live)
        with QtCore.QSignalBlocker(self.debug_images):
            self.debug_images.setChecked(node.debug_images)
        self._set_debug_images_visual(node.debug_images)
        if node.live:
            self.mode.setText("LIVE ROBOT · Actual commands enabled · SpeedFactor 100%")
            self.mode.setStyleSheet(
                "font-size:20px;font-weight:600;padding:12px;color:#b51f24")
            self.home.setText("Go Home")
            self.pick.setText("Pick Item")
            self.stop.setText("STOP")
        else:
            self.mode.setText("TF-ONLY · Live OFF · No hardware commands")
            self.mode.setStyleSheet(
                "font-size:20px;font-weight:600;padding:12px;color:#1573b7")
            self.home.setText("Preview Home TF")
            self.pick.setText("Preview Pick TF")
            self.stop.setText("Clear / Cancel")
        self.status.setText(
            f"{node.execution_state} · {node.execution_message}\n"
            f"Debug images: {node.debug_capture_status}")
        if node.selection is not None:
            selected = node.selection
            warning = selected.warning() or "Destination station and portable bin validated."
            self.station.setText(
                "Platform: " + selected.station.platform.path.name + "\nCamera: " +
                selected.station.camera.path.name + "\n" + warning)
        if node.fatal_error is not None or node.shutdown_requested.is_set() or not rclpy.ok():
            self.close()


def run_gui(node, executor):
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = ControllerWindow(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    window.show()
    try:
        application.exec_()
    finally:
        node.close_runtime()
    return thread
