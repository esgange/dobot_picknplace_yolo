"""Explicit-action operator window; restored filenames remain unapplied prefill."""

import threading

from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.executors import ExternalShutdownException

from .ui_state import save_state


class ControllerWindow(QtWidgets.QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.state_path = node.root / "logs/robot_controller/last_session.json"
        restored = node.ui_prefill  # Validated before real startup; never auto-applied.
        suffix = "TF-only Debug" if node.debug else "REAL ROBOT"
        self.setWindowTitle("Robot Controller — " + suffix)
        self.resize(960, 550)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        mode = QtWidgets.QLabel("TF-ONLY DEBUG · No hardware commands" if node.debug else
                                "REAL ROBOT · Startup enables robot · SpeedFactor 100%")
        color = "#1573b7" if node.debug else "#b51f24"
        mode.setStyleSheet("font-size:20px;font-weight:600;padding:12px;color:" + color)
        layout.addWidget(mode)
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
        self.home = QtWidgets.QPushButton("Preview Home TF" if node.debug else "Go Home")
        self.pick = QtWidgets.QPushButton("Preview Pick TF" if node.debug else "Pick Item")
        self.stop = QtWidgets.QPushButton("Clear / Cancel" if node.debug else "STOP")
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
            "Picks hold at final retract with suction ON. No placement.\n"
            "Live canonical feedback is required, including for debug Home.\n"
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
        try:
            self.node.start_action(action)
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, "Action rejected", str(exc))
        self.refresh()

    def stop_action(self):
        from types import SimpleNamespace
        self.node._stop_service(None, SimpleNamespace())
        self.refresh()

    def refresh(self):
        node = self.node
        busy = node.action_thread is not None and node.action_thread.is_alive()
        ready = node.execution_state in ("DEBUG", "READY", "HOLDING", "NO_PICK") and not busy
        self.apply.setEnabled(not busy)
        self.item_path.setReadOnly(busy)
        self.bin_path.setReadOnly(busy)
        for button in self.browse_buttons:
            button.setEnabled(not busy)
        self.home.setEnabled(ready and node.profile_path is not None)
        self.pick.setEnabled(ready and node.selection is not None and not node.holding_item)
        self.status.setText(f"{node.execution_state} · {node.execution_message}")
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
