"""Qt client for Robot Controller; contains no Dobot command clients."""

from collections import deque
from pathlib import Path
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
from robot_controller_interfaces.srv import Command, Configure, Preview, SetGlobalCP, SetGlobalSpeed

from .ui_state import load_state, save_state
from .feedback import FEEDBACK_MAX_AGE_SEC
from .placement import validate_target
from .operator_ui import acquisition_paused, button_policy, item_acquisition_paused, presentation
from .auto_detection import AutoDetectors, idle_reason, providers
from .perception_mode import ANY, TEACH, HEADLESS, check_mode
from .errors import FeedbackFailure
from .hmi_tools import BY_KEY, TOOLS, ToolLaunches, change_reason
from .hmi_modes import HmiModes
from .hmi_widgets import STATION_STYLE, ElidedLabel, hint, section


AUTO_READY_STYLE = (
    'QPushButton{background:#198754;color:white;font-weight:700;'
    'border:1px solid #146c43;border-radius:3px;padding:3px}'
    'QPushButton:enabled:hover{background:#157347}'
    'QPushButton:enabled:pressed{background:#12643e}'
    'QPushButton:disabled{background:#d7eadf;color:#5b7867}')


class GuiNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("robot_controller_gui")
        self.root = workspace_root()
        self.manage_controller = self.declare_parameter('manage_controller', False).value
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
            "clear_repick_history": self.create_client(
                Command, "/robot_controller/clear_repick_history"),
            "speed": self.create_client(
                SetGlobalSpeed, "/robot_controller/set_global_speed"),
            "cp": self.create_client(SetGlobalCP, "/robot_controller/set_global_cp"),
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

    def clear_controller_status(self):
        self._status_sample = None

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
        self.cp_pending_percent = None
        self.cp_syncing = False
        self.auto_detectors = AutoDetectors(node)
        self.auto_preparing = False
        self.auto_prepare_canceled = False
        self.auto_stop_again = False
        self.tools = ToolLaunches(node)
        self.modes = HmiModes(node, self.tools, self.auto_detectors, lambda: bool(
            self._tools_pending() or self.preview_mode or self.preview_clear_future is not None
            or self.auto_preparing or self.auto_stop_again))
        self.tool_controls = {}
        self.hmi_nodes = ()
        self.hmi_graph_at = -float('inf')
        self.setWindowTitle("Pick & Place — Station HMI")
        self.resize(1120, 760)
        font = self.font()
        font.setPointSize(10)
        self.setFont(font)
        self.setStyleSheet(STATION_STYLE)
        central = QtWidgets.QWidget()
        central.setObjectName('stationHmi')
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(16, 12, 16, 10)
        layout.setSpacing(10)
        self.pages = QtWidgets.QTabWidget()
        self.page_layouts = {}
        for name in ('Production', 'Recipe', 'Teach / Test', 'Diagnostics'):
            page = QtWidgets.QWidget()
            page.setProperty('role', 'page')
            page_layout = QtWidgets.QVBoxLayout(page)
            page_layout.setContentsMargins(0, 6, 0, 6)
            page_layout.setSpacing(12)
            page_layout.setAlignment(QtCore.Qt.AlignTop)
            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
            scroll.setWidget(page)
            self.pages.addTab(scroll, name)
            self.page_layouts[name] = page_layout
        production = self.page_layouts['Production']
        self.recipe_layout = self.page_layouts['Recipe']
        testing = self.page_layouts['Teach / Test']
        test_columns = QtWidgets.QHBoxLayout()
        test_columns.setSpacing(16)
        self.teaching_layout = QtWidgets.QVBoxLayout()
        self.testing_layout = QtWidgets.QVBoxLayout()
        self.teaching_layout.setSpacing(12)
        test_columns.addLayout(self.teaching_layout, 1)
        test_columns.addLayout(self.testing_layout, 1)
        testing.addLayout(test_columns)

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
        header.addWidget(self.robot_panel, 4)
        header.addWidget(self.gripper_panel, 3)
        self.station_panel, station_layout = self._status_panel('Station services')
        self.station_services = QtWidgets.QLabel('Waiting for station status…')
        self.station_services.setTextFormat(QtCore.Qt.PlainText)
        self.station_services.setWordWrap(True)
        self.station_services.setStyleSheet('font-size:12px;color:#edf3f8')
        station_layout.addWidget(self.station_services)
        station_layout.addStretch()
        header.addWidget(self.station_panel, 3)

        self.teach_panel = QtWidgets.QGroupBox("Teach files")
        self.teach_panel.setMinimumWidth(300)
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
            field_label = QtWidgets.QLabel(label)
            field_label.setBuddy(edit)
            edit.setAccessibleName(label + ' file')
            teach.addWidget(field_label, index, 0)
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
        teach.setColumnStretch(1, 1)
        self.recipe_layout.addWidget(self.teach_panel)
        layout.addLayout(header)

        mode_row = QtWidgets.QHBoxLayout()
        mode_row.addWidget(QtWidgets.QLabel('Control ownership'))
        self.manual_mode = QtWidgets.QPushButton('Manual — tools')
        self.auto_mode = QtWidgets.QPushButton('Auto — production')
        self.mode_buttons = QtWidgets.QButtonGroup(self)
        for button, mode in ((self.auto_mode, 'AUTO'), (self.manual_mode, 'MANUAL')):
            button.setCheckable(True)
            button.setMinimumHeight(36)
            button.setMinimumWidth(210)
            button.setStyleSheet('QPushButton{padding:5px 10px}'
                                 'QPushButton:checked{background:#315d7d;color:white;'
                                 'font-weight:700}')
            button.clicked.connect(lambda _checked=False, target=mode: self._set_mode(target))
            self.mode_buttons.addButton(button)
            mode_row.addWidget(button)
        self.mode_detail = QtWidgets.QLabel()
        self.mode_detail.setWordWrap(True)
        mode_row.addWidget(self.mode_detail, 1)
        layout.addLayout(mode_row)

        self.clear_repick = QtWidgets.QPushButton("Clear Repick History")
        self.clear_repick.clicked.connect(lambda: self._command("clear_repick_history"))
        self.load_auto = QtWidgets.QPushButton("Load Auto: OFF")
        self.load_auto.setCheckable(True)
        self.load_auto.clicked.connect(self._toggle_auto)
        production_columns = QtWidgets.QHBoxLayout()
        production_columns.setSpacing(16)
        job_panel, job = section('Selected job')
        self.recipe_summary = {}
        for name in ('Item', 'Bin', 'Tray'):
            job.addWidget(hint(name + ' teach file'))
            value = ElidedLabel()
            value.setMinimumWidth(0)
            job.addWidget(value)
            self.recipe_summary[name] = value
        self.job_settings = hint('')
        job.addWidget(self.job_settings)
        job.addWidget(self._page_link('Edit recipe / quantity', 'Recipe'))
        job.addStretch()
        prepare_panel, preparation = section('Offline production')
        preparation.addWidget(hint(
            '1 · Select the recipe and quantity.\n'
            '2 · Load Auto starts headless detectors and prepares the robot.\n'
            '3 · When READY, start the run.'))
        self.load_auto.setMinimumHeight(44)
        preparation.addWidget(self.load_auto)
        self.load_auto_status = QtWidgets.QLabel()
        self.load_auto_status.setTextFormat(QtCore.Qt.PlainText)
        self.load_auto_status.setWordWrap(True)
        self.load_auto_status.setStyleSheet('font-size:12px')
        self.load_auto_status.setMinimumHeight(32)
        preparation.addWidget(self.load_auto_status)
        production_columns.addWidget(job_panel, 1)
        production_columns.addWidget(prepare_panel, 1)
        production.addLayout(production_columns)
        preparation.addWidget(hint('Close teaching windows first. No server connection is needed.'))
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
            "QPushButton{background:#b51f24;color:white;font-weight:800;font-size:18px}"
            "QPushButton:disabled{background:#e2e2e2;color:#999}")
        self.stop.setToolTip("Stop immediately and preserve the gripper outputs")
        self.recover.clicked.connect(lambda: self._command("recover"))
        self.recover.setToolTip(
            "Cancel the action, preserve grip while lifting and returning Home, "
            "then test suction; detected suction offers Return Item before resetting outputs")
        self.stop.clicked.connect(self._immediate_stop)
        for button in (self.recover, self.pause, self.stop):
            button.setMinimumHeight(48)
            lifecycle.addWidget(button, 1)
        layout.addWidget(self.pages, 1)
        layout.addLayout(lifecycle)

        test_panel, test_run = section('Test with teaching detectors')
        test_run.addWidget(hint(
            'Return to Auto — production with Item / Tray Teach armed, '
            'then load the matching recipe. Keep Load Auto OFF.'))
        self.configure.setMinimumHeight(40)
        test_run.addWidget(self.configure)
        self.test_settings = hint('')
        test_run.addWidget(self.test_settings)
        self.testing_layout.addWidget(test_panel)
        self.single_actions, single_layout = section('Single actions')
        self.motion_grid = QtWidgets.QGridLayout()
        operations = self.motion_grid
        self.testing_layout.addWidget(self.single_actions)
        single_layout.addWidget(hint('Home, pick or place individually. Preview shows TF targets only.'))
        single_layout.addLayout(operations)
        single_layout.addWidget(hint('Place Item uses the same tray X/Y and rotation settings.'))
        single_layout.addStretch()
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
            "Move to Tray Detect if needed, observe tray, then place and retract; item optional")
        self.debug_images = QtWidgets.QCheckBox("Save item/tray debug RGB/depth")
        self.debug_images.setToolTip(
            "Save detection images for Pick Item, Place Item and both local runs. "
            "Item images: debug/pick_img; tray images: debug/tray_img. "
            "Keep the newest 500 captures per detector; delete older RGB/depth sets together.")
        self.preview_toggle.toggled.connect(self._toggle_preview)
        self.home_button.clicked.connect(lambda: self._action("home"))
        self.pick_item.clicked.connect(lambda: self._action("pick"))
        self.place_item.clicked.connect(lambda: self._action("place"))
        for button in (self.home_button, self.preview_toggle,
                       self.pick_item, self.place_item):
            button.setMinimumHeight(44)
        operations.addWidget(self.home_button, 0, 0)
        operations.addWidget(self.preview_toggle, 0, 1)
        operations.addWidget(self.pick_item, 1, 0)
        operations.addWidget(self.place_item, 1, 1)
        self.place_x = QtWidgets.QLineEdit()
        self.place_y = QtWidgets.QLineEdit()
        self.place_x.setPlaceholderText("Positive mm")
        self.place_y.setPlaceholderText("Positive mm")
        self.place_rotation = QtWidgets.QDoubleSpinBox()
        self.place_rotation.setMinimumHeight(36)
        self.place_rotation.setRange(-180., 180.)
        self.place_rotation.setDecimals(1)
        self.place_rotation.setValue(0.)
        self.place_rotation.setSuffix("°")
        self.place_rotation.setToolTip(
            "0° aligns item/tray X axis lines; add offset, then choose the nearest of two "
            "orientations 180° apart, preserving taught tilt")
        target_row = QtWidgets.QGridLayout()
        for column, (label, field) in enumerate((
                ("Tray X (mm)", self.place_x), ("Tray Y (mm)", self.place_y),
                ("Rotation offset", self.place_rotation))):
            target_row.addWidget(hint(label), 0, column)
            target_row.addWidget(field, 1, column)
            target_row.setColumnStretch(column, 1)
            field.setAccessibleName(label)
        if node.prefill and node.prefill["placement"] is not None:
            x, y, rotation = node.prefill["placement"]
            self.place_x.setText(str(x))
            self.place_y.setText(str(y))
            self.place_rotation.setValue(rotation)
        self.auto_run_button = QtWidgets.QPushButton("Manual/Test Run")
        self.auto_run_button.setMinimumHeight(48)
        self.auto_run_button.clicked.connect(lambda: self._action("auto_run", detector_mode=TEACH))
        self.headless_run_button = QtWidgets.QPushButton("Offline Auto Run")
        self.headless_run_button.setMinimumHeight(48)
        self.headless_run_button.clicked.connect(
            lambda: self._action("auto_run", detector_mode=HEADLESS))
        self.auto_quantity = QtWidgets.QSpinBox()
        self.auto_quantity.setMinimumHeight(36)
        self.auto_quantity.setRange(1, 10000)
        self.auto_quantity.setValue(1)
        self.auto_quantity.setAccessibleName("Run quantity")
        self.auto_progress = QtWidgets.QLabel("0 completed · Elapsed: 0.0 s")
        self.auto_progress.setWordWrap(True)
        self.auto_progress.setStyleSheet('font-size:24px;font-weight:600;color:#203e57')
        self.auto_progress.setToolTip(
            "Total run time, including retries, pauses and final Home. Resets on the next run.")
        quantity_row = QtWidgets.QHBoxLayout()
        quantity_row.addWidget(QtWidgets.QLabel('Quantity'))
        self.auto_quantity.setMaximumWidth(110)
        quantity_row.addWidget(self.auto_quantity)
        quantity_row.addSpacing(12)
        quantity_row.addStretch()
        settings_panel, settings = section('Run settings · shared by production and testing')
        settings.addLayout(quantity_row)
        settings.addLayout(target_row)
        settings.addWidget(hint(
            'X/Y are tray-local millimetres. 0° aligns item and tray X directions; '
            'the offset is added while preserving taught tilt.'))
        self.recipe_layout.addWidget(settings_panel)
        self.recipe_layout.addWidget(hint(
            'Selecting files does not load or prepare the robot. '
            'Use Load Auto on Production, or Load Teach Configuration on Teach / Test.'))
        recipe_nav = QtWidgets.QHBoxLayout()
        recipe_nav.addWidget(self._page_link('Go to Production', 'Production'))
        recipe_nav.addWidget(self._page_link('Go to Teach / Test', 'Teach / Test'))
        recipe_nav.addStretch()
        self.recipe_layout.addLayout(recipe_nav)
        self.recipe_layout.addStretch()
        preparation.addWidget(self.headless_run_button)
        self.production_blocker = hint('')
        self.production_blocker.setTextFormat(QtCore.Qt.PlainText)
        preparation.addWidget(self.production_blocker)
        preparation.addStretch()
        test_run.addWidget(self.auto_run_button)
        self.test_blocker = hint('')
        self.test_blocker.setTextFormat(QtCore.Qt.PlainText)
        test_run.addWidget(self.test_blocker)
        monitor_panel, monitor = section('Run monitor')
        monitor.addWidget(self.auto_progress)
        self.run_progress = QtWidgets.QProgressBar()
        self.run_progress.setAccessibleName('Confirmed placement progress')
        self.run_progress.setTextVisible(False)
        self.run_progress.setFixedHeight(14)
        monitor.addWidget(self.run_progress)
        monitor.addWidget(hint('Confirmed placements · elapsed time includes retries and operator waits.'))
        production.addWidget(monitor_panel)
        production.addStretch()
        self.availability_details = QtWidgets.QLabel()
        self.availability_details.setTextFormat(QtCore.Qt.PlainText)
        self.availability_details.setWordWrap(True)
        self.availability_details.setStyleSheet("color:#785000")
        self.testing_layout.addWidget(self.availability_details)
        self.testing_layout.addStretch()

        tuning_panel, tuning = section('Motion settings')
        tuning_grid = QtWidgets.QGridLayout()
        tuning_grid.setHorizontalSpacing(16)
        tuning_grid.setColumnStretch(1, 1)
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
        tuning_grid.addWidget(self.speed_label, 0, 0)
        tuning_grid.addWidget(self.speed_slider, 0, 1)

        self.cp_label = QtWidgets.QLabel("Global CP: unavailable")
        self.cp_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.cp_slider.setRange(0, 100)
        self.cp_slider.setValue(100)
        self.cp_slider.setTracking(True)
        self.cp_slider.sliderMoved.connect(self._cp_preview)
        self.cp_slider.sliderReleased.connect(self._cp_released)
        self.cp_slider.valueChanged.connect(self._cp_value_changed)
        self.cp_debounce = QtCore.QTimer(self)
        self.cp_debounce.setSingleShot(True)
        self.cp_debounce.setInterval(350)
        self.cp_debounce.timeout.connect(self._cp)
        tuning_grid.addWidget(self.cp_label, 1, 0)
        tuning_grid.addWidget(self.cp_slider, 1, 1)
        tuning.addLayout(tuning_grid)
        self.page_layouts['Diagnostics'].addWidget(tuning_panel)
        self._build_tool_pages()

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
        log_panel, log_layout = section('Command log')
        log_layout.addLayout(log_header)
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
        log_layout.addWidget(self.log_view)
        self.page_layouts['Diagnostics'].addWidget(log_panel)
        self.log_view.hide()
        focus_order = (
            self.auto_mode, self.manual_mode, self.recover, self.pause, self.stop, self.pages,
            self.auto_quantity, self.place_x, self.place_y, self.place_rotation, self.debug_images,
            self.headless_run_button, self.auto_run_button, self.home_button, self.preview_toggle,
            self.pick_item, self.place_item, self.speed_slider, self.cp_slider,
            self.item_path, self.teach_browse_buttons[0], self.bin_path, self.teach_browse_buttons[1],
            self.tray_path, self.teach_browse_buttons[2], self.load_auto, self.configure,
            *(self.tool_controls[key][0] for key in
              ('item', 'tray', 'bin', 'platform', 'rviz', 'cameras', 'calibration',
               'remote', 'motion', 'gripper')),
            self.clear_repick, self.log_toggle, self.copy_log, self.log_view)
        for previous, following in zip(focus_order, focus_order[1:]):
            self.setTabOrder(previous, following)
        self.item_path.textEdited.connect(self._selection_changed)
        self.bin_path.textEdited.connect(self._selection_changed)
        self.tray_path.textEdited.connect(self._selection_changed)
        self.place_x.textEdited.connect(self._clear_preview)
        self.place_y.textEdited.connect(self._clear_preview)
        self.place_rotation.valueChanged.connect(self._clear_preview)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(100)
        if self.modes.managed:
            try:
                self.modes.request('AUTO')
            except (ValueError, RuntimeError) as exc:
                self.modes.fail(str(exc))
        self._refresh()

    def _page_link(self, title, page):
        """Navigation only: never changes ownership, prepares or starts a process."""
        button = QtWidgets.QPushButton(title)
        index = tuple(self.page_layouts).index(page)
        button.clicked.connect(lambda: self.pages.setCurrentIndex(index))
        return button

    @staticmethod
    def _status_panel(title):
        panel = QtWidgets.QFrame()
        panel.setMinimumHeight(132)
        panel.setMaximumHeight(180)
        panel.setStyleSheet("QFrame{background:#202a35;border-radius:6px}"
                            "QLabel{color:#edf3f8;border:0;font-size:14px}")
        column = QtWidgets.QVBoxLayout(panel)
        column.setContentsMargins(12, 10, 12, 10)
        column.setSpacing(5)
        heading = QtWidgets.QLabel(title)
        heading.setStyleSheet("font-size:17px;font-weight:700;color:white")
        column.addWidget(heading)
        return panel, column

    def _build_tool_pages(self):
        self.teaching_layout.addWidget(self._tool_group(
            'Teaching windows', ('item', 'tray', 'bin', 'platform'),
            'Select Manual — tools to open. Load and arm profiles explicitly; '
            'keep Item / Tray Teach open for testing.', compact=True))
        self.teaching_layout.addStretch()

        debug = self.page_layouts['Diagnostics']
        debug.addWidget(hint(
            'Use Manual — tools for setup and direct maintenance. '
            'Use the active tool’s Stop controls while production is stopped.'))
        columns = QtWidgets.QHBoxLayout()
        columns.setSpacing(16)
        inspect = QtWidgets.QVBoxLayout()
        maintenance = QtWidgets.QVBoxLayout()
        inspect.addWidget(self._tool_group(
            'Inspection / calibration', ('rviz', 'cameras', 'calibration')))
        inspect.addWidget(self._tool_group('Server connection', ('remote',)))
        inspect.addStretch()
        maintenance.addWidget(self._tool_group(
            'Direct robot tools', ('motion', 'gripper'),
            'One command owner at a time. Stop motion in the tool before closing it.'))
        history_panel, history = section('Detection history')
        history.addWidget(hint(
            'Clear pick-attempt exclusions from the idle production controller. '
            'Requires Auto — production.'))
        history.addWidget(self.clear_repick)
        maintenance.addWidget(history_panel)
        capture_panel, capture = section('Detection captures')
        capture.addWidget(self.debug_images)
        capture.addWidget(hint(
            'Keep up to 500 captures per detector. Older RGB/depth sets are removed together.'))
        maintenance.addWidget(capture_panel)
        maintenance.addStretch()
        columns.addLayout(inspect, 1)
        columns.addLayout(maintenance, 1)
        debug.addLayout(columns)

    def _tool_group(self, title, keys, description='', *, compact=False):
        panel, body = section(title)
        if description:
            body.addWidget(hint(description))
        for index, key in enumerate(keys):
            tool = BY_KEY[key]
            if index:
                separator = QtWidgets.QFrame()
                separator.setFrameShape(QtWidgets.QFrame.HLine)
                separator.setStyleSheet('color:#d5dfe8')
                body.addWidget(separator)
            heading = QtWidgets.QLabel(tool.title)
            heading.setStyleSheet('font-weight:600')
            heading.setToolTip(tool.description)
            line = QtWidgets.QHBoxLayout()
            line.addWidget(heading, 1)
            button = QtWidgets.QPushButton('Open')
            button.setAccessibleName(tool.title + ' window')
            button.clicked.connect(
                lambda _checked=False, key=tool.key: self._tool_clicked(key))
            line.addWidget(button)
            body.addLayout(line)
            if not compact:
                body.addWidget(hint(tool.description))
            label = hint('Not running')
            label.setTextFormat(QtCore.Qt.PlainText)
            label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
            body.addWidget(label)
            self.tool_controls[tool.key] = (button, label)
        return panel

    def _tools_pending(self):
        return bool(self.pending or self.pending_goal is not None
                    or self.result_future is not None)

    def _set_mode(self, target):
        try:
            self.modes.request(target)
            if target == 'MANUAL':
                self.pages.setCurrentIndex(2)
        except (ValueError, RuntimeError) as exc:
            QtWidgets.QMessageBox.warning(self, 'Station mode', str(exc))
        self._refresh_controls()
        self._refresh_hmi()

    def _manual_tool_reason(self, tool):
        if (self.modes.managed and tool.kind not in ('viewer', 'communication')
                and self.modes.state != 'MANUAL'):
            return 'Select Manual mode before opening teaching/setup/debug tools'
        return ''

    def _tool_start_check(self, key):
        # Also called by the worker immediately before spawn: do not access Qt widgets here.
        tool = BY_KEY[key]
        nodes = self.node.get_node_names_and_namespaces()
        reason = self._manual_tool_reason(tool) or change_reason(
            tool, self.node.status, nodes, self.auto_detectors.snapshot[0], self._tools_pending())
        if reason:
            raise ValueError(reason)
        if any(name in tool.nodes for name, _namespace in nodes):
            raise ValueError(f'{tool.title} is already running; its existing window is external')
        if tool.kind == 'maintenance' and any(
                BY_KEY[other].kind in ('maintenance', 'calibration')
                for other in self.tools.active() - {key}):
            raise ValueError('Close other maintenance tools before opening this one')
        if tool.kind == 'maintenance':
            local = self.modes.local_owner()
            if local:
                raise ValueError('Close existing local command applications: ' + local)

    def _tool_clicked(self, key):
        tool = BY_KEY[key]
        state, _message = self.tools.snapshot(key)
        try:
            if state not in ('OFF', 'FAILED'):
                reason = ('' if tool.kind in ('viewer', 'maintenance', 'communication') else change_reason(
                    tool, self.node.status, self.hmi_nodes, self.auto_detectors.snapshot[0],
                    self._tools_pending()))
                if reason:
                    raise ValueError(reason)
                self.tools.stop(key)
            else:
                self.tools.start(key, lambda: self._tool_start_check(key))
        except (ValueError, RuntimeError) as exc:
            QtWidgets.QMessageBox.warning(self, tool.title, str(exc))
        self._refresh_hmi()

    def _refresh_hmi(self):
        self.mode_buttons.setExclusive(False)
        for button, target in ((self.manual_mode, 'MANUAL'), (self.auto_mode, 'AUTO')):
            selected = self.modes.state == target
            reason = self.modes.reason(target) if not selected else ''
            button.setChecked(selected)
            button.setEnabled(not selected and not reason)
            button.setToolTip(reason or (
                'Stop owned production processes; keep cameras/bringup' if target == 'MANUAL'
                else 'Start production controller; Load Auto and both local runs remain explicit'))
        self.mode_buttons.setExclusive(True)
        self.mode_detail.setText(self.modes.message if self.modes.managed else
                                 'External client; use the normal launch for mode switching')
        if time.monotonic() - self.hmi_graph_at >= 1.:
            try:
                self.hmi_nodes = tuple(self.node.get_node_names_and_namespaces())
            except RuntimeError:
                self.hmi_nodes = ()
            self.hmi_graph_at = time.monotonic()
        names = {name for name, _namespace in self.hmi_nodes}
        auto_state = self.auto_detectors.snapshot[0]
        for tool in TOOLS:
            state, message = self.tools.snapshot(tool.key)
            button, label = self.tool_controls[tool.key]
            owned = state not in ('OFF', 'FAILED')
            external = not owned and bool(names.intersection(tool.nodes))
            reason = change_reason(tool, self.node.status, self.hmi_nodes, auto_state,
                                   self._tools_pending())
            if owned and tool.kind in ('viewer', 'maintenance', 'communication'):
                reason = ''
            elif not owned:
                reason = self._manual_tool_reason(tool) or reason
            button.setText('External' if external else 'Close' if owned else 'Open')
            button.setEnabled(not external and state != 'STOPPING' and not reason)
            button.setToolTip('Opened elsewhere; this HMI cannot close it' if external else
                              reason or ('Close owned window' if owned else tool.description))
            label.setText('Running outside this HMI' if external else
                          'Opening…' if state == 'STARTING' else
                          'Closing…' if state == 'STOPPING' else
                          'Opened by this HMI' if owned else
                          'Failed — see log' if state == 'FAILED' else reason or 'Not running')
            label.setToolTip(message if state == 'FAILED' else reason or message)
        for key, state, message in self.tools.take_messages():
            self.log_view.appendPlainText(f'{BY_KEY[key].title} {state}: {message}')
        for key, state, message in self.modes.core.take_messages():
            self.log_view.appendPlainText(f'{BY_KEY[key].title} {state}: {message}')

        def detector(prefix):
            present = names.intersection({prefix + '_teach', prefix + '_detect'})
            return ('Teach + headless' if len(present) == 2 else
                    'Teach' if prefix + '_teach' in present else
                    'Headless' if present else 'Not running')
        status = self.node.status
        cameras = ('Supervisor seen' if ('supervisor', '/camera_watchdog') in self.hmi_nodes
                   else 'No supervisor')
        self.station_services.setText(
            f'Controller: {"Connected" if status is not None else "Unavailable"}\n'
            f'Robot feedback: {"Live" if status and status.feedback_fresh else "Unavailable"}'
            f' · TF: {"Seen" if "dobot_robot_state_publisher" in names else "Missing"}\n'
            f'Cameras: {cameras}\n'
            f'Item: {detector("item")} · Tray: {detector("tray")}\n'
            f'RViz: {"Open" if "dobot_rviz" in names else "Closed (optional)"}')
        self.station_services.setToolTip(
            'Observed services only; seeing a node does not prove input readiness. '
            'Load Auto verifies both headless detectors and robot preparation '
            'before enabling Offline Auto Run.')

    def _refresh_status(self, state):
        label, detail = presentation(state, self.preview_mode)
        self.status.setText(label)
        self.status_detail.setText(detail)
        color = ("#61dfa5" if label == "READY" else
                 "#ff9393" if label in ("ATTENTION REQUIRED", "EMERGENCY STOP PRESSED") else
                 "#ffd077" if label in (
                     "PAUSED", "OFFLINE", "HOLDING ITEM", "WAITING FOR ITEMS", "WAITING FOR TRAY")
                 else "#edf3f8")
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
        self.log_view.setVisible(visible)
        self.copy_log.setVisible(visible)
        self.log_toggle.setText("Hide command log" if visible else "Show command log")

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
        self.cp_debounce.stop()
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
        state = self.node.status
        owns_providers = (self.auto_detectors.snapshot[0] not in ('OFF', 'FAILED')
                          or bool(self.tools.active().intersection({'item', 'tray'})))
        if ((self.auto_preparing and not self.auto_prepare_canceled)
                or (owns_providers and state is not None and state.operation_active)):
            self._immediate_stop()
        self.auto_detectors.close()
        self.tools.close()
        self.modes.close()
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
        self._configure_paths(tuple(field.text().strip() for field in (
            self.item_path, self.bin_path, self.tray_path)))

    def _configure_paths(self, paths):
        request = Configure.Request()
        request.item_teach_file, request.bin_teach_file, request.tray_teach_file = paths
        self.saved_selection = paths
        if not self._call("configure", request):
            self.saved_selection = None
            return False
        return True

    def _prepare_auto(self):
        auto_state = self.auto_detectors.snapshot[0]
        if (self.auto_preparing and not self.auto_prepare_canceled
                and auto_state in ('STOPPING', 'FAILED', 'OFF')):
            # Detector failure/cancellation must also contain an issued Configure.
            self._immediate_stop()
        paths = self.auto_detectors.take_preparation()
        if paths is None:
            return
        reason = (('Select Auto mode first' if self.modes.managed
                   and self.modes.state != 'AUTO' else '') or idle_reason(self.node.status))
        if (reason or self.preview_mode or self.pending or self.pending_goal is not None
                or self.result_future is not None or self.preview_clear_future is not None):
            self.auto_detectors.prepared(error=reason or 'Another GUI operation is pending')
            return
        self.auto_preparing = True
        self.auto_prepare_canceled = False
        try:
            if not self._configure_paths(paths):
                self.auto_preparing = False
                self.auto_detectors.prepared(error='Controller Configure service is unavailable')
        except Exception as exc:
            self.auto_detectors.prepared(error=str(exc))
            self._immediate_stop()
            self.auto_preparing = False

    def _command(self, name):
        if name != "stop" and self._availability().get(name, "Unavailable command"):
            return False
        return self._call(name, Command.Request())

    _acquisition_paused = staticmethod(acquisition_paused)

    def _immediate_stop(self):
        if self.preview_mode:
            self._clear_preview()
        # Dispatch the independent Stop before optional action cancellation.
        if self.auto_detectors.snapshot[0] in ('DEPLOYING', 'STARTING', 'PREPARING'):
            self.auto_detectors.stop()
        if self.auto_preparing:
            self.auto_prepare_canceled = True
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

    def _sync_cp_slider(self, percent):
        self.cp_syncing = True
        try:
            self.cp_slider.setValue(percent)
        finally:
            self.cp_syncing = False

    def _cp_preview(self, percent):
        self.cp_label.setText(f"Global CP: {int(percent)}% selected")

    def _cp_value_changed(self, percent):
        if self.cp_syncing:
            return
        self._cp_preview(percent)
        if not self.cp_slider.isSliderDown() and self.cp_slider.hasFocus():
            self.cp_debounce.start()

    def _cp_released(self):
        self.cp_debounce.stop()
        self._cp()

    def _cp(self):
        self.cp_debounce.stop()
        if self._availability()["cp"]:
            return
        if "cp" in self.pending:
            return
        percent = int(self.cp_slider.sliderPosition())
        status = self.node.status
        if (status is not None and status.global_cp_percent == percent
                and self.cp_pending_percent is None):
            self.cp_label.setText(f"Global CP: {percent}%")
            return
        request = SetGlobalCP.Request()
        request.percent = percent
        if self._call("cp", request):
            self.cp_pending_percent = percent
            self.cp_label.setText(f"Global CP: requesting {percent}%")

    def _feedback(self, message):
        feedback = message.feedback
        suffix = ""
        if hasattr(feedback, "candidate_total") and feedback.candidate_total:
            suffix = f" · candidate {feedback.candidate_index}/{feedback.candidate_total}"
        self.feedback_message = f"{feedback.phase} · {feedback.message}{suffix}"

    def _action(self, name, *, detector_mode=ANY):
        if self._availability()[name]:
            return
        if name == 'auto_run' and self._run_reason(detector_mode):
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
        if name in ("pick", "place", "auto_run"):
            goal.save_debug_images = self.debug_images.isChecked()
        if name == "auto_run":
            goal.quantity = self.auto_quantity.value()
            goal.detector_mode = detector_mode
            self.auto_progress.setText(f"0/{goal.quantity} completed")
        self.pending_goal = client.send_goal_async(goal, feedback_callback=self._feedback)
        self._refresh_controls()

    def _collect(self):
        self._prepare_auto()
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
                if (name == 'configure' and self.auto_preparing
                        and self.auto_prepare_canceled and accepted):
                    # The service may have completed after cancellation's first Stop.
                    self.auto_stop_again = True
                if result is None or not result.success:
                    message = "No response" if result is None else result.message
                    if name == 'configure' and self.auto_preparing:
                        self.auto_detectors.prepared(error=message)
                    if name == "speed":
                        self.speed_pending_percent = None
                        if result is not None and result.confirmed_percent >= 1:
                            self._sync_speed_slider(result.confirmed_percent)
                    elif name == "cp":
                        self.cp_pending_percent = None
                        if result is not None and result.confirmed_percent >= 0:
                            self._sync_cp_slider(result.confirmed_percent)
                    QtWidgets.QMessageBox.warning(self, f"{name} rejected", message)
                elif name == "configure" and self.saved_selection is not None:
                    save_state(
                        self.node.root / "logs/robot_controller/last_session.json",
                        *self.saved_selection)
                    self._clear_preview()
                    if self.auto_preparing and not self.auto_prepare_canceled:
                        self.auto_detectors.prepared(result.configuration_id)
                elif name == "speed":
                    self.speed_pending_percent = result.confirmed_percent
                    self._sync_speed_slider(result.confirmed_percent)
                    self.speed_label.setText(
                        f"Global SpeedFactor: {result.confirmed_percent}% confirmed")
                elif name == "cp":
                    self.cp_pending_percent = result.confirmed_percent
                    self._sync_cp_slider(result.confirmed_percent)
                    self.cp_label.setText(
                        f"Global CP: {result.confirmed_percent}% confirmed")
                elif name == "preview":
                    self.feedback_message = result.message
                    self.log_view.appendPlainText("PREVIEW: " + result.message)
                elif name == "recover" and result.state == "READY":
                    QtWidgets.QMessageBox.information(
                        self, "Recovered — gripper relaxed",
                        "Recovery completed at Home and the suction test passed. "
                        "Fingers are relaxed, suction and "
                        "exhaust are OFF, and DI1 is LOW. The interrupted action was cancelled.")
            except Exception as exc:
                if name == 'configure' and self.auto_preparing:
                    self.auto_detectors.prepared(error=str(exc))
                    self._immediate_stop()
                if name == "speed":
                    self.speed_pending_percent = None
                elif name == "cp":
                    self.cp_pending_percent = None
                QtWidgets.QMessageBox.warning(self, f"{name} failed", str(exc))
            finally:
                if name == 'configure':
                    self.auto_preparing = False
                if name == "pause" and not accepted:
                    self.pause_requested_locally = False
                if name == "return_item" and not accepted:
                    self.return_requested_locally = False
        if self.auto_stop_again and 'stop' not in self.pending:
            if self._command('stop'):
                self.auto_stop_again = False
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
                        f"{result.completed_quantity}/{result.requested_quantity} completed"
                        f" · Total: {result.elapsed_sec:.1f} s")
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
        self.modes.poll()
        self._refresh_controls()
        self._refresh_hmi()

    def _auto_change_reason(self):
        if self.modes.managed and self.modes.state != 'AUTO':
            return 'Select Auto mode and wait for the production controller'
        state, _message = self.auto_detectors.snapshot
        if state in ('DEPLOYING', 'STARTING', 'PREPARING'):
            return ''  # PREPARING cancellation also uses the independent Stop.
        if state == 'STOPPING':
            return 'Wait for the owned detectors to stop'
        if (self.preview_mode or self.pending or self.auto_stop_again
                or self.pending_goal is not None
                or self.result_future is not None or self.preview_clear_future is not None):
            return 'Finish the pending action and leave Preview before changing detectors'
        reason = idle_reason(self.node.status)
        if reason:
            return reason
        if state != 'RUNNING' and not all(field.text().strip() for field in (
                self.item_path, self.bin_path, self.tray_path)):
            return 'Select Item Teach, Bin Teach and Tray Teach first'
        if state in ('OFF', 'FAILED'):
            blockers = [BY_KEY[key].title for key in self.tools.active()
                        if BY_KEY[key].kind in ('teach', 'calibration', 'maintenance')]
            if blockers:
                return ('Close HMI teaching/maintenance windows first: '
                        + ', '.join(sorted(blockers)))
        return ''

    def _toggle_auto(self, enabled):
        if not self._auto_change_reason():
            if enabled:
                try:
                    self.auto_detectors.start(tuple(field.text().strip() for field in (
                        self.item_path, self.bin_path, self.tray_path)))
                except ValueError as exc:
                    QtWidgets.QMessageBox.warning(self, 'Load Auto', str(exc))
            else:
                if self.auto_preparing:
                    self._immediate_stop()
                else:
                    self.auto_detectors.stop()
        self._refresh_controls()

    def _run_reason(self, mode):
        if mode == ANY:
            return ''
        if mode == HEADLESS and self.auto_detectors.snapshot[0] != 'RUNNING':
            return 'Enable Load Auto and wait for both headless detectors to be ready'
        try:
            for kind in ('pick', 'place'):
                check_mode(providers(self.node, kind), kind, mode)
        except (FeedbackFailure, RuntimeError) as exc:
            return str(exc)
        return ''

    def _availability(self):
        try:
            validate_target(float(self.place_x.text()), float(self.place_y.text()),
                            self.place_rotation.value())
            target_error = ""
        except ValueError:
            target_error = "Enter positive X/Y in mm and a valid rotation"
        reasons = button_policy(
            self.node.status, preview=self.preview_mode,
            pending=bool(self.pending or self.auto_stop_again
                         or self.preview_clear_future is not None
                         or self.auto_detectors.snapshot[0] in
                         ('DEPLOYING', 'STARTING', 'PREPARING', 'STOPPING')),
            action_pending=self.pending_goal is not None or self.result_future is not None,
            managed_pending=self.pause_requested_locally or self.return_requested_locally,
            target_error=target_error, item_selected=bool(self.item_path.text().strip()),
            bin_selected=bool(self.bin_path.text().strip()),
            tray_selected=bool(self.tray_path.text().strip()))
        if self.modes.managed and self.modes.state != 'AUTO':
            return dict.fromkeys(reasons, 'Select Auto mode for production controls')
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
        if self.modes.managed and self.modes.state != 'AUTO':
            self.status.setText('MANUAL TOOLS' if self.modes.state == 'MANUAL' else
                                'CHANGING MODE' if self.modes.state != 'FAILED' else
                                'MODE ATTENTION')
            self.status_detail.setText(
                'Production controller stopped. Use the active tool’s controls and Stop.'
                if self.modes.state == 'MANUAL' else self.modes.message)
        self.stop.setText('STOP (production)' if self.modes.state == 'MANUAL' else 'STOP')
        self.stop.setToolTip(
            'Production controller is stopped; use Stop in the active manual tool'
            if self.modes.state == 'MANUAL' else 'Stop immediately and preserve gripper outputs')
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
        recovery_paused = bool(paused and state.phase == "RECOVERY_SUCTION_BLOCKED")
        self.managed_actions["continue"].setText(
            "RETEST SUCTION" if recovery_paused else "CONTINUE")
        if paused:
            managed_name = ("return_item" if state.can_return_item
                            and not reasons["return_item"] else "continue")
        for name, action in self.managed_actions.items():
            action.setEnabled(not reasons[name])
        self.managed_primary = managed_name
        self.pause.setText(self.managed_actions[managed_name].text())
        menu = self.pause_menu if paused and state.can_return_item else None
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
            "clear_repick_history": self.clear_repick,
            "configure": self.configure, "recover": self.recover, managed_name: self.pause,
            "home": self.home_button, "pick": self.pick_item, "place": self.place_item,
            "preview_toggle": self.preview_toggle, "speed": self.speed_slider,
            "cp": self.cp_slider,
            "auto_run": self.auto_run_button,
        }
        descriptions = {
            "clear_repick_history": "Clear tracked pick attempts and active exclusion circles",
            "auto_run": "Pick and place the requested quantity, then return Home",
            "configure": "Load teach files, enable and initialize the robot to READY",
            "continue": ("Retest suction at Home after clearing the item or obstruction"
                         if recovery_paused else
                         "Clear repick/trickle history and retry with fresh item detections"
                         if item_acquisition_paused(state) else
                         "Retry tray detection for the retained item" if retry else
                         "Resume the retained operation"),
            "recover": "Cancel the action, preserve grip to Home, then test suction",
            "pause": "Stop and confirm the operation's paused position",
            "return_item": "Return the held item to its saved bin position, then Home",
            "home": "Move Home while preserving any held item",
            "pick": "Pick an item and carry it to Tray Detect",
            "place": "Retry tray acquisition with up to 10 fresh requests" if retry else
                     "Move to Tray Detect if needed, then place and retract; item optional",
            "preview_toggle": "ON: motion buttons show TFs only. OFF: real robot motion.",
            "speed": "Change the global motion speed factor",
            "cp": "Change global path blending: 0% minimum, 100% maximum",
        }
        for name, action in self.managed_actions.items():
            action.setToolTip(reasons[name] or descriptions[name])
        for name, button in buttons.items():
            button.setEnabled(not reasons[name])
            tip = reasons[name] or descriptions[name]
            if self.preview_mode and name in ("home", "pick", "place") and not reasons[name]:
                tip = "Show planned TF targets; no robot motion or gripper output"
            button.setToolTip(tip)
        for button, mode, description in (
                (self.auto_run_button, TEACH,
                 'Debug/test the requested quantity using armed Item Teach and Tray Teach windows'),
                (self.headless_run_button, HEADLESS,
                 'Run offline using the Load Auto headless detectors; no server needed')):
            reason = reasons['auto_run'] or self._run_reason(mode)
            button.setEnabled(not reason)
            button.setToolTip(reason or description)
        for button, label in ((self.headless_run_button, self.production_blocker),
                              (self.auto_run_button, self.test_blocker)):
            label.setText(button.toolTip() if not button.isEnabled() else 'Ready to start a new run.')
        auto_state, auto_message = self.auto_detectors.snapshot
        self.headless_run_button.setStyleSheet(
            AUTO_READY_STYLE if self.headless_run_button.isEnabled() else '')
        self.load_auto.setStyleSheet(AUTO_READY_STYLE if auto_state == 'RUNNING' else '')
        auto_active = auto_state not in ('OFF', 'FAILED')
        self.load_auto.blockSignals(True)
        self.load_auto.setChecked(auto_active)
        self.load_auto.blockSignals(False)
        self.load_auto.setText('Load Auto: ' + ('ON' if auto_state == 'RUNNING' else auto_state))
        auto_reason = self._auto_change_reason()
        self.load_auto.setEnabled(not auto_reason)
        self.load_auto.setToolTip(auto_reason or (
            'OFF stops the owned detectors. ON replaces runtime_teach with the selected '
            'teach files and models, then starts Item Detect and Tray Detect. '
            'It also loads the selected teach configuration and prepares/enables the robot. '
            'Offline Auto Run remains a separate click.'))
        self.load_auto_status.setText(auto_message if len(auto_message) <= 180 else
                                      'Load Auto failed; see command log for details')
        self.load_auto_status.setToolTip(auto_message)
        for event_state, message in self.auto_detectors.take_messages():
            self.log_view.appendPlainText(f'Load Auto {event_state}: {message}')
        self.stop.setEnabled(self.node.service_clients["stop"].service_is_ready())
        editable = (state is not None and not state.operation_active and not self.pending
                    and self.pending_goal is None and self.result_future is None)
        for field in (self.place_x, self.place_y, self.place_rotation, self.auto_quantity):
            field.setEnabled(editable)
        if state is None:
            self.auto_progress.setText("Run status unavailable")
        elif state.auto_run_requested:
            timing = "Elapsed" if state.auto_run_active else "Total"
            self.auto_progress.setText(
                f"{state.auto_run_completed}/{state.auto_run_requested} completed"
                f" · {timing}: {state.auto_run_elapsed_sec:.1f} s")
        else:
            self.auto_progress.setText("0 completed · Elapsed: 0.0 s")
        requested = state.auto_run_requested if state else 0
        self.run_progress.setRange(0, max(1, requested))
        self.run_progress.setValue(state.auto_run_completed if state and requested else 0)
        self.run_progress.setEnabled(state is not None)
        self.run_progress.setAccessibleDescription(self.auto_progress.text())
        for name, field in (('Item', self.item_path), ('Bin', self.bin_path), ('Tray', self.tray_path)):
            path = field.text().strip()
            self.recipe_summary[name].setText(Path(path).name if path else 'Not selected')
            self.recipe_summary[name].setToolTip(path or 'Select a teach file on Recipe')
        settings = (
            f'Quantity: {self.auto_quantity.text()} · '
            f'Tray X: {self.place_x.text() or "—"} mm · '
            f'Y: {self.place_y.text() or "—"} mm · Rotation: {self.place_rotation.value():g}°')
        self.job_settings.setText(settings)
        self.test_settings.setText(settings)
        for field in (self.item_path, self.bin_path, self.tray_path, *self.teach_browse_buttons):
            field.setEnabled(editable and not auto_active
                             and current in ("UNCONFIGURED", "INACTIVE", "READY"))
        self.debug_images.setEnabled(not self.preview_mode and editable)
        visible_reasons = []
        if not self.pending and current in ("READY", "HOLDING", "PAUSED", "INACTIVE",
                                            "FAULT", "RECOVERY_REQUIRED", "HELD_UNKNOWN"):
            for name in (("place", "return_item") if retry else
                         ("continue", "return_item") if paused else
                         ("recover",) if current in
                         ("FAULT", "RECOVERY_REQUIRED", "HELD_UNKNOWN") else
                         ("configure",) if current == "INACTIVE" and not self.preview_mode else
                         ("pick", "place", "auto_run")):
                if reasons[name] and reasons[name] != "Unavailable in the current state":
                    label = (self.managed_actions[name].text() if name in self.managed_actions
                             else "Manual/Test Run / Offline Auto Run" if name == "auto_run"
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
        if state:
            if (self.cp_pending_percent is not None
                    and "cp" not in self.pending
                    and state.global_cp_percent == self.cp_pending_percent):
                self.cp_pending_percent = None
            editing_cp = (
                self.cp_slider.isSliderDown() or self.cp_debounce.isActive()
                or "cp" in self.pending or self.cp_pending_percent is not None)
            if state.global_cp_percent >= 0 and not editing_cp:
                self._sync_cp_slider(state.global_cp_percent)
            if self.cp_slider.isSliderDown() or self.cp_debounce.isActive():
                cp_text = f"{self.cp_slider.sliderPosition()}% selected"
            elif self.cp_pending_percent is not None:
                suffix = "requesting" if "cp" in self.pending else "confirmed"
                cp_text = f"{self.cp_pending_percent}% {suffix}"
            else:
                cp_text = (f"{state.global_cp_percent}%"
                           if state.global_cp_percent >= 0 else "unknown")
            self.cp_label.setText(f"Global CP: {cp_text}")
        else:
            self.cp_label.setText("Global CP: unavailable")


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
