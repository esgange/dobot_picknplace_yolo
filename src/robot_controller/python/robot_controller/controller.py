"""Robot Controller v2: explicit Startup, deterministic Home/Pick, native cancellation."""

import fcntl
import json
import os
from pathlib import Path
import signal
import threading

import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from dobot_msgs_v4.msg import RobotStatus
from item_perception_yolo.item_teach_core import utc_now
from item_perception_yolo.platform_teach_core import (
    _parse_env_file, load_robot_lan1_ip, workspace_root)
from robot_controller_interfaces.action import GoHome, PickItem
from robot_controller_interfaces.msg import ControllerStatus
from robot_controller_interfaces.srv import Command, Configure, SetGlobalSpeed

from .candidates import CandidateClient
from .configuration import load_configuration, load_runtime_configuration
from .errors import (CommandRejected, CommandResponseTimeout, FeedbackFailure,
                     HeldUnknown, OperationCanceled, StopUnconfirmed)
from .feedback import FeedbackMonitor, enabled_blockers
from .hardware import DobotTransport
from .kinematics import Cr10Kinematics
from .motion import PickExecutor, home_targets, pick_targets
from .state_machine import ControllerStateMachine


class PackageEventLogger:
    """Package-owned bounded JSONL event recorder."""

    def __init__(self, root, node_name="robot_controller"):
        self.path = Path(root) / "logs/robot_controller/events.jsonl"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self.node_name = node_name

    def record(self, level, event, message, **fields):
        payload = {
            "timestamp_utc": utc_now(), "package": "robot_controller",
            "node": self.node_name, "level": level, "event": event,
            "message": message, **fields,
        }
        with self.lock:
            with self.path.open("a+", encoding="utf-8") as stream:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
                stream.seek(0)
                count = sum(bool(line.strip()) for line in stream)
                if count >= 1000:
                    stream.seek(0)
                    stream.truncate()
                else:
                    stream.seek(0, os.SEEK_END)
                stream.write(json.dumps(payload, sort_keys=True, allow_nan=False) + "\n")
                stream.flush()
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


class RobotController(Node):
    """The sole application-level Dobot command authority."""

    def __init__(self):
        super().__init__("robot_controller")
        self.root = workspace_root()
        self.robot_ip = load_robot_lan1_ip(self.root)
        env = _parse_env_file(self.root / ".env")
        self.bringup_node = "/" + env["DOBOT_ROBOT_NODE_NAME"]
        self.headless = self.declare_parameter("headless", False).value
        if type(self.headless) is not bool:
            raise ValueError("headless must be an explicit Boolean")

        self.events = PackageEventLogger(self.root)
        model_path = Path(get_package_share_directory("cra_description")) / "urdf/cr10_robot.xacro"
        self.kinematics = Cr10Kinematics(model_path)
        self.machine = ControllerStateMachine()
        self.configuration = None
        self.startup_complete = False
        self.global_speed_percent = None
        self.holding_item = False
        self.expected_outputs = {}

        self.operation_lock = threading.Lock()
        self.control_group = ReentrantCallbackGroup()
        self.cancel_event = threading.Event()
        self.shutdown_event = threading.Event()
        self.active_goal = None
        self.active_action = ""
        self.phase = ""
        self.waypoint = ""
        self.candidate_index = 0
        self.candidate_total = 0

        self.stop_guard = threading.RLock()
        self.stop_confirmation_guard = threading.Lock()
        self.stop_future = None
        self.stop_confirmed = False
        self.stop_error = None
        self.state_before_stop = None
        self.late_stop_thread = None
        self.supervision_stop_thread = None

        self.monitor = FeedbackMonitor(lambda: self.get_clock().now().nanoseconds)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(
            RobotStatus, "/dobot_msgs_v4/msg/RobotStatus", self._on_status, 10)
        self.create_subscription(
            String, "/dobot_bringup_ros2/msg/FeedInfo", self._on_feed, 10)
        self.hardware = DobotTransport(self, self.monitor)
        self.candidates = CandidateClient(self, self.root)

        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_publisher = self.create_publisher(
            ControllerStatus, "/robot_controller/status", status_qos)
        self.startup_service = self.create_service(
            Command, "/robot_controller/startup", self._startup,
            callback_group=self.control_group)
        self.recover_service = self.create_service(
            Command, "/robot_controller/recover", self._recover,
            callback_group=self.control_group)
        self.stop_service = self.create_service(
            Command, "/robot_controller/stop", self._stop,
            callback_group=self.control_group)
        self.configure_service = self.create_service(
            Configure, "/robot_controller/configure", self._configure,
            callback_group=self.control_group)
        self.speed_service = self.create_service(
            SetGlobalSpeed, "/robot_controller/set_global_speed", self._set_global_speed,
            callback_group=self.control_group)
        self.home_server = ActionServer(
            self, GoHome, "/robot_controller/go_home",
            execute_callback=self._execute_home_action,
            goal_callback=self._home_goal, cancel_callback=self._cancel_goal,
            callback_group=self.control_group)
        self.pick_server = ActionServer(
            self, PickItem, "/robot_controller/pick_item",
            execute_callback=self._execute_pick_action,
            goal_callback=self._pick_goal, cancel_callback=self._cancel_goal,
            callback_group=self.control_group)

        if self.headless:
            self.configuration = load_runtime_configuration(self.root, self.kinematics)
            self._transition("INACTIVE", "Runtime teach loaded; explicit Startup required")
            self._log_configuration("runtime_configuration_loaded")
        self.create_timer(0.2, self.publish_status)
        self.create_timer(0.1, self._supervise, callback_group=self.control_group)
        self.publish_status()
        self.events.record(
            "INFO", "node_started",
            "Controller available; launch performed no enable or motion command",
            headless=self.headless, state=self.machine.state,
            cr10_model_sha256=self.kinematics.sha256)

    # ---------- feedback and ownership ----------

    def _on_joints(self, message):
        try:
            self.monitor.update_joints(message)
        except FeedbackFailure as exc:
            self.events.record("WARNING", "invalid_joint_feedback", str(exc))

    def _on_status(self, message):
        self.monitor.update_status(message)

    def _on_feed(self, message):
        try:
            self.monitor.update_feed(message.data)
        except FeedbackFailure as exc:
            self.events.record("WARNING", "invalid_feed_feedback", str(exc))

    def _sole_publisher(self, topic):
        endpoints = self.get_publishers_info_by_topic(topic)
        if (len(endpoints) != 1 or endpoints[0].node_namespace != "/"
                or "/" + endpoints[0].node_name != self.bringup_node):
            raise CommandRejected(
                f"{topic} requires sole canonical publisher {self.bringup_node}")

    def check_feedback_owners(self):
        for topic in ("/joint_states", "/dobot_msgs_v4/msg/RobotStatus",
                      "/dobot_bringup_ros2/msg/FeedInfo"):
            self._sole_publisher(topic)

    def _service_providers(self, absolute_name):
        providers = []
        for node_name, namespace in self.get_node_names_and_namespaces():
            try:
                names = self.get_service_names_and_types_by_node(node_name, namespace)
            except RuntimeError:
                continue
            if absolute_name in (name for name, _types in names):
                providers.append((node_name, namespace))
        return providers

    def check_command_owner(self, service):
        nodes = self.get_node_names_and_namespaces()
        if nodes.count((self.get_name(), self.get_namespace())) != 1:
            raise CommandRejected("Duplicate robot_controller identity")
        forbidden = {"motion_debug_gui", "gripper_control_gui", "motion_debug",
                     "gripper_control"}
        running = sorted(name for name, _namespace in nodes if name in forbidden)
        if running:
            raise CommandRejected(
                "Competing maintenance application must be stopped: " + ", ".join(running))
        absolute = f"/dobot_bringup_ros2/srv/{service}"
        expected = (self.bringup_node[1:], "/")
        providers = self._service_providers(absolute)
        if providers != [expected]:
            raise CommandRejected(
                f"{absolute} requires sole canonical provider {expected}; got {providers}")

    def check_all_command_owners(self, services):
        for service in dict.fromkeys(services):
            self.check_command_owner(service)

    def check_detector_owner(self):
        providers = self._service_providers("/item_detect/get_item_poses")
        if providers != [("item_detect", "/")]:
            raise FeedbackFailure(
                "Pose service requires the sole canonical /item_detect node provider")

    # ---------- state/status ----------

    def _transition(self, state, message):
        before = self.machine.state
        snapshot = (self.machine.update(message) if before == state
                    else self.machine.transition(state, message))
        self.events.record(
            "INFO", "state_transition", message, previous=before, state=snapshot.state,
            configuration_id=(self.configuration.configuration_id
                              if self.configuration else ""))
        self.publish_status()

    def operation_progress(self, phase, message, *, waypoint="", candidate_index=None,
                           candidate_total=None):
        self.phase, self.waypoint = phase, waypoint
        if candidate_index is not None:
            self.candidate_index = candidate_index
        if candidate_total is not None:
            self.candidate_total = candidate_total
        self.machine.update(message)
        self.events.record(
            "INFO", "operation_phase", message, operation=self.active_action,
            phase=phase, waypoint=waypoint, candidate_index=self.candidate_index,
            candidate_total=self.candidate_total)
        goal = self.active_goal
        if goal is not None:
            feedback = GoHome.Feedback() if self.active_action == "home" else PickItem.Feedback()
            if self.active_action == "pick":
                feedback.candidate_index = self.candidate_index
                feedback.candidate_total = self.candidate_total
            feedback.phase, feedback.waypoint, feedback.message = phase, waypoint, message
            goal.publish_feedback(feedback)
        self.publish_status()

    def publish_status(self):
        if not hasattr(self, "status_publisher"):
            return
        status = ControllerStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "base_link"
        status.state, status.message = self.machine.state, self.machine.message
        status.configuration_id = (
            self.configuration.configuration_id if self.configuration else "")
        status.configured = self.configuration is not None
        status.holding_item = self.holding_item
        status.operation_active = self.operation_lock.locked()
        status.operation, status.phase, status.waypoint = (
            self.active_action, self.phase, self.waypoint)
        status.candidate_index, status.candidate_total = (
            self.candidate_index, self.candidate_total)
        status.global_speed_percent = (
            self.global_speed_percent if self.global_speed_percent is not None else -1)
        status.startup_complete = self.startup_complete
        try:
            self.monitor.snapshot(require_enabled=False)
            status.feedback_fresh = True
        except FeedbackFailure:
            status.feedback_fresh = False
        self.status_publisher.publish(status)

    def _begin_operation(self, name):
        if not self.operation_lock.acquire(blocking=False):
            raise CommandRejected("Another controller operation is active")
        self.cancel_event.clear()
        with self.stop_guard:
            self.stop_future = None
            self.stop_confirmed = False
            self.stop_error = None
            self.state_before_stop = None
        self.active_action, self.phase, self.waypoint = name, "", ""
        self.candidate_index = self.candidate_total = 0

    def _end_operation(self):
        self.active_goal = None
        self.active_action = self.phase = self.waypoint = ""
        self.candidate_index = self.candidate_total = 0
        self.operation_lock.release()
        self.publish_status()

    def cancel_requested(self):
        return (self.cancel_event.is_set() or self.shutdown_event.is_set()
                or not rclpy.ok())

    def raise_if_cancelled(self):
        if self.cancel_requested():
            raise OperationCanceled("Controller operation cancelled")

    def wait_control(self, seconds):
        self.shutdown_event.wait(seconds)

    # ---------- configuration and lifecycle services ----------

    def _log_configuration(self, event):
        config = self.configuration
        self.events.record(
            "INFO", event, "Controller configuration installed",
            configuration_id=config.configuration_id,
            item_teach_file=str(config.item_path),
            bin_teach_file=str(config.bin_path) if config.bin_path else "",
            profile_sha256=config.profile_sha256,
            pose_candidates=config.pose_candidates,
            warning=config.selection.warning() if config.selection else "")

    def _configure(self, request, response):
        acquired = False
        try:
            if self.headless:
                raise CommandRejected("Headless runtime_teach configuration is immutable")
            if self.machine.state not in ("UNCONFIGURED", "INACTIVE"):
                raise CommandRejected("Configure requires UNCONFIGURED or INACTIVE state")
            self._begin_operation("configure")
            acquired = True
            config = load_configuration(
                request.item_teach_file, request.bin_teach_file, self.root,
                self.kinematics, deployment=False)
            self.configuration = config
            self.startup_complete = False
            self.global_speed_percent = None
            self.expected_outputs.clear()
            self._transition("INACTIVE", "Teach files loaded; explicit Startup required")
            self._log_configuration("configuration_loaded")
            response.success = True
            response.message = self.machine.message
            response.configuration_id = config.configuration_id
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.configuration_id = ""
            self.events.record("ERROR", "configuration_rejected", str(exc))
        finally:
            if acquired:
                self._end_operation()
        return response

    def _startup(self, _request, response):
        acquired = False
        try:
            if self.configuration is None or self.machine.state != "INACTIVE":
                raise CommandRejected("Startup requires configured INACTIVE state")
            self._begin_operation("startup")
            acquired = True
            self.configuration.validate_sources(self.root)
            self._transition("STARTING", "Explicit Startup accepted")
            self.hardware.startup()
            self.startup_complete = True
            self.global_speed_percent = 100
            self._transition("READY", "Startup completed; robot is READY")
            response.success = True
        except HeldUnknown as exc:
            self.startup_complete = False
            self._transition("HELD_UNKNOWN", str(exc))
            response.success = False
        except OperationCanceled as exc:
            response.success = False
            self._settle_lifecycle_cancellation(str(exc))
        except Exception as exc:
            response.success = False
            if acquired and self.machine.state != "STOPPING":
                self._transition("FAULT", f"Startup failed: {exc}")
                event, level = "startup_failed", "ERROR"
            else:
                event, level = "startup_rejected", "WARNING"
            self.events.record(level, event, str(exc))
        finally:
            if acquired:
                self._end_operation()
        response.message, response.state = self.machine.message, self.machine.state
        return response

    def _recover(self, _request, response):
        acquired = False
        try:
            if self.machine.state not in ("FAULT", "RECOVERY_REQUIRED"):
                raise CommandRejected("Recover requires FAULT or RECOVERY_REQUIRED state")
            self._begin_operation("recover")
            acquired = True
            if self.configuration is not None:
                self.configuration.validate_sources(self.root)
            self._transition("RECOVERING", "Explicit recovery accepted")
            self.hardware.recover(self.global_speed_percent)
            self.startup_complete = True
            if self.global_speed_percent is None:
                self.global_speed_percent = 100
            target = "HOLDING" if self.holding_item else "READY"
            self._transition(target, "Recovery completed; robot is " + target)
            response.success = True
        except HeldUnknown as exc:
            self.startup_complete = False
            self._transition("HELD_UNKNOWN", str(exc))
            response.success = False
        except OperationCanceled as exc:
            response.success = False
            self._settle_lifecycle_cancellation(str(exc))
        except Exception as exc:
            response.success = False
            if acquired and self.machine.state != "STOPPING":
                self._transition("FAULT", f"Recovery failed: {exc}")
                event, level = "recovery_failed", "ERROR"
            else:
                event, level = "recovery_rejected", "WARNING"
            self.events.record(level, event, str(exc))
        finally:
            if acquired:
                self._end_operation()
        response.message, response.state = self.machine.message, self.machine.state
        return response

    def _settle_lifecycle_cancellation(self, message):
        try:
            future = self._request_stop(message)
            self._confirm_shared_stop(future)
            self._finish_stop_state()
        except Exception as exc:
            if self.machine.state != "FAULT":
                self._transition("FAULT", f"Cancellation Stop unconfirmed: {exc}")

    def _set_global_speed(self, request, response):
        acquired = False
        try:
            if self.machine.state not in ("READY", "HOLDING"):
                raise CommandRejected("Global speed requires stationary READY or HOLDING")
            self._begin_operation("set_global_speed")
            acquired = True
            value = int(request.percent)
            self.hardware.set_global_speed(value)
            self.global_speed_percent = value
            response.success = True
            response.message = f"SpeedFactor confirmed at {value}%"
            response.confirmed_percent = value
            self.events.record("INFO", "global_speed", response.message)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.confirmed_percent = (
                self.global_speed_percent if self.global_speed_percent is not None else 0)
            if acquired and self.machine.state != "STOPPING":
                self._transition("FAULT", f"Global speed failed: {exc}")
            self.events.record("WARNING", "global_speed_rejected", str(exc))
        finally:
            if acquired:
                self._end_operation()
        return response

    # ---------- Stop and cancellation ----------

    def _request_stop(self, reason):
        self.cancel_event.set()
        with self.stop_guard:
            if self.state_before_stop is None:
                self.state_before_stop = self.machine.state
            if self.machine.state != "STOPPING":
                self._transition("STOPPING", reason)
            if self.stop_future is None:
                self.stop_future = self.hardware.request_stop(reason)
            return self.stop_future

    def _confirm_shared_stop(self, future):
        with self.stop_confirmation_guard:
            if self.stop_confirmed:
                return
            if self.stop_error is not None:
                raise self.stop_error
            try:
                self.hardware.confirm_stop(future)
                self.stop_confirmed = True
            except Exception as exc:
                self.stop_error = StopUnconfirmed(str(exc))
                raise self.stop_error

    def _finish_stop_state(self):
        previous = self.state_before_stop
        if previous == "UNCONFIGURED":
            target = "UNCONFIGURED"
        elif previous == "INACTIVE":
            target = "INACTIVE"
        elif previous == "HELD_UNKNOWN":
            try:
                suction = bool(self.monitor.snapshot(
                    require_enabled=False).feed["digital_input_bits"] & 1)
            except FeedbackFailure:
                suction = True
            target = "HELD_UNKNOWN" if suction else "RECOVERY_REQUIRED"
        else:
            target = "RECOVERY_REQUIRED"
        self.startup_complete = False
        message = ("Stop confirmed; explicit recovery is required"
                   if target == "RECOVERY_REQUIRED" else "Stop confirmed")
        self._transition(target, message)

    def _stop(self, _request, response):
        try:
            future = self._request_stop("Explicit Stop requested")
            self._confirm_shared_stop(future)
            self._finish_stop_state()
            response.success = True
        except Exception as exc:
            if self.machine.state != "FAULT":
                self._transition("FAULT", f"Stop unconfirmed: {exc}")
            response.success = False
        response.message, response.state = self.machine.message, self.machine.state
        return response

    def _cancel_goal(self, _goal_handle):
        try:
            self._request_stop("Native action cancellation requested")
        except Exception as exc:
            self.events.record("ERROR", "cancel_stop_dispatch_failed", str(exc))
        return CancelResponse.ACCEPT

    def on_late_motion_ack(self, future):
        def confirm():
            try:
                self.hardware.confirm_stop(future)
                if self.machine.state not in ("STOPPING", "RECOVERY_REQUIRED", "FAULT"):
                    self._transition(
                        "RECOVERY_REQUIRED",
                        "Late motion acknowledgement stopped; recovery required")
                self.events.record(
                    "WARNING", "late_ack_stop_confirmed",
                    "Second Stop after late motion acknowledgement confirmed")
            except Exception as exc:
                self._transition("FAULT", f"Late-ack Stop unconfirmed: {exc}")
        if self.late_stop_thread is None or not self.late_stop_thread.is_alive():
            self.late_stop_thread = threading.Thread(target=confirm, daemon=True)
            self.late_stop_thread.start()

    # ---------- Home/Pick actions ----------

    def _reserve_goal(self, action, requested_id):
        config = self.configuration
        allowed = ("READY", "HOLDING") if action == "home" else ("READY",)
        if (not self.startup_complete or config is None or self.machine.state not in allowed
                or requested_id != config.configuration_id):
            return GoalResponse.REJECT
        if action == "pick" and (config.selection is None or self.holding_item):
            return GoalResponse.REJECT
        try:
            self._begin_operation(action)
        except CommandRejected:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _home_goal(self, request):
        return self._reserve_goal("home", request.configuration_id)

    def _pick_goal(self, request):
        return self._reserve_goal("pick", request.configuration_id)

    def _preflight_item_state(self, expected_holding=None):
        snapshot = self.monitor.snapshot(require_enabled=True)
        suction = bool(snapshot.feed["digital_input_bits"] & 1)
        expected = self.holding_item if expected_holding is None else expected_holding
        if expected and not suction:
            raise HeldUnknown("Trusted held-item context lost DI1")
        if not expected and suction:
            raise HeldUnknown("DI1 active without trusted held-item context")

    def _home_plan(self, preceding=()):
        config = self.configuration
        config.validate_sources(self.root)
        current = preceding[-1].matrix if preceding else self.hardware.current_pose()
        return home_targets(
            current, config.home_matrix, config.home_joints,
            speed_percent=config.profile["speed"]["travel_percent"],
            acceleration_percent=config.profile["acceleration"]["travel_percent"])

    def _execute_home(self, *, preceding=(), require_suction=None, forbid_suction=None):
        self.raise_if_cancelled()
        holding = self.holding_item if require_suction is None else require_suction
        forbidden = not holding if forbid_suction is None else forbid_suction
        self._preflight_item_state(holding)
        targets = self._home_plan(preceding)
        self.operation_progress("HOME", "Executing shared Home function", waypoint="home")
        self.hardware.move_batch(
            (*preceding, *targets), require_suction=holding,
            forbid_suction=forbidden)
        return targets

    @staticmethod
    def _failure_outcome(result, exc):
        if isinstance(exc, (CommandRejected, CommandResponseTimeout)):
            return result.COMMAND_REJECTED
        if isinstance(exc, (FeedbackFailure, HeldUnknown)):
            return result.FEEDBACK_FAILURE
        return result.CONTROLLER_FAULT

    def _action_failure(self, goal, result, exc, outcome):
        message = str(exc)
        cancelled = isinstance(exc, OperationCanceled) or self.cancel_event.is_set()
        try:
            future = self._request_stop(
                "Action cancellation" if cancelled else "Non-suction action failure")
            self._confirm_shared_stop(future)
            self._finish_stop_state()
        except Exception as stop_exc:
            outcome = result.STOP_UNCONFIRMED
            message = f"{message}; Stop unconfirmed: {stop_exc}"
            if self.machine.state != "FAULT":
                self._transition("FAULT", message)
        result.outcome = (result.CANCELED
                          if cancelled and outcome != result.STOP_UNCONFIRMED else outcome)
        result.message = message
        result.final_state = self.machine.state
        if cancelled and goal.is_cancel_requested:
            goal.canceled()
        else:
            goal.abort()
        self.events.record(
            "ERROR", "action_result", message, operation=self.active_action,
            outcome=int(result.outcome), state=result.final_state)
        return result

    def _execute_home_action(self, goal):
        self.active_goal = goal
        result = GoHome.Result()
        try:
            self.configuration.validate_sources(self.root)
            self._transition("HOMING", "Home action started")
            self._execute_home()
            state = "HOLDING" if self.holding_item else "READY"
            self._transition(state, "Home completed at taught joints")
            result.outcome = result.SUCCESS
            result.message = self.machine.message
            result.final_state = state
            goal.succeed()
            self.events.record("INFO", "action_result", result.message,
                               operation="home", outcome=int(result.outcome), state=state)
            return result
        except Exception as exc:
            return self._action_failure(goal, result, exc, self._failure_outcome(result, exc))
        finally:
            self._end_operation()

    def _candidate_progress(self, phase, message, index):
        self.operation_progress(
            phase, message, candidate_index=index, candidate_total=self.candidate_total)

    def _execute_pick_action(self, goal):
        self.active_goal = goal
        result = PickItem.Result()
        result.attempted_candidates = 0
        result.selected_candidate_id = ""
        try:
            config = self.configuration
            config.validate_sources(self.root)
            self._transition("PICKING", "Pick action started")
            self._execute_home()
            self.operation_progress("DETECT", "Requesting one fresh candidate batch")
            batch = self.candidates.request(
                config, save_debug_images=goal.request.save_debug_images,
                cancel=self.cancel_requested)
            self.candidate_total = len(batch.candidates)
            self.operation_progress(
                "PLAN", f"Validated {self.candidate_total} fresh candidates",
                candidate_total=self.candidate_total)
            if not batch.candidates:
                self._transition("READY", "No valid pick candidates; robot remains Home")
                result.outcome = result.NO_PICK
                result.message = self.machine.message
                result.final_state = "READY"
                goal.succeed()
                return result
            plans = []
            for index, candidate in enumerate(batch.candidates, 1):
                xyz = config.selection.station.platform.base_from_platform @ np.array(
                    [*candidate.position_m, 1.0])
                plans.append(pick_targets(
                    config.home_matrix, xyz[:3], config.profile, index))
            oldest_stamp = min(batch.observation_stamp_ns, batch.depth_stamp_ns)

            def check(index):
                self.raise_if_cancelled()
                config.validate_sources(self.root)
                age = (self.get_clock().now().nanoseconds - oldest_stamp) / 1e9
                if not 0 <= age <= config.profile["quality"]["result_max_age_sec"]:
                    raise FeedbackFailure(
                        f"Candidate {index} expired; request a new Pick action")
                result.attempted_candidates = index

            outcome = PickExecutor(self.hardware, finish_home=True).run(
                plans, config.profile, check=check,
                return_home=lambda **kwargs: self._execute_home(**kwargs),
                progress=self._candidate_progress,
                holding_changed=lambda value: setattr(self, "holding_item", value))
            self.holding_item = outcome["holding_item"]
            if outcome["picked"]:
                candidate = batch.candidates[outcome["candidate"] - 1]
                result.selected_candidate_id = candidate.identifier
                self._transition("HOLDING", "Pick completed; item held at Home")
                result.outcome = result.SUCCESS
            else:
                self._transition("READY", "Candidate batch exhausted; no item picked")
                result.outcome = result.NO_PICK
            result.message = self.machine.message
            result.final_state = self.machine.state
            goal.succeed()
            self.events.record(
                "INFO", "action_result", result.message, operation="pick",
                outcome=int(result.outcome), attempted=result.attempted_candidates,
                selected_candidate_id=result.selected_candidate_id,
                state=result.final_state, debug_capture=batch.debug_message)
            return result
        except Exception as exc:
            return self._action_failure(goal, result, exc, self._failure_outcome(result, exc))
        finally:
            self._end_operation()

    # ---------- supervision/shutdown ----------

    def _stop_unexpected_idle_motion(self):
        """Pre-empt motion that was not started by the active executor."""
        reason = "Unexpected queued/running motion while controller idle"
        try:
            future = self._request_stop(reason)
        except Exception as exc:
            self._transition("FAULT", f"Unexpected-motion Stop dispatch failed: {exc}")
            return

        def confirm():
            try:
                self._confirm_shared_stop(future)
                self._finish_stop_state()
                self.events.record(
                    "ERROR", "unexpected_motion_stopped",
                    "Unexpected idle motion was stopped; explicit recovery required")
            except Exception as exc:
                if self.machine.state != "FAULT":
                    self._transition("FAULT", f"Unexpected-motion Stop unconfirmed: {exc}")

        if (self.supervision_stop_thread is None
                or not self.supervision_stop_thread.is_alive()):
            self.supervision_stop_thread = threading.Thread(target=confirm, daemon=True)
            self.supervision_stop_thread.start()

    def _supervise(self):
        if (not self.startup_complete or self.operation_lock.locked()
                or self.machine.state not in ("READY", "HOLDING")):
            return
        try:
            snapshot = self.monitor.snapshot(require_enabled=False)
            feed = snapshot.feed
            blockers = enabled_blockers(feed, snapshot.robot_enabled)
            transient = {"ErrorStatus", "CollisionStates", "isPauseCmdFlag"}
            blockers = [item for item in blockers
                        if not any(item.startswith(key + "=") for key in transient)]
            flags = self.monitor.consistent_flags(3)
            if flags is not None:
                pause, error, collision = flags
                if pause or error or collision:
                    blockers.append(
                        f"persistent flags pause={pause}, error={error}, collision={collision}")
            if blockers:
                raise FeedbackFailure("; ".join(blockers))
            if feed["isRunQueuedCmd"] or feed["RunningStatus"]:
                self._stop_unexpected_idle_motion()
                return
            suction = bool(feed["digital_input_bits"] & 1)
            if self.holding_item != suction:
                if suction:
                    self._transition("HELD_UNKNOWN", "DI1 active without trusted context")
                    return
                raise HeldUnknown("DI1 lost while controller expected a held item")
            for channel, expected in self.expected_outputs.items():
                actual = bool(feed["digital_outputs"] & (1 << (channel - 1)))
                if actual != expected:
                    raise FeedbackFailure(
                        f"Unexpected DO{channel}={int(actual)}; expected {int(expected)}")
        except Exception as exc:
            if self.machine.state in ("READY", "HOLDING"):
                self._transition("FAULT", f"Runtime supervision failed: {exc}")

    def shutdown_runtime(self):
        self.shutdown_event.set()
        if self.operation_lock.locked() or self.hardware.moving:
            try:
                future = self._request_stop("Controller shutdown during active operation")
                self._confirm_shared_stop(future)
            except Exception as exc:
                self.events.record("ERROR", "shutdown_stop_unconfirmed", str(exc))
        if self.operation_lock.locked():
            completed = self.operation_lock.acquire(timeout=5.0)
            if completed:
                self.operation_lock.release()
            else:
                self.events.record(
                    "ERROR", "shutdown_operation_unconfirmed",
                    "Active operation did not terminate before client teardown")
        if self.late_stop_thread is not None:
            self.late_stop_thread.join(timeout=2.0)
        if self.supervision_stop_thread is not None:
            self.supervision_stop_thread.join(timeout=2.0)
        self.candidates.close()
        self.hardware.close()


def main(args=None):
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError(
            "Source scripts/source_ros_workspace.bash; ROS_LOCALHOST_ONLY=1 required")
    stop = threading.Event()
    previous = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}
    for number in previous:
        signal.signal(number, lambda _number, _frame: stop.set())
    node = executor = spin_thread = None
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = RobotController()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        while rclpy.ok() and not stop.wait(0.1):
            pass
    finally:
        if node is not None:
            node.shutdown_runtime()
            node.events.record(
                "INFO", "node_stopped", "Controller stopped; gripper outputs preserved",
                state=node.machine.state, holding_item=node.holding_item)
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for number, handler in previous.items():
            signal.signal(number, handler)
