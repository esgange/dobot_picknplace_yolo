"""Direct, guarded Dobot access for attended camera-calibration maintenance."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import threading
import time
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from dobot_msgs_v4.msg import RobotStatus
from dobot_msgs_v4.srv import CP, MovJ, Stop
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from .calibration_core import JOINT_NAMES, rotation_angle_deg, workspace_root


FEED_TOPIC = "/dobot_bringup_ros2/msg/FeedInfo"
STATUS_TOPIC = "/dobot_msgs_v4/msg/RobotStatus"
SERVICE_PREFIX = "/dobot_bringup_ros2/srv/"
RESPONSE_TIMEOUT = 5.0
FEEDBACK_MAX_AGE = 1.0
MOTION_TIMEOUT = 300.0
MOTION_PROGRESS_TIMEOUT = 3.0
RATE_PERCENT = 20


class ReplayStopped(RuntimeError):
    pass


class MotionArrivalTimeout(RuntimeError):
    """A replied-to move needs confirmed Stop before its target may be retried."""


def rpy_rotation(angles):
    roll, pitch, yaw = angles
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


class Cr10Model:
    """Read the canonical CR10 chain; no dependency on perception or controller."""

    def __init__(self, path):
        robot = ElementTree.parse(path).getroot()
        if robot.attrib.get("name") != "cr10_robot":
            raise ValueError("Calibration replay requires the canonical CR10 model")
        self.chain = []
        parent = "base_link"
        for index, name in enumerate(JOINT_NAMES, 1):
            matches = robot.findall(f"joint[@name='{name}']")
            if len(matches) != 1:
                raise ValueError(f"CR10 model requires exactly one {name}")
            joint = matches[0]
            if (joint.attrib["type"] != "revolute"
                    or joint.find("parent").attrib["link"] != parent
                    or joint.find("child").attrib["link"] != f"Link{index}"
                    or list(map(float, joint.find("axis").attrib["xyz"].split())) != [0, 0, 1]):
                raise ValueError("Invalid canonical CR10 joint chain")
            origin = joint.find("origin").attrib
            matrix = np.eye(4)
            matrix[:3, 3] = list(map(float, origin["xyz"].split()))
            matrix[:3, :3] = rpy_rotation(map(float, origin["rpy"].split()))
            limits = joint.find("limit").attrib
            self.chain.append((matrix, float(limits["lower"]), float(limits["upper"])))
            parent = f"Link{index}"

    def forward(self, joints):
        if len(joints) != 6:
            raise ValueError("Calibration position requires exactly six joints")
        matrix = np.eye(4)
        for angle, (origin, lower, upper) in zip(joints, self.chain):
            if not math.isfinite(angle) or not lower <= angle <= upper:
                raise ValueError("Saved joints are outside canonical CR10 limits")
            rotation = np.eye(4)
            rotation[:3, :3] = rpy_rotation((0., 0., angle))
            matrix = matrix @ origin @ rotation
        return matrix


@dataclass(frozen=True)
class RobotSample:
    sequence: int
    feed: dict
    status_enabled: bool

    @property
    def joints(self):
        return np.deg2rad(self.feed["q_actual"])

    @property
    def tcp(self):
        return np.asarray(self.feed["tool_vector_actual"])


def stationary(previous, current):
    return bool(previous is not None and current.sequence > previous.sequence
                and current.feed["controller_timer"] > previous.feed["controller_timer"]
                and np.max(np.abs(current.tcp - previous.tcp)) <= 0.05
                and np.max(np.abs(current.joints - previous.joints)) <= math.radians(.05))


def idle(sample):
    feed = sample.feed
    return (sample.status_enabled and feed["robot_mode"] == 5
            and not feed["RunningStatus"] and not feed["isRunQueuedCmd"])


def arrived(sample, joints, matrix):
    rotation = rpy_rotation(np.deg2rad(sample.tcp[3:]))
    return (np.max(np.abs(sample.joints - joints)) <= math.radians(1.)
            and np.linalg.norm(sample.tcp[:3] / 1000. - matrix[:3, 3]) <= .005
            and rotation_angle_deg(rotation @ matrix[:3, :3].T) <= 1.)


def configured_bringup(root):
    values = {}
    for number, raw in enumerate((root / ".env").read_text().splitlines(), 1):
        if not raw or raw.startswith("#"):
            continue
        key, separator, value = raw.partition("=")
        if (not separator or not re.fullmatch(r"[A-Z][A-Z0-9_]*", key)
                or value != value.strip() or key in values):
            raise ValueError(f"Invalid canonical .env entry at line {number}")
        values[key] = value
    name = values.get("DOBOT_ROBOT_NODE_NAME", "")
    if (values.get("ROS_LOCALHOST_ONLY") != "1" or values.get("DOBOT_ROBOT_TYPE") != "cr10"
            or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name)):
        raise ValueError("Canonical .env requires local-only CR10 and DOBOT_ROBOT_NODE_NAME")
    return name


class MaintenanceRobot:
    """Responses/feedback use a reentrant group independent of serial RGB detection."""

    def __init__(self, node):
        self.node = node
        self.group = ReentrantCallbackGroup()
        self.condition = threading.Condition(threading.RLock())
        self.cancel = threading.Event()
        self.operator_cancel = threading.Event()
        self.feed = self.status = None
        self.sequence = 0
        self.progress_time = None
        self.bringup = None
        self.pending = []
        self.stop_attempt = None
        self.output_bits = None
        self.watching = False
        self.feedback_failure = None
        self.last_owner_check = 0.
        self.clients = {kind.__name__: node.create_client(
            kind, SERVICE_PREFIX + kind.__name__, callback_group=self.group)
            for kind in (CP, MovJ, Stop)}
        node.create_subscription(String, FEED_TOPIC, self._on_feed, 10,
                                 callback_group=self.group)
        node.create_subscription(RobotStatus, STATUS_TOPIC, self._on_status, 10,
                                 callback_group=self.group)
        self.stop_timer = node.create_timer(.02, self._check_stop, callback_group=self.group)

    def _on_status(self, message):
        with self.condition:
            self.status = message.is_connected, message.is_enable, time.monotonic()
            if self.watching and not message.is_connected:
                self.feedback_failure = "Dobot disconnected during automatic capture"
            self.condition.notify_all()

    def _on_feed(self, message):
        try:
            data = json.loads(message.data)
            for key in ("robot_mode", "controller_timer", "currentCommandId", "RunningStatus",
                        "isRunQueuedCmd", "EnableStatus", "ErrorStatus", "CollisionStates",
                        "userCoordinate", "toolCoordinate", "digital_input_bits",
                        "digital_outputs"):
                if type(data[key]) is not int or data[key] < 0:
                    raise ValueError(f"FeedInfo {key} must be a nonnegative integer")
            for key in ("q_actual", "tool_vector_actual"):
                if (len(data[key]) != 6 or any(type(v) not in (int, float) or not math.isfinite(v)
                                               for v in data[key])):
                    raise ValueError(f"FeedInfo {key} must contain six finite numbers")
        except (KeyError, TypeError, ValueError) as exc:
            with self.condition:
                self.feed = None
                if self.watching:
                    self.feedback_failure = f"Invalid FeedInfo during automatic capture: {exc}"
            self.node._event_logger.record("WARNING", "replay_feedback_invalid", str(exc))
            return
        now = time.monotonic()
        with self.condition:
            if self.watching:
                if (self.feed is not None
                        and data["controller_timer"] < self.feed[0]["controller_timer"]):
                    self.feedback_failure = "Robot controller timer restarted during capture"
                if (data["EnableStatus"] != 1 or data["robot_mode"] not in (5, 7, 8)
                        or data["ErrorStatus"] or data["CollisionStates"]
                        or data["userCoordinate"] != 0 or data["toolCoordinate"] != 0
                        or data["digital_input_bits"] & 1
                        or data["digital_outputs"] != self.output_bits):
                    self.feedback_failure = "Robot state or I/O changed during automatic capture"
            if self.feed is None or data["controller_timer"] != self.feed[0]["controller_timer"]:
                self.progress_time = now
            self.feed = data, now
            self.sequence += 1
            self.condition.notify_all()

    def snapshot(self, enabled=True):
        with self.condition:
            now = time.monotonic()
            if (self.feed is None or self.status is None or not self.status[0]
                    or now - self.feed[1] > FEEDBACK_MAX_AGE
                    or now - self.status[2] > FEEDBACK_MAX_AGE
                    or self.progress_time is None or now - self.progress_time > FEEDBACK_MAX_AGE):
                raise RuntimeError("Canonical robot feedback is missing, stale or not advancing")
            sample = RobotSample(self.sequence, self.feed[0], self.status[1])
        feed = sample.feed
        if enabled and (feed["EnableStatus"] != 1 or feed["robot_mode"] not in (5, 7, 8)
                        or feed["ErrorStatus"] or feed["CollisionStates"]
                        or feed["userCoordinate"] != 0 or feed["toolCoordinate"] != 0):
            raise RuntimeError("Robot must be enabled, fault-free, with user/tool zero")
        return sample

    def check_owners(self):
        nodes = self.node.get_node_names_and_namespaces()
        if nodes.count((self.node.get_name(), self.node.get_namespace())) != 1:
            raise RuntimeError("Duplicate camera_calibration node")
        competing = {"robot_controller", "motion_debug", "motion_debug_gui",
                     "gripper_control", "gripper_control_gui"}
        running = sorted(name for name, _ in nodes if name in competing)
        if running:
            raise RuntimeError("Close competing command applications: " + ", ".join(running))
        for topic in (FEED_TOPIC, STATUS_TOPIC, "/joint_states"):
            publishers = self.node.get_publishers_info_by_topic(topic)
            if (len(publishers) != 1 or publishers[0].node_name != self.bringup
                    or publishers[0].node_namespace != "/"):
                raise RuntimeError(f"{topic} requires sole canonical publisher /{self.bringup}")
        self.last_owner_check = time.monotonic()

    def _service_owner(self, name):
        providers = []
        for node, namespace in self.node.get_node_names_and_namespaces():
            names = self.node.get_service_names_and_types_by_node(node, namespace)
            if SERVICE_PREFIX + name in (service for service, _types in names):
                providers.append((node, namespace))
        if providers != [(self.bringup, "/")] or not self.clients[name].service_is_ready():
            raise RuntimeError(f"Canonical Dobot {name} service unavailable or ambiguous")

    def prepare(self, samples):
        self.bringup = configured_bringup(workspace_root())
        self.check_owners()
        for name in self.clients:
            self._service_owner(name)
        model = Cr10Model(Path(get_package_share_directory("cra_description"))
                          / "urdf/cr10_robot.xacro")
        if not 5 <= len(samples) <= 1000:
            raise ValueError("Automatic capture requires 5 through 1000 saved positions")
        targets = [(tuple(s.joint_positions_rad), model.forward(s.joint_positions_rad))
                   for s in samples]
        with self.condition:
            if any(not record["future"].done() for record in self.pending):
                raise RuntimeError("A previous Dobot response is still unanswered")
            if self.stop_attempt is not None and not self.stop_attempt.get("confirmed"):
                raise RuntimeError("Stop is unconfirmed; use Stop again before restarting")
            sample = self.snapshot()
            if not idle(sample) or sample.feed["digital_input_bits"] & 1:
                raise RuntimeError("Automatic capture requires an enabled idle robot and DI1 LOW")
            self.output_bits = sample.feed["digital_outputs"]
            for first, second in ((1, 13), (2, 14)):
                if (self.output_bits & (1 << (first - 1))
                        and self.output_bits & (1 << (second - 1))):
                    raise RuntimeError("Opposing gripper outputs must not both be HIGH")
            self.cancel.clear()
            self.operator_cancel.clear()
            self.pending.clear()
            self.stop_attempt = None
            self.feedback_failure = None
            self.watching = True
        return targets

    def guard(self):
        if self.cancel.is_set() or self.operator_cancel.is_set():
            raise ReplayStopped("Automatic capture stopped")
        return self._state_guard()

    def _state_guard(self):
        if self.feedback_failure is not None:
            raise RuntimeError(self.feedback_failure)
        if self.node._fatal_error is not None:
            raise RuntimeError(self.node._fatal_error)
        if time.monotonic() - self.last_owner_check >= .1:
            self.check_owners()
        sample = self.snapshot()
        if sample.feed["digital_input_bits"] & 1:
            raise RuntimeError("DI1 became HIGH during automatic capture")
        if sample.feed["digital_outputs"] != self.output_bits:
            raise RuntimeError("Digital outputs changed during automatic capture")
        return sample

    def stopped_retry_snapshot(self, attempt):
        with self.condition:
            if self.operator_cancel.is_set():
                raise ReplayStopped("Operator stopped automatic capture")
            if (self.stop_attempt is not attempt or not attempt.get("confirmed")
                    or any(not record["future"].done() for record in self.pending)):
                raise RuntimeError(
                    "Retry requires the same confirmed Stop and all command replies")
            sample = self._state_guard()
            if not idle(sample):
                raise RuntimeError("Retry requires a fresh enabled idle robot")
            return sample

    def hold_stopped(self, anchor, attempt):
        sample = self.stopped_retry_snapshot(attempt)
        if (np.max(np.abs(sample.tcp - anchor.tcp)) > .05
                or np.max(np.abs(sample.joints - anchor.joints)) > math.radians(.05)):
            raise RuntimeError("Robot moved while waiting for Continue")

    def resume_after_stop(self, attempt):
        with self.condition:
            self.stopped_retry_snapshot(attempt)
            self.cancel.clear()
            self.stop_attempt = None

    def end(self):
        with self.condition:
            self.watching = False

    def _late_reply(self, record):
        with self.condition:
            if not record.get("abandoned") or record.get("contained"):
                return
            record["contained"] = True
            try:
                result = record["future"].result()
                if record["name"] == "MovJ" and result is not None and result.res == 0:
                    self.node._event_logger.record(
                        "WARNING", "replay_late_motion_acceptance",
                        "Stopping late MovJ acceptance")
                    self.request_stop(fresh=True)
            except Exception as exc:
                self.node._event_logger.record("ERROR", "replay_late_reply_failed", str(exc))

    def command(self, name, request):
        self._service_owner(name)
        with self.condition:
            self.guard()
            if any(not record["future"].done() for record in self.pending):
                raise RuntimeError("Previous Dobot response remains unanswered")
            future = self.clients[name].call_async(request)
            record = {"future": future, "name": name}
            self.pending.append(record)
            future.add_done_callback(lambda _done: self._late_reply(record))
        self.node._event_logger.record("INFO", "replay_command_sent", name)
        deadline = time.monotonic() + RESPONSE_TIMEOUT
        try:
            while not future.done():
                self.guard()
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"{name} response timed out after five seconds")
                self.cancel.wait(.01)
            self.guard()
            result = future.result()
            if result is None or result.res != 0:
                raise RuntimeError(f"{name} rejected: {None if result is None else result.res}")
            self.node._event_logger.record("INFO", "replay_command_accepted", name)
            return result
        except Exception:
            with self.condition:
                record["abandoned"] = True
                if future.done():
                    self._late_reply(record)
            raise

    def move(self, joints, matrix, *, monitor):
        initial = self.guard()
        if not idle(initial):
            raise RuntimeError("Robot must be idle before the next saved position")
        fields = dict(zip("abcdef", map(float, np.rad2deg(joints))))
        result = self.command("MovJ", MovJ.Request(
            mode=True, param_value=["user=0", "tool=0", f"v={RATE_PERCENT}",
                                    f"a={RATE_PERCENT}"], **fields))
        match = re.fullmatch(r"\{\s*([0-9]+)\s*\}", result.robot_return.strip())
        if match is None or int(match[1]) > 2**64 - 1:
            raise RuntimeError("MovJ acceptance lacks a valid queue ID")
        command_id = int(match[1])
        accepted = self.guard()
        previous = None
        deadline = time.monotonic() + MOTION_TIMEOUT
        progress_at, progress_tcp = time.monotonic(), initial.tcp
        progress_joints = initial.joints
        while True:
            sample = self.guard()
            monitor()
            now = time.monotonic()
            if (np.max(np.abs(sample.tcp - progress_tcp)) > .05
                    or np.max(np.abs(sample.joints - progress_joints)) > math.radians(.05)):
                progress_at, progress_tcp = now, sample.tcp
                progress_joints = sample.joints
            endpoint = (sample.sequence > accepted.sequence
                        and sample.feed["controller_timer"] > accepted.feed["controller_timer"]
                        and sample.feed["currentCommandId"] == command_id
                        and idle(sample) and arrived(sample, joints, matrix))
            if endpoint and stationary(previous, sample):
                return sample
            previous = sample if endpoint else None
            if now >= deadline or now - progress_at >= MOTION_PROGRESS_TIMEOUT:
                joint_error = float(np.max(np.abs(np.rad2deg(sample.joints - joints))))
                position_error = float(np.linalg.norm(sample.tcp[:3] - matrix[:3, 3] * 1000.))
                angle_error = rotation_angle_deg(
                    rpy_rotation(np.deg2rad(sample.tcp[3:])) @ matrix[:3, :3].T)
                reason = ("motion deadline expired" if now >= deadline
                          else "no motion progress for 3 seconds")
                message = (
                    f"Robot arrival not confirmed: {reason}; "
                    f"queue ID {sample.feed['currentCommandId']}/{command_id}, "
                    f"idle={bool(idle(sample))}, joint error={joint_error:.3f} deg, "
                    f"TCP error={position_error:.3f} mm/{angle_error:.3f} deg")
                self.node._event_logger.record("WARNING", "replay_arrival_timeout", message)
                raise MotionArrivalTimeout(message)
            self.cancel.wait(.01)

    def hold(self, anchor, joints, matrix):
        sample = self.guard()
        if (not idle(sample) or not arrived(sample, joints, matrix)
                or np.max(np.abs(sample.tcp - anchor.tcp)) > .05
                or np.max(np.abs(sample.joints - anchor.joints)) > math.radians(.05)):
            raise RuntimeError("Robot moved while collecting the calibration sample")
        return sample

    def request_stop(self, *, fresh=False):
        self.cancel.set()
        with self.condition:
            if self.stop_attempt is not None and not fresh:
                return
            attempt = {"deadline": time.monotonic() + RESPONSE_TIMEOUT}
            self.stop_attempt = attempt
            try:
                self._service_owner("Stop")
                attempt["future"] = self.clients["Stop"].call_async(Stop.Request())
                self.node._event_logger.record(
                    "WARNING", "replay_stop_requested", "Direct Dobot Stop")
            except Exception as exc:
                attempt["error"] = str(exc)
            self.condition.notify_all()

    def _check_stop(self):
        with self.condition:
            attempt = self.stop_attempt
            if attempt is None or attempt.get("confirmed") or attempt.get("error"):
                return
            try:
                now = time.monotonic()
                if now > attempt["deadline"]:
                    raise RuntimeError("Stop acknowledgement or physical confirmation timed out")
                future = attempt["future"]
                if not future.done():
                    return
                if "accepted_sequence" not in attempt:
                    result = future.result()
                    if result is None or result.res != 0:
                        raise RuntimeError("Dobot Stop was rejected")
                    attempt.update(accepted_sequence=self.sequence, deadline=now + 2.)
                try:
                    sample = self.snapshot(enabled=False)
                except RuntimeError:
                    return
                empty = (not sample.feed["RunningStatus"] and not sample.feed["isRunQueuedCmd"]
                         and sample.feed["robot_mode"] in (4, 5, 9, 10)
                         and sample.sequence > attempt["accepted_sequence"])
                if empty and stationary(attempt.get("anchor"), sample):
                    attempt["confirmed"] = True
                    self.node._event_logger.record(
                        "INFO", "replay_stop_confirmed",
                        "Stationary robot and empty queue confirmed")
                attempt["anchor"] = sample if empty else None
            except Exception as exc:
                attempt["error"] = str(exc)
                self.node._event_logger.record("ERROR", "replay_stop_unconfirmed", str(exc))
            self.condition.notify_all()

    def wait_stop(self):
        deadline = time.monotonic() + RESPONSE_TIMEOUT + 3.
        with self.condition:
            while True:
                attempt = self.stop_attempt
                if attempt is None:
                    raise RuntimeError("No Stop attempt exists")
                if attempt.get("error"):
                    raise RuntimeError(attempt["error"])
                if attempt.get("confirmed"):
                    return
                if time.monotonic() >= deadline:
                    raise RuntimeError("Stop confirmation unavailable")
                self.condition.wait(.02)
