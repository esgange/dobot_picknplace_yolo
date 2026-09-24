"""Exclusive, attended calibration replay using the controller's guarded transport."""

import re
import time

import numpy as np
from rclpy.action import ActionServer, GoalResponse
from robot_controller_interfaces.action import ReplayCalibration
from robot_controller_interfaces.srv import CaptureCalibration

from .errors import CommandRejected, FeedbackFailure
from .feedback import enabled_blockers
from .motion import Target


CAPTURE_SERVICE = "/camera_calibration/capture_automatic"
REPLAY_ACTION = "/robot_controller/replay_calibration"
REPLAY_STATES = ("UNCONFIGURED", "INACTIVE", "READY")
REPLAY_RATE_PERCENT = 20


def replay_targets(request, kinematics):
    if not re.fullmatch(r"[0-9a-f]{32}", request.run_id):
        raise CommandRejected("Calibration run ID must be a fresh UUID hex token")
    if not 5 <= len(request.positions) <= 1000:
        raise CommandRejected("Calibration replay requires 5 through 1000 positions")
    targets = []
    for index, position in enumerate(request.positions, 1):
        joints = tuple(float(value) for value in position.joints_rad)
        if len(joints) != 6:
            raise CommandRejected("Each calibration position requires six joints")
        matrix = kinematics.forward(joints)  # Checks finite values and CR10 joint limits.
        targets.append(Target(f"calibration_{index}", matrix, REPLAY_RATE_PERCENT,
                              REPLAY_RATE_PERCENT, joints_rad=joints, joint_motion=True))
    return tuple(targets)


class StationaryGate:
    """Two distinct advancing TCP/joint observations, with no timed dwell."""

    def __init__(self):
        self.previous = None

    def update(self, sample):
        previous, self.previous = self.previous, sample
        return bool(previous is not None and sample.sequence > previous.sequence
                    and sample.feed["controller_timer"] != previous.feed["controller_timer"]
                    and np.max(np.abs(np.asarray(sample.feed["tool_vector_actual"])
                                      - previous.feed["tool_vector_actual"])) <= 0.05
                    and np.max(np.abs(np.asarray(sample.joints)
                                      - previous.joints)) <= np.deg2rad(0.05))


class CalibrationReplay:
    def __init__(self, node):
        self.node = node
        self.capture_client = node.create_client(CaptureCalibration, CAPTURE_SERVICE)
        self.server = ActionServer(
            node, ReplayCalibration, REPLAY_ACTION, execute_callback=self.execute,
            goal_callback=self.reserve, cancel_callback=node._cancel_goal,
            callback_group=node.control_group)

    def _admission(self):
        node = self.node
        if (node.headless or node.machine.state not in REPLAY_STATES
                or node.holding_item or node.managed.session is not None):
            raise CommandRejected(
                "Replay requires attended UNCONFIGURED/INACTIVE/READY without item context")
        node.check_feedback_owners()
        node.check_all_command_owners(("MovJ", "CP", "Stop"))
        if (node._service_providers(CAPTURE_SERVICE) != [("camera_calibration", "/")]
                or not self.capture_client.service_is_ready()):
            raise CommandRejected("Exactly one camera_calibration capture provider is required")
        for name in ("MovJ", "CP"):
            if not node.hardware.clients[name].service_is_ready():
                raise CommandRejected(f"Canonical {name} service unavailable")
        node.hardware.ensure_no_pending_response()
        snapshot = node.monitor.snapshot(require_enabled=True)
        if not node.hardware._idle(snapshot) or snapshot.feed["digital_input_bits"] & 1:
            raise CommandRejected("Replay requires enabled idle user/tool 0 and DI1 LOW")
        return snapshot

    def reserve(self, request):
        acquired = False
        try:
            replay_targets(request, self.node.kinematics)
            self._admission()
            self.node._begin_operation("calibration")
            acquired = True
            self._admission()
            return GoalResponse.ACCEPT
        except Exception as exc:
            self.node.events.record("WARNING", "calibration_replay_rejected", str(exc))
            if acquired:
                self.node._end_operation()
            return GoalResponse.REJECT

    def _guard(self, sample):
        self.node.raise_if_cancelled()
        blockers = enabled_blockers(sample.feed, sample.robot_enabled)
        if blockers:
            raise FeedbackFailure("Calibration robot feedback: " + "; ".join(blockers))
        if not self.capture_client.service_is_ready():
            raise FeedbackFailure("Calibration capture provider disappeared")
        if sample.feed["digital_input_bits"] & 1:
            raise FeedbackFailure("DI1 became HIGH during calibration replay")
        if sample.feed["digital_outputs"] != self.outputs:
            raise FeedbackFailure("Digital outputs changed during calibration replay")
        now = time.monotonic()
        if now - self.last_heartbeat >= 0.5:
            self.last_heartbeat = now
            self.node.active_goal.publish_feedback(ReplayCalibration.Feedback(
                position_index=self.node.candidate_index,
                position_total=self.node.candidate_total,
                phase=self.node.phase, message=self.node.machine.message))

    def _capture(self, request, target=None):
        node = self.node
        future = self.capture_client.call_async(request)
        deadline = time.monotonic() + (20. if target is not None else 5.)
        try:
            while True:
                sample = node.monitor.snapshot(require_enabled=True)
                self._guard(sample)
                if not node.hardware._idle(sample):
                    raise FeedbackFailure("Robot left idle while preparing/capturing calibration")
                if target is not None:
                    if (not node.hardware._target_reached(target, sample)
                            or np.max(np.abs(np.asarray(sample.feed["tool_vector_actual"])
                                             - self.capture_anchor)) > 0.05
                            or np.max(np.abs(np.asarray(sample.joints)
                                             - self.capture_joints)) > np.deg2rad(0.05)):
                        raise FeedbackFailure("Robot moved while capturing calibration")
                if future.done():
                    result = future.result()
                    if result is None or not result.success:
                        raise FeedbackFailure(
                            "Calibration capture failed: "
                            + (result.message if result else "empty reply"))
                    return
                if time.monotonic() >= deadline:
                    raise FeedbackFailure("Calibration capture service timed out")
                node.wait_control(0.02)
        finally:
            if not future.done():
                self.capture_client.remove_pending_request(future)

    def execute(self, goal):
        node, result = self.node, ReplayCalibration.Result()
        node.active_goal = goal
        previous = node.machine.state
        old_outputs = dict(node.expected_outputs)
        try:
            node.raise_if_cancelled()
            sample = self._admission()
            targets = replay_targets(goal.request, node.kinematics)
            self.outputs = sample.feed["digital_outputs"]
            self.last_heartbeat = 0.
            node.expected_outputs = {
                channel: bool(self.outputs & (1 << (channel - 1)))
                for channel in (1, 2, 13, 14)}
            node.candidate_total = len(targets)
            node._transition("CALIBRATING", "Explicit calibration replay started")
            for index, target in enumerate(targets, 1):
                node.raise_if_cancelled()
                node.check_feedback_owners()
                if node._service_providers(CAPTURE_SERVICE) != [("camera_calibration", "/")]:
                    raise FeedbackFailure("Calibration receiver ownership changed")
                node.operation_progress("PREPARING", "Checking calibration receiver",
                                        candidate_index=index)
                self._capture(CaptureCalibration.Request(
                    phase=CaptureCalibration.Request.PREPARE, run_id=goal.request.run_id,
                    position_index=index))
                if index == 1:
                    node.hardware.call("CP", r=100, progress=self._guard)
                node.operation_progress("MOVING", "Moving to saved calibration position",
                                        waypoint=target.name)
                node.hardware.move_batch(
                    (target,), batch_name=target.name, forbid_suction=True,
                    preserve_outputs=True, guard=self._guard)
                gate = StationaryGate()

                def stationary(observation):
                    self._guard(observation)
                    if (not node.hardware._idle(observation)
                            or not node.hardware._target_reached(target, observation)):
                        gate.previous = None
                        return False
                    return gate.update(observation)

                sample = node.monitor.wait(
                    stationary, 2., cancel=node.cancel_requested, require_enabled=True,
                    description="advancing stationary calibration endpoint")
                self.capture_anchor = np.asarray(sample.feed["tool_vector_actual"])
                self.capture_joints = np.asarray(sample.joints)
                node.operation_progress("CAPTURING", "Waiting for a fresh ChArUco sample",
                                        waypoint=target.name)
                self._capture(CaptureCalibration.Request(
                    phase=CaptureCalibration.Request.CAPTURE, run_id=goal.request.run_id,
                    position_index=index, not_before=node.get_clock().now().to_msg()), target)
                result.captured_count = index
                node.operation_progress("CAPTURED", "Fresh sample accepted", waypoint=target.name)
            with node.stop_guard:
                node.raise_if_cancelled()
                node._transition(previous, "Calibration captured; robot remains at final position")
                result.outcome, result.message = result.SUCCESS, node.machine.message
                result.final_state = node.machine.state
                goal.succeed()
            return result
        except Exception as exc:
            return node._action_failure(goal, result, exc, node._failure_outcome(result, exc))
        finally:
            node.expected_outputs = old_outputs
            node._end_operation()
