"""Calibration admission, sequence and physical-evidence checks without hardware."""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock
import threading

import numpy as np
import pytest
from rclpy.action import GoalResponse
from rclpy.time import Time
from robot_controller_interfaces.action import ReplayCalibration
from robot_controller_interfaces.msg import CalibrationPosition
from robot_controller_interfaces.srv import CaptureCalibration

from robot_controller.calibration_replay import CalibrationReplay, StationaryGate, replay_targets
from robot_controller.errors import CommandRejected, FeedbackFailure, OperationCanceled
from robot_controller.hardware import DobotTransport
from robot_controller.kinematics import Cr10Kinematics
from robot_controller.motion import Target
from robot_controller.state_machine import ControllerStateMachine
from test_feedback_v2 import feed, primed_monitor
from test_motion_completion import MotionRig


def request():
    return ReplayCalibration.Goal(
        run_id="a" * 32,
        positions=[CalibrationPosition(joints_rad=[i * .05] * 6) for i in range(5)])


def test_saved_positions_keep_order_and_cr10_limits_are_checked():
    root = Path(__file__).resolve().parents[3]
    kinematics = Cr10Kinematics(
        root / "src/DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr10_robot.xacro")
    goal = request()
    targets = replay_targets(goal, kinematics)
    assert [t.joints_rad for t in targets] == [tuple(p.joints_rad) for p in goal.positions]
    assert all(t.joint_motion and not t.motion_io and t.speed_percent == 20
               and t.acceleration_percent == 20 for t in targets)
    for invalid in (float("nan"), float("inf"), 100.):
        goal.positions[0].joints_rad[0] = invalid
        with pytest.raises(ValueError, match="limits"):
            replay_targets(goal, kinematics)
    goal = request()
    goal.positions.pop()
    with pytest.raises(CommandRejected, match="5 through"):
        replay_targets(goal, kinematics)


def test_stationarity_needs_new_timer_tcp_and_joint_evidence_without_dwell():
    monitor = primed_monitor()
    gate = StationaryGate()
    first = monitor.snapshot()
    assert not gate.update(first)
    assert not gate.update(first)
    assert not gate.update(replace(first, sequence=2))
    second = replace(first, sequence=3, feed=feed(controller_timer=2))
    assert gate.update(second)
    assert not gate.update(replace(second, sequence=4, feed=feed(
        controller_timer=3, tool_vector_actual=[1., 0., 0., 0., 0., 0.])))
    assert not gate.update(replace(second, sequence=5, joints=(.1,) * 6,
                                   feed=feed(controller_timer=4)))


def test_joint_replay_also_requires_tcp_arrival():
    transport = object.__new__(DobotTransport)
    target = Target("replay", np.eye(4), 20, 20, joints_rad=(0.,) * 6, joint_motion=True)
    sample = primed_monitor().snapshot()
    assert transport._target_reached(target, sample)
    assert not transport._target_reached(target, replace(sample, feed=feed(
        tool_vector_actual=[20., 0., 0., 0., 0., 0.])))
    assert not transport._target_reached(target, replace(sample, joints=(.1,) * 6))


def test_movj_binds_returned_command_id_and_has_no_io():
    rig = MotionRig()
    rig.transport.node.kinematics = SimpleNamespace(forward=lambda _j: np.eye(4))

    def dispatch(calls, **_kwargs):
        rig.calls.extend(calls)
        rig.emit()
        return (SimpleNamespace(res=0, robot_return="{7}"),)

    rig.transport.call_group = dispatch
    rig.steps = iter([{}, {"command_id": 7}])
    rig.run(targets=(Target("replay", np.eye(4), 20, 20,
                            joints_rad=(0.,) * 6, joint_motion=True),))
    assert len(rig.waited) == 2
    name, fields = rig.calls[0]
    assert name == "MovJ" and fields["mode"] is True and "mdis" not in fields
    assert fields["param_value"] == ["user=0", "tool=0", "v=20", "a=20"]


class ReplayRig:
    def __init__(self):
        self.engine = object.__new__(CalibrationReplay)
        self.monitor = primed_monitor()
        self.history = []
        self.fail_capture = None
        self.cancel = False
        self.node = SimpleNamespace(
            headless=False, machine=ControllerStateMachine(), holding_item=False,
            managed=SimpleNamespace(session=None), expected_outputs={},
            monitor=self.monitor, events=MagicMock(), stop_guard=threading.RLock(),
            kinematics=SimpleNamespace(forward=lambda _j: np.eye(4)),
            check_feedback_owners=MagicMock(), check_all_command_owners=MagicMock(),
            _service_providers=lambda _name: [("camera_calibration", "/")],
            _begin_operation=MagicMock(), _end_operation=MagicMock(),
            raise_if_cancelled=self.check_cancel, cancel_requested=lambda: self.cancel,
            get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=10.)),
            _failure_outcome=lambda _r, _e: ReplayCalibration.Result.FEEDBACK_FAILURE,
            _action_failure=self.failure, operation_progress=self.progress,
            candidate_index=0, candidate_total=0, phase="")
        self.node._transition = self.node.machine.transition
        self.hardware = SimpleNamespace(
            _idle=DobotTransport._idle, clients={"MovJ": MagicMock(), "CP": MagicMock()},
            ensure_no_pending_response=MagicMock(), call=self.command,
            move_batch=self.move, _target_reached=lambda _t, _s: True)
        self.node.hardware = self.hardware
        self.engine.node = self.node
        self.engine.capture_client = SimpleNamespace(
            service_is_ready=lambda: True, call_async=self.capture)
        self.monitor.wait = self.wait_stationary
        self.goal = SimpleNamespace(request=request(), succeed=MagicMock(), abort=MagicMock(),
                                    publish_feedback=MagicMock())

    def check_cancel(self):
        if self.cancel:
            raise OperationCanceled("Stop requested")

    def progress(self, phase, _message, **fields):
        self.node.phase = phase
        self.node.candidate_index = fields.get("candidate_index", self.node.candidate_index)

    def command(self, name, **_fields):
        self.history.append(name)

    def move(self, targets, **options):
        options["guard"](self.monitor.snapshot())
        self.history.append(("move", targets[0].name))

    def wait_stationary(self, predicate, *_args, **_kwargs):
        assert not predicate(self.monitor.snapshot())
        self.monitor.update_feed(feed(controller_timer=self.monitor.sequence + 1))
        sample = self.monitor.snapshot()
        assert predicate(sample)
        return sample

    def capture(self, req):
        self.history.append(("capture" if req.phase == req.CAPTURE else "prepare",
                             req.position_index))
        success = not (req.phase == req.CAPTURE and req.position_index == self.fail_capture)
        result = SimpleNamespace(success=success, message="test capture")
        return SimpleNamespace(done=lambda: True, result=lambda: result)

    def failure(self, goal, result, exc, outcome):
        self.history.append("Stop")
        goal.abort()
        result.outcome, result.message = outcome, str(exc)
        return result


def test_complete_replay_serializes_each_move_capture_and_preserves_outputs():
    rig = ReplayRig()
    assert rig.engine.reserve(rig.goal.request) == GoalResponse.ACCEPT
    result = rig.engine.execute(rig.goal)
    assert result.outcome == result.SUCCESS and result.captured_count == 5
    expected = [("prepare", 1), "CP"]
    for index in range(1, 6):
        if index > 1:
            expected.append(("prepare", index))
        expected.extend([("move", f"calibration_{index}"), ("capture", index)])
    assert rig.history == expected
    assert rig.node.machine.state == "UNCONFIGURED"
    rig.goal.succeed.assert_called_once()
    rig.node._end_operation.assert_called_once()


def test_failed_capture_stops_without_next_position_or_success():
    rig = ReplayRig()
    rig.fail_capture = 2
    result = rig.engine.execute(rig.goal)
    assert result.outcome == result.FEEDBACK_FAILURE and result.captured_count == 1
    assert rig.history[-1] == "Stop"
    assert ("prepare", 3) not in rig.history
    rig.goal.succeed.assert_not_called()


def test_cancel_before_execution_never_moves():
    rig = ReplayRig()
    rig.cancel = True
    rig.engine.execute(rig.goal)
    assert rig.history == ["Stop"]


@pytest.mark.parametrize("field,value", [
    ("headless", True), ("holding_item", True), ("state", "PAUSED"),
    ("session", object()), ("di1", 1), ("running", 1), ("enabled", 0),
    ("user", 1), ("tool", 1)])
def test_admission_rejects_unsafe_or_conflicting_robot_context(field, value):
    rig = ReplayRig()
    if field == "state":
        rig.node.machine = ControllerStateMachine(value)
    elif field == "session":
        rig.node.managed.session = value
    elif field in ("di1", "running", "enabled", "user", "tool"):
        key = {"di1": "digital_input_bits", "running": "RunningStatus",
               "enabled": "EnableStatus", "user": "userCoordinate",
               "tool": "toolCoordinate"}[field]
        rig.monitor.update_feed(feed(**{key: value}))
    else:
        setattr(rig.node, field, value)
    assert rig.engine.reserve(rig.goal.request) == GoalResponse.REJECT
    assert not rig.history


@pytest.mark.parametrize("changes", [{"digital_outputs": 1}, {"digital_input_bits": 1},
                                     {"EnableStatus": 0}])
def test_feedback_violation_during_capture_blocks_next_move(changes):
    rig = ReplayRig()
    original = rig.capture

    def violate(req):
        result = original(req)
        if req.phase == req.CAPTURE:
            rig.monitor.update_feed(feed(controller_timer=2, **changes))
        return result

    rig.engine.capture_client.call_async = violate
    result = rig.engine.execute(rig.goal)
    assert result.outcome == result.FEEDBACK_FAILURE and result.captured_count == 0
    assert rig.history[-1] == "Stop"


def test_capture_wait_rejects_motion_even_inside_arrival_tolerance():
    rig = ReplayRig()
    rig.engine.outputs = 0
    rig.engine.last_heartbeat = float("inf")
    rig.engine.capture_anchor = np.zeros(6)
    rig.engine.capture_joints = np.zeros(6)
    rig.monitor.update_feed(feed(tool_vector_actual=[.1, 0., 0., 0., 0., 0.]))
    with pytest.raises(FeedbackFailure, match="moved while capturing"):
        rig.engine._capture(CaptureCalibration.Request(), target=object())


def test_capture_service_deadline_removes_pending_request(monkeypatch):
    from robot_controller import calibration_replay

    rig = ReplayRig()
    rig.engine.outputs = 0
    rig.engine.last_heartbeat = float("inf")
    rig.engine.capture_anchor = rig.engine.capture_joints = np.zeros(6)
    rig.node.wait_control = lambda _seconds: None
    pending = SimpleNamespace(done=lambda: False)
    rig.engine.capture_client.call_async = lambda _request: pending
    rig.engine.capture_client.remove_pending_request = MagicMock()
    ticks = iter(range(100))
    monkeypatch.setattr(calibration_replay, "time", SimpleNamespace(
        monotonic=lambda: next(ticks)))
    with pytest.raises(FeedbackFailure, match="timed out"):
        rig.engine._capture(CaptureCalibration.Request(), target=object())
    rig.engine.capture_client.remove_pending_request.assert_called_once_with(pending)
