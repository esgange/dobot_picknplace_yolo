from types import SimpleNamespace

import pytest

from robot_controller.errors import FeedbackFailure, HeldUnknown
from robot_controller.hardware import DobotTransport, HOME_JOINT_TOLERANCE_RAD


class EventLog:
    def __init__(self):
        self.entries = []

    def record(self, *_args, **_kwargs):
        self.entries.append((_args, _kwargs))


def snapshot(*, di1=False, outputs=0, joints=None):
    return SimpleNamespace(
        robot_enabled=True,
        joints=tuple([0.0] * 6 if joints is None else joints),
        feed={
            "digital_input_bits": int(di1), "digital_outputs": outputs,
            "robot_mode": 5, "EnableStatus": 1, "isRunQueuedCmd": 0,
            "RunningStatus": 0, "ErrorStatus": 0, "CollisionStates": 0,
            "isPauseCmdFlag": 0, "userCoordinate": 0, "toolCoordinate": 0,
        })


def test_cold_di1_is_held_unknown_and_preserves_output_context():
    transport = object.__new__(DobotTransport)
    transport.monitor = SimpleNamespace(snapshot=lambda **_kwargs: snapshot(di1=True))
    with pytest.raises(HeldUnknown, match="without trusted"):
        transport._check_held_context(False, {})


def test_known_holding_requires_di1_and_exact_expected_outputs():
    transport = object.__new__(DobotTransport)
    transport.monitor = SimpleNamespace(
        snapshot=lambda **_kwargs: snapshot(di1=True, outputs=1 << 12))
    transport._check_held_context(True, {13: True})
    with pytest.raises(HeldUnknown, match="DO2"):
        transport._check_held_context(True, {2: True, 13: True})
    transport.monitor = SimpleNamespace(snapshot=lambda **_kwargs: snapshot(di1=False))
    with pytest.raises(HeldUnknown, match="lost DI1"):
        transport._check_held_context(True, {13: True})


def test_startup_sequence_is_explicit_and_never_calls_home():
    transport = object.__new__(DobotTransport)
    order = []
    transport.node = SimpleNamespace(
        check_all_command_owners=lambda services: order.append(("owners", tuple(services))),
        check_feedback_owners=lambda: order.append(("feedback_owners",)),
        cancel_requested=lambda: False, events=EventLog(),
        operation_progress=lambda phase, message, **fields: order.append(
            ("phase", phase, message, fields)))
    transport.clients = {"EnableRobot": object()}
    transport.queue_clients = {"Pause": object(), "Continue": object()}
    transport.monitor = SimpleNamespace(
        snapshot=lambda **_kwargs: snapshot(),
        wait_samples=lambda *_args, **_kwargs: order.append(("wait_samples",)))
    transport.wait_services = lambda **kwargs: order.append(
        ("services", tuple(kwargs.get("optional", ()))))
    transport._call_startup = lambda name, **_kwargs: order.append(("call", name))
    transport.request_stop = lambda reason: order.append(("stop", reason)) or object()
    transport.confirm_stop = lambda _future: order.append(("stop_confirmed",))
    transport._check_held_context = lambda known, expected: order.append(
        ("held_check", known, expected))
    transport._clear_errors_if_needed = lambda: order.append(("clear_if_needed",))
    transport._wait_enabled = lambda: order.append(("enabled",))
    transport._apply_settings = lambda speed: order.append(("settings", speed))
    transport._reset_outputs_if_unheld = lambda: order.append(("output_reset",))
    transport._confirm_ready = lambda: order.append(("ready",))

    DobotTransport.startup(transport)

    calls = [entry[1] for entry in order if entry[0] == "call"]
    assert calls == ["StopMoveJog", "DisableRobot", "EnableRobot"]
    assert order.index(("stop_confirmed",)) < order.index(("call", "DisableRobot"))
    assert order.index(("call", "EnableRobot")) < order.index(("settings", 100))
    assert order.count(("enabled",)) == 1
    assert order.index(("output_reset",)) < order.index(("ready",))
    assert order[-2:] == [("output_reset",), ("ready",)]
    assert all("Home" not in str(entry) and "MovL" not in str(entry) for entry in order)


def test_final_ready_accepts_idle_pause_latch():
    transport = object.__new__(DobotTransport)
    paused = snapshot()
    paused.feed["isPauseCmdFlag"] = 1

    class Monitor:
        def wait(self, predicate, *_args, **_kwargs):
            assert predicate(paused)
            return paused

        def snapshot(self, **_kwargs):
            return paused

    transport.monitor = Monitor()
    transport.node = SimpleNamespace(
        holding_item=False, expected_outputs={}, cancel_requested=lambda: False,
        operation_progress=lambda *_args, **_kwargs: None)
    assert transport._confirm_ready() is paused


class CompletedFuture:
    def __init__(self, result=0):
        self._result = SimpleNamespace(res=result)

    def done(self):
        return True

    def result(self):
        return self._result


class StopMonitor:
    def __init__(self, sample):
        self.sample = sample

    def wait(self, predicate, *_args, **_kwargs):
        assert not predicate(self.sample)
        assert predicate(self.sample)
        return self.sample


def test_stop_confirmation_detects_held_item_loss_without_changing_outputs():
    transport = object.__new__(DobotTransport)
    transport.node = SimpleNamespace(
        holding_item=True, expected_outputs={13: True}, events=EventLog(),
        wait_control=lambda _seconds: None)
    stopped = snapshot(di1=False, outputs=1 << 12)
    stopped.feed["tool_vector_actual"] = [0.0] * 6
    transport.monitor = StopMonitor(stopped)
    transport.moving = True
    with pytest.raises(Exception, match="held-item integrity"):
        transport.confirm_stop(CompletedFuture())
    assert transport.node.expected_outputs == {13: True}


def test_stop_confirmation_accepts_a_latched_pause_after_queue_is_empty():
    transport = object.__new__(DobotTransport)
    transport.node = SimpleNamespace(
        holding_item=False, expected_outputs={}, events=EventLog(),
        wait_control=lambda _seconds: None)
    stopped = snapshot()
    stopped.feed["isPauseCmdFlag"] = 1
    stopped.feed["tool_vector_actual"] = [0.0] * 6
    transport.monitor = StopMonitor(stopped)
    transport.moving = True
    transport.confirm_stop(CompletedFuture())
    assert not transport.moving


class SensorMonitor:
    def __init__(self, *, fresh):
        self.fresh = fresh

    def wait(self, *_args, **_kwargs):
        raise FeedbackFailure("Timed out waiting for DI1=1")

    def snapshot(self, **_kwargs):
        if not self.fresh:
            raise FeedbackFailure("stale feedback")
        return snapshot(di1=False)


def test_only_fresh_coherent_opposite_di1_becomes_a_missed_pick():
    transport = object.__new__(DobotTransport)
    transport.node = SimpleNamespace(cancel_requested=lambda: False)
    transport.monitor = SensorMonitor(fresh=True)
    assert not transport.sensor(True, 0.2, settling_sec=0)
    transport.monitor = SensorMonitor(fresh=False)
    with pytest.raises(FeedbackFailure, match="Timed out waiting"):
        transport.sensor(True, 0.2, settling_sec=0)


def test_service_audit_records_exact_send_and_terminal_response():
    transport = object.__new__(DobotTransport)
    transport.service_audit_lock = __import__("threading").Lock()
    transport.service_audit_sequence = 0
    events = EventLog()
    console = []
    logger = SimpleNamespace(
        info=console.append, warning=console.append, error=console.append)
    transport.node = SimpleNamespace(
        events=events, get_logger=lambda: logger)

    audit = transport._begin_service_audit("SpeedFactor", {"ratio": 37})
    transport._finish_service_audit(
        audit, "accepted", result=SimpleNamespace(res=0, robot_return="{}"))

    assert "SEND /dobot_bringup_ros2/srv/SpeedFactor" in console[0]
    assert 'request={"ratio":37}' in console[0]
    assert "ACCEPTED /dobot_bringup_ros2/srv/SpeedFactor res=0" in console[1]
    sent = events.entries[0][1]
    response = events.entries[1][1]
    assert sent["request_id"] == response["request_id"] == 1
    assert sent["endpoint"] == "/dobot_bringup_ros2/srv/SpeedFactor"
    assert response["response_res"] == 0
    assert response["robot_return"] == "{}"


class HomeReachedMonitor:
    def __init__(self, sample):
        self.sample = sample
        self.wait_calls = 0

    def snapshot(self, **_kwargs):
        return self.sample

    def wait(self, predicate, _timeout, **kwargs):
        self.wait_calls += 1
        assert kwargs["stable_sec"] == pytest.approx(0.3)
        assert predicate(self.sample)
        return self.sample


def home_reached_transport(sample):
    transport = object.__new__(DobotTransport)
    transport.monitor = HomeReachedMonitor(sample)
    transport.node = SimpleNamespace(
        holding_item=False, expected_outputs={}, cancel_requested=lambda: False)
    return transport


def test_existing_home_uses_same_one_degree_stable_completion_gate():
    transport = home_reached_transport(
        snapshot(joints=[HOME_JOINT_TOLERANCE_RAD] * 6))
    assert transport.home_already_reached((0.0,) * 6)
    assert transport.monitor.wait_calls == 1

    transport = home_reached_transport(
        snapshot(joints=[1.01 * HOME_JOINT_TOLERANCE_RAD] * 6))
    assert not transport.home_already_reached((0.0,) * 6)
    assert transport.monitor.wait_calls == 0


def test_existing_home_skip_requires_idle_empty_queue():
    sample = snapshot(joints=[0.0] * 6)
    sample.feed["isRunQueuedCmd"] = 1
    transport = home_reached_transport(sample)
    assert not transport.home_already_reached((0.0,) * 6)
    assert transport.monitor.wait_calls == 0


class CurrentPoseMonitor:
    def __init__(self, sample, *, fail=False):
        self.sample = sample
        self.fail = fail
        self.wait_kwargs = None

    def wait(self, predicate, timeout, **kwargs):
        self.wait_kwargs = {"timeout": timeout, **kwargs}
        if self.fail:
            raise FeedbackFailure("synthetic stationary timeout")
        assert predicate(self.sample)
        return self.sample

    def snapshot(self, **_kwargs):
        return self.sample


def current_pose_transport(sample, *, fail=False):
    transport = object.__new__(DobotTransport)
    transport.monitor = CurrentPoseMonitor(sample, fail=fail)
    transport.node = SimpleNamespace(
        holding_item=False, expected_outputs={}, cancel_requested=lambda: False)
    calls = []
    transport.call = lambda name, **fields: (
        calls.append((name, fields)) or SimpleNamespace(robot_return="{1,2,3,0,0,0}"))
    return transport, calls


def test_current_pose_waits_for_300ms_stationary_idle_before_getpose():
    transport, calls = current_pose_transport(snapshot())

    matrix = transport.current_pose()

    assert transport.monitor.wait_kwargs["timeout"] == pytest.approx(2.0)
    assert transport.monitor.wait_kwargs["stable_sec"] == pytest.approx(0.3)
    assert transport.monitor.wait_kwargs["require_enabled"] is True
    assert calls == [("GetPose", {"user": 0, "tool": 0})]
    assert tuple(matrix[:3, 3]) == pytest.approx((0.001, 0.002, 0.003))


def test_current_pose_timeout_reports_exact_idle_blockers_and_skips_getpose():
    blocked = snapshot()
    blocked.robot_enabled = False
    blocked.feed.update(robot_mode=7, isRunQueuedCmd=1, RunningStatus=1)
    transport, calls = current_pose_transport(blocked, fail=True)

    with pytest.raises(FeedbackFailure) as captured:
        transport.current_pose()

    message = str(captured.value)
    assert "RobotStatus.is_enable=False" in message
    assert "robot_mode=7" in message
    assert "isRunQueuedCmd=1" in message
    assert "RunningStatus=1" in message
    assert calls == []


def test_current_pose_reports_unstable_idle_without_inventing_a_blocker():
    transport, calls = current_pose_transport(snapshot(), fail=True)

    with pytest.raises(FeedbackFailure, match="did not remain coherent for 300 ms"):
        transport.current_pose()
    assert calls == []
