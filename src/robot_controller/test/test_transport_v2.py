from types import SimpleNamespace

import pytest

from robot_controller.errors import FeedbackFailure, HeldUnknown
from robot_controller.hardware import DobotTransport


class EventLog:
    def record(self, *_args, **_kwargs):
        pass


def snapshot(*, di1=False, outputs=0):
    return SimpleNamespace(robot_enabled=True, feed={
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
