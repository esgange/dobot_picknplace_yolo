"""Synthetic E-stop responses; never connect to a robot or clear a real alarm."""

from types import SimpleNamespace
import threading
import time

import pytest

from robot_controller.errors import (
    CommandRejected, EMERGENCY_STOP_MESSAGE, EmergencyStopPressed,
    FeedbackFailure, HeldSuctionLost, StopUnconfirmed, alarm_ids)
from robot_controller.hardware import DobotTransport
from robot_controller.controller import RobotController
from robot_controller.state_machine import ControllerStateMachine


@pytest.mark.parametrize("payload,expected", [
    ("{[]}", ()), ("{[1537]}", (1537,)), ("{[2048,1537]}", (2048, 1537)),
])
def test_alarm_payload(payload, expected):
    assert alarm_ids(payload) == expected


@pytest.mark.parametrize("payload", [
    None, "", "{}", "[1537]", "{[[1537]]}", '{["1537"]}', "{[true]}",
    "{[1537.0]}", "{[-3]}", "{[1537]}extra",
])
def test_invalid_alarm_payload_is_never_treated_as_clear(payload):
    with pytest.raises(FeedbackFailure, match="malformed GetErrorID"):
        alarm_ids(payload)


def lifecycle_transport(*, payload="{[1537]}", clearable=False):
    transport = object.__new__(DobotTransport)
    calls = []
    sample = SimpleNamespace(feed={"robot_mode": 9, "ErrorStatus": 1, "CollisionStates": 0})
    transport.node = SimpleNamespace(
        holding_item=False, expected_outputs={}, cancel_requested=lambda: False,
        check_all_command_owners=lambda _services: None, check_feedback_owners=lambda: None,
        events=SimpleNamespace(record=lambda *_args, **_kwargs: None),
        operation_progress=lambda *_args, **_kwargs: None)

    def wait_samples(predicate, *_args, description, **_kwargs):
        if description == "cleared error/collision feedback":
            if not clearable:
                raise FeedbackFailure("Timed out waiting for cleared error/collision feedback")
            sample.feed.update(robot_mode=4, ErrorStatus=0)
        assert predicate(sample)

    transport.monitor = SimpleNamespace(
        snapshot=lambda **_kwargs: sample, wait_samples=wait_samples)
    transport.clients = {"GetErrorID": object()}
    transport.wait_services = lambda **_kwargs: None
    transport.ensure_no_pending_response = lambda: None
    transport._check_held_context = lambda *_args: None
    transport._validate_held_snapshot = lambda value: value
    transport._wait_enabled = lambda: calls.append("enabled_confirmed")
    transport._apply_settings = lambda _speed: calls.append("settings")
    transport._reset_outputs_if_unheld = lambda: calls.append("outputs")
    transport._confirm_ready = lambda: calls.append("ready")
    transport.request_stop = lambda _reason: calls.append("Stop") or object()
    transport.confirm_stop = lambda *_args, **_kwargs: None

    def call(name, **_kwargs):
        calls.append(name)
        return SimpleNamespace(res=0, robot_return=payload)
    transport.call = call
    return transport, calls


def test_pressed_startup_only_reads_alarm_and_never_changes_hardware():
    transport, calls = lifecycle_transport()
    with pytest.raises(EmergencyStopPressed, match=EMERGENCY_STOP_MESSAGE):
        transport.startup()
    assert calls == ["GetErrorID"]


@pytest.mark.parametrize("payload,error,match", [
    ("{[1537]}", EmergencyStopPressed, EMERGENCY_STOP_MESSAGE),
    ("{[2048]}", FeedbackFailure, "Timed out waiting"),
    ("{[]}", FeedbackFailure, "Timed out waiting"),
    ("{}", FeedbackFailure, "malformed GetErrorID"),
])
def test_recovery_reports_alarm_after_clear_ack_but_never_enables(payload, error, match):
    transport, calls = lifecycle_transport(payload=payload)
    with pytest.raises(error, match=match):
        transport.recover(100)
    assert calls == ["Stop", "ClearError", "GetErrorID"]


def test_released_estop_can_clear_latched_alarm_and_recover_explicitly():
    transport, calls = lifecycle_transport(clearable=True)
    transport.recover(100)
    assert calls == ["Stop", "ClearError", "EnableRobot", "enabled_confirmed",
                     "settings", "outputs", "ready"]


def test_diagnostic_does_not_replace_held_suction_loss():
    transport, calls = lifecycle_transport()

    def wait(_predicate, *_args, description, **_kwargs):
        if description == "cleared error/collision feedback":
            raise HeldSuctionLost("held suction lost")
    transport.monitor.wait_samples = wait
    with pytest.raises(HeldSuctionLost, match="held suction lost"):
        transport.recover(100)
    assert calls == ["Stop", "ClearError"]


def test_estop_pressed_after_preflight_is_not_swallowed_as_best_effort():
    transport, calls = lifecycle_transport(payload="{[]}")

    def reject(name, **_kwargs):
        calls.append(name)
        raise EmergencyStopPressed(EMERGENCY_STOP_MESSAGE)
    transport._call_startup = reject
    with pytest.raises(EmergencyStopPressed):
        transport.startup()
    assert calls == ["GetErrorID", "StopMoveJog"]


@pytest.mark.parametrize("operation,initial", [("startup", "INACTIVE"), ("recover", "FAULT")])
def test_lifecycle_service_returns_explicit_estop_failure(operation, initial):
    machine = ControllerStateMachine(initial=initial)

    def pressed(*_args, **_kwargs):
        raise EmergencyStopPressed(EMERGENCY_STOP_MESSAGE)
    node = SimpleNamespace(
        machine=machine, root=object(), global_speed_percent=100,
        configuration=SimpleNamespace(validate_sources=lambda _root: None),
        managed=SimpleNamespace(recovery_return_needed=lambda: False),
        _begin_operation=lambda _name: None, _end_operation=lambda: None,
        _transition=lambda state, message: machine.transition(state, message),
        events=SimpleNamespace(record=lambda *_args, **_kwargs: None),
        hardware=SimpleNamespace(startup=pressed, recover=pressed))
    response = SimpleNamespace(success=True, state="", message="")
    getattr(RobotController, "_" + operation)(node, None, response)
    assert not response.success and response.state == "FAULT"
    assert EMERGENCY_STOP_MESSAGE in response.message


class CompletedFuture:
    def __init__(self, code):
        self.value = SimpleNamespace(res=code, robot_return="{}")

    def done(self):
        return True

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        callback(self)


@pytest.mark.parametrize("path", ["single", "group", "stop"])
@pytest.mark.parametrize("code", [-3, -2])
def test_real_transport_formats_rejections_and_keeps_stop_containment(path, code):
    transport = object.__new__(DobotTransport)
    calls, audits = [], []
    transport.response_lock = threading.RLock()
    transport.pending_response = transport.pending_group = None
    transport.suction_interrupted = False
    transport.node = SimpleNamespace(
        check_command_owner=lambda _name: None, check_all_command_owners=lambda _names: None,
        cancel_requested=lambda: False)
    transport.monitor = SimpleNamespace(snapshot=lambda **_kwargs: None)
    transport._wait_for_resume = lambda: None
    transport._pending_completed = transport._group_completed = lambda *_args: None
    transport._begin_service_audit = lambda name, _fields: {
        "name": name, "started": time.monotonic()}
    transport._finish_service_audit = lambda _audit, _outcome, **fields: audits.append(fields)
    future = CompletedFuture(code)
    transport.types = {"MovL": SimpleNamespace(Request=lambda **fields: fields)}
    transport.clients = {"MovL": SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda _request: calls.append("MovL") or future)}
    transport.request_stop = lambda _reason: calls.append("Stop")
    error = StopUnconfirmed if path == "stop" else (
        EmergencyStopPressed if code == -3 else CommandRejected)
    with pytest.raises(error) as caught:
        if path == "single":
            transport.call("MovL")
        elif path == "group":
            transport.call_group((("MovL", {}), ("MovL", {})))
        else:
            transport.confirm_stop(future)
    assert (EMERGENCY_STOP_MESSAGE in str(caught.value)) is (code == -3)
    assert (EMERGENCY_STOP_MESSAGE in audits[0]["detail"]) is (code == -3)
    assert calls == {"single": ["MovL"], "group": ["MovL", "Stop"], "stop": []}[path]
