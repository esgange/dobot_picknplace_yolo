"""A later explicit Stop must have its own dispatch and physical confirmation."""

from types import SimpleNamespace
import threading

import pytest

from robot_controller.controller import RobotController
from robot_controller.errors import CommandRejected, StopUnconfirmed
from robot_controller.hardware import DobotTransport
from robot_controller.state_machine import ControllerStateMachine

from test_recovery_return import RecoveryRig


def rig_at_ready():
    rig = RecoveryRig()
    rig.machine = ControllerStateMachine(initial="READY")
    rig.holding_item = False
    rig.set_di1(False)
    rig.managed.session = None
    return rig


def stop(rig):
    return RobotController._stop(rig, None, SimpleNamespace())


def test_two_separate_clicks_send_and_confirm_twice():
    rig = rig_at_ready()
    assert stop(rig).success
    assert stop(rig).success
    assert sum(x[0] == "stop" for x in rig.log) == 2
    assert sum(x[0] == "stop_confirmed" for x in rig.log) == 2


def test_explicit_stop_can_retry_after_failed_confirmation():
    rig = rig_at_ready()
    confirm = rig.hardware.confirm_stop
    rig.hardware.confirm_stop = lambda *_a, **_k: (_ for _ in ()).throw(
        StopUnconfirmed("first response timed out"))
    assert not stop(rig).success
    rig.hardware.confirm_stop = confirm
    assert stop(rig).success
    assert sum(x[0] == "stop" for x in rig.log) == 2
    assert rig.machine.state == "RECOVERY_REQUIRED"


def test_concurrent_callers_share_only_the_ongoing_attempt():
    rig = rig_at_ready()
    first = rig._request_stop("first", fresh=True)
    second = rig._request_stop("second", fresh=True)
    assert first is second
    rig._confirm_shared_stop(first)
    rig._confirm_shared_stop(second)
    rig._finish_stop_state(first)
    assert sum(x[0] == "stop" for x in rig.log) == 1
    assert sum(x[0] == "stop_confirmed" for x in rig.log) == 1
    assert rig._request_stop("third", fresh=True) is not first


def test_old_confirmation_and_error_cannot_finish_a_new_stop_or_operation():
    rig = rig_at_ready()
    first = rig._request_stop("first", fresh=True)
    rig._confirm_shared_stop(first)
    second = rig._request_stop("second", fresh=True)
    rig._finish_stop_state(first)
    RobotController._fail_stop_state(rig, first, "old error")
    assert rig.machine.state == "STOPPING"
    assert not second.confirmed
    rig._confirm_shared_stop(second)
    rig._finish_stop_state(second)
    rig._begin_operation("recover")
    rig._transition("RECOVERING", "new operation")
    rig._finish_stop_state(first)
    rig._finish_stop_state(second)
    RobotController._fail_stop_state(rig, second, "old error")
    assert rig.machine.state == "RECOVERING"
    rig._end_operation()


def test_transport_fresh_stop_bypasses_an_unanswered_earlier_stop():
    sent = []

    def dispatch(_request):
        future = SimpleNamespace(done=lambda: False, add_done_callback=lambda _cb: None)
        sent.append(future)
        return future
    transport = object.__new__(DobotTransport)
    transport.stop_lock = threading.Lock()
    transport.stop_future = None
    transport.stop_type = SimpleNamespace(Request=lambda: object())
    transport.stop_client = SimpleNamespace(service_is_ready=lambda: True, call_async=dispatch)
    transport.node = SimpleNamespace(check_command_owner=lambda _name: None,
                                     events=SimpleNamespace(record=lambda *_a, **_k: None))
    transport._begin_service_audit = lambda *_a, **_k: {}
    first = transport.request_stop()
    assert transport.request_stop() is first
    assert transport.request_stop(fresh=True) is not first
    assert len(sent) == 2


def test_operation_cannot_clear_an_in_progress_stop():
    rig = rig_at_ready()
    attempt = rig._request_stop("first", fresh=True)
    with pytest.raises(CommandRejected, match="Stop is still"):
        rig._begin_operation("pick")
    assert rig.stop_attempt is attempt and rig.cancel_event.is_set()
    assert not rig.operation_lock.locked()
