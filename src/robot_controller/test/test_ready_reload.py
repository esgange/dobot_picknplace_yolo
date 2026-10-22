"""Runtime deployment can reuse proven readiness without issuing hardware commands."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import robot_controller.controller as controller_module
from item_perception_yolo.repick import REPICK_DEFAULTS
from robot_controller.controller import RobotController
from robot_controller.errors import CommandRejected, FeedbackFailure, HeldUnknown, OperationCanceled
from robot_controller.hardware import DobotTransport
from test_lifecycle_v2 import configuration_node
from test_load_prepare import request, response
from test_transport_v2 import snapshot


def runtime_configuration(monkeypatch):
    config = SimpleNamespace(configuration_id='runtime', deployment=True,
                             validate_sources=Mock(), profile={'retry': dict(REPICK_DEFAULTS)})
    monkeypatch.setattr(controller_module, 'load_configuration', Mock(return_value=config))
    return config


def test_runtime_reload_preserves_ready_settings_and_outputs_without_startup(monkeypatch):
    node, _old, calls = configuration_node()
    config = runtime_configuration(monkeypatch)
    node.hardware.confirm_prepared_configuration = lambda: calls.append(('verify_ready',))
    outputs = dict(node.expected_outputs)
    result = RobotController._configure(node, request(), response())
    assert result.success and result.configuration_id == 'runtime'
    assert node.configuration is config and node.machine.state == 'READY'
    assert node.startup_complete and node.global_speed_percent == 35
    assert node.global_cp_percent == 25 and node.expected_outputs == outputs
    assert 'without re-enabling' in result.message
    assert calls == [('begin', 'configure'), ('verify_ready',),
                     ('log', 'configuration_loaded'), ('end',)]
    config.validate_sources.assert_called_once_with(node.root)


@pytest.mark.parametrize('initial', ['UNCONFIGURED', 'INACTIVE', 'READY'])
def test_unprepared_runtime_load_retains_explicit_startup(monkeypatch, initial):
    node, _old, calls = configuration_node(initial)
    node.startup_complete = False
    runtime_configuration(monkeypatch)
    node.hardware.confirm_prepared_configuration = Mock()
    result = RobotController._configure(node, request(), response())
    assert result.success and node.startup_complete and node.machine.state == 'READY'
    assert calls.count(('startup',)) == 1
    node.hardware.confirm_prepared_configuration.assert_not_called()


@pytest.mark.parametrize('error,state', [
    (FeedbackFailure('stale feedback'), 'FAULT'),
    (FeedbackFailure('robot disabled'), 'FAULT'),
    (CommandRejected('retained operation'), 'FAULT'),
    (HeldUnknown('DI1 HIGH'), 'HELD_UNKNOWN'),
])
def test_failed_ready_verification_never_falls_back_to_enable(monkeypatch, error, state):
    node, old, calls = configuration_node()
    runtime_configuration(monkeypatch)
    node.hardware.confirm_prepared_configuration = Mock(side_effect=error)
    result = RobotController._configure(node, request(), response())
    assert not result.success and result.configuration_id == ''
    assert str(error) in result.message and node.machine.state == state
    assert node.configuration is old and not node.startup_complete
    assert ('startup',) not in calls
    assert node.global_speed_percent == 35 and node.global_cp_percent == 25
    assert node.expected_outputs == {13: 1}


@pytest.mark.parametrize('stage', ['loading', 'verification', 'installed'])
def test_stop_during_runtime_reload_never_enables_or_reports_success(monkeypatch, stage):
    node, old, calls = configuration_node()
    config = runtime_configuration(monkeypatch)
    canceled = []

    def stop():
        canceled.append(True)
        node.machine.transition('STOPPING', 'Explicit Stop')

    def check():
        if canceled:
            raise OperationCanceled('Explicit Stop')

    node.raise_if_cancelled = check
    node._settle_lifecycle_cancellation = lambda _message: calls.append(('settle',))
    node.hardware.confirm_prepared_configuration = stop if stage == 'verification' else check
    if stage == 'loading':
        controller_module.load_configuration.side_effect = lambda *_a, **_kw: (stop(), config)[1]
    if stage == 'installed':
        node._log_configuration = lambda _event: stop()
    result = RobotController._configure(node, request(), response())
    assert not result.success and 'Explicit Stop' in result.message
    assert node.machine.state == 'STOPPING' and ('settle',) in calls
    assert ('startup',) not in calls
    assert node.configuration is (config if stage == 'installed' else old)


def prepared_transport():
    transport = object.__new__(DobotTransport)
    samples = [snapshot(), snapshot()]
    for index, sample in enumerate(samples, 1):
        sample.feed['currentCommandId'] = 12
        sample.feed['controller_timer'] = index
        sample.sequence = sample.joint_stamp_ns = sample.status_sequence = sample.revision = index
    transport.node = SimpleNamespace(
        holding_item=False, global_speed_percent=35, global_cp_percent=25,
        expected_outputs=dict.fromkeys((1, 2, 13, 14), False),
        managed=SimpleNamespace(kind=None, session=None, held_loss_pending=False,
                                return_progress=None),
        check_all_command_owners=Mock(), check_feedback_owners=Mock(),
        raise_if_cancelled=Mock(), cancel_requested=lambda: False, operation_progress=Mock())
    transport.monitor = SimpleNamespace(
        snapshot=Mock(side_effect=samples),
        wait_next=Mock(side_effect=FeedbackFailure('feedback did not advance')))
    transport.clients = {}
    transport.moving = False
    transport.ensure_no_pending_response = Mock()
    transport.call = Mock(side_effect=AssertionError('Hardware commands are forbidden'))
    return transport, samples


@pytest.mark.parametrize('outputs', [0, 1 << 13, 1 << 12])
def test_ready_verification_only_reads_advancing_feedback(outputs):
    transport, samples = prepared_transport()
    transport.clients = {'EnableRobot': object(), 'StopMoveJog': object()}
    for sample in samples:
        sample.feed['digital_outputs'] = outputs
    transport.node.expected_outputs = {
        channel: bool(outputs & (1 << (channel - 1))) for channel in (1, 2, 13, 14)}
    expected = dict(transport.node.expected_outputs)
    transport.confirm_prepared_configuration()
    assert transport.monitor.snapshot.call_count == 2
    transport.ensure_no_pending_response.assert_called_once()
    transport.node.check_feedback_owners.assert_called_once()
    transport.node.check_all_command_owners.assert_called_once_with(('EnableRobot', 'Stop'))
    transport.call.assert_not_called()
    assert transport.node.expected_outputs == expected


@pytest.mark.parametrize('index', [0, 1])
@pytest.mark.parametrize('field,value', [
    ('robot_mode', 6), ('EnableStatus', 0), ('isRunQueuedCmd', 1), ('RunningStatus', 1),
    ('ErrorStatus', 1), ('CollisionStates', 1), ('userCoordinate', 1), ('toolCoordinate', 1),
    ('digital_input_bits', 1), ('digital_outputs', 1),
])
def test_unhealthy_feedback_cannot_reuse_ready(index, field, value):
    transport, samples = prepared_transport()
    samples[index].feed[field] = value
    with pytest.raises((FeedbackFailure, HeldUnknown)):
        transport.confirm_prepared_configuration()
    transport.call.assert_not_called()


@pytest.mark.parametrize('change', ['joints', 'command_id', 'timer', 'joint_stamp', 'status'])
def test_moving_or_nonadvancing_robot_cannot_reuse_ready(change):
    transport, samples = prepared_transport()
    if change == 'joints':
        samples[1].joints = (.1,) * 6
    elif change == 'command_id':
        samples[1].feed['currentCommandId'] += 1
    elif change == 'timer':
        samples[1].feed['controller_timer'] = 1
    elif change == 'joint_stamp':
        samples[1].joint_stamp_ns = 1
    else:
        samples[1].status_sequence = 1
    with pytest.raises(FeedbackFailure):
        transport.confirm_prepared_configuration()
    transport.call.assert_not_called()


@pytest.mark.parametrize('blocker', [
    'stale', 'disabled', 'pending_reply', 'command_owner', 'feedback_owner',
    'speed', 'cp', 'unknown_outputs', 'opposing_outputs', 'moving',
    'held_source', 'return', 'held_loss', 'pause', 'recovery',
])
def test_runtime_reuse_retains_ownership_settings_and_context_gates(blocker):
    transport, samples = prepared_transport()
    node = transport.node
    if blocker == 'stale':
        transport.monitor.snapshot.side_effect = FeedbackFailure('stale feedback')
    elif blocker == 'disabled':
        samples[0].robot_enabled = False
    elif blocker == 'pending_reply':
        transport.ensure_no_pending_response.side_effect = CommandRejected('pending reply')
    elif blocker.endswith('_owner'):
        callback = (node.check_all_command_owners if blocker == 'command_owner'
                    else node.check_feedback_owners)
        callback.side_effect = CommandRejected('competing owner')
    elif blocker == 'speed':
        node.global_speed_percent = None
    elif blocker == 'cp':
        node.global_cp_percent = None
    elif blocker == 'unknown_outputs':
        node.expected_outputs = {}
    elif blocker == 'opposing_outputs':
        node.expected_outputs.update({1: True, 13: True})
        samples[0].feed['digital_outputs'] = (1 << 12) | 1
    elif blocker == 'moving':
        transport.moving = True
    elif blocker == 'held_source':
        node.managed.session = SimpleNamespace(held_index=1)
    elif blocker == 'return':
        node.managed.return_progress = object()
    elif blocker == 'held_loss':
        node.managed.held_loss_pending = True
    elif blocker == 'pause':
        node.managed.kind = 'pause'
    else:
        node.recovery_home = SimpleNamespace(awaiting_return=True)
    with pytest.raises((CommandRejected, FeedbackFailure)):
        transport.confirm_prepared_configuration()
    transport.call.assert_not_called()
