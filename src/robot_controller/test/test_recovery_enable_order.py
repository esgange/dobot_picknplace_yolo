"""Recovery clears/enables before standstill checks; all robot calls are fake."""

from types import SimpleNamespace

import pytest
from rclpy.task import Future

import robot_controller.hardware as hardware_module
from robot_controller.errors import (
    CommandRejected, CommandResponseTimeout, FeedbackFailure, HeldUnknown,
    OperationCanceled, StopUnconfirmed)
from robot_controller.recovery import HomeRecovery
from test_feedback_v2 import feed, joint_message
from test_placement_queue import HELD, QueueRig


class RecoverRig(QueueRig):
    def __init__(self, monkeypatch, *, held=False, failure=None):
        super().__init__()
        self.failure = failure
        self.cancelled = False
        self.enable_sent = False
        self.phase = ''
        self.fields = feed(robot_mode=11, CollisionStates=1, EnableStatus=1,
                           digital_outputs=HELD, digital_input_bits=int(held))
        self.node.check_feedback_owners = lambda: None
        self.node.global_cp_percent = 100
        self.node.cancel_requested = lambda: self.cancelled
        self.node.raise_if_cancelled = self.check_cancel
        self.node.operation_progress = self.progress
        self.node.wait_control = self.tick
        self.node.publish_status = lambda: None
        self.transport._wait_for_resume = self.check_cancel
        # The real FeedbackMonitor and service transport use this bounded fake clock.
        monkeypatch.setattr(hardware_module, 'time', SimpleNamespace(monotonic=lambda: self.clock))
        self.monitor._condition.wait = self.tick
        for _ in range(60):
            self.tick()
        self.recovery = HomeRecovery.cancel_action(self.node)
        self.on_request = self.respond
        real_stop = self.transport.stop_client.call_async

        def stop(request):
            result = real_stop(request)
            if failure == 'stop_rejected':
                result.result().res = -1
            if failure == 'stop_timeout':
                return Future()
            if failure == 'stop_cancel':
                self.cancelled = True
            return result
        self.transport.stop_client.call_async = stop

    @property
    def commands(self):
        return [name for name, _ in self.requests]

    def check_cancel(self):
        if self.cancelled:
            raise OperationCanceled('Operator Stop')

    def progress(self, phase, *_args, **_kwargs):
        self.phase = phase
        if phase == 'RECOVERY_STANDSTILL':
            assert self.enable_sent

    def tick(self, *_args):
        self.clock += .05
        self.timer += 1
        post_enable = self.phase == 'RECOVERY_STANDSTILL'
        if post_enable and self.failure == 'stationary_cancel':
            self.cancelled = True
        if post_enable and self.failure == 'stale':
            return
        joints = joint_message()
        # Before Enable, mode 11 and changing joints cannot confirm Stop.
        if not self.enable_sent or post_enable and self.failure == 'moving':
            joints.position[0] = self.timer * .01
        if post_enable and self.failure == 'outputs':
            self.fields['digital_outputs'] = 0
        if post_enable and self.failure == 'queue':
            self.fields['isRunQueuedCmd'] = 1
        self.monitor.update_joints(joints)
        self.monitor.update_status(SimpleNamespace(
            is_connected=True, is_enable=self.fields['robot_mode'] == 5))
        self.monitor.update_feed(dict(self.fields, controller_timer=self.timer))

    def respond(self, _index):
        name = self.commands[-1]
        if name == 'ClearError':
            assert not self.enable_sent
            assert self.phase != 'RECOVERY_STANDSTILL'
            if self.failure != 'alarm':
                self.fields.update(robot_mode=4, CollisionStates=0, EnableStatus=0)
        elif name == 'EnableRobot':
            self.enable_sent = True
            if self.failure != 'enable_feedback':
                self.fields.update(robot_mode=5, EnableStatus=1)
        elif name in ('SpeedFactor', 'User', 'Tool', 'SetTool', 'CP'):
            assert any(call.args[1] == 'stop_confirmed'
                       for call in self.node.events.record.call_args_list)
        self.tick()

    def recover(self):
        self.transport.recover(60, home_recovery=self.recovery)


@pytest.mark.parametrize('held', [False, True])
def test_collision_recovery_accepts_stop_then_clears_enables_and_proves_standstill(
        monkeypatch, held):
    rig = RecoverRig(monkeypatch, held=held)
    rig.recover()
    assert rig.commands == ['Stop', 'ClearError', 'EnableRobot',
                            'SpeedFactor', 'User', 'Tool', 'SetTool', 'CP']
    assert rig.recovery.holding is held
    assert rig.fields['digital_outputs'] == HELD  # No gripper reset during preparation.
    assert not rig.transport.moving


@pytest.mark.parametrize('failure,error', [
    ('stop_rejected', StopUnconfirmed), ('stop_timeout', StopUnconfirmed),
    ('stop_cancel', OperationCanceled),
])
def test_failed_stop_acceptance_never_clears_or_enables(monkeypatch, failure, error):
    rig = RecoverRig(monkeypatch, failure=failure)
    with pytest.raises(error):
        rig.recover()
    assert rig.commands == ['Stop']


@pytest.mark.parametrize('service', ['ClearError', 'EnableRobot'])
@pytest.mark.parametrize('unanswered', [False, True])
def test_failed_setup_never_sends_settings_or_motion(monkeypatch, service, unanswered):
    rig = RecoverRig(monkeypatch)
    original = rig.transport.clients[service].call_async

    def reject(request):
        result = original(request)
        if unanswered:
            return Future()
        result.result().res = -1
        return result
    rig.transport.clients[service].call_async = reject
    with pytest.raises(CommandResponseTimeout if unanswered else CommandRejected):
        rig.recover()
    assert rig.commands == ['Stop', 'ClearError'] + (['EnableRobot'] if service == 'EnableRobot'
                                                   else [])


@pytest.mark.parametrize('failure,error', [
    ('enable_feedback', FeedbackFailure), ('moving', StopUnconfirmed),
    ('queue', StopUnconfirmed), ('stale', StopUnconfirmed),
    ('outputs', StopUnconfirmed), ('stationary_cancel', OperationCanceled),
])
def test_post_enable_failure_blocks_settings_and_motion(monkeypatch, failure, error):
    rig = RecoverRig(monkeypatch, failure=failure)
    with pytest.raises(error):
        rig.recover()
    assert rig.commands == ['Stop', 'ClearError', 'EnableRobot']


@pytest.mark.parametrize('invalid', ['unknown_suction', 'opposing', 'stale'])
def test_fresh_grip_guards_still_apply_before_enabling(monkeypatch, invalid):
    rig = RecoverRig(monkeypatch)
    if invalid == 'unknown_suction':
        rig.fields['digital_input_bits'] = 1
        rig.recovery.trusted_held = False
    elif invalid == 'opposing':
        rig.fields['digital_outputs'] |= 1
    else:
        rig.on_request = lambda _index: None
        rig.clock += 1
    with pytest.raises(HeldUnknown if invalid == 'unknown_suction' else FeedbackFailure):
        rig.recover()
    assert rig.commands == ['Stop']


def test_unresolved_prior_motion_blocks_recovery_commands(monkeypatch):
    rig = RecoverRig(monkeypatch)
    rig.transport.pending_response = ('MovL', Future(), {})
    with pytest.raises(CommandResponseTimeout, match='Still awaiting MovL'):
        rig.recover()
    assert not rig.commands
