"""MovL then MovJ Tray Detect queues with real transport and synthetic feedback."""

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.task import Future

from robot_controller.errors import CommandRejected, OperationCanceled
from robot_controller.motion import tray_detect_targets
from test_feedback_v2 import joint_message
from test_placement_queue import QueueRig, HELD


class TrayQueueRig(QueueRig):
    def __init__(self):
        self.reject = None
        self.linear_reply = None
        super().__init__()
        self.node.placement = None

    def client(self, kind, endpoint):
        client = super().client(kind, endpoint)
        call = client.call_async

        def reply(request):
            call(request)
            name = endpoint.rsplit('/', 1)[-1]
            if name == 'MovL' and self.linear_reply is not None:
                return self.linear_reply
            future = Future()
            future.set_result(SimpleNamespace(
                res=-1 if name == self.reject else 0,
                robot_return=f'{{{3 + len(self.requests)}}}'))
            return future

        client.call_async = reply
        return client

    def emit(self, *, wrist_error=0., **kwargs):
        sample = super().emit(**kwargs)
        if wrist_error:
            joints = joint_message()
            joints.position = list(sample.joints)
            joints.position[5] += wrist_error
            self.joint_poses[tuple(joints.position)] = self.node.configuration.tray.detect_matrix
            self.monitor.update_joints(joints)
        return self.monitor.snapshot(require_enabled=True)

    def travel(self):
        config = self.node.configuration
        targets = tray_detect_targets(config.tray.detect_matrix, config.tray.detect_joints,
                                      speed_percent=80, acceleration_percent=70)
        return self.transport.move_batch(targets, batch_name='tray_position',
                                         require_suction=True, preserve_outputs=True)


@pytest.mark.parametrize('wrist_degrees', [-347.524, 12.476, 372.476])
def test_both_commands_send_identical_unwrapped_saved_joints_before_any_arrival(wrist_degrees):
    rig = TrayQueueRig()
    taught = [10., -20., 30., -40., 50., wrist_degrees]
    rig.node.configuration.tray.detect_joints = tuple(np.deg2rad(taught))
    rig.emit(outputs=HELD, inputs=1, running=0)
    rig.order.clear()
    rig.steps = iter([dict(outputs=HELD, inputs=1, running=0, currentCommandId=5)])
    rig.travel()
    assert rig.order == ['MovL', 'MovJ', 'feedback']
    for _, request in rig.requests:
        assert request.mode
        assert [getattr(request, key) for key in 'abcdef'] == pytest.approx(taught)
        assert list(request.param_value) == ['user=0', 'tool=0', 'v=80', 'a=70']
    assert rig.node.expected_outputs == {1: False, 2: True, 13: True, 14: False}


@pytest.mark.parametrize('wrist_error', [-2 * np.pi, np.pi, 2 * np.pi])
def test_only_final_movj_id_idle_and_exact_raw_joints_can_complete(wrist_error):
    rig = TrayQueueRig()
    script = iter([
        dict(outputs=HELD, inputs=1, running=0, currentCommandId=4),
        dict(outputs=HELD, inputs=1, running=0, currentCommandId=5, wrist_error=wrist_error),
        dict(outputs=HELD, inputs=1, running=1, currentCommandId=5),
        dict(outputs=HELD, inputs=1, running=0, currentCommandId=5),
    ])

    def advance(*_args, **_kwargs):
        assert [name for name, _ in rig.requests] == ['MovL', 'MovJ']
        rig.emit(**next(script))
        return rig.monitor.sequence

    rig.monitor.wait_next = advance
    rig.travel()
    assert next(script, None) is None
    assert rig.transport.home_already_reached(rig.node.configuration.tray.detect_joints)


def test_movj_waits_for_linear_acceptance_but_not_linear_arrival():
    rig = TrayQueueRig()
    rig.linear_reply = Future()

    def accept(_seconds):
        assert [name for name, _ in rig.requests] == ['MovL']
        assert rig.monitor.snapshot().feed['currentCommandId'] != 4
        rig.linear_reply.set_result(SimpleNamespace(res=0, robot_return='{4}'))

    rig.node.wait_control = Mock(side_effect=accept)
    rig.steps = iter([dict(outputs=HELD, inputs=1, running=0, currentCommandId=5)])
    rig.travel()
    rig.node.wait_control.assert_called_once()
    assert [name for name, _ in rig.requests] == ['MovL', 'MovJ']


@pytest.mark.parametrize('command', ['MovL', 'MovJ'])
def test_rejection_blocks_completion_without_extra_motion(command):
    rig = TrayQueueRig()
    rig.reject = command
    with pytest.raises(CommandRejected):
        rig.travel()
    assert [name for name, _ in rig.requests if name != 'Stop'] == (
        ['MovL'] if command == 'MovL' else ['MovL', 'MovJ'])


@pytest.mark.parametrize('at', [1, 2])
def test_stop_during_admission_prevents_later_motion(at):
    rig = TrayQueueRig()

    def stop(index):
        if index == at:
            rig.node.cancel_requested = lambda: True
            rig.node.raise_if_cancelled = Mock(side_effect=OperationCanceled('Stop'))

    rig.on_request = stop
    with pytest.raises(OperationCanceled):
        rig.travel()
    assert [name for name, _ in rig.requests if name != 'Stop'] == ['MovL', 'MovJ'][:at]
