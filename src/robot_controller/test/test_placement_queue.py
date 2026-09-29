"""Placement exercises the real transport and feedback; no ROS/hardware calls."""

from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.task import Future

from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, HeldSuctionLost, OperationCanceled
from robot_controller.hardware import DobotTransport
from robot_controller.kinematics import pose_values
from robot_controller.motion import Target
from test_feedback_v2 import feed, primed_monitor, joint_message
from test_placement import operation_node


HELD = (1 << 12) | 2
RELEASE = (1 << 13) | 1
OPEN = 1 << 11


class QueueRig:
    def __init__(self, *, during_reply=False):
        self.node = node = operation_node()
        self.monitor = node.monitor = primed_monitor()
        self.clock = 0.
        self.monitor._monotonic = lambda: self.clock
        self.timer = 1
        self.requests = []
        self.order = []
        self.steps = iter(())
        self.on_request = lambda _index: None
        self.monitor.update_joints(joint_message())
        self.monitor.update_status(SimpleNamespace(is_connected=True, is_enable=True))
        self.emit(outputs=HELD, inputs=1, running=0)
        node.create_client = self.client
        node.check_all_command_owners = Mock()
        node.check_command_owner = Mock()
        node.on_late_motion_ack = Mock()
        node.cancel_requested = lambda: False
        node.observe_managed_feedback = lambda sample: RobotController.observe_managed_feedback(
            node, sample)
        node.wait_control = lambda _seconds: None
        node.hardware = self.transport = DobotTransport(node, self.monitor)
        self.transport.current_pose = lambda: node.configuration.tray.detect_matrix.copy()
        self.monitor.wait_next = self.next_sample
        self.during_reply = during_reply
        self.script = [
            dict(outputs=HELD, inputs=1),
            dict(outputs=RELEASE, inputs=OPEN),
            dict(outputs=0, inputs=OPEN),
            dict(outputs=0, inputs=OPEN, home=True),
        ]

    def emit(self, *, outputs, inputs, home=False, running=1):
        self.timer += 1
        self.clock += .01
        matrix = (self.node.configuration.home_matrix if home
                  else self.node.configuration.tray.detect_matrix)
        self.monitor.update_feed(feed(
            controller_timer=self.timer, digital_outputs=outputs, digital_input_bits=inputs,
            currentCommandId=4 if home else 2, RunningStatus=0 if home else running,
            isRunQueuedCmd=0 if home else running, tool_vector_actual=pose_values(matrix)))
        self.order.append('feedback')
        return self.monitor.snapshot(require_enabled=True)

    def client(self, kind, endpoint):
        def call(request):
            name = endpoint.rsplit('/', 1)[-1]
            self.requests.append((name, request))
            self.order.append(name)
            self.on_request(len(self.requests))
            if len(self.requests) == 4 and self.during_reply:
                for values in self.script:
                    self.emit(**values)
            future = Future()
            future.set_result(SimpleNamespace(res=0, robot_return='{4}'))
            return future
        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)

    def next_sample(self, *_args, **_kwargs):
        values = next(self.steps, None)
        if values is None:
            pytest.fail('Unexpected feedback wait after final Home')
        self.emit(**values)
        return self.monitor.sequence

    def run(self):
        self.order.clear()
        self.steps = iter(([dict(outputs=0, inputs=OPEN, home=True)] if self.during_reply
                           else self.script))
        self.node.placement.run(self.node)


@pytest.mark.parametrize('during_reply', [False, True])
def test_real_transport_queues_four_moves_with_exact_percentages_and_no_settling(during_reply):
    rig = QueueRig(during_reply=during_reply)
    rig.run()
    assert [name for name, _ in rig.requests] == ['MovL', 'MovLIO', 'MovLIO', 'MovL']
    assert rig.order[:4] == ['MovL', 'MovLIO', 'MovLIO', 'MovL']
    assert all(not request.mode for _, request in rig.requests)
    assert list(rig.requests[1][1].mdis) == [
        '{0,80,2,0}', '{0,80,14,1}', '{0,80,13,0}', '{0,80,1,1}']
    assert list(rig.requests[2][1].mdis) == [
        '{0,50,2,0}', '{0,50,14,0}', '{0,50,1,0}', '{0,50,13,0}']
    assert all(not any(v.startswith(('cp=', 'r=')) for v in request.param_value)
               for _, request in rig.requests)
    assert rig.node.placement.phase == 'DONE'
    assert not rig.node.holding_item
    assert rig.node.managed.session.attempts[0].state == 'PLACED'
    assert not any(rig.node.expected_outputs.values())
    completed = [call for call in rig.node.events.record.call_args_list
                 if call.args[1] == 'motion_batch_completed']
    assert completed[-1].kwargs['terminal_stable_sec'] == 0.


@pytest.mark.parametrize('inputs', [0, OPEN | 1])
def test_missing_open_or_released_suction_fails_without_separate_release_commands(inputs):
    rig = QueueRig()
    rig.script[1]['inputs'] = inputs
    with pytest.raises(FeedbackFailure, match='relaxed before'):
        rig.run()
    assert all(name != 'DO' for name, _ in rig.requests)
    assert rig.node.placement.needs_recovery
    assert rig.node.managed.session.held_index == 1


def test_skipped_exhaust_evidence_cannot_report_a_success():
    rig = QueueRig()
    rig.script.pop(1)
    with pytest.raises(FeedbackFailure, match='relaxed before'):
        rig.run()


def test_opposing_outputs_fail_during_queue():
    rig = QueueRig()
    rig.script[1]['outputs'] = HELD | RELEASE
    with pytest.raises(FeedbackFailure, match='Unexpected placement output'):
        rig.run()


def test_drop_before_release_is_not_treated_as_intentional_release():
    rig = QueueRig()
    rig.script = [dict(outputs=HELD, inputs=0)] * 8
    with pytest.raises(HeldSuctionLost, match='before queued placement release'):
        rig.run()


def test_stop_during_admission_prevents_later_requests_and_retains_context():
    rig = QueueRig()

    def cancel(index):
        if index == 2:
            rig.node.cancel_requested = lambda: True
    rig.on_request = cancel
    with pytest.raises(OperationCanceled):
        rig.run()
    assert [name for name, _ in rig.requests if name != 'Stop'] == ['MovL', 'MovLIO']
    assert rig.node.placement.needs_recovery


def test_idle_joint_arrival_completes_on_first_new_sample_without_pose_service():
    rig = QueueRig()
    rig.node.placement = None
    rig.node.holding_item = False
    rig.node.expected_outputs = {1: False, 2: False, 13: False, 14: False}
    rig.emit(outputs=0, inputs=0, home=True)
    rig.transport.current_pose = lambda: np.eye(4)
    rig.node.kinematics = SimpleNamespace(forward=lambda _joints: np.eye(4))
    rig.steps = iter([dict(outputs=0, inputs=0, home=True)])
    # Cartesian feedback deliberately differs; tray arrival uses actual joint topic.
    rig.monitor.update_joints(joint_message())
    rig.transport.move_batch((Target('tray_detect_position', np.eye(4), 80, 70,
                                     joints_rad=(0.,) * 6),), batch_name='tray_position')
    assert [name for name, _ in rig.requests] == ['MovL']
    assert rig.requests[0][1].mode
    assert next(rig.steps, None) is None


def test_stop_after_release_preserves_exhaust_without_waiting_for_a_pulse_end():
    from robot_controller.placement import place_targets
    from test_transport_v2 import CompletedFuture

    rig = QueueRig()
    node, operation = rig.node, rig.node.placement
    operation.plan = place_targets(node.configuration.tray.detect_matrix, [.3, .2, .25],
                                   node.configuration.profile, 0., node.configuration.home_matrix)
    operation.phase = 'APPROACH'
    operation.begin_queue(node)
    operation.issued(1)
    rig.emit(outputs=RELEASE, inputs=OPEN, running=0)

    def advancing_stationary(predicate, *_args, **_kwargs):
        assert not predicate(rig.monitor.snapshot(require_enabled=True))
        sample = rig.emit(outputs=RELEASE, inputs=OPEN, running=0)
        assert predicate(sample)
        return sample

    rig.monitor.wait = advancing_stationary
    rig.transport.confirm_stop(CompletedFuture())
    assert operation.release_confirmed and operation.needs_recovery
    assert node.expected_outputs == {1: True, 2: False, 13: False, 14: True}
    assert not rig.requests  # Stop confirmation never issues release/reset commands.


def test_ambiguous_partial_release_blocks_recovery_without_releasing_or_descending():
    from robot_controller.placement import place_targets

    rig = QueueRig()
    node, operation = rig.node, rig.node.placement
    operation.plan = place_targets(node.configuration.tray.detect_matrix, [.3, .2, .25],
                                   node.configuration.profile, 0., node.configuration.home_matrix)
    operation.phase = 'APPROACH'
    operation.begin_queue(node)
    operation.issued(1)
    sample = rig.emit(outputs=(1 << 12) | (1 << 13), inputs=1 | OPEN, running=0)
    operation.observe(node, sample)
    assert operation.phase == 'RELEASING'
    with pytest.raises(FeedbackFailure, match='release unconfirmed'):
        operation.run(node)
    assert not rig.requests


def test_release_requires_coherent_open_exhaust_and_sensor_evidence():
    rig = QueueRig()
    rig.script = [dict(outputs=1 << 13, inputs=OPEN),
                  dict(outputs=RELEASE, inputs=OPEN | 1),
                  dict(outputs=0, inputs=OPEN)]
    with pytest.raises(FeedbackFailure, match='relaxed before'):
        rig.run()
    assert rig.node.managed.session.attempts[0].state == 'HELD'
    assert not rig.node.placement.release_confirmed


@pytest.mark.parametrize('busy,wrong_joints', [(True, False), (False, True)])
def test_tray_arrival_requires_both_idle_and_actual_joint_tolerance(busy, wrong_joints):
    rig = QueueRig()
    rig.node.placement = None
    rig.emit(outputs=HELD, inputs=1, running=int(busy))
    joints = joint_message()
    if wrong_joints:
        joints.position[2] = np.deg2rad(2.)
    rig.monitor.update_joints(joints)
    assert not rig.transport.home_already_reached((0.,) * 6)
    assert not rig.requests
