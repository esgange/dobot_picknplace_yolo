"""Placement exercises the real transport and feedback; no ROS/hardware calls."""

from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.task import Future

from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, OperationCanceled
from robot_controller.hardware import DobotTransport
from robot_controller.kinematics import pose_values
from robot_controller.motion import Target
from robot_controller.placement import place_targets
from test_feedback_v2 import feed, primed_monitor, joint_message
from test_placement import operation_node


HELD = (1 << 12) | 2
RELEASE = 1  # Exhaust ON, fingers relaxed, suction OFF.
OPEN = 1 << 11


class QueueRig:
    def __init__(self, *, during_reply=False):
        self.node = node = operation_node()
        self.monitor = node.monitor = primed_monitor()
        self.joint_poses = {}
        node.kinematics = SimpleNamespace(forward=lambda joints: self.joint_poses[tuple(joints)])
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
            dict(outputs=0, inputs=OPEN, retract=True),
        ]

    def emit(self, *, outputs, inputs, home=False, retract=False, drop=False,
             running=1, **feedback):
        self.timer += 1
        self.clock += .01
        matrix = (self.node.configuration.home_matrix if home
                  else self.node.configuration.tray.detect_matrix)
        if drop:
            owner = self.node.managed.return_progress or self.node.placement
            matrix = owner.plan[owner.release_index].matrix
        if retract:
            matrix = place_targets(
                self.node.configuration.tray.detect_matrix, [.3, .2, .25],
                self.node.configuration.profile, self.node.placement.rotation_deg,
                self.node.configuration.home_matrix)[-1].matrix
        fields = dict(
            controller_timer=self.timer, digital_outputs=outputs, digital_input_bits=inputs,
            currentCommandId=4 if home else 3 if retract else 2,
            RunningStatus=0 if home or retract or drop else running,
            isRunQueuedCmd=0 if home or retract or drop else running,
            tool_vector_actual=pose_values(matrix))
        fields.update(feedback)
        # The fake robot publishes independent joint/status streams. Its test
        # model maps those joints to the scripted pose; production uses CR10 FK.
        from robot_controller.kinematics import pose_matrix
        matrix = pose_matrix(fields['tool_vector_actual'])
        joints = joint_message()
        if home and 'tool_vector_actual' not in feedback:
            joints.position = list(getattr(self.node.configuration, 'home_joints', (0.,) * 6))
        elif not home and not retract and not drop and 'tool_vector_actual' not in feedback:
            joints.position = list(self.node.configuration.tray.detect_joints)
        else:
            joints.position = list(matrix[:3, 3]) + [0., 0., 0.]
        self.joint_poses[tuple(joints.position)] = matrix.copy()
        self.monitor.update_joints(joints)
        self.monitor.update_status(SimpleNamespace(
            is_connected=True, is_enable=not (fields['RunningStatus'] or fields['isRunQueuedCmd'])))
        self.monitor.update_feed(feed(**fields))
        self.order.append('feedback')
        return self.monitor.snapshot(require_enabled=True)

    def client(self, kind, endpoint):
        def call(request):
            name = endpoint.rsplit('/', 1)[-1]
            self.requests.append((name, request))
            self.order.append(name)
            self.on_request(len(self.requests))
            if len(self.requests) == 3 and self.during_reply:
                for values in self.script:
                    self.emit(**values)
            future = Future()
            future.set_result(SimpleNamespace(res=0, robot_return='{4}'))
            return future
        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)

    def next_sample(self, *_args, **_kwargs):
        values = next(self.steps, None)
        if values is None:
            pytest.fail('Unexpected feedback wait after final retract')
        self.emit(**values)
        return self.monitor.sequence

    def run(self):
        self.order.clear()
        self.steps = iter(([dict(outputs=0, inputs=OPEN, retract=True)] if self.during_reply
                           else self.script))
        self.node.placement.run(self.node)
        self.node.placement.finish_pending(self.node)


@pytest.mark.parametrize('during_reply', [False, True])
@pytest.mark.parametrize('approach_speed', [6, 17])
def test_real_transport_queues_three_moves_with_exact_percentages_and_no_settling(
        during_reply, approach_speed):
    rig = QueueRig(during_reply=during_reply)
    rig.node.configuration.profile["motion"]["trayplace_height"] = 42.5
    rig.node.configuration.profile["speed"]["approach_percent"] = approach_speed
    rig.run()
    assert [name for name, _ in rig.requests] == ['MovL', 'MovLIO', 'MovLIO']
    assert rig.order[:3] == ['MovL', 'MovLIO', 'MovLIO']
    assert all(not request.mode for _, request in rig.requests)
    assert [list(request.param_value) for _, request in rig.requests] == [
        ['user=0', 'tool=0', f'v={speed}', f'a={acceleration}']
        for speed, acceleration in ((100, 70), (100, 30), (100, 40))]
    assert [request.c for _, request in rig.requests] == pytest.approx([800., 292.5, 800.])
    assert all((request.a, request.b) == pytest.approx((300., 200.))
               for _, request in rig.requests[:3])
    assert list(rig.requests[1][1].mdis) == [
        '{0,90,2,0}', '{0,90,14,0}', '{0,90,13,0}', '{0,100,1,1}']
    assert list(rig.requests[2][1].mdis) == [
        '{1,0,2,0}', '{1,0,14,0}', '{1,0,1,0}', '{1,0,13,0}']
    assert all(not any(v.startswith(('cp=', 'r=')) for v in request.param_value)
               for _, request in rig.requests)
    assert rig.node.placement.phase == 'DONE'
    assert not rig.node.holding_item
    assert rig.node.managed.session.attempts[0].state == 'PLACED'
    assert not any(rig.node.expected_outputs.values())
    completed = [call for call in rig.node.events.record.call_args_list
                 if call.args[1] == 'motion_batch_completed']
    assert completed[-1].kwargs['terminal_stable_sec'] == 0.


@pytest.mark.parametrize('inputs', [0, 1, OPEN, OPEN | 1])
@pytest.mark.parametrize('during_reply', [False, True])
def test_release_evidence_uses_di1_without_requiring_open_sensor(inputs, during_reply):
    rig = QueueRig(during_reply=during_reply)
    rig.script[1]['inputs'] = inputs
    rig.run()
    assert all(name != 'DO' for name, _ in rig.requests)
    assert rig.node.placement.phase == 'DONE'
    assert rig.node.placement.release_confirmed == (not bool(inputs & 1))
    assert rig.node.managed.session.held_index is None
    assert rig.node.managed.session.attempts[0].state == 'PLACED'
    finished = [c for c in rig.node.events.record.call_args_list
                if c.args[1] == 'placement_retract_completed']
    assert finished[-1].kwargs['release_feedback_observed'] == (not bool(inputs & 1))


def test_skipped_exhaust_evidence_does_not_block_retract():
    rig = QueueRig()
    rig.script.pop(1)
    rig.run()
    assert rig.node.placement.phase == 'DONE'
    assert not rig.node.placement.release_confirmed


def test_expired_output_history_during_admission_does_not_block_retract():
    rig = QueueRig(during_reply=True)
    rig.monitor._outputs = deque(maxlen=1)
    rig.run()
    assert rig.node.placement.phase == 'DONE'
    assert not rig.node.placement.release_confirmed


@pytest.mark.parametrize('fault', [
    {'ErrorStatus': 1}, {'CollisionStates': 1}, {'EnableStatus': 0},
])
def test_robot_faults_still_interrupt_placement(fault):
    rig = QueueRig()
    rig.script[1].update(fault)
    with pytest.raises(FeedbackFailure):
        rig.run()
    assert rig.node.placement.phase != 'DONE'
    assert rig.node.managed.session.attempts[0].state == 'HELD'


def test_opposing_outputs_fail_during_queue():
    rig = QueueRig()
    rig.script[1]['outputs'] = HELD | RELEASE
    with pytest.raises(FeedbackFailure, match='Invalid vacuum state'):
        rig.run()


def test_suction_loss_during_queue_does_not_block_retract_or_claim_release_evidence():
    rig = QueueRig()
    rig.script = ([dict(outputs=HELD, inputs=0)] * 8
                  + [dict(outputs=0, inputs=0, retract=True)])
    rig.run()
    assert rig.node.placement.phase == 'DONE'
    assert not rig.node.placement.release_confirmed


def test_renewed_suction_after_release_is_checked_only_at_retract():
    rig = QueueRig()
    rig.script[2]['inputs'] = 1
    rig.run()
    assert rig.node.placement.phase == 'DONE'


@pytest.mark.parametrize('outputs,inputs', [(0, 1), (RELEASE, 0), (HELD, 1)])
def test_bad_final_grip_is_reported_at_retract_without_an_intermediate_stop(outputs, inputs):
    rig = QueueRig()
    rig.script[-1].update(outputs=outputs, inputs=inputs)
    with pytest.raises(FeedbackFailure, match='Retract reached; final placement'):
        rig.run()
    assert [name for name, _ in rig.requests] == ['MovL', 'MovLIO', 'MovLIO']
    assert rig.monitor.snapshot().feed['tool_vector_actual'] == pose_values(
        rig.node.placement.plan[-1].matrix)
    assert rig.node.managed.session.attempts[0].state == 'HELD'


@pytest.mark.parametrize('feedback', [
    {'tool_vector_actual': [0.] * 6}, {'isRunQueuedCmd': 1, 'RunningStatus': 1},
])
def test_placement_still_requires_physical_idle_retract(feedback):
    rig = QueueRig()
    rig.script.insert(-1, dict(outputs=0, inputs=0, retract=True, **feedback))
    rig.run()
    assert next(rig.steps, None) is None  # Must consume the later genuine retract sample.
    assert rig.node.placement.phase == 'DONE'


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
    assert rig.node.placement.phase == 'RELEASING'
    # An issued-but-unobserved release is never automatically repeated.
    with pytest.raises(FeedbackFailure, match='release unconfirmed'):
        rig.node.placement.run(rig.node)
    assert [name for name, _ in rig.requests if name != 'Stop'] == ['MovL', 'MovLIO']


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
    assert node.expected_outputs == {1: True, 2: False, 13: False, 14: False}
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


def test_incoherent_release_evidence_is_diagnostic_only_until_retract():
    rig = QueueRig()
    rig.script = [dict(outputs=1 << 13, inputs=OPEN),
                  dict(outputs=RELEASE, inputs=OPEN | 1),
                  dict(outputs=0, inputs=OPEN),
                  dict(outputs=0, inputs=0, retract=True)]
    rig.run()
    assert rig.node.managed.session.attempts[0].state == 'PLACED'
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


@pytest.mark.parametrize('start_z', [.1, .8, 1.2])
@pytest.mark.parametrize('holding', [False, True])
def test_controller_queues_direct_tray_position_without_safety_z(start_z, holding):
    rig = QueueRig()
    node = rig.node
    node.holding_item = holding
    outputs, inputs = (HELD, 1) if holding else (0, 0)
    node.expected_outputs = {ch: bool(outputs & (1 << (ch - 1))) for ch in (1, 2, 13, 14)}
    rig.emit(outputs=outputs, inputs=inputs, running=0)
    joints = joint_message()
    joints.position[0] = .2  # Not already at the saved joints.
    rig.monitor.update_joints(joints)
    current = node.configuration.tray.detect_matrix.copy()
    current[:3, 3] = [.1, -.1, start_z]
    rig.transport.current_pose = lambda: current.copy()
    node.kinematics = SimpleNamespace(forward=lambda _j: node.configuration.tray.detect_matrix)
    rig.steps = iter([dict(outputs=outputs, inputs=inputs, home=True)])

    def arrived(*args, **kwargs):
        rig.monitor.update_joints(joint_message())
        return rig.next_sample(*args, **kwargs)

    rig.monitor.wait_next = arrived
    RobotController._execute_tray_position(node)
    assert [name for name, _ in rig.requests] == ['MovL']
    request = rig.requests[0][1]
    assert request.mode  # Linear move to the exact recorded joint target.
    assert [request.a, request.b, request.c, request.d, request.e, request.f] == [0.] * 6
    assert list(request.param_value) == ['user=0', 'tool=0', 'v=100', 'a=70']
    node.configuration.validate_sources.assert_called()
    assert node.expected_outputs == {ch: bool(outputs & (1 << (ch - 1)))
                                     for ch in (1, 2, 13, 14)}


def test_controller_skips_tray_move_when_idle_at_saved_joints():
    rig = QueueRig()
    RobotController._execute_tray_position(rig.node)
    assert not rig.requests
