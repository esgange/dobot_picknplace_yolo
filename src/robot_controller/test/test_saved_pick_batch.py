"""Drain saved bin candidates across placements without early source handoff."""

import copy
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from robot_controller.autorun import AutoRunOperation
from robot_controller.errors import FeedbackFailure, HeldSuctionLost, OperationCanceled
from robot_controller.motion import pick_targets
from robot_controller.pick_session import PickSession
from robot_controller.release import ReleaseQueue
from test_auto_run import cycle_rig, queue_rig, request
from test_automatic_return import action_rig
from test_continuous_drop import recovery_rig
from test_placement_queue import RELEASE, OPEN


def placement_completed(node):
    """Supply confirmed neutral terminal feedback to the real completion path."""
    node._transition('PLACING', 'Placement')
    node.lose_suction()
    node.feed['digital_outputs'] = 0
    ReleaseQueue(phase='RELEASED', release_confirmed=True).complete(node, node.snapshot())
    node._transition('READY', 'Placement completed')


def next_manual_pick(node):
    node._begin_operation('pick')
    return node.execute()


def manual_rig(monkeypatch, counts, acquisitions):
    node = action_rig(monkeypatch, losses=())
    node.hardware.acquisitions = iter(acquisitions)

    def detect(*_args, **_kwargs):
        assert not node.holding_item
        count = counts[len(node.requests)]
        node.requests.append('detect')
        batch = len(node.requests)
        return SimpleNamespace(identifier=f'batch{batch}', debug_message='', candidates=[
            SimpleNamespace(identifier=f'b{batch}:{i + 1}', position_m=(i * .1, 0., .3),
                            quaternion=(0., 0., 0., 1.)) for i in range(count)])
    node.candidates.request = detect
    return node


def test_manual_placements_drain_one_batch_then_request_again(monkeypatch):
    node = manual_rig(monkeypatch, [3, 1], [True] * 4)
    session = None
    for index in range(4):
        result = node.execute() if index == 0 else next_manual_pick(node)
        assert result.outcome == result.SUCCESS and result.attempted_candidates == 1
        assert result.selected_candidate_id == (f'b1:{index + 1}' if index < 3 else 'b2:1')
        assert len(node.requests) == (1 if index < 3 else 2)
        if index == 0:
            session = node.managed.session
        assert (node.managed.session is session) == (index < 3)
        placement_completed(node)
    assert [a.state for a in session.attempts] == ['PLACED'] * 3


def test_manual_missed_and_dropped_poses_stay_excluded_between_successes(monkeypatch):
    node = manual_rig(monkeypatch, [5], [False, True, True, True, True])
    node.lost = []  # Drop on candidate 3's held lift; recovery picks candidate 4.
    original = node.hardware.on_move

    def drop(targets, policy):
        original(targets, policy)
        if policy.get('require_suction') and node.managed.session.held_index == 3:
            if not node.lost:
                node.lost.append(3)
                node.lose_suction()
                raise HeldSuctionLost('synthetic confirmed drop')
    node.hardware.on_move = drop
    first = node.execute()
    assert first.selected_candidate_id == 'b1:2' and first.attempted_candidates == 2
    placement_completed(node)
    second = next_manual_pick(node)
    assert second.selected_candidate_id == 'b1:4' and second.attempted_candidates == 2
    placement_completed(node)
    third = next_manual_pick(node)
    assert third.selected_candidate_id == 'b1:5' and third.attempted_candidates == 1
    assert len(node.requests) == 1
    assert [a.state for a in node.managed.session.attempts] == [
        'FAILED', 'PLACED', 'DROPPED', 'PLACED', 'HELD']


def test_saved_batch_can_continue_from_manual_to_auto_and_back(monkeypatch):
    node = manual_rig(monkeypatch, [3], [True] * 3)
    assert node.execute().selected_candidate_id == 'b1:1'
    placement_completed(node)
    session = node.managed.session
    node._begin_operation('auto_run')
    run = AutoRunOperation(node, request(1))
    assert run._pick()
    assert session.held_index == 2 and node.managed.session is session
    placement_completed(node)
    node._end_operation()
    assert next_manual_pick(node).selected_candidate_id == 'b1:3'
    assert len(node.requests) == 1 and node.managed.session is session


def test_pause_before_reused_manual_pick_continues_saved_pose_without_home_detour(monkeypatch):
    node = manual_rig(monkeypatch, [2], [True, True])
    assert node.execute().selected_candidate_id == 'b1:1'
    placement_completed(node)
    node._begin_operation('pick')
    node.managed.request('pause')
    node._execute_home = Mock(side_effect=AssertionError('Continue uses its parked pose'))
    result = node.execute()
    assert result.outcome == result.SUCCESS and result.selected_candidate_id == 'b1:2'
    assert len(node.requests) == 1


@pytest.mark.parametrize('terminal', ['FAILED', 'DROPPED', 'PLACED', 'RETURNED'])
def test_saved_retry_skips_terminal_candidates_instead_of_reactivating_them(
        monkeypatch, terminal):
    node = manual_rig(monkeypatch, [4], [True, False, True])
    assert node.execute().selected_candidate_id == 'b1:1'
    placement_completed(node)
    session = node.managed.session
    session.set_state(3, 'ACTIVE')
    if terminal != 'FAILED':
        session.set_state(3, 'HELD')
    session.set_state(3, terminal)
    session.held_index = None
    result = next_manual_pick(node)
    assert result.outcome == result.SUCCESS and result.selected_candidate_id == 'b1:4'
    assert result.attempted_candidates == 2 and len(node.requests) == 1
    assert [a.state for a in session.attempts] == ['PLACED', 'FAILED', terminal, 'HELD']


@pytest.mark.parametrize('invalidation', ['reload', 'recover', 'source_error'])
def test_old_configuration_or_canceled_batch_cannot_supply_a_new_manual_pick(
        monkeypatch, invalidation):
    node = manual_rig(monkeypatch, [2, 1], [True, True])
    first = node.execute()
    assert first.outcome == first.SUCCESS
    placement_completed(node)
    old = node.managed.session
    if invalidation == 'reload':
        node.configuration = copy.copy(node.configuration)
    elif invalidation == 'recover':
        from robot_controller.recovery import HomeRecovery
        HomeRecovery.cancel_action(node)
    else:
        node.configuration.validate_sources = Mock(side_effect=FeedbackFailure('source changed'))
    result = next_manual_pick(node)
    if invalidation == 'source_error':
        assert result.outcome != result.SUCCESS and len(node.requests) == 1
    else:
        assert result.outcome == result.SUCCESS and len(node.requests) == 2
        assert node.managed.session is not old and result.selected_candidate_id == 'b2:1'


@pytest.mark.parametrize('quantity', [1, 2, 3, 4, 7])
@pytest.mark.parametrize('slow', [False, True])
def test_auto_run_reuses_saved_order_and_prefetches_only_after_batch_exhaustion(
        monkeypatch, quantity, slow):
    run, node, order, workers = cycle_rig(monkeypatch, quantity, slow=slow)
    sessions, picked = [], []

    def pick(batch, bridge):
        session = node.managed.session
        if session is None or batch is not None:
            assert session is None or session.next_eligible is None
            identifiers = [f'b{len(sessions) + 1}:{i}' for i in range(1, 4)]
            session = PickSession(identifiers, [()] * 3, configuration=node.configuration,
                                  batch=SimpleNamespace(identifier=identifiers[0]))
            sessions.append(session)
        else:
            assert session.reusable(node.configuration)
        if bridge is not None:
            bridge.next_session = session
            order.append('append next Pick')
            bridge.home_id = 4
            bridge._complete()
        else:
            node.managed.session = session
            node._transition('PICKING', 'Pick')
            order.append('Pick')
        index = session.next_eligible
        session.set_state(index, 'ACTIVE')
        session.set_state(index, 'HELD')
        picked.append(session.attempts[index - 1].identifier)
        node._transition('HOLDING', 'Tray Detect reached')
        return True

    run._pick = pick
    finish = node.hardware.finish_batch

    def finish_placement(pending, *, handoff):
        if node.managed.session.reusable(node.configuration):
            assert handoff() and run.prefetch is None
        else:
            finish(pending, handoff=handoff)
    node.hardware.finish_batch = finish_placement
    assert run.run() and run.completed == quantity
    assert len(workers) == (quantity - 1) // 3
    assert picked == [f'b{i // 3 + 1}:{i % 3 + 1}' for i in range(quantity)]
    assert [a.state for s in sessions for a in s.attempts] == (
        ['PLACED'] * quantity + ['PENDING'] * (len(sessions) * 3 - quantity))
    assert all(worker.closed for worker in workers)
    assert order[-1] == 'append final Home'


def shared_queue():
    rig, run, bridge = queue_rig()
    node = rig.node
    node.configuration.profile['gripper'] = {'use_grip': False, 'grip_onpick': False}
    plans = []
    for index in (1, 2, 3):
        pose = np.eye(4)
        pose[:3, 3] = [.4 + index * .02, .1, .25]
        plans.append(pick_targets(node.configuration.home_matrix, pose,
                                  node.configuration.profile, index))
    candidates = [SimpleNamespace(identifier=f'item{i}') for i in (1, 2, 3)]
    batch = SimpleNamespace(identifier='original', candidates=candidates)
    session = PickSession([c.identifier for c in candidates], plans,
                          batch=batch, configuration=node.configuration)
    session.set_state(1, 'ACTIVE')
    session.set_state(1, 'HELD')
    node.managed.session = bridge.old_session = session
    node.candidates = SimpleNamespace(request=Mock(side_effect=AssertionError('No detection')))
    node._plan_candidate_batch = Mock(side_effect=AssertionError('Do not replan saved poses'))
    node._execute_home = Mock(side_effect=AssertionError('Home must be queued, not awaited'))
    node._candidate_progress = Mock()
    node._attempt_changed = Mock()
    run.seen_batches.add('original')
    pending, node.placement.pending_motion = node.placement.pending_motion, None
    node.hardware.finish_batch(pending, handoff=lambda: True)
    return rig, run, bridge, session


def test_real_transport_appends_saved_pick_before_placement_arrival_and_defers_ownership():
    rig, run, bridge, session = shared_queue()
    node = rig.node
    move = node.hardware.move_batch

    def dispatch(targets, **kwargs):
        if kwargs['batch_name'] == 'candidate_2_pick_to_tray':
            raise OperationCanceled('Reached saved candidate 2')
        assert session.held_index == 1 and session.attempts[0].state == 'HELD'
        assert [t.name for t in targets] == ['home', 'p2_transit', 'p2_prepick', 'p2_pick']
        return move(targets, **kwargs)
    node.hardware.move_batch = dispatch

    def admission(index):
        if index <= 7:
            assert session.held_index == 1 and session.attempts[1].state == 'PENDING'
    rig.on_request = admission
    rig.steps = iter([
        dict(outputs=RELEASE, inputs=OPEN, currentCommandId=2),
        dict(outputs=0, inputs=OPEN, currentCommandId=3),
        dict(outputs=0, inputs=OPEN, currentCommandId=4),
        dict(outputs=(1 << 12) | (1 << 13), inputs=1, currentCommandId=7),
    ])
    with pytest.raises(OperationCanceled, match='Reached saved'):
        run._pick(None, bridge)
    assert run.completed == 1 and node.managed.session is session
    assert [a.state for a in session.attempts] == ['PLACED', 'HELD', 'PENDING']
    assert session.held_index == 2
    assert rig.order[:7] == [name for name, _ in rig.requests[:7]]
    node.candidates.request.assert_not_called()
    run.close()
    assert session.attempts[2].state == 'PENDING'


def test_drop_before_shared_placement_boundary_keeps_remaining_candidates():
    node = recovery_rig()
    session = node.managed.session
    run = node.auto_run = AutoRunOperation(node, request(2))
    run.bridge = SimpleNamespace(old_session=session, next_session=session)
    run._run_cycles = Mock(side_effect=[HeldSuctionLost('lost before release'), True])
    assert run.run()
    assert node.managed.session is session and run.completed == 0
    assert [a.state for a in session.attempts] == ['DROPPED', 'HELD', 'PENDING']
