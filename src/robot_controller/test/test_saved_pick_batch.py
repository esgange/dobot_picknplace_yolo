"""Refresh after placement; preserve original candidates for retries and returns."""

import copy
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from robot_controller.autorun import AutoRunOperation
from robot_controller.controller import RobotController
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
    node._plan_candidate_batch = lambda batch: RobotController._plan_candidate_batch(node, batch)

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


def test_manual_placement_discards_remaining_poses_and_next_pick_requests_fresh(monkeypatch):
    node = manual_rig(monkeypatch, [3, 3, 3, 3], [True] * 4)
    sessions = []
    for index in range(4):
        result = node.execute() if index == 0 else next_manual_pick(node)
        assert result.outcome == result.SUCCESS and result.attempted_candidates == 1
        assert result.selected_candidate_id == f'b{index + 1}:1'
        assert len(node.requests) == index + 1
        session = node.managed.session
        assert all(session is not previous for previous in sessions)
        sessions.append(session)
        assert session.reusable(node.configuration)  # Available for held-drop recovery.
        placement_completed(node)
        assert not session.reusable(node.configuration) and session.held_index is None
        assert [a.state for a in session.attempts] == ['PLACED', 'CANCELED', 'CANCELED']


@pytest.mark.parametrize('blocked', ['suction', 'outputs'])
def test_unconfirmed_placement_does_not_invalidate_recovery_poses(monkeypatch, blocked):
    node = manual_rig(monkeypatch, [3], [True])
    result = node.execute()
    assert result.outcome == result.SUCCESS
    session = node.managed.session
    node.feed['digital_input_bits'] = 1 if blocked == 'suction' else 0
    node.feed['digital_outputs'] = 1 if blocked == 'outputs' else 0
    with pytest.raises(FeedbackFailure, match='neutral and DI1 LOW'):
        ReleaseQueue().complete(node, node.snapshot())
    assert session.held_index == 1 and session.reusable(node.configuration)
    assert [a.state for a in session.attempts] == ['HELD', 'PENDING', 'PENDING']


def test_misses_and_drops_retry_original_batch_until_successful_placement(monkeypatch):
    node = manual_rig(monkeypatch, [5, 2], [False, True, True, True])
    original = node.hardware.on_move

    def drop(targets, policy):
        original(targets, policy)
        if policy.get('require_suction') and node.managed.session.held_index == 2:
            if not node.lost:
                node.lost.append(2)
                node.lose_suction()
                raise HeldSuctionLost('synthetic confirmed drop')
    node.hardware.on_move = drop
    first = node.execute()
    assert first.selected_candidate_id == 'b1:3' and first.attempted_candidates == 3
    session = node.managed.session
    assert len(node.requests) == 1
    assert [a.state for a in session.attempts] == [
        'FAILED', 'DROPPED', 'HELD', 'PENDING', 'PENDING']
    placement_completed(node)
    assert [a.state for a in session.attempts] == [
        'FAILED', 'DROPPED', 'PLACED', 'CANCELED', 'CANCELED']
    second = next_manual_pick(node)
    assert second.selected_candidate_id == 'b2:1' and second.attempted_candidates == 1
    assert len(node.requests) == 2


def test_successful_placement_refreshes_between_manual_and_auto_picks(monkeypatch):
    node = manual_rig(monkeypatch, [3, 3, 3], [True] * 3)
    assert node.execute().selected_candidate_id == 'b1:1'
    placement_completed(node)
    session = node.managed.session
    node._begin_operation('auto_run')
    run = AutoRunOperation(node, request(1))
    assert run._pick()
    assert node.managed.session is not session and node.managed.session.held_index == 1
    assert node.managed.session.attempts[0].identifier == 'b2:1'
    placement_completed(node)
    node._end_operation()
    assert next_manual_pick(node).selected_candidate_id == 'b3:1'
    assert len(node.requests) == 3


def item_returned(node):
    """Explicit return keeps unattempted poses; tray placement invalidation does not apply."""
    from robot_controller.item_return import ItemReturnOperation
    node._transition('RETURNING_ITEM', 'Item return')
    node.lose_suction()
    node.feed['digital_outputs'] = 0
    ItemReturnOperation(index=node.managed.session.held_index, phase='RELEASED',
                        release_confirmed=True).complete(node, node.snapshot())
    node._transition('READY', 'Item returned')


def test_pause_before_pick_after_return_retries_saved_pose_without_home_detour(monkeypatch):
    node = manual_rig(monkeypatch, [2], [True, True])
    assert node.execute().selected_candidate_id == 'b1:1'
    item_returned(node)
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
    item_returned(node)
    session = node.managed.session
    session.set_state(3, 'ACTIVE')
    if terminal != 'FAILED':
        session.set_state(3, 'HELD')
    session.set_state(3, terminal)
    session.held_index = None
    result = next_manual_pick(node)
    assert result.outcome == result.SUCCESS and result.selected_candidate_id == 'b1:4'
    assert result.attempted_candidates == 2 and len(node.requests) == 1
    assert [a.state for a in session.attempts] == ['RETURNED', 'FAILED', terminal, 'HELD']


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
def test_auto_run_refreshes_every_successful_placement_even_with_unused_poses(
        monkeypatch, quantity, slow):
    run, node, order, workers = cycle_rig(monkeypatch, quantity, slow=slow)
    sessions, picked = [], []

    def pick(batch, bridge):
        if sessions:
            assert batch is not None  # Never reuse leftover poses after a placement.
        identifiers = [f'b{len(sessions) + 1}:{i}' for i in range(1, 4)]
        session = PickSession(identifiers, [()] * 3, configuration=node.configuration,
                              batch=SimpleNamespace(identifier=identifiers[0]))
        sessions.append(session)
        if bridge is not None and not bridge.completed:
            assert bridge.old_session is sessions[-2]
            assert bridge.old_session.held_index == 1
            assert bridge.old_session.attempts[1].state == 'PENDING'
            bridge.next_session = session
            order.append('append next Pick')
            bridge.boundary_id = 4
            bridge._complete()
        else:
            node.managed.session = session
            node._transition('PICKING', 'Pick')
            order.append('Pick')
        assert len(sessions) == 1 or not sessions[-2].reusable(node.configuration)
        session.set_state(1, 'ACTIVE')
        session.set_state(1, 'HELD')
        picked.append(identifiers[0])
        node._transition('HOLDING', 'Tray Detect reached')
        return True

    run._pick = pick
    assert run.run() and run.completed == quantity
    assert len(workers) == quantity - 1
    assert picked == [f'b{i + 1}:1' for i in range(quantity)]
    assert all([a.state for a in session.attempts] == ['PLACED', 'CANCELED', 'CANCELED']
               for session in sessions)
    assert all(worker.closed for worker in workers)
    assert order[-1] == 'append final Home'


def shared_queue(*, slow=False):
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
    node.motion_admitted = lambda target: RobotController.motion_admitted(node, target)
    node._attempt_changed = Mock()
    run.seen_batches.add('original')
    pending, node.placement.pending_motion = node.placement.pending_motion, None
    if slow:
        rig.steps = iter([dict(outputs=0, inputs=OPEN, retract=True)])
    node.hardware.finish_batch(pending, handoff=lambda: not slow)
    if slow:
        bridge.complete_idle()
        node.hardware.current_pose = lambda: bridge.origin.copy()
    return rig, run, bridge, session


def test_final_home_boundary_discards_unused_poses_only_after_confirmed_placement():
    rig, run, bridge, session = shared_queue()

    def admission(_index):
        assert session.attempts[0].state == 'HELD'
        assert session.attempts[1].state == 'PENDING'
    rig.on_request = admission
    rig.steps = iter([
        dict(outputs=RELEASE, inputs=OPEN, currentCommandId=2),
        dict(outputs=0, inputs=OPEN, currentCommandId=3),
        dict(outputs=0, inputs=OPEN, currentCommandId=4),
        dict(outputs=0, inputs=OPEN, home=True),
    ])
    run.finish_home(bridge)
    assert run.completed == 1 and session.held_index is None
    assert [a.state for a in session.attempts] == ['PLACED', 'CANCELED', 'CANCELED']
    assert not session.reusable(rig.node.configuration)
    rig.node.candidates.request.assert_not_called()


@pytest.mark.parametrize("slow", [False, True])
def test_fresh_batch_skips_home_before_or_after_placement_finishes(slow):
    rig, run, bridge, old = shared_queue(slow=slow)
    node = rig.node
    pose = np.eye(4)
    pose[:3, 3] = [.2, .2, .25]
    plan = pick_targets(node.configuration.home_matrix, pose, node.configuration.profile, 1)
    batch = SimpleNamespace(identifier='fresh', candidates=[SimpleNamespace(identifier='fresh:1')])
    node._plan_candidate_batch = Mock(return_value=[plan])
    move = node.hardware.move_batch

    def dispatch(targets, **kwargs):
        if kwargs['batch_name'] == 'candidate_1_pick_to_tray':
            raise OperationCanceled('Reached fresh candidate')
        if not slow:
            assert node.managed.session is old and old.held_index == 1
        assert [t.name for t in targets] == ['p1_transit', 'p1_prepick', 'p1_pick']
        assert not any(t.joint_motion for t in targets)
        return move(targets, **kwargs)
    node.hardware.move_batch = dispatch

    def admission(index):
        if not slow and index <= 6:
            assert old.held_index == 1 and old.attempts[1].state == 'PENDING'
    rig.on_request = admission
    rig.steps = iter([
        dict(outputs=RELEASE, inputs=OPEN, currentCommandId=2),
        dict(outputs=0, inputs=OPEN, currentCommandId=3),
        dict(outputs=0, inputs=OPEN, currentCommandId=4),
        dict(outputs=(1 << 12) | (1 << 13), inputs=1, currentCommandId=7),
    ])
    with pytest.raises(OperationCanceled, match='Reached fresh'):
        run._pick(batch, bridge)
    assert run.completed == 1
    if not slow:
        assert node.managed.session is bridge.next_session
    assert node.managed.session.batch is batch
    assert node.managed.session.attempts[0].identifier == 'fresh:1'
    assert node.managed.session.attempts[0].state == 'HELD'
    assert [a.state for a in old.attempts] == ['PLACED', 'CANCELED', 'CANCELED']
    if not slow:
        assert rig.order[:6] == [name for name, _ in rig.requests[:6]]
    assert not any(name == 'MovJ' for name, _ in rig.requests)
    node._execute_home.assert_not_called()
    node.candidates.request.assert_not_called()
    run.close()


def test_drop_before_shared_placement_boundary_keeps_remaining_candidates():
    node = recovery_rig()
    session = node.managed.session
    run = node.auto_run = AutoRunOperation(node, request(2))
    run.bridge = SimpleNamespace(old_session=session, next_session=session)
    run._run_cycles = Mock(side_effect=[HeldSuctionLost('lost before release'), True])
    assert run.run()
    assert node.managed.session is session and run.completed == 0
    assert [a.state for a in session.attempts] == ['DROPPED', 'HELD', 'PENDING']
