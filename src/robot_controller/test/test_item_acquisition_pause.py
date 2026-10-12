"""Operator refill waits retain ownership and revalidate before a fresh Pick."""

from types import SimpleNamespace
from unittest.mock import Mock
import threading

import numpy as np
import pytest

from robot_controller.autorun import AutoRunOperation
from robot_controller.candidates import CandidateBatch
from robot_controller.controller import RobotController
from robot_controller.errors import FeedbackFailure, OperationCanceled, StopUnconfirmed
from robot_controller.operator_ui import presentation
from robot_controller_interfaces.srv import Command
from test_auto_run import request
from test_operation_retries import pick_rig
from test_status_ui import status, window  # noqa: F401
from test_trickle_depth import configure


def history(node):
    node.repick = configure(node.configuration, trigger=1)
    node.repick.completed((9., 9., 0.), 'old-bin')
    detect = node.candidates.request

    def batch(*args, **kwargs):
        result = detect(*args, **kwargs)
        return CandidateBatch(result.identifier, 1, 1, tuple(result.candidates), {}, 'OFF')
    node.candidates.request = batch
    return node.repick


@pytest.mark.parametrize('rounds', [1, 2])
def test_manual_continue_waits_without_detection_then_grants_fresh_budget(monkeypatch, rounds):
    node = pick_rig(monkeypatch, [0] * (10 * rounds) + [1], [True])
    h = history(node)
    before = h.snapshot()
    waits = []

    def choose():
        assert node.managed.item_waiting and node.machine.state == 'PAUSED'
        assert node.operation_lock.locked() and node.finished == []
        assert np.allclose(node.hardware.current_pose(), node.configuration.home_matrix)
        assert not node.holding_item and node.feed['digital_outputs'] == 0
        assert len(node.requests) == 10 * (len(waits) + 1)
        assert node.managed.continue_block_reason(node.snapshot()) == ''
        if not waits:
            assert h.snapshot() == before  # No reset merely from exhausting or waiting.
        old_count = len(node.requests)
        for _ in range(3):
            node.managed._check_parked(node.snapshot())
        assert len(node.requests) == old_count
        # A distinct service thread must acquire the managed lock while the
        # action owner is waiting. Holding it across the loop would deadlock.
        replies = []
        worker = threading.Thread(target=lambda: replies.append(
            RobotController._continue(node, None, Command.Response())))
        worker.start()
        worker.join(1.)
        assert not worker.is_alive() and replies[0].success
        waits.append(True)
    node.on_wait = choose
    result = node.execute()
    assert result.outcome == result.SUCCESS and result.final_state == 'HOLDING'
    assert len(waits) == rounds and len(node.requests) == 10 * rounds + 1
    assert not h.blocked((9., 9., 0.)) and h.prior_attempts((9., 9., 0.))[0] == 0
    assert node.finished == ['success'] and not node.managed.item_waiting


@pytest.mark.parametrize('cause', ['blacklist', 'pick_batches', 'trickle'])
def test_other_no_pick_paths_wait_and_continue_with_fresh_poses(monkeypatch, cause):
    counts = [1, 1, 1, 1] if cause == 'pick_batches' else [0, 1]
    node = pick_rig(monkeypatch, counts, [False, False, False, True]
                    if cause == 'pick_batches' else [True])
    if cause == 'blacklist':
        detect = node.candidates.request

        def excluded(*args, **kwargs):
            batch = detect(*args, **kwargs)
            batch.blacklist_only = len(node.requests) == 1
            return batch
        node.candidates.request = excluded
    elif cause == 'trickle':
        # Planning rejects every candidate before motion; retain the real wait.
        original = node.managed.run_pick

        def no_safe(*args, **kwargs):
            if len(node.requests) == 1:
                node.managed.session.trickle_rejections = {1: 'unsafe'}
                node.managed.session.prepare_trickle = lambda _index: False
                return dict(picked=False, holding_item=False)
            return original(*args, **kwargs)
        counts[0] = 1
        node.managed.run_pick = no_safe
    pauses = []

    def choose():
        pauses.append(len(node.requests))
        node.managed.continue_operation()
    node.on_wait = choose
    result = node.execute()
    assert result.outcome == result.SUCCESS
    assert pauses == [3 if cause == 'pick_batches' else 1]
    assert result.attempted_candidates == (4 if cause == 'pick_batches' else 1)


def test_auto_wait_preserves_95_count_and_restarts_only_after_continue(monkeypatch):
    node = pick_rig(monkeypatch, [0] * 10 + [1], [True])
    node.managed.session = None
    node._plan_candidate_batch = lambda batch: RobotController._plan_candidate_batch(node, batch)
    h = history(node)
    run = node.auto_run = AutoRunOperation(node, request(100))
    run.completed = 95
    old_pick = run._pick
    picks = []

    def pick(*args):
        result = old_pick(*args)
        picks.append(result)
        if result:
            raise OperationCanceled('Verified fresh next pickup; end synthetic test before Place')
        return result
    run._pick = pick

    def choose():
        assert node.managed.item_waiting and run.completed == 95
        assert node.operation_lock.locked() and run.prefetch is None
        assert len(node.requests) == 10 and h.snapshot()[1]
        assert RobotController._continue(node, None, Command.Response()).success
    node.on_wait = choose
    with pytest.raises(OperationCanceled, match='Verified fresh'):
        run.run()
    assert picks == [False, True] and run.completed == 95
    assert len(node.requests) == 11 and not h.blocked((9., 9., 0.))


@pytest.mark.parametrize('failure', [
    'stop', 'stop_after_continue', 'pose', 'outputs', 'motion', 'di1', 'stale', 'source'])
def test_item_wait_stop_and_faults_never_clear_history_or_detect_again(monkeypatch, failure):
    node = pick_rig(monkeypatch, [0] * 10, [])
    h = history(node)
    before = h.snapshot()

    def fail():
        if failure == 'stop_after_continue':
            assert RobotController._continue(node, None, Command.Response()).success
        if failure.startswith('stop'):
            node.cancel_event.set()
        elif failure == 'pose':
            node.feed['tool_vector_actual'][0] += 20.
        elif failure == 'outputs':
            node.feed['digital_outputs'] ^= 2
        elif failure == 'motion':
            node.feed['RunningStatus'] = 1
        elif failure == 'di1':
            node.set_di1(True)
        elif failure == 'stale':
            node.monitor.snapshot = Mock(side_effect=FeedbackFailure('stale feedback'))
        else:
            node.configuration.validate_sources = Mock(
                side_effect=FeedbackFailure('source changed'))
            reply = RobotController._continue(node, None, Command.Response())
            assert not reply.success
            node.cancel_event.set()
    node.on_wait = fail
    result = node.execute()
    assert result.outcome != result.SUCCESS
    assert len(node.requests) == 10 and h.snapshot() == before
    assert not node.managed.item_waiting and not node.operation_lock.locked()


@pytest.mark.parametrize('failure', ['home', 'reply', 'stop'])
def test_item_wait_is_not_exposed_until_home_responses_and_stop_pass(monkeypatch, failure):
    node = pick_rig(monkeypatch, [0] * 10, [])
    node._execute_home()
    node._transition('READY', 'test')
    if failure == 'home':
        node.hardware.home_already_reached = lambda _: False
    elif failure == 'reply':
        node.hardware.ensure_no_pending_response = Mock(
            side_effect=FeedbackFailure('pending reply'))
    else:
        node.hardware.confirm_stop = Mock(side_effect=StopUnconfirmed('Stop failed'))
    with pytest.raises((FeedbackFailure, StopUnconfirmed)):
        node.managed.wait_for_items('No items')
    assert ('state', 'PAUSED') not in node.log and not node.managed.item_waiting
    assert not node.requests


def test_missing_item_provider_blocks_continue_without_reset(monkeypatch):
    node = pick_rig(monkeypatch, [0] * 10, [])
    h = history(node)
    before = h.snapshot()

    def choose():
        node._perception_ready = lambda _: False
        response = RobotController._continue(node, None, Command.Response())
        assert not response.success and 'Item' in response.message
        assert not node.managed.resume.is_set()
        node.cancel_event.set()
    node.on_wait = choose
    assert node.execute().outcome == 2
    assert len(node.requests) == 10 and h.snapshot() == before


@pytest.mark.parametrize('change', ['source_stop', 'provider_lost'])
def test_continue_rechecks_before_clearing_history_and_does_not_lock_out_stop(monkeypatch, change):
    node = pick_rig(monkeypatch, [0] * 10, [])
    h = history(node)
    before = h.snapshot()
    choices = []

    def choose():
        if choices:
            assert not node.managed.resume.is_set() and h.snapshot() == before
            node.cancel_event.set()
            return
        choices.append(True)
        assert RobotController._continue(node, None, Command.Response()).success
        if change == 'provider_lost':
            node._perception_ready = lambda _: False
            return

        def source_check(_root):
            acquired = []

            def stop():
                with node.stop_guard:
                    acquired.append(True)
                    node.cancel_event.set()
            worker = threading.Thread(target=stop, daemon=True)
            worker.start()
            worker.join(1.)
            assert acquired, 'Direct Stop must not wait for source hashing'
        node.configuration.validate_sources = source_check
    node.on_wait = choose
    result = node.execute()
    assert result.outcome == result.CANCELED
    assert len(node.requests) == 10 and h.snapshot() == before


@pytest.mark.parametrize('auto', [False, True])
def test_item_wait_gui_offers_continue_and_stop_with_action_pending(window, auto):  # noqa: F811
    window.node.status = status(
        state='PAUSED', phase='ITEM_ACQUISITION_PAUSED', operation='auto_run' if auto else 'pick',
        operation_active=True, auto_run_active=auto, auto_run_requested=100,
        auto_run_completed=95, holding_item=False, can_return_item=False)
    window.result_future = SimpleNamespace(done=lambda: False)
    window._refresh()
    assert presentation(window.node.status)[0] == 'WAITING FOR ITEMS'
    assert window.pause.isEnabled() and window.pause.text() == 'CONTINUE'
    assert 'history' in window.pause.toolTip()
    assert window.stop.isEnabled() and not window.pick_item.isEnabled()
    assert not window.place_item.isEnabled() and not window.home_button.isEnabled()
    assert not window.clear_repick.isEnabled() and not window.auto_run_button.isEnabled()
    window.node.status.item_detector_ready = False
    window._refresh()
    assert not window.pause.isEnabled() and window.stop.isEnabled()
