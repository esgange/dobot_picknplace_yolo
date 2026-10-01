"""Bounded Pick acquisition/physical retries and Place requests without hardware."""

import copy
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.task import Future
from rclpy.time import Time
from tray_perception_interfaces.srv import GetTrayPose

import robot_controller.tray_client as tray_module
from robot_controller.errors import FeedbackFailure, ManagedInterruption, OperationCanceled
from robot_controller.tray_client import TrayAcquisitionExhausted, TrayAttempts, TrayClient
from test_automatic_return import action_rig
from test_placement import response_fixture


def pick_rig(monkeypatch, counts, acquisitions):
    rig = action_rig(monkeypatch, losses=())
    rig.hardware.acquisitions = iter(acquisitions)

    def detect(*_args, **_kwargs):
        if rig.requests:
            assert np.allclose(rig.hardware.current_pose(), rig.configuration.home_matrix)
        assert not rig.holding_item
        index = len(rig.requests)
        rig.requests.append("detect")
        rig.log.append(("detect", index + 1))
        return SimpleNamespace(identifier=f"batch{index + 1}", debug_message="OFF", candidates=[
            SimpleNamespace(identifier=f"b{index + 1}:{i + 1}", position_m=(i * .1, 0., .3),
                            quaternion=(0., 0., 0., 1.)) for i in range(counts[index])])
    rig.candidates.request = detect
    previous_home = rig._execute_home

    def home(**kwargs):
        rig.log.append(("ensure_home",))
        return previous_home(**kwargs)
    rig._execute_home = home
    return rig


@pytest.mark.parametrize("successful_attempt", [1, 2, 3])
def test_pick_success_ends_retries_and_counts_all_previous_candidates(
        monkeypatch, successful_attempt):
    rig = pick_rig(monkeypatch, [2] * 3, [False] * (2 * (successful_attempt - 1)) + [True])
    result = rig.execute()
    assert result.outcome == result.SUCCESS and result.final_state == "HOLDING"
    assert result.selected_candidate_id == f"b{successful_attempt}:1"
    assert result.attempted_candidates == (successful_attempt - 1) * 2 + 1
    assert len(rig.requests) == successful_attempt
    assert rig.finished == ["success"]
    final = [row for row in rig.log if row[0] == "move"][-1]
    assert final[1] == ("p1_retract", "p1_final", "tray_detect_position")
    assert np.array_equal(rig.hardware.current_pose(), rig.configuration.tray.detect_matrix)
    assert "Tray Detect" in result.message


@pytest.mark.parametrize("counts", [[2, 1, 3], [0, 2, 1, 3], [2, 0, 1, 3]])
def test_pick_exhausts_three_complete_batches_and_returns_home_between_them(monkeypatch, counts):
    rig = pick_rig(monkeypatch, counts, [False] * sum(counts))
    result = rig.execute()
    assert result.outcome == result.NO_PICK and result.final_state == "READY"
    assert result.attempted_candidates == sum(counts)
    assert "3 Pick attempts" in result.message
    assert len(rig.requests) == len(counts) and rig.finished == ["success"]
    for attempt, count in enumerate(counts, 1):
        start = rig.log.index(("detect", attempt))
        end = rig.log.index(("detect", attempt + 1)) if attempt < len(counts) else len(rig.log)
        moves = [row for row in rig.log[start:end] if row[0] == "move"]
        if count:
            assert len([row for row in moves if row[2].get("stop_on_suction")]) == count
            assert moves[-1][1][-1] == "home"
        else:
            assert not moves


def test_pick_requests_poses_before_home_and_homes_before_candidate_motion(monkeypatch):
    rig = pick_rig(monkeypatch, [1], [True])
    assert not np.allclose(rig.hardware.current_pose(), rig.configuration.home_matrix)
    result = rig.execute()
    assert result.outcome == result.SUCCESS
    detect = rig.log.index(("detect", 1))
    home = rig.log.index(("ensure_home",))
    first_move = next(i for i, row in enumerate(rig.log) if row[0] == "move")
    assert detect < home < first_move
    assert len(rig.requests) == 1


@pytest.mark.parametrize("retry_count", [0, 1])
def test_empty_pose_result_homes_then_retries_only_once(monkeypatch, retry_count):
    rig = pick_rig(monkeypatch, [0, retry_count], [True] if retry_count else [])
    result = rig.execute()
    assert result.outcome == (result.SUCCESS if retry_count else result.NO_PICK)
    assert result.attempted_candidates == retry_count
    assert len(rig.requests) == 2
    assert rig.log.index(("detect", 1)) < rig.log.index(("ensure_home",)) < rig.log.index(
        ("detect", 2))
    if not retry_count:
        assert "one Home acquisition retry" in result.message
        assert not any(row[0] == "move" for row in rig.log)
        assert np.array_equal(rig.hardware.current_pose(), rig.configuration.home_matrix)


def test_empty_retry_budget_survives_a_physical_pick_miss(monkeypatch):
    rig = pick_rig(monkeypatch, [0, 1, 0], [False])
    result = rig.execute()
    assert result.outcome == result.NO_PICK and result.attempted_candidates == 1
    assert len(rig.requests) == 3
    assert "one Home acquisition retry" in result.message


@pytest.mark.parametrize("counts", [[1], [0]])
@pytest.mark.parametrize("failure", [FeedbackFailure, OperationCanceled])
def test_home_failure_after_detection_blocks_retry_or_candidate_motion(
        monkeypatch, counts, failure):
    rig = pick_rig(monkeypatch, counts, [])
    rig._execute_home = Mock(side_effect=failure("Home interrupted"))
    result = rig.execute()
    assert result.outcome == (result.CANCELED if failure is OperationCanceled
                              else result.FEEDBACK_FAILURE)
    assert len(rig.requests) == 1
    assert not any(row[0] == "move" for row in rig.log)


def test_detector_error_is_terminal_without_home_or_retry(monkeypatch):
    rig = pick_rig(monkeypatch, [], [])
    rig.candidates.request = Mock(side_effect=FeedbackFailure("detector failed"))
    result = rig.execute()
    assert result.outcome == result.FEEDBACK_FAILURE
    rig.candidates.request.assert_called_once()
    assert ("ensure_home",) not in rig.log


@pytest.mark.parametrize("counts", [[0, 0], [1]])
def test_pause_during_home_preserves_acquired_batch_or_one_retry_budget(monkeypatch, counts):
    rig = pick_rig(monkeypatch, counts, [True] if counts[0] else [])
    rig.hardware.on_move = Mock()  # No held-loss injection before a candidate ledger exists.
    home = rig._execute_home
    interrupted = []

    def pause(**kwargs):
        if not interrupted:
            interrupted.append(True)
            rig.managed.request("pause")
            rig.managed.checkpoint()
        return home(**kwargs)
    rig._execute_home = pause
    result = rig.execute()
    assert result.outcome == (result.SUCCESS if counts[0] else result.NO_PICK)
    assert len(rig.requests) == len(counts) and ("state", "PAUSED") in rig.log


@pytest.mark.parametrize("cancel", [False, True])
def test_pick_fault_or_stop_in_second_batch_ends_retry_and_keeps_total(monkeypatch, cancel):
    rig = pick_rig(monkeypatch, [2] * 3, [False, False])
    original = rig.hardware.on_move

    def fail(targets, kwargs):
        original(targets, kwargs)
        if len(rig.requests) == 2:
            rig.managed.session.set_state(1, "ACTIVE")
            if cancel:
                rig.cancel_event.set()
            raise (OperationCanceled if cancel else FeedbackFailure)("injected failure")
    rig.hardware.on_move = fail
    result = rig.execute()
    assert result.outcome == (result.CANCELED if cancel else result.FEEDBACK_FAILURE)
    assert result.attempted_candidates == 3
    assert len(rig.requests) == 2 and rig.finished == ["abort"]


def test_pick_pause_between_batches_preserves_budget(monkeypatch):
    rig = pick_rig(monkeypatch, [1] * 3, [False] * 3)
    progress = rig.operation_progress
    paused = []

    def pause(phase, message, **kwargs):
        if phase == "DETECT" and len(rig.requests) == 1 and not paused:
            paused.append(True)
            rig.managed.request("pause")
        progress(phase, message, **kwargs)
    rig.operation_progress = pause
    result = rig.execute()
    assert result.outcome == result.NO_PICK and result.attempted_candidates == 3
    assert paused and len(rig.requests) == 3
    assert ("state", "PAUSED") in rig.log


def test_pick_refuses_replayed_batch_without_retrying_its_old_poses(monkeypatch):
    rig = pick_rig(monkeypatch, [1] * 3, [False])
    detect = rig.candidates.request

    def repeated(*args, **kwargs):
        batch = detect(*args, **kwargs)
        batch.identifier = "replayed"
        return batch
    rig.candidates.request = repeated
    result = rig.execute()
    assert result.outcome == result.FEEDBACK_FAILURE
    assert "reused" in result.message and result.attempted_candidates == 1
    assert len(rig.requests) == 2


@pytest.mark.parametrize('tray', [None, SimpleNamespace(detect_joints=None)])
def test_pick_execution_rechecks_tray_before_home_or_detection(monkeypatch, tray):
    rig = pick_rig(monkeypatch, [1], [True])
    rig.configuration.tray = tray
    result = rig.execute()
    assert result.outcome != result.SUCCESS
    assert 'Tray Detect Pose' in result.message
    assert not rig.requests
    assert not any(row[0] == 'move' for row in rig.log)


def test_stop_during_successful_tray_group_cannot_report_success(monkeypatch):
    rig = pick_rig(monkeypatch, [1], [True])
    original = rig.hardware.on_move

    def stop(targets, kwargs):
        original(targets, kwargs)
        if kwargs['batch_name'] == 'candidate_1_pick_to_tray':
            rig.cancel_event.set()
            raise OperationCanceled('Stop during tray travel')
    rig.hardware.on_move = stop
    result = rig.execute()
    assert result.outcome == result.CANCELED
    assert result.final_state == 'RECOVERY_REQUIRED'
    assert rig.finished == ['abort'] and len(rig.requests) == 1
    assert rig.managed.session.attempts[0].state == 'HELD'


def test_held_pause_during_successful_route_continues_directly_to_tray(monkeypatch):
    rig = pick_rig(monkeypatch, [1], [True])
    original = rig.hardware.on_move
    paused = []

    def pause(targets, kwargs):
        original(targets, kwargs)
        if kwargs['batch_name'] == 'candidate_1_pick_to_tray' and not paused:
            paused.append(True)
            rig.managed.request('pause')
            rig.managed.checkpoint()
    rig.hardware.on_move = pause
    result = rig.execute()
    assert result.outcome == result.SUCCESS and result.final_state == 'HOLDING'
    assert paused and ('state', 'PAUSED') in rig.log
    assert len(rig.requests) == 1
    final = [row for row in rig.log if row[0] == 'move'][-1]
    assert final[1] == ('tray_detect_position',)
    assert final[2]['require_suction']
    assert np.array_equal(rig.hardware.current_pose(), rig.configuration.tray.detect_matrix)


class TrayRig:
    def __init__(self, monkeypatch, replies):
        self.result, self.config, self.sampling = response_fixture()
        self.config.validate_sources = Mock()
        self.now = 100.
        monkeypatch.setattr(tray_module.time, "monotonic", lambda: self.now)
        self.replies = iter(replies)
        self.futures = []
        self.on_send = lambda _future: None
        self.node = SimpleNamespace(
            root=None, create_client=lambda *_: self.client,
            _service_providers=lambda _: [("tray_teach", "/")],
            get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=self.now)),
            events=Mock(), operation_progress=Mock(), wait_for_resume=Mock(),
            _preflight_item_state=Mock(), monitor=Mock(), wait_control=self.advance)
        self.client = SimpleNamespace(service_is_ready=lambda: True,
                                      call_async=Mock(side_effect=self.send))
        self.observer = TrayClient(self.node)
        self.attempts = TrayAttempts()

    def advance(self, _seconds):
        self.now += self.sampling["request_timeout_sec"] + 2

    def send(self, _request):
        future = Future()
        self.futures.append(future)
        reply = next(self.replies)
        if reply == "valid":
            result = copy.deepcopy(self.result)
            self.now += .01
            stamp = Time(seconds=self.now).to_msg()
            result.header.stamp = result.placement.depth_header.stamp = stamp
            future.set_result(result)
        elif reply != "timeout":
            future.set_result(reply)
        self.on_send(future)
        return future

    def request(self, **kwargs):
        return self.observer.request(self.config, 30., 40., attempts=self.attempts, **kwargs)


@pytest.mark.parametrize("failure", [
    None, "timeout",
    GetTrayPose.Response(success=True, found=False, status="NO_VALID_TRAY", message="No tray"),
    GetTrayPose.Response(status="ERROR", message="No fresh calibrated observation"),
    GetTrayPose.Response(status="BUSY", message="Previous request active"),
])
@pytest.mark.parametrize("require_held_item", [True, False])
def test_tray_retries_missing_pose_or_reply_and_accepts_third_fresh_result(
        monkeypatch, failure, require_held_item):
    rig = TrayRig(monkeypatch, [failure, failure, "valid"])
    assert rig.request(require_held_item=require_held_item) == pytest.approx([.13, .24, .25])
    assert rig.client.call_async.call_count == rig.attempts.count == 3
    assert rig.observer.pending is None
    if failure == "timeout":
        assert all(f.cancelled() for f in rig.futures[:2])
    if require_held_item:
        rig.node._preflight_item_state.assert_called_with(True)
    else:
        rig.node.monitor.snapshot.assert_called_with(require_enabled=True)
        rig.node._preflight_item_state.assert_not_called()


@pytest.mark.parametrize("failure", [None, "timeout"])
def test_tray_exhausts_three_failures_with_typed_final_reason(monkeypatch, failure):
    rig = TrayRig(monkeypatch, [failure] * 3)
    with pytest.raises(TrayAcquisitionExhausted, match="failed after 3 attempts"):
        rig.request()
    assert rig.client.call_async.call_count == rig.attempts.count == 3
    assert rig.observer.pending is None
    with pytest.raises(TrayAcquisitionExhausted, match="failed after 3 attempts"):
        rig.request()
    assert rig.client.call_async.call_count == 3


@pytest.mark.parametrize("attempt", [1, 2])
def test_tray_first_usable_observation_ends_retry(monkeypatch, attempt):
    rig = TrayRig(monkeypatch, [None] * (attempt - 1) + ["valid"])
    assert rig.request() == pytest.approx([.13, .24, .25])
    assert rig.attempts.count == rig.client.call_async.call_count == attempt


@pytest.mark.parametrize("error", [OperationCanceled, FeedbackFailure, ManagedInterruption])
def test_tray_stop_feedback_loss_or_pause_prevents_a_retry(monkeypatch, error):
    rig = TrayRig(monkeypatch, ["timeout", "valid"])
    rig.on_send = lambda _: setattr(rig.node.wait_for_resume, "side_effect", error("interrupted"))
    with pytest.raises(error, match="interrupted"):
        rig.request()
    assert rig.client.call_async.call_count == rig.attempts.count == 1


def test_tray_pause_drains_old_request_and_keeps_three_attempt_limit(monkeypatch):
    rig = TrayRig(monkeypatch, ["timeout", None, "valid"])
    rig.on_send = lambda _: setattr(
        rig.node.wait_for_resume, "side_effect", ManagedInterruption("Pause"))
    with pytest.raises(ManagedInterruption):
        rig.request()
    assert rig.observer.pending and rig.attempts.count == 1
    rig.node.wait_for_resume.side_effect = None
    rig.on_send = lambda _: None
    assert rig.request() == pytest.approx([.13, .24, .25])
    assert rig.futures[0].cancelled() and rig.attempts.count == 3


def test_tray_discards_late_first_reply_and_validates_new_request_time(monkeypatch):
    rig = TrayRig(monkeypatch, ["timeout", "valid"])

    def late(_future):
        if len(rig.futures) == 2:
            old = copy.deepcopy(rig.result)
            old.placement.surface_base.z = .9
            rig.futures[0].set_result(old)
    rig.on_send = late
    assert rig.request() == pytest.approx([.13, .24, .25])
    assert rig.attempts.count == 2


@pytest.mark.parametrize("damage", ["hash", "cached", "geometry"])
def test_tray_invalid_success_evidence_is_terminal_without_retry(monkeypatch, damage):
    rig = TrayRig(monkeypatch, [None, "valid"])
    reply = copy.deepcopy(rig.result)
    reply.header.stamp.sec = reply.placement.depth_header.stamp.sec = 100
    if damage == "hash":
        reply.diagnostics_json = reply.diagnostics_json.replace('"camera"', '"changed"')
    elif damage == "geometry":
        reply.placement.surface_base.x = 20.
    rig.replies = iter([None, reply])
    if damage != "cached":
        def fresh(_future):
            rig.now += .01
            reply.header.stamp = reply.placement.depth_header.stamp = Time(
                seconds=rig.now).to_msg()
        rig.on_send = fresh
    reason = {"hash": "mismatch", "cached": "cached/future", "geometry": "X/Y differs"}
    with pytest.raises(FeedbackFailure, match=reason[damage]):
        rig.request()
    assert rig.attempts.count == 2


def test_tray_source_change_after_failed_observation_blocks_resend(monkeypatch):
    rig = TrayRig(monkeypatch, [None, "valid"])
    rig.on_send = lambda _: setattr(
        rig.config.validate_sources, "side_effect", FeedbackFailure("source changed"))
    with pytest.raises(FeedbackFailure, match="source changed"):
        rig.request()
    assert rig.client.call_async.call_count == 1


def test_tray_late_completed_reply_uses_retry_budget(monkeypatch):
    rig = TrayRig(monkeypatch, ["valid", "valid", "valid"])
    rig.on_send = lambda _: rig.advance(0)
    with pytest.raises(FeedbackFailure, match="after its deadline"):
        rig.request()
    assert rig.attempts.count == 3


def test_pause_after_third_tray_request_cannot_dispatch_a_fourth(monkeypatch):
    rig = TrayRig(monkeypatch, [None, None, "timeout"])

    def pause(_future):
        if len(rig.futures) == 3:
            rig.node.wait_for_resume.side_effect = ManagedInterruption("Pause")
    rig.on_send = pause
    with pytest.raises(ManagedInterruption):
        rig.request()
    rig.node.wait_for_resume.side_effect = None
    with pytest.raises(FeedbackFailure, match="after 3 attempts"):
        rig.request()
    assert rig.client.call_async.call_count == 3 and rig.observer.pending is None
