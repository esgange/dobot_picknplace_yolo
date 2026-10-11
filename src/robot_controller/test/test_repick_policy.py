"""Physical attempt accounting, saved-batch filtering and placement-owned clearing."""

import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from item_perception_yolo.repick import REPICK_DEFAULTS, contains
from robot_controller.candidates import CandidateBatch
from robot_controller.errors import FeedbackFailure
from robot_controller.motion import PickExecutor, pick_tray_target
from robot_controller.pick_session import PickSession
from robot_controller.repick import RepickHistory
from test_candidates_v2 import client, configuration, valid_result
from test_managed_control import Rig


def history(trigger=3, cooldown=5):
    return RepickHistory({"blaclist_trigger": trigger, "blackist_radius": 30.,
                          "blackist_cooldown": cooldown}, Mock())


def complete(h, x=0., z=.1):
    return h.completed((x, 0., z), "candidate")


def test_disabled_does_not_track_or_age():
    h = RepickHistory(REPICK_DEFAULTS, Mock())
    for _ in range(10):
        assert complete(h) is None
    assert h.snapshot() == (0, []) and not h.regions


def test_third_attempt_activates_fixed_circle_despite_id_height_and_pose_jitter():
    h = history()
    first = complete(h)
    assert not h.blocked((0., 0., .1))
    assert complete(h, .02, .7) == first
    assert not h.blocked((0., 0., .1))
    assert complete(h, -.02, -.1) == first
    assert h.blocked((.03, 0., 10.))
    assert not h.blocked((.03001, 0., .1))
    region = h.snapshot()[1][0]
    assert region["x_m"] == 0. and region["attempts"] == 3
    assert region["cooldown_remaining"] == 5


def test_only_five_other_completed_attempts_expire_and_reset_count():
    h = history()
    for _ in range(3):
        complete(h)
    for remaining in (4, 3, 2, 1):
        complete(h, .2 * (5 - remaining))
        assert h.blocked((0., 0.))["cooldown_remaining"] == remaining
        for _ in range(10):
            h.snapshot()  # captures and filtering never advance a cooldown
            h.blocked((0., 0.))
    complete(h, 1.)
    assert not h.blocked((0., 0.))
    complete(h)
    assert next(r for r in h.regions if r["x_m"] == 0.)["attempts"] == 1


def test_overlap_uses_oldest_center_and_success_removes_only_owned_area():
    h = history(trigger=2)
    first = complete(h, 0.)
    second = complete(h, .05)
    assert complete(h, .025) == first
    assert h.blocked((.025, 0.))["id"] == first
    h.placed(second)
    assert [r["id"] for r in h.regions] == [first]
    h.placed(second)  # duplicate placement callbacks cannot remove another region
    h.placed(first)
    assert not h.regions


def test_bounded_history_retires_only_oldest_inactive_area(monkeypatch):
    monkeypatch.setattr("robot_controller.repick.MAX_REGIONS", 3)
    h = history(trigger=2, cooldown=3)
    oldest = complete(h, 0.)
    active = complete(h, .1)
    complete(h, .1)
    later = complete(h, .2)
    newest = complete(h, .3)
    assert oldest not in [r["id"] for r in h.regions]
    assert [r["id"] for r in h.regions] == [active, later, newest]
    assert h.blocked((.1, 0.))["cooldown_remaining"] == 1


def session(h, points):
    batch = SimpleNamespace(candidates=[SimpleNamespace(position_m=p) for p in points])
    return PickSession([str(i) for i in range(len(points))], [()] * len(points),
                       batch=batch, repick=h)


@pytest.mark.parametrize("outcome", ["FAILED", "HELD"])
def test_session_records_only_completed_outcome_once_and_skips_saved_neighbors(outcome):
    h = history(trigger=1)
    s = session(h, [(0., 0., .1), (.02, 0., .4), (.1, 0., .1)])
    s.set_state(1, "ACTIVE")
    s.interrupt_active()
    assert not h.regions
    s.set_state(1, "ACTIVE")
    s.set_state(1, outcome)
    s.set_state(1, outcome)
    assert h.regions[0]["attempts"] == 1
    assert s.next_eligible == 3 and s.attempted_count == 1
    s.set_state(3, "CANCELED")
    assert s.blacklist_only
    if outcome == "HELD":
        s.complete_release("PLACED")
        assert not h.regions


@pytest.mark.parametrize("release", ["RETURNED", "DROPPED"])
def test_return_or_drop_does_not_clear_source_history(release):
    h = history(trigger=1)
    s = session(h, [(0., 0., .1)])
    s.set_state(1, "ACTIVE")
    s.set_state(1, "HELD")
    s.set_state(1, release)
    assert h.blocked((0., 0.))


def test_motion_skips_newly_blocked_saved_candidate_without_changing_other_route():
    rig = Rig(count=3)
    h = history(trigger=1)
    s = rig.managed.session
    s.repick = h
    s.batch = SimpleNamespace(candidates=[SimpleNamespace(position_m=p) for p in (
        (0., 0., .3), (.02, 0., .3), (.2, 0., .3))])
    rig.hardware.acquisitions = iter([False, True])
    result = PickExecutor(rig.hardware, finish_home=True).run(
        [a.plan for a in s.attempts], rig.configuration.profile, session=s,
        tray_target=pick_tray_target(rig.configuration.tray, rig.configuration.profile),
        check=lambda _: None, return_home=rig._execute_home)
    assert result["picked"] and result["candidate"] == 3
    assert [a.state for a in s.attempts] == ["FAILED", "PENDING", "HELD"]
    assert s.attempted_count == 2
    assert not any(row[0] == "move" and "retry_2" in str(row) for row in rig.log)


def test_client_independently_filters_and_checks_exclusion_echo(tmp_path):
    h = history(trigger=1)
    complete(h)
    result = valid_result(candidate_count=3)
    result.candidates[2].pose.position.x = .1
    config = configuration()
    config.profile["retry"] = h.settings
    evidence = json.loads(result.diagnostics_json)
    revision, regions = h.snapshot()
    evidence["repick"].update(revision=revision, regions=regions)
    result.diagnostics_json = json.dumps(evidence)
    observer = client(tmp_path)
    batch = observer._validate_result(result, config, False, expected_repick=evidence["repick"])
    assert [c.identifier for c in batch.candidates] == ["batch:2"]
    with pytest.raises(FeedbackFailure, match="repick response"):
        observer._validate_result(result, config, False,
                                  expected_repick={**evidence["repick"], "revision": 999})


def test_history_snapshot_and_batch_are_not_mutated_by_prefetch_or_placement():
    h = history(trigger=1)
    source = complete(h)
    revision, regions = h.snapshot()
    batch = CandidateBatch("batch", 1, 1, (), {}, "OFF", True, revision)
    h.placed(source)
    assert regions[0]["cooldown_remaining"] == 5
    assert batch.blacklist_revision != h.snapshot()[0]
    assert h.filter_batch(batch).blacklist_only  # caller must reacquire omitted poses


@pytest.mark.parametrize("position,blocked", [((.03, 0.), True), ((0., -.03), True),
                                            ((.03, .001), False)])
def test_circle_is_radial_not_square(position, blocked):
    assert contains({"x_m": 0., "y_m": 0., "radius_m": .03}, position) is blocked


def test_blacklisted_only_manual_pick_homes_without_empty_acquisition_loop(monkeypatch):
    from test_operation_retries import pick_rig
    rig = pick_rig(monkeypatch, [0], [])
    detect = rig.candidates.request

    def excluded(*args, **kwargs):
        batch = detect(*args, **kwargs)
        batch.blacklist_only = True
        return batch
    rig.candidates.request = excluded
    result = rig.execute()
    assert result.outcome == result.NO_PICK and result.attempted_candidates == 0
    assert len(rig.requests) == 1 and ("ensure_home",) in rig.log
    assert "unblocked" in result.message


@pytest.mark.parametrize("placement_clears", [False, True])
def test_auto_blacklisted_prefetch_confirms_release_then_rechecks_changed_history(placement_clears):
    from robot_controller.errors import OperationCanceled
    from test_auto_run import queue_rig
    from test_placement_queue import RELEASE, OPEN
    rig, run, bridge = queue_rig(quantity=2)
    h = history(trigger=1)
    source = complete(h)
    rig.node.repick = h
    if placement_clears:
        bridge.old_session.repick = h
        bridge.old_session.attempts[0].area_id = source
    rig.steps = iter([
        dict(outputs=RELEASE, inputs=OPEN, currentCommandId=2),
        dict(outputs=0, inputs=OPEN, currentCommandId=3),
        dict(outputs=0, inputs=OPEN, currentCommandId=4),
        dict(outputs=0, inputs=0, home=True),
    ])

    def fresh(*_args, **_kwargs):
        assert bridge.completed and run.completed == 1
        assert not h.regions
        raise OperationCanceled("Fresh request reached after successful placement")
    rig.node.candidates = SimpleNamespace(request=Mock(side_effect=fresh))
    batch = CandidateBatch("blocked-prefetch", 1, 1, (), {}, "OFF", True, h.revision)
    if placement_clears:
        with pytest.raises(OperationCanceled, match="Fresh request"):
            run._pick(batch, bridge)
        assert rig.node.candidates.request.call_count == 1
    else:
        assert not run._pick(batch, bridge)
        rig.node.candidates.request.assert_not_called()
        assert h.blocked((0., 0.))
    assert bridge.completed and run.completed == 1
    assert [name for name, _ in rig.requests] == ["MovL", "MovLIO", "MovLIO", "MovJ"]


@pytest.mark.parametrize("state,held,busy,accepted", [
    ("READY", False, False, True), ("INACTIVE", False, False, True),
    ("READY", True, False, False), ("HOLDING", True, False, False),
    ("PICKING", False, True, False), ("READY", False, True, False),
])
def test_clear_history_requires_idle_unheld_and_discards_saved_poses(state, held, busy, accepted):
    import threading
    from robot_controller.controller import RobotController
    from robot_controller.errors import CommandRejected
    h = history(trigger=1)
    complete(h)
    saved = Mock()
    begin = Mock(side_effect=CommandRejected("busy") if busy else None)
    node = SimpleNamespace(
        machine=SimpleNamespace(state=state), holding_item=held, configuration=object(),
        repick=h, managed=SimpleNamespace(session=saved), _begin_operation=begin,
        _end_operation=Mock(), stop_guard=threading.RLock(), raise_if_cancelled=Mock(),
        publish_operator_log=Mock())
    result = RobotController._clear_repick_history(node, None, SimpleNamespace())
    assert result.success is accepted
    if accepted:
        assert not h.regions and node.managed.session is None
        saved.cancel_remaining.assert_called_once()
    else:
        assert h.regions
        saved.cancel_remaining.assert_not_called()
