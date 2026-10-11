"""Synthetic endpoint, physical-attempt, and queued placement reset coverage."""

from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest

from item_perception_yolo.pick_planning import camera_body_footprint
from item_perception_yolo.surface_guard import HeightPlane
from robot_controller.motion import PickExecutor, pick_targets, pick_tray_target
from robot_controller.pick_session import PickSession
from robot_controller.repick import RepickHistory, TrickleTargetRejected, trickle_plan
from test_managed_control import Rig, matrix
from test_preview_routes import preview  # noqa: F401


def configure(config, step=10., trigger=3):
    config.profile["retry"] = dict(blaclist_trigger=trigger, blackist_radius=30.,
                                   blackist_cooldown=5, trickle_depth=step)
    config.selection = NS(
        station=NS(platform=NS(base_from_platform=np.eye(4))),
        robot_camera=NS(reference_from_camera_link=np.eye(4), sha256="c" * 64),
        bin=NS(points=[NS(x_m=x, y_m=y) for x, y in
                       ((-2., -2.), (2., -2.), (2., 2.), (-2., 2.))]))
    return RepickHistory(config.profile["retry"], Mock())


def session(config, history, points, **kwargs):
    batch = NS(identifier="batch", candidates=[NS(position_m=p) for p in points])
    plans = [pick_targets(config.home_matrix, matrix(x=p[0], z=p[2]),
                          config.profile, i) for i, p in enumerate(points, 1)]
    return PickSession([f"pose{i}" for i in range(len(points))], plans, batch=batch,
                       configuration=config, repick=history, **kwargs)


def complete(s, index, state="FAILED"):
    s.set_state(index, "ACTIVE")
    s.set_state(index, state)


def test_same_batch_uses_0_10_20_then_blocks_without_compounding_or_route_changes():
    rig = Rig(count=4)
    history = configure(rig.configuration)
    s = session(rig.configuration, history, [(0., 0., .3)] * 4)
    rig.managed.session = s
    sent = []
    rig.hardware.on_target = lambda target, _: sent.append(target)
    result = PickExecutor(rig.hardware, finish_home=True).run(
        [a.plan for a in s.attempts], rig.configuration.profile, session=s,
        tray_target=pick_tray_target(rig.configuration.tray, rig.configuration.profile),
        check=lambda _: None, return_home=rig._execute_home)
    assert not result["picked"] and s.attempted_count == 3 and s.blacklist_only
    picks = [t for t in sent if t.name.endswith("_pick")]
    assert [t.matrix[2, 3] for t in picks] == pytest.approx([.3, .29, .28])
    for original, attempt in zip(s.nominal_plans, s.attempts):
        for index in (0, 1, 2, 4, 5):
            assert attempt.plan[index] is original[index]
        actual = attempt.plan[3]
        np.testing.assert_array_equal(actual.matrix[:2], original[3].matrix[:2])
        assert actual.motion_io == original[3].motion_io
        assert actual.speed_percent == original[3].speed_percent
        assert actual.acceleration_percent == original[3].acceleration_percent
    assert np.array_equal(rig.hardware.current_pose(), rig.configuration.home_matrix)


def test_fresh_measurement_is_baseline_and_interrupted_attempt_keeps_depth():
    config = Rig().configuration
    history = configure(config)
    history.completed((0., 0., .1), "old")
    s = session(config, history, [(0.02, 0., .4)])
    for _ in range(3):
        assert s.next_eligible == 1
        assert s.attempts[0].plan[3].matrix[2, 3] == pytest.approx(.39)
    s.set_state(1, "ACTIVE")
    s.interrupt_active()
    assert s.next_eligible == 1
    assert s.attempts[0].plan[3].matrix[2, 3] == pytest.approx(.39)
    assert history.regions[0]["attempts"] == 1
    complete(s, 1)
    assert history.regions[0]["attempts"] == 2


@pytest.mark.parametrize("reset", ["placed", "clear", "cooldown", "restart"])
def test_history_resets_restore_first_attempt_depth(reset):
    config = Rig().configuration
    history = configure(config, trigger=1)
    area = history.completed((0., 0., .1), "old")
    if reset == "placed":
        history.placed(area)
    elif reset == "clear":
        history.clear()
    elif reset == "restart":
        history = RepickHistory(history.settings, Mock())
    else:
        for _ in range(5):
            history.completed((.5, 0., .1), "elsewhere")
    s = session(config, history, [(0., 0., .3)])
    assert s.next_eligible == 1
    assert s.attempts[0].plan[3].matrix[2, 3] == pytest.approx(.3)


@pytest.mark.parametrize("step,trigger", [(0., 3), (10., 0)])
def test_disabled_behavior_preserves_exact_nominal_plan(step, trigger):
    config = Rig().configuration
    history = configure(config, step, trigger)
    s = session(config, history, [(0., 0., .3)])
    assert s.next_eligible == 1
    assert s.attempts[0].plan is s.nominal_plans[0]


@pytest.mark.parametrize("angle", [0., 25., 180., 205.])
@pytest.mark.parametrize("deficit,accepted", [
    (0., True), (.019, True), (.020001, True), (.020002, False)])
def test_guard_checks_adjusted_endpoint_at_actual_xy_with_tilt_and_reversed_normal(
        angle, deficit, accepted):
    config = Rig().configuration
    configure(config)
    angle = np.deg2rad(angle)
    plane = np.eye(4)
    plane[:3, :3] = [[np.cos(angle), 0., np.sin(angle)], [0., 1., 0.],
                     [-np.sin(angle), 0., np.cos(angle)]]
    plane[2, 3] = .1
    config.selection.station.platform.base_from_platform = plane
    plane_z = HeightPlane(plane, "bin floor").measure((.1, 0., 0.)).plane_z_m
    plan = pick_targets(config.home_matrix, matrix(x=.1, z=plane_z + .01), config.profile, 1)
    if accepted:
        actual, height = trickle_plan(plan, config, 10. + deficit * 1000)
        assert actual[3].matrix[2, 3] == pytest.approx(plane_z - deficit)
        assert not height.below_plane
    else:
        with pytest.raises(TrickleTargetRejected, match="target Z=.*plane Z=.*deficit="):
            trickle_plan(plan, config, 10. + deficit * 1000)


@pytest.mark.parametrize("damage", ["vertical", "nonrigid", "nonfinite", "overflow"])
def test_unusable_geometry_remains_error_not_ordinary_rejection(damage):
    config = Rig().configuration
    configure(config)
    plan = pick_targets(config.home_matrix, matrix(), config.profile, 1)
    plane = config.selection.station.platform.base_from_platform
    if damage == "vertical":
        plane[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    elif damage == "nonrigid":
        plane[0, 0] = 2.
    elif damage == "nonfinite":
        plane[2, 3] = np.nan
    with pytest.raises(ValueError) as exc:
        trickle_plan(plan, config, float("inf") if damage == "overflow" else 10.)
    assert not isinstance(exc.value, TrickleTargetRejected)


def test_deeper_endpoint_rechecks_camera_body_without_changing_attitude():
    config = Rig().configuration
    configure(config)
    plane = config.selection.station.platform.base_from_platform
    a = np.deg2rad(30.)
    plane[:3, :3] = [[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]]
    plan = pick_targets(config.home_matrix, matrix(), config.profile, 1)
    footprint = camera_body_footprint(np.linalg.inv(plane) @ plan[3].matrix)
    right = max(x for x, y in footprint) + .001
    config.selection.bin.points = [NS(x_m=x, y_m=y) for x, y in
                                   ((-2, -2), (right, -2), (right, 2), (-2, 2))]
    trickle_plan(plan, config, 0.)
    with pytest.raises(TrickleTargetRejected, match="camera.*outside"):
        trickle_plan(plan, config, 10.)


def test_rejects_excess_descent_skips_to_survivor_without_counting_rejection():
    config = Rig().configuration
    history = configure(config, step=350.)
    history.completed((0., 0., .3), "old")
    s = session(config, history, [(0., 0., .3), (.1, 0., .3)])
    assert s.next_eligible == 2
    assert s.attempted_count == 0 and 1 in s.trickle_rejections
    assert s.attempts[0].state == "PENDING"
    assert s.attempts[0].plan[3].matrix[2, 3] == .3  # never clamped
    assert history.regions[0]["attempts"] == 1


@pytest.mark.parametrize("auto", [False, True])
def test_all_unsafe_targets_confirm_home_without_spending_detection_retry_budget(monkeypatch, auto):
    from test_operation_retries import pick_rig
    from test_auto_run import request
    from robot_controller.autorun import AutoRunOperation
    from robot_controller.controller import RobotController
    rig = pick_rig(monkeypatch, [1], [])
    rig.managed.session = None
    rig.repick = configure(rig.configuration, step=350.)
    rig.repick.completed((0., 0., .3), "old")
    # This fixture uses a minimal response rather than CandidateBatch.
    rig.repick.filter_batch = lambda batch: batch
    if auto:
        rig._plan_candidate_batch = lambda batch: RobotController._plan_candidate_batch(rig, batch)
        assert not AutoRunOperation(rig, request())._pick()
    else:
        result = rig.execute()
        assert result.outcome == result.NO_PICK and result.attempted_candidates == 0
    assert len(rig.requests) == 1
    assert "trickle-depth" in rig.machine.message
    assert np.array_equal(rig.hardware.current_pose(), rig.configuration.home_matrix)
    assert not any(row[0] == "move" and row[2].get("stop_on_suction") for row in rig.log)


def test_placement_bridge_plans_cleared_source_but_does_not_erase_history_early():
    config = Rig().configuration
    history = configure(config)
    source = history.completed((0., 0., .3), "held")
    other = history.completed((.1, 0., .3), "miss")
    s = session(config, history, [(0., 0., .3), (.1, 0., .3)], clearing_area_id=source)
    assert s.next_eligible == 1
    assert s.attempts[0].plan[3].matrix[2, 3] == .3
    assert history.prior_attempts((0., 0.)) == (1, source)
    assert s.prepare_trickle(2)
    assert s.attempts[1].plan[3].matrix[2, 3] == pytest.approx(.29)
    history.placed(source)
    assert history.prior_attempts((.1, 0.)) == (1, other)


def test_auto_prefetch_wires_expected_placement_clear_before_next_queue(monkeypatch):
    from robot_controller.autorun import AutoRunOperation
    from robot_controller.controller import RobotController
    from robot_controller.errors import OperationCanceled
    from test_auto_run import request
    from test_operation_retries import pick_rig
    node = pick_rig(monkeypatch, [1], [])
    node.repick = configure(node.configuration)
    old = session(node.configuration, node.repick, [(0., 0., .3)])
    complete(old, 1, "HELD")
    source = old.attempts[0].area_id
    node.managed.session = old
    node.repick.filter_batch = lambda batch: batch
    node._plan_candidate_batch = lambda batch: RobotController._plan_candidate_batch(node, batch)
    bridge = NS(completed=False, old_session=old, next_session=None)
    batch = node.candidates.request(node.configuration)

    def verify_queue(_plans, **kwargs):
        assert kwargs["placement_bridge"] is bridge
        assert bridge.next_session.clearing_area_id == source
        assert bridge.next_session.next_eligible == 1
        assert bridge.next_session.attempts[0].plan[3].matrix[2, 3] == .3
        assert node.repick.prior_attempts((0., 0.)) == (1, source)
        assert old.attempts[0].state == "HELD"
        raise OperationCanceled("Test stops before motion")
    node.managed.run_pick = verify_queue
    with pytest.raises(OperationCanceled, match="Test stops"):
        AutoRunOperation(node, request())._pick(batch, bridge)


def test_drop_with_only_unsafe_saved_targets_finishes_return_at_home(monkeypatch):
    from test_automatic_return import action_rig
    node = action_rig(monkeypatch, count=2, losses=(1,))
    node.repick = configure(node.configuration, step=350.)
    node.repick.filter_batch = lambda batch: batch
    detect = node.candidates.request

    def same_area(*args, **kwargs):
        batch = detect(*args, **kwargs)
        for candidate in batch.candidates:
            candidate.position_m = (0., 0., .3)
        return batch
    node.candidates.request = same_area
    result = node.execute()
    assert result.outcome == result.NO_PICK and result.attempted_candidates == 1
    assert node.requests == ["detect"] and node.lost == [1]
    assert "trickle-depth" in result.message
    assert np.array_equal(node.hardware.current_pose(), node.configuration.home_matrix)
    assert node.repick.prior_attempts((0., 0.))[0] == 1  # return is not placement


@pytest.mark.parametrize("unsafe", [False, True])
def test_preview_uses_same_guard_with_zero_physical_history(preview, unsafe):  # noqa: F811
    from robot_controller_interfaces.srv import Preview
    config = preview.config
    configure(config)
    batch = preview.client.request.return_value
    batch.identifier = "preview-batch"
    for index, candidate in enumerate(batch.candidates):
        candidate.identifier = str(index)
        if unsafe:
            candidate.position_m = (candidate.position_m[0], .2, -.1)
    config.profile["motion"]["standoff_height"] = 0.
    result = preview.run(Preview.Request.PICK)
    assert result.success and "attempt 1 (0 mm extra)" in result.message
    picks = [t for t in preview.targets if t.name.endswith("_pick")]
    assert len(picks) == (0 if unsafe else 2)
    assert ("2 unsafe targets skipped" in result.message) is unsafe
