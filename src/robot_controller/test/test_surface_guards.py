"""Controller independently enforces the 20 mm plane allowance before offsets."""

import copy
import json
from unittest.mock import Mock

import numpy as np
import pytest

from camera_calibration_gui.calibration_core import rotation_matrix_to_quaternion
from item_perception_yolo.pick_planning import rpy_matrix
from robot_controller.errors import FeedbackFailure, OperationCanceled
from robot_controller.tray_client import TrayAcquisitionExhausted, validate_result
from test_candidates_v2 import client, configuration, valid_result
from test_operation_retries import TrayRig, pick_rig
from test_placement import response_fixture
from test_tray_acquisition_pause import acquisition_rig, exhaust


@pytest.mark.parametrize("flipped", [False, True])
def test_bin_filters_before_home_ranking_without_changing_ids_or_evidence(tmp_path, flipped):
    config = configuration()
    config.home_matrix[2, 3] = .8
    config.profile["motion"] = {"standoff_height": 500.}
    plane = config.selection.station.platform.base_from_platform
    if flipped:
        plane[:] = np.diag([1., -1., -1., 1.])
    result = valid_result(candidate_count=3)
    for candidate, z in zip(result.candidates, [-.021, .1, .2]):
        candidate.pose.position.z = -z if flipped else z
    original = copy.deepcopy(result)
    observer = client(tmp_path)
    observer.node.events = Mock()
    batch = observer._validate_result(result, config, False)
    assert [c.identifier for c in batch.candidates] == ["batch:2", "batch:1"]
    assert [c.priority for c in batch.candidates] == [3, 2]
    assert result == original
    assert batch.evidence == json.loads(result.diagnostics_json)
    rejection = observer.node.events.record.call_args_list[0]
    assert rejection.args[1] == "candidate_plane_rejected"
    assert "deficit=21.000 mm" in rejection.args[2]
    assert "allowed deficit=20.000 mm" in rejection.args[2]
    assert rejection.kwargs["candidate_id"] == "batch:0"


@pytest.mark.parametrize("flipped", [False, True])
def test_bin_keeps_within_allowance_survivors_and_nearest_home_order(tmp_path, flipped):
    config = configuration()
    config.home_matrix[2, 3] = .8
    if flipped:
        config.selection.station.platform.base_from_platform[:] = np.diag([1., -1., -1., 1.])
    result = valid_result(candidate_count=3)
    for candidate, z in zip(result.candidates, [-.02, -.021, .1]):
        candidate.pose.position.z = -z if flipped else z
    original = copy.deepcopy(result)
    batch = client(tmp_path)._validate_result(result, config, False)
    assert [c.identifier for c in batch.candidates] == ["batch:2", "batch:0"]
    assert [c.priority for c in batch.candidates] == [3, 1]
    assert result == original


def test_all_below_floor_is_a_valid_empty_batch(tmp_path):
    result = valid_result(candidate_count=2)
    for candidate in result.candidates:
        candidate.pose.position.z = -.021
    batch = client(tmp_path)._validate_result(result, configuration(), False)
    assert batch.candidates == () and batch.identifier == "batch"
    # A bad pose is not allowed to hide malformed response evidence.
    result.candidates[0].priority = 5
    with pytest.raises(FeedbackFailure, match="Malformed"):
        client(tmp_path)._validate_result(result, configuration(), False)


def test_bad_floor_is_fatal_even_with_no_candidates(tmp_path):
    config = configuration()
    config.selection.station.platform.base_from_platform = np.array([
        [0., 0., 1., 0.], [0., 1., 0., 0.], [-1., 0., 0., 0.], [0., 0., 0., 1.]])
    with pytest.raises(FeedbackFailure, match="parallel to vertical travel"):
        client(tmp_path)._validate_result(valid_result(candidate_count=0), config, False)


def test_all_below_floor_uses_existing_nine_home_retries(monkeypatch, tmp_path):
    rig = pick_rig(monkeypatch, [0] * 10, [])
    detect = rig.candidates.request
    observer = client(tmp_path)
    config = configuration()

    def filtered(*args, **kwargs):
        existing = detect(*args, **kwargs)
        result = valid_result()
        result.batch_id = existing.identifier
        result.candidates[0].id = f"{existing.identifier}:0"
        result.candidates[0].pose.position.z = -.021
        return observer._validate_result(result, config, False)

    rig.candidates.request = filtered
    result = rig.execute()
    assert result.outcome == result.NO_PICK and result.final_state == "READY"
    assert result.attempted_candidates == 0 and len(rig.requests) == 10
    assert sum(row == ("ensure_home",) for row in rig.log) == 9
    assert not any(row[0] == "move" and row[2].get("stop_on_suction") for row in rig.log)


@pytest.mark.parametrize("preview", [False, True])
def test_tray_below_plane_retries_fresh_observations_then_accepts(monkeypatch, preview):
    rig = TrayRig(monkeypatch, ["valid"] * 3)
    rig.config.profile["motion"]["trayplace_height"] = 500.
    stamps = []

    def lower_first_two(future):
        result = future.result()
        stamps.append(result.header.stamp.nanosec)
        if len(stamps) < 3:
            result.placement.surface_base.z = -.021
    rig.on_send = lower_first_two
    options = {"check_state": Mock()} if preview else {}
    point = rig.request(**options)
    assert point == pytest.approx([.13, .24, .25])
    assert rig.attempts.count == 3 and stamps == sorted(set(stamps))
    failed = [call for call in rig.node.events.record.call_args_list
              if call.args[1] == "tray_observation_attempt_failed"]
    assert len(failed) == 2 and all("below tray plane" in call.args[2] for call in failed)


@pytest.mark.parametrize("provider_rejected", [False, True])
def test_tray_below_plane_exhausts_ten_requests(monkeypatch, provider_rejected):
    rig = TrayRig(monkeypatch, ["valid"] * 10)
    rig.result.placement.surface_base.z = -.021
    if provider_rejected:
        rig.result.status = "NO_VALID_PLACEMENT_DEPTH"
        rig.result.placement.valid = False
    with pytest.raises(TrayAcquisitionExhausted, match="below tray plane"):
        rig.request()
    assert rig.attempts.count == rig.client.call_async.call_count == 10


def test_tray_below_plane_pauses_with_item_held_and_return_available(monkeypatch):
    node, tray = acquisition_rig(monkeypatch, ["valid"] * 10)
    tray.result.placement.surface_base.z = -.021
    exhaust(node)

    def paused():
        assert node.machine.state == "PAUSED" and node.holding_item
        assert node.managed.can_return_item()
        assert "below tray plane" in node.machine.message
        node.cancel_event.set()
    node.on_wait = paused
    with pytest.raises(OperationCanceled):
        node.placement.handle_pause(node)


@pytest.mark.parametrize("damage", ["plane", "provider", "hash", "rejected_hash",
                                    "rejected_above", "rejected_valid", "rejected_missing"])
def test_geometry_and_source_errors_do_not_consume_tray_retries(monkeypatch, damage):
    rig = TrayRig(monkeypatch, ["valid"])
    rig.result.placement.surface_base.z = -.021
    if damage == "plane":
        rig.config.tray.profile["reference_plane"]["base_from_plane"][2][2] = 0.
    elif damage == "provider":
        rig.result.success = False
        rig.result.status = "INVALID_GEOMETRY"
        rig.result.message = "bad saved plane"
    elif damage in ("hash", "rejected_hash"):
        evidence = json.loads(rig.result.diagnostics_json)
        evidence["camera_sha256"] = "wrong"
        rig.result.diagnostics_json = json.dumps(evidence)
    if damage.startswith("rejected_"):
        rig.result.status = "NO_VALID_PLACEMENT_DEPTH"
        rig.result.placement.valid = damage == "rejected_valid"
        if damage == "rejected_above":
            rig.result.placement.surface_base.z = .01
        elif damage == "rejected_missing":
            rig.result.found = False
    with pytest.raises(FeedbackFailure) as error:
        rig.request()
    assert not isinstance(error.value, TrayAcquisitionExhausted)
    assert rig.client.call_async.call_count == 1


@pytest.mark.parametrize("z,valid", [
    (0., True), (-.01, True), (-.02, True), (-.020001, True), (-.020002, False)])
def test_tray_exact_boundary_and_tolerance(z, valid):
    result, config, sampling = response_fixture()
    result.placement.surface_base.z = z
    if valid:
        point, _ = validate_result(result, config, sampling, 1_000_000_000, 3_000_000_000)
        assert point[2] == z
    else:
        from robot_controller.tray_client import TrayObservationUnavailable
        with pytest.raises(TrayObservationUnavailable, match="below tray plane"):
            validate_result(result, config, sampling, 1_000_000_000, 3_000_000_000)


@pytest.mark.parametrize("flipped", [False, True])
@pytest.mark.parametrize("height", [-.021, -.02, -.005, 0., .005])
def test_controller_uses_saved_tilted_plane_at_returned_xy(flipped, height):
    from robot_controller.tray_client import TrayObservationUnavailable
    result, config, sampling = response_fixture()
    plane = np.eye(4)
    plane[:3, :3] = rpy_matrix(.2, -.3, .1)
    plane[:3, 3] = [.1, .2, .3]
    rotation = plane[:3, :3].copy()
    # Saved normal direction may differ from the detected tray's inward axes.
    if flipped:
        plane = plane @ np.diag([1., -1., -1., 1.])
    config.tray.profile["reference_plane"]["base_from_plane"] = plane.tolist()
    evidence = json.loads(result.diagnostics_json)
    evidence["reference_plane"] = config.tray.profile["reference_plane"]
    result.diagnostics_json = json.dumps(evidence)
    p, q = result.tray.pose.position, result.tray.pose.orientation
    p.x, p.y, p.z = map(float, plane[:3, 3])
    q.x, q.y, q.z, q.w = rotation_matrix_to_quaternion(rotation)
    point = plane[:3, 3] + rotation @ [.03, .04, 0.]
    point[2] += height
    surface = result.placement.surface_base
    surface.x, surface.y, surface.z = map(float, point)
    if height < -.02:
        with pytest.raises(TrayObservationUnavailable, match="deficit=21.000 mm"):
            validate_result(result, config, sampling, 1_000_000_000, 3_000_000_000)
    else:
        accepted, _ = validate_result(result, config, sampling, 1_000_000_000, 3_000_000_000)
        assert accepted == pytest.approx(point)
