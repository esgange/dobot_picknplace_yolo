import copy
import json
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from item_perception_interfaces.msg import ItemCandidate
from item_perception_interfaces.srv import GetItemPoses
from robot_controller.candidates import CandidateClient
from robot_controller.errors import FeedbackFailure


class Events:
    def record(self, *_args, **_kwargs):
        pass


def configuration():
    profile = {
        "model": {"sha256": "model"}, "geometry": {"depth_frame_count": 3},
        "yolo": {"confidence": 0.4, "class_ids": [0, 2]},
        "quality": {"sync_tolerance_sec": 0.1},
    }
    station = SimpleNamespace(
        camera=SimpleNamespace(sha256="camera"),
        platform=SimpleNamespace(sha256="platform", base_from_platform=np.eye(4)))
    selection = SimpleNamespace(bin=SimpleNamespace(sha256="bin"), station=station,
                                robot_camera=SimpleNamespace(sha256="robot_camera"))
    return SimpleNamespace(
        profile=profile, profile_sha256="profile", selection=selection,
        pose_candidates=3, configuration_id="configuration", home_matrix=np.eye(4))


def valid_result(*, candidate_count=1):
    result = GetItemPoses.Response()
    result.success = True
    result.status = "OK"
    result.message = "fresh"
    result.batch_id = "batch"
    result.header.frame_id = "platform_reference"
    result.header.stamp.sec = result.depth_stamp.sec = 100
    result.diagnostics_json = json.dumps({
        "pose_convention": GetItemPoses.Request.POSE_CONVENTION,
        "profile_sha256": "profile", "model_sha256": "model",
        "camera_sha256": "camera", "platform_sha256": "platform",
        "robot_camera_sha256": "robot_camera",
        "bin_sha256": "bin",
        "depth_frame_stamps_ns": [99_934_000_000, 99_967_000_000, 100_000_000_000],
        "debug_capture": {
            "requested": False, "rgb_path": "", "depth_path": "", "error": ""},
    })
    for index in range(candidate_count):
        candidate = ItemCandidate()
        candidate.id = f"batch:{index}"
        candidate.priority = index + 1
        candidate.class_id = 0
        candidate.confidence = 0.9 - index * 0.1
        candidate.center_distance = index * 0.01
        candidate.pose.position.x = index * 0.01
        candidate.pose.position.z = 0.1
        candidate.pose.orientation.w = 1.0
        result.candidates.append(candidate)
    return result


def client(tmp_path):
    value = object.__new__(CandidateClient)
    value.root = tmp_path
    value.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_200_000_000)),
        events=Events())
    return value


def test_fresh_hash_matched_ranked_batch_is_accepted(tmp_path):
    batch = client(tmp_path)._validate_result(
        valid_result(candidate_count=2), configuration(), False)
    assert batch.identifier == "batch"
    assert [candidate.priority for candidate in batch.candidates] == [1, 2]
    assert batch.candidates[0].position_m == (0.0, 0.0, 0.1)
    assert batch.debug_message == "OFF"


@pytest.mark.parametrize("positions,expected", [
    ([(.3, .2, .1), (.1, .2, .1), (.2, .2, .1)], [1, 2, 0]),
    ([(.1, .4, .1), (.1, .1, .1), (.1, .3, .1)], [1, 2, 0]),
    # Identical XY: the higher surface is closer to a Home above the items.
    ([(.1, .2, .1), (.1, .2, .6), (.1, .2, .3)], [1, 2, 0]),
    # XY alone would prefer the first pose; all three dimensions must count.
    ([(0., 0., .1), (.2, 0., .7), (.3, 0., .5)], [1, 2, 0]),
])
def test_home_distance_reorders_validated_poses_without_changing_detector_evidence(
        tmp_path, positions, expected):
    config = configuration()
    config.home_matrix[2, 3] = .8
    result = valid_result(candidate_count=3)
    for value, position in zip(result.candidates, positions):
        value.pose.position.x, value.pose.position.y, value.pose.position.z = position
    original = copy.deepcopy(result)
    observer = client(tmp_path)
    observer.node.events = Mock()
    batch = observer._validate_result(result, config, False)
    assert [c.identifier for c in batch.candidates] == [f"batch:{i}" for i in expected]
    assert [c.priority for c in batch.candidates] == [i + 1 for i in expected]
    assert result == original
    assert batch.evidence == json.loads(original.diagnostics_json)
    details = observer.node.events.record.call_args.kwargs
    assert details["ranking"] == "home_distance_3d"
    assert details["home_position_base_m"] == [0., 0., .8]
    for rank, (entry, index) in enumerate(zip(details["candidate_order"], expected), 1):
        assert entry["controller_priority"] == rank
        assert entry["detector_priority"] == index + 1
        assert entry["candidate_id"] == f"batch:{index}"
        assert entry["home_distance_m"] == pytest.approx(
            np.linalg.norm(np.array(positions[index]) - [0., 0., .8]))


def test_home_ranking_uses_base_transform_and_taught_home_instead_of_origin(tmp_path):
    config = configuration()
    # Platform local X points along base Y, and its origin is translated.
    config.selection.station.platform.base_from_platform = np.array([
        [0., -1., 0., 1.], [1., 0., 0., 2.], [0., 0., 1., .3], [0., 0., 0., 1.]])
    config.home_matrix[:3, 3] = [1., 2.02, .8]
    batch = client(tmp_path)._validate_result(valid_result(candidate_count=3), config, False)
    assert [c.identifier for c in batch.candidates] == ["batch:2", "batch:1", "batch:0"]
    # Retain platform-relative poses for the existing motion/preview planners.
    assert batch.candidates[0].position_m == (.02, 0., .1)


def test_equal_home_distances_keep_detector_priority_even_with_different_confidence(tmp_path):
    result = valid_result(candidate_count=3)
    for value, xy in zip(result.candidates, [(1., 0.), (0., 1.), (-1., 0.)]):
        value.pose.position.x, value.pose.position.y = xy
    result.candidates[0].confidence = .5
    result.candidates[2].confidence = .99
    batch = client(tmp_path)._validate_result(result, configuration(), False)
    assert [c.priority for c in batch.candidates] == [1, 2, 3]


@pytest.mark.parametrize("frame", ["home", "platform"])
def test_invalid_ranking_transform_cannot_admit_a_batch(tmp_path, frame):
    config = configuration()
    matrix = (config.home_matrix if frame == "home" else
              config.selection.station.platform.base_from_platform)
    matrix[0, 3] = float("nan")
    with pytest.raises(FeedbackFailure, match="Cannot prioritize item poses from Home"):
        client(tmp_path)._validate_result(valid_result(), config, False)


@pytest.mark.parametrize("key", [
    "profile_sha256", "model_sha256", "camera_sha256", "robot_camera_sha256",
    "platform_sha256", "bin_sha256"])
def test_every_detector_binding_hash_is_mandatory(tmp_path, key):
    result = valid_result()
    evidence = json.loads(result.diagnostics_json)
    evidence[key] = "changed"
    result.diagnostics_json = json.dumps(evidence)
    with pytest.raises(FeedbackFailure, match=key):
        client(tmp_path)._validate_result(result, configuration(), False)


@pytest.mark.parametrize("convention", [None, "", "item_long_x_short_y"])
def test_detector_pose_convention_must_match_before_planning(tmp_path, convention):
    result = valid_result()
    evidence = json.loads(result.diagnostics_json)
    if convention is None:
        evidence.pop("pose_convention")
    else:
        evidence["pose_convention"] = convention
    result.diagnostics_json = json.dumps(evidence)
    with pytest.raises(FeedbackFailure, match="pose_convention"):
        client(tmp_path)._validate_result(result, configuration(), False)


@pytest.mark.parametrize("mutation,match", [
    (lambda result: setattr(result.header, "frame_id", "base_link"), "frame"),
    (lambda result: setattr(result, "batch_id", ""), "batch ID/count"),
    (lambda result: setattr(result.header.stamp, "sec", 0), "invalid"),
    (lambda result: setattr(result.depth_stamp, "nanosec", 200_000_000),
     "not synchronized"),
    (lambda result: setattr(result.candidates[0], "priority", 2), "Malformed"),
    (lambda result: setattr(result.candidates[0], "class_id", 9), "Malformed"),
    (lambda result: setattr(result.candidates[0], "confidence", 0.1), "Malformed"),
    (lambda result: setattr(result.candidates[0].pose.orientation, "w", 0.5), "Malformed"),
])
def test_malformed_candidate_evidence_is_rejected(tmp_path, mutation, match):
    result = valid_result()
    mutation(result)
    with pytest.raises(FeedbackFailure, match=match):
        client(tmp_path)._validate_result(result, configuration(), False)


def test_candidate_orientation_must_be_platform_plane_yaw(tmp_path):
    result = valid_result()
    result.candidates[0].pose.orientation.x = 0.1
    result.candidates[0].pose.orientation.w = (1.0 - 0.1**2) ** 0.5
    with pytest.raises(FeedbackFailure, match="Malformed"):
        client(tmp_path)._validate_result(result, configuration(), False)


def test_synchronized_positive_candidate_timestamps_do_not_expire(tmp_path):
    result = valid_result()
    result.header.stamp.sec = result.depth_stamp.sec = 1
    evidence = json.loads(result.diagnostics_json)
    evidence["depth_frame_stamps_ns"] = [940_000_000, 970_000_000, 1_000_000_000]
    result.diagnostics_json = json.dumps(evidence)
    batch = client(tmp_path)._validate_result(result, configuration(), False)
    assert batch.observation_stamp_ns == 1_000_000_000


def test_duplicate_and_out_of_order_candidates_are_rejected(tmp_path):
    result = valid_result(candidate_count=2)
    result.candidates[1].id = result.candidates[0].id
    with pytest.raises(FeedbackFailure, match="Malformed"):
        client(tmp_path)._validate_result(result, configuration(), False)
    result = valid_result(candidate_count=2)
    result.candidates[1].center_distance = -1.0
    with pytest.raises(FeedbackFailure, match="Malformed"):
        client(tmp_path)._validate_result(result, configuration(), False)


def test_debug_capture_must_stay_in_exact_debug_directory(tmp_path):
    result = valid_result(candidate_count=0)
    evidence = json.loads(result.diagnostics_json)
    evidence["debug_capture"] = {
        "requested": True,
        "rgb_path": str(tmp_path / "debug/pick_img/rgb.png"),
        "depth_path": str(tmp_path / "debug/pick_img/depth.png"), "error": ""}
    result.diagnostics_json = json.dumps(evidence)
    batch = client(tmp_path)._validate_result(result, configuration(), True)
    assert batch.debug_message.startswith("SAVED:")
    escaped = copy.deepcopy(evidence)
    escaped["debug_capture"]["rgb_path"] = str(tmp_path / "rgb.png")
    result.diagnostics_json = json.dumps(escaped)
    with pytest.raises(FeedbackFailure, match="escaped"):
        client(tmp_path)._validate_result(result, configuration(), True)


def test_detector_cannot_return_more_than_taught_pose_candidates(tmp_path):
    with pytest.raises(FeedbackFailure, match="batch ID/count"):
        client(tmp_path)._validate_result(
            valid_result(candidate_count=4), configuration(), False)


@pytest.mark.parametrize("stamps", [[], [100_000_000_000],
                                    [99_934_000_000, 99_934_000_000, 100_000_000_000],
                                    [99_800_000_000, 99_967_000_000, 100_000_000_000],
                                    [99_934_000_000, 99_967_000_000, 100_100_000_000]])
def test_controller_rejects_incompatible_depth_bundle_evidence(tmp_path, stamps):
    result = valid_result()
    evidence = json.loads(result.diagnostics_json)
    evidence["depth_frame_stamps_ns"] = stamps
    result.diagnostics_json = json.dumps(evidence)
    with pytest.raises(FeedbackFailure, match="temporal depth"):
        client(tmp_path)._validate_result(result, configuration(), False)


def test_controller_rejects_pre_request_bundle_even_if_last_frame_is_new(tmp_path):
    with pytest.raises(FeedbackFailure, match="predates request"):
        client(tmp_path)._validate_result(valid_result(), configuration(), False,
                                          requested_at_ns=99_950_000_000)
