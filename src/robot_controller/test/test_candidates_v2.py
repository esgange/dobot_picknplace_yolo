import copy
import json
from types import SimpleNamespace

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
        "model": {"sha256": "model"},
        "yolo": {"confidence": 0.4, "class_ids": [0, 2]},
        "quality": {"result_max_age_sec": 2.0, "sync_tolerance_sec": 0.1},
    }
    station = SimpleNamespace(
        camera=SimpleNamespace(sha256="camera"),
        platform=SimpleNamespace(sha256="platform"))
    selection = SimpleNamespace(bin=SimpleNamespace(sha256="bin"), station=station)
    return SimpleNamespace(
        profile=profile, profile_sha256="profile", selection=selection,
        pose_candidates=3, configuration_id="configuration")


def valid_result(*, candidate_count=1):
    result = GetItemPoses.Response()
    result.success = True
    result.status = "OK"
    result.message = "fresh"
    result.batch_id = "batch"
    result.header.frame_id = "platform_reference"
    result.header.stamp.sec = result.depth_stamp.sec = 100
    result.diagnostics_json = json.dumps({
        "profile_sha256": "profile", "model_sha256": "model",
        "camera_sha256": "camera", "platform_sha256": "platform",
        "bin_sha256": "bin",
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


@pytest.mark.parametrize("key", [
    "profile_sha256", "model_sha256", "camera_sha256", "platform_sha256", "bin_sha256"])
def test_every_detector_binding_hash_is_mandatory(tmp_path, key):
    result = valid_result()
    evidence = json.loads(result.diagnostics_json)
    evidence[key] = "changed"
    result.diagnostics_json = json.dumps(evidence)
    with pytest.raises(FeedbackFailure, match=key):
        client(tmp_path)._validate_result(result, configuration(), False)


@pytest.mark.parametrize("mutation,match", [
    (lambda result: setattr(result.header, "frame_id", "base_link"), "frame"),
    (lambda result: setattr(result, "batch_id", ""), "batch ID/count"),
    (lambda result: setattr(result.header.stamp, "sec", 90), "stale"),
    (lambda result: setattr(result.depth_stamp, "nanosec", 200_000_000),
     "not synchronized"),
    (lambda result: setattr(result.candidates[0], "priority", 2), "Malformed"),
    (lambda result: setattr(result.candidates[0], "class_id", 9), "Malformed"),
    (lambda result: setattr(result.candidates[0], "confidence", 0.1), "Malformed"),
    (lambda result: setattr(result.candidates[0].pose.orientation, "w", 0.5), "Malformed"),
])
def test_malformed_or_stale_candidate_evidence_is_rejected(tmp_path, mutation, match):
    result = valid_result()
    mutation(result)
    with pytest.raises(FeedbackFailure, match=match):
        client(tmp_path)._validate_result(result, configuration(), False)


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
