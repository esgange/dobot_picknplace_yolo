import copy
import json
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from item_perception_interfaces.srv import GetItemPoses
from item_perception_yolo import item_detector as detector
from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS


def test_pair_freshness_sync_registration():
    rgb = {"stamp_ns": 100_000_000_000, "received_at": time.monotonic(), "width": 640,
           "height": 480}
    depth = dict(rgb)
    info = {"width": 640, "height": 480, "k": [1], "d": [0]}
    detector.validate_pair(rgb, depth, info, info, 100_200_000_000, QUALITY_DEFAULTS)
    # Gemini 335 D2C depth has zero distortion; raw RGB retains factory D.
    detector.validate_pair(rgb, depth, {**info, "d": [.007, -.048, -.0002, .0002, .033]},
                           {**info, "d": [0.] * 5}, 100_200_000_000, QUALITY_DEFAULTS)
    for bad in ({**depth, "stamp_ns": 99_000_000_000}, {**depth, "width": 320},
                {**depth, "stamp_ns": 100_150_000_000},
                {**depth, "received_at": time.monotonic()-1}):
        with pytest.raises(ValueError):
            detector.validate_pair(rgb, bad, info, info, 100_200_000_000, QUALITY_DEFAULTS)
    with pytest.raises(ValueError, match="intrinsics"):
        detector.validate_pair(rgb, depth, info, {**info, "k": [2]}, 100_200_000_000,
                               QUALITY_DEFAULTS)


def test_station_source_changes_disarm(monkeypatch):
    node = SimpleNamespace(applied=object(),
        bin_artifact=SimpleNamespace(path="bin", sha256="old"), disarm=MagicMock(),
        service=object(), last_view={"old": "overlay"})
    monkeypatch.setattr(detector, "validate_applied_sources", lambda _: None)
    monkeypatch.setattr(detector, "file_sha256", lambda _: "changed")
    with pytest.raises(ValueError, match="bin teach changed"):
        detector.ItemDetectNode._validate_sources(node)
    node.disarm.assert_called_once()
    assert node.last_view is None
    node.service = None
    with pytest.raises(ValueError, match="bin teach changed"):
        detector.ItemDetectNode._validate_sources(node)
    node.disarm.assert_called_once()  # No endless generation changes in unarmed teaching.


@pytest.fixture
def service_node(monkeypatch):
    candidate = {"source_index": 4, "class_id": 1, "class_name": "part", "confidence": .8,
                 "position": [.01, .02, .1], "quaternion": [0., 0., 0., 1.],
                 "length": .1, "width": .05, "center_distance": .01,
                 "filtered_camera_depth": .7, "depth_sigma": .001,
                 "accepted_depth_count": 100, "rejected_depth_count": 10, "pixel": [100., 100.]}
    result = {"candidates": [candidate], "rejected": [], "count": 1, "inference_ms": 10.}
    node = SimpleNamespace(request_lock=threading.Lock(), operation_lock=threading.Lock(),
                           preview_mode="all",
                           service=object(), arm_epoch=1, yolo_enabled=True, retry_limit=3,
                           profile_digest="a"*64, profile_path="profile.yaml", native=SimpleNamespace(failed=False),
                           settings={"quality": dict(QUALITY_DEFAULTS), "geometry_source": "mask",
                                     "yolo": {"max_detections": 20, "class_ids": [1], "confidence": .6}},
                           _validate_sources=MagicMock(), events=MagicMock(), disarm=MagicMock(),
                           model_config={"sha256": "b"*64}, get_logger=MagicMock(),
                           applied=SimpleNamespace(camera=SimpleNamespace(sha256="c"*64),
                                                   platform=SimpleNamespace(sha256="d"*64)),
                           bin_artifact=SimpleNamespace(sha256="e"*64),
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_200_000_000)))
    frame = {"stamp_ns": 100_200_000_000}
    node._snapshot = MagicMock(return_value=(frame, frame, {"synthetic": True}))
    node.infer = MagicMock(return_value={"metadata": result})
    monkeypatch.setattr(detector, "load_item_profile", lambda _: ({}, "a"*64))
    monkeypatch.setattr(detector, "file_sha256", lambda _: "a"*64)
    return node, candidate


def call(node, count=3, digest="a"*64):
    return detector.ItemDetectNode._request(node,
                                            GetItemPoses.Request(max_candidates=count, profile_sha256=digest), GetItemPoses.Response())


def test_request_new_observation_shortage_identity_and_no_cache(service_node):
    node, _ = service_node
    result = call(node)
    assert result.success and result.status == "SHORTAGE"
    assert len(result.candidates) == 1 and result.header.frame_id == "platform_reference"
    assert result.candidates[0].priority == 1
    assert result.candidates[0].id.startswith(result.batch_id + ":")
    assert json.loads(result.diagnostics_json)["profile_sha256"] == "a"*64
    assert node._snapshot.call_args.args[0] == 100_200_000_000
    again = call(node)
    assert again.batch_id != result.batch_id
    assert node.infer.call_count == 2  # no cached pose response


def test_disarmed_busy_mismatch_count_and_no_valid_items(service_node):
    node, _ = service_node
    assert not call(node, digest="f"*64).success
    assert not call(node, count=4).success
    assert not call(node, count=0).success
    node.request_lock.acquire()
    assert call(node).status == "BUSY"
    node.request_lock.release()
    node.service = None
    assert not call(node).success
    node.service = object()
    node.infer.return_value["metadata"]["candidates"] = []
    result = call(node)
    assert result.success and result.status == "NO_VALID_ITEMS" and not result.candidates


def test_disarm_during_request_and_expired_result_never_return_pose(service_node):
    node, _ = service_node
    original = node.infer.return_value

    def disarmed(*args, **kwargs):
        node.arm_epoch += 1
        return original
    node.infer.side_effect = disarmed
    result = call(node)
    assert not result.success and not result.candidates
    assert "disarmed" in result.message
    node.infer.side_effect = None
    old = {"stamp_ns": 95_000_000_000}
    node._snapshot.return_value = (old, old, {})
    assert "expired" in call(node).message


def test_native_protocol_candidate_checks(service_node):
    node, candidate = service_node
    result = {"candidates": [candidate]}
    detector.validate_candidates(result, node.settings)
    for changes in ({"class_id": 20}, {"position": [float("nan"), 0., 1.]},
                    {"quaternion": [0., 0., 0., 0.]}, {"accepted_depth_count": 0}):
        bad = {"candidates": [{**candidate, **changes}]}
        with pytest.raises(RuntimeError):
            detector.validate_candidates(bad, node.settings)
    with pytest.raises(RuntimeError, match="duplicate"):
        detector.validate_candidates(
            {"candidates": [candidate, copy.deepcopy(candidate)]}, node.settings)


def test_teaching_preview_keeps_rgb_when_depth_or_station_missing():
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    rgb = {"stamp_ns": 100_000_000_000, "received_at": time.monotonic()}
    node = SimpleNamespace(
        yolo_enabled=True, preview_mode="all", preview_source="mask", arm_epoch=1,
        request_lock=threading.Lock(), operation_lock=threading.Lock(),
        settings=None, model_config={"path": "verified.pt"},
        model_metadata={"task": "segment", "classes": {"0": "first", "7": "second"},
                        "geometry_sources": ["mask"]}, events=MagicMock(),
        camera_snapshot=lambda: (rgb, "RGB live"),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_100_000_000)),
        _measurement_context=MagicMock(side_effect=ValueError("No platform applied")),
        _snapshot=MagicMock(side_effect=AssertionError("Must not request depth")),
        infer=MagicMock(return_value={"rgb": b"preview"}),
        condition=threading.Condition(), _depth=None, _color_info=None, _depth_info=None,
        _camera_generation=1,
    )
    yolo = {**detector.INITIAL_PREVIEW_YOLO, "confidence": .9, "iou": .35,
            "max_detections": 20, "class_ids": []}
    detector.ItemDetectNode.enable_preview(node, "mask", yolo,
                                          geometry=None, quality=QUALITY_DEFAULTS, diameter_mm=30.)
    assert node.settings is None
    view = detector.ItemDetectNode.preview_once(node)
    assert view["rgb"] == b"preview" and view["observation"]["depth"] is None
    node._snapshot.assert_not_called()
    preview = node.infer.call_args.kwargs["preview"]
    assert preview["settings"]["yolo"] == {**yolo, "class_ids": [0, 7]}
    yolo["confidence"] = .8  # Applied state is an owned snapshot, not a mutable form dict.
    assert node.preview_yolo["confidence"] == .9
    detector.ItemDetectNode.enable_preview(node, "mask", yolo,
                                          geometry=None, quality=QUALITY_DEFAULTS, diameter_mm=60.)
    assert preview["error"] == "No platform applied" and preview["context"] is None
    node._measurement_context.side_effect = None
    node._measurement_context.return_value = {"calibrated": True}
    detector.ItemDetectNode.preview_once(node)
    assert node.infer.call_args.kwargs["preview"]["settings"]["pickdepth_radius"] == 60.
    assert node.infer.call_args.kwargs["preview"]["settings"]["yolo"]["confidence"] == .8
    assert node.infer.call_args.kwargs["preview"]["context"] == {"calibrated": True}
    # Preserve each camera model with the exact frozen depth observation.
    rgb.update(width=848, height=480)
    color = {"width": 848, "height": 480, "k": [461., 0., 424., 0., 461., 240., 0., 0., 1.],
             "d": [.007, -.048, -.0002, .0002, .033], "distortion_model": "plumb_bob"}
    node._color_info = color
    node._depth_info = {**color, "d": [0.] * 5}
    node._depth = {**rgb, "depth": b"native depth"}
    node._measurement_context.return_value = {"camera": copy.deepcopy(color)}
    view = detector.ItemDetectNode.preview_once(node)
    context = view["observation"]["context"]
    assert not view["depth_error"]
    assert context["camera"]["d"] == color["d"]
    assert context["depth_camera"]["d"] == [0.] * 5
    assert view["observation"]["depth"]["depth"] == b"native depth"
    node._depth_info["d"][0] = .1
    assert context["depth_camera"]["d"] == [0.] * 5  # Snapshot owns its metadata.
    node._depth_info = {**node._depth_info, "k": [400., 0., 424., 0., 400., 240., 0., 0., 1.]}
    view = detector.ItemDetectNode.preview_once(node)
    assert view["observation"]["depth"] is None
    assert "intrinsics K" in view["depth_error"]  # A genuinely wrong model still blocks poses.
    rgb["received_at"] -= 1
    with pytest.raises(ValueError, match="stale"):
        detector.ItemDetectNode.preview_once(node)


def test_measurement_protocol_rejects_bad_geometry_and_units():
    item = {"source_index": 0, "class_id": 7, "class_name": "part", "confidence": .8,
            "rectangle": [[0, 0], [10, 0], [10, 5], [0, 5]],
            "polygon": [[0, 0], [10, 0], [10, 5]],
            "measurement": {"length_mm": 80., "width_mm": 32.}, "measurement_error": "",
            "size_valid": True, "size_reason": "Size within tolerance",
            "sampling_circle": [[1., 2.]] * 96, "sampling_circle_error": ""}
    detector.validate_preview_detections([item], 1, [7])
    for change in ({"rectangle": [[0, 0]]}, {"class_id": 0}, {"measurement": None},
                   {"measurement": {"length_mm": 10., "width_mm": 32.}},
                   {"sampling_circle": [[0, 0]]}, {"sampling_circle_error": "unexpected"},
                   {"sampling_circle": [[float("nan"), 0]] * 96},
                   {"sampling_circle": [[2_000_000_001, 0]] * 96},
                   {"polygon": [[float("nan"), 0], [1, 1], [0, 1]]}):
        with pytest.raises(RuntimeError):
            detector.validate_preview_detections([{**item, **change}], 1, [7])
    detector.validate_preview_detections(
        [{**item, "measurement": None, "measurement_error": "Missing platform"}], 1, [7])
    detector.validate_preview_detections(
        [{**item, "sampling_circle": None, "sampling_circle_error": "Missing platform"}], 1, [7])


@pytest.mark.parametrize("failure", [None, "depth", "epoch", "old", "busy", "native_id",
                                      "changed_during_call", "rejected"])
def test_selected_pose_snapshot_contract(service_node, failure):
    node, candidate = service_node
    node._camera_generation = 1
    node.settings.update(model_task="segment", geometry_source="mask",
                         geometry={"height": 100., "width": 50., "tolerance": 1.,
                                   "pickdepth_radius": 30.})
    node.settings["yolo"].update(iou=.35, image_size=640)
    rgb = {"stamp_ns": 100_000_000_000, "width": 2, "height": 2, "rgb": bytes(12)}
    depth = {"stamp_ns": 100_000_000_000, "depth": bytes(8)}
    detection = {"source_index": 4}
    observation = {"rgb": rgb, "depth": depth, "context": {"frozen": True},
                   "epoch": 1, "camera_generation": 1, "error": "Depth missing"}
    view = {"observation": observation, "metadata": {"detections": [detection]}}
    response = {"state": "ok", "generation": 1, "width": 2, "height": 2,
                "candidates": [candidate], "rejected": []}
    node.native = MagicMock(failed=False)
    node.native.call.return_value = (response, bytes(12))
    if failure == "depth":
        observation["depth"] = None
    elif failure == "epoch":
        observation["epoch"] = 0
    elif failure == "old":
        rgb["stamp_ns"] -= 3_000_000_000
    elif failure == "busy":
        node.operation_lock.acquire()
    elif failure == "native_id":
        response["candidates"] = [{**candidate, "source_index": 2}]
    elif failure == "changed_during_call":
        def change(*args):
            node.arm_epoch += 1
            return response, bytes(12)
        node.native.call.side_effect = change
    elif failure == "rejected":
        response.update(candidates=[], rejected=[{"source_index": 4, "reason": "bad depth"}])
    if failure not in (None, "rejected"):
        with pytest.raises((ValueError, RuntimeError)):
            detector.ItemDetectNode.clicked_pose(node, view, detection, node.settings)
        assert node.native.failed is (failure == "native_id")
    else:
        result = detector.ItemDetectNode.clicked_pose(node, view, detection, node.settings)
        assert result["candidate"] == (None if failure else candidate)
        sent, payload = node.native.call.call_args.args[:2]
        assert sent["context"] is observation["context"]
        assert sent["detection"] is detection
        assert payload == rgb["rgb"] + depth["depth"]
    node.infer.assert_not_called()  # No prediction, newer observation or cached service request.


def test_roi_off_uses_same_worker_and_never_infers():
    rgb = {"stamp_ns": 100_000_000_000, "received_at": time.monotonic(),
            "width": 2, "height": 2, "rgb": b"x" * 12}
    node = SimpleNamespace(
        yolo_enabled=False, applied=object(), arm_epoch=2,
        request_lock=threading.Lock(), operation_lock=threading.Lock(),
        camera_snapshot=lambda: (rgb, "live"),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_100_000_000)),
        _measurement_context=MagicMock(return_value={"projection": "synthetic"}),
        native=MagicMock(), last_view=None, events=MagicMock(), infer=MagicMock())
    node.native.call.return_value = ({"state": "ok", "generation": 2, "width": 2, "height": 2,
                                      "roi_overlay": {"visible": True, "reason": ""}}, b"y"*12)
    result = detector.ItemDetectNode.roi_once(node)
    assert result["preview_mode"] == "roi" and result["rgb"] == b"y"*12
    assert node.native.call.call_args.args[0]["operation"] == "overlay_roi"
    assert "model" not in node.native.call.call_args.args[0]
    node.infer.assert_not_called()
    node._measurement_context.side_effect = ValueError("TF missing")
    result = detector.ItemDetectNode.roi_once(node)
    assert result["rgb"] == rgb["rgb"]
    assert result["metadata"]["roi_overlay"] == {"visible": False, "reason": "TF missing"}
    assert node.native.call.call_count == 1
    rgb["received_at"] -= 2
    result = detector.ItemDetectNode.roi_once(node)
    assert "stale" in result["metadata"]["roi_overlay"]["reason"]
    node.native.call.assert_called_once()
    node.yolo_enabled = True
    assert detector.ItemDetectNode.roi_once(node) is None


def test_roi_native_malformed_result_remains_terminal():
    for status in ({"visible": True, "reason": "missing"}, {"visible": False, "reason": ""},
                   {"visible": 1, "reason": ""}, {}):
        with pytest.raises(RuntimeError, match="Malformed ROI"):
            detector.validate_roi_status(status)
