import copy
from collections import deque
from types import SimpleNamespace
import threading
import time

import numpy as np
import pytest

from item_perception_yolo.depth_snapshot import (
    median_depth_snapshot, camera_window_stationary, item_depth_limits,
    validate_depth_frame_count, DEPTH_ENCODING)
from item_perception_yolo.item_detector import ItemDetectNode
from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS


def frame(values, stamp):
    data = np.asarray([values], dtype="<u2")
    return {"width": data.shape[1], "height": 1, "stamp_ns": stamp,
            "received_at": time.monotonic(), "depth": data.tobytes()}


def test_temporal_median_majority_fractional_and_item_only_limits():
    original = copy.deepcopy(QUALITY_DEFAULTS)
    frames = [frame(values, n+1) for n, values in enumerate((
        [500, 499, 600, 500, 1000, 1001],
        [501, 501, 0, 500, 1000, 700],
        [0, 0, 0, 900, 1001, 800]))]
    result = median_depth_snapshot(frames, QUALITY_DEFAULTS)
    actual = np.frombuffer(result["depth"], "<f4")
    assert np.allclose(actual, [500.5, np.nan, np.nan, 500, 1000, 750], equal_nan=True)
    assert result["encoding"] == DEPTH_ENCODING and result["frame_stamps_ns"] == [1, 2, 3]
    assert result["stamp_ns"] == 3 and isinstance(result["depth"], bytes)
    assert QUALITY_DEFAULTS == original and QUALITY_DEFAULTS["depth_min_mm"] == 200.
    assert np.frombuffer(frames[0]["depth"], "<u2")[1] == 499


@pytest.mark.parametrize("count", [1, 3, 5])
def test_strict_majority_for_supported_counts(count):
    frames = [frame([600 if i < count//2+1 else 0, 600 if i < count//2 else 0], i+1)
              for i in range(count)]
    actual = np.frombuffer(median_depth_snapshot(frames, QUALITY_DEFAULTS)["depth"], "<f4")
    assert actual[0] == 600 and np.isnan(actual[1])


@pytest.mark.parametrize("value", [0, 2, 4, 6, True, 3., "3", None])
def test_frame_count_is_explicit_odd_bounded_integer(value):
    with pytest.raises(ValueError):
        validate_depth_frame_count(value)


def test_duplicate_depth_stamps_and_layout_changes_reject():
    frames = [frame([600], stamp) for stamp in (1, 1, 3)]
    with pytest.raises(ValueError, match="advancing"):
        median_depth_snapshot(frames, QUALITY_DEFAULTS)
    frames[1] = frame([600, 600], 2)
    with pytest.raises(ValueError, match="layout"):
        median_depth_snapshot(frames, QUALITY_DEFAULTS)
    assert item_depth_limits({**QUALITY_DEFAULTS, "depth_min_mm": 650}) == (650, 1000.)
    assert item_depth_limits({**QUALITY_DEFAULTS, "depth_max_mm": 500}) == (500., 500.)
    with pytest.raises(ValueError):
        item_depth_limits({**QUALITY_DEFAULTS, "depth_max_mm": 499})


def test_stationarity_uses_window_anchor_translation_and_rotation():
    base = np.eye(4)
    tiny = base.copy()
    tiny[0, 3] = .00004
    moved = base.copy()
    moved[0, 3] = .00008
    assert camera_window_stationary([base, tiny])
    assert not camera_window_stationary([base, tiny, moved])
    angle = np.radians(.051)
    rotation = base.copy()
    rotation[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    assert not camera_window_stationary([base, rotation])


def window_node():
    info = {"width": 1, "height": 1, "k": [1], "d": [0]}
    frames = [frame([600], stamp) for stamp in (100_000_000_000, 100_033_000_000, 100_066_000_000)]
    for value in frames:
        value.update(color_info=copy.deepcopy(info), depth_info=copy.deepcopy(info))
    node = SimpleNamespace(_depth_history=deque(frames, maxlen=5), _color_info=info,
                           _depth_info=info, condition=threading.Condition(),
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                               nanoseconds=100_080_000_000)))
    rgb = {**frames[-1], "rgb": b"\0\0\0"}
    return node, rgb


def test_capture_history_must_be_post_trigger_synchronized_and_same_calibration():
    node, rgb = window_node()
    frames, _, _ = ItemDetectNode._depth_window(node, rgb, QUALITY_DEFAULTS, 3, 99_999_999_999)
    assert len(frames) == 3
    with pytest.raises(ValueError, match="after request"):
        ItemDetectNode._depth_window(node, rgb, QUALITY_DEFAULTS, 3, 100_000_000_000)
    with pytest.raises(ValueError, match="after request"):
        ItemDetectNode._depth_window(node, rgb, QUALITY_DEFAULTS, 3, 100_001_000_000)
    node._depth_history[0]["color_info"]["d"] = [1]
    with pytest.raises(ValueError, match="CameraInfo"):
        ItemDetectNode._depth_window(node, rgb, QUALITY_DEFAULTS, 3)
    node._depth_history[0]["color_info"]["d"] = [0]
    node._depth_history[0]["stamp_ns"] -= 200_000_000
    with pytest.raises(ValueError, match="synchronization"):
        ItemDetectNode._depth_window(node, rgb, QUALITY_DEFAULTS, 3)


def test_five_frame_capture_uses_midpoint_rgb_without_relaxing_sync():
    node, rgb = window_node()
    start = node._depth_history[0]["stamp_ns"]
    node._depth_history = deque([{**node._depth_history[0], "stamp_ns": start+i*33_000_000}
                                 for i in range(5)], maxlen=5)
    node._rgb_history = deque([{**rgb, "stamp_ns": f["stamp_ns"]}
                              for f in node._depth_history], maxlen=7)
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=start+140_000_000))
    with pytest.raises(ValueError, match="synchronization"):
        ItemDetectNode._depth_window(node, node._rgb_history[-1], QUALITY_DEFAULTS, 5, start-1)
    selected = ItemDetectNode._bundle_rgb(node, node._rgb_history[-1], 5, start-1)
    assert selected["stamp_ns"] == start+66_000_000
    frames, _, _ = ItemDetectNode._depth_window(node, selected, QUALITY_DEFAULTS, 5, start-1)
    assert len(frames) == 5


def test_stationarity_bounds_every_pair_and_invalid_transform():
    left = np.eye(4)
    left[0, 3] = -.00004
    right = np.eye(4)
    right[0, 3] = .00004
    assert not camera_window_stationary([np.eye(4), left, right])
    assert not camera_window_stationary([np.full((4, 4), np.nan)])
    assert not camera_window_stationary([])


def test_moving_camera_reacquires_bundle_within_original_deadline():
    from unittest.mock import Mock
    node, rgb = window_node()
    node.settings = {"quality": dict(QUALITY_DEFAULTS), "geometry": {"depth_frame_count": 3}}
    node.applied = SimpleNamespace(
        camera=SimpleNamespace(settings=SimpleNamespace(camera_prefix="bin")))
    node.camera_prefix = "bin"
    node.arm_epoch = 1
    node._image = rgb
    node._bundle_rgb = lambda image, *args: image
    node._depth_window = lambda *args: ItemDetectNode._depth_window(node, *args)
    node._snapshot_context = Mock(side_effect=[ValueError("Camera moved"), {"fixed": True}])
    observed_rgb, depth, context = ItemDetectNode._snapshot(
        node, 0, time.monotonic()+.5, wait=True)
    assert node._snapshot_context.call_count == 2 and context == {"fixed": True}
    assert observed_rgb == rgb
    assert depth["frame_stamps_ns"] == [f["stamp_ns"] for f in node._depth_history]
    node._snapshot_context = Mock(side_effect=ValueError("Camera moved"))
    with pytest.raises(ValueError):
        ItemDetectNode._snapshot(node, 0, time.monotonic()+.04, wait=True)


def test_immediate_item_retries_cannot_reuse_any_rgb_or_depth_frame():
    node, rgb = window_node()
    node.settings = {"quality": dict(QUALITY_DEFAULTS), "geometry": {"depth_frame_count": 3}}
    node.applied = SimpleNamespace(
        camera=SimpleNamespace(settings=SimpleNamespace(camera_prefix="bin")))
    node.camera_prefix, node.arm_epoch = "bin", 1
    node._image = rgb
    node._bundle_rgb = lambda image, *args: ItemDetectNode._bundle_rgb(node, image, *args)
    node._depth_window = lambda *args: ItemDetectNode._depth_window(node, *args)
    node._snapshot_context = lambda *_args: {}
    used_rgb, used_depth = set(), set()
    start = 99_999_999_999
    for _attempt in range(5):
        observed, depth, _ = ItemDetectNode._snapshot(
            node, start, time.monotonic()+1, wait=False)
        assert observed["stamp_ns"] > start and observed["stamp_ns"] not in used_rgb
        assert min(depth["frame_stamps_ns"]) > start
        assert not used_depth.intersection(depth["frame_stamps_ns"])
        used_rgb.add(observed["stamp_ns"])
        used_depth.update(depth["frame_stamps_ns"])
        start = node.get_clock().now().nanoseconds
        # No new publication: even an immediate retry must wait, not reuse the buffer.
        with pytest.raises(ValueError, match="new RGB"):
            ItemDetectNode._snapshot(node, start, time.monotonic()+1, wait=False)
        node._image = {**rgb, "stamp_ns": start}  # Equal boundary also predates capture.
        with pytest.raises(ValueError, match="new RGB"):
            ItemDetectNode._snapshot(node, start, time.monotonic()+1, wait=False)
        frames = [{**f, "stamp_ns": start+(i+1)*20_000_000}
                  for i, f in enumerate(node._depth_history)]
        node._depth_history = deque(frames, maxlen=5)
        node._image = {**rgb, "stamp_ns": frames[-1]["stamp_ns"]}
        node._rgb_history = deque([node._image])
        now = start + 80_000_000
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now))


def test_production_status_only_suppresses_fresh_active_pick_place_or_auto():
    from item_perception_yolo.item_teach_gui import ItemTeachNode
    node = SimpleNamespace(_production_status=None,
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                               nanoseconds=100_000_000_000)))
    assert not ItemTeachNode.background_suspended(node)
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=100, nanosec=0)),
                              auto_run_active=False, operation_active=True, operation="pick")
    for operation in ("pick", "place", "auto_run"):
        message.operation = operation
        node._production_status = (message, time.monotonic())
        assert ItemTeachNode.background_suspended(node)
    message.operation_active = False
    assert not ItemTeachNode.background_suspended(node)
    message.auto_run_active = True  # Includes waiting for detections and between phases.
    assert ItemTeachNode.background_suspended(node)
    node._production_status = (message, time.monotonic()-1.01)
    assert not ItemTeachNode.background_suspended(node)
    node._production_status = (message, time.monotonic())
    message.header.stamp.sec = 98
    assert not ItemTeachNode.background_suspended(node)


def test_active_production_starts_no_new_preview_or_voxel_job():
    from unittest.mock import Mock
    from item_perception_yolo.item_teach_rviz import TeachingRvizPreview
    node = SimpleNamespace(yolo_enabled=True, request_lock=threading.Lock(),
                           background_suspended=lambda: True, operation_lock=Mock(), native=Mock())
    assert ItemDetectNode.preview_once(node) is None
    preview = SimpleNamespace(node=node)
    result = TeachingRvizPreview.compute(preview, {"rgb": b"capture"}, {})
    assert "error" in result
    node.operation_lock.acquire.assert_not_called()
    node.native.call.assert_not_called()


def test_camera_info_change_invalidates_history_and_inflight_generation(monkeypatch):
    from unittest.mock import Mock
    from item_perception_yolo import item_detector
    node, _ = window_node()
    node._camera_generation = 1
    node._input_revision = 0
    node.arm_epoch = 4
    node._rgb_history = deque([{}], maxlen=7)
    node.last_view = {"old": True}
    node.disarm = Mock()
    node.events = Mock()
    changed = {**node._color_info, "d": [1]}
    monkeypatch.setattr(item_detector, "validate_camera_info", lambda *args: changed)
    ItemDetectNode._receive(node, object(), "bin", 1, "color_info")
    assert node._input_revision == 1 and node.arm_epoch == 5
    assert not node._depth_history and not node._rgb_history and node.last_view is None
    node.disarm.assert_called_once_with(
        "color_info CameraInfo changed: d", stream="color_info",
        changed_fields={"d": {"old": [0], "new": [1]}})


@pytest.mark.parametrize("kind", ["color_info", "depth_info"])
def test_repeated_camera_info_timestamps_do_not_disarm_or_clear_history(kind):
    from unittest.mock import Mock
    from sensor_msgs.msg import CameraInfo
    from item_perception_yolo.item_detector import validate_camera_info
    node, _ = window_node()
    message = CameraInfo(width=424, height=240, distortion_model="plumb_bob",
                         k=[230., 0., 212., 0., 230., 120., 0., 0., 1.], d=[0.]*8)
    message.header.frame_id = "bin_color_optical_frame"
    message.header.stamp.sec = 100
    setattr(node, "_" + kind, validate_camera_info(message, "bin"))
    node._camera_generation = 1
    node._input_revision = 0
    node.arm_epoch = 4
    node._rgb_history = deque([{}], maxlen=7)
    node.last_view = {"old": True}
    node.disarm = Mock()
    node.events = Mock()
    for ns in range(0, 1_000_000_000, 33_333_333):
        message.header.stamp.nanosec = ns
        ItemDetectNode._receive(node, message, "bin", 1, kind)
    assert node.arm_epoch == 4 and node._input_revision == 0
    assert len(node._depth_history) == 3 and len(node._rgb_history) == 1
    assert node.last_view == {"old": True}
    node.disarm.assert_not_called()


def test_invalid_camera_info_reports_error_without_rearming(monkeypatch):
    from unittest.mock import Mock
    from item_perception_yolo import item_detector
    node, _ = window_node()
    node._camera_generation = 1
    node._input_revision = 0
    node.arm_epoch = 4
    node._rgb_history = deque([{}], maxlen=7)
    node.disarm = Mock()
    node.events = Mock()
    monkeypatch.setattr(item_detector, "validate_camera_info",
                        Mock(side_effect=ValueError("Unexpected optical frame")))
    ItemDetectNode._receive(node, object(), "bin", 1, "depth_info")
    node.disarm.assert_called_once_with(
        "Invalid depth_info CameraInfo: Unexpected optical frame", stream="depth_info")
    assert node._depth_info is None and not node._depth_history
