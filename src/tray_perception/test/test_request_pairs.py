"""Shared service/simulation pairing, interruption and second-frame depth ownership."""
import copy
import json
import time
from unittest.mock import MagicMock

import pytest
from tray_perception_interfaces.srv import GetTrayPose
from tray_perception_interfaces.msg import PlacementDepthRequest
from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
from tray_perception import requests
from test_requests import backend, artifact, arm, trigger, short_deadline


@pytest.mark.parametrize("sequence,frames", [([0., .02, .021], 3), ([0., None, .02, .021], 4)])
def test_mismatch_retains_newer_seed_and_empty_breaks_pair(backend, sequence, frames):
    node, _, _ = backend
    arm(backend)
    ordinary = node.preview.side_effect
    offsets = iter(sequence)

    def preview(*args, **kw):
        view = ordinary(*args, **kw)
        x = next(offsets)
        candidate = view["result"]["selected"]
        if x is None:
            view["result"]["selected"] = None
        else:
            candidate["position"][0] += x
            for corner in candidate["corners_base_m"]:
                corner[0] += x
        return view
    node.preview.side_effect = preview
    reply = trigger(backend)
    assert reply.success and reply.found
    assert reply.tray.pose.position.x == pytest.approx(.121)  # No averaging.
    proof = json.loads(reply.diagnostics_json)["stability"]
    assert proof["first"]["position"][0] == pytest.approx(.12)
    assert proof["processed_frames"] == node.preview.call_count == frames
    assert proof["second"]["stamp_ns"] - proof["first"]["stamp_ns"] == 1
    node.render_capture.assert_not_called()


def test_placement_samples_only_second_snapshot(backend):
    node, _, digest = backend
    arm(backend)
    ordinary = node.snapshot.side_effect
    snapshots = []

    def snapshot(**_kw):
        view = ordinary()
        view["depth"] = {"stamp_ns": view["rgb"]["stamp_ns"],
                         "received_at": time.monotonic(),
                         "depth": bytes([len(snapshots) + 1]) * 8}
        snapshots.append(copy.deepcopy(view))
        return view
    node.snapshot.side_effect = snapshot
    node.native.call = MagicMock(return_value=({"state": "ok", "surface_base": [.12, .13, .24],
        "accepted_samples": 50, "total_samples": 60, "median_mm": 700., "sigma_mm": 1.}, b""))
    reply = node.requests.handle(GetTrayPose.Request(profile_sha256=digest,
        sample_placement_depth=True, placement=PlacementDepthRequest(
            x_mm=20., y_mm=30., diameter_mm=30., **QUALITY_DEFAULTS)), GetTrayPose.Response())
    assert reply.found and reply.placement.valid
    assert len(snapshots) == 2 and node.native.call.call_count == 1
    assert node.native.call.call_args.args[1] == snapshots[1]["depth"]["depth"]
    assert reply.placement.depth_header.stamp == reply.header.stamp


@pytest.mark.parametrize("change", ["disarm", "camera", "repeat"])
def test_interrupted_or_nonadvancing_pair_cannot_authorize_pose(backend, change):
    node, _, _ = backend
    arm(backend)
    ordinary = node.preview.side_effect
    snapshot = node.snapshot.side_effect

    def preview(*args, **kw):
        view = ordinary(*args, **kw)
        if node.preview.call_count == 1:
            if change == "disarm":
                node.requests.disarm()
            elif change == "camera":
                def changed():
                    frame = snapshot()
                    frame["info"]["k"][0] += 1.
                    return frame
                node.snapshot.side_effect = changed
            else:
                node.snapshot.side_effect = lambda: kw["view"]
        return view
    node.preview.side_effect = preview
    reply = trigger(backend)
    assert not reply.found
    assert reply.status == ("NO_VALID_TRAY" if change == "repeat" else "ERROR")
    node.render_capture.assert_not_called()


def test_mismatch_deadline_is_not_reset_and_no_render_occurs(backend, monkeypatch):
    node, _, _ = backend
    arm(backend)
    ordinary = node.preview.side_effect
    monkeypatch.setattr(requests, "REQUEST_TIMEOUT", .06)

    def preview(*args, **kw):
        time.sleep(.01)
        view = ordinary(*args, **kw)
        offset = .1 * (node.preview.call_count % 2)
        view["result"]["selected"]["position"][0] += offset
        for c in view["result"]["selected"]["corners_base_m"]:
            c[0] += offset
        return view
    node.preview.side_effect = preview
    started = time.monotonic()
    reply = trigger(backend)
    assert time.monotonic() - started < .3
    assert reply.status == "NO_VALID_TRAY" and not reply.found
    assert "deadline" in reply.message
    assert node.preview.call_count >= 2
    node.render_capture.assert_not_called()


def test_stale_processed_frame_breaks_pair_and_restarts_with_new_rgb(backend):
    node, _, _ = backend
    arm(backend)
    ordinary = node.preview.side_effect

    def preview(*args, **kw):
        if node.preview.call_count == 2:
            raise ValueError('Processed RGB observation became stale')
        return ordinary(*args, **kw)
    node.preview.side_effect = preview
    reply = trigger(backend)
    assert reply.found and node.preview.call_count == 4
    proof = json.loads(reply.diagnostics_json)['stability']
    assert proof['first']['stamp_ns'] == node.preview.call_args_list[2].kwargs['view']['rgb']['stamp_ns']


def test_native_deadline_is_empty_result_without_disarming(backend):
    node, _, _ = backend
    arm(backend)
    node.preview.side_effect = TimeoutError('Native request expired; previous operation is retiring')
    reply = trigger(backend)
    assert reply.status == 'NO_VALID_TRAY' and not reply.found
    assert 'retiring' in reply.message
    assert node.requests.service is not None and not node.native.failed


def test_capture_waits_for_native_retirement_and_stays_cancellable(backend, monkeypatch):
    node, _, _ = backend
    arm(backend)
    node.native.retiring = True
    node.snapshot.reset_mock()
    assert node.requests.busy
    original_sleep = time.sleep

    def disarm_on_wait(seconds):
        node.requests.disarm('Stop while native result retires')
        original_sleep(min(seconds, .001))
    monkeypatch.setattr(requests.time, 'sleep', disarm_on_wait)
    reply = trigger(backend)
    assert not reply.success and 'cancelled' in reply.message
    node.snapshot.assert_not_called()
    node.preview.assert_not_called()
