import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
from rclpy.time import Time
from tray_perception_interfaces.srv import GetTrayPose
from tray_perception_interfaces.msg import PlacementDepthRequest
from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS

from item_perception_yolo.runtime_teach import runtime_teach_catalog, runtime_tray_catalog
from tray_perception import core, requests
from tray_perception import documents
from tray_perception.placement import sampling_from_item
from test_documents import ready_form
from test_core import artifact as saved_artifact
from test_core import camera_info, plane, position, settings


@pytest.fixture
def artifact(tmp_path):
    return saved_artifact.__wrapped__(tmp_path)


def selected():
    return {"valid": True, "source_index": 2, "class_id": 0, "class_name": "tray",
            "confidence": .9, "position": [.1, .1, .2], "quaternion": [0., 0., 0., 1.],
            "length_mm": 200., "width_mm": 100., "center_distance_px": 4.,
            "corners_base_m": [[.1, .1, .2], [.2, .1, .2], [.2, .3, .2], [.1, .3, .2]],
            "reason": "Dimensions within tolerance"}


@pytest.fixture
def backend(tmp_path, artifact):
    path, camera, model = artifact
    camera.settings = SimpleNamespace(camera_prefix="cam")
    node = SimpleNamespace(
        root=tmp_path, deployment=False, yolo_enabled=True, native=SimpleNamespace(
            failed=False, closed=False), fatal_error="", work_lock=threading.RLock(),
        lock=threading.RLock(), generation=4, camera=camera, camera_prefix="cam",
        model={"path": str(model), "sha256": core.file_sha256(model), "task": "segment"},
        model_metadata={"classes": {"0": "tray", "1": "other"}},
        plane=plane(), position=position(),
        validate_sources=MagicMock(), _check_snapshot=MagicMock(), events=MagicMock(),
        create_service=MagicMock(return_value=object()), destroy_service=MagicMock(),
        get_name=lambda: "tray_teach", get_namespace=lambda: "/",
        get_node_names_and_namespaces=lambda: [("tray_teach", "/")],
        get_clock=lambda: SimpleNamespace(now=lambda: Time(seconds=100)))

    def snapshot():
        return {"generation": node.generation, "connection": 1, "depth": None, "depth_info": None,
                "info": camera_info(), "metric_error": "", "depth_error": "Depth unavailable",
                "rgb": {"width": 2, "height": 2, "stamp_ns": 100_000_000_001,
                        "received_at": time.monotonic(), "rgb": bytes(12)},
                "camera_context": {"camera": camera_info(), "depth_camera": None,
                                   "base_from_optical": np.eye(4).tolist()}}

    node.snapshot = MagicMock(side_effect=snapshot)

    def preview(_settings, *, view, **_kwargs):
        result = {"count": 3, "detections": [selected()],
                  "selected": selected(), "inference_ms": 2.}
        return {**view, "result": result, "overlay": bytes(12), "depth_overlay": b"",
                "cloud": None, "samples": []}

    node.preview = MagicMock(side_effect=preview)
    node.requests = requests.TrayRequests(node)

    def invalidate(reason):
        node.generation += 1
        node.requests.disarm(reason)
    node.invalidate = invalidate
    return node, path, core.file_sha256(path)


def arm(backend):
    node, path, digest = backend
    node.requests.arm(path, settings(), digest)
    return node.requests


def trigger(backend, digest=None, debug=False):
    node, _, expected = backend
    return node.requests.handle(GetTrayPose.Request(
        profile_sha256=expected if digest is None else digest, save_debug_images=debug),
        GetTrayPose.Response())


@pytest.mark.parametrize("simulated", [False, True])
def test_cleanup_evidence_survives_request_and_event_log(backend, simulated):
    node, path, digest = backend
    preview = node.preview.side_effect
    cleaning = {"status": "cleaned", "retained_fraction": .95, "removed_pixels": 50}

    def cleaned(*args, **kwargs):
        view = preview(*args, **kwargs)
        view["result"]["selected"]["mask_clean"] = cleaning
        view["result"]["detections"][0]["mask_clean"] = cleaning
        return view

    node.preview.side_effect = cleaned
    arm(backend)
    if simulated:
        requested_at = (node.get_clock().now().nanoseconds, time.monotonic())
        response = node.requests.simulate(path, settings(), digest, requested_at)["response"]
    else:
        response = trigger(backend)
    assert response.success and response.found
    assert json.loads(response.diagnostics_json)["detections"][0]["mask_clean"] == cleaning
    event = "tray_simulated" if simulated else "tray_pose_response"
    logged = next(call for call in node.events.record.call_args_list if call.args[1] == event)
    assert logged.kwargs["returned_tray"]["mask_clean"] == cleaning


@pytest.mark.parametrize("usable", [True, False])
@pytest.mark.parametrize("save_images", [False, True])
def test_placement_request_uses_one_fresh_observation_and_reports_depth(
        backend, usable, save_images):
    node, path, digest = backend
    ordinary = node.snapshot.side_effect

    def snapshot(**kwargs):
        view = ordinary()
        view["depth"] = {"stamp_ns": view["rgb"]["stamp_ns"],
                         "received_at": view["rgb"]["received_at"], "depth": bytes(8)}
        return view

    node.snapshot.side_effect = snapshot
    preview = node.preview.side_effect
    node.preview.side_effect = lambda *args, **kwargs: {
        **preview(*args, **kwargs), "depth_overlay": bytes(12)}
    sample = {"x_mm": 20., "y_mm": 30., "diameter_mm": 30., **QUALITY_DEFAULTS}
    result = ({"state": "ok", "surface_base": [.12, .13, .24],
               "accepted_samples": 50, "total_samples": 60, "median_mm": 700., "sigma_mm": 1.}
              if usable else {"state": "ok", "error": "Insufficient placement depth"})
    node.native.call = MagicMock(return_value=(result, b""))
    arm(backend)
    response = node.requests.handle(GetTrayPose.Request(
        profile_sha256=digest, sample_placement_depth=True, save_debug_images=save_images,
        placement=PlacementDepthRequest(**sample)), GetTrayPose.Response())
    assert response.success is usable and response.placement.valid is usable
    node.snapshot.assert_called_with(depth_required=True, quality=sample)
    node.preview.assert_called_once()
    assert node.preview.call_args.kwargs["visualize"] is save_images
    assert node.native.call.call_args.args[0]["sampling"] == sample
    if usable:
        assert response.placement.surface_base.z == .24
        assert json.loads(response.diagnostics_json)["placement_sampling"] == sample
        capture = json.loads(response.diagnostics_json)["debug_capture"]
        assert capture["requested"] is save_images
        assert not capture["error"]
        if save_images:
            for key in ("rgb_path", "depth_path"):
                image = Path(capture[key])
                assert image.parent == node.root / "debug/tray_img"
                assert image.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
        else:
            assert not capture["rgb_path"] and not capture["depth_path"]
            assert not (node.root / "debug").exists()
    else:
        assert "Insufficient" in response.message and not node.native.failed
        assert node.requests.service is not None  # Bad depth must not disarm the provider.
    records = node.events.record.call_args_list
    started = next(call for call in records if call.args[1] == 'tray_request_started')
    finished = next(call for call in records if call.args[1] == (
        'tray_pose_response' if usable else 'tray_request_failed'))
    assert started.kwargs['request_id'] == finished.kwargs['request_id']
    assert finished.kwargs['elapsed_sec'] >= 0
    if not usable:
        assert finished.kwargs['phase'] == 'placement_depth'
        assert 'Insufficient placement depth' in finished.kwargs['traceback']


@pytest.mark.parametrize("last_usable", [False, True])
def test_depth_failure_restarts_pose_and_depth_from_new_observation(backend, last_usable):
    node, _, digest = backend
    ordinary_snapshot = node.snapshot.side_effect
    ordinary_preview = node.preview.side_effect
    attempt = 0
    observations, order = [], []

    def snapshot(**kwargs):
        view = ordinary_snapshot()
        view["rgb"]["stamp_ns"] += attempt * 1_000_000_000
        view["depth"] = {"stamp_ns": view["rgb"]["stamp_ns"],
                         "received_at": view["rgb"]["received_at"],
                         "depth": bytes([attempt + 1]) * 8}
        observations.append(view)
        return view

    def preview(*args, **kwargs):
        order.append("pose")
        view = ordinary_preview(*args, **kwargs)
        # A changed tray position exposes accidental reuse of the old pose.
        view["result"]["selected"]["position"][0] += attempt * .01
        for corner in view["result"]["selected"]["corners_base_m"]:
            corner[0] += attempt * .01
        return view

    def depth(request, data, **kwargs):
        order.append("depth")
        assert request["selected"]["position"][0] == pytest.approx(.1 + attempt * .01)
        assert data == bytes([attempt + 1]) * 8
        if attempt < 2 or not last_usable:
            return {"state": "ok",
                    "error": "Insufficient accepted placement depth samples/fraction"}, b""
        return {"state": "ok", "surface_base": [.14, .13, .24],
                "accepted_samples": 50, "total_samples": 60,
                "median_mm": 700., "sigma_mm": 1.}, b""

    node.snapshot.side_effect = snapshot
    node.preview.side_effect = preview
    node.native.call = MagicMock(side_effect=depth)
    arm(backend)
    observations.clear()  # Arming performs its own read-only readiness snapshot.
    sample = {"x_mm": 20., "y_mm": 30., "diameter_mm": 30., **QUALITY_DEFAULTS}
    for attempt in range(3):
        node.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=100 + attempt))
        response = node.requests.handle(GetTrayPose.Request(
            profile_sha256=digest, sample_placement_depth=True,
            placement=PlacementDepthRequest(**sample)), GetTrayPose.Response())
        assert response.success is (attempt == 2 and last_usable)
        assert response.placement.valid is response.success
        if not response.success:
            assert response.message == "Insufficient accepted placement depth samples/fraction"
        assert observations[-1]["rgb"]["stamp_ns"] > (100 + attempt) * 1_000_000_000
        assert node.requests.service is not None and not node.native.failed
    assert order == ["pose", "depth"] * 3
    assert len(observations) == 3
    assert node.preview.call_count == node.native.call.call_count == 3


def test_armed_and_simulated_requests_share_fresh_single_pose_pipeline(backend):
    node, path, digest = backend
    api = node.requests
    local = api.simulate(path, settings(), digest, (100_000_000_000, time.monotonic()))
    assert local["response"].success and local["response"].found
    node.create_service.assert_not_called()
    api.validate_view(local["view"])
    arm(backend)
    actual = trigger(backend)
    assert actual.success and actual.found and actual.status == "OK"
    assert actual.header.frame_id == "base_link" and actual.header.stamp.nanosec == 1
    assert actual.tray.pose == local["response"].tray.pose
    assert actual.tray.length == .2 and actual.tray.width == .1
    assert actual.tray.extent_x == pytest.approx(.1)
    assert actual.tray.extent_y == pytest.approx(.2)
    assert actual.tray.id.startswith(actual.batch_id + ":")
    assert actual.batch_id != local["response"].batch_id
    assert actual.valid_count == 1 and actual.detected_count == 3
    assert node.preview.call_count == 2  # One inference for each trigger.
    assert node.preview.call_args.args[0]["yolo"]["class_ids"] == [0, 1]
    assert node.preview.call_args.args[0]["accepted_class_ids"] == [0]
    assert not node.preview.call_args.kwargs["visualize"]
    assert not (node.root / "debug").exists()
    assert json.loads(actual.diagnostics_json)["profile_sha256"] == digest


def test_stale_rgb_cannot_satisfy_trigger_and_fresh_depth_is_not_required(backend):
    node, _, _ = backend
    arm(backend)
    fresh = node.snapshot.side_effect
    calls = []

    def snapshots():
        view = fresh()
        if len(calls) < 2:
            view["rgb"]["stamp_ns"] = 99_000_000_000
        calls.append(view)
        return view

    node.snapshot.side_effect = snapshots
    response = trigger(backend)
    assert response.success and len(calls) == 3
    assert node.preview.call_args.kwargs["view"]["depth"] is None
    assert node.preview.call_args.kwargs["view"]["rgb"]["stamp_ns"] > 100_000_000_000


@pytest.mark.parametrize("stale_stream", ["rgb", "depth"])
def test_immediate_tray_retries_require_new_raw_rgb_and_depth(backend, monkeypatch, stale_stream):
    node, _, _ = backend
    ordinary = node.snapshot.side_effect
    used = {"rgb": set(), "depth": set()}
    monkeypatch.setattr(requests.time, "sleep", lambda _seconds: None)
    for attempt in range(5):
        start_ns = 100_000_000_000 + attempt*100_000_000
        started = time.monotonic()
        view = ordinary()
        view["rgb"].update(stamp_ns=start_ns+1, received_at=started)
        view["depth"] = {"stamp_ns": start_ns+1, "received_at": started, "depth": bytes(8)}
        stale = {**view, stale_stream: {**view[stale_stream], "stamp_ns": start_ns}}
        if attempt:
            old = {**view, stale_stream: {
                **view[stale_stream], "stamp_ns": max(used[stale_stream])}}
            node.snapshot.side_effect = [old, stale, view]
        else:
            node.snapshot.side_effect = [stale, view]
        captured = node.requests._fresh_view(
            start_ns, started, started+1., lambda: None, sampling=dict(QUALITY_DEFAULTS))
        assert captured is view
        for stream in used:
            stamp = captured[stream]["stamp_ns"]
            assert stamp > start_ns and stamp not in used[stream]
            used[stream].add(stamp)


@pytest.mark.parametrize("sample_depth", [False, True])
def test_saved_gui_and_headless_tray_pose_and_depth_match(backend, sample_depth):
    node, path, digest = backend
    runtime = node.root / "runtime_teach"
    runtime.mkdir()
    for source in (path, path.with_suffix(".pt")):
        shutil.copy2(source, runtime / source.name)
    ordinary = node.snapshot.side_effect

    def snapshot(**kwargs):
        view = ordinary()
        if sample_depth:
            view["depth"] = {"stamp_ns": view["rgb"]["stamp_ns"],
                             "received_at": view["rgb"]["received_at"], "depth": bytes(8)}
        return view

    node.snapshot.side_effect = snapshot
    node.native.call = MagicMock(return_value=({
        "state": "ok", "surface_base": [.12, .13, .24], "accepted_samples": 2,
        "total_samples": 10, "median_mm": 700., "sigma_mm": 1.}, b""))
    sample = sampling_from_item({"geometry": {"pickdepth_radius": 30.},
                                 "quality": dict(QUALITY_DEFAULTS)}, 20., 30.)
    responses = []
    for deployment in (False, True):
        node.requests.disarm("Switch synthetic provider")
        node.deployment = deployment
        name = "tray_detect" if deployment else "tray_teach"
        node.get_name = lambda: name
        node.get_node_names_and_namespaces = lambda: [(name, "/")]
        current = runtime / path.name if deployment else path
        node.model["path"] = str(current.with_suffix(".pt"))
        node.requests.arm(current, settings(), digest)
        response = node.requests.handle(GetTrayPose.Request(
            profile_sha256=digest, sample_placement_depth=sample_depth,
            placement=PlacementDepthRequest(**sample)), GetTrayPose.Response())
        assert response.success and response.found
        assert response.placement.valid is sample_depth
        if sample_depth:
            evidence = json.loads(response.diagnostics_json)
            assert evidence["placement_sampling"]["minimum_depth_fraction"] == .2
        responses.append(response)
    assert responses[0].batch_id != responses[1].batch_id
    for response in responses:
        response.batch_id = ""
        response.tray.id = response.tray.id.split(":", 1)[1]
    assert responses[0] == responses[1]
    assert node.preview.call_count == 2
    assert node.native.call.call_count == (2 if sample_depth else 0)
    assert node.preview.call_args_list[0].args == node.preview.call_args_list[1].args


def test_no_detection_is_explicit_success_without_a_pose(backend):
    node, _, _ = backend
    arm(backend)
    original = node.preview.side_effect

    def empty(*args, **kwargs):
        value = original(*args, **kwargs)
        value["result"].update(selected=None, detections=[])
        return value
    node.preview.side_effect = empty
    response = trigger(backend)
    assert response.success and not response.found and response.status == "NO_VALID_TRAY"
    assert not response.tray.id and response.valid_count == 0


@pytest.mark.parametrize("old_draft", [False, True])
def test_no_position_or_detection_needed_to_arm_loaded_file_and_simulate(backend, old_draft):
    node, path, _ = backend
    _, target = documents.open_document(path, node.root)
    path, document, target, _ = documents.save_document(
        ready_form("cam"), None if old_draft else settings(), None, node.plane,
        node.camera, node.model, node.root, target=target)
    original = path.read_bytes()
    node.position = None
    current = node, path, target.yaml_sha256
    api = arm(current)
    node.preview.assert_not_called()  # Arming needs no visible tray or inference.
    assert api.service is not None
    assert trigger(current).found
    local = api.simulate(path, settings(), target.yaml_sha256,
                         (100_000_000_000, time.monotonic()))
    assert local["response"].found and path.read_bytes() == original
    assert document["tray_teach_position"] is None


@pytest.mark.parametrize("changed", ["width_mm", "class_ids", "prefix", "model", "plane"])
def test_old_draft_detection_checks_still_bind_saved_fields_and_sources(backend, changed):
    node, path, _ = backend
    _, target = documents.open_document(path, node.root)
    state = ready_form("other" if changed == "prefix" else "cam")
    if changed in ("width_mm", "class_ids"):
        state["draft"][changed] = [] if changed == "class_ids" else ""
    path, _, target, _ = documents.save_document(
        state, None, None, None if changed == "plane" else node.plane,
        node.camera, node.model, node.root, target=target)
    if changed == "model":
        path.with_suffix(".pt").write_bytes(b"unexpected replacement")
    with pytest.raises(ValueError):
        arm((node, path, target.yaml_sha256))
    node.create_service.assert_not_called()
    node.preview.assert_not_called()


@pytest.mark.parametrize("cause", ["hash", "disarm", "busy", "settings", "timeout"])
def test_rejected_requests_never_infer_or_return_cached_pose(backend, monkeypatch, cause):
    node, _, _ = backend
    api = arm(backend)
    if cause == "disarm":
        api.disarm()
    elif cause == "busy":
        api.request_lock.acquire()
    elif cause == "settings":
        node.plane = None
    elif cause == "timeout":
        monkeypatch.setattr(requests, "REQUEST_TIMEOUT", .02)
        node.snapshot.side_effect = ValueError("RGB not available")
    try:
        response = trigger(backend, "f" * 64 if cause == "hash" else None)
        assert not response.success and not response.found and not response.tray.id
        if cause == "busy":
            assert response.status == "BUSY"
        node.preview.assert_not_called()
    finally:
        if cause == "busy":
            api.request_lock.release()


@pytest.mark.parametrize("cause", ["disarm", "yaml", "weights", "native"])
def test_inflight_change_rejects_completed_result(backend, cause):
    node, path, _ = backend
    api = arm(backend)
    original = node.preview.side_effect

    def changed(*args, **kwargs):
        value = original(*args, **kwargs)
        if cause == "disarm":
            api.disarm()
        elif cause == "yaml":
            path.write_text(path.read_text() + "\n# changed\n")
        elif cause == "weights":
            path.with_suffix(".pt").write_bytes(b"changed model")
        else:
            raise RuntimeError("Native worker failure")
        return value
    node.preview.side_effect = changed
    response = trigger(backend)
    assert not response.success and not response.found and not response.tray.id
    api.tick()
    assert api.service is None
    assert bool(node.fatal_error) == (cause == "native")


def test_debug_images_only_on_request_and_missing_depth_is_nonfatal(backend):
    node, _, _ = backend
    arm(backend)
    assert trigger(backend).success and not (node.root / "debug").exists()
    result = trigger(backend, debug=True)
    evidence = json.loads(result.diagnostics_json)["debug_capture"]
    assert result.success and result.found and evidence["rgb_path"]
    assert not evidence["depth_path"] and "depth unavailable" in evidence["error"]
    assert len(list((node.root / "debug/tray_img").glob("*.png"))) == 1


def test_arming_rejects_another_canonical_provider(backend):
    node, _, _ = backend
    node.get_node_names_and_namespaces = lambda: [("tray_detect", "/")]
    node.get_service_names_and_types_by_node = lambda *_: [(requests.SERVICE_NAME, ["type"])]
    with pytest.raises(ValueError, match="Another node"):
        arm(backend)
    node.create_service.assert_not_called()


def test_rearming_retires_old_endpoint_and_rejects_its_callback(backend):
    node, path, digest = backend
    api = arm(backend)
    old_callback = node.create_service.call_args.args[2]
    api.disarm()
    node.create_service.reset_mock()

    def create(*_args, **_kwargs):
        node.destroy_service.assert_called_once()
        return object()
    node.create_service.side_effect = create
    api.arm(path, settings(), digest)
    response = old_callback(GetTrayPose.Request(profile_sha256=digest), GetTrayPose.Response())
    assert not response.success and "disarmed" in response.message
    node.preview.assert_not_called()
    assert trigger(backend).success


def test_deployed_tray_pair_is_independent_and_can_coexist_with_item_catalog(tmp_path, artifact):
    path, _, _ = artifact
    directory = tmp_path / "runtime_teach"
    directory.mkdir()
    for source in (path, path.with_suffix(".pt")):
        shutil.copyfile(source, directory / source.name)
    deployed, model = runtime_tray_catalog(tmp_path)
    assert deployed.name == path.name and model.name == path.with_suffix(".pt").name
    loaded = core.load_profile(deployed, tmp_path, deployment=True)
    assert loaded == core.load_profile(path, tmp_path)
    with pytest.raises(ValueError, match="offline_teach"):
        core.load_profile(deployed, tmp_path)
    for name in ("item_teach_part.yaml", "item_teach_part.pt", "bin_teach_station.yaml"):
        (directory / name).write_bytes(b"independent catalog fixture")
    assert runtime_teach_catalog(tmp_path).item_yaml.name == "item_teach_part.yaml"
    assert runtime_tray_catalog(tmp_path)[0] == deployed


@pytest.mark.parametrize("names, reason", [
    ([], "Missing required tray_teach_ YAML"),
    (["tray_teach_a.yaml"], "Missing required tray_teach_ PT model"),
    (["tray_teach_a.yaml", "tray_teach_b.yaml"], "Multiple tray_teach_ YAML"),
    (["tray_teach_a.yaml", "tray_teach_a.pt", "tray_teach_b.pt"], "Multiple tray_teach_ PT"),
    (["tray_teach_a.yaml", "tray_teach_b.pt"], "same filename stem"),
    (["tray_teach_a.txt"], "extension"),
    (["unknown.yaml"], "filename prefix"),
])
def test_runtime_tray_catalog_fails_explicitly(tmp_path, names, reason):
    directory = tmp_path / "runtime_teach"
    directory.mkdir()
    for name in names:
        (directory / name).write_bytes(b"synthetic")
    with pytest.raises(ValueError, match=reason):
        runtime_tray_catalog(tmp_path)


def exercise_ros_transport(root):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from tray_perception import node as module
    from tray_perception.execution import spin_checked

    repository = Path(__file__).parents[3]
    for filename in (".env", ".env.example"):
        (root / filename).write_bytes((repository / ".env.example").read_bytes())
    artifacts = artifact.__wrapped__(root)
    prototype, original_path, _ = backend.__wrapped__(root, artifacts)
    directory = root / "runtime_teach"
    directory.mkdir()
    for source in (original_path, original_path.with_suffix(".pt")):
        shutil.copyfile(source, directory / source.name)
    path = directory / original_path.name
    module.workspace_root = lambda: root
    rclpy.init()
    node = module.TrayTeachNode(deployment=True)
    peer = Node("synthetic_tray_client_camera")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(peer)
    thread = threading.Thread(target=spin_checked, args=(node, executor), daemon=True)
    thread.start()
    try:
        node.connect_camera("cam")
        for name in ("camera", "model", "model_metadata", "plane", "position", "yolo_enabled"):
            setattr(node, name, getattr(prototype, name))
        node.model["path"] = str(path.with_suffix(".pt"))
        node.color_info = camera_info()

        def snapshot(**_kwargs):
            view = node.raw_snapshot()
            if _kwargs.get('depth_required') and view['depth'] is None:
                raise ValueError('Waiting for registered depth')
            view["camera_context"] = {"camera": camera_info(), "depth_camera": None,
                                      "base_from_optical": np.eye(4).tolist()}
            return view
        node.snapshot = snapshot  # Synthetic TF/context; real ROS RGB reception and service.
        node.preview = prototype.preview
        publisher = peer.create_publisher(Image, "/cam/color/image_raw", qos_profile_sensor_data)

        def publish():
            message = Image()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = "cam_color_optical_frame"
            message.encoding, message.width, message.height, message.step = "rgb8", 640, 480, 1920
            message.data = bytes(640 * 480 * 3)
            publisher.publish(message)
            return message

        deadline = time.monotonic() + 5
        while node.rgb is None and time.monotonic() < deadline:
            publish()
            time.sleep(.03)
        assert node.rgb is not None
        digest = core.file_sha256(path)
        client = peer.create_client(GetTrayPose, requests.SERVICE_NAME)
        assert requests.SERVICE_NAME == '/tray_detect/get_tray_pose_v3'
        legacy_calls = []
        for endpoint in ('/tray_detect/get_tray_pose', '/tray_detect/get_tray_pose_v2'):
            peer.create_service(GetTrayPose, endpoint,
                                lambda req, reply: legacy_calls.append(req) or reply)
        assert not client.wait_for_service(timeout_sec=.2)
        publish()
        node.requests.arm(path, settings(), digest)
        assert client.wait_for_service(timeout_sec=3)
        first = client.call_async(GetTrayPose.Request(profile_sha256=digest))
        deadline = time.monotonic() + 3
        while not node.requests.busy and time.monotonic() < deadline:
            time.sleep(.01)
        assert node.requests.busy
        second = client.call_async(GetTrayPose.Request(profile_sha256=digest))
        while not second.done() and time.monotonic() < deadline:
            time.sleep(.01)
        assert second.done() and second.result().status == "BUSY"
        assert not first.done()  # The retained pre-request frame did not satisfy it.
        fresh = publish()
        while not first.done() and time.monotonic() < deadline:
            time.sleep(.01)
        assert first.done() and first.result().success and first.result().found
        assert first.result().header.stamp == fresh.header.stamp
        assert node.preview.call_count == 1
        cancelled = client.call_async(GetTrayPose.Request(profile_sha256=digest))
        deadline = time.monotonic() + 3
        while not node.requests.busy and time.monotonic() < deadline:
            time.sleep(.01)
        assert node.requests.busy
        node.requests.disarm("Operator disarmed")
        while not cancelled.done() and time.monotonic() < deadline:
            time.sleep(.01)
        assert cancelled.done() and not cancelled.result().success
        assert "cancelled" in cancelled.result().message and thread.is_alive()
        # Re-arm and exercise the controller's depth-bearing wire request after
        # ordinary errors and no detection. RGB callbacks must keep advancing.
        publish()
        node.requests.arm(path, settings(), digest)
        depth = peer.create_publisher(Image, '/cam/depth/image_raw', qos_profile_sensor_data)
        node.native.call = MagicMock()
        original_preview = node.preview.side_effect
        for outcome in ('bad_depth', 'no_tray', 'valid', 'valid'):
            def preview(*args, **kwargs):
                value = original_preview(*args, **kwargs)
                if outcome == 'no_tray':
                    value['result'].update(selected=None, detections=[])
                return value
            node.preview.side_effect = preview
            node.native.call.return_value = (
                {'state': 'ok', 'error': 'Insufficient placement depth'} if outcome == 'bad_depth'
                else {'state': 'ok', 'surface_base': [.12, .13, .24],
                      'accepted_samples': 50, 'total_samples': 60,
                      'median_mm': 700., 'sigma_mm': 1.}, b'')
            placement = client.call_async(GetTrayPose.Request(
                profile_sha256=digest, sample_placement_depth=True,
                placement=PlacementDepthRequest(
                    x_mm=20., y_mm=30., diameter_mm=30., **QUALITY_DEFAULTS)))
            deadline = time.monotonic() + 4
            while not placement.done() and time.monotonic() < deadline:
                fresh = publish()
                image = Image(header=fresh.header, width=640, height=480,
                              encoding='16UC1', step=1280, data=bytes(640 * 480 * 2))
                depth.publish(image)
                time.sleep(.03)
            assert placement.done(), node.fatal_error
            response = placement.result()
            assert response.success is (outcome != 'bad_depth'), response.message
            assert response.placement.valid is (outcome == 'valid')
            assert thread.is_alive() and not node.fatal_error
            before = node.rgb['stamp_ns']
            deadline = time.monotonic() + 1
            while node.rgb['stamp_ns'] <= before and time.monotonic() < deadline:
                publish()
                time.sleep(.03)
            assert node.rgb['stamp_ns'] > before
        assert not legacy_calls
        assert node.rviz.publisher.topic_name == "/tray_detect/voxel_cloud"
        assert node.rviz.diagnostics.topic_name == "/tray_detect/rviz_diagnostics"
        assert node.rviz.pose_guides is None
        assert node.broadcaster is None and node.native.process is None
        assert not (root / "debug").exists()
        assert all(name not in sys.modules for name in ("cv2", "torch", "ultralytics"))
    finally:
        node.close()
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        peer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


def test_real_ros_requests_keep_rgb_callbacks_live_with_two_executor_threads(tmp_path):
    command = ("import sys; from pathlib import Path; sys.path.insert(0,sys.argv[1]); "
               "from test_requests import exercise_ros_transport; "
               "exercise_ros_transport(Path(sys.argv[2]))")
    result = subprocess.run([sys.executable, "-c", command, str(Path(__file__).parent),
                             str(tmp_path)], capture_output=True, text=True, timeout=35,
                            env={**os.environ, "ROS_DOMAIN_ID": "203", "ROS_LOCALHOST_ONLY": "1"})
    assert result.returncode == 0, result.stdout + result.stderr


def test_headless_launch_is_argument_free_and_local_only(monkeypatch):
    import runpy
    path = Path(__file__).parents[1] / "launch/tray_detect.launch.py"
    factory = runpy.run_path(str(path))["generate_launch_description"]
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "0")
    with pytest.raises(RuntimeError, match="ROS_LOCALHOST_ONLY"):
        factory()
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    assert len(factory().entities) == 1


@pytest.mark.parametrize("cause", ["nan", "wrong_axis", "wrong_origin", "long_x"])
def test_tray_extent_evidence_cannot_return_invalid_geometry(cause):
    value = selected()
    if cause == "nan":
        value["corners_base_m"][1][0] = float("nan")
    elif cause == "wrong_axis":
        value["quaternion"] = [0., 0., 1., 0.]
    elif cause == "long_x":
        value["corners_base_m"] = [[.1, .1, .2], [.3, .1, .2], [.3, .2, .2], [.1, .2, .2]]
    else:
        value["position"][0] += .1
    with pytest.raises(RuntimeError, match="extent evidence"):
        requests.pose_extents(value)


@pytest.mark.parametrize("debug", [False, True])
@pytest.mark.parametrize("display", [False, True])
def test_controller_capture_matches_saved_debug_and_saving_stays_optional(
        backend, monkeypatch, debug, display):
    from item_perception_yolo.item_preview import CaptureMailbox
    node, _, _ = backend
    if display:
        node.capture_preview = CaptureMailbox()
    save = MagicMock(return_value={"requested": True, "rgb_path": "rgb.png",
                                   "depth_path": "depth.png", "error": ""})
    monkeypatch.setattr(requests, "save_debug", save)
    arm(backend)
    response = trigger(backend, debug=debug)
    assert response.success
    assert node.preview.call_args.kwargs["visualize"] is (debug or display)
    assert node.preview.call_args.kwargs["returned_only"] is True
    assert save.call_count == int(debug)
    if display:
        captured = node.capture_preview.take()
        assert captured["response"] is response
        assert captured["view"]["trigger_epoch"] == node.requests.epoch
        if debug:
            saved = save.call_args.args[2]
            assert (captured["view"]["overlay"], captured["view"]["depth_overlay"]) == \
                (saved["overlay"], saved["depth_overlay"])
        assert node.capture_preview.take() is None
    assert not (node.root / "debug").exists()
