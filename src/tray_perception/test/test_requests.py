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

from item_perception_yolo.runtime_teach import runtime_teach_catalog, runtime_tray_catalog
from tray_perception import core, requests
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
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node.connect_camera("cam")
        for name in ("camera", "model", "model_metadata", "plane", "position", "yolo_enabled"):
            setattr(node, name, getattr(prototype, name))
        node.model["path"] = str(path.with_suffix(".pt"))
        node.color_info = camera_info()

        def snapshot():
            view = node.raw_snapshot()
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
        node.requests.arm(path, settings(), digest)
        client = peer.create_client(GetTrayPose, requests.SERVICE_NAME)
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
        assert node.rviz.publisher.topic_name == "/tray_detect/voxel_cloud"
        assert node.rviz.diagnostics.topic_name == "/tray_detect/rviz_diagnostics"
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
                             str(tmp_path)], capture_output=True, text=True, timeout=20,
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
