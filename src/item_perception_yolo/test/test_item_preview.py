import io
import os
from pathlib import Path
import signal
import struct
import subprocess
from types import SimpleNamespace
from unittest.mock import MagicMock

from ament_index_python.packages import get_package_prefix
import pytest
from sensor_msgs.msg import Image

from item_perception_yolo import item_preview as preview
from item_perception_yolo.item_teach_core import file_sha256
from item_perception_yolo.preview_protocol import receive_packet, send_packet, MAX_HEADER


def test_protocol_roundtrip_and_bounds():
    stream = io.BytesIO()
    send_packet(stream, {"generation": 3}, b"123")
    stream.seek(0)
    assert receive_packet(stream) == ({"generation": 3}, b"123")
    with pytest.raises(ValueError, match="header size"):
        receive_packet(io.BytesIO(struct.pack("!I", MAX_HEADER + 1)))
    with pytest.raises(EOFError):
        receive_packet(io.BytesIO(b"\0"))


def message():
    msg = Image()
    msg.header.stamp.sec = 100
    msg.header.frame_id = "test_color_optical_frame"
    msg.width, msg.height, msg.step = 16, 8, 48
    msg.encoding = "rgb8"
    msg.data = bytes([10, 20, 30] * 128)
    return msg


def test_rgb_contract_and_stale_messages():
    msg = message()
    frame = preview.frame_from_message(msg, "test", 100_200_000_000)
    assert frame["rgb"][:3] == b"\x0a\x14\x1e"
    for key, value in (("encoding", "bgr8"), ("step", 49), ("is_bigendian", 1)):
        bad = message()
        setattr(bad, key, value)
        with pytest.raises(ValueError, match="rgb8"):
            preview.frame_from_message(bad, "test", 100_200_000_000)
    with pytest.raises(ValueError, match="stale"):
        preview.frame_from_message(msg, "test", 101_000_000_000)
    with pytest.raises(ValueError, match="frame_id"):
        preview.frame_from_message(msg, "wrong", 100_200_000_000)


def test_new_camera_subscription_discards_old_generation():
    from item_perception_yolo.item_teach_gui import ItemTeachNode
    import threading
    node = SimpleNamespace(
        _camera_generation=2, _feedback_lock=threading.Lock(), _image=None,
        condition=threading.Condition(),
        _image_sequence=0, camera_status="waiting", events=MagicMock(),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_100_000_000)),
    )
    ItemTeachNode._receive(node, message(), "test", 1, "rgb")
    assert node._image is None
    ItemTeachNode._receive(node, message(), "test", 2, "rgb")
    assert node._image["sequence"] == 1


@pytest.fixture(scope="module")
def native_paths():
    prefix = Path(get_package_prefix("item_perception_yolo")) / "lib/item_perception_yolo"
    return prefix / "item_preview_worker", prefix / "yolo_runtime"


@pytest.fixture(scope="module")
def synthetic_model(tmp_path_factory, native_paths):
    root = tmp_path_factory.mktemp("synthetic-yolo")
    model = root / "synthetic.pt"
    config = root / "synthetic.yaml"
    config.write_text(
        "nc: 2\nbackbone:\n  - [-1, 1, Conv, [16, 3, 2]]\n"
        "  - [-1, 1, Conv, [32, 3, 2]]\nhead:\n  - [[0, 1], 1, Detect, [nc]]\n"
    )
    environment = dict(os.environ, YOLO_CONFIG_DIR=str(root), YOLO_AUTOINSTALL="false",
                       YOLO_OFFLINE="true", YOLO_VERBOSE="false", MPLCONFIGDIR=str(root))
    code = (
        "import sys; sys.path.insert(0,sys.argv[1]); from ultralytics import YOLO; "
        "YOLO(sys.argv[2],task='detect').save(sys.argv[3])"
    )
    subprocess.run(["/usr/bin/python3", "-c", code, str(native_paths[1]), str(config), str(model)],
                   env=environment, check=True, capture_output=True, timeout=45)
    return model


def start_worker(native_paths):
    child = subprocess.Popen(
        ["/usr/bin/python3", str(native_paths[0]), str(native_paths[1])],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
    )
    # Native tests have a test-runner timeout as well as the worker client's deadline.
    header, data = receive_packet(child.stdout)
    assert header["state"] == "ready", header
    assert header["ultralytics"] == "8.4.150" and not data
    return child


def request(model, generation=1):
    return {
        "operation": "detect", "generation": generation, "width": 320, "height": 240,
        "model": {"path": str(model), "sha256": file_sha256(model), "task": "detect",
                  "yolo": {"confidence": 0.8, "iou": 0.5, "image_size": 320,
                           "max_detections": 20, "class_ids": [0, 1]}},
    }


def test_native_inference_repeated_rgb_frames(native_paths, synthetic_model):
    child = start_worker(native_paths)
    try:
        for generation in range(1, 4):
            pixels = bytes([10, 20, 30] * (320 * 240))
            send_packet(child.stdin, request(synthetic_model, generation), pixels)
            response, data = receive_packet(child.stdout)
            assert response["state"] == "ok", response
            assert response["generation"] == generation
            assert len(data) == len(pixels)
            assert 0 <= response["count"] <= 20
            assert data[:3] == pixels[:3]  # RGB/BGR conversion preserves background color
        child.stdin.close()
        assert child.wait(timeout=5) == 0
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=5)


def test_native_model_inspection_and_classes(native_paths, synthetic_model):
    child = start_worker(native_paths)
    try:
        send_packet(child.stdin, {"operation": "inspect", "model": {
            "path": str(synthetic_model), "sha256": file_sha256(synthetic_model)}})
        response, payload = receive_packet(child.stdout)
        assert response["state"] == "ok" and not payload
        assert response["task"] == "detect"
        assert set(response["classes"]) == {"0", "1"}
        assert response["geometry_sources"] == []
    finally:
        child.stdin.close()
        child.wait(timeout=5)


def test_roi_then_model_load_and_detection_share_worker(synthetic_model):
    from item_perception_yolo.item_native_client import NativeClient
    client = NativeClient(MagicMock())
    try:
        header = {"operation": "overlay_roi", "generation": 1, "width": 320,
                  "height": 240, "context": None, "error": "Synthetic missing TF",
                  "bin_clearance": {"p1_p2": None, "p2_p3": None,
                                    "p3_p4": None, "p4_p1": None}}
        pixels = bytes(320 * 240 * 3)
        metadata, overlay = client.call(header, pixels)
        assert metadata["roi_overlay"]["visible"] is False and overlay == pixels
        pid = client.process.pid
        metadata, payload = client.call({"operation": "inspect", "model": {
            "path": str(synthetic_model), "sha256": file_sha256(synthetic_model)}})
        assert metadata["task"] == "detect" and set(metadata["classes"]) == {"0", "1"}
        assert not payload and client.process.pid == pid
        result, overlay = client.call(request(synthetic_model), pixels)
        assert result["state"] == "ok" and len(overlay) == len(pixels)
        assert client.process.pid == pid and client.process.poll() is None
        assert not client.failed
    finally:
        client.close()


def test_native_all_detections_without_depth_or_production_settings(native_paths, synthetic_model):
    from item_perception_yolo.item_detector import validate_preview_detections
    child = start_worker(native_paths)
    try:
        header = request(synthetic_model)
        # Low confidence on this untrained synthetic network exercises clickable boxes.
        header["model"]["yolo"]["confidence"] = 0.0
        header.update(operation="preview", geometry_source="none", measurement_context=None,
                      measurement_error="No platform applied",
                      settings={"geometry": None, "pickdepth_radius": 30.,
                                "bin_clearance": {"p1_p2": None, "p2_p3": None,
                                                  "p3_p4": None, "p4_p1": None}})
        send_packet(child.stdin, header, bytes(320 * 240 * 3))
        result, pixels = receive_packet(child.stdout)
        assert result["state"] == "ok", result
        assert not result["has_depth_view"] and not result["candidates"]
        assert len(pixels) == 320 * 240 * 3
        assert result["count"] > 0 and len(result["detections"]) == result["count"]
        validate_preview_detections(result["detections"], result["count"], [0, 1])
        assert all(item["measurement"] is None for item in result["detections"])
        assert all(item["sampling_circle"] is None and item["sampling_circle_error"]
                   for item in result["detections"])
        # Change only confidence on the next request, without reloading the model
        # or restarting the worker. The untrained synthetic model has no score 1.
        header["model"]["yolo"]["confidence"] = 1.0
        send_packet(child.stdin, header, bytes(320 * 240 * 3))
        strict, _pixels = receive_packet(child.stdout)
        assert strict["state"] == "ok" and strict["count"] == 0
        assert child.poll() is None
    finally:
        child.stdin.close()
        child.wait(timeout=5)


def test_current_native_client_signal_is_terminal(native_paths):
    from item_perception_yolo.item_native_client import NativeClient
    client = NativeClient(MagicMock())
    child = start_worker(native_paths)
    client.process, client.ready = child, True
    os.kill(child.pid, signal.SIGSEGV)
    child.wait(timeout=5)
    with pytest.raises(RuntimeError, match="Terminal"):
        client.call({"operation": "inspect", "model": {}}, timeout=2)
    assert client.failed and client.closed and client.process.pid == child.pid
    with pytest.raises(RuntimeError, match="no replacement"):
        client.call({"operation": "inspect", "model": {}})


def test_private_roi_overlay_without_model_or_depth(native_paths):
    child = start_worker(native_paths)
    try:
        pixels = bytes(320 * 240 * 3)
        context = {
            "camera": {"k": [400., 0., 160., 0., 400., 120., 0., 0., 1.], "d": [0.] * 5},
            "platform_from_optical": [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, .8], [0, 0, 0, 1]],
            "roi": [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]]}
        header = {"operation": "overlay_roi", "generation": 4, "width": 320,
                  "height": 240, "context": context, "error": "",
                  "bin_clearance": {"p1_p2": None, "p2_p3": None,
                                    "p3_p4": None, "p4_p1": None}}
        # No model path/trust, YOLO setting, geometry filter or depth payload supplied.
        send_packet(child.stdin, header, pixels)
        result, image = receive_packet(child.stdout)
        assert result["state"] == "ok", result
        assert result["roi_overlay"] == {"visible": True, "reason": ""}
        assert len(image) == len(pixels) and image != pixels
        assert image[(120 * 320 + 60)*3:(120 * 320 + 60)*3+3] == bytes([0, 255, 0])
        header.update(context=None, error="No fresh camera TF")
        send_packet(child.stdin, header, pixels)
        result, image = receive_packet(child.stdout)
        assert result["roi_overlay"] == {"visible": False, "reason": "No fresh camera TF"}
        assert image == pixels  # No border from the previous request retained.
        assert child.poll() is None
    finally:
        child.stdin.close()
        child.wait(timeout=5)


def test_current_native_client_timeout_and_close_are_terminal():
    from item_perception_yolo.item_native_client import NativeClient
    client = NativeClient(MagicMock())
    child = subprocess.Popen(["/usr/bin/python3", "-c", "import time; time.sleep(30)"],
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE)
    client.process = child
    with pytest.raises(RuntimeError, match="Terminal"):
        client.call({"operation": "inspect", "model": {}}, timeout=.1)
    assert child.poll() is not None and client.failed and client.closed
    assert client.process.pid == child.pid
    client = NativeClient(MagicMock())
    client.close()
    with pytest.raises(RuntimeError, match="closed/failed"):
        client.call({"operation": "inspect", "model": {}})
    assert client.process is None


def test_private_rgb_depth_worker_end_to_end(native_paths, tmp_path):
    root = tmp_path
    model = root / "segment.pt"
    config = root / "segment.yaml"
    config.write_text("nc: 2\nbackbone:\n  - [-1, 1, Conv, [16, 3, 2]]\n"
                      "  - [-1, 1, Conv, [32, 3, 2]]\n"
                      "head:\n  - [[0, 1], 1, Segment, [nc, 32, 64]]\n")
    environment = dict(os.environ, YOLO_CONFIG_DIR=str(root), YOLO_AUTOINSTALL="false",
                       YOLO_OFFLINE="true", MPLCONFIGDIR=str(root))
    subprocess.run([
        "/usr/bin/python3", "-c",
        "import sys; sys.path.insert(0,sys.argv[1]); from ultralytics import YOLO; "
        "YOLO(sys.argv[2],task='segment').save(sys.argv[3])", str(native_paths[1]),
        str(config), str(model)], env=environment, check=True, capture_output=True, timeout=30)
    child = start_worker(native_paths)
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    header = request(model)
    header["model"]["task"] = "segment"
    header["settings"] = {
        "model_task": "segment", "yolo": header["model"]["yolo"],
        "geometry_source": "mask", "quality": dict(QUALITY_DEFAULTS),
        "bin_clearance": {"p1_p2": None, "p2_p3": None,
                          "p3_p4": None, "p4_p1": None},
        "geometry": {"height": 80., "width": 40., "tolerance": 5., "pickdepth_radius": 30.}}
    header["context"] = {
        "camera": {"k": [400., 0., 160., 0., 400., 120., 0., 0., 1.], "d": [0.] * 5},
        "depth_camera": {"k": [400., 0., 160., 0., 400., 120., 0., 0., 1.], "d": [0.] * 5},
        "platform_from_optical": [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, .8], [0, 0, 0, 1]],
        "roi": [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]],
        "pick_planning": {
            "home_matrix": [[1, 0, 0, 0], [0, 1, 0, 0],
                            [0, 0, 1, 0], [0, 0, 0, 1]],
            "base_from_platform": [[1, 0, 0, 0], [0, 1, 0, 0],
                                   [0, 0, 1, 0], [0, 0, 0, 1]],
            "link6_from_robot_camera": [[1, 0, 0, .03], [0, 1, 0, 0],
                                        [0, 0, 1, 0], [0, 0, 0, 1]],
            "pick_rotation_deg": 0.0, "standoff_height_mm": 90.0}}
    try:
        pixels = bytes([40, 50, 60] * (320 * 240))
        depth = struct.pack("<H", 700) * (320 * 240)
        send_packet(child.stdin, header, pixels + depth)
        result, data = receive_packet(child.stdout)
        assert result["state"] == "ok", result
        assert result["has_depth_view"] and result["geometry_sources"] == ["mask"]
        assert len(data) == len(pixels)*2
        assert isinstance(result["candidates"], list)
        # Real private-worker protocol supports bounded service/simulation overlays
        # without restarting the child or changing its full candidate list.
        send_packet(child.stdin, {**header, "candidate_limit": 3}, pixels + depth)
        batch_result, batch_pixels = receive_packet(child.stdout)
        assert batch_result["state"] == "ok", batch_result
        assert batch_result["has_depth_view"] and len(batch_pixels) == len(pixels)*2
        assert batch_result["candidates"] == result["candidates"]
        assert child.poll() is None
        # Unified RGB/depth preview does not auto-generate a pose for any item.
        preview_request = {**header, "operation": "preview", "context": None,
                           "geometry_source": "mask", "measurement_context": header["context"],
                           "depth_cameras": {key: header["context"][key]
                                             for key in ("camera", "depth_camera")},
                           "measurement_error": "", "preview_depth": True,
                           "settings": {**header["settings"], "pickdepth_radius": 30.}}
        send_packet(child.stdin, preview_request, pixels + depth)
        preview_result, preview_pixels = receive_packet(child.stdout)
        assert preview_result["state"] == "ok", preview_result
        assert preview_result["has_depth_view"] and len(preview_pixels) == 2*len(pixels)
        assert not preview_result["candidates"] and not preview_result["rejected"]
        # One selected exact-center rectangle: no second YOLO prediction/model load.
        rectangle = [[140, 110], [180, 110], [180, 130], [140, 130]]
        click = {"operation": "selected_pose", "generation": 2, "width": 320, "height": 240,
                 "context": header["context"], "settings": header["settings"],
                 "display_detections": preview_result["detections"],
                 "detection": {"source_index": 7, "class_id": 1, "class_name": "test",
                               "confidence": .9, "rectangle": rectangle, "polygon": rectangle}}
        other = [[240, 170], [280, 170], [280, 190], [240, 190]]
        click["display_detections"] = [
            {**click["detection"], "size_valid": True},
            {"source_index": 8, "rectangle": other, "polygon": other, "size_valid": False},
        ]
        send_packet(child.stdin, click, pixels + depth)
        selected, selected_pair = receive_packet(child.stdout)
        assert selected["state"] == "ok", selected
        assert not selected["rejected"] and len(selected["candidates"]) == 1
        point = selected["candidates"][0]
        assert point["source_index"] == 7 and point["pixel"] == [160., 120.]
        assert point["position"] == pytest.approx([0, 0, .1])
        assert len(selected_pair) == 2 * len(pixels)
        depth_overlay = selected_pair[len(pixels):]
        assert depth_overlay[(120*320+162)*3:(120*320+162)*3+3] == bytes(3)
        assert depth_overlay[(180*320+260)*3:(180*320+260)*3+3] == bytes([255]*3)
        # Other displayed items remain overlaid, but only the clicked item gets a pose.
        send_packet(child.stdin, click, pixels + bytes(len(depth)))
        rejected, _pixels = receive_packet(child.stdout)
        assert rejected["state"] == "ok" and not rejected["candidates"]
        assert "depth" in rejected["rejected"][0]["reason"]
        assert child.poll() is None  # Same lifetime child survives ordinary depth rejection.
    finally:
        child.stdin.close()
        child.wait(timeout=5)


def test_native_bad_weights_hard_fail(native_paths, tmp_path):
    model = tmp_path / "bad.pt"
    model.write_bytes(b"not a valid checkpoint")
    child = start_worker(native_paths)
    try:
        send_packet(child.stdin, request(model), bytes(320 * 240 * 3))
        response, _ = receive_packet(child.stdout)
        assert response["state"] == "error"
        assert child.wait(timeout=10) != 0
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=5)
