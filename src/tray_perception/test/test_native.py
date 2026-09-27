import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix


def exercise_geometry():
    import cv2
    import numpy as np
    from item_perception_yolo.item_geometry import project
    from tray_perception.native import corner_frame, evaluate_objects, fit_plane, capture_plane

    camera = {"width": 640, "height": 480,
              "k": [400., 0., 320., 0., 400., 240., 0., 0., 1.],
              "d": [0.] * 5, "distortion_model": "plumb_bob"}
    optical = np.diag([1., -1., -1., 1.])
    optical[:3, 3] = [.3, .2, 1.]
    plane_transform = np.eye(4)
    plane_transform[2, 3] = .2
    plane = {"base_from_plane": plane_transform.tolist()}
    context = {"camera": camera, "depth_camera": camera,
               "base_from_optical": optical.tolist()}
    settings = {"name": "tray", "model_task": "segment", "geometry_source": "mask",
                "geometry": {"length_mm": 200., "width_mm": 100., "tolerance_mm": 1.},
                "yolo": {"class_ids": [1], "confidence": .5, "iou": .7,
                         "max_detections": 100, "image_size": 640}}
    objects = []
    for index, (x, scale) in enumerate(((.1, 1), (.3, 1), (.31, .5))):
        rect = np.array([[-.1, -.05, 0], [.1, -.05, 0], [.1, .05, 0], [-.1, .05, 0]])
        rect = rect * scale + [x, .2, .2]
        polygon = project(rect, camera, optical, cv2, np).astype(np.float32)
        objects.append({"index": index, "class_id": 1, "class_name": "tray",
                        "confidence": .9, "polygon": polygon})
    detections, selected = evaluate_objects(objects, settings, plane, context, 640, 480, cv2, np)
    assert selected["source_index"] == 1 and len(detections) == 3
    assert not detections[2]["valid"]  # Closest raw detection loses because its size is wrong.
    assert np.allclose(selected["position"], [.2, .15, .2], atol=1e-6)
    assert abs(selected["length_mm"] - 200) < .001
    assert abs(selected["width_mm"] - 100) < .001
    context["depth_camera"] = None  # Detection needs neither current depth nor depth CameraInfo.
    assert evaluate_objects(objects, settings, plane, context, 640, 480, cv2, np)[1] == selected
    for x in (-.4, .4):
        for y in (-.4, .4):
            for angle in (0, 25, 75, 150):
                rect = cv2.boxPoints(((x, y), (.2, .1), angle))
                corners = np.column_stack((rect, np.full(4, .2)))
                matrix, order = corner_frame(corners, np.array([0., 0., 1.]), np)
                assert order[0] == min(range(4), key=lambda i: (
                    float(corners[i] @ corners[i]), *corners[i]))
                local = (corners - matrix[:3, 3]) @ matrix[:3, :3]
                assert np.all(local[:, :2] >= -1e-7)  # Positive X/Y always point into tray.
                assert np.linalg.det(matrix[:3, :3]) > .999999
    # A tilted plane retains Z and full 3D origin selection, never flattens onto base XY.
    rotation = cv2.Rodrigues(np.array([.4, .2, .1]))[0]
    corners = np.array([[.1, .1, 0], [.3, .1, 0], [.3, .2, 0], [.1, .2, 0]]) @ rotation.T
    corners += [.1, .1, .2]
    matrix, order = corner_frame(corners, rotation[:, 2], np)
    assert np.allclose(matrix[:3, 3], corners[order[0]])
    assert np.allclose(matrix[:3, 2], rotation[:, 2])
    assert np.all(((corners - matrix[:3, 3]) @ matrix[:3, :3])[:, :2] >= -1e-7)
    sloping = np.array([[.1, .1, .9], [.3, .1, .1], [.3, .2, .1], [.1, .2, .9]])
    normal = np.array([.8, 0., .2]) / np.linalg.norm([.8, 0., .2])
    closest, _ = corner_frame(sloping, normal, np)
    assert np.allclose(closest[:3, 3], [.3, .1, .1])  # XY-only ranking would choose .1,.1,.9.
    pixels = project(corners, camera, optical, cv2, np)
    fitted = fit_plane(corners[[2, 0, 3, 1]], pixels[[2, 0, 3, 1]], optical, cv2, np)
    assert fitted["max_error_mm"] < 1e-6
    assert np.allclose(np.asarray(fitted["base_from_plane"])[:3, 3], matrix[:3, 3])
    for invalid in (np.zeros((4, 3)), corners + [[0, 0, .1], [0, 0, 0], [0, 0, 0], [0, 0, 0]]):
        try:
            fit_plane(invalid, pixels, optical, cv2, np)
        except ValueError:
            pass
        else:
            raise AssertionError("Bad plane was accepted")
    request = {"width": 640, "height": 480, "camera_context": {
        **context, "depth_camera": camera}, "pixels": objects[1]["polygon"].tolist(),
        "source_stamp_ns": 100, "depth_stamp_ns": 100}
    reply, _ = capture_plane(request, np.full((480, 640), 800, "<u2").tobytes(), cv2, np)
    assert reply["plane"] is not None, reply
    reply, _ = capture_plane(request, np.zeros((480, 640), "<u2").tobytes(), cv2, np)
    assert reply["plane"] is None and "30" in reply["error"]
    # Registered depth can be rectified while color is distorted: sample mapped rays.
    from item_perception_yolo.item_geometry import reproject_pixels
    distorted = {**camera, "d": [.9, .2, 0., 0., 0.]}
    pixels = [[100., 100.], [540., 100.], [540., 380.], [100., 380.]]
    sparse = np.zeros((480, 640), "<u2")
    for x, y in np.rint(reproject_pixels(pixels, distorted, camera, cv2, np)).astype(int):
        sparse[y - 3:y + 4, x - 3:x + 4] = 800
    request["camera_context"] = {**context, "camera": distorted, "depth_camera": camera}
    request["pixels"] = pixels
    reply, _ = capture_plane(request, sparse.tobytes(), cv2, np)
    assert reply["plane"] is not None, reply


def test_private_geometry():
    runtime = Path(get_package_prefix("item_perception_yolo")) / \
        "lib/item_perception_yolo/yolo_runtime"
    command = "import sys,runpy; sys.path.insert(0,sys.argv[1]); " \
        "runpy.run_path(sys.argv[2])['exercise_geometry']()"
    result = subprocess.run(["/usr/bin/python3", "-c", command, str(runtime), __file__],
                            env=dict(os.environ, OPENBLAS_NUM_THREADS="1"),
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def test_real_private_worker_plane_and_prediction(tmp_path):
    """Exercise the new worker entry, shared model lifecycle and tray hook end to end."""
    from unittest.mock import MagicMock
    import numpy as np
    from item_perception_yolo.item_native_client import NativeClient
    from item_perception_yolo.item_teach_core import file_sha256
    runtime = Path(get_package_prefix("item_perception_yolo")) / \
        "lib/item_perception_yolo/yolo_runtime"
    config = tmp_path / "synthetic.yaml"
    model = tmp_path / "synthetic.pt"
    config.write_text("nc: 2\nbackbone:\n  - [-1, 1, Conv, [16, 3, 2]]\n"
                      "  - [-1, 1, Conv, [32, 3, 2]]\n"
                      "head:\n  - [[0, 1], 1, Segment, [nc, 32, 64]]\n")
    code = "import sys; sys.path.insert(0,sys.argv[1]); from ultralytics import YOLO; " \
        "YOLO(sys.argv[2],task='segment').save(sys.argv[3])"
    subprocess.run(["/usr/bin/python3", "-c", code, str(runtime), str(config), str(model)],
                   env=dict(os.environ, YOLO_CONFIG_DIR=str(tmp_path), YOLO_OFFLINE="true",
                            YOLO_AUTOINSTALL="false", MPLCONFIGDIR=str(tmp_path)),
                   check=True, capture_output=True, timeout=30)
    client = NativeClient(MagicMock(), worker_package="tray_perception",
                          worker_executable="tray_worker")
    camera = {"width": 64, "height": 48, "k": [100., 0., 32., 0., 100., 24., 0., 0., 1.],
              "d": [0.] * 5, "distortion_model": "plumb_bob"}
    optical = np.diag([1., -1., -1., 1.])
    optical[:3, 3] = [.3, .2, .8]
    context = {"camera": camera, "depth_camera": camera,
               "base_from_optical": optical.tolist()}
    try:
        result, data = client.call(
            {"operation": "tray_plane", "width": 64, "height": 48,
             "camera_context": context, "source_stamp_ns": 100, "depth_stamp_ns": 100,
             "pixels": [[10., 10.], [50., 10.], [50., 35.], [10., 35.]]},
            np.full((48, 64), 600, "<u2").tobytes())
        assert result["plane"] is not None and not data
        plane = result["plane"]
        model_config = {"path": str(model), "sha256": file_sha256(model)}
        result, data = client.call({"operation": "inspect", "model": model_config})
        assert result["task"] == "segment" and not data
        settings = {"name": "tray", "model_task": "segment", "geometry_source": "mask",
                    "geometry": {"length_mm": 200., "width_mm": 100., "tolerance_mm": 2.},
                    "yolo": {"class_ids": [0], "confidence": .99, "iou": .7,
                             "max_detections": 10, "image_size": 64}}
        result, data = client.call(
            {"operation": "tray_preview", "width": 64, "height": 48,
             "camera_context": context, "plane": plane, "settings": settings,
             "model": {**model_config, "task": "segment", "yolo": settings["yolo"]}},
            bytes(64 * 48 * 3))
        assert result["selected"] is None and result["reason"] == "No valid tray"
        assert len(data) == 64 * 48 * 3 and not client.failed
        # Losing the saved outline behind the camera is an observation issue, not worker failure.
        optical[:3, :3] = np.eye(3)
        result, _ = client.call(
            {"operation": "tray_preview", "width": 64, "height": 48,
             "camera_context": {**context, "base_from_optical": optical.tolist()},
             "plane": plane, "settings": settings,
             "model": {**model_config, "task": "segment", "yolo": settings["yolo"]}},
            bytes(64 * 48 * 3))
        assert "outline unavailable" in result["reason"] and not client.failed
    finally:
        client.close()
