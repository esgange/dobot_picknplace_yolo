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
    settings = {"accepted_class_ids": [1], "model_task": "segment", "geometry_source": "mask",
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
    # Dimensions can be inspected before typing a size or accepting any class.
    draft = {**settings, "geometry": None, "accepted_class_ids": []}
    measured, winner = evaluate_objects(objects, draft, plane, context, 640, 480, cv2, np)
    assert winner is None and measured[0]["length_mm"] > 199
    assert all(d["size_status"] == "unchecked" for d in measured)
    measured, winner = evaluate_objects(objects, {**settings, "accepted_class_ids": []},
                                        plane, context, 640, 480, cv2, np)
    assert winner is None and measured[0]["size_status"] == "pass"
    assert measured[2]["size_status"] == "fail"
    measured, winner = evaluate_objects(objects, draft, None, None, 640, 480, cv2, np)
    assert winner is None and len(measured) == 3 and "length_mm" not in measured[0]
    context["depth_camera"] = None  # Detection needs neither current depth nor depth CameraInfo.
    without_depth = evaluate_objects(objects, settings, plane, context, 640, 480, cv2, np)[1]
    assert without_depth == {k: v for k, v in selected.items() if k != "depth_rectangle"}
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
    assert reply["plane"] is None and "no valid" in reply["error"]
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
    from tray_perception.native import tray_visuals
    result, visual = tray_visuals({**request, "detections": [], "cloud": False},
                                  bytes([120]) * (640 * 480 * 3) + sparse.tobytes(), cv2, np)
    rgb = np.frombuffer(visual[640 * 480 * 3:], np.uint8).reshape(480, 640, 3)
    for sample, (x, y) in zip(result["samples"], pixels):
        assert sample["accepted"] == 49 and sample["median_mm"] == 800
        assert (rgb[int(y), int(x)] == 0).all()  # Evidence follows distorted RGB rays.
    # Sparse corners use every remaining valid sample, including only one.
    for counts in ((33, 49, 28, 26), (1, 1, 1, 1), (0, 49, 49, 49)):
        sparse[:] = 0
        for count, (x, y) in zip(counts, np.rint(reproject_pixels(
                pixels, distorted, camera, cv2, np)).astype(int)):
            patch = np.zeros(49, dtype="<u2")
            patch[:count] = 800
            sparse[y - 3:y + 4, x - 3:x + 4] = patch.reshape(7, 7)
        reply, _ = capture_plane(request, sparse.tobytes(), cv2, np)
        result, _ = tray_visuals({**request, "detections": [], "cloud": False},
                                 bytes(640 * 480 * 3) + sparse.tobytes(), cv2, np)
        assert [sample["accepted"] for sample in result["samples"]] == list(counts)
        for sample, count in zip(result["samples"], counts):
            assert sample["reason"] == ("OK" if count else "No valid depth samples")
        assert (reply["plane"] is not None) == all(counts), reply
        if not all(counts):
            assert "Corner 1: no valid" in reply["error"]
    # Exact inclusive tolerance boundary and the first value outside it.
    expected = {"length_mm": selected["length_mm"] + 1.,
                "width_mm": selected["width_mm"], "tolerance_mm": 1.}
    boundary = {**settings, "geometry": expected}
    assert evaluate_objects([objects[1]], boundary, plane, context, 640, 480, cv2, np)[1]
    boundary["geometry"] = {**expected, "length_mm": expected["length_mm"] + .0001}
    assert evaluate_objects([objects[1]], boundary, plane, context, 640, 480, cv2, np)[1] is None
    small_camera = {**camera, "width": 2, "height": 2,
                    "k": [10000., 0., 1., 0., 10000., 1., 0., 0., 1.]}
    small_optical = optical.copy()
    small_optical[:3, 3] = [.305, .205, .8]
    colors = np.array([[0, 0, 0], [100, 200, 100], [20, 40, 60], [80, 0, 40]], np.uint8)
    result, visual = tray_visuals({"width": 2, "height": 2, "cloud": True, "pixels": [],
                                  "detections": [], "camera_context": {
                                      "camera": small_camera, "depth_camera": small_camera,
                                      "base_from_optical": small_optical.tolist()}},
                                  colors.tobytes() + np.full(4, 600, "<u2").tobytes(), cv2, np)
    assert result["point_count"] == 1
    point = np.frombuffer(visual[24:], dtype=[("xyz", "<f4", 3), ("rgb", "<u4")])[0]
    assert np.allclose(point["xyz"], [.30497, .20503, .2])
    assert point["rgb"] == 0x323c32  # Mean source RGB, not an arbitrary retained pixel.


def test_private_geometry():
    runtime = Path(get_package_prefix("item_perception_yolo")) / \
        "lib/item_perception_yolo/yolo_runtime"
    command = "import sys,runpy; sys.path.insert(0,sys.argv[1]); " \
        "runpy.run_path(sys.argv[2])['exercise_geometry']()"
    result = subprocess.run(["/usr/bin/python3", "-c", command, str(runtime), __file__],
                            env=dict(os.environ, OPENBLAS_NUM_THREADS="1"),
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def exercise_tray_axis_overlays():
    import cv2
    import numpy as np
    from camera_calibration_gui.calibration_core import quaternion_to_rotation_matrix
    from item_perception_yolo.item_geometry import project
    from tray_perception import native

    camera = {"width": 640, "height": 480,
              "k": [400., 0., 320., 0., 400., 240., 0., 0., 1.],
              "d": [.9, .2, 0., 0., 0.], "distortion_model": "plumb_bob"}
    depth_camera = {**camera, "d": [0.] * 5}
    optical = np.diag([1., -1., -1., 1.])
    optical[:3, 3] = [.3, .2, 1.]
    plane_transform = np.eye(4)
    plane_transform[2, 3] = .2
    plane = {"base_from_plane": plane_transform.tolist(),
             "corners_base_m": [[.2, .15, .2], [.4, .15, .2], [.4, .25, .2], [.2, .25, .2]]}
    context = {"camera": camera, "depth_camera": depth_camera,
               "base_from_optical": optical.tolist()}
    corners = np.asarray(plane["corners_base_m"])
    objects = [{"index": 0, "class_id": 1, "class_name": "tray", "confidence": .9,
                "polygon": project(corners, camera, optical, cv2, np).astype(np.float32)}]
    settings = {"accepted_class_ids": [], "model_task": "segment", "geometry_source": "mask",
                "geometry": None, "yolo": {"class_ids": [1], "confidence": .5, "iou": .7,
                                           "max_detections": 100, "image_size": 640}}
    request = {"settings": settings, "plane": plane, "camera_context": context}
    rgb = np.zeros((480, 640, 3), np.uint8)
    native.render_result = lambda *_: (rgb.copy(), 1)
    native.objects_from_result = lambda *_: objects
    native.clean_objects = lambda *_: objects
    result, data = native.predict_trays(request, None, rgb, {1: "tray"}, cv2, np)
    item = result["detections"][0]
    assert result["selected"] is None and not item["valid"]
    assert "1 tray(s) measured" in result["reason"]
    assert "Size filter inactive" in result["reason"]
    assert item["reason"].startswith("Measured;")
    assert item["size_status"] == "unchecked"
    assert np.isclose(item["length_mm"], 200, atol=.01)
    assert np.isclose(item["width_mm"], 100, atol=.01)
    frame = quaternion_to_rotation_matrix(*item["quaternion"])
    assert np.allclose(frame[:, 0], [0, 1, 0], atol=1e-5)
    assert np.allclose(frame[:, 1], [1, 0, 0], atol=1e-5)
    assert np.allclose(frame[:, 2], [0, 0, -1], atol=1e-5)
    assert np.allclose(item["position"], [.2, .15, .2], atol=1e-6)
    assert np.allclose(item["depth_rectangle"], project(
        item["corners_base_m"], depth_camera, optical, cv2, np))
    assert not np.allclose(item["rectangle"], item["depth_rectangle"])
    overlay = np.frombuffer(data, np.uint8).reshape(rgb.shape)
    assert np.any(np.all(overlay == [255, 0, 0], axis=2))
    assert np.any(np.all(overlay == [0, 255, 0], axis=2))
    # Simulation keeps the exact detection evidence but draws no rejected pose.
    frozen_result, frozen = native.predict_trays(
        {**request, "returned_only": True}, None, rgb, {1: "tray"}, cv2, np)
    assert frozen_result == result
    _, plane_only = native.overlay_plane(
        {"width": 640, "height": 480, "plane": plane, "camera_context": context},
        rgb.tobytes(), cv2, np)
    assert frozen == plane_only
    accepted_settings = {**settings, "accepted_class_ids": [1],
                         "geometry": {"length_mm": 200., "width_mm": 100., "tolerance_mm": 1.}}
    accepted_request = {**request, "settings": accepted_settings}
    accepted_result, _ = native.predict_trays(accepted_request, None, rgb, {1: "tray"}, cv2, np)
    returned_result, returned_pixels = native.predict_trays(
        {**accepted_request, "returned_only": True}, None, rgb, {1: "tray"}, cv2, np)
    assert returned_result == accepted_result and returned_result["selected"] is not None
    assert returned_pixels != plane_only
    _, data = native.tray_visuals(
        {"width": 640, "height": 480, "camera_context": context,
         "detections": [item], "pixels": [], "cloud": False},
        rgb.tobytes() + np.full((480, 640), 800, "<u2").tobytes(), cv2, np)
    depth_overlay = np.frombuffer(data[:rgb.size], np.uint8).reshape(rgb.shape)
    assert np.all(depth_overlay[240, 270] == [255, 0, 0])  # X/short edge in depth pixels.
    assert np.all(depth_overlay[265, 320] == [0, 255, 0])  # Y/long edge in depth pixels.
    # Moving the saved outline far away must not alter independently measured trays.
    far_plane = {**plane, "corners_base_m": (corners + [10000., 0., 0.]).tolist()}
    far_result, _ = native.predict_trays(
        {**request, "plane": far_plane}, None, rgb, {1: "tray"}, cv2, np)
    assert far_result["detections"] == [item] and far_result["selected"] is None
    assert "outline unavailable" in far_result["reason"]
    # Size failure still provides inspection geometry, but never a production winner.
    settings["geometry"] = {"length_mm": 300., "width_mm": 150., "tolerance_mm": 1.}
    result, _ = native.predict_trays(request, None, rgb, {1: "tray"}, cv2, np)
    assert result["selected"] is None and result["detections"][0]["size_status"] == "fail"
    assert "rectangle" in result["detections"][0]
    result, data = native.predict_trays({**request, "plane": None}, None, rgb, {}, cv2, np)
    assert "rectangle" not in result["detections"][0] and result["selected"] is None
    assert "Create the reference plane" in result["reason"]
    assert "Create the reference plane" in result["detections"][0]["reason"]
    image_preview = np.frombuffer(data, np.uint8).reshape(rgb.shape)
    assert np.any(np.all(image_preview == [255, 0, 0], axis=2))
    assert np.any(np.all(image_preview == [0, 255, 0], axis=2))
    assert "position" not in result["detections"][0] and "length_mm" not in result["detections"][0]
    _, data = native.tray_visuals(
        {"width": 640, "height": 480, "camera_context": context,
         "detections": result["detections"], "pixels": [], "cloud": False},
        rgb.tobytes() + np.full((480, 640), 800, "<u2").tobytes(), cv2, np)
    image_preview = np.frombuffer(data[:rgb.size], np.uint8).reshape(rgb.shape)
    assert np.any(np.all(image_preview == [255, 0, 0], axis=2))
    result, data = native.predict_trays(
        {**request, "camera_context": None}, None, rgb, {}, cv2, np)
    assert "RGB-time TF" in result["reason"]
    assert "rectangle" not in result["detections"][0]
    image_preview = np.frombuffer(data, np.uint8).reshape(rgb.shape)
    assert np.any(np.all(image_preview == [255, 0, 0], axis=2))
    # Taught-plane marks persist even without YOLO/detections, in each camera model.
    _, data = native.overlay_plane({"width": 640, "height": 480, "plane": plane,
                                    "camera_context": context}, rgb.tobytes(), cv2, np)
    green_rgb = np.frombuffer(data, np.uint8).reshape(rgb.shape)
    _, data = native.tray_visuals(
        {"width": 640, "height": 480, "camera_context": context, "plane": plane,
         "detections": [], "pixels": [], "cloud": False},
        rgb.tobytes() + np.full((480, 640), 800, "<u2").tobytes(), cv2, np)
    green_depth = np.frombuffer(data[:rgb.size], np.uint8).reshape(rgb.shape)
    for camera_model, overlay in ((camera, green_rgb), (depth_camera, green_depth)):
        for x, y in np.rint(project(corners, camera_model, optical, cv2, np)).astype(int):
            assert np.all(overlay[y, x] == [0, 255, 0])
    # Every quadrant, tilt and rotation keeps inward short X / long Y and right-handed Z.
    tilt = cv2.Rodrigues(np.array([.4, .2, .1]))[0]
    for x in (-.4, .4):
        for y in (-.4, .4):
            for angle in (0, 25, 75, 150):
                rectangle = cv2.boxPoints(((0, 0), (.2, .1), angle))
                base = np.column_stack((rectangle, np.zeros(4))) @ tilt.T + [x, y, .2]
                for points in (base, base[::-1]):
                    frame, order = native.corner_frame(points, tilt[:, 2], np, short_x=True)
                    lengths = np.linalg.norm(points[[order[1], order[3]]] - frame[:3, 3], axis=1)
                    assert np.allclose(lengths, [.1, .2])
                    local = (points - frame[:3, 3]) @ frame[:3, :3]
                    assert np.all(local[:, :2] >= -1e-7)
                    assert np.linalg.det(frame[:3, :3]) > .999999
                    assert order[0] == min(range(4), key=lambda i: (
                        float(points[i] @ points[i]), *points[i]))
    square = np.array([[.1, .1, .2], [.3, .1, .2], [.3, .3, .2], [.1, .3, .2]])
    first, _ = native.corner_frame(square, np.array([0., 0., 1.]), np, short_x=True)
    second, _ = native.corner_frame(square[::-1], np.array([0., 0., 1.]), np, short_x=True)
    assert np.allclose(first, second)


def test_private_tray_axes_and_inspection_overlays():
    runtime = Path(get_package_prefix("item_perception_yolo")) / \
        "lib/item_perception_yolo/yolo_runtime"
    command = "import sys,runpy; sys.path.insert(0,sys.argv[1]); " \
        "runpy.run_path(sys.argv[2])['exercise_tray_axis_overlays']()"
    result = subprocess.run(["/usr/bin/python3", "-c", command, str(runtime), __file__],
                            env=dict(os.environ, OPENBLAS_NUM_THREADS="1"),
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def exercise_plane_projection_bounds():
    import copy
    import warnings
    import cv2
    import numpy as np
    from tray_perception import native

    camera = {"width": 640, "height": 480,
              "k": [400., 0., 320., 0., 400., 240., 0., 0., 1.],
              "d": [.9, .2, 0., 0., 0.], "distortion_model": "plumb_bob"}
    corners = np.array([[-.4, -.2, 0.], [.4, -.2, 0.], [.4, .2, 0.], [-.4, .2, 0.]])
    plane = {"base_from_plane": np.eye(4).tolist(), "corners_base_m": corners.tolist()}
    original_plane = copy.deepcopy(plane)
    optical = np.diag([1., -1., -1., 1.])
    rgb = np.zeros((480, 640, 3), np.uint8)
    request = {"width": 640, "height": 480, "plane": plane}
    # A moving camera can leave all corners forward but project them outside int32,
    # or even to infinity. Neither case may reach an integer cast or drawing call.
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        for distance in (.0001, 1e-60):
            optical[:3, 3] = [0., 0., distance]
            context = {"camera": camera, "depth_camera": camera,
                       "base_from_optical": optical.tolist()}
            result, data = native.overlay_plane(
                {**request, "camera_context": context}, rgb.tobytes(), cv2, np)
            assert result["state"] == "ok" and "safe drawing range" in result["error"]
            assert data == rgb.tobytes()  # No partially drawn/wrapped green outline.
            visual_request = {**request, "camera_context": context, "pixels": [],
                              "detections": [], "cloud": False}
            depths = np.full((480, 640), 600, "<u2").tobytes()
            result, data = native.tray_visuals(visual_request, rgb.tobytes() + depths, cv2, np)
            _, plain = native.tray_visuals(
                {**visual_request, "plane": None}, rgb.tobytes() + depths, cv2, np)
            assert result["state"] == "ok" and data == plain

    # A finite off-screen plane also leaves the image alone with a useful reason.
    camera = {**camera, "d": [0.] * 5}
    optical[:3, 3] = [100., 0., 1.]
    context = {"camera": camera, "base_from_optical": optical.tolist()}
    result, data = native.overlay_plane(
        {**request, "camera_context": context}, rgb.tobytes(), cv2, np)
    assert "outside the image" in result["error"] and data == rgb.tobytes()

    # Partly visible edges are clipped, while hidden corners get no invented markers.
    from unittest.mock import patch
    optical[:3, 3] = [.6, 0., .8]
    context["base_from_optical"] = optical.tolist()
    with patch.object(cv2, "circle", wraps=cv2.circle) as circles:
        result, data = native.overlay_plane(
            {**request, "camera_context": context}, rgb.tobytes(), cv2, np)
        centers = {call.args[1] for call in circles.call_args_list}
    assert not result["error"] and centers == {(220, 140), (220, 340)}
    image = np.frombuffer(data, np.uint8).reshape(rgb.shape)
    assert np.all(image[140, 0] == [0, 255, 0])
    assert np.all(image[340, 0] == [0, 255, 0])
    assert np.all(image[240, 220] == [0, 255, 0])
    assert plane == original_plane


def test_private_plane_projection_bounds():
    runtime = Path(get_package_prefix("item_perception_yolo")) / \
        "lib/item_perception_yolo/yolo_runtime"
    command = "import sys,runpy; sys.path.insert(0,sys.argv[1]); " \
        "runpy.run_path(sys.argv[2])['exercise_plane_projection_bounds']()"
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
        rgb = np.broadcast_to(np.array([17, 34, 51], np.uint8), (48, 64, 3)).tobytes()
        depths = np.full((48, 64), 600, "<u2").tobytes()
        visual_request = {"operation": "tray_visuals", "width": 64, "height": 48,
                          "camera_context": context, "cloud": True, "detections": [],
                          "pixels": [[10, 10]]}
        result, data = client.call(visual_request, rgb + depths)
        points = np.frombuffer(data[64 * 48 * 6:], dtype=[
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
        assert 0 < result["point_count"] == len(points) < 64 * 48
        assert np.allclose(points["z"], .2) and (points["rgb"] == 0x112233).all()
        xyz = np.column_stack([points[axis] for axis in ("x", "y", "z")])
        assert len(np.unique(np.floor(xyz / .01), axis=0)) == len(points)
        assert result["samples"][0]["accepted"] == 49
        assert result["samples"][0]["median_mm"] == 600
        rgb_overlay = np.frombuffer(data[64 * 48 * 3:64 * 48 * 6], np.uint8).reshape(48, 64, 3)
        assert (rgb_overlay[10, 10] == 0).all()
        result, data = client.call({**visual_request, "cloud": False}, rgb + bytes(64 * 48 * 2))
        assert result["point_count"] == 0 and len(data) == 64 * 48 * 6
        assert result["samples"][0]["median_mm"] is None
        result, _ = client.call({**visual_request, "camera_context": None,
                                 "pixels": [], "cloud": False}, rgb + depths)
        assert result["point_count"] == 0  # Uncalibrated depth view without a cloud.
        model_config = {"path": str(model), "sha256": file_sha256(model)}
        result, data = client.call({"operation": "inspect", "model": model_config})
        assert result["task"] == "segment" and not data
        settings = {"accepted_class_ids": [0], "model_task": "segment", "geometry_source": "mask",
                    "geometry": {"length_mm": 200., "width_mm": 100., "tolerance_mm": 2.},
                    "yolo": {"class_ids": [0], "confidence": .99, "iou": .7,
                             "max_detections": 10, "image_size": 64}}
        result, data = client.call(
            {"operation": "tray_preview", "width": 64, "height": 48,
             "camera_context": context, "plane": plane, "settings": settings,
             "model": {**model_config, "task": "segment", "yolo": settings["yolo"]}},
            bytes(64 * 48 * 3))
        assert result["selected"] is None and result["reason"] == "No eligible tray"
        assert len(data) == 64 * 48 * 3 and not client.failed
        # All three native entry points survive an overflowing projected outline
        # and the same worker can still draw a subsequent healthy observation.
        worker_pid = client.process.pid
        near_optical = optical.copy()
        near_optical[2, 3] = plane["base_from_plane"][2][3] + .0001
        distorted = {**camera, "d": [.9, .2, 0., 0., 0.]}
        near_context = {"camera": distorted, "depth_camera": distorted,
                        "base_from_optical": near_optical.tolist()}
        for operation in ("tray_preview", "tray_overlay", "tray_visuals"):
            request = {"operation": operation, "width": 64, "height": 48,
                       "camera_context": near_context, "plane": plane}
            if operation == "tray_preview":
                request.update(settings=settings, model={
                    **model_config, "task": "segment", "yolo": settings["yolo"]})
            elif operation == "tray_visuals":
                request.update(pixels=[], detections=[], cloud=False)
            result, _ = client.call(request, rgb + depths if operation == "tray_visuals" else rgb)
            assert result["state"] == "ok" and not client.failed
            if operation != "tray_visuals":
                assert "safe drawing range" in result.get("error", result.get("reason", ""))
        result, data = client.call(
            {"operation": "tray_overlay", "width": 64, "height": 48,
             "camera_context": context, "plane": plane}, rgb)
        assert not result["error"] and data != rgb
        assert not client.failed and client.process.pid == worker_pid
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
