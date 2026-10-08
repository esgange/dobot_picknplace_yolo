import os
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_prefix


def exercise_placement_depth():
    import cv2
    import numpy as np
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    from tray_perception.native import placement_depth

    camera = {"width": 640, "height": 480,
              "k": [400., 0., 320., 0., 400., 240., 0., 0., 1.],
              "d": [0.] * 5, "distortion_model": "plumb_bob"}
    optical = np.diag([1., -1., -1., 1.])
    optical[:3, 3] = [.3, .2, 1.]
    selected = {"position": [.2, .15, .2], "quaternion": [0., 0., 0., 1.],
                "width_mm": 200., "length_mm": 300.}
    sampling = {"x_mm": 30., "y_mm": 40., "diameter_mm": 30., **QUALITY_DEFAULTS}
    request = {"width": 640, "height": 480, "sampling": sampling, "selected": selected,
               "camera_context": {"camera": camera, "depth_camera": camera,
                                  "base_from_optical": optical.tolist()}}
    depth = np.full((480, 640), 700, dtype="<u2")
    depth[245, 285] = 990  # In-circle outlier; MAD rejects it.
    result, data = placement_depth(request, depth.tobytes(), cv2, np)
    assert data == b"" and result["median_mm"] == 700 and result["sigma_mm"] == 0
    assert np.allclose(result["surface_base"], [.23, .19, .3])
    assert result["accepted_samples"] < result["total_samples"]
    assert 150 < result["total_samples"] < 200  # 30 mm diameter, not radius.
    # Registered depth has its own distortion/intrinsics. It is not resized or
    # assumed to use RGB pixels, even though both streams share the optical frame.
    depth_camera = {**camera, "k": [405., 0., 318., 0., 405., 242., 0., 0., 1.],
                    "d": [.08, -.02, 0., 0., 0.]}
    request["camera_context"]["depth_camera"] = depth_camera
    result, _ = placement_depth(request, depth.tobytes(), cv2, np)
    assert result["median_mm"] == 700 and np.allclose(result["surface_base"], [.23, .19, .3])
    for change, frame in (({"x_mm": 201.}, depth), ({"y_mm": -1.}, depth),
                          ({}, np.zeros_like(depth)),
                          ({"x_mm": 1., "minimum_depth_fraction": .95}, depth)):
        invalid = {**request, "sampling": {**sampling, **change}}
        result, data = placement_depth(invalid, frame.tobytes(), cv2, np)
        assert result["error"] and data == b"", f"Accepted invalid placement depth: {change}"

    # The same physical footprint accepts the same coverage at 360p and 720p,
    # including a high-quality footprint containing fewer than 30 pixels.
    for scale in (1, 2):
        width, height, focal = 640 * scale, 360 * scale, 300. * scale
        camera = {"width": width, "height": height,
                  "k": [focal, 0., width / 2, 0., focal, height / 2, 0., 0., 1.],
                  "d": [0.] * 5, "distortion_model": "plumb_bob"}
        request = {"width": width, "height": height, "selected": selected,
                   "sampling": {**sampling, "x_mm": 100., "y_mm": 50., "diameter_mm": 10.},
                   "camera_context": {"camera": camera, "depth_camera": camera,
                                      "base_from_optical": optical.tolist()}}
        yy, xx = np.mgrid[:height, :width]
        circle = np.argwhere((xx - width / 2)**2 + (yy - height / 2)**2
                             <= (.005 * focal / .8)**2)
        total = len(circle)
        for fraction in (.2, .3, .5):
            request["sampling"]["minimum_depth_fraction"] = fraction
            minimum = int(np.ceil(total * fraction))
            for good in (0, minimum - 1, minimum, total):
                frame = np.zeros((height, width), dtype="<u2")
                frame[tuple(circle[:good].T)] = 700
                result, _ = placement_depth(request, frame.tobytes(), cv2, np)
                if good / total >= fraction:
                    assert result["accepted_samples"] == good and result["total_samples"] == total
                    assert np.allclose(result["surface_base"], [.3, .2, .3])
                    if scale == 1:
                        assert good < 30
                else:
                    assert f"requires {fraction:.1%}" in result["error"]


def test_private_placement_depth_geometry():
    runtime = Path(get_package_prefix("item_perception_yolo")) / \
        "lib/item_perception_yolo/yolo_runtime"
    command = ("import sys, runpy; sys.path.insert(0, sys.argv[1]); "
               "runpy.run_path(sys.argv[2])['exercise_placement_depth']()")
    result = subprocess.run(["/usr/bin/python3", "-c", command, str(runtime), __file__],
                            capture_output=True, text=True, timeout=60, env=os.environ.copy())
    assert result.returncode == 0, result.stdout + result.stderr


def test_real_worker_survives_unusable_placement_depth():
    import numpy as np
    from unittest.mock import Mock
    from item_perception_yolo.item_native_client import NativeClient
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS

    camera = {"width": 64, "height": 48,
              "k": [100., 0., 32., 0., 100., 24., 0., 0., 1.],
              "d": [0.] * 5, "distortion_model": "plumb_bob"}
    optical = np.diag([1., -1., -1., 1.])
    optical[:3, 3] = [.3, .2, .8]
    request = {"operation": "tray_placement_depth", "width": 64, "height": 48,
               "selected": {"position": [.2, .1, .2], "quaternion": [0., 0., 0., 1.],
                            "width_mm": 200., "length_mm": 300.},
               "sampling": {"x_mm": 100., "y_mm": 100., "diameter_mm": 30.,
                            **QUALITY_DEFAULTS},
               "camera_context": {"camera": camera, "depth_camera": camera,
                                  "base_from_optical": optical.tolist()}}
    client = NativeClient(Mock(), worker_package="tray_perception", worker_executable="tray_worker")
    try:
        failed, _ = client.call(request, bytes(64 * 48 * 2))
        assert failed["error"] and not client.failed
        pid = client.process.pid
        good, _ = client.call(request, np.full((48, 64), 600, "<u2").tobytes())
        assert good["median_mm"] == 600 and good["accepted_samples"] >= 3
        assert not client.failed and client.process.pid == pid
    finally:
        client.close()
