"""Synthetic voxel/pose checks in the private OpenCV runtime, without a model."""

import os
from pathlib import Path
import subprocess

import pytest
from ament_index_python.packages import get_package_prefix


def exercise_voxels():
    import cv2
    import numpy as np
    from item_perception_yolo.item_rviz_native import colored_voxels
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    from item_perception_yolo.item_geometry import reproject_pixels, rays

    camera = {"k": [1000., 0., 0., 0., 1000., 0., 0., 0., 1.], "d": [0.] * 5}
    context = {"camera": camera, "depth_camera": camera,
               "platform_from_optical": np.eye(4).tolist(),
               "roi": [[0., 0.], [.002, 0.], [.002, .002], [0., .002]]}
    rgb = np.array([[[255, 0, 0], [0, 255, 0]], [[0, 0, 255], [255, 255, 255]]], np.uint8)
    depth = np.full((2, 2), 500, np.uint16)
    base = np.eye(4)
    # All four points occupy one cell: average physical positions and original color.
    cloud = colored_voxels(rgb, depth, context, base, QUALITY_DEFAULTS, cv2, np)
    assert len(cloud) == 1 and cloud.dtype.itemsize == 16
    assert np.allclose([cloud[0][f] for f in ("x", "y", "z")], [.00025, .00025, .5])
    assert cloud["rgb"].tolist() == [0x808080]
    assert all(cloud.dtype.fields[f][1] == offset
               for f, offset in (("x", 0), ("y", 4), ("z", 8), ("rgb", 12)))
    # A 6.25 mm separation merges into one 10 mm voxel (four cells at 5 mm).
    wider = {**camera, "k": [80., 0., 0., 0., 80., 0., 0., 0., 1.]}
    merged = colored_voxels(rgb, depth, {**context, "camera": wider, "depth_camera": wider},
                            base, QUALITY_DEFAULTS, cv2, np)
    assert len(merged) == 1 and merged["rgb"].tolist() == [0x808080]
    assert np.allclose([merged[0][f] for f in ("x", "y", "z")], [.003125, .003125, .5])
    # Invalid depth/range is excluded, with no nearest-point filling.
    depth[:] = [[500, 0], [199, 1001]]
    cloud = colored_voxels(rgb, depth, context, base, QUALITY_DEFAULTS, cv2, np)
    assert cloud["rgb"].tolist() == [0xff0000]
    depth[:] = 0
    assert not len(colored_voxels(rgb, depth, context, base, QUALITY_DEFAULTS, cv2, np))
    depth[:] = 500
    # Surroundings outside the bin stay visible; the ROI limits item poses only.
    context["roi"] = [[1., 1.], [2., 1.], [2., 2.], [1., 2.]]
    assert len(colored_voxels(rgb, depth, context, base, QUALITY_DEFAULTS, cv2, np)) == 1

    # Different depth/RGB distortion must affect original RGB lookup, while complete
    # platform/base rotations and translations affect XYZ, not only planar yaw.
    camera = {"k": [20., 0., 10., 0., 20., 10., 0., 0., 1.], "d": [.4, 0., 0., 0., 0.]}
    depth_camera = {**camera, "d": [0.] * 5}
    optical = np.array([[0., 0., 1., .1], [0., 1., 0., .2], [-1., 0., 0., .3],
                        [0., 0., 0., 1.]])
    base = np.array([[1., 0., 0., .4], [0., 0., -1., .5], [0., 1., 0., .6],
                     [0., 0., 0., 1.]])
    context = {"camera": camera, "depth_camera": depth_camera,
               "platform_from_optical": optical.tolist(),
               "roi": [[-2., -2.], [2., -2.], [2., 2.], [-2., 2.]]}
    rgb = np.zeros((32, 32, 3), np.uint8)
    depth = np.zeros((32, 32), np.uint16)
    depth[20, 20] = 700
    color_pixel = np.rint(reproject_pixels([[20, 20]], depth_camera, camera, cv2, np))[0]
    assert color_pixel.tolist() != [20, 20]
    rgb[int(color_pixel[1]), int(color_pixel[0])] = [23, 90, 170]
    expected = base @ optical @ np.r_[rays([[20, 20]], depth_camera, cv2, np)[0] * .7, 1.]
    cloud = colored_voxels(rgb, depth, context, base, QUALITY_DEFAULTS, cv2, np)
    assert len(cloud) == 1 and cloud["rgb"][0] == (23 << 16) | (90 << 8) | 170
    assert np.allclose([cloud[0][f] for f in ("x", "y", "z")], expected[:3])


def exercise_all_candidates():
    import cv2
    import numpy as np
    from item_perception_yolo.item_rviz_native import teaching_rviz
    from item_perception_yolo.item_geometry import selected_pose
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS

    camera = {"k": [1000., 0., 320., 0., 1000., 240., 0., 0., 1.], "d": [0.] * 5}
    optical = np.diag([1., -1., -1., 1.])
    optical[2, 3] = .8
    context = {"camera": camera, "depth_camera": camera,
               "platform_from_optical": optical.tolist(),
               "roi": [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]],
               "pick_planning": {"home_matrix": np.eye(4).tolist(),
                                 "base_from_platform": np.eye(4).tolist(),
                                 "link6_from_robot_camera": np.eye(4).tolist(),
                                 "pick_rotation_deg": 0., "standoff_height_mm": 0.}}
    settings = {"model_task": "segment", "geometry_source": "mask",
                "quality": dict(QUALITY_DEFAULTS),
                "bin_clearance": dict.fromkeys(("p1_p2", "p2_p3", "p3_p4", "p4_p1")),
                "geometry": {"nearby_depth_radius_mm": 150., "nearby_depth_height_mm": 60.,
                             "height": 80., "width": 32., "tolerance": .1,
                             "pickdepth_radius": 30.},
                "yolo": {"class_ids": [1], "confidence": .5, "iou": .7,
                         "max_detections": 100, "image_size": 640}}
    rgb = np.full((480, 640, 3), 80, np.uint8)
    depth = np.full((480, 640), 700, np.uint16)
    rectangle = np.array([[-50, -20], [50, -20], [50, 20], [-50, 20]], np.float32)
    detections = [{"source_index": i, "class_id": 1 if i < 4 else 2,
                   "class_name": "part", "confidence": .9 - i * .05,
                   "rectangle": (rectangle + [x, 240]).tolist(),
                   "polygon": (rectangle + [x, 240]).tolist()}
                  for i, x in enumerate((180, 280, 380, 480, 580))]
    request = {"width": 640, "height": 480, "generation": 7, "context": context,
               "nearby_overlay": False,
               "base_from_platform": np.eye(4).tolist(), "quality": dict(QUALITY_DEFAULTS),
               "settings": settings, "detections": detections}
    result, data = teaching_rviz(request, rgb.tobytes() + depth.tobytes(), cv2, np)
    assert result["generation"] == 7 and len(data) == result["point_count"] * 16
    assert 0 < result["point_count"] < depth.size
    assert {c["source_index"] for c in result["candidates"]} == {0, 1, 2, 3}
    assert len(result["rejected"]) == 1 and result["rejected"][0]["source_index"] == 4
    # Each pose is exactly the existing full-resolution clicked-pose result.
    for detection in detections[:4]:
        _, _, candidates, rejected = selected_pose(
            detection, rgb, depth, context, settings, cv2, np)
        assert not rejected
        assert candidates[0] in result["candidates"]
    cloud_only, cloud_data = teaching_rviz(
        {**request, "settings": None}, rgb.tobytes() + depth.tobytes(), cv2, np)
    assert not cloud_only["candidates"] and not cloud_only["rejected"]
    assert cloud_data == data
    # Live all-class imagery keeps its existing annotations and receives only
    # nearby diagnostics from the same full-resolution check, with no inference.
    annotated = rgb.copy()
    annotated[200:205, 300:310] = (12, 34, 56)
    visual, payload = teaching_rviz(
        {**request, "nearby_overlay": True},
        rgb.tobytes() + depth.tobytes() + annotated.tobytes() + annotated.tobytes(), cv2, np)
    assert visual["nearby_overlay"] is True
    assert visual["candidates"] == result["candidates"]
    assert visual["rejected"] == result["rejected"]
    assert payload[:len(data)] == data
    assert len(payload) == len(data) + rgb.nbytes * 2
    for start in (len(data), len(data) + rgb.nbytes):
        pixels = np.frombuffer(payload[start:start+rgb.nbytes], np.uint8).reshape(rgb.shape)
        assert np.array_equal(pixels[200:205, 300:310], annotated[200:205, 300:310])
        assert not np.array_equal(pixels, annotated)
    with pytest.raises(RuntimeError, match="Malformed"):
        teaching_rviz({**request, "nearby_overlay": True},
                      rgb.tobytes() + depth.tobytes(), cv2, np)


@pytest.mark.parametrize("exercise", ["exercise_voxels", "exercise_all_candidates"])
def test_native_teaching_visualization(exercise):
    runtime = Path(get_package_prefix("item_perception_yolo")) / \
        "lib/item_perception_yolo/yolo_runtime"
    code = ("import sys,runpy; sys.path.insert(0,sys.argv[1]); "
            "runpy.run_path(sys.argv[2])[sys.argv[3]]()")
    result = subprocess.run(["/usr/bin/python3", "-c", code, str(runtime), __file__, exercise],
                            env=dict(os.environ, QT_QPA_PLATFORM="offscreen",
                                     OPENBLAS_NUM_THREADS="1"),
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
