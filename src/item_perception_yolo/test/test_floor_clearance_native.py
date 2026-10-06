"""Floor geometry and projection-cache checks in the pinned OpenCV runtime."""

import os
from pathlib import Path
import subprocess

import pytest
from ament_index_python.packages import get_package_prefix


def exercise_floor_measurement():
    import cv2
    import numpy as np
    from item_perception_yolo.floor_clearance import (
        floor_depth, floor_plane, nearby_depth_check, usable_scene_depth, inside_outer_bin)
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    cv2.setNumThreads(1)
    geometry = {"nearby_depth_radius_mm": 150., "nearby_depth_height_mm": 60.}
    plane = np.array([.5, 0., 1., -1.])
    candidate = np.array([0., 0., .8])

    def check(point, *, expected=None):
        points = np.array([candidate, point])
        scene = {"pixels": np.array([[1, 1], [2, 1]]), "points": points, "plane": plane,
                 "heights_mm": (floor_depth(points, plane, np)-points[:, 2])*1000.}
        drawing = {}
        if expected == "reject":
            with pytest.raises(ValueError, match="floor height"):
                nearby_depth_check(scene, candidate, geometry, np, visualization=drawing)
            assert len(drawing["blocking_points"]) == 1
        else:
            result = nearby_depth_check(scene, candidate, geometry, np)
            assert result["reference"] == "platform_floor_camera_z_v1"
            assert abs(result["maximum_height_difference_mm"] - expected) < 1e-6

    check([.1, 0., .72], expected=30.)  # Raw camera-depth difference is 80 mm.
    check([.1, 0., .69], expected="reject")
    check([.15, 0., .665], expected="reject")  # Exact radius and exact height.
    check([.15001, 0., .5], expected=0.)
    check([.149, 0., .66551], expected=59.99)
    empty = {"pixels": np.empty((0, 2)), "points": np.empty((0, 3)),
             "heights_mm": np.empty(0), "plane": plane}
    with pytest.raises(ValueError, match="No usable nearby"):
        nearby_depth_check(empty, candidate, geometry, np)
    with pytest.raises(ValueError, match="500"):
        nearby_depth_check(empty, [0., 0., .499], geometry, np)
    for bad in ([0., 0., 0., 0.], [1., 0., 0., 0.]):
        transform = np.eye(4)
        if bad[0] == 1:
            transform[:3, :3] = [[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]]
        else:
            transform[2] = bad
        with pytest.raises(ValueError):
            floor_plane({"platform_from_optical": transform.tolist()}, np)
    camera = {"width": 5, "height": 1, "k": [10., 0., 2., 0., 10., 0., 0., 0., 1.],
              "d": [0.]*5, "distortion_model": "plumb_bob"}
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .8
    context = {"camera": camera, "depth_camera": camera,
               "platform_from_optical": transform.tolist(),
               "roi": [[-.1, -.1], [-.1, .1], [.1, .1], [.1, -.1]]}
    depth = np.array([[550., 700., 700., 700., 700.]])
    scene = usable_scene_depth(depth, context, QUALITY_DEFAULTS, cv2, np)
    assert scene["pixels"].tolist() == [[1, 0], [2, 0], [3, 0]]
    evidence = nearby_depth_check(scene, [0., 0., .7], geometry, np)
    assert evidence["maximum_height_difference_mm"] == 0.
    depth[0, 1] = 550.  # Physical X=-55 mm: outside a 40 mm inset, inside outer bin.
    scene = usable_scene_depth(depth, context, QUALITY_DEFAULTS, cv2, np)
    with pytest.raises(ValueError, match="150.00 mm"):
        nearby_depth_check(scene, [0., 0., .7], geometry, np)
    boundary_points = np.array([[.1, 0], [.100001, 0], [-.1, -.1]])
    assert inside_outer_bin(boundary_points, context["roi"], np).tolist() == [True, False, True]
    # Vectorized edge masks agree with exhaustive signed-cross membership in both windings.
    random = np.random.default_rng(7).uniform(-.2, .2, (5000, 2))
    for boundary in (context["roi"], context["roi"][::-1]):
        roi = np.asarray(boundary)
        edges = np.roll(roi, -1, axis=0)-roi
        delta = random[:, None, :]-roi[None, :, :]
        cross = edges[None, :, 0]*delta[:, :, 1]-edges[None, :, 1]*delta[:, :, 0]
        expected = np.all(cross >= -1e-10, axis=1) | np.all(cross <= 1e-10, axis=1)
        assert np.array_equal(inside_outer_bin(random, boundary, np), expected)
    depth[0, :] = [float("nan"), 499., 500., float("inf"), 1001.]
    scene = usable_scene_depth(depth, context, QUALITY_DEFAULTS, cv2, np)
    assert scene["pixels"].tolist() == [[2, 0]]


def exercise_tray_depth_limits_unchanged():
    import cv2
    import numpy as np
    from item_perception_yolo.item_rviz_native import colored_voxels
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    camera = {"width": 2, "height": 1, "k": [10., 0., 0., 0., 10., 0., 0., 0., 1.],
              "d": [0.]*5, "distortion_model": "plumb_bob"}
    context = {"camera": camera, "depth_camera": camera,
               "platform_from_optical": np.eye(4).tolist()}
    rgb = np.full((1, 2, 3), 80, np.uint8)
    depth = np.array([[200., 499.]], np.float32)
    assert len(colored_voxels(rgb, depth, context, np.eye(4), QUALITY_DEFAULTS, cv2, np)) == 0
    assert len(colored_voxels(rgb, depth, context, np.eye(4), QUALITY_DEFAULTS, cv2, np,
                              item_minimum=False)) == 2
    assert QUALITY_DEFAULTS["depth_min_mm"] == 200.


def exercise_projection_cache():
    import cv2
    import numpy as np
    from item_perception_yolo.projection_cache import (
        cached_rays, cached_mapping, raw_rays, clear_projection_caches, _RAYS, _MAPPINGS)
    cv2.setNumThreads(1)
    camera = {"width": 30, "height": 20, "k": [25., 0., 15., 0., 25., 10., 0., 0., 1.],
              "d": [.1, -.04, .001, .002, 0.], "distortion_model": "plumb_bob"}
    depth = {**camera, "d": [0.]*5}
    yy, xx = np.indices((20, 30))
    pixels = np.column_stack((xx.ravel(), yy.ravel()))
    clear_projection_caches()
    actual = cached_rays(pixels, depth, cv2, np)
    assert np.array_equal(actual, raw_rays(pixels, depth, cv2, np))
    mapped = cached_mapping(pixels, depth, camera, cv2, np)
    expected, _ = cv2.projectPoints(actual, np.zeros(3), np.zeros(3),
                                    np.asarray(camera["k"]).reshape(3, 3), np.asarray(camera["d"]))
    assert np.array_equal(mapped, expected[:, 0])
    assert np.array_equal(cached_mapping(pixels[::5], depth, camera, cv2, np), mapped[::5])
    altered = {**depth, "d": [.2, 0., 0., 0., 0.]}
    assert not np.array_equal(cached_rays(pixels, altered, cv2, np), actual)
    for i in range(5):
        cached_rays(pixels, {**camera, "d": [.01*i, 0., 0., 0., 0.]}, cv2, np)
    assert len(_RAYS) <= 2 and len(_MAPPINGS) <= 2
    clear_projection_caches()
    assert not _RAYS and not _MAPPINGS


@pytest.mark.parametrize("exercise", ["exercise_floor_measurement", "exercise_projection_cache",
                                      "exercise_tray_depth_limits_unchanged"])
def test_floor_native(exercise):
    runtime = (Path(get_package_prefix("item_perception_yolo"))
               / "lib/item_perception_yolo/yolo_runtime")
    code = ("import sys,runpy; sys.path.insert(0,sys.argv[1]); "
            "runpy.run_path(sys.argv[2])[sys.argv[3]]()")
    result = subprocess.run(["/usr/bin/python3", "-c", code, str(runtime), __file__, exercise],
                            env=dict(os.environ, OPENBLAS_NUM_THREADS="1"), capture_output=True,
                            text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
