"""Synthetic geometry executes only inside the installed private native runtime."""

import os
from pathlib import Path
import subprocess

import pytest
from ament_index_python.packages import get_package_prefix


def exercise_geometry():
    import cv2
    import numpy as np
    from types import SimpleNamespace
    from item_perception_yolo.item_geometry import (
        generate_candidates, filter_depth, objects_from_result, on_plane,
        plane_dimensions, preview_detections, rectangle_axes, draw_pick_axes, draw_pick_geometry,
        draw_bin_roi, draw_bin_clearance, project_bin_roi, classify_size, selected_pose,
        depth_sampling_circle,
    )
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    assert cv2.__version__ == "4.10.0"
    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    camera = {"k": [1000., 0., 320., 0., 1000., 240., 0., 0., 1.], "d": [0.] * 5}
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .8
    context = {"camera": camera, "depth_camera": camera,
               "platform_from_optical": transform.tolist(),
               "roi": [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]]}
    settings = {"geometry_source": "mask",
                "bin_clearance": {"p1_p2": None, "p2_p3": None,
                                  "p3_p4": None, "p4_p1": None},
                "geometry": {"height": 80., "width": 32., "tolerance": .1,
                             "pickdepth_radius": 30.},
                "quality": dict(QUALITY_DEFAULTS), "yolo": {"class_ids": [1], "confidence": .5}}
    rgb = np.full((480, 640, 3), 80, np.uint8)
    depth = np.full((480, 640), 700, np.uint16)
    polygon = np.array([[270, 220], [370, 220], [370, 260], [270, 260]], np.float32)
    item = {"index": 0, "class_id": 1, "class_name": "test", "confidence": .7,
            "polygon": polygon, "rectangle": polygon, "center": np.array([320., 240.])}
    depth[240, 320] = 0  # Exact center invalid; never move the pick pixel.
    depth[240, 321] = 990
    overlay, depth_view, candidates, rejected = generate_candidates(
        [item], rgb, depth, context, settings, cv2, np)
    assert not rejected and len(candidates) == 1
    result = candidates[0]
    assert np.allclose(result["position"], [0, 0, .1])
    assert np.allclose([result["length"], result["width"]], [.08, .032])
    assert np.allclose(result["quaternion"], [0, 0, 0, 1])
    assert depth_view[240, 320].tolist() == [255, 0, 0]  # null rejected red
    assert depth_view[240, 321].tolist() == [255, 0, 0]  # outlier rejected red
    assert depth_view[240, 322].tolist() == [0, 0, 0]    # accepted black
    assert np.allclose(result["pixel"], [320, 240])
    # Size color is independent of depth validity; no item disappears when red.
    measured = {"length_mm": 80., "width_mm": 32.}
    for measured_size, taught, expected in (
            (measured, settings["geometry"], True),
            ({"length_mm": 95., "width_mm": 32.}, settings["geometry"], False),
            (None, settings["geometry"], None), (measured, None, None)):
        valid, reason = classify_size(measured_size, taught)
        assert valid is expected and reason
        if valid is not None:
            color = (0, 255, 0) if valid else (255, 0, 0)
            colored = rgb.copy()
            draw_pick_geometry(colored, polygon, cv2, np, color=color)
            assert tuple(colored[250, 270]) == color
    selected = {"source_index": 0, "class_id": 1, "class_name": "test", "confidence": .7,
                "rectangle": polygon.tolist(), "polygon": polygon.tolist()}
    _, clicked_depth, clicked, rejected_click = selected_pose(
        selected, rgb, depth, context, settings, cv2, np)
    assert clicked == candidates and not rejected_click
    assert np.array_equal(clicked_depth, depth_view)
    # Filtered view retains both segmentation shading and one pick rectangle.
    assert overlay[250, 280, 1] > rgb[250, 280, 1]
    assert overlay[250, 270].tolist() == [255, 225, 0]
    assert overlay[240, 320].tolist() == [255, 255, 255]
    assert draw_bin_roi(overlay, context, "", cv2, np)["visible"]
    assert overlay[240, 70, 1] > 240  # Bin border and item annotations coexist.
    clearance = {"p1_p2": 50., "p2_p3": 50., "p3_p4": 50., "p4_p1": 50.}
    assert draw_bin_clearance(overlay, context, clearance, cv2, np)
    assert np.any(np.all(overlay == [102, 204, 255], axis=2))
    # The footprint stays inside green, but the exact depth pick point is beyond
    # the right-edge 100 mm light-blue clearance and is therefore excluded.
    near_wall = {**item, "center": np.array([480., 240.]),
                 "polygon": polygon + [160., 0.], "rectangle": polygon + [160., 0.]}
    wall_settings = {**settings, "bin_clearance": {
        "p1_p2": None, "p2_p3": None, "p3_p4": 100., "p4_p1": None}}
    _, _, wall_candidates, wall_rejected = generate_candidates(
        [near_wall], rgb, depth, context, wall_settings, cv2, np)
    assert not wall_candidates
    assert wall_rejected == [{"source_index": 0,
                              "reason": "pick point outside bin-wall clearance"}]
    for z in (600, 900):
        depth[:] = z
        _, _, points, _ = generate_candidates([item], rgb, depth, context, settings, cv2, np)
        assert np.allclose([points[0]["length"], points[0]["width"]], [.08, .032])
        assert np.isclose(points[0]["position"][2], .8-z/1000)
    # Center-first beats confidence. Second rectangle has confidence 0.99.
    other = {**item, "index": 1, "confidence": .99,
             "center": item["center"]+[80, 0], "polygon": polygon+[80, 0],
             "rectangle": polygon+[80, 0]}
    _, _, ranked, _ = generate_candidates([other, item], rgb, depth, context, settings, cv2, np)
    assert [p["source_index"] for p in ranked] == [0, 1]
    # Depth validity, size, class, complete footprint and center-in-mask filters.
    depth[:] = 0
    assert not generate_candidates([item], rgb, depth, context, settings, cv2, np)[2]
    depth[:] = 700
    for changed in ({**item, "class_id": 2}, {**item, "confidence": .1},
                    {**item, "center": np.array([100., 100.])}):
        assert not generate_candidates([changed], rgb, depth, context, settings, cv2, np)[2]
    too_large = {**settings, "geometry": {**settings["geometry"], "height": 100.}}
    assert not generate_candidates([item], rgb, depth, context, too_large, cv2, np)[2]
    tiny_roi = {**context, "roi": [[-.02, -.02], [-.02, .02], [.02, .02], [.02, -.02]]}
    assert not generate_candidates([item], rgb, depth, tiny_roi, settings, cv2, np)[2]
    values = np.array([699., 700., 700., 700., 701., 999., 0., np.nan, np.inf])
    accepted, median, sigma = filter_depth(values, 200, 1000, cv2, np)
    assert median == 700 and not accepted[-4:].any() and sigma > 0
    accepted, median, sigma = filter_depth(np.array([700., 700., 700., 900.]), 200, 1000, cv2, np)
    assert accepted.tolist() == [True, True, True, False] and sigma == 0
    # Both output types: explicit source chooses only the requested geometry.

    class Tensor:
        def __init__(self, value):
            self.value = np.array(value)

        def cpu(self):
            return self

        def numpy(self):
            return self.value

    class Boxes(SimpleNamespace):
        def __len__(self):
            return 1
    boxes = Boxes(cls=Tensor([1]), conf=Tensor([.8]), xyxy=Tensor([[270, 220, 370, 260]]))
    obb = Boxes(cls=Tensor([1]), conf=Tensor([.9]), xyxyxyxy=Tensor([polygon+[10, 0]]))
    native = SimpleNamespace(boxes=boxes, obb=obb, masks=SimpleNamespace(xy=[polygon]))
    mask_object = objects_from_result(native, "mask", {1: "test"}, 20, cv2, np)[0]
    obb_object = objects_from_result(native, "obb", {1: "test"}, 20, cv2, np)[0]
    assert np.allclose(mask_object["center"], [320, 240])
    assert np.allclose(obb_object["center"], [330, 240])
    # Teaching dimensions use exactly the production plane calculation, without
    # size/class/ROI/depth filtering. Changing a production filter cannot hide these.
    measured = preview_detections(native, "mask", {1: "test"}, 100, context, "", cv2, np,
                                  diameter_mm=30.)
    assert len(measured) == 1
    assert np.allclose(list(measured[0]["measurement"].values()), [80., 32.])
    assert measured[0]["measurement_error"] == ""
    for source in ("mask", "obb"):
        geometry_object = objects_from_result(native, source, {1: "test"}, 100, cv2, np)[0]
        size = plane_dimensions(geometry_object["rectangle"], context, cv2, np)[:2]
        unfiltered = preview_detections(native, source, {1: "test"}, 100, context, "", cv2, np,
                                        diameter_mm=30.)
        assert np.allclose(size, np.array(list(unfiltered[0]["measurement"].values())) / 1000)
        center, radius, outline = depth_sampling_circle(
            geometry_object["center"], 30., context, cv2, np)
        assert radius == .015 and not unfiltered[0]["sampling_circle_error"]
        assert np.array_equal(outline, unfiltered[0]["sampling_circle"])
        assert np.allclose(np.ptp(outline, axis=0), [37.5, 37.5])
        larger = preview_detections(native, source, {1: "test"}, 100, context, "", cv2, np,
                                    diameter_mm=60.)
        assert np.allclose(np.ptp(larger[0]["sampling_circle"], axis=0), [75., 75.])
    unavailable = preview_detections(native, "mask", {1: "test"}, 100, None,
                                     "No calibration", cv2, np, diameter_mm=30.)
    assert unavailable[0]["measurement"] is None
    assert unavailable[0]["measurement_error"] == "No calibration"
    assert unavailable[0]["sampling_circle"] is None
    assert unavailable[0]["sampling_circle_error"] == "No calibration"
    tilted = transform.copy()
    angle = .2
    rotation = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0],
                         [-np.sin(angle), 0, np.cos(angle)]])
    tilted[:3, :3] = rotation @ transform[:3, :3]
    p = on_plane([[320, 240]], camera, tilted, cv2, np)[0]
    assert abs(p[2]) < 1e-9 and abs(p[0]) > .1  # true ray/plane intersection
    tilt_context = {**context, "platform_from_optical": tilted.tolist()}
    measured = preview_detections(native, "mask", {1: "test"}, 100, tilt_context, "", cv2, np,
                                  diameter_mm=30.)
    dims = plane_dimensions(mask_object["rectangle"], tilt_context, cv2, np)[:2]
    assert np.allclose(dims, np.array(list(measured[0]["measurement"].values())) / 1000)
    # Perspective/distortion must map back onto the same physical 15 mm radius,
    # not a screen-space circle estimated from rectangle size or camera depth.
    distorted = {**tilt_context, "camera": {**camera, "d": [.15, -.03, .002, -.001, .01]}}
    center, radius, outline = depth_sampling_circle([320., 240.], 30., distorted, cv2, np)
    back_projected = on_plane(outline, distorted["camera"], tilted, cv2, np)
    assert np.allclose(np.linalg.norm(back_projected[:, :2] - center[:2], axis=1), .015,
                       atol=1e-8)
    assert not np.isclose(np.ptp(outline[:, 0]), np.ptp(outline[:, 1]))
    for angle in (0, 30, 89, 135):
        rect = cv2.boxPoints(((320., 240.), (100., 40.), float(angle)))
        x_axis, y_axis, center = rectangle_axes(rect, np)
        assert np.allclose(center, [320, 240])
        assert np.allclose(x_axis.mean(axis=0), center)
        assert np.allclose(y_axis.mean(axis=0), center)
        assert np.isclose(np.linalg.norm(x_axis[1]-x_axis[0]), 100)
        assert np.isclose(np.linalg.norm(y_axis[1]-y_axis[0]), 40)
        mids = (rect + np.roll(rect, -1, axis=0)) / 2
        assert all(any(np.allclose(end, mid) for mid in mids) for end in np.r_[x_axis, y_axis])
    axes = rgb.copy()
    draw_pick_axes(axes, polygon, cv2, np)
    assert axes[240, 320].tolist() == [255, 255, 255]  # Exact pick point, not label-covered.
    assert axes[240, 300].tolist() == [255, 0, 0]  # Long X red, through middle.
    assert axes[230, 320].tolist() == [0, 255, 0]  # Short Y green, through middle.
    assert axes[220, 270].tolist() == rgb[220, 270].tolist()  # No rectangle outline.
    from item_perception_yolo.yolo_worker_native import render_result

    class NoBoxes:
        def __getattr__(self, key):
            assert key not in ("rectangle", "polylines"), "YOLO box rendering is forbidden"
            return getattr(cv2, key)
    masked, count = render_result(native, rgb, {1: "test"}, "segment", 100, NoBoxes(), np)
    assert count == 1 and not np.array_equal(masked, rgb)  # Mask remains visible.
    draw_pick_geometry(masked, mask_object["rectangle"], cv2, np)
    assert draw_bin_roi(masked, context, "", cv2, np)["visible"]
    assert masked[225, 285, 1] > rgb[225, 285, 1]  # Mask fill stays under the geometry.
    assert masked[220, 285].tolist() == [255, 225, 0]
    assert masked[240, 320].tolist() == [255, 255, 255]
    assert masked[240, 70, 1] > 240
    obb_image, count = render_result(native, rgb, {1: "test"}, "obb", 100, NoBoxes(), np)
    assert count == 1 and np.array_equal(obb_image, rgb)  # No invented mask or raw box.

    class OneOutline:
        calls = []

        def polylines(self, image, polygons, *args):
            self.calls.append(polygons)
            return cv2.polylines(image, polygons, *args)

        def __getattr__(self, key):
            assert key != "rectangle", "No additional axis-aligned YOLO box"
            return getattr(cv2, key)
    obb_renderer = OneOutline()
    draw_pick_geometry(obb_image, obb_object["rectangle"], obb_renderer, np)
    assert len(obb_renderer.calls) == 1
    assert np.allclose(obb_renderer.calls[0][0], obb_object["rectangle"])
    assert draw_bin_roi(obb_image, context, "", cv2, np)["visible"]
    assert obb_image[225, 295].tolist() == rgb[225, 295].tolist()  # No mask fill for OBB.
    assert obb_image[220, 295].tolist() == [255, 225, 0]
    assert obb_image[240, 330].tolist() == [255, 255, 255]
    assert obb_image[240, 70, 1] > 240
    roi_image = rgb.copy()
    status = draw_bin_roi(roi_image, context, "", cv2, np)
    assert status == {"visible": True, "reason": ""}
    assert roi_image[240, 70, 1] > 240  # Projected left bin boundary.
    assert np.array_equal(roi_image[240, 320], rgb[240, 320])  # Unfilled border.
    behind = np.array(transform)
    behind[2, 3] = -.8
    hidden = rgb.copy()
    status = draw_bin_roi(
        hidden, {**context, "platform_from_optical": behind.tolist()}, "", cv2, np)
    assert not status["visible"] and "behind camera" in status["reason"]
    assert np.array_equal(hidden, rgb)
    status = draw_bin_roi(hidden, None, "TF missing", cv2, np)
    assert status == {"visible": False, "reason": "TF missing"}

    # The two GUIs must project the same portable border, including platform tilt.
    from item_perception_yolo.bin_teach_core import BinRoiPoint, bin_border_in_optical
    saved = tuple(BinRoiPoint(x, y, index, 0) for index, (x, y) in enumerate(context["roi"]))
    original_xy = np.asarray(context["roi"]).copy()
    for rvec, translation in (([.12, -.09, .2], [.01, -.02, .9]),
                              ([-.08, .16, -.3], [-.03, .04, 1.1])):
        rotation, _ = cv2.Rodrigues(np.array(rvec, np.float64))
        destination_view = np.eye(4)
        destination_view[:3, :3] = rotation @ np.diag([1., -1., -1.])
        destination_view[:3, 3] = translation
        for distortion in ([0.] * 5, [.06, -.02, .001, -.002, .001, .004, -.001, .0002]):
            deployed = {**context, "camera": {**camera, "d": distortion},
                        "platform_from_optical": destination_view.tolist()}
            taught_optical = bin_border_in_optical(saved, destination_view)
            expected, _ = cv2.projectPoints(
                taught_optical, np.zeros(3), np.zeros(3),
                np.asarray(camera["k"]).reshape(3, 3), np.asarray(distortion))
            actual = project_bin_roi(deployed, cv2, np)
            assert actual.shape == (128, 2)
            assert np.allclose(actual, expected[:, 0], rtol=0, atol=1e-9)
            # Forward projection then plane intersection preserves saved metric XY.
            recovered = on_plane(actual[::32], deployed["camera"], destination_view, cv2, np)
            assert np.allclose(recovered[:, :2], original_xy, rtol=0, atol=1e-7)
            combined = masked.copy()
            assert draw_bin_roi(combined, deployed, "", cv2, np)["visible"]
            assert combined[240, 320].tolist() == masked[240, 320].tolist()
            assert np.array_equal(np.asarray(context["roi"]), original_xy)

    # A ROI-only projection rejects malformed/behind-camera geometry without
    # painting a stale or flattened border onto a different frame.
    malformed = np.eye(4)
    malformed[0, 0] = 2
    hidden = rgb.copy()
    status = draw_bin_roi(hidden, {**context, "platform_from_optical": malformed.tolist()},
                          "", cv2, np)
    assert not status["visible"] and "rigid transform" in status["reason"]
    assert np.array_equal(hidden, rgb)
    print("geometry: MAD, colors, centers, dimensions, ranking, masks/OBB, tilted rays passed")


def exercise_registered_depth():
    import cv2
    import numpy as np
    from item_perception_yolo.item_geometry import (
        generate_candidates, depth_sampling_circle, reproject_pixels, rays,
        on_plane, plane_dimensions, draw_depth_geometry,
    )
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    color = {"k": [461.215, 0., 424.829, 0., 461.19, 239.585, 0., 0., 1.],
             "d": [.2, -.05, .002, -.001, .02]}
    depth_camera = {**color, "d": [0.] * 5}
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .8
    context = {"camera": color, "depth_camera": depth_camera,
               "platform_from_optical": transform.tolist(),
               "roi": [[-.7, -.4], [-.7, .4], [.7, .4], [.7, -.4]]}
    center = np.array([660., 360.])  # Off-axis: equal pixel indices would be incorrect.
    polygon = center + np.array([[-34., -20.], [34., -20.], [34., 20.], [-34., 20.]])
    item = {"index": 3, "class_id": 1, "class_name": "part", "confidence": .9,
            "center": center, "rectangle": polygon, "polygon": polygon}
    mapped = reproject_pixels([center], color, depth_camera, cv2, np)[0]
    assert np.linalg.norm(mapped - center) > 10
    assert np.allclose(reproject_pixels([mapped], depth_camera, color, cv2, np)[0], center,
                       atol=1e-3)
    flat_center, radius, rgb_circle = depth_sampling_circle(center, 30., context, cv2, np)
    _, _, depth_circle = depth_sampling_circle(
        center, 30., context, cv2, np, output_camera=depth_camera)
    assert not np.allclose(rgb_circle, depth_circle)
    recovered = on_plane(depth_circle, depth_camera, transform, cv2, np)
    assert np.allclose(np.linalg.norm(recovered[:, :2] - flat_center[:2], axis=1), radius)
    assert np.allclose(reproject_pixels(depth_circle, depth_camera, color, cv2, np),
                       rgb_circle, atol=1e-6)
    for size_valid in (True, False, None):
        display = {**item, "size_valid": size_valid, "sampling_circle": rgb_circle.tolist()}
        empty = np.zeros((480, 848, 3), np.uint8)
        draw_depth_geometry(empty, [display], "mask", context, context, cv2, np)
        mapped_center = np.rint(mapped).astype(int)
        assert empty[mapped_center[1], mapped_center[0]].tolist() == [255, 255, 255]
        assert empty[360, 660].tolist() != [255, 255, 255]  # Not the RGB pixel center.
        assert np.allclose(display["depth_sampling_circle"], depth_circle, atol=1e-3)
        color_value = ((0, 255, 0) if size_valid is True else
                       (255, 0, 0) if size_valid is False else (180, 180, 180))
        edge = polygon[0] + .25 * (polygon[1] - polygon[0])
        qx, qy = np.rint(reproject_pixels([edge], color, depth_camera, cv2, np)[0]).astype(int)
        assert np.any(np.all(empty[qy-1:qy+2, qx-1:qx+2] == color_value, axis=2))
    yy, xx = np.mgrid[:480, :848]
    native_pixels = np.column_stack((xx.ravel(), yy.ravel()))
    points = on_plane(native_pixels, depth_camera, transform, cv2, np)
    in_circle = np.linalg.norm(points[:, :2] - flat_center[:2], axis=1) <= radius
    depth = np.zeros((480, 848), np.uint16)
    depth.ravel()[in_circle] = 700
    invalid = np.rint(mapped).astype(int)
    depth[invalid[1], invalid[0]] = 0
    original = depth.copy()
    length, width, _ = plane_dimensions(polygon, context, cv2, np)
    settings = {"geometry_source": "mask", "quality": dict(QUALITY_DEFAULTS),
                "bin_clearance": {"p1_p2": None, "p2_p3": None,
                                  "p3_p4": None, "p4_p1": None},
                "yolo": {"class_ids": [1], "confidence": .5},
                "geometry": {"height": length*1000, "width": width*1000,
                             "tolerance": .1, "pickdepth_radius": 30.}}
    rgb = np.zeros((480, 848, 3), np.uint8)
    _, view, candidates, rejected = generate_candidates(
        [item], rgb, depth, context, settings, cv2, np)
    assert not rejected and len(candidates) == 1
    result = candidates[0]
    expected = transform[:3, :3] @ (rays([center], color, cv2, np)[0] * .7) + transform[:3, 3]
    assert np.allclose(result["position"], expected)
    assert result["pixel"] == center.tolist()  # Original RGB center, never relocated.
    assert result["accepted_depth_count"] == int(in_circle.sum()) - 1
    assert result["rejected_depth_count"] == 1
    assert view[invalid[1], invalid[0]].tolist() == [255, 0, 0]
    assert view[invalid[1], invalid[0] + 1].tolist() == [0, 0, 0]
    assert np.array_equal(depth, original)  # No resizing/interpolation or frame mutation.
    # Shared optical origin is mandatory; missing metadata cannot reuse RGB D.
    with pytest.raises(KeyError, match="depth_camera"):
        generate_candidates([item], rgb, depth,
                            {k: v for k, v in context.items() if k != "depth_camera"},
                            settings, cv2, np)


def exercise_candidate_batch_overlay():
    import cv2
    import numpy as np
    from item_perception_yolo.item_geometry import generate_candidates, render_depth
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    camera = {"k": [1000., 0., 320., 0., 1000., 240., 0., 0., 1.], "d": [0.] * 5}
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .8
    context = {"camera": camera, "depth_camera": camera,
               "platform_from_optical": transform.tolist(),
               "roi": [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]]}
    settings = {"geometry_source": "mask", "quality": dict(QUALITY_DEFAULTS),
                "bin_clearance": {"p1_p2": None, "p2_p3": None,
                                  "p3_p4": None, "p4_p1": None},
                "geometry": {"height": 80., "width": 32., "tolerance": .1, "pickdepth_radius": 30.},
                "yolo": {"class_ids": [1], "confidence": .5}}
    rgb = np.full((480, 640, 3), 80, np.uint8)
    depth = np.full((480, 640), 700, np.uint16)
    polygon = np.array([[-50, -20], [50, -20], [50, 20], [-50, 20]], np.float32)
    objects = [{"index": index, "class_id": cls, "class_name": "part", "confidence": conf,
                "rectangle": polygon + [x, y], "polygon": polygon + [x, y],
                "center": np.array([x, y], np.float64)}
               for index, x, y, cls, conf in ((0, 180, 240, 1, .99), (1, 320, 240, 1, .6),
                                             (2, 460, 240, 1, .9), (3, 460, 350, 2, .99))]
    depth[240, 320] = 0
    depth[240, 321] = 990
    for source in ("mask", "obb"):
        config = {**settings, "geometry_source": source}
        rgb1, depth1, valid, rejected = generate_candidates(
            objects, rgb, depth, context, config, cv2, np, candidate_limit=1)
        assert [c["source_index"] for c in valid] == [1, 0, 2]  # Center, then confidence tie.
        assert rejected == [{"source_index": 3, "reason": "class not selected"}]
        baseline_depth = render_depth(depth, config["quality"], cv2, np)
        for x, y in ((180, 240), (460, 240), (460, 350)):
            # No mask, outline, sample dots, label or axes left on non-returned items.
            assert np.array_equal(rgb1[y-25:y+28, x-55:x+55], rgb[y-25:y+28, x-55:x+55])
            assert np.array_equal(depth1[y-25:y+28, x-55:x+55],
                                  baseline_depth[y-25:y+28, x-55:x+55])
        assert rgb1[250, 270].tolist() == [0, 255, 0]  # Single valid-size rectangle.
        assert rgb1[240, 320].tolist() == [255, 255, 255]
        assert depth1[240, 320].tolist() == [255, 0, 0]  # Null never enters MAD.
        assert depth1[240, 321].tolist() == [255, 0, 0]
        assert depth1[240, 322].tolist() == [0, 0, 0]
        assert rgb1[240, 70, 1] > 240 and depth1[240, 70, 1] > 240  # ROI retained on both.
        assert np.allclose(valid[0]["position"], [0, 0, .1])
        rgb3, depth3, all_valid, _ = generate_candidates(
            objects, rgb, depth, context, config, cv2, np, candidate_limit=3)
        assert valid == all_valid  # Rendering cap never changes candidate math/ranking.
        assert not np.array_equal(rgb3[220:270, 130:230], rgb1[220:270, 130:230])
        assert not np.array_equal(depth3[220:270, 130:230], depth1[220:270, 130:230])
        # No items is a frozen raw pair + bin ROI, not all rejected-object overlays.
        invalid = {**config, "yolo": {**config["yolo"], "class_ids": [9]}}
        empty_rgb, empty_depth, empty, _ = generate_candidates(
            objects, rgb, depth, context, invalid, cv2, np, candidate_limit=3)
        assert empty == []
        assert np.array_equal(empty_rgb[200:300, 260:380], rgb[200:300, 260:380])
        assert np.array_equal(empty_depth[200:300, 260:380], baseline_depth[200:300, 260:380])


@pytest.mark.parametrize("exercise", ["exercise_geometry", "exercise_registered_depth",
                                     "exercise_candidate_batch_overlay"])
def test_private_native_geometry(exercise):
    runtime = Path(get_package_prefix("item_perception_yolo")) / \
        "lib/item_perception_yolo/yolo_runtime"
    code = ("import sys,runpy; sys.path.insert(0,sys.argv[1]); "
            "runpy.run_path(sys.argv[2])[sys.argv[3]]()")
    result = subprocess.run(["/usr/bin/python3", "-c", code, str(runtime), __file__, exercise],
                            env=dict(os.environ, QT_QPA_PLATFORM="offscreen",
                                     OPENBLAS_NUM_THREADS="1"),
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
