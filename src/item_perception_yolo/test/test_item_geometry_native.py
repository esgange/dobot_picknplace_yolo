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
        depth_sampling_circle, polygons_overlap_or_touch, inside,
    )
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS, inset_bin_roi
    assert cv2.__version__ == "4.10.0"
    from item_perception_yolo.item_preview import _TURBO
    assert np.array_equal(_TURBO, cv2.cvtColor(cv2.applyColorMap(
        np.arange(256, dtype=np.uint8), cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB).reshape(256, 3))
    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    camera = {"k": [1000., 0., 320., 0., 1000., 240., 0., 0., 1.], "d": [0.] * 5}
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .8
    context = {"camera": camera, "depth_camera": camera,
               "platform_from_optical": transform.tolist(),
               "roi": [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]],
               "pick_planning": {
                   "home_matrix": np.eye(4).tolist(),
                   "base_from_platform": np.eye(4).tolist(),
                   "link6_from_robot_camera": [[1., 0., 0., .04], [0., 1., 0., 0.],
                                                [0., 0., 1., 0.], [0., 0., 0., 1.]],
                   "pick_rotation_deg": 0.0,
                   "standoff_height_mm": 0.0,
               }}
    roi = np.asarray(context["roi"])
    assert polygons_overlap_or_touch(
        [[.19, -.02], [.25, -.02], [.25, .02], [.19, .02]], roi, cv2, np)
    assert polygons_overlap_or_touch(
        [[.2, -.02], [.25, -.02], [.25, .02], [.2, .02]], roi, cv2, np)
    assert not polygons_overlap_or_touch(
        [[.201, -.02], [.25, -.02], [.25, .02], [.201, .02]], roi, cv2, np)
    # Edge crossings count even when neither polygon contains a vertex.
    assert polygons_overlap_or_touch(
        [[-2., -.1], [2., -.1], [2., .1], [-2., .1]],
        [[-.1, -2.], [.1, -2.], [.1, 2.], [-.1, 2.]], cv2, np)
    settings = {"geometry_source": "mask",
                "bin_clearance": {"p1_p2": None, "p2_p3": None,
                                  "p3_p4": None, "p4_p1": None},
                "geometry": {"depth_frame_count": 3, "nearby_depth_radius_mm": 150., "nearby_depth_height_mm": 60.,
                             "height": 70., "width": 28., "tolerance": .1,
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
    assert np.allclose([result["length"], result["width"]], [.07, .028])
    assert np.allclose(result["quaternion"], [0, 0, np.sqrt(.5), np.sqrt(.5)])
    # X is the short line; Y is the long line. Link6 keeps its old attitude.
    assert np.allclose(np.asarray(result["planned_link6_matrix"])[:3, :3], np.eye(3))
    from item_perception_yolo.pick_planning import candidate_pose_in_base
    for angle_deg in (-80, -35, 0, 25, 80):
        angle = np.deg2rad(angle_deg)
        pixel_rotation = np.array([[np.cos(angle), -np.sin(angle)],
                                   [np.sin(angle), np.cos(angle)]])
        rotated = (polygon - [320., 240.]) @ pixel_rotation.T + [320., 240.]
        observed = {**item, "polygon": rotated, "rectangle": rotated}
        _, _, poses, reasons = generate_candidates(
            [observed], rgb, depth, context, settings, cv2, np)
        assert not reasons and len(poses) == 1
        measured = poses[0]
        pose = candidate_pose_in_base(np.eye(4), measured["position"], measured["quaternion"])
        # Camera optical Y maps to negative platform Y in this fixture.
        expected_tool = np.array([[np.cos(angle), np.sin(angle), 0.],
                                  [-np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
        assert abs(np.dot(pose[:3, 0], expected_tool[:, 1])) > .99999
        assert abs(np.dot(pose[:3, 1], expected_tool[:, 0])) > .99999
        assert np.allclose(np.cross(pose[:3, 0], pose[:3, 1]), pose[:3, 2])
        assert np.allclose(measured["position"], [0., 0., .1], atol=1e-6)
        assert np.allclose([measured["length"], measured["width"]], [.07, .028])
        assert np.allclose(np.asarray(measured["planned_link6_matrix"])[:3, :3],
                           expected_tool, atol=1e-6)
    assert depth_view[240, 320].tolist() == [255, 0, 0]  # null rejected red
    assert depth_view[240, 321].tolist() == [255, 0, 0]  # outlier rejected red
    assert depth_view[240, 322].tolist() == [0, 0, 0]    # accepted black
    assert np.allclose(result["pixel"], [320, 240])
    # Measured size color is independent of taught values; red items remain visible.
    measured = {"length_mm": 70., "width_mm": 28.}
    for measured_size, taught, expected in (
            (measured, settings["geometry"], True),
            ({"length_mm": 95., "width_mm": 28.}, settings["geometry"], False),
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
    # A farther-than-plane point can look inside blue while its actual metric XY
    # is outside; metric containment remains independently mandatory.
    near_wall = {**item, "center": np.array([440., 240.]),
                 "polygon": polygon + [120., 0.], "rectangle": polygon + [120., 0.]}
    wall_settings = {**settings, "bin_clearance": {
        "p1_p2": None, "p2_p3": None, "p3_p4": 100., "p4_p1": None},
        "geometry": {**settings["geometry"], "height": 90., "width": 36.}}
    wall_depth = np.full_like(depth, 900)
    _, _, wall_candidates, wall_rejected = generate_candidates(
        [near_wall], rgb, wall_depth, context, wall_settings, cv2, np)
    assert not wall_candidates
    assert wall_rejected == [{"source_index": 0,
                              "reason": "pick point outside bin-wall clearance"}]
    # A closer item can be metric-safe but visibly outside the blue platform-Z=0
    # projection. Reject it too so frozen candidate feedback matches the overlay.
    parallax = {**item, "center": np.array([540., 240.]),
                "polygon": polygon + [220., 0.], "rectangle": polygon + [220., 0.]}
    parallax_settings = {**settings, "bin_clearance": {
        "p1_p2": None, "p2_p3": None, "p3_p4": 30., "p4_p1": None}}
    inner = np.asarray(inset_bin_roi(context["roi"], parallax_settings["bin_clearance"]))
    projected_inner = project_bin_roi({**context, "roi": inner}, cv2, np)
    parallax_position = transform[:3, :3] @ (
        np.array([(540.-320.)/1000., 0., 1.]) * .7) + transform[:3, 3]
    assert inside(parallax_position[:2], inner, cv2, np)
    assert not inside(parallax["center"], projected_inner, cv2, np)
    _, _, parallax_candidates, parallax_rejected = generate_candidates(
        [parallax], rgb, depth, context, parallax_settings, cv2, np)
    assert not parallax_candidates
    assert parallax_rejected == [{
        "source_index": 0, "reason": "pick pixel outside projected bin-wall clearance"}]
    boundary_center = np.array([532.5, 240.])
    boundary = {**item, "center": boundary_center,
                "polygon": polygon + [212.5, 0.], "rectangle": polygon + [212.5, 0.]}
    _, _, boundary_candidates, boundary_rejected = generate_candidates(
        [boundary], rgb, depth, context, parallax_settings, cv2, np)
    assert len(boundary_candidates) == 1 and not boundary_rejected
    # Green ROI membership is intersection-based: a partially crossing
    # footprint is eligible when its final depth-derived point remains inside.
    crossing = {**item, "center": np.array([540., 240.]),
                "polygon": polygon + [220., 0.], "rectangle": polygon + [220., 0.]}
    _, _, crossing_candidates, crossing_rejected = generate_candidates(
        [crossing], rgb, depth, context, settings, cv2, np)
    assert len(crossing_candidates) == 1 and not crossing_rejected
    disjoint = {**item, "center": np.array([650., 240.]),
                "polygon": polygon + [330., 0.], "rectangle": polygon + [330., 0.]}
    _, _, disjoint_candidates, disjoint_rejected = generate_candidates(
        [disjoint], rgb, depth, context, settings, cv2, np)
    assert not disjoint_candidates
    assert disjoint_rejected == [{"source_index": 0,
                                  "reason": "item footprint fully outside bin ROI"}]
    for z in (600, 900):
        depth[:] = z
        depth_settings = {**settings, "geometry": {**settings["geometry"],
                          "height": z*.1, "width": z*.04}}
        _, _, points, _ = generate_candidates([item], rgb, depth, context, depth_settings, cv2, np)
        assert np.allclose([points[0]["length"], points[0]["width"]], [z*.0001, z*.00004])
        assert np.isclose(points[0]["position"][2], .8-z/1000)
    depth[:] = 700
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
    tiny_roi = {**context, "roi": [[-.02, -.02], [-.02, .02], [.02, .02], [.02, -.02]],
                "pick_planning": {**context["pick_planning"],
                                  "link6_from_robot_camera": np.eye(4).tolist()}}
    # Item overlap/pick-point gates pass, but a 90 mm camera cannot fit in 40 mm.
    tiny_result = generate_candidates([item], rgb, depth, tiny_roi, settings, cv2, np)
    assert not tiny_result[2] and "robot-camera body" in tiny_result[3][0]["reason"]
    # Mount width vertically and center the offset housing: its 30×25 mm
    # projection fits and item overlap stays eligible.
    tiny_roi["pick_planning"]["link6_from_robot_camera"] = [
        [1., 0., 0., .01077], [0., 0., -1., 0.], [0., 1., 0., 0.], [0., 0., 0., 1.]]
    assert len(generate_candidates([item], rgb, depth, tiny_roi, settings, cv2, np)[2]) == 1
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

        def __getitem__(self, index):
            return Tensor(self.value[index])

        def __int__(self):
            return int(self.value)

        def __float__(self):
            return float(self.value)

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
    # Teaching shares the measured surface plane and depth gates, while size/class
    # rejection does not hide the detected outline or an available measurement.
    measured = preview_detections(native, "mask", {1: "test"}, 100, context, "", cv2, np,
                                  diameter_mm=30., depth_mm=depth, quality=settings["quality"])
    assert len(measured) == 1
    assert np.allclose(list(measured[0]["measurement"].values()), [70., 28.])
    assert measured[0]["measurement_error"] == ""
    outside_polygon = polygon + [350., 0.]
    outside_boxes = Boxes(cls=Tensor([1]), conf=Tensor([.8]),
                          xyxy=Tensor([[620, 220, 720, 260]]))
    outside_native = SimpleNamespace(
        boxes=outside_boxes, obb=None, masks=SimpleNamespace(xy=[outside_polygon]))
    assert preview_detections(
        outside_native, "mask", {1: "test"}, 100, context, "", cv2, np,
        diameter_mm=30., depth_mm=depth, quality=settings["quality"]) == []
    assert preview_detections(
        outside_native, "none", {1: "test"}, 100, context, "", cv2, np,
        diameter_mm=30., depth_mm=depth, quality=settings["quality"]) == []
    # Without calibrated ROI geometry the same raw detections remain visible.
    assert len(preview_detections(
        outside_native, "mask", {1: "test"}, 100, None, "No calibration", cv2, np,
        diameter_mm=30., depth_mm=depth, quality=settings["quality"])) == 1
    for source in ("mask", "obb"):
        geometry_object = objects_from_result(native, source, {1: "test"}, 100, cv2, np)[0]
        size = plane_dimensions(geometry_object["rectangle"], context, cv2, np, surface_z=.1)[:2]
        unfiltered = preview_detections(native, source, {1: "test"}, 100, context, "", cv2, np,
                                        diameter_mm=30., depth_mm=depth, quality=settings["quality"])
        assert np.allclose(size, np.array(list(unfiltered[0]["measurement"].values())) / 1000)
        center, radius, outline = depth_sampling_circle(
            geometry_object["center"], 30., context, cv2, np)
        assert radius == .015 and not unfiltered[0]["sampling_circle_error"]
        assert np.array_equal(outline, unfiltered[0]["sampling_circle"])
        assert np.allclose(np.ptp(outline, axis=0), [37.5, 37.5])
        larger = preview_detections(native, source, {1: "test"}, 100, context, "", cv2, np,
                                    diameter_mm=60., depth_mm=depth, quality=settings["quality"])
        assert np.allclose(np.ptp(larger[0]["sampling_circle"], axis=0), [75., 75.])
    unavailable = preview_detections(native, "mask", {1: "test"}, 100, None,
                                     "No calibration", cv2, np, diameter_mm=30., depth_mm=depth, quality=settings["quality"])
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
                                  diameter_mm=30., depth_mm=depth, quality=settings["quality"])
    dims = plane_dimensions(mask_object["rectangle"], tilt_context, cv2, np,
                            surface_z=float((tilted @ [0., 0., .7, 1.])[2]))[:2]
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
    assert axes[240, 300].tolist() == [0, 255, 0]  # Long Y green, through middle.
    assert axes[230, 320].tolist() == [255, 0, 0]  # Short X red, through middle.
    assert axes[220, 270].tolist() == rgb[220, 270].tolist()  # No rectangle outline.
    from item_perception_yolo.yolo_worker_native import render_result

    class NoBoxes:
        def __getattr__(self, key):
            assert key not in ("rectangle", "polylines"), "YOLO box rendering is forbidden"
            return getattr(cv2, key)
    masked, count = render_result(native, rgb, {1: "test"}, "segment", 100, NoBoxes(), np)
    assert count == 1 and not np.array_equal(masked, rgb)  # Mask remains visible.
    omitted, count = render_result(
        native, rgb, {1: "test"}, "segment", 100, NoBoxes(), np,
        included_indices=set())
    assert count == 1 and np.array_equal(omitted, rgb)  # Disjoint masks are absent.
    included, count = render_result(
        native, rgb, {1: "test"}, "segment", 100, NoBoxes(), np,
        included_indices={0})
    assert count == 1 and np.array_equal(included, masked)
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


def exercise_resolution_depth_coverage():
    import cv2
    import numpy as np
    from item_perception_yolo.item_geometry import generate_candidates, project
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS

    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .8
    mount = np.eye(4)
    mount[0, 3] = .04
    physical = np.array([[-.04, -.016, .1], [.04, -.016, .1],
                         [.04, .016, .1], [-.04, .016, .1]])
    settings = {"geometry_source": "mask", "bin_clearance": dict.fromkeys(
        ("p1_p2", "p2_p3", "p3_p4", "p4_p1")),
        "geometry": {"depth_frame_count": 3, "nearby_depth_radius_mm": 150., "nearby_depth_height_mm": 60.,
                     "height": 80., "width": 32., "tolerance": .1, "pickdepth_radius": 10.},
        "quality": dict(QUALITY_DEFAULTS), "yolo": {"class_ids": [1], "confidence": .5}}
    for scale in (1, 2):
        width, height, focal = 640 * scale, 360 * scale, 300. * scale
        center = np.array([width / 2, height / 2])
        camera = {"k": [focal, 0., center[0], 0., focal, center[1], 0., 0., 1.],
                  "d": [0.] * 5}
        context = {"camera": camera, "depth_camera": camera,
                   "platform_from_optical": transform.tolist(),
                   "roi": [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]],
                   "pick_planning": {
                       "home_matrix": np.eye(4).tolist(),
                       "base_from_platform": np.eye(4).tolist(),
                       "link6_from_robot_camera": mount.tolist(),
                       "pick_rotation_deg": 0., "standoff_height_mm": 0.}}
        polygon = project(physical, camera, transform, cv2, np).astype(np.float32)
        item = {"index": 0, "class_id": 1, "class_name": "test", "confidence": .9,
                "polygon": polygon, "rectangle": polygon, "center": center}
        yy, xx = np.mgrid[:height, :width]
        circle = np.argwhere((xx - center[0])**2 + (yy - center[1])**2
                             <= (.005 * focal / .8)**2)
        total = len(circle)
        for good in (0, total // 2, (total + 1) // 2, total):
            depth = np.zeros((height, width), np.uint16)
            depth[tuple(circle[:good].T)] = 700
            _, _, candidates, rejected = generate_candidates(
                [item], np.zeros((height, width, 3), np.uint8), depth,
                context, settings, cv2, np)
            if good / total >= .5:
                assert len(candidates) == 1 and not rejected
                assert candidates[0]["accepted_depth_count"] == good
                assert candidates[0]["rejected_depth_count"] == total - good
                assert np.allclose(candidates[0]["position"], [0., 0., .1])
                if scale == 1:
                    assert good < 30
            else:
                assert not candidates and "requires 50.0%" in rejected[0]["reason"]


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
               "roi": [[-.7, -.4], [-.7, .4], [.7, .4], [.7, -.4]],
               "pick_planning": {
                   "home_matrix": np.eye(4).tolist(),
                   "base_from_platform": np.eye(4).tolist(),
                   "link6_from_robot_camera": [[1., 0., 0., 0.], [0., 1., 0., -.07],
                                                [0., 0., 1., 0.], [0., 0., 0., 1.]],
                   "pick_rotation_deg": 0.0, "standoff_height_mm": 0.0}}
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
    length, width, _ = plane_dimensions(polygon, context, cv2, np, surface_z=.1)
    settings = {"geometry_source": "mask", "quality": dict(QUALITY_DEFAULTS),
                "bin_clearance": {"p1_p2": None, "p2_p3": None,
                                  "p3_p4": None, "p4_p1": None},
                "yolo": {"class_ids": [1], "confidence": .5},
                "geometry": {"depth_frame_count": 3, "nearby_depth_radius_mm": 150., "nearby_depth_height_mm": 60.,
                             "height": length*1000, "width": width*1000,
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
               "roi": [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]],
               "pick_planning": {
                   "home_matrix": np.eye(4).tolist(),
                   "base_from_platform": np.eye(4).tolist(),
                   "link6_from_robot_camera": [[1., 0., 0., 0.], [0., 1., 0., -.07],
                                                [0., 0., 1., 0.], [0., 0., 0., 1.]],
                   "pick_rotation_deg": 0.0, "standoff_height_mm": 0.0}}
    settings = {"geometry_source": "mask", "quality": dict(QUALITY_DEFAULTS),
                "bin_clearance": {"p1_p2": None, "p2_p3": None,
                                  "p3_p4": None, "p4_p1": None},
                "geometry": {"depth_frame_count": 3, "nearby_depth_radius_mm": 150., "nearby_depth_height_mm": 60.,
                             "height": 70., "width": 28., "tolerance": .1, "pickdepth_radius": 30.},
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
        assert [c["source_index"] for c in valid] == [1]
        assert rejected == [{"source_index": 3, "reason": "class not selected"}]
        baseline_depth = render_depth(depth, {**config["quality"], "depth_min_mm": 500.}, cv2, np)
        only_rgb, only_depth, _, _ = generate_candidates(
            [objects[1]], rgb, depth, context, config, cv2, np, candidate_limit=1)
        # The chosen item's radius can cross another item. No pick annotations
        # or separate nearby rings remain for valid items excluded by the cap.
        assert np.array_equal(rgb1, only_rgb)
        assert np.array_equal(depth1, only_depth)
        assert rgb1[250, 270].tolist() == [0, 255, 0]  # Single valid-size rectangle.
        assert rgb1[240, 320].tolist() == [255, 255, 255]
        assert depth1[240, 320].tolist() == [255, 0, 0]  # Null never enters MAD.
        assert depth1[240, 321].tolist() == [255, 0, 0]
        assert depth1[240, 322].tolist() == [0, 0, 0]
        assert rgb1[240, 70, 1] > 240 and depth1[240, 70, 1] > 240  # ROI retained on both.
        assert np.allclose(valid[0]["position"], [0, 0, .1])
        rgb3, depth3, all_valid, _ = generate_candidates(
            objects, rgb, depth, context, config, cv2, np, candidate_limit=3)
        assert [c["source_index"] for c in all_valid] == [1, 0, 2]
        assert valid == all_valid[:1]  # Early termination keeps the same ranked prefix.
        assert not np.array_equal(rgb3[220:270, 130:230], rgb1[220:270, 130:230])
        assert not np.array_equal(depth3[220:270, 130:230], depth1[220:270, 130:230])
        # No items is a frozen raw pair + bin ROI, not all rejected-object overlays.
        invalid = {**config, "yolo": {**config["yolo"], "class_ids": [9]}}
        empty_rgb, empty_depth, empty, _ = generate_candidates(
            objects, rgb, depth, context, invalid, cv2, np, candidate_limit=3)
        assert empty == []
        assert np.array_equal(empty_rgb[200:300, 260:380], rgb[200:300, 260:380])
        assert np.array_equal(empty_depth[200:300, 260:380], baseline_depth[200:300, 260:380])


def exercise_robot_camera_rejects_before_ranking():
    import cv2
    import numpy as np
    from item_perception_yolo.item_geometry import generate_candidates
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS

    camera = {"k": [1000., 0., 320., 0., 1000., 240., 0., 0., 1.], "d": [0.] * 5}
    optical = np.diag([1., -1., -1., 1.])
    optical[2, 3] = .8
    link6_camera = np.eye(4)
    link6_camera[0, 3] = .25
    context = {"camera": camera, "depth_camera": camera,
               "platform_from_optical": optical.tolist(),
               "roi": [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]],
               "pick_planning": {
                   "home_matrix": np.eye(4).tolist(),
                   "base_from_platform": np.eye(4).tolist(),
                   "link6_from_robot_camera": link6_camera.tolist(),
                   "pick_rotation_deg": 0., "standoff_height_mm": 90.}}
    settings = {"geometry_source": "mask", "quality": dict(QUALITY_DEFAULTS),
                "bin_clearance": dict.fromkeys(("p1_p2", "p2_p3", "p3_p4", "p4_p1")),
                "geometry": {"depth_frame_count": 3, "nearby_depth_radius_mm": 150., "nearby_depth_height_mm": 60.,
                             "height": 70., "width": 28., "tolerance": .1,
                             "pickdepth_radius": 30.},
                "yolo": {"class_ids": [1], "confidence": .5}}
    rect = np.array([[-50, -20], [50, -20], [50, 20], [-50, 20]], np.float32)
    objects = [{"index": index, "class_id": 1, "class_name": "part", "confidence": .8,
                "rectangle": rect + [x, 240], "polygon": rect + [x, 240],
                "center": np.array([x, 240.])}
               for index, x in ((0, 320), (1, 420))]
    rgb = np.full((480, 640, 3), 80, np.uint8)
    depth = np.full((480, 640), 700, np.uint16)
    overlay, depth_view, candidates, rejected = generate_candidates(
        objects, rgb, depth, context, settings, cv2, np, candidate_limit=1)
    assert [entry["source_index"] for entry in candidates] == [1]
    assert rejected == [{"source_index": 0, "reason":
                         "robot-camera body extends outside bin ROI for normal and "
                         "180-degree attitudes"}]
    selected = candidates[0]["robot_camera_clearance"]
    assert selected["mirrored"] is True
    assert selected["normal_platform_xy"][0] > .2
    assert -.2 < selected["selected_platform_xy"][0] < .2
    # The native preview displays exactly the selected mirrored footprint on
    # both images, and excludes the rejected item from the capped pose batch.
    assert np.any(np.all(overlay == [255, 0, 255], axis=2))
    assert np.any(np.all(depth_view == [255, 0, 255], axis=2))


def exercise_nearby_depth_filter():
    import copy
    import cv2
    import numpy as np
    from item_perception_yolo.item_geometry import (
        generate_candidates, nearby_depth_check, selected_pose, usable_scene_depth, rays)
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    from item_perception_yolo.pick_planning import rpy_matrix
    cv2.setNumThreads(1)
    geometry = {"depth_frame_count": 3, "nearby_depth_radius_mm": 150., "nearby_depth_height_mm": 60.,
                "height": 70., "width": 28., "tolerance": .1, "pickdepth_radius": 30.}
    # Inclusive cylinder boundaries: horizontal radius, not 3D distance.
    for point, rejected in [([.150, 0., .060], True), ([.15001, 0., .100], False),
                            ([.100, 0., .05999], False), ([0., .150, .200], True),
                            ([0., 0., -.100], False)]:
        optical = np.array(point) * [1., -1., -1.] + [0., 0., .8]
        scene = {"pixels": np.array([[42, 12], [0, 0]]),
                 "points": np.array([optical, [0., 0., .8]]),
                 "heights_mm": np.array([point[2] * 1000, 0.]),
                 "plane": np.array([0., 0., -1., .8])}
        drawing = {}
        try:
            evidence = nearby_depth_check(scene, np.array([0., 0., .8]), geometry, np,
                                          visualization=drawing)
        except ValueError as exc:
            assert rejected and "(42, 12)" in str(exc)
        else:
            assert not rejected, evidence
        assert drawing["blocked"] is rejected
        assert len(drawing["blocking_points"]) == int(rejected)
        if rejected:
            assert np.array_equal(drawing["blocking_points"][0], optical)
    camera = {"k": [1000., 0., 320., 0., 1000., 240., 0., 0., 1.], "d": [0.] * 5}
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .8
    context = {"camera": camera, "depth_camera": camera,
               "platform_from_optical": transform.tolist(),
               "roi": [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]],
               "pick_planning": {"home_matrix": np.eye(4).tolist(),
                                 "base_from_platform": np.eye(4).tolist(),
                                 "link6_from_robot_camera": np.eye(4).tolist(),
                                 "pick_rotation_deg": 0., "standoff_height_mm": 0.}}
    settings = {"geometry_source": "mask", "geometry": geometry,
                "quality": dict(QUALITY_DEFAULTS),
                "bin_clearance": dict.fromkeys(("p1_p2", "p2_p3", "p3_p4", "p4_p1")),
                "yolo": {"class_ids": [1], "confidence": .5}}
    rgb = np.full((480, 640, 3), 80, np.uint8)
    depth = np.full((480, 640), 700, np.uint16)
    polygon = np.array([[270, 220], [370, 220], [370, 260], [270, 260]], np.float32)
    item = {"index": 0, "class_id": 1, "class_name": "test", "confidence": .7,
            "polygon": polygon, "rectangle": polygon, "center": np.array([320., 240.])}
    second = {**item, "index": 1, "polygon": polygon - [120, 0],
              "rectangle": polygon - [120, 0], "center": np.array([200., 240.])}
    # A single depth point OUTSIDE the item and its sampling circle rejects the
    # best-ranked candidate. The farther saved candidate is still returned.
    depth[240, 554] = 640  # base XY = (149.76, 0) mm; Z = pick + 60 mm.
    overlay, depth_view, candidates, rejected = generate_candidates(
        [item, second], rgb, depth, context, settings, cv2, np, candidate_limit=1)
    assert [c["source_index"] for c in candidates] == [1]
    assert len(rejected) == 1 and "60.00 mm above candidate floor height" in rejected[0]["reason"]
    from item_perception_yolo.nearby_depth_overlay import RADIUS_COLOR, HEIGHT_COLOR
    for pixels in (overlay, depth_view):
        # A rejected item remains explainable even after the valid batch is capped.
        assert pixels[240, 554].tolist() == [255, 0, 0]
        assert np.any(np.linalg.norm(pixels.astype(float) - RADIUS_COLOR, axis=2) < 30)
        assert np.any(np.linalg.norm(pixels.astype(float) - HEIGHT_COLOR, axis=2) < 30)
    clicked = {**item, "source_index": item["index"]}
    clicked_rgb, clicked_depth, candidates, reasons = selected_pose(
        clicked, rgb, depth, context, settings, cv2, np)
    assert not candidates and reasons == rejected  # Same clicked/service filter.
    assert clicked_rgb[240, 554].tolist() == [255, 0, 0]
    assert clicked_depth[240, 554].tolist() == [255, 0, 0]
    # Production NO_VALID_ITEMS images still identify the blocking point.
    for limit in (None, 1):
        blocked_rgb, blocked_depth, poses, reasons = generate_candidates(
            [item], rgb, depth, context, settings, cv2, np, candidate_limit=limit)
        assert not poses and reasons == rejected
        assert blocked_rgb[240, 554].tolist() == [255, 0, 0]
        assert blocked_depth[240, 554].tolist() == [255, 0, 0]
    # Outside the outer ROI does not count, including in OBB mode.
    narrower = {**context, "roi": [[-.14, -.14], [-.14, .14], [.14, .14], [.14, -.14]]}
    _, _, candidates, reasons = generate_candidates(
        [item], rgb, depth, narrower, {**settings, "geometry_source": "obb"}, cv2, np)
    assert len(candidates) == 1 and not reasons
    # Tool/standoff compensation cannot turn an obstructed item into a valid one.
    raised = copy.deepcopy(context)
    for standoff in (1., 70., 200.):
        raised["pick_planning"]["standoff_height_mm"] = standoff
        for limit in (None, 1):
            _, _, candidates, reasons = generate_candidates(
                [item], rgb, depth, raised, settings, cv2, np, candidate_limit=limit)
            assert not candidates and reasons == rejected
        _, _, candidates, reasons = selected_pose(
            clicked, rgb, depth, raised, settings, cv2, np)
        assert not candidates and reasons == rejected
    # Last collision batch recorded 8.616 mm above Link6 with 70 mm standoff:
    # the same nearby maximum is 78.616 mm above the item and exceeds 50 mm.
    measured = 78.6163444482813
    collision_scene = {"pixels": np.array([[42, 12]]),
                       "points": np.array([[.1, 0., .8 - measured / 1000.]]),
                       "heights_mm": np.array([measured]),
                       "plane": np.array([0., 0., -1., .8])}
    with pytest.raises(ValueError, match="78.62 mm above candidate floor height"):
        nearby_depth_check(collision_scene, np.array([0., 0., .8]),
                           {**geometry, "nearby_depth_height_mm": 50.}, np)
    # Increasing standoff must not alter accepted evidence either.
    safe_settings = {**settings, "geometry": {**geometry, "nearby_depth_height_mm": 61.}}
    for standoff in (0., 70.):
        raised["pick_planning"]["standoff_height_mm"] = standoff
        _, _, candidates, reasons = generate_candidates(
            [item], rgb, depth, raised, safe_settings, cv2, np)
        assert not reasons and len(candidates) == 1
        assert abs(candidates[0]["nearby_depth_filter"]["maximum_height_difference_mm"] - 60) < 1e-6
    # Custom settings really alter eligibility; no hidden 150/60 constants.
    for changes in ({"nearby_depth_radius_mm": 149.}, {"nearby_depth_height_mm": 61.}):
        _, _, candidates, reasons = generate_candidates(
            [item], rgb, depth, context, {**settings, "geometry": {**geometry, **changes}}, cv2, np)
        assert not reasons and len(candidates) == 1
    # Only finite, positive, in-range original depth samples are usable. Do not
    # suppress a high reading as a statistical outlier or use RGB distortion.
    depth = np.zeros((480, 640), np.float64)
    depth[240, 554:561] = [640, 0, float("nan"), float("inf"), -1, 199, 1001]
    tilted = copy.deepcopy(context)
    tilted["camera"]["d"] = [.3, .1, 0., 0., 0.]
    tilted["depth_camera"] = {**camera, "d": [.02, -.01, 0., 0., 0.]}
    base = np.eye(4)
    base[:3, :3] = rpy_matrix(.3, -.5, .2)
    base[:3, 3] = [.4, -.2, .7]
    tilted["pick_planning"]["base_from_platform"] = base.tolist()
    scene = usable_scene_depth(depth, tilted, settings["quality"], cv2, np)
    pixels, points = scene["pixels"], scene["points"]
    assert pixels.tolist() == [[554, 240]]
    optical = rays([[554, 240]], tilted["depth_camera"], cv2, np)[0] * .640
    assert np.allclose(points[0], optical)
    try:
        nearby_depth_check(scene, points[0] + [-.15, 0., .06], geometry, np)
    except ValueError:
        pass
    else:
        raise AssertionError("Robot base tilt must not affect camera XY/floor-relative clearance")


def exercise_ranked_nearby_acquisition():
    import cv2
    import numpy as np
    from unittest.mock import patch
    from item_perception_yolo import item_geometry as geometry
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
    camera = {"k": [1000., 0., 320., 0., 1000., 240., 0., 0., 1.], "d": [0.] * 5}
    transform = np.diag([1., -1., -1., 1.])
    transform[2, 3] = .8
    context = {"camera": camera, "depth_camera": camera,
               "platform_from_optical": transform.tolist(),
               "roi": [[-.2, -.15], [-.2, .15], [.2, .15], [.2, -.15]],
               "pick_planning": {"home_matrix": np.eye(4).tolist(),
                                 "base_from_platform": np.eye(4).tolist(),
                                 "link6_from_robot_camera": np.eye(4).tolist(),
                                 "pick_rotation_deg": 0., "standoff_height_mm": 70.}}
    settings = {"geometry_source": "mask", "quality": dict(QUALITY_DEFAULTS),
                "geometry": {"depth_frame_count": 3, "nearby_depth_radius_mm": 20., "nearby_depth_height_mm": 60.,
                             "height": 70., "width": 28., "tolerance": .1, "pickdepth_radius": 30.},
                "bin_clearance": dict.fromkeys(("p1_p2", "p2_p3", "p3_p4", "p4_p1")),
                "yolo": {"class_ids": [1], "confidence": .5}}
    rgb = np.full((480, 640, 3), 80, np.uint8)
    depth = np.full((480, 640), 700, np.uint16)
    polygon = np.array([[-50, -20], [50, -20], [50, 20], [-50, 20]], np.float32)
    items = [{"index": i, "class_id": 1, "class_name": "part", "confidence": .7,
              "center": np.array([320.+40*i, 240.]), "polygon": polygon+[320+40*i, 240],
              "rectangle": polygon+[320+40*i, 240]} for i in range(6)]
    # Only candidates 0 and 5 are obstructed. Reverse detector order to prove
    # that selection follows exact depth-derived rank rather than YOLO order.
    depth[240, 300] = depth[240, 540] = 640
    for limit, expected, checked, deferred in (
            (1, [1], 2, [2, 3, 4, 5]), (3, [1, 2, 3], 4, [4, 5]),
            (5, [1, 2, 3, 4], 6, [])):
        unchecked = []
        with patch.object(geometry, "nearby_depth_check",
                          wraps=geometry.nearby_depth_check) as scan:
            with patch.object(geometry, "usable_scene_depth", wraps=geometry.usable_scene_depth) \
                    as scene:
                overlay, _, candidates, rejected = geometry.generate_candidates(
                    items[::-1], rgb, depth, context, settings, cv2, np,
                    candidate_limit=limit, unchecked=unchecked)
        assert [candidate["source_index"] for candidate in candidates] == expected
        assert unchecked == deferred
        assert scan.call_count == checked and scene.call_count == 1
        assert np.allclose([call.args[1][0] for call in scan.call_args_list],
                           np.arange(checked) * .028)
        assert [entry["source_index"] for entry in rejected] == ([0, 5] if limit == 5 else [0])
        assert all("nearby_depth_filter" in candidate for candidate in candidates)
        assert overlay[240, 300].tolist() == [255, 0, 0]
        assert (overlay[240, 540].tolist() == [255, 0, 0]) is (limit == 5)
    depth[240, 300] = 700
    with patch.object(geometry, "nearby_depth_check", wraps=geometry.nearby_depth_check) as scan:
        unchecked = []
        _, _, candidates, rejected = geometry.generate_candidates(
            items[::-1], rgb, depth, context, settings, cv2, np,
            candidate_limit=3, unchecked=unchecked)
        assert scan.call_count == 3 and not rejected
        assert [candidate["source_index"] for candidate in candidates] == [0, 1, 2]
        assert unchecked == [3, 4, 5]
    # Rendering and camera-math caching cannot alter eligibility, rank or poses.
    from item_perception_yolo.projection_cache import clear_projection_caches
    clear_projection_caches()
    full = geometry.generate_candidates(items[::-1], rgb, depth, context, settings, cv2, np)
    with patch.object(geometry, "render_depth", side_effect=AssertionError("unrequested image")):
        with patch.object(geometry, "shade_masks", side_effect=AssertionError("unrequested image")):
            compact = geometry.generate_candidates(items[::-1], rgb, depth, context, settings,
                                                   cv2, np, candidate_limit=3, render_images=False)
    drawn = geometry.generate_candidates(items[::-1], rgb, depth, context, settings, cv2, np,
                                         candidate_limit=3)
    assert compact[:2] == (None, None)
    assert compact[2:] == drawn[2:] and compact[2] == full[2][:3]
    # No scene back-projection at all when ordinary geometry rejects every item.
    with patch.object(geometry, "usable_scene_depth", side_effect=AssertionError("unused scan")):
        _, _, candidates, rejected = geometry.generate_candidates(
            items, rgb, depth, context,
            {**settings, "yolo": {"class_ids": [9], "confidence": .5}}, cv2, np,
            candidate_limit=3)
        assert not candidates and len(rejected) == 6


def exercise_nearby_overlay_projection():
    import cv2
    import numpy as np
    from item_perception_yolo.item_geometry import nearby_depth_check
    from item_perception_yolo.nearby_depth_overlay import (
        draw_nearby_depth_overlays, projected_base_points)
    from item_perception_yolo.pick_planning import rpy_matrix
    camera = {"k": [500., 0., 320., 0., 500., 240., 0., 0., 1.],
              "d": [.3, -.1, 0., 0., 0.]}
    depth_camera = {**camera, "d": [0.] * 5}
    transform = np.eye(4)
    transform[:3, :3] = rpy_matrix(.3, -.2, .1)
    transform[:3, 3] = [.4, -.2, .8]
    optical_point = np.array([.2, .03, .7])
    point = transform[:3, :3] @ optical_point + transform[:3, 3]
    for calibration in (camera, depth_camera):
        actual = projected_base_points([point], transform, calibration, cv2, np)
        expected, _ = cv2.projectPoints(
            optical_point.reshape(1, 3), np.zeros(3), np.zeros(3),
            np.asarray(calibration["k"]).reshape(3, 3), np.asarray(calibration["d"]))
        assert np.allclose(actual, expected[:, 0, :])
    # Different RGB/depth distortion locates the same blocker in different pixels.
    assert not np.allclose(projected_base_points([point], transform, camera, cv2, np),
                           projected_base_points([point], transform, depth_camera, cv2, np))
    geometry = {"nearby_depth_radius_mm": 150., "nearby_depth_height_mm": 50.}
    check = {}
    with pytest.raises(ValueError):
        nearby_depth_check({"pixels": np.array([[1, 1]]),
                            "points": np.array([optical_point]),
                            "heights_mm": np.array([100.]),
                            "plane": np.array([0., 0., -1., .8])},
                           optical_point + [-.1, 0., .06], geometry, np, visualization=check)
    context = {"camera": camera, "depth_camera": depth_camera,
               "platform_from_optical": transform.tolist(),
               "pick_planning": {"base_from_platform": np.eye(4).tolist()}}
    views = tuple(np.zeros((480, 640, 3), np.uint8) for _ in range(2))
    draw_nearby_depth_overlays(views, context, {2: check}, cv2, np)
    for view, calibration in zip(views, (camera, depth_camera)):
        pixel = np.rint(projected_base_points([point], transform, calibration, cv2, np)[0])
        assert view[int(pixel[1]), int(pixel[0])].tolist() == [255, 0, 0]
    # A 150 mm radius at surface depth 700 mm is not a 150-pixel circle.
    top_down = np.diag([1., -1., -1., 1.])
    top_down[2, 3] = .8
    edges = projected_base_points([[.15, 0., .1], [.15, 0., .15]],
                                  top_down, depth_camera, cv2, np)
    assert np.allclose(edges[:, 0], 320. + 500. * .15 / np.array([.7, .65]))
    # Even partially clipped / behind-camera rings cannot reject a valid pose.
    check["item_xyz"] = np.array([0., 0., 0.])
    check["evidence"]["height_mm"] = 10000.
    draw_nearby_depth_overlays(views, context, {2: check}, cv2, np)
    hidden = transform[:3, 3] - transform[:3, 2]
    assert np.isnan(projected_base_points([hidden], transform, camera, cv2, np)).all()
    unchanged = views[0].copy()
    draw_nearby_depth_overlays(views, context, {}, cv2, np)
    assert np.array_equal(views[0], unchanged)


def exercise_height_corrected_size():
    import copy
    from types import SimpleNamespace
    import cv2
    import numpy as np
    from item_perception_yolo.item_geometry import (
        generate_candidates, selected_pose, preview_detections, objects_from_result,
        plane_dimensions, project, classify_size,
    )
    from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS

    cv2.setNumThreads(1)
    cv2.ocl.setUseOpenCL(False)
    camera = {"k": [500., 0., 320., 0., 500., 240., 0., 0., 1.], "d": [0.] * 5}
    transform = np.diag([1., -1., -1., 1.])
    transform[:3, 3] = [.02, -.03, .9]
    context = {"camera": camera, "depth_camera": camera,
               "platform_from_optical": transform.tolist(),
               "roi": [[-.3, -.3], [-.3, .3], [.3, .3], [.3, -.3]],
               "pick_planning": {"home_matrix": np.eye(4).tolist(),
                                 "base_from_platform": np.eye(4).tolist(),
                                 "link6_from_robot_camera": np.eye(4).tolist(),
                                 "pick_rotation_deg": 0., "standoff_height_mm": 90.}}
    quality = dict(QUALITY_DEFAULTS)
    settings = {"geometry_source": "mask", "quality": quality,
                "bin_clearance": dict.fromkeys(("p1_p2", "p2_p3", "p3_p4", "p4_p1")),
                "geometry": {"depth_frame_count": 3, "height": 80., "width": 50.,
                             "tolerance": .1, "pickdepth_radius": 30.,
                             "nearby_depth_radius_mm": 100., "nearby_depth_height_mm": 60.},
                "yolo": {"class_ids": [1], "confidence": .5}}
    rgb = np.zeros((480, 640, 3), np.uint8)

    class Tensor:
        def __init__(self, values):
            self.values = np.asarray(values)

        def cpu(self):
            return self

        def numpy(self):
            return self.values

    class Boxes(SimpleNamespace):
        def __len__(self):
            return 1

    # A fixed physical 80 x 50 mm flat item retains its size at different pile
    # elevations. At 200 mm the old floor projection would be 102.86 x 64.29 mm.
    for elevation in (0., .1, .2):
        physical = np.array([[-.04, -.025, elevation], [.04, -.025, elevation],
                             [.04, .025, elevation], [-.04, .025, elevation]])
        rectangle = project(physical, camera, transform, cv2, np).astype(np.float32)
        native = SimpleNamespace(
            boxes=Boxes(cls=Tensor([1]), conf=Tensor([.9])),
            masks=SimpleNamespace(xy=[rectangle]),
            obb=Boxes(cls=Tensor([1]), conf=Tensor([.9]), xyxyxyxy=Tensor([rectangle])))
        depth = np.full((480, 640), (.9-elevation)*1000, np.float32)
        center = np.rint(rectangle.mean(axis=0)).astype(int)
        depth[center[1], center[0]] = 0  # Center holes/outliers do not move the ray.
        depth[center[1], center[0]+1] = 999
        for source in ("mask", "obb"):
            config = {**settings, "geometry_source": source}
            objects = objects_from_result(native, source, {1: "part"}, 20, cv2, np)
            displayed = preview_detections(
                native, source, {1: "part"}, 20, context, "", cv2, np,
                diameter_mm=30., depth_mm=depth, quality=quality)
            measurement = displayed[0]["measurement"]
            assert np.allclose(list(measurement.values()), [80., 50.], atol=.001)
            assert classify_size(measurement, settings["geometry"])[0] is True
            assert classify_size(measurement, None)[0] is None
            _, _, candidates, rejected = generate_candidates(
                objects, rgb, depth, context, config, cv2, np, render_images=False)
            assert not rejected and len(candidates) == 1
            candidate = candidates[0]
            assert np.allclose([candidate["length"], candidate["width"]], [.08, .05], atol=1e-6)
            assert np.allclose(candidate["position"], [0., 0., elevation], atol=1e-6)
            assert np.allclose(candidate["pixel"], rectangle.mean(axis=0), atol=1e-4)
            _, _, clicked, reasons = selected_pose(
                displayed[0], rgb, depth, context, config, cv2, np)
            assert not reasons and clicked == candidates
            wrong = {**config, "geometry": {**config["geometry"], "height": 100.}}
            _, _, invalid, reasons = generate_candidates(
                objects, rgb, depth, context, wrong, cv2, np, render_images=False)
            assert not invalid and reasons[0]["reason"] == "height-corrected size outside tolerance"
            # Missing/invalid depth never supplies a floor-size fallback, but the
            # all-class preview retains the outline and an explicit unknown size.
            for missing in (None, np.zeros_like(depth), np.full_like(depth, np.nan),
                            np.full_like(depth, 499.), np.full_like(depth, 1001.)):
                unknown = preview_detections(
                    native, source, {1: "part"}, 20, context, "", cv2, np,
                    diameter_mm=30., depth_mm=missing, quality=quality)
                assert len(unknown) == 1 and unknown[0]["measurement"] is None
                assert unknown[0]["measurement_error"]
                assert classify_size(None, config["geometry"])[0] is None
                if missing is not None:
                    _, _, invalid, reasons = generate_candidates(
                        objects, rgb, missing, context, config, cv2, np, render_images=False)
                    assert not invalid and "insufficient valid depth" in reasons[0]["reason"]
    # Camera tilt/distortion and either platform-Z convention still recover a
    # known flat rectangle using a floor-parallel plane (no item-tilt fitting).
    distorted = {**camera, "d": [.06, -.01, .001, -.002, 0.]}
    rotation, _ = cv2.Rodrigues(np.array([.12, -.16, .2]))
    for sign in (1., -1.):
        flipped = np.diag([1., sign, sign, 1.])
        angled = transform.copy()
        angled[:3, :3] = rotation @ angled[:3, :3]
        angled = flipped @ angled
        physical = np.array([[-.04, -.025, .15*sign], [.04, -.025, .15*sign],
                             [.04, .025, .15*sign], [-.04, .025, .15*sign]])
        pixels = project(physical, distorted, angled, cv2, np)
        measured = plane_dimensions(pixels, {**context, "camera": distorted,
                                            "platform_from_optical": angled.tolist()},
                                    cv2, np, surface_z=.15*sign)
        assert np.allclose(measured[:2], [.08, .05], atol=1e-6)
    # Surface coordinates must remain finite and in front of the camera.
    for invalid_z in (float("nan"), float("inf"), .9, 1.):
        with pytest.raises(ValueError):
            plane_dimensions(rectangle, context, cv2, np, surface_z=invalid_z)
    # Gripper/tool offsets do not participate in item dimensions.
    shifted = copy.deepcopy(context)
    shifted["pick_planning"]["standoff_height_mm"] = 300.
    assert np.allclose(plane_dimensions(rectangle, shifted, cv2, np, surface_z=.2)[:2],
                       [.08, .05], atol=1e-6)


@pytest.mark.parametrize("exercise", [
    "exercise_geometry", "exercise_registered_depth", "exercise_resolution_depth_coverage",
    "exercise_candidate_batch_overlay", "exercise_robot_camera_rejects_before_ranking",
    "exercise_nearby_depth_filter", "exercise_nearby_overlay_projection",
    "exercise_ranked_nearby_acquisition", "exercise_height_corrected_size"])
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
