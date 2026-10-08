"""Native-worker geometry. OpenCV/NumPy are passed in from the private runtime."""

import math
import time

from .planar_bin_roi import border_in_optical
from .item_teach_core import depth_coverage_ok, inset_bin_roi
from .pick_planning import (
    CAMERA_BODY_CENTER_RGB_M, CAMERA_BODY_REFERENCE_FRAME, CAMERA_BODY_SIZE_RGB_M,
    candidate_pose_in_base, select_pick_attitude)
from .depth_snapshot import item_depth_limits
from .floor_clearance import usable_scene_depth, nearby_depth_check
from .projection_cache import cached_rays, cached_mapping
from .surface_guard import HeightPlane, SurfaceGeometryError


BIN_CLEARANCE_COLOR = (102, 204, 255)
ROBOT_CAMERA_COLOR = (255, 0, 255)
ROBOT_CAMERA_REJECTED_COLOR = (255, 0, 0)


def passive_detection_color(detection, candidates=(), rejected=(), *, tray=False):
    """Size failures win; green requires complete eligibility, never size alone."""
    if detection.get("size_valid") is False or detection.get("size_status") == "fail":
        return (255, 50, 50)
    index = detection["source_index"]
    if not tray and any(r["source_index"] == index and r.get("rejection_stage") == "height"
                        for r in rejected):
        return (255, 220, 0)
    valid = detection.get("valid", False) if tray else any(
        c["source_index"] == index for c in candidates)
    return (0, 220, 0) if valid else (160, 160, 160)


def passive_overlay(image, detections, source, cv2, np, *, candidates=(), rejected=(),
                    tray=False, cameras=None):
    """One completed pane: translucent masks and size rectangles, without annotations."""
    output = image.copy()
    for detection in detections:
        polygon = np.asarray(detection["polygon"], dtype=float)
        if len(polygon) < 3:
            continue
        rectangle = np.asarray(detection.get("rectangle", polygon), dtype=float)
        if cameras is not None:
            polygon = reproject_pixels(polygon, cameras["camera"], cameras["depth_camera"],
                                       cv2, np)
            rectangle = reproject_pixels(rectangle, cameras["camera"],
                                         cameras["depth_camera"], cv2, np)
        color = passive_detection_color(detection, candidates, rejected, tray=tray)
        polygon = np.rint(polygon).astype(np.int32)
        rectangle = np.rint(rectangle).astype(np.int32)
        if source == "mask":
            mask = np.zeros(image.shape[:2], np.uint8)
            cv2.fillPoly(mask, [polygon], 1)
            pixels = mask.astype(bool)
            output[pixels] = np.rint(
                .75 * output[pixels] + .25 * np.asarray(color)).astype(np.uint8)
        cv2.polylines(output, [rectangle], True, color, 2, cv2.LINE_AA)
    return output


def rectangle_axes(rectangle, np):
    """Return long/short pixel midlines and their immutable pick-pixel intersection."""
    rect = np.asarray(rectangle, dtype=np.float64)
    if rect.shape != (4, 2) or not np.isfinite(rect).all():
        raise RuntimeError("Malformed pick rectangle")
    midpoints = (rect + np.roll(rect, -1, axis=0)) / 2
    pairs = np.array([[midpoints[0], midpoints[2]], [midpoints[1], midpoints[3]]])
    spans = np.linalg.norm(pairs[:, 1] - pairs[:, 0], axis=1)
    if np.min(spans) <= 0:
        raise ValueError("Degenerate pick rectangle")
    long_index = int(np.argmax(spans))
    return pairs[long_index], pairs[1-long_index], rect.mean(axis=0)


def draw_pick_axes(overlay, rectangle, cv2, np):
    y_axis, x_axis, center = rectangle_axes(rectangle, np)
    for axis, label, color in ((x_axis, "X", (255, 0, 0)), (y_axis, "Y", (0, 255, 0))):
        start, end = np.rint(axis).astype(int)
        cv2.line(overlay, tuple(start), tuple(end), color, 2, cv2.LINE_AA)
        cv2.putText(overlay, label, tuple(end + [4, -4]), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, color, 1, cv2.LINE_AA)
    pick = tuple(np.rint(center).astype(int))
    cv2.circle(overlay, pick, 5, (0, 0, 0), -1, cv2.LINE_AA)
    cv2.circle(overlay, pick, 3, (255, 255, 255), -1, cv2.LINE_AA)


def shade_masks(rgb, polygons, cv2, np):
    tinted = rgb.copy()
    for polygon in polygons:
        points = np.asarray(polygon)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            raise RuntimeError("Malformed segmentation polygon")
        if len(points) >= 3:
            cv2.fillPoly(tinted, [np.rint(points).astype(np.int32)], (0, 220, 60))
    return cv2.addWeighted(tinted, 0.3, rgb, 0.7, 0)


def draw_pick_geometry(overlay, rectangle, cv2, np, color=(255, 225, 0)):
    """One selected geometry outline: mask-derived rectangle OR native OBB, never both."""
    rectangle_axes(rectangle, np)  # Validate before drawing native coordinates.
    points = np.rint(rectangle).astype(np.int32)
    cv2.polylines(overlay, [points], True, color, 2, cv2.LINE_AA)
    draw_pick_axes(overlay, rectangle, cv2, np)


def project_bin_roi(context, cv2, np):
    """Use Bin Teach's exact plane border and the current camera's lens distortion."""
    roi = np.asarray(context["roi"], dtype=np.float64)
    if roi.shape != (4, 2) or not np.isfinite(roi).all():
        raise RuntimeError("Invalid loaded bin ROI")
    optical = border_in_optical(roi, context["platform_from_optical"], np)
    camera = context["camera"]
    pixels, _ = cv2.projectPoints(
        optical, np.zeros(3), np.zeros(3),
        np.asarray(camera["k"]).reshape(3, 3), np.asarray(camera["d"]),
    )
    return pixels[:, 0, :]


def draw_bin_roi(overlay, context, unavailable_reason, cv2, np):
    """Project saved XY at platform Z=0, including lens distortion. No stale pixels."""
    if context is None:
        return {"visible": False, "reason": unavailable_reason}
    try:
        pixels = project_bin_roi(context, cv2, np)
    except ValueError as exc:
        return {"visible": False, "reason": str(exc)}
    if not np.isfinite(pixels).all() or np.any(np.abs(pixels) > 2_000_000_000):
        return {"visible": False, "reason": "Loaded ROI projection exceeds safe drawing range"}
    # Clip only to the display boundary; never change the projected geometry.
    height, width = overlay.shape[:2]
    visible = False
    for start, end in zip(pixels, np.roll(pixels, -1, axis=0)):
        start = tuple(np.rint(start).astype(int))
        end = tuple(np.rint(end).astype(int))
        intersects, clipped_start, clipped_end = cv2.clipLine((0, 0, width, height), start, end)
        if intersects:
            cv2.line(overlay, clipped_start, clipped_end, (0, 255, 0), 2, cv2.LINE_AA)
            visible = True
    if not visible:
        return {"visible": False, "reason": "Loaded bin ROI border is outside the image"}
    return {"visible": True, "reason": ""}


def draw_bin_clearance(overlay, context, clearance, cv2, np):
    """Draw the optional pick-point-only wall clearance on the platform plane."""
    if context is None:
        return False
    inner = inset_bin_roi(context["roi"], clearance)
    if inner is None:
        return False
    pixels = project_bin_roi({**context, "roi": inner}, cv2, np)
    if not np.isfinite(pixels).all() or np.any(np.abs(pixels) > 2_000_000_000):
        raise ValueError("Bin-wall inset projection exceeds safe drawing range")
    height, width = overlay.shape[:2]
    visible = False
    for start, end in zip(pixels, np.roll(pixels, -1, axis=0)):
        start = tuple(np.rint(start).astype(int))
        end = tuple(np.rint(end).astype(int))
        intersects, clipped_start, clipped_end = cv2.clipLine((0, 0, width, height), start, end)
        if intersects:
            cv2.line(overlay, clipped_start, clipped_end, BIN_CLEARANCE_COLOR,
                     2, cv2.LINE_AA)
            visible = True
    return visible


def draw_robot_camera_footprint(overlay, context, footprint_xy, mirrored, accepted, cv2, np):
    """Draw the complete planned housing outline projected onto platform Z=0."""
    pixels = project([[float(x), float(y), 0.0] for x, y in footprint_xy],
                     context["camera"],
                     np.asarray(context["platform_from_optical"], dtype=np.float64),
                     cv2, np)
    if not np.isfinite(pixels).all() or np.any(np.abs(pixels) > 2_000_000_000):
        raise ValueError("Robot-camera footprint projection exceeds safe drawing range")
    point = tuple(np.rint(pixels.mean(axis=0)).astype(int))
    color = ROBOT_CAMERA_COLOR if accepted else ROBOT_CAMERA_REJECTED_COLOR
    cv2.polylines(overlay, [np.rint(pixels).astype(np.int32)], True, color, 2, cv2.LINE_AA)
    cv2.line(overlay, (point[0] - 5, point[1]), (point[0] + 5, point[1]),
             color, 2, cv2.LINE_AA)
    cv2.line(overlay, (point[0], point[1] - 5), (point[0], point[1] + 5),
             color, 2, cv2.LINE_AA)
    label = "CAM 180" if mirrored else "CAM"
    if not accepted:
        label += " REJECT"
    cv2.putText(overlay, label, (point[0] + 10, point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, .5, color, 2, cv2.LINE_AA)


def rays(pixels, camera, cv2, np):
    return cached_rays(pixels, camera, cv2, np)


def reproject_pixels(pixels, source_camera, target_camera, cv2, np):
    """Map rays between the two registered camera models, never round mask samples."""
    if len(pixels) == 0:
        return np.empty((0, 2), dtype=np.float64)
    return cached_mapping(pixels, source_camera, target_camera, cv2, np)


def on_plane(pixels, camera, transform, cv2, np, *, plane_z=0.0):
    if not math.isfinite(plane_z):
        raise ValueError("Non-finite measurement plane height")
    direction = rays(pixels, camera, cv2, np) @ transform[:3, :3].T
    if np.any(np.abs(direction[:, 2]) < 1e-9):
        raise ValueError("Viewing ray is parallel to platform plane")
    distance = (plane_z - transform[2, 3]) / direction[:, 2]
    if np.any(distance <= 0) or not np.isfinite(distance).all():
        raise ValueError("Platform projection is behind camera")
    return transform[:3, 3] + direction * distance[:, None]


def project(points, camera, transform, cv2, np):
    optical = (np.asarray(points) - transform[:3, 3]) @ transform[:3, :3]
    if np.any(optical[:, 2] <= 0) or not np.isfinite(optical).all():
        raise ValueError("Projected geometry is behind camera")
    projected, _ = cv2.projectPoints(optical, np.zeros(3), np.zeros(3),
                                     np.asarray(camera["k"]).reshape(3, 3),
                                     np.asarray(camera["d"]))
    return projected[:, 0, :]


def polygon_centroid(points, np):
    xy = np.asarray(points)
    nxt = np.roll(xy, -1, axis=0)
    cross = xy[:, 0] * nxt[:, 1] - nxt[:, 0] * xy[:, 1]
    area2 = cross.sum()
    if abs(area2) < 1e-12:
        raise ValueError("Degenerate bin polygon")
    return ((xy + nxt) * cross[:, None]).sum(axis=0) / (3 * area2)


def inside(point, polygon, cv2, np):
    return cv2.pointPolygonTest(np.asarray(polygon, np.float32), tuple(map(float, point)),
                                False) >= 0


def polygons_overlap_or_touch(first, second, cv2, np):
    """Return true unless two finite 2-D polygons are completely disjoint."""
    polygons = []
    for value in (first, second):
        polygon = np.asarray(value, dtype=np.float64)
        if (polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3
                or not np.isfinite(polygon).all()):
            raise RuntimeError("Malformed polygon overlap geometry")
        polygons.append(polygon)
    first, second = polygons
    if (any(inside(point, second, cv2, np) for point in first)
            or any(inside(point, first, cv2, np) for point in second)):
        return True

    # Vertex containment misses the valid crossing case where long, thin
    # polygons intersect but every vertex remains outside the other polygon.
    def cross(start, end, point):
        edge, offset = end - start, point - start
        return float(edge[0] * offset[1] - edge[1] * offset[0])

    def on_segment(start, end, point, epsilon):
        return (abs(cross(start, end, point)) <= epsilon
                and np.all(point >= np.minimum(start, end) - epsilon)
                and np.all(point <= np.maximum(start, end) + epsilon))

    scale = max(1.0, float(np.max(np.abs(np.vstack((first, second))))))
    epsilon = 1e-10 * scale
    for a, b in zip(first, np.roll(first, -1, axis=0)):
        for c, d in zip(second, np.roll(second, -1, axis=0)):
            ab_c, ab_d = cross(a, b, c), cross(a, b, d)
            cd_a, cd_b = cross(c, d, a), cross(c, d, b)
            if ((ab_c > epsilon and ab_d < -epsilon
                 or ab_c < -epsilon and ab_d > epsilon)
                    and (cd_a > epsilon and cd_b < -epsilon
                         or cd_a < -epsilon and cd_b > epsilon)):
                return True
            if (on_segment(a, b, c, epsilon) or on_segment(a, b, d, epsilon)
                    or on_segment(c, d, a, epsilon) or on_segment(c, d, b, epsilon)):
                return True
    return False


def filter_depth(values, minimum, maximum, cv2, np):
    del cv2
    valid = np.isfinite(values) & (values > 0) & (values >= minimum) & (values <= maximum)
    accepted = np.zeros(values.shape, dtype=bool)
    if not valid.any():
        return accepted, None, None
    median = np.median(values[valid])
    sigma = float(1.4826 * np.median(np.abs(values[valid] - median)))
    accepted[valid] = np.abs(values[valid] - median) <= 3 * sigma
    return accepted, float(np.median(values[accepted])), sigma


def objects_from_result(result, source, names, maximum, cv2, np):
    """No box-to-mask/OBB conversion. Both outputs use the explicit chosen source."""
    if source == "obb":
        boxes = result.obb
        if boxes is None:
            raise RuntimeError("Requested OBB output is absent")
        shapes = boxes.xyxyxyxy.cpu().numpy()
    elif source == "mask":
        boxes = result.boxes
        if boxes is None:
            raise RuntimeError("Requested mask output has no class/confidence records")
        if len(boxes) == 0:
            return []
        if result.masks is None:
            raise RuntimeError("Requested mask output is absent")
        shapes = result.masks.xy
    else:
        raise RuntimeError("A mask or OBB geometry source is required")
    labels, scores = boxes.cls.cpu().numpy(), boxes.conf.cpu().numpy()
    if len(labels) > maximum or len(shapes) != len(labels) or scores.shape != labels.shape:
        raise RuntimeError("Malformed geometry output count")
    objects = []
    for index, (shape, label, confidence) in enumerate(zip(shapes, labels, scores)):
        polygon = np.asarray(shape, np.float32)
        if (polygon.ndim != 2 or polygon.shape[1] != 2 or not np.isfinite(polygon).all()
                or (source == "obb" and len(polygon) != 4)
                or not math.isfinite(float(label)) or int(label) != label
                or int(label) not in names or not math.isfinite(float(confidence))
                or not 0 <= confidence <= 1):
            raise RuntimeError("Malformed native item geometry/class/confidence")
        if len(polygon) < 3 or abs(cv2.contourArea(polygon)) < 1:
            continue  # An empty/degenerate mask has no pick rectangle.
        rectangle = cv2.boxPoints(cv2.minAreaRect(polygon)) if source == "mask" else polygon
        objects.append({"index": index, "class_id": int(label), "class_name": names[int(label)],
                        "confidence": float(confidence), "polygon": polygon,
                        "rectangle": rectangle, "center": rectangle.mean(axis=0)})
    return objects


def plane_dimensions(rectangle, context, cv2, np, *, surface_z):
    """Measure a flat item on the floor-parallel plane through its observed center."""
    transform = np.asarray(context["platform_from_optical"], dtype=np.float64)
    flat = on_plane(rectangle, context["camera"], transform, cv2, np,
                    plane_z=surface_z)[:, :2]
    rect = cv2.boxPoints(cv2.minAreaRect(flat.astype(np.float32)))
    edges = np.roll(rect, -1, axis=0) - rect
    lengths = np.linalg.norm(edges, axis=1)
    length, width = float(lengths.max()), float(lengths.min())
    if not math.isfinite(length + width) or width <= 0:
        raise ValueError("Degenerate projected rectangle")
    return length, width, edges


def depth_sampling_circle(center, diameter_mm, context, cv2, np, *, output_camera=None):
    """One platform-plane circle for the RGB selection and actual depth sampling."""
    if not math.isfinite(diameter_mm) or diameter_mm <= 0:
        raise ValueError("pickdepth_radius must be a positive diameter in millimetres")
    transform = np.asarray(context["platform_from_optical"], dtype=np.float64)
    flat_center = on_plane([center], context["camera"], transform, cv2, np)[0]
    radius = diameter_mm / 2000
    angles = np.arange(96) * 2 * np.pi / 96
    circle = flat_center + np.column_stack((radius * np.cos(angles),
                                            radius * np.sin(angles), np.zeros(96)))
    camera = context["camera"] if output_camera is None else output_camera
    pixels = project(circle, camera, transform, cv2, np)
    if not np.isfinite(pixels).all() or np.any(np.abs(pixels) > 2_000_000_000):
        raise ValueError("Sampling circle projection exceeds safe drawing range")
    return flat_center, radius, pixels


def sample_item_depth(item, depth_mm, context, quality, diameter_mm, cv2, np, *, samples=None):
    """One unchanged pick ray and filtered depth sample for size and pick position."""
    if depth_mm is None or quality is None:
        raise ValueError("Height-corrected size requires valid synchronized depth")
    center = item["center"]
    if not inside(center, item["polygon"], cv2, np):
        raise ValueError("rectangle center is outside item mask")
    camera = context["camera"]
    depth_camera = context["depth_camera"]
    transform = np.asarray(context["platform_from_optical"], dtype=np.float64)
    flat_center, radius, circle_px = depth_sampling_circle(
        center, diameter_mm, context, cv2, np, output_camera=depth_camera)
    height_px, width_px = depth_mm.shape
    if (np.any(circle_px < 0) or np.any(circle_px[:, 0] >= width_px)
            or np.any(circle_px[:, 1] >= height_px)):
        raise ValueError("depth sampling circle is clipped by image edge")
    xmin, ymin = np.floor(circle_px.min(axis=0)).astype(int)
    xmax, ymax = np.ceil(circle_px.max(axis=0)).astype(int)
    xmax, ymax = min(xmax, width_px - 1), min(ymax, height_px - 1)
    yy, xx = np.mgrid[ymin:ymax + 1, xmin:xmax + 1]
    pixels = np.column_stack((xx.ravel(), yy.ravel()))
    flat_pixels = on_plane(pixels, depth_camera, transform, cv2, np)
    in_circle = np.linalg.norm(flat_pixels[:, :2] - flat_center[:2], axis=1) <= radius
    pixels = pixels[in_circle]
    values = depth_mm[pixels[:, 1], pixels[:, 0]].astype(np.float64)
    # Each original depth pixel is sampled once through its own camera model.
    rgb_pixels = reproject_pixels(pixels, depth_camera, camera, cv2, np)
    in_item = np.array([inside(p, item["polygon"], cv2, np) for p in rgb_pixels], dtype=bool)
    values[~in_item] = np.nan
    low, high = item_depth_limits(quality)
    accepted, median, sigma = filter_depth(values, low, high, cv2, np)
    if samples is not None:
        samples[item["index"]] = (pixels, accepted, circle_px)
    good, total = int(accepted.sum()), len(pixels)
    if not depth_coverage_ok(good, total, quality["minimum_depth_fraction"]):
        fraction = good / total if total else 0.
        raise ValueError(
            f"insufficient valid depth pixels: {good}/{total} ({fraction:.1%}); "
            f"requires {quality['minimum_depth_fraction']:.1%}")
    optical_point = rays([center], camera, cv2, np)[0] * median / 1000
    position = transform[:3, :3] @ optical_point + transform[:3, 3]
    return optical_point, position, median, sigma, good, total


def preview_detections(result, source, names, maximum, context, reason, cv2, np, *,
                       diameter_mm, depth_mm=None, quality=None):
    """Frame-local clickable geometry overlapping the green ROI when calibrated."""
    if source == "none":
        # Detection-only boxes are clickable but are not a production geometry source.
        boxes = result.obb if result.obb is not None else result.boxes
        if boxes is None:
            raise RuntimeError("Missing preview boxes")
        objects = []
        for index in range(len(boxes)):
            if result.obb is not None:
                rect = boxes.xyxyxyxy[index].cpu().numpy()
            else:
                x, y, right, bottom = boxes.xyxy[index].cpu().numpy()
                rect = np.array([[x, y], [right, y], [right, bottom], [x, bottom]])
            label = int(boxes.cls[index])
            objects.append({"index": index, "class_id": label, "class_name": names[label],
                            "confidence": float(boxes.conf[index]), "polygon": rect,
                            "rectangle": rect})
    else:
        objects = objects_from_result(result, source, names, maximum, cv2, np)
    detections = []
    for item in objects:
        if context is not None:
            try:
                footprint = on_plane(
                    item["polygon"], context["camera"],
                    np.asarray(context["platform_from_optical"], dtype=np.float64), cv2, np,
                )[:, :2]
            except ValueError:
                # Preserve a visible detection with an explicit measurement error
                # when its plane projection itself is unavailable. Production pose
                # generation will reject the same geometry.
                footprint = None
            if (footprint is not None
                    and not polygons_overlap_or_touch(footprint, context["roi"], cv2, np)):
                continue
        measurement, error = None, reason
        circle, circle_error = None, reason
        if source == "none":
            error = "Select a verified mask or OBB output for metric dimensions"
            circle_error = error
        elif context is not None:
            try:
                _, position, _, _, _, _ = sample_item_depth(
                    item, depth_mm, context, quality, diameter_mm, cv2, np)
                length, width, _ = plane_dimensions(item["rectangle"], context, cv2, np,
                                                    surface_z=float(position[2]))
                measurement = {"length_mm": length * 1000, "width_mm": width * 1000}
                error = ""
            except ValueError as exc:
                error = str(exc)
            try:
                center = np.asarray(item["rectangle"], dtype=np.float32).mean(axis=0)
                _, _, points = depth_sampling_circle(center, diameter_mm, context, cv2, np)
                circle, circle_error = points.tolist(), ""
            except ValueError as exc:
                circle_error = str(exc)
        detections.append({"source_index": item["index"], "class_id": item["class_id"],
                           "class_name": item["class_name"], "confidence": item["confidence"],
                           "polygon": item["polygon"].tolist(),
                           "rectangle": item["rectangle"].tolist(),
                           "measurement": measurement, "measurement_error": error,
                           "sampling_circle": circle, "sampling_circle_error": circle_error,
                           "depth_sampling_circle": None})
    return detections


def classify_size(measurement, geometry):
    """Neutral unknown state is distinct from a measured size rejection."""
    if measurement is None:
        return None, "Size unavailable: calibrated surface depth required"
    if geometry is None:
        return None, "Size not checked: enter length, width and tolerance"
    valid = (abs(measurement["length_mm"] - geometry["height"]) <= geometry["tolerance"]
             and abs(measurement["width_mm"] - geometry["width"]) <= geometry["tolerance"])
    return valid, "Size within tolerance" if valid else "Size outside tolerance"


def render_depth(depth_mm, quality, cv2, np):
    low, high = quality["depth_min_mm"], quality["depth_max_mm"]
    scaled = np.clip((np.nan_to_num(depth_mm.astype(np.float64), nan=low) - low)
                     / max(high - low, 1.), 0, 1)
    view = cv2.applyColorMap((scaled * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    view = cv2.cvtColor(view, cv2.COLOR_BGR2RGB)
    view[~np.isfinite(depth_mm) | (depth_mm < low) | (depth_mm > high)] = (90, 90, 90)
    return view


def draw_depth_geometry(view, detections, source, cameras, context, cv2, np,
                        *, bin_clearance=None):
    """Mirror RGB geometry onto original depth pixels, never copy RGB pixel indices."""
    color_camera, depth = cameras["camera"], cameras["depth_camera"]

    def mapped(points):
        return reproject_pixels(points, color_camera, depth, cv2, np)

    def segments(points, closed):
        points = np.asarray(points, dtype=np.float64)
        ends = np.roll(points, -1, axis=0) if closed else points[1:]
        starts = points if closed else points[:-1]
        t = np.arange(32, dtype=np.float64) / 32
        pixels = mapped(np.concatenate([a + (b-a)*t[:, None] for a, b in zip(starts, ends)]))
        if not closed:
            pixels = np.vstack((pixels, mapped([points[-1]])))
        return np.rint(pixels).astype(np.int32)

    if source == "mask":
        view[:] = shade_masks(view, [mapped(item["polygon"]) for item in detections], cv2, np)
    if context is not None:
        depth_context = {**context, "camera": depth}
        draw_bin_roi(view, depth_context, "", cv2, np)
        if bin_clearance is not None:
            draw_bin_clearance(view, depth_context, bin_clearance, cv2, np)
    if source == "none":
        return
    for item in detections:
        if item.get("sampling_circle") is not None:
            item["depth_sampling_circle"] = mapped(item["sampling_circle"]).tolist()
        valid = item["size_valid"]
        border = ((0, 255, 0) if valid is True else
                  (255, 0, 0) if valid is False else (180, 180, 180))
        cv2.polylines(view, [segments(item["rectangle"], True)], True, border, 2, cv2.LINE_AA)
        y_axis, x_axis, center = rectangle_axes(item["rectangle"], np)
        for axis, label, color in ((x_axis, "X", (255, 0, 0)), (y_axis, "Y", (0, 255, 0))):
            line = segments(axis, False)
            cv2.polylines(view, [line], False, color, 2, cv2.LINE_AA)
            cv2.putText(view, label, tuple(line[-1] + [4, -4]), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, color, 1, cv2.LINE_AA)
        pick = tuple(np.rint(mapped([center])[0]).astype(int))
        cv2.circle(view, pick, 5, (0, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(view, pick, 3, (255, 255, 255), -1, cv2.LINE_AA)


def selected_pose(item, rgb, depth, context, settings, cv2, np, *, display_detections=None):
    """Re-use exact displayed geometry, without prediction or moving the pick pixel."""
    rectangle = np.asarray(item["rectangle"], dtype=np.float32)
    rectangle_axes(rectangle, np)
    polygon = np.asarray(item["polygon"], dtype=np.float32)
    if (polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3
            or not np.isfinite(polygon).all()):
        raise RuntimeError("Malformed selected polygon")
    obj = {"index": item["source_index"], "class_id": item["class_id"],
           "class_name": item["class_name"], "confidence": item["confidence"],
           "polygon": polygon, "rectangle": rectangle, "center": rectangle.mean(axis=0)}
    return generate_candidates([obj], rgb, depth, context, settings, cv2, np,
                               display_detections=display_detections)


def generate_candidates(objects, rgb, depth_mm, context, settings, cv2, np,
                        *, display_detections=None, candidate_limit=None, nearby_views=None,
                        unchecked=None, render_images=True, prepared_scene=None, timings=None,
                        roi_status=None):
    started = time.monotonic()
    from .nearby_depth_overlay import draw_nearby_depth_overlays
    if candidate_limit is not None and (type(candidate_limit) is not int
                                        or not 1 <= candidate_limit <= 1000):
        raise RuntimeError("Invalid candidate acquisition limit")
    camera = context["camera"]
    depth_camera = context["depth_camera"]
    transform = np.asarray(context["platform_from_optical"], dtype=np.float64)
    roi = np.asarray(context["roi"], dtype=np.float64)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1])
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-6)
            or abs(np.linalg.det(transform[:3, :3]) - 1) > 1e-6):
        raise RuntimeError("Invalid platform-from-optical rigid transform")
    if roi.shape != (4, 2) or not np.isfinite(roi).all():
        raise RuntimeError("Invalid bin ROI")
    planning = context.get("pick_planning")
    if type(planning) is not dict or set(planning) != {
            "home_matrix", "base_from_platform", "link6_from_robot_camera",
            "pick_rotation_deg", "standoff_height_mm"}:
        raise SurfaceGeometryError("Robot-camera pick-planning context is missing or malformed")
    floor = HeightPlane(planning["base_from_platform"], "bin floor")
    clearance_roi = inset_bin_roi(context["roi"], settings["bin_clearance"])
    clearance_roi = (None if clearance_roi is None
                     else np.asarray(clearance_roi, dtype=np.float64))
    projected_pick_roi = project_bin_roi(
        {**context, "roi": roi if clearance_roi is None else clearance_roi}, cv2, np)
    quality, geometry = settings["quality"], settings["geometry"]
    low = item_depth_limits(quality)[0]
    center_roi = polygon_centroid(roi, np)
    labels, depth_objects = {}, []
    candidates, rejected = [], []
    samples = {}
    camera_plans = {}
    candidate_optical = {}
    nearby_checks = {}
    scene_depth, scene_error = None, None
    for item in objects:
        center = item["center"]
        label = f"#{item['index']} {item['class_name']} {item['confidence']:.2f}"
        rejection_stage = ""
        try:
            if item["class_id"] not in settings["yolo"]["class_ids"]:
                raise ValueError("class not selected")
            if item["confidence"] < settings["yolo"]["confidence"]:
                raise ValueError("confidence below threshold")
            if not inside(center, item["polygon"], cv2, np):
                raise ValueError("rectangle center is outside item mask")
            flat = on_plane(item["polygon"], camera, transform, cv2, np)[:, :2]
            if not polygons_overlap_or_touch(flat, roi, cv2, np):
                raise ValueError("item footprint fully outside bin ROI")
            if not inside(center, projected_pick_roi, cv2, np):
                raise ValueError(
                    "pick pixel outside projected bin ROI" if clearance_roi is None
                    else "pick pixel outside projected bin-wall clearance")
            optical_point, position, median, sigma, good, total = sample_item_depth(
                item, depth_mm, context, quality, geometry["pickdepth_radius"], cv2, np,
                samples=samples if render_images and nearby_views is None else None)
            label += f" depth {good}/{total}"
            # Measure only after valid depth determines the floor-parallel item plane.
            length, width, edges = plane_dimensions(
                item["rectangle"], context, cv2, np, surface_z=float(position[2]))
            depth_objects.append({**item, "size_valid": classify_size(
                {"length_mm": length*1000, "width_mm": width*1000}, geometry)[0]})
            axis_index = int(np.argmax(np.linalg.norm(edges, axis=1)))
            label += f" {length * 1000:.1f}x{width * 1000:.1f}mm"
            if (abs(length * 1000 - geometry["height"]) > geometry["tolerance"]
                    or abs(width * 1000 - geometry["width"]) > geometry["tolerance"]):
                raise ValueError("height-corrected size outside tolerance")
            if not inside(position[:2], roi, cv2, np):
                raise ValueError("depth-derived pick position outside bin ROI")
            if clearance_roi is not None and not inside(position[:2], clearance_roi, cv2, np):
                raise ValueError("pick point outside bin-wall clearance")
            axis = edges[axis_index] / length
            if axis[0] < 0 or (abs(axis[0]) < 1e-9 and axis[1] < 0):
                axis = -axis
            # Keep the same measured rectangle and pick point. New +X is the
            # previous +Y (short side); +Y is -previous +X, preserving handedness.
            yaw = math.atan2(float(axis[1]), float(axis[0])) + math.pi / 2
            quaternion = [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]
            item_in_base = candidate_pose_in_base(
                planning["base_from_platform"], position, quaternion)
            height = floor.measure(item_in_base[:3, 3])
            if height.below_plane:
                rejection_stage = "height"
                raise ValueError(height.reason)
            attitude = select_pick_attitude(
                planning["home_matrix"], item_in_base, planning["pick_rotation_deg"],
                planning["standoff_height_mm"], planning["base_from_platform"],
                planning["link6_from_robot_camera"], roi)
            camera_plans[item["index"]] = attitude
            if not attitude.accepted:
                raise ValueError(
                    "robot-camera body extends outside bin ROI for normal and 180-degree attitudes")
            candidate_optical[item["index"]] = optical_point
            candidates.append({
                "source_index": item["index"], "class_id": item["class_id"],
                "class_name": item["class_name"], "confidence": item["confidence"],
                "position": position.tolist(),
                "quaternion": quaternion,
                "length": length, "width": width,
                "center_distance": float(np.linalg.norm(position[:2] - center_roi)),
                "filtered_camera_depth": median / 1000, "depth_sigma": sigma / 1000,
                "accepted_depth_count": good, "rejected_depth_count": total - good,
                "pixel": center.tolist(),
                "planned_link6_matrix": attitude.planned_link6.tolist(),
                "robot_camera_clearance": {
                    "mirrored": attitude.mirrored,
                    "normal_platform_xy": list(attitude.normal_camera_platform_xy),
                    "mirrored_platform_xy": list(attitude.mirrored_camera_platform_xy),
                    "selected_platform_xy": list(attitude.selected_camera_platform_xy),
                    "body_reference_frame": CAMERA_BODY_REFERENCE_FRAME,
                    "body_size_color_optical_m": list(CAMERA_BODY_SIZE_RGB_M),
                    "body_center_color_optical_m": list(CAMERA_BODY_CENTER_RGB_M),
                    "normal_footprint_platform_xy": [list(p) for p in
                                                     attitude.normal_camera_footprint_xy],
                    "mirrored_footprint_platform_xy": [list(p) for p in
                                                       attitude.mirrored_camera_footprint_xy],
                    "selected_footprint_platform_xy": [list(p) for p in
                                                       attitude.selected_camera_footprint_xy],
                    "rotation_from_home_deg": attitude.rotation_from_home_deg,
                    "offset_direction": attitude.offset_direction,
                },
            })
            label += f" Z={position[2] * 1000:.1f}mm"
        except SurfaceGeometryError:
            raise
        except ValueError as exc:
            rejection = {"source_index": item["index"], "reason": str(exc)}
            if rejection_stage:
                rejection["rejection_stage"] = rejection_stage
            rejected.append(rejection)
            label += " | " + str(exc)
        labels[item["index"]] = label
    candidates.sort(key=lambda c: (c["center_distance"], -c["confidence"], c["source_index"]))
    geometry_ms = (time.monotonic() - started) * 1000.
    clearance_started = time.monotonic()
    ranked, candidates = candidates, []
    for index, candidate in enumerate(ranked):
        if candidate_limit is not None and len(candidates) == candidate_limit:
            if unchecked is not None:
                unchecked.extend(entry["source_index"] for entry in ranked[index:])
            break
        # Select in final rank order, then scan nearby depth only until the
        # requested batch is full. Later geometric candidates stay unchecked.
        source_id = candidate["source_index"]
        drawing = {} if render_images or nearby_views is not None else None
        try:
            if scene_depth is None and scene_error is None:
                try:
                    scene_depth = usable_scene_depth(depth_mm, context, quality, cv2, np,
                                                     prepared_scene=prepared_scene)
                except ValueError as exc:
                    scene_error = str(exc)
            if scene_error is not None:
                raise ValueError(scene_error)
            candidate["nearby_depth_filter"] = nearby_depth_check(
                scene_depth, candidate_optical[source_id], geometry, np, visualization=drawing)
        except ValueError as exc:
            rejection = {"source_index": source_id, "reason": str(exc)}
            if hasattr(exc, "evidence"):
                rejection["nearby_depth_filter"] = exc.evidence
                rejection["rejection_stage"] = "height"
            rejected.append(rejection)
            if drawing:
                nearby_checks[source_id] = drawing
            continue
        if drawing:
            nearby_checks[source_id] = drawing
        candidates.append(candidate)
    clearance_ms = (time.monotonic() - clearance_started) * 1000.
    render_started = time.monotonic()
    overlay = depth_view = None
    if render_images and nearby_views is None:
        chosen_ids = {c["source_index"] for c in candidates}
        shown = (objects if candidate_limit is None
                 else [o for o in objects if o["index"] in chosen_ids])
        overlay = (shade_masks(rgb, [o["polygon"] for o in shown], cv2, np)
                   if settings["geometry_source"] == "mask" else rgb.copy())
        depth_view = render_depth(depth_mm, {**quality, "depth_min_mm": low}, cv2, np)
        status = draw_bin_roi(overlay, context, "", cv2, np)
        if roi_status is not None:
            roi_status.update(status)
        draw_bin_clearance(overlay, context, settings["bin_clearance"], cv2, np)
        shown_depth = (display_detections
                       if display_detections is not None and candidate_limit is None
                       else [o for o in depth_objects
                             if candidate_limit is None or o["index"] in chosen_ids])
        draw_depth_geometry(depth_view, shown_depth, settings["geometry_source"], context,
                            context, cv2, np, bin_clearance=settings["bin_clearance"])
        by_id = {o["index"]: o for o in objects}
        for source_id, (pixels, accepted, boundary) in samples.items():
            if candidate_limit is not None and source_id not in chosen_ids:
                continue
            depth_view[pixels[:, 1], pixels[:, 0]] = (255, 0, 0)
            depth_view[pixels[accepted, 1], pixels[accepted, 0]] = (0, 0, 0)
            cv2.polylines(depth_view, [np.rint(boundary).astype(np.int32)], True, (0, 255, 255), 2)
        for source_id, attitude in camera_plans.items():
            if candidate_limit is not None and source_id not in chosen_ids:
                continue
            plans = ([(attitude.mirrored, attitude.selected_camera_footprint_xy)]
                     if attitude.accepted
                     else [(False, attitude.normal_camera_footprint_xy),
                           (True, attitude.mirrored_camera_footprint_xy)])
            for mirrored, footprint in plans:
                for image, model in ((overlay, camera), (depth_view, depth_camera)):
                    draw_robot_camera_footprint(image, {**context, "camera": model}, footprint,
                                                mirrored, attitude.accepted, cv2, np)
        for rank, candidate in enumerate(candidates, 1):
            item = by_id[candidate["source_index"]]
            draw_pick_geometry(overlay, item["rectangle"], cv2, np,
                               color=(0, 255, 0) if candidate_limit is not None else (255, 225, 0))
            _, _, circle = depth_sampling_circle(item["center"], geometry["pickdepth_radius"],
                                                 context, cv2, np)
            boundary = samples[candidate["source_index"]][2]
            depth_center = reproject_pixels([candidate["pixel"]], camera, depth_camera, cv2, np)[0]
            for image, ring, center in ((overlay, circle, candidate["pixel"]),
                                        (depth_view, boundary, depth_center)):
                cv2.polylines(image, [np.rint(ring).astype(np.int32)],
                              True, (0, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(image, f"P{rank}", tuple(np.rint(center).astype(int) + [8, 20]),
                            cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2, cv2.LINE_AA)
        if candidate_limit is None:
            for item in objects:
                center = item["center"]
                cv2.putText(overlay, labels[item["index"]],
                            (max(0, round(float(center[0]))-50),
                             max(22, round(float(center[1]))-15)),
                            cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
        draw_nearby_depth_overlays((overlay, depth_view), context, nearby_checks, cv2, np)
    elif nearby_views is not None:
        draw_nearby_depth_overlays(nearby_views, context, nearby_checks, cv2, np)
    if timings is not None:
        timings.update(geometry_ms=geometry_ms, clearance_ms=clearance_ms,
                       rendering_ms=(time.monotonic() - render_started)*1000.)
    return overlay, depth_view, candidates, rejected
