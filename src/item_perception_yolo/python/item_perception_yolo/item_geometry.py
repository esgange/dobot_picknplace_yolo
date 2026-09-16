"""Native-worker geometry. OpenCV/NumPy are passed in from the private runtime."""

import math

from .planar_bin_roi import border_in_optical
from .item_teach_core import inset_bin_roi


BIN_CLEARANCE_COLOR = (102, 204, 255)


def rectangle_axes(rectangle, np):
    """Full midpoint-to-midpoint pixel axes intersect at the immutable pick pixel."""
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
    x_axis, y_axis, center = rectangle_axes(rectangle, np)
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
    cv2.putText(overlay, "Loaded Bin ROI", (12, height - 15), cv2.FONT_HERSHEY_SIMPLEX,
                .65, (0, 255, 0), 2, cv2.LINE_AA)
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
    if visible:
        cv2.putText(overlay, "Pick clearance", (12, height - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, BIN_CLEARANCE_COLOR, 2, cv2.LINE_AA)
    return visible


def rays(pixels, camera, cv2, np):
    points = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    xy = cv2.undistortPoints(points, np.asarray(camera["k"]).reshape(3, 3),
                             np.asarray(camera["d"]))[:, 0, :]
    return np.column_stack((xy, np.ones(len(xy))))


def reproject_pixels(pixels, source_camera, target_camera, cv2, np):
    """Map rays between two pixel models of the SAME registered optical frame."""
    if len(pixels) == 0:
        return np.empty((0, 2), dtype=np.float64)
    directions = rays(pixels, source_camera, cv2, np)
    projected, _ = cv2.projectPoints(
        directions, np.zeros(3), np.zeros(3),
        np.asarray(target_camera["k"]).reshape(3, 3), np.asarray(target_camera["d"]))
    result = projected[:, 0, :]
    if not np.isfinite(result).all() or np.any(np.abs(result) > 2_000_000_000):
        raise ValueError("RGB/depth ray projection exceeds safe coordinate range")
    return result


def on_plane(pixels, camera, transform, cv2, np):
    direction = rays(pixels, camera, cv2, np) @ transform[:3, :3].T
    if np.any(np.abs(direction[:, 2]) < 1e-9):
        raise ValueError("Viewing ray is parallel to platform plane")
    distance = -transform[2, 3] / direction[:, 2]
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


def plane_dimensions(rectangle, context, cv2, np):
    """The one measurement definition shared by teaching and production filters."""
    transform = np.asarray(context["platform_from_optical"], dtype=np.float64)
    flat = on_plane(rectangle, context["camera"], transform, cv2, np)[:, :2]
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


def preview_detections(result, source, names, maximum, context, reason, cv2, np, *, diameter_mm):
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
                length, width, _ = plane_dimensions(item["rectangle"], context, cv2, np)
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
        return None, "Size unavailable: calibrated plane measurement required"
    if geometry is None:
        return None, "Size not checked: enter length, width and tolerance"
    valid = (abs(measurement["length_mm"] - geometry["height"]) <= geometry["tolerance"]
             and abs(measurement["width_mm"] - geometry["width"]) <= geometry["tolerance"])
    return valid, "Size within tolerance" if valid else "Size outside tolerance"


def render_depth(depth_mm, quality, cv2, np):
    low, high = quality["depth_min_mm"], quality["depth_max_mm"]
    scaled = np.clip((depth_mm.astype(np.float64) - low) / (high - low), 0, 1)
    view = cv2.applyColorMap((scaled * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    view = cv2.cvtColor(view, cv2.COLOR_BGR2RGB)
    view[(depth_mm < low) | (depth_mm > high)] = (90, 90, 90)
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
        x_axis, y_axis, center = rectangle_axes(item["rectangle"], np)
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
                        *, display_detections=None, candidate_limit=None):
    if candidate_limit is not None and (type(candidate_limit) is not int
                                        or not 1 <= candidate_limit <= 1000):
        raise RuntimeError("Invalid candidate overlay limit")
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
    clearance_roi = inset_bin_roi(context["roi"], settings["bin_clearance"])
    clearance_roi = (None if clearance_roi is None
                     else np.asarray(clearance_roi, dtype=np.float64))
    quality, geometry = settings["quality"], settings["geometry"]
    low, high = quality["depth_min_mm"], quality["depth_max_mm"]
    overlay = (shade_masks(rgb, [item["polygon"] for item in objects], cv2, np)
               if settings["geometry_source"] == "mask" else rgb.copy())
    depth_view = render_depth(depth_mm, quality, cv2, np)
    depth_objects = []
    for item in objects:
        try:
            length, width, _ = plane_dimensions(item["rectangle"], context, cv2, np)
            measured = {"length_mm": length*1000, "width_mm": width*1000}
        except ValueError:
            measured = None
        valid, _ = classify_size(measured, geometry)
        depth_objects.append({**item, "size_valid": valid})
    depth_objects = depth_objects if display_detections is None else display_detections
    draw_depth_geometry(depth_view, depth_objects, settings["geometry_source"], context,
                        context, cv2, np, bin_clearance=settings["bin_clearance"])
    center_roi = polygon_centroid(roi, np)
    draw_bin_roi(overlay, context, "", cv2, np)
    draw_bin_clearance(overlay, context, settings["bin_clearance"], cv2, np)
    candidates, rejected = [], []
    samples = {}
    for item in objects:
        center = item["center"]
        label = f"#{item['index']} {item['class_name']} {item['confidence']:.2f}"
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
            # Perspective can turn a pixel OBB into a quadrilateral. Fit the
            # metric enclosing rectangle on the agreed Z=0 plane, never at depth Z.
            length, width, edges = plane_dimensions(item["rectangle"], context, cv2, np)
            lengths = np.linalg.norm(edges, axis=1)
            axis_index = int(np.argmax(lengths))
            label += f" {length * 1000:.1f}x{width * 1000:.1f}mm"
            if (abs(length * 1000 - geometry["height"]) > geometry["tolerance"]
                    or abs(width * 1000 - geometry["width"]) > geometry["tolerance"]):
                raise ValueError("projected size outside tolerance")
            flat_center, radius, circle_px = depth_sampling_circle(
                center, geometry["pickdepth_radius"], context, cv2, np,
                output_camera=depth_camera)
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
            # Sample each original depth pixel once; map its ray into the RGB
            # mask. No depth resizing/interpolation or duplicated samples.
            rgb_pixels = reproject_pixels(pixels, depth_camera, camera, cv2, np)
            in_item = np.array([inside(p, item["polygon"], cv2, np) for p in rgb_pixels],
                               dtype=bool)
            values[~in_item] = np.nan
            accepted, median, sigma = filter_depth(values, low, high, cv2, np)
            depth_view[pixels[:, 1], pixels[:, 0]] = (255, 0, 0)
            depth_view[pixels[accepted, 1], pixels[accepted, 0]] = (0, 0, 0)
            cv2.polylines(depth_view, [np.rint(circle_px).astype(np.int32)], True,
                          (0, 255, 255), 2)
            good, total = int(accepted.sum()), len(pixels)
            label += f" depth {good}/{total}"
            if (good < quality["minimum_depth_samples"] or total == 0
                    or good / total < quality["minimum_depth_fraction"]):
                raise ValueError("insufficient accepted depth samples/fraction")
            optical_point = rays([center], camera, cv2, np)[0] * median / 1000
            position = transform[:3, :3] @ optical_point + transform[:3, 3]
            if not inside(position[:2], roi, cv2, np):
                raise ValueError("depth-derived pick position outside bin ROI")
            if clearance_roi is not None and not inside(position[:2], clearance_roi, cv2, np):
                raise ValueError("pick point outside bin-wall clearance")
            axis = edges[axis_index] / length
            if axis[0] < 0 or (abs(axis[0]) < 1e-9 and axis[1] < 0):
                axis = -axis
            yaw = math.atan2(float(axis[1]), float(axis[0]))
            draw_pick_geometry(overlay, item["rectangle"], cv2, np)
            candidates.append({
                "source_index": item["index"], "class_id": item["class_id"],
                "class_name": item["class_name"], "confidence": item["confidence"],
                "position": position.tolist(),
                "quaternion": [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)],
                "length": length, "width": width,
                "center_distance": float(np.linalg.norm(position[:2] - center_roi)),
                "filtered_camera_depth": median / 1000, "depth_sigma": sigma / 1000,
                "accepted_depth_count": good, "rejected_depth_count": total - good,
                "pixel": center.tolist(),
            })
            if candidate_limit is not None:
                samples[item["index"]] = (pixels, accepted, circle_px)
            label += f" Z={position[2] * 1000:.1f}mm"
        except ValueError as exc:
            rejected.append({"source_index": item["index"], "reason": str(exc)})
            label += " | " + str(exc)
        cv2.putText(overlay, label, (max(0, round(float(center[0])) - 50),
                                     max(22, round(float(center[1])) - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    candidates.sort(key=lambda c: (c["center_distance"], -c["confidence"], c["source_index"]))
    if candidate_limit is not None:
        # Render from the untouched pair AFTER ranking/capping. Excluded items
        # must leave no masks, boxes, sampling points, labels or pose axes behind.
        chosen = candidates[:candidate_limit]
        by_id = {item["index"]: item for item in objects}
        chosen_objects = [by_id[c["source_index"]] for c in chosen]
        overlay = (shade_masks(rgb, [o["polygon"] for o in chosen_objects], cv2, np)
                   if settings["geometry_source"] == "mask" else rgb.copy())
        depth_view = render_depth(depth_mm, quality, cv2, np)
        draw_bin_roi(overlay, context, "", cv2, np)
        draw_bin_clearance(overlay, context, settings["bin_clearance"], cv2, np)
        draw_depth_geometry(depth_view, [{**o, "size_valid": True} for o in chosen_objects],
                            settings["geometry_source"], context, context, cv2, np,
                            bin_clearance=settings["bin_clearance"])
        for rank, candidate in enumerate(chosen, 1):
            item = by_id[candidate["source_index"]]
            draw_pick_geometry(overlay, item["rectangle"], cv2, np, color=(0, 255, 0))
            _, _, circle = depth_sampling_circle(
                item["center"], geometry["pickdepth_radius"], context, cv2, np)
            pixels, accepted, depth_circle = samples[candidate["source_index"]]
            depth_view[pixels[:, 1], pixels[:, 0]] = (255, 0, 0)
            depth_view[pixels[accepted, 1], pixels[accepted, 0]] = (0, 0, 0)
            depth_center = reproject_pixels([candidate["pixel"]], camera, depth_camera, cv2, np)[0]
            for image, boundary, center in ((overlay, circle, candidate["pixel"]),
                                            (depth_view, depth_circle, depth_center)):
                cv2.polylines(image, [np.rint(boundary).astype(np.int32)], True,
                              (0, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(image, f"P{rank}", tuple(np.rint(center).astype(int) + [8, 20]),
                            cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2, cv2.LINE_AA)
        # All candidates/diagnostics still describe the uncapped valid set.
        # The shared response builder selects the identical leading subset.
        return overlay, depth_view, candidates, rejected
    for rank, candidate in enumerate(candidates, 1):
        cv2.putText(overlay, f"P{rank}", tuple(np.rint(candidate["pixel"]).astype(int) + [8, 20]),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        depth_label = (f"P{rank}: Z={candidate['position'][2]*1000:.1f}mm "
                       f"camera depth={candidate['filtered_camera_depth']*1000:.1f}mm "
                       f"{candidate['accepted_depth_count']} OK / "
                       f"{candidate['rejected_depth_count']} rejected")
        cv2.putText(depth_view, depth_label, (10, 65 + rank * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(depth_view, "Accepted BLACK | Rejected RED | MAD 3 sigma", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return overlay, depth_view, candidates, rejected
