"""Tray geometry executed only in the pinned private YOLO/OpenCV worker."""

from camera_calibration_gui.calibration_core import rotation_matrix_to_quaternion
from item_perception_yolo.item_geometry import (
    objects_from_result, on_plane, project, rays, reproject_pixels, filter_depth, rectangle_axes)
from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
from item_perception_yolo.item_rviz_native import colored_voxels
from item_perception_yolo.yolo_worker_native import render_result

from .core import MAX_PLANE_ERROR_MM, validate_plane, validate_preview


def corner_frame(corners, normal, np, *, short_x=False):
    """Nearest base-origin corner; both planar axes point along edges into the tray."""
    corners = np.asarray(corners, dtype=float)
    origin = min(range(4), key=lambda i: (float(corners[i] @ corners[i]), *corners[i]))
    following, previous = (origin + 1) % 4, (origin - 1) % 4
    if short_x:
        # A detected rectangle has orthogonal adjacent edges. Fix their physical
        # lengths before choosing Z; camera-facing Z cannot always satisfy this.
        following, previous = sorted((following, previous), key=lambda i: (
            float(np.linalg.norm(corners[i] - corners[origin])), *corners[i]))
    x = corners[following] - corners[origin]
    y = corners[previous] - corners[origin]
    if short_x:
        normal = np.cross(x, y)
        normal /= np.linalg.norm(normal)
    elif np.dot(np.cross(x, y), normal) < 0:
        following, previous = previous, following
        x, y = y, x
    x = x / np.linalg.norm(x)
    y = np.cross(normal, x)
    y /= np.linalg.norm(y)
    matrix = np.eye(4)
    matrix[:3, :3] = np.column_stack((x, y, np.cross(x, y)))
    matrix[:3, 3] = corners[origin]
    return matrix, [origin, following, (origin + 2) % 4, previous]


def draw_tray_image_axes(overlay, polygon, cv2, np):
    """Image-only orientation, never a metric tray frame or a base-frame origin."""
    rectangle = cv2.boxPoints(cv2.minAreaRect(np.asarray(polygon, np.float32)))
    try:
        long_axis, short_axis, _ = rectangle_axes(rectangle, np)
    except ValueError:
        return
    cv2.polylines(overlay, [np.rint(rectangle).astype(np.int32)], True,
                  (160, 160, 160), 2, cv2.LINE_AA)
    for axis, label, color in ((short_axis, "X 2D", (255, 0, 0)),
                               (long_axis, "Y 2D", (0, 255, 0))):
        start, end = np.rint(axis).astype(int)
        cv2.line(overlay, tuple(start), tuple(end), (0, 0, 0), 4, cv2.LINE_AA)
        cv2.line(overlay, tuple(start), tuple(end), color, 2, cv2.LINE_AA)
        cv2.putText(overlay, label, tuple(end + [4, -4]), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, label, tuple(end + [4, -4]), cv2.FONT_HERSHEY_SIMPLEX,
                    .5, color, 1, cv2.LINE_AA)


def draw_tray_axes(overlay, detection, cv2, np, *, depth=False, preview_polygon=None):
    """Measured corner axes are independent of production size/class eligibility."""
    key = "depth_rectangle" if depth else "rectangle"
    if key not in detection:
        polygon = preview_polygon if depth else detection["polygon"]
        if polygon is not None:
            draw_tray_image_axes(overlay, polygon, cv2, np)
        return
    rectangle = np.rint(detection[key]).astype(np.int32)
    origin = tuple(rectangle[0])
    cv2.polylines(overlay, [rectangle], True, (0, 210, 255), 1, cv2.LINE_AA)
    for index, label, dimension, color in (
            (1, "X", "width_mm", (255, 0, 0)), (3, "Y", "length_mm", (0, 255, 0))):
        end = tuple(rectangle[index])
        cv2.arrowedLine(overlay, origin, end, color, 3, cv2.LINE_AA, tipLength=.08)
        text = f"{label} {detection[dimension]:.1f} mm"
        (width, height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, .5, 1)
        x = int(np.clip(end[0] + 5, 0, max(0, overlay.shape[1] - width - 1)))
        y = int(np.clip(end[1] - 5, height + 1, overlay.shape[0] - baseline - 1))
        cv2.putText(overlay, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .5,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
    cv2.circle(overlay, origin, 5, (0, 0, 0), -1, cv2.LINE_AA)
    cv2.circle(overlay, origin, 3, (0, 255, 255), -1, cv2.LINE_AA)


def fit_plane(points, pixels, base_from_optical, cv2, np):
    points = np.asarray(points, dtype=float)
    center = points.mean(axis=0)
    _, singular, axes = np.linalg.svd(points - center)
    if singular[1] < .001:
        raise ValueError("Four corners must span a plane, not a line")
    normal = axes[-1]
    if normal @ (base_from_optical[:3, 3] - center) < 0:
        normal = -normal
    distances = (points - center) @ normal
    error = float(np.abs(distances).max() * 1000)
    if error > MAX_PLANE_ERROR_MM:
        raise ValueError(f"Corners are not coplanar: {error:.2f} mm error (limit 5 mm)")
    flat = points - distances[:, None] * normal
    coordinates = ((flat - center) @ axes[:2].T).astype(np.float32)
    order = cv2.convexHull(coordinates, returnPoints=False).reshape(-1).tolist()
    if len(order) != 4 or abs(cv2.contourArea(coordinates[order])) < 1e-6:
        raise ValueError("Choose four distinct outside corners of a convex tray")
    matrix, ordered = corner_frame(flat[order], normal, np)
    order = [order[index] for index in ordered]
    return {"corners_base_m": points[order].tolist(), "pixels": np.asarray(pixels)[order].tolist(),
            "base_from_plane": matrix.tolist(), "max_error_mm": error}


def capture_plane(request, data, cv2, np):
    width, height = request["width"], request["height"]
    if len(data) != width * height * 2:
        raise RuntimeError("Malformed tray depth snapshot")
    depth = np.frombuffer(data, "<u2").reshape(height, width)
    context = request["camera_context"]
    camera, depth_camera = context["camera"], context["depth_camera"]
    pixels = np.asarray(request["pixels"], float)
    if pixels.shape != (4, 2) or not np.isfinite(pixels).all():
        raise RuntimeError("Exactly four finite clicked points are required")
    try:
        points = []
        mapped = reproject_pixels(pixels, camera, depth_camera, cv2, np)
        ray_vectors = rays(pixels, camera, cv2, np)
        transform = np.asarray(context["base_from_optical"])
        for index, (pixel, ray) in enumerate(zip(mapped, ray_vectors), 1):
            x, y = np.rint(pixel).astype(int)
            if not (3 <= x < width - 3 and 3 <= y < height - 3):
                raise ValueError(f"Corner {index}: depth sampling patch falls outside image")
            values = depth[y - 3:y + 4, x - 3:x + 4].reshape(-1)
            accepted, median, _ = filter_depth(
                values, QUALITY_DEFAULTS["depth_min_mm"],
                QUALITY_DEFAULTS["depth_max_mm"], cv2, np)
            if not accepted.any():
                raise ValueError(f"Corner {index}: no valid local depth samples")
            optical = ray * (median / 1000)
            points.append(transform[:3, :3] @ optical + transform[:3, 3])
        plane = fit_plane(points, pixels, transform, cv2, np)
        plane.update(source_stamp_ns=request["source_stamp_ns"],
                     depth_stamp_ns=request["depth_stamp_ns"], camera=camera,
                     depth_camera=depth_camera)
        validate_plane(plane)
        return {"state": "ok", "plane": plane, "error": ""}, b""
    except ValueError as exc:
        return {"state": "ok", "plane": None, "error": str(exc)}, b""


def draw_plane(overlay, plane, context, cv2, np):
    optical = np.asarray(context["base_from_optical"])
    corners = np.asarray(plane["corners_base_m"])
    transform = np.asarray(plane["base_from_plane"])
    distances = (corners - transform[:3, 3]) @ transform[:3, 2]
    corners = corners - distances[:, None] * transform[:3, 2]
    # Project densely sampled edges so lens distortion remains visible.
    border = np.concatenate([np.linspace(corners[i], corners[(i + 1) % 4], 32)
                             for i in range(4)])
    image_points = project(border, context["camera"], optical, cv2, np)
    corner_pixels = project(corners, context["camera"], optical, cv2, np)
    # Forward-facing points can still explode near the camera plane, especially
    # with distortion. Validate both projections before casting or painting:
    # OpenCV reports out-of-int32 circle centers as a misleading "wrong type".
    for pixels in (image_points, corner_pixels):
        if not np.isfinite(pixels).all() or np.any(np.abs(pixels) > 2_000_000_000):
            raise ValueError("Reference plane projection exceeds safe drawing range")
    image_height, image_width = overlay.shape[:2]
    border_pixels = np.rint(image_points).astype(int)
    visible = False
    for start, end in zip(border_pixels, np.roll(border_pixels, -1, axis=0)):
        intersects, clipped_start, clipped_end = cv2.clipLine(
            (0, 0, image_width, image_height), tuple(start), tuple(end))
        if intersects:
            cv2.line(overlay, clipped_start, clipped_end, (0, 255, 0), 5, cv2.LINE_AA)
            visible = True
    if not visible:
        raise ValueError("Reference plane outline is outside the image")
    center = corner_pixels.mean(axis=0)
    for index, point in enumerate(corner_pixels, 1):
        point = tuple(np.rint(point).astype(int))
        if not (0 <= point[0] < image_width and 0 <= point[1] < image_height):
            continue
        cv2.circle(overlay, point, 6, (0, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(overlay, point, 4, (0, 255, 0), -1, cv2.LINE_AA)
        label = f"P{index}"
        (width, height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .5, 1)
        at = (point[0] + 12 if center[0] >= point[0] else point[0] - width - 12,
              point[1] + height + 8 if center[1] >= point[1] else point[1] - 8)
        cv2.putText(overlay, label, at, cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, label, at, cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1, cv2.LINE_AA)


def overlay_plane(request, data, cv2, np):
    width, height = request["width"], request["height"]
    if len(data) != width * height * 3:
        raise RuntimeError("Malformed tray RGB snapshot")
    overlay = np.frombuffer(data, np.uint8).reshape(height, width, 3).copy()
    error = ""
    if request["plane"] is not None:
        try:
            draw_plane(overlay, request["plane"], request["camera_context"], cv2, np)
        except ValueError as exc:
            error = str(exc)
    return {"state": "ok", "width": width, "height": height, "error": error}, overlay.tobytes()


def evaluate_objects(objects, settings, plane, context, width, height, cv2, np):
    """Depth-free measurement and exactly one winner; also used by teaching previews."""
    validate_preview(settings)
    if plane is not None and context is not None:
        transform = np.asarray(plane["base_from_plane"])
        optical = np.asarray(context["base_from_optical"])
        plane_from_optical = np.linalg.inv(transform) @ optical
    detections, valid = [], []
    expected = settings["geometry"]
    for item in objects:
        detection = {"source_index": item["index"], "class_id": item["class_id"],
                     "class_name": item["class_name"], "confidence": item["confidence"],
                     "polygon": item["polygon"].tolist(), "valid": False, "reason": "",
                     "size_status": "unchecked"}
        detections.append(detection)
        try:
            if context is None:
                raise ValueError("Measurement needs matching camera calibration and RGB-time TF")
            if plane is None:
                raise ValueError("Create the reference plane from four corners to measure trays")
            polygon = item["polygon"]
            if (np.any(polygon[:, 0] <= 0) or np.any(polygon[:, 0] >= width - 1)
                    or np.any(polygon[:, 1] <= 0) or np.any(polygon[:, 1] >= height - 1)):
                raise ValueError("Tray touches image edge; full dimensions unavailable")
            flat = on_plane(polygon, context["camera"], plane_from_optical, cv2, np)
            rectangle = cv2.boxPoints(cv2.minAreaRect(flat[:, :2].astype(np.float32)))
            lengths = np.linalg.norm(np.roll(rectangle, -1, axis=0) - rectangle, axis=1) * 1000
            length, short = float(lengths.max()), float(lengths.min())
            detection.update(length_mm=length, width_mm=short)
            if short <= 0:
                raise ValueError("Degenerate tray rectangle")
            base = np.column_stack((rectangle, np.zeros(4))) @ transform[:3, :3].T + \
                transform[:3, 3]
            frame, order = corner_frame(base, transform[:3, 2], np, short_x=True)
            rectangle_pixels = project(base, context["camera"], optical, cv2, np)
            center = project([base.mean(axis=0)], context["camera"], optical, cv2, np)[0]
            distance = float(np.linalg.norm(center - [width / 2, height / 2]))
            detection.update(position=frame[:3, 3].tolist(),
                             quaternion=list(rotation_matrix_to_quaternion(frame[:3, :3])),
                             rectangle=rectangle_pixels[order].tolist(),
                             corners_base_m=base[order].tolist(), center_distance_px=distance)
            if context.get("depth_camera") is not None:
                detection["depth_rectangle"] = project(
                    base[order], context["depth_camera"], optical, cv2, np).tolist()
            if expected is None:
                raise ValueError("Measured; size filter inactive until length, width and "
                                 "tolerance are entered")
            if (abs(length - expected["length_mm"]) > expected["tolerance_mm"]
                    or abs(short - expected["width_mm"]) > expected["tolerance_mm"]):
                detection["size_status"] = "fail"
                raise ValueError("Tray dimensions outside tolerance")
            detection["size_status"] = "pass"
            if item["class_id"] not in settings["accepted_class_ids"]:
                raise ValueError("Unselected tray class")
            if item["confidence"] < settings["yolo"]["confidence"]:
                raise ValueError("Below confidence threshold")
            detection.update(valid=True, reason="Dimensions within tolerance")
            valid.append(detection)
        except ValueError as exc:
            detection["reason"] = str(exc)
    valid.sort(key=lambda item: (
        item["center_distance_px"], -item["confidence"], item["source_index"]))
    return detections, valid[0] if valid else None


def predict_trays(request, result, rgb, names, cv2, np):
    settings, plane, context = request["settings"], request["plane"], request["camera_context"]
    validate_preview(settings)
    overlay, count = render_result(result, rgb, names, settings["model_task"],
                                   settings["yolo"]["max_detections"], cv2, np)
    detections, selected = [], None
    reason = "Teach the reference plane before measuring trays"
    if settings["geometry_source"] == "none":
        reason = "Box-only model: load a segmentation or OBB model for tray poses"
    else:
        objects = objects_from_result(result, settings["geometry_source"], names,
                                      settings["yolo"]["max_detections"], cv2, np)
        detections, selected = evaluate_objects(
            objects, settings, plane, context, rgb.shape[1], rgb.shape[0], cv2, np)
        measured = sum("rectangle" in item for item in detections)
        if selected is not None:
            reason = "One valid tray selected"
        elif context is None:
            reason = "Measurement needs matching camera calibration and RGB-time TF"
        elif plane is None:
            reason = "Create the reference plane from four corners to measure trays"
        elif measured:
            reason = f"{measured} tray(s) measured — click a tray to read X/width and Y/length"
            reason += (" | Size filter inactive" if settings["geometry"] is None else
                       " | No accepted tray pose")
        else:
            reason = "No eligible tray"
        if plane is not None and context is not None:
            try:
                draw_plane(overlay, plane, context, cv2, np)
            except ValueError as exc:
                reason += f" | Reference plane outline unavailable: {exc}"
        for detection in detections:
            polygon = np.rint(detection["polygon"]).astype(np.int32)
            color = {"pass": (0, 220, 0), "fail": (255, 50, 50),
                     "unchecked": (160, 160, 160)}[detection["size_status"]]
            cv2.polylines(overlay, [polygon], True, color, 2)
            draw_tray_axes(overlay, detection, cv2, np)
    return {"state": "ok", "width": rgb.shape[1], "height": rgb.shape[0], "count": count,
            "detections": detections, "selected": selected, "reason": reason}, overlay.tobytes()


def tray_visuals(request, data, cv2, np):
    """Depth evidence and cloud from the original observation, never a second inference."""
    width, height = request["width"], request["height"]
    if len(data) != width * height * 5:
        raise RuntimeError("Malformed tray RGB/depth visualization snapshot")
    rgb = np.frombuffer(data[:width * height * 3], np.uint8).reshape(height, width, 3)
    rgb_overlay = rgb.copy()
    depth = np.frombuffer(data[width * height * 3:], "<u2").reshape(height, width)
    low, high = QUALITY_DEFAULTS["depth_min_mm"], QUALITY_DEFAULTS["depth_max_mm"]
    scaled = np.clip((depth.astype(float) - low) * 255 / (high - low), 0, 255).astype(np.uint8)
    overlay = cv2.cvtColor(cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB)
    overlay[(depth < low) | (depth > high)] = 0
    context, samples = request["camera_context"], []
    if context is not None:
        color_info, depth_info = context["camera"], context["depth_camera"]
        if request.get("plane") is not None:
            try:
                draw_plane(overlay, request["plane"], {**context, "camera": depth_info}, cv2, np)
            except ValueError:
                pass  # An off-camera taught plane cannot suppress the available depth scene.
        for detection in request["detections"]:
            pixels = reproject_pixels(np.asarray(detection["polygon"]),
                                      color_info, depth_info, cv2, np)
            color = {"pass": (0, 220, 0), "fail": (255, 50, 50),
                     "unchecked": (160, 160, 160)}[detection["size_status"]]
            cv2.polylines(overlay, [np.rint(pixels).astype(np.int32)], True, color, 2)
            draw_tray_axes(overlay, detection, cv2, np, depth=True, preview_polygon=pixels)
        for index, pixel in enumerate(request["pixels"], 1):
            mapped = reproject_pixels(np.asarray([pixel]), color_info, depth_info, cv2, np)[0]
            x, y = np.rint(mapped).astype(int)
            sample = {"index": index, "depth_pixel": [int(x), int(y)], "accepted": 0,
                      "median_mm": None, "reason": "Depth sampling patch falls outside image"}
            if 3 <= x < width - 3 and 3 <= y < height - 3:
                values = depth[y - 3:y + 4, x - 3:x + 4].reshape(-1)
                accepted, median, _ = filter_depth(values, low, high, cv2, np)
                sample.update(accepted=int(accepted.sum()),
                              median_mm=median,
                              reason="OK" if accepted.any() else "No valid depth samples")
                patch = overlay[y - 3:y + 4, x - 3:x + 4]
                patch[:] = np.where(accepted.reshape(7, 7, 1), (0, 0, 0), (255, 0, 0))
                yy, xx = np.mgrid[y - 3:y + 4, x - 3:x + 4]
                rgb_samples = reproject_pixels(
                    np.column_stack((xx.ravel(), yy.ravel())), depth_info, color_info, cv2, np)
                for point, valid in zip(np.rint(rgb_samples).astype(int), accepted):
                    cv2.circle(rgb_overlay, tuple(point), 1,
                               (0, 0, 0) if valid else (255, 0, 0), -1)
            cv2.circle(overlay, (int(x), int(y)), 8, (0, 255, 255), 2)
            cv2.putText(overlay, str(index), (int(x + 10), int(y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
            samples.append(sample)
    cloud = b""
    if request["cloud"]:
        # Base is the intermediate frame; no platform artifact is involved.
        cloud_context = {**context, "platform_from_optical": context["base_from_optical"]}
        cloud = colored_voxels(rgb, depth, cloud_context, np.eye(4),
                               QUALITY_DEFAULTS, cv2, np).tobytes()
    return {"state": "ok", "width": width, "height": height,
            "point_count": len(cloud) // 16, "samples": samples}, \
        overlay.tobytes() + rgb_overlay.tobytes() + cloud
