"""Tray geometry executed only in the pinned private YOLO/OpenCV worker."""

from camera_calibration_gui.calibration_core import rotation_matrix_to_quaternion
from item_perception_yolo.item_geometry import (
    objects_from_result, on_plane, project, rays, reproject_pixels, filter_depth)
from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS
from item_perception_yolo.yolo_worker_native import render_result

from .core import MAX_PLANE_ERROR_MM, validate_plane, validate_settings


def corner_frame(corners, normal, np):
    """Nearest base-origin corner; both planar axes point along edges into the tray."""
    corners = np.asarray(corners, dtype=float)
    origin = min(range(4), key=lambda i: (float(corners[i] @ corners[i]), *corners[i]))
    following, previous = (origin + 1) % 4, (origin - 1) % 4
    x = corners[following] - corners[origin]
    y = corners[previous] - corners[origin]
    if np.dot(np.cross(x, y), normal) < 0:
        following, previous = previous, following
        x, y = y, x
    x = x / np.linalg.norm(x)
    y = np.cross(normal, x)
    y /= np.linalg.norm(y)
    matrix = np.eye(4)
    matrix[:3, :3] = np.column_stack((x, y, np.cross(x, y)))
    matrix[:3, 3] = corners[origin]
    return matrix, [origin, following, (origin + 2) % 4, previous]


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
            if int(accepted.sum()) < 30:
                raise ValueError(f"Corner {index}: fewer than 30 valid local depth samples")
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
    cv2.polylines(overlay, [np.rint(image_points).astype(np.int32)], True, (0, 210, 255), 2)


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
    validate_settings(settings)
    transform = np.asarray(plane["base_from_plane"])
    optical = np.asarray(context["base_from_optical"])
    plane_from_optical = np.linalg.inv(transform) @ optical
    detections, valid = [], []
    expected = settings["geometry"]
    for item in objects:
        detection = {"source_index": item["index"], "class_id": item["class_id"],
                     "class_name": item["class_name"], "confidence": item["confidence"],
                     "polygon": item["polygon"].tolist(), "valid": False, "reason": ""}
        detections.append(detection)
        try:
            if item["class_id"] not in settings["yolo"]["class_ids"]:
                raise ValueError("Unselected tray class")
            if item["confidence"] < settings["yolo"]["confidence"]:
                raise ValueError("Below confidence threshold")
            polygon = item["polygon"]
            if (np.any(polygon[:, 0] <= 0) or np.any(polygon[:, 0] >= width - 1)
                    or np.any(polygon[:, 1] <= 0) or np.any(polygon[:, 1] >= height - 1)):
                raise ValueError("Tray touches image edge; full dimensions unavailable")
            flat = on_plane(polygon, context["camera"], plane_from_optical, cv2, np)
            rectangle = cv2.boxPoints(cv2.minAreaRect(flat[:, :2].astype(np.float32)))
            lengths = np.linalg.norm(np.roll(rectangle, -1, axis=0) - rectangle, axis=1) * 1000
            length, short = float(lengths.max()), float(lengths.min())
            detection.update(length_mm=length, width_mm=short)
            if (short <= 0 or abs(length - expected["length_mm"]) > expected["tolerance_mm"]
                    or abs(short - expected["width_mm"]) > expected["tolerance_mm"]):
                raise ValueError("Tray dimensions outside tolerance")
            base = np.column_stack((rectangle, np.zeros(4))) @ transform[:3, :3].T + \
                transform[:3, 3]
            frame, order = corner_frame(base, transform[:3, 2], np)
            rectangle_pixels = project(base, context["camera"], optical, cv2, np)
            center = project([base.mean(axis=0)], context["camera"], optical, cv2, np)[0]
            distance = float(np.linalg.norm(center - [width / 2, height / 2]))
            detection.update(valid=True, reason="Dimensions within tolerance",
                             position=frame[:3, 3].tolist(),
                             quaternion=list(rotation_matrix_to_quaternion(frame[:3, :3])),
                             rectangle=rectangle_pixels[order].tolist(),
                             corners_base_m=base[order].tolist(), center_distance_px=distance)
            valid.append(detection)
        except ValueError as exc:
            detection["reason"] = str(exc)
    valid.sort(key=lambda item: (
        item["center_distance_px"], -item["confidence"], item["source_index"]))
    return detections, valid[0] if valid else None


def predict_trays(request, result, rgb, names, cv2, np):
    settings, plane, context = request["settings"], request["plane"], request["camera_context"]
    validate_settings(settings)
    overlay, count = render_result(result, rgb, names, settings["model_task"],
                                   settings["yolo"]["max_detections"], cv2, np)
    detections, selected = [], None
    reason = "Teach the reference plane before measuring trays"
    if settings["geometry_source"] == "none":
        reason = "Box-only model: load a segmentation or OBB model for tray poses"
    else:
        objects = objects_from_result(result, settings["geometry_source"], names,
                                      settings["yolo"]["max_detections"], cv2, np)
        if plane is not None:
            detections, selected = evaluate_objects(
                objects, settings, plane, context, rgb.shape[1], rgb.shape[0], cv2, np)
            reason = "One valid tray selected" if selected is not None else "No valid tray"
            try:
                draw_plane(overlay, plane, context, cv2, np)
            except ValueError as exc:
                reason += f" | Reference plane outline unavailable: {exc}"
            for detection in detections:
                polygon = np.rint(detection["polygon"]).astype(np.int32)
                cv2.polylines(overlay, [polygon], True,
                              (0, 220, 0) if detection["valid"] else (255, 50, 50), 2)
            if selected is not None:
                rectangle = np.rint(selected["rectangle"]).astype(np.int32)
                cv2.circle(overlay, tuple(rectangle[0]), 8, (0, 255, 255), 3)
                for index, color in ((1, (255, 0, 0)), (3, (0, 255, 0))):
                    cv2.arrowedLine(overlay, tuple(rectangle[0]), tuple(rectangle[index]),
                                    color, 3, tipLength=.08)
    return {"state": "ok", "width": rgb.shape[1], "height": rgb.shape[0], "count": count,
            "detections": detections, "selected": selected, "reason": reason}, overlay.tobytes()
