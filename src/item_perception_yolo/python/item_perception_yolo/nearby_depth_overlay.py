"""Read-only drawing of the exact nearby-depth checks, inside the native worker."""

from .floor_clearance import floor_depth


RADIUS_COLOR = (255, 210, 0)
HEIGHT_COLOR = (255, 140, 0)
OBSTACLE_COLOR = (255, 0, 0)


def projected_base_points(points, base_from_optical, camera, cv2, np):
    """Keep invalid/behind-camera projections as NaN so drawing can skip them."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    optical = (points - base_from_optical[:3, 3]) @ base_from_optical[:3, :3]
    valid = np.isfinite(optical).all(axis=1) & (optical[:, 2] > 1e-6)
    result = np.full((len(points), 2), np.nan)
    if valid.any():
        pixels, _ = cv2.projectPoints(
            optical[valid], np.zeros(3), np.zeros(3),
            np.asarray(camera["k"]).reshape(3, 3), np.asarray(camera["d"]))
        result[valid] = pixels[:, 0, :]
    result[~np.isfinite(result).all(axis=1) | (np.abs(result) > 1e8).any(axis=1)] = np.nan
    return result


def _lines(view, segments, color, cv2, np):
    clipped = []
    for first, second in segments:
        if not np.isfinite([first, second]).all():
            continue
        start, end = (tuple(np.rint(point).astype(int)) for point in (first, second))
        visible, start, end = cv2.clipLine((0, 0, view.shape[1], view.shape[0]), start, end)
        if visible:
            clipped.append(np.asarray([start, end], dtype=np.int32))
    if clipped:
        # Complete the outline first; a later segment must not erase the
        # previous segment's color and make a solid circle appear dashed.
        cv2.polylines(view, clipped, False, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.polylines(view, clipped, False, color, 1, cv2.LINE_AA)


def _text(view, message, origin, color, cv2):
    height, width = view.shape[:2]
    scale = min(.43, max(.2, (width - 12) / max(1, len(message) * 10)))
    (text_width, text_height), baseline = cv2.getTextSize(
        message, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x = max(2, min(int(origin[0]), width - text_width - 3))
    y = max(text_height + 2, min(int(origin[1]), height - baseline - 2))
    cv2.rectangle(view, (x-1, y-text_height-1), (x+text_width+1, y+baseline+1), (0, 0, 0), -1)
    cv2.putText(view, message, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def draw_nearby_depth_overlays(views, context, checks, cv2, np):
    """Draw camera-XY rings on equal floor-height surfaces; never change eligibility."""
    if not checks:
        return
    base_from_optical = np.eye(4)  # Checks already carry camera optical coordinates.
    angles = np.linspace(0., 2*np.pi, 97)
    circle = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(len(angles))))
    # Paint blockers last so other candidates' rings cannot hide an offending pixel.
    for view, camera in zip(views, (context["camera"], context["depth_camera"])):
        annotations = []
        obstacles = []
        maxima = []
        for source_id, check in checks.items():
            evidence, center = check["evidence"], check["item_xyz"]
            radius, height = evidence["radius_mm"], evidence["height_mm"]
            surface = center + circle * radius / 1000.
            surface[:, 2] = (floor_depth(surface, check["plane"], np)
                             - evidence["candidate_height_mm"] / 1000.)
            limit = surface - [0., 0., height / 1000.]
            lower = projected_base_points(surface, base_from_optical, camera, cv2, np)
            upper = projected_base_points(limit, base_from_optical, camera, cv2, np)
            _lines(view, zip(lower[:-1], lower[1:]), RADIUS_COLOR, cv2, np)
            _lines(view, ((upper[i], upper[i+1]) for i in range(len(circle)-1) if i % 4 < 2),
                   HEIGHT_COLOR, cv2, np)
            _lines(view, ((lower[i], upper[i]) for i in (0, 24, 48, 72)),
                   HEIGHT_COLOR, cv2, np)
            obstacles.append(projected_base_points(
                check["blocking_points"], base_from_optical, camera, cv2, np))
            maximum = evidence["maximum_height_difference_mm"]
            maximum_text = "no depth" if maximum is None else f"max {maximum:.1f}mm"
            label = f"#{source_id} NEAR {'BLOCKED' if check['blocked'] else 'OK'} | {maximum_text}"
            anchor = projected_base_points([center], base_from_optical, camera, cv2, np)[0]
            if np.isfinite(anchor).all():
                annotations.append((label, anchor + [-40, 43],
                                    OBSTACLE_COLOR if check["blocked"] else RADIUS_COLOR))
            if check["maximum_point"] is not None:
                point = projected_base_points(
                    [check["maximum_point"]], base_from_optical, camera, cv2, np)[0]
                maxima.append((point, source_id, maximum, check["blocked"]))
        for pixels in obstacles:
            visible = (np.isfinite(pixels).all(axis=1)
                       & (pixels[:, 0] >= 0) & (pixels[:, 0] < view.shape[1] - .5)
                       & (pixels[:, 1] >= 0) & (pixels[:, 1] < view.shape[0] - .5))
            pixels = np.rint(pixels[visible]).astype(int)
            view[pixels[:, 1], pixels[:, 0]] = OBSTACLE_COLOR
        for label, anchor, color in annotations:
            _text(view, label, anchor, color, cv2)
        for point, source_id, maximum, blocked in maxima:
            if not (np.isfinite(point).all() and 0 <= point[0] < view.shape[1]
                    and 0 <= point[1] < view.shape[0]):
                continue
            # Only blocked maxima get a cross; safe flat scenes have arbitrary
            # equal maxima and must not suggest one pixel is a special obstacle.
            if blocked:
                pixel = tuple(np.rint(point).astype(int))
                cv2.drawMarker(view, pixel, (255, 255, 255), cv2.MARKER_TILTED_CROSS, 13, 3)
                cv2.drawMarker(view, pixel, OBSTACLE_COLOR, cv2.MARKER_TILTED_CROSS, 11, 1)
                _text(view, f"#{source_id} +{maximum:.1f}mm", point + [9, -10],
                      OBSTACLE_COLOR, cv2)
        evidence = next(iter(checks.values()))["evidence"]
        _text(view, f"NEAR: solid R={evidence['radius_mm']:g}mm | "
              f"dashed H={evidence['height_mm']:g}mm (camera XY / floor height)",
              (5, view.shape[0]-23), RADIUS_COLOR, cv2)
        _text(view, "Floor-relative difference | red/X=obstacle | cyan=depth sample",
              (5, view.shape[0]-7), (255, 255, 255), cv2)
