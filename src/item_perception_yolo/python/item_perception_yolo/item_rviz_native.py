"""Teaching visualization geometry, called only inside the private native worker."""

from .item_geometry import generate_candidates, rays, reproject_pixels, rectangle_axes
from .item_teach_core import validate_detection_settings, validate_quality


VOXEL_METRES = 0.010


def colored_voxels(rgb, depth, context, base_from_platform, quality, cv2, np):
    """Centroid/color-average occupied 10 mm cells; never alter pose sampling inputs."""
    valid = ((depth > 0) & (depth >= quality["depth_min_mm"])
             & (depth <= quality["depth_max_mm"]))
    yy, xx = np.nonzero(valid)
    dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    if not len(xx):
        return np.empty(0, dtype=dtype)
    pixels = np.column_stack((xx, yy))
    optical = rays(pixels, context["depth_camera"], cv2, np) * depth[yy, xx, None] / 1000.
    transform = np.asarray(context["platform_from_optical"], dtype=float)
    platform = optical @ transform[:3, :3].T + transform[:3, 3]
    color_pixels = np.rint(reproject_pixels(
        pixels, context["depth_camera"], context["camera"], cv2, np)).astype(np.int64)
    height, width = depth.shape
    keep = ((color_pixels[:, 0] >= 0) & (color_pixels[:, 0] < width)
            & (color_pixels[:, 1] >= 0) & (color_pixels[:, 1] < height))
    color_pixels, platform = color_pixels[keep], platform[keep]
    if not len(platform):
        return np.empty(0, dtype=dtype)
    base = np.asarray(base_from_platform, dtype=float)
    points = platform @ base[:3, :3].T + base[:3, 3]
    colors = rgb[color_pixels[:, 1], color_pixels[:, 0]].astype(float)
    cells, inverse = np.unique(np.floor(points / VOXEL_METRES).astype(np.int64),
                               axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    output = np.empty(len(cells), dtype=dtype)
    for axis, field in enumerate(("x", "y", "z")):
        output[field] = np.bincount(inverse, weights=points[:, axis]) / counts
    averaged = [np.rint(np.bincount(inverse, weights=colors[:, axis]) / counts).astype(np.uint32)
                for axis in range(3)]
    output["rgb"] = (averaged[0] << 16) | (averaged[1] << 8) | averaged[2]
    return output


def teaching_rviz(request, data, cv2, np):
    """Use the displayed detections and exact RGB/depth snapshot without another YOLO call."""
    width, height = request["width"], request["height"]
    overlay_requested = request["nearby_overlay"]
    if (type(width) is not int or type(height) is not int
            or type(overlay_requested) is not bool
            or not 0 < width <= 4096 or not 0 < height <= 4096
            or len(data) != width * height * (11 if overlay_requested else 5)
            or (overlay_requested and request["settings"] is None)):
        raise RuntimeError("Malformed teaching RViz RGB/depth snapshot")
    quality = request["quality"]
    validate_quality(quality)
    rgb_bytes = width * height * 3
    rgb = np.frombuffer(data[:rgb_bytes], np.uint8).reshape(height, width, 3)
    source_end = width * height * 5
    depth = np.frombuffer(data[rgb_bytes:source_end], "<u2").reshape(height, width)
    views = None
    if overlay_requested:
        views = tuple(np.frombuffer(pixels, np.uint8).reshape(height, width, 3).copy()
                      for pixels in (data[source_end:source_end+rgb_bytes],
                                     data[source_end+rgb_bytes:]))
    cloud = colored_voxels(rgb, depth, request["context"], request["base_from_platform"],
                           quality, cv2, np)
    settings = request["settings"]
    candidates, rejected, unchecked = [], [], []
    if settings is not None:
        validate_detection_settings(settings, geometry_required=True)
        limit = request["candidate_limit"]
        if type(limit) is not int or not 1 <= limit <= settings["yolo"]["max_detections"]:
            raise RuntimeError("Invalid teaching candidate acquisition count")
        objects = []
        for item in request["detections"]:
            rectangle = np.asarray(item["rectangle"], dtype=np.float32)
            rectangle_axes(rectangle, np)
            polygon = np.asarray(item["polygon"], dtype=np.float32)
            if (polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3
                    or not np.isfinite(polygon).all()):
                raise RuntimeError("Malformed teaching RViz polygon")
            objects.append({"index": item["source_index"], "class_id": item["class_id"],
                            "class_name": item["class_name"], "confidence": item["confidence"],
                            "polygon": polygon, "rectangle": rectangle,
                            "center": rectangle.mean(axis=0)})
        # Match acquisition: stop nearby scans once the taught batch is full.
        _, _, candidates, rejected = generate_candidates(
            objects, rgb, depth, request["context"], settings, cv2, np, nearby_views=views,
            candidate_limit=limit, unchecked=unchecked)
    result = {"state": "ok", "generation": request["generation"],
              "nearby_overlay": overlay_requested,
              "point_count": len(cloud), "candidates": candidates, "rejected": rejected,
              "unchecked": unchecked}
    return result, cloud.tobytes() + (b"" if views is None else b"".join(
        view.tobytes() for view in views))
