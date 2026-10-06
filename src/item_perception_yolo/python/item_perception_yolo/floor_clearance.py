"""Native floor-relative clearance. Coordinates and depth are never voxelized."""

import math

from .depth_snapshot import item_depth_limits
from .projection_cache import cached_rays
from .pick_planning import rigid_matrix


REFERENCE = "platform_floor_camera_z_v1"


class ClearanceBlocked(ValueError):
    """A rejected check retains measured evidence without requiring images."""

    def __init__(self, message, evidence):
        super().__init__(message)
        self.evidence = evidence


def floor_plane(context, np):
    transform = rigid_matrix(context["platform_from_optical"], "platform-from-optical")
    plane = np.asarray(transform[2], dtype=np.float64)
    if abs(plane[2]) < 1e-9:
        raise ValueError("Floor plane is parallel to the camera depth axis")
    return plane


def floor_depth(points, plane, np):
    points = np.asarray(points, dtype=np.float64)
    depth = -(points[..., 0] * plane[0] + points[..., 1] * plane[1] + plane[3]) / plane[2]
    if not np.isfinite(depth).all() or np.any(depth <= 0):
        raise ValueError("Floor depth is non-finite or behind the camera")
    return depth


def inside_outer_bin(points, roi, np):
    """Inclusive convex physical footprint; accepts either polygon winding."""
    roi = np.asarray(roi, dtype=np.float64)
    if roi.shape != (4, 2) or not np.isfinite(roi).all():
        raise ValueError("A finite four-corner outer bin boundary is required")
    edges = np.roll(roi, -1, axis=0) - roi
    turns = edges[:, 0] * np.roll(edges[:, 1], -1) - edges[:, 1] * np.roll(edges[:, 0], -1)
    if not (np.all(turns > 1e-12) or np.all(turns < -1e-12)):
        raise ValueError("Outer bin boundary must be nondegenerate and convex")
    # Evaluate four edges in separate vectors: avoid an N×4×2 temporary scene.
    # The convex winding is already validated, so one sign is sufficient.
    points = np.asarray(points)
    keep = np.ones(len(points), dtype=bool)
    positive = turns[0] > 0
    for origin, edge in zip(roi, edges):
        cross = edge[0] * (points[:, 1] - origin[1]) - edge[1] * (points[:, 0] - origin[0])
        keep &= cross >= -1e-10 if positive else cross <= 1e-10
    return keep


def depth_scene(depth_mm, context, quality, cv2, np, *, item_minimum=True):
    """One full-resolution back projection, shared by clearance and teaching voxels."""
    low, high = (item_depth_limits(quality) if item_minimum else
                 (quality["depth_min_mm"], quality["depth_max_mm"]))
    yy, xx = np.nonzero(np.isfinite(depth_mm) & (depth_mm >= low) & (depth_mm <= high))
    pixels = np.column_stack((xx, yy))
    optical = (cached_rays(pixels, context["depth_camera"], cv2, np)
               * depth_mm[yy, xx, None] / 1000.) if len(pixels) else np.empty((0, 3))
    transform = rigid_matrix(context["platform_from_optical"], "platform-from-optical")
    platform = optical @ transform[:3, :3].T + transform[:3, 3]
    return {"pixels": pixels, "optical": optical, "platform": platform}


def usable_scene_depth(depth_mm, context, quality, cv2, np, *, prepared_scene=None):
    scene = (depth_scene(depth_mm, context, quality, cv2, np)
             if prepared_scene is None else prepared_scene)
    plane = floor_plane(context, np)
    keep = inside_outer_bin(scene["platform"], context["roi"], np)
    points = scene["optical"][keep]
    heights = (floor_depth(points, plane, np) - points[:, 2]) * 1000.
    return {"pixels": scene["pixels"][keep], "points": points,
            "heights_mm": heights, "plane": plane}


def nearby_depth_check(scene, item_xyz, geometry, np, *, visualization=None):
    """Maximum floor-relative height within inclusive camera-XY distance."""
    pixels, points = scene["pixels"], scene["points"]
    item_xyz = np.asarray(item_xyz, dtype=np.float64)
    if item_xyz.shape != (3,) or not np.isfinite(item_xyz).all() or item_xyz[2] < .5:
        raise ValueError("Candidate camera depth must be finite and at least 500 mm")
    candidate_height = float((floor_depth(item_xyz, scene["plane"], np) - item_xyz[2]) * 1000.)
    radius_mm, height_mm = geometry["nearby_depth_radius_mm"], geometry["nearby_depth_height_mm"]
    delta = points[:, :2] - item_xyz[:2]
    distances = np.einsum("ij,ij->i", delta, delta)
    nearby = np.flatnonzero(distances <= ((radius_mm + 1e-6) / 1000.)**2)
    if not len(nearby):
        raise ValueError("No usable nearby depth inside the outer bin boundary")
    index = nearby[np.argmax(scene["heights_mm"][nearby])]
    maximum = float(scene["heights_mm"][index])
    difference = maximum - candidate_height
    evidence = {"reference": REFERENCE, "radius_mm": radius_mm, "height_mm": height_mm,
                "usable_point_count": len(points), "nearby_point_count": len(nearby),
                "candidate_height_mm": candidate_height, "maximum_nearby_height_mm": maximum,
                "maximum_height_difference_mm": difference}
    blocked = difference >= height_mm - 1e-6
    if visualization is not None:
        blockers = nearby[scene["heights_mm"][nearby] - candidate_height >= height_mm - 1e-6]
        visualization.update(evidence=evidence, item_xyz=item_xyz.copy(), plane=scene["plane"],
                             blocking_points=points[blockers], blocked=blocked,
                             maximum_point=points[index])
    if blocked:
        x, y = pixels[index]
        raise ClearanceBlocked(
            f"nearby depth point ({x}, {y}): {difference:.2f} mm above candidate floor height "
            f"at {math.sqrt(distances[index])*1000:.2f} mm radius; "
            f"limits {height_mm:g} mm / {radius_mm:g} mm (camera XY/floor-relative Z)", evidence)
    return evidence
