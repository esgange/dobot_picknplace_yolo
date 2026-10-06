"""Pure CR10 pick-attitude and robot-camera clearance planning.

This module deliberately has no ROS, Qt, OpenCV, or hardware-command imports so
Item Detect, controller preview, and hardware execution use one calculation.
"""

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from camera_calibration_gui.calibration_core import quaternion_to_rotation_matrix

from .item_teach_core import JOINT_NAMES, file_sha256


# Gemini 335 nominal mechanical model, Orbbec datasheet V1.6 §§3.2.3, 4.8–4.9
# and Appendix B; see the package README for sources and the derivation.
# RGB optical axes: X right/width, Y down/height, Z forward/depth.
CAMERA_BODY_REFERENCE_FRAME = "robot_camera_color_optical_frame"
CAMERA_BODY_SIZE_RGB_M = (0.090, 0.025, 0.030)
CAMERA_BODY_CENTER_RGB_M = (0.011, 0.0, -0.01279)


def camera_link_from_rgb_optical():
    """Nominal mechanical RGB origin in the saved depth-origin camera_link.

    This bridge positions the housing only. It neither replaces factory optical
    TF nor supplies the transforms used for RGB/depth measurement/calibration.
    """
    return np.array([[0., 0., 1., .00202],
                     [-1., 0., 0., -.014],
                     [0., -1., 0., 0.],
                     [0., 0., 0., 1.]])


def camera_body_pose_from_rgb(rgb_optical_transform):
    """Body-center pose, with optical axes, from a destination <- RGB pose."""
    pose = rigid_matrix(rgb_optical_transform, "RGB optical body reference").copy()
    pose[:3, 3] += pose[:3, :3] @ np.asarray(CAMERA_BODY_CENTER_RGB_M)
    return pose


def camera_body_pose(camera_link_transform):
    """Position the RGB-referenced housing through the existing camera-link mount."""
    return camera_body_pose_from_rgb(
        rigid_matrix(camera_link_transform, "Camera link transform")
        @ camera_link_from_rgb_optical())


def camera_body_corners(camera_transform):
    """Eight housing corners in the destination frame, including mounting tilt."""
    transform = camera_body_pose(camera_transform)
    half = np.asarray(CAMERA_BODY_SIZE_RGB_M) / 2
    corners = np.array([[x, y, z] for x in (-half[0], half[0])
                        for y in (-half[1], half[1]) for z in (-half[2], half[2])])
    return corners @ transform[:3, :3].T + transform[:3, 3]


def camera_body_footprint(camera_transform):
    """Convex projected outline on destination XY, in boundary order."""
    points = sorted(set(map(tuple, camera_body_corners(camera_transform)[:, :2])))

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def chain(values):
        result = []
        for point in values:
            while len(result) >= 2 and cross(result[-2], result[-1], point) <= 0:
                result.pop()
            result.append(point)
        return result

    return tuple(chain(points)[:-1] + chain(reversed(points))[:-1])


def camera_body_fits(footprint, bin_roi):
    """Full containment in the validated convex Bin Teach quadrilateral."""
    polygon = np.asarray(bin_roi, dtype=float)
    if polygon.shape != (4, 2) or not np.isfinite(polygon).all():
        raise ValueError("Camera-body clearance requires a finite convex bin quadrilateral")
    edges = np.roll(polygon, -1, axis=0) - polygon
    following = np.roll(edges, -1, axis=0)
    turns = edges[:, 0] * following[:, 1] - edges[:, 1] * following[:, 0]
    if not (np.all(turns > 0) or np.all(turns < 0)):
        raise ValueError("Camera-body clearance requires a convex bin quadrilateral")
    return all(point_in_polygon(point, polygon) for point in footprint)


def rigid_matrix(value, label):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.all(np.isfinite(matrix))
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3),
                               atol=1e-6, rtol=0)
            or not math.isclose(float(np.linalg.det(matrix[:3, :3])), 1., abs_tol=1e-6)):
        raise ValueError(f"{label} must be a finite rigid transform")
    return matrix


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


@dataclass(frozen=True)
class Joint:
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class Cr10Kinematics:
    """Read-only FK for the one canonical CR10 base_link <- Link6 chain."""

    def __init__(self, path):
        self.path = Path(path).resolve()
        self.sha256 = file_sha256(self.path)
        robot = ET.parse(self.path).getroot()
        if robot.attrib.get("name") != "cr10_robot":
            raise ValueError("Only the canonical CR10 model is supported")
        self.joints = []
        parent = "base_link"
        for index, name in enumerate(JOINT_NAMES, 1):
            matches = robot.findall(f"joint[@name='{name}']")
            if len(matches) != 1:
                raise ValueError(f"CR10 model requires exactly one {name}")
            node = matches[0]
            if (node.attrib["type"] != "revolute"
                    or node.find("parent").attrib["link"] != parent
                    or node.find("child").attrib["link"] != f"Link{index}"):
                raise ValueError("CR10 model has an unsupported base_link <- Link6 chain")
            origin = np.eye(4)
            spec = node.find("origin").attrib
            origin[:3, 3] = [float(v) for v in spec["xyz"].split()]
            origin[:3, :3] = rpy_matrix(*map(float, spec["rpy"].split()))
            axis = np.array([float(v) for v in node.find("axis").attrib["xyz"].split()])
            if not np.allclose(axis, [0, 0, 1], atol=1e-12):
                raise ValueError("CR10 model requires each local revolute axis to be Z")
            limit = node.find("limit").attrib
            self.joints.append(Joint(origin, axis, float(limit["lower"]),
                                     float(limit["upper"])))
            parent = f"Link{index}"

    def forward(self, positions):
        if len(positions) != 6:
            raise ValueError("CR10 kinematics requires six Home/actual joints in radians")
        result = np.eye(4)
        for spec, angle in zip(self.joints, positions):
            if (type(angle) not in (float, int) or not math.isfinite(angle)
                    or not spec.lower <= angle <= spec.upper):
                raise ValueError("Joint is invalid or outside the canonical CR10 model limits")
            rotation = np.eye(4)
            rotation[:3, :3] = rpy_matrix(0, 0, angle)
            result = result @ spec.origin @ rotation
        return result


def candidate_pose_in_base(base_from_platform, position_m, quaternion):
    """Compose the detector's raw platform XYZ/yaw without treating it as tool attitude."""
    platform = rigid_matrix(base_from_platform, "Platform transform")
    position = np.asarray(position_m, dtype=float)
    q = np.asarray(quaternion, dtype=float)
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("Candidate position must contain three finite metres")
    if (q.shape != (4,) or not np.all(np.isfinite(q))
            or abs(float(np.dot(q, q)) - 1.0) > 1e-5):
        raise ValueError("Candidate quaternion must be finite and normalized")
    if abs(float(q[0])) > 1e-6 or abs(float(q[1])) > 1e-6:
        raise ValueError("Candidate quaternion must contain platform-plane yaw only")
    item = np.eye(4)
    item[:3, :3] = quaternion_to_rotation_matrix(*q)
    item[:3, 3] = position
    return rigid_matrix(platform @ item, "Base-relative candidate pose")


def _spin_about_axis(vector, axis, angle):
    return (vector * math.cos(angle) + np.cross(axis, vector) * math.sin(angle)
            + axis * float(np.dot(axis, vector)) * (1.0 - math.cos(angle)))


def pick_attitude(home, item_in_base, pick_rotation_deg=0.0):
    """Align tool Y to item short X with the nearest legal +/-offset from Home."""
    home = rigid_matrix(home, "Home")
    item = rigid_matrix(item_in_base, "Base-relative candidate pose")
    if (type(pick_rotation_deg) not in (int, float)
            or not math.isfinite(pick_rotation_deg)
            or not 0.0 <= pick_rotation_deg <= 90.0):
        raise ValueError("pick_rotation must be a finite number from 0 to 90 degrees")
    home_rotation = home[:3, :3]
    tool_z = home_rotation[:, 2]
    item_short = item[:3, 0]
    projected = item_short - tool_z * float(np.dot(item_short, tool_z))
    norm = float(np.linalg.norm(projected))
    if norm <= 1e-6:
        raise ValueError("Item short axis cannot be projected perpendicular to tool Z")
    projected /= norm
    reference_green = home_rotation[:, 1]
    offset = math.radians(pick_rotation_deg)
    choices = (("none", projected),) if offset == 0.0 else (
        ("ccw", _spin_about_axis(projected, tool_z, offset)),
        ("cw", _spin_about_axis(projected, tool_z, -offset)),
    )
    candidates = []
    for priority, (direction, desired_line) in enumerate(choices):
        sine = float(np.dot(tool_z, np.cross(reference_green, desired_line)))
        cosine = float(np.dot(reference_green, desired_line))
        raw = math.atan2(sine, cosine)
        # Both directions along the desired line are physically equivalent.
        delta = (raw + math.pi / 2) % math.pi - math.pi / 2
        target_green = _spin_about_axis(reference_green, tool_z, delta)
        target_green /= np.linalg.norm(target_green)
        target_red = np.cross(target_green, tool_z)
        target_red /= np.linalg.norm(target_red)
        rotation = np.column_stack((target_red, target_green, tool_z))
        if abs(abs(float(np.dot(target_green, desired_line))) - 1.0) > 1e-6:
            raise ValueError("Failed to construct offset item pick attitude")
        candidates.append((abs(delta), priority, rotation, math.degrees(delta), direction))
    # Equivalent frame labels can differ by roundoff. Preserve the explicit
    # CCW-before-CW preference for travel ties rather than magnifying that noise.
    minimum_travel = min(value[0] for value in candidates)
    _, _, rotation, travel_deg, direction = min(
        (value for value in candidates if value[0] <= minimum_travel + 1e-12),
        key=lambda value: value[1])
    return rotation, travel_deg, direction


def point_in_polygon(point, polygon):
    """Boundary-inclusive finite point-in-polygon test without OpenCV."""
    point = np.asarray(point, dtype=float)
    polygon = np.asarray(polygon, dtype=float)
    if (point.shape != (2,) or polygon.ndim != 2 or polygon.shape[1] != 2
            or len(polygon) < 3 or not np.all(np.isfinite(point))
            or not np.all(np.isfinite(polygon))):
        raise ValueError("Robot-camera clearance requires finite 2-D point/polygon geometry")
    scale = max(1.0, float(np.max(np.abs(np.vstack((polygon, point))))))
    epsilon = 1e-10 * scale
    inside = False
    x, y = point
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        edge, offset = end - start, point - start
        cross = float(edge[0] * offset[1] - edge[1] * offset[0])
        if (abs(cross) <= epsilon
                and np.all(point >= np.minimum(start, end) - epsilon)
                and np.all(point <= np.maximum(start, end) + epsilon)):
            return True
        if ((start[1] > y) != (end[1] > y)):
            crossing_x = start[0] + (y - start[1]) * (end[0] - start[0]) / (
                end[1] - start[1])
            if crossing_x >= x - epsilon:
                inside = not inside
    return inside


@dataclass(frozen=True)
class PickAttitudeSelection:
    accepted: bool
    rotation: np.ndarray | None
    rotation_from_home_deg: float | None
    offset_direction: str
    mirrored: bool | None
    planned_link6: np.ndarray | None
    normal_camera_platform_xy: tuple
    mirrored_camera_platform_xy: tuple
    selected_camera_platform_xy: tuple | None
    normal_camera_footprint_xy: tuple
    mirrored_camera_footprint_xy: tuple
    selected_camera_footprint_xy: tuple | None


def select_pick_attitude(home, item_in_base, pick_rotation_deg, standoff_height_mm,
                         base_from_platform, link6_from_robot_camera, bin_roi):
    """Select normal or exact tool-Z 180-degree mirror by full camera-body clearance."""
    home = rigid_matrix(home, "Home")
    item = rigid_matrix(item_in_base, "Base-relative candidate pose")
    base_from_platform = rigid_matrix(base_from_platform, "Platform transform")
    link6_from_camera = rigid_matrix(
        link6_from_robot_camera, "Link6 from robot_camera_link calibration")
    if (type(standoff_height_mm) not in (int, float)
            or not math.isfinite(standoff_height_mm)):
        raise ValueError("standoff_height must be finite millimetres")
    normal_rotation, normal_degrees, direction = pick_attitude(
        home, item, pick_rotation_deg)
    platform_from_base = np.linalg.inv(base_from_platform)

    def planned(rotation):
        link6 = np.eye(4)
        link6[:3, :3] = rotation
        link6[:3, 3] = item[:3, 3]
        link6[2, 3] += float(standoff_height_mm) / 1000.0
        camera_in_platform = platform_from_base @ link6 @ link6_from_camera
        point = tuple(map(float, camera_in_platform[:2, 3]))
        return link6, point, camera_body_footprint(camera_in_platform)

    normal_link6, normal_point, normal_footprint = planned(normal_rotation)
    # Post-multiplication is exactly 180 degrees around unchanged local tool Z.
    mirror_rotation = normal_rotation @ np.diag([-1.0, -1.0, 1.0])
    mirror_link6, mirror_point, mirror_footprint = planned(mirror_rotation)
    if camera_body_fits(normal_footprint, bin_roi):
        return PickAttitudeSelection(
            True, normal_rotation, normal_degrees, direction, False, normal_link6,
            normal_point, mirror_point, normal_point,
            normal_footprint, mirror_footprint, normal_footprint)
    if camera_body_fits(mirror_footprint, bin_roi):
        mirror_degrees = (normal_degrees + 180.0 + 180.0) % 360.0 - 180.0
        return PickAttitudeSelection(
            True, mirror_rotation, mirror_degrees, direction, True, mirror_link6,
            normal_point, mirror_point, mirror_point,
            normal_footprint, mirror_footprint, mirror_footprint)
    return PickAttitudeSelection(
        False, None, None, direction, None, None,
        normal_point, mirror_point, None, normal_footprint, mirror_footprint, None)
