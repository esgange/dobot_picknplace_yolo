"""Tray-relative item alignment with the recorded observation tool tilt preserved."""

import math

import numpy as np

from .errors import FeedbackFailure
from .motion import candidate_pose_in_base, rigid_matrix


def held_item_axis(session, configuration):
    """Recover the item's short X in tool coordinates from its measured pickup."""
    if (session is None or session.configuration is not configuration
            or session.batch is None or session.held_index is None):
        raise FeedbackFailure("Item alignment requires a retained controller pickup")
    index = session.held_index - 1
    attempt = session.attempts[index]
    if attempt.state != "HELD" or attempt.pickup_matrix is None:
        raise FeedbackFailure("Item alignment requires a confirmed pickup orientation")
    candidate = session.batch.candidates[index]
    item = candidate_pose_in_base(
        configuration.selection.station.platform.base_from_platform,
        candidate.position_m, candidate.quaternion)
    pickup = rigid_matrix(attempt.pickup_matrix, "Measured pickup pose")
    return pickup[:3, :3].T @ item[:3, 0]


def aligned_rotation(detect_matrix, tray_rotation, item_x_in_tool, offset_deg,
                     *, current_matrix=None):
    """Choose the nearest of two offset-aligned attitudes, keeping taught tilt."""
    detect = rigid_matrix(detect_matrix, "Recorded Tray Detect Pose")
    current = (detect if current_matrix is None
               else rigid_matrix(current_matrix, "Current placement Link6 pose"))
    tray = np.eye(4)
    tray[:3, :3] = tray_rotation
    tray = rigid_matrix(tray, "Observed tray orientation")
    axis = np.asarray(item_x_in_tool, dtype=float)
    if (axis.shape != (3,) or not np.isfinite(axis).all()
            or not math.isclose(float(np.linalg.norm(axis)), 1., abs_tol=1e-6)):
        raise ValueError("Held item X must be a finite unit tool-frame vector")
    # Both directions are projected into the plane perpendicular to unchanged
    # taught tool Z. Never copy tray Z: its nearest-corner convention can flip it.
    target = detect[:3, :3].T @ tray[:3, 0]
    if min(float(np.linalg.norm(axis[:2])), float(np.linalg.norm(target[:2]))) <= 1e-6:
        raise ValueError("Item/tray X cannot be aligned around the taught tool Z")
    delta = math.atan2(target[1], target[0]) - math.atan2(axis[1], axis[0])
    delta += math.radians(offset_deg)
    c, s = math.cos(delta), math.sin(delta)
    aligned = detect[:3, :3] @ np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    reverse = aligned @ np.diag([-1., -1., 1.])
    # For rigid rotations, larger relative trace means smaller angular travel.
    # Compare after applying the offset. Keep the directed solution on a tie
    # instead of allowing roundoff to alternate between the two valid attitudes.
    reference = current[:3, :3].T
    if np.trace(reference @ reverse) > np.trace(reference @ aligned) + 1e-12:
        return reverse
    return aligned
