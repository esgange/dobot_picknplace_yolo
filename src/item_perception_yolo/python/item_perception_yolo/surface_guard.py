"""Shared measured-surface height guard; no motion or tool-offset calculation."""

from dataclasses import dataclass

import numpy as np

from .pick_planning import rigid_matrix


SURFACE_TOLERANCE_M = 1e-6  # 0.001 mm: arithmetic tolerance, not physical clearance.


class SurfaceGeometryError(ValueError):
    """An invalid reference or measurement must not become an empty observation."""


@dataclass(frozen=True)
class SurfaceHeight:
    reference: str
    surface_z_m: float
    plane_z_m: float

    @property
    def below_plane(self):
        return self.surface_z_m < self.plane_z_m - SURFACE_TOLERANCE_M

    @property
    def reason(self):
        return (f"Measured surface below {self.reference}: "
                f"surface Z={self.surface_z_m * 1000:.3f} mm, "
                f"plane Z={self.plane_z_m * 1000:.3f} mm, "
                f"deficit={(self.plane_z_m - self.surface_z_m) * 1000:.3f} mm")


class HeightPlane:
    """A saved plane evaluated along base Z at the measured surface's X/Y."""

    def __init__(self, base_from_plane, reference):
        self.reference = reference
        try:
            matrix = rigid_matrix(base_from_plane, reference)
            self.origin = matrix[:3, 3].copy()
            self.normal = matrix[:3, 2].copy()
            if abs(self.normal[2]) < 1e-9:
                raise ValueError("plane is parallel to vertical travel")
        except (ValueError, TypeError, OverflowError) as exc:
            raise SurfaceGeometryError(f"Invalid {reference}: {exc}") from exc

    def measure(self, surface_base):
        try:
            point = np.asarray(surface_base, dtype=float)
            if point.shape != (3,) or not np.isfinite(point).all():
                raise ValueError("surface must contain three finite base-frame coordinates")
            plane_z = self.origin[2] - np.dot(
                self.normal[:2], point[:2] - self.origin[:2]) / self.normal[2]
            if not np.isfinite(plane_z):
                raise ValueError("nonfinite plane height at surface X/Y")
        except (ValueError, TypeError, OverflowError) as exc:
            raise SurfaceGeometryError(f"Invalid {self.reference} measurement: {exc}") from exc
        return SurfaceHeight(self.reference, float(point[2]), float(plane_z))
