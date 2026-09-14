"""One portable XY border definition for Bin Teach and Item Teach.

NumPy is supplied by the caller: this module imports no ROS or native runtime.
Only the destination platform-from-optical transform places the saved geometry.
"""

ROI_BORDER_SAMPLES_PER_EDGE = 32


def border_in_optical(roi_xy, platform_from_optical, np):
    """Return the saved Z=0 border in current optical coordinates, without flattening."""
    xy = np.asarray(roi_xy, dtype=np.float64)
    transform = np.asarray(platform_from_optical, dtype=np.float64)
    if xy.shape != (4, 2) or not np.isfinite(xy).all():
        raise ValueError("Loaded bin ROI must contain four finite metric XY points")
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], rtol=0, atol=1e-6)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3],
                               np.eye(3), rtol=0, atol=1e-6)
            or abs(np.linalg.det(transform[:3, :3]) - 1) > 1e-6):
        raise ValueError("Invalid platform-from-optical rigid transform for loaded bin ROI")
    fractions = np.arange(ROI_BORDER_SAMPLES_PER_EDGE) / ROI_BORDER_SAMPLES_PER_EDGE
    samples = np.concatenate([
        start + fractions[:, None] * (end - start)
        for start, end in zip(xy, np.roll(xy, -1, axis=0))
    ])
    platform_points = np.column_stack((samples, np.zeros(len(samples))))
    optical = (platform_points - transform[:3, 3]) @ transform[:3, :3]
    if not np.isfinite(optical).all() or np.any(optical[:, 2] <= 0.0):
        raise ValueError("Saved bin border is at or behind camera; projection is unavailable")
    return np.ascontiguousarray(optical)
