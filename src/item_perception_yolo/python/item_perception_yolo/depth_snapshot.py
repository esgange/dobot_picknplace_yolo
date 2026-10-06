"""Immutable temporal depth snapshots; no OpenCV, model or hardware dependency."""

import time
import warnings

import numpy as np


DEPTH_FRAME_COUNTS = (1, 3, 5)
DEFAULT_DEPTH_FRAME_COUNT = 3
ITEM_MIN_DEPTH_MM = 500.0
DEPTH_ENCODING = "32FC1_mm"


def validate_depth_frame_count(value):
    if type(value) is not int or value not in DEPTH_FRAME_COUNTS:
        raise ValueError("depth_frame_count must be 1, 3 or 5")
    return value


def item_depth_limits(quality):
    low = max(ITEM_MIN_DEPTH_MM, quality["depth_min_mm"])
    high = quality["depth_max_mm"]
    if not np.isfinite([low, high]).all() or high < low:
        raise ValueError(
            "Item depth maximum must reach its effective item minimum (at least 500 mm)")
    return low, high


def median_depth_snapshot(frames, quality):
    """Median of a strict majority of valid readings, retaining half millimetres."""
    started = time.monotonic()
    count = validate_depth_frame_count(len(frames))
    newest = frames[-1]
    width, height = newest["width"], newest["height"]
    stamps = [frame["stamp_ns"] for frame in frames]
    if any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError("Depth frames must have distinct advancing timestamps")
    low, high = item_depth_limits(quality)
    arrays = []
    for frame in frames:
        if ((frame["width"], frame["height"]) != (width, height)
                or len(frame["depth"]) != width * height * 2):
            raise ValueError("Temporal depth frames must share the original 16UC1 layout")
        arrays.append(np.frombuffer(frame["depth"], "<u2").reshape(height, width))
    values = np.stack(arrays).astype(np.float32)
    usable = np.isfinite(values) & (values >= low) & (values <= high)
    values[~usable] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # Explicitly reject all-NaN pixels below.
        median = np.nanmedian(values, axis=0).astype("<f4")
    median[usable.sum(axis=0) < count // 2 + 1] = np.nan
    return {**newest, "depth": median.tobytes(), "encoding": DEPTH_ENCODING,
            "frame_stamps_ns": stamps,
            "capture_span_ms": (stamps[-1] - stamps[0]) / 1e6,
            "median_ms": (time.monotonic() - started) * 1000.}


def camera_window_stationary(transforms):
    """Bound every pair in the capture window, including RGB-time camera pose."""
    matrices = [np.asarray(value, dtype=float) for value in transforms]
    if not matrices or any(m.shape != (4, 4) or not np.isfinite(m).all() for m in matrices):
        return False
    for index, anchor in enumerate(matrices):
        for matrix in matrices[index+1:]:
            translation = np.linalg.norm(matrix[:3, 3] - anchor[:3, 3])
            cosine = np.clip((np.trace(anchor[:3, :3].T @ matrix[:3, :3]) - 1.) / 2., -1., 1.)
            if translation > .00005 + 1e-12 or np.degrees(np.arccos(cosine)) > .05 + 1e-10:
                return False
    return True
