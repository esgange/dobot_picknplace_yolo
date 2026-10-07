"""Strict placement sampling contract; no model, ROS or native imports."""

import math

from item_perception_yolo.item_teach_core import QUALITY_DEFAULTS, validate_quality


FIELDS = ("x_mm", "y_mm", "diameter_mm", *QUALITY_DEFAULTS)
PLACEMENT_MINIMUM_DEPTH_FRACTION = 0.3


def validate_sampling(value):
    if type(value) is not dict or set(value) != set(FIELDS):
        raise ValueError("Placement sampling requires the exact typed settings")
    for key in ("x_mm", "y_mm", "diameter_mm"):
        number = value[key]
        if type(number) not in (int, float) or not math.isfinite(number) or number <= 0:
            raise ValueError(f"Placement {key} must be finite and positive")
    validate_quality({key: value[key] for key in QUALITY_DEFAULTS})
    return value


def sampling_from_message(message):
    return validate_sampling({key: getattr(message, key) for key in FIELDS})


def sampling_from_item(profile, x_mm, y_mm):
    sampling = validate_sampling({"x_mm": x_mm, "y_mm": y_mm,
                                  "diameter_mm": profile["geometry"]["pickdepth_radius"],
                                  **profile["quality"]})
    # Placement has its own coverage requirement; keep the saved pick quality intact.
    return {**sampling, "minimum_depth_fraction": PLACEMENT_MINIMUM_DEPTH_FRACTION}
