"""Two fresh tray poses, compared in base_link without averaging or axis relabelling."""

import math

import numpy as np

from camera_calibration_gui.calibration_core import quaternion_to_rotation_matrix


SCHEMA_VERSION = 1
WIDTH_FRACTION = .05
ROTATION_LIMIT_DEG = 5.


def sample(selected, stamp_ns):
    return {"stamp_ns": stamp_ns, "class_id": selected["class_id"],
            "position": list(selected["position"]),
            "quaternion": list(selected["quaternion"]),
            "width_mm": selected["width_mm"], "length_mm": selected["length_mm"]}


def geometry(value, taught):
    if (type(value) is not dict or set(value) != {
            "stamp_ns", "class_id", "position", "quaternion", "width_mm", "length_mm"}
            or type(value["stamp_ns"]) is not int or value["stamp_ns"] <= 0
            or type(value["class_id"]) is not int):
        raise ValueError("Malformed tray stability sample")
    position = np.asarray(value["position"], dtype=float)
    quaternion = np.asarray(value["quaternion"], dtype=float)
    size = np.asarray([value["width_mm"], value["length_mm"]], dtype=float)
    if (position.shape != (3,) or quaternion.shape != (4,)
            or not all(np.isfinite(v).all() for v in (position, quaternion, size))
            or abs(np.linalg.norm(quaternion) - 1) > 1e-6
            or not 0 < size[0] <= size[1]
            or np.any(np.abs(size - [taught["width_mm"], taught["length_mm"]])
                      > taught["tolerance_mm"] + 1e-9)):
        raise ValueError("Invalid tray stability pose or dimensions outside taught tolerance")
    rotation = quaternion_to_rotation_matrix(*quaternion)
    center = position + rotation @ np.r_[size / 2000, 0.]
    return position, rotation, center


def compare(first, second, taught, placement=None, *, attempts=2):
    a, ra, ca = geometry(first, taught)
    b, rb, cb = geometry(second, taught)
    if second["stamp_ns"] <= first["stamp_ns"]:
        raise ValueError("Tray stability requires two advancing RGB timestamps")
    limit = WIDTH_FRACTION * taught["width_mm"]
    deltas = {"origin_mm": float(np.linalg.norm(a - b) * 1000),
              "center_mm": float(np.linalg.norm(ca - cb) * 1000),
              "rotation_deg": math.degrees(math.acos(float(np.clip(
                  (np.trace(ra.T @ rb) - 1) / 2, -1., 1.))))}
    if placement is not None:
        offset = np.array([placement["x_mm"], placement["y_mm"], 0.]) / 1000
        deltas["placement_mm"] = float(np.linalg.norm((a + ra @ offset) -
                                                     (b + rb @ offset)) * 1000)
    accepted = (first["class_id"] == second["class_id"]
                and max(v for k, v in deltas.items() if k.endswith("_mm")) <= limit + 1e-9
                and deltas["rotation_deg"] <= ROTATION_LIMIT_DEG + 1e-9)
    details = ", ".join(f"{key}={value:.3f}" for key, value in deltas.items())
    details += (f"; limits {limit:.3f} mm / {ROTATION_LIMIT_DEG:.1f} deg; "
                f"classes {first['class_id']}/{second['class_id']}")
    return {"schema_version": SCHEMA_VERSION, "accepted": bool(accepted),
            "first": first, "second": second, "processed_frames": attempts,
            "translation_limit_mm": limit, "rotation_limit_deg": ROTATION_LIMIT_DEG,
            "differences": deltas,
            "reason": ("Stable tray pair: " if accepted else "Tray pair disagrees: ") + details}


def validate_evidence(evidence, selected, taught, placement, start_ns, now_ns):
    """Recompute the proof at the controller boundary; never trust an accepted flag."""
    if (type(evidence) is not dict or type(evidence.get("schema_version")) is not int
            or evidence["schema_version"] != SCHEMA_VERSION
            or type(evidence.get("accepted")) is not bool
            or type(evidence.get("processed_frames")) is not int
            or evidence["processed_frames"] < 2):
        raise ValueError("Two-pose tray evidence missing/unsupported; rebuild tray provider")
    first, second = evidence["first"], evidence["second"]
    geometry(first, taught)
    geometry(second, taught)
    if not start_ns < first["stamp_ns"] < second["stamp_ns"] <= now_ns:
        raise ValueError("Cached, repeated or future tray stability sample")
    if (set(selected) != set(second) or selected["stamp_ns"] != second["stamp_ns"]
            or selected["class_id"] != second["class_id"]
            or any(not np.allclose(selected[key], second[key], rtol=0., atol=1e-9)
                   for key in ("position", "quaternion", "width_mm", "length_mm"))):
        raise ValueError("Returned tray differs from the second stability sample")
    expected = compare(first, second, taught, placement, attempts=evidence["processed_frames"])
    timings = evidence.get("timings_ms")
    if (type(timings) is not dict or set(timings) != {
            "first_prediction", "second_prediction", "search"}
            or any(type(v) not in (int, float) or not math.isfinite(v) or v < 0
                   for v in timings.values())):
        raise ValueError("Malformed two-pose timing evidence")
    if not expected["accepted"] or evidence != {**expected, "timings_ms": timings}:
        raise ValueError("Tray stability proof disagrees with its poses or configured limits")
