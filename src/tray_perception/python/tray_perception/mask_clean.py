"""Detection Mask Clean in the private native worker, before metric tray fitting."""

import math


KERNEL_PX = 3
MIN_RETAINED_FRACTION = .8


def validate_evidence(value):
    """Validate cleanup diagnostics at the ROS boundary without native imports."""
    counts = ("raw_pixels", "kept_pixels", "removed_pixels", "components_before",
              "components_after")
    if (type(value) is not dict or any(
            type(value.get(key)) is not int or value[key] < 0 for key in counts)
            or value.get("kernel_px") != KERNEL_PX
            or value.get("minimum_retained_fraction") != MIN_RETAINED_FRACTION
            or type(value.get("source_touches_image_edge")) is not bool
            or value.get("status") not in ("unchanged", "cleaned", "rejected")
            or type(value.get("reason")) is not str):
        raise ValueError("Malformed Detection Mask Clean evidence")
    raw, kept, removed = (value[key] for key in counts[:3])
    fraction = value.get("retained_fraction")
    rejected = value["status"] == "rejected"
    if (raw != kept + removed or type(fraction) not in (int, float)
            or not math.isfinite(fraction)
            or abs(fraction - (kept / raw if raw else 0.)) > 1e-9
            or rejected != bool(value["reason"])
            or (not rejected and (not kept or fraction < MIN_RETAINED_FRACTION))
            or (value["status"] == "unchanged" and removed != 0)
            or (value["status"] == "cleaned" and removed == 0)):
        raise ValueError("Inconsistent Detection Mask Clean evidence")


def summary(detection):
    """Compact operator-facing summary; OBB detections have no cleanup stage."""
    value = detection.get("mask_clean") if detection is not None else None
    if value is None:
        return ""
    return (f"Detection Mask Clean: {value['status']} | "
            f"{value['retained_fraction']:.0%} retained | "
            f"{value['removed_pixels']} pixels removed")


def clean_mask(mask, cv2, np):
    """Split narrow bridges and retain a dominant connected region by pixel area."""
    if (mask.ndim != 2 or not mask.size or not np.isfinite(mask).all()
            or not np.isin(mask, (0, 1)).all()):
        raise RuntimeError("Detection Mask Clean requires a finite binary mask")
    binary = np.ascontiguousarray(mask, dtype=np.uint8)
    before, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    total = int(binary.sum())
    opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((KERNEL_PX, KERNEL_PX), np.uint8),
                              borderType=cv2.BORDER_CONSTANT, borderValue=0)
    after, cleaned_labels, cleaned_stats, _ = cv2.connectedComponentsWithStats(
        opened, connectivity=8)
    selected = np.zeros_like(binary)
    original = np.zeros_like(binary)
    if after > 1:
        largest = 1 + int(np.argmax(cleaned_stats[1:, cv2.CC_STAT_AREA]))
        selected[cleaned_labels == largest] = 1
        source = int(labels[selected.astype(bool)][0])
        original[labels == source] = 1
    elif before > 1:
        largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        original[labels == largest] = 1
    kept = int(selected.sum())
    fraction = kept / total if total else 0.
    reason = ("No solid mask region remains" if not kept else
              "No dominant region after splitting blobs "
              f"({fraction:.1%} retained; need {MIN_RETAINED_FRACTION:.0%})"
              if fraction < MIN_RETAINED_FRACTION else "")
    evidence = {"kernel_px": KERNEL_PX, "minimum_retained_fraction": MIN_RETAINED_FRACTION,
                "raw_pixels": total, "kept_pixels": kept, "removed_pixels": total - kept,
                "components_before": before - 1, "components_after": after - 1,
                "retained_fraction": fraction,
                "status": "rejected" if reason else "cleaned" if kept != total else "unchanged",
                "reason": reason}
    return selected, original, evidence


def _polygon(mask, original_shape, cv2, np):
    # Import only in the pinned native process; use its exact letterbox inverse.
    from ultralytics.utils.ops import scale_coords

    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    if not contours:
        return np.empty((0, 2), np.float32)
    contour = max(contours, key=cv2.contourArea).reshape(-1, 2).astype(np.float32)
    return scale_coords(mask.shape, contour, original_shape)


def clean_objects(result, names, maximum, cv2, np):
    """One independent cleanup per model instance; never access merged masks.xy."""
    boxes = result.boxes
    if boxes is None:
        raise RuntimeError("Requested mask output has no class/confidence records")
    labels, scores = boxes.cls.cpu().numpy(), boxes.conf.cpu().numpy()
    if (labels.ndim != 1 or scores.shape != labels.shape or len(labels) > maximum):
        raise RuntimeError("Malformed segmentation detection count")
    if not len(labels):
        return []
    if result.masks is None:
        raise RuntimeError("Requested mask output is absent")
    masks = result.masks.data.cpu().numpy()
    if (masks.ndim != 3 or masks.shape[0] != len(labels)
            or not all(0 < size <= 4096 for size in masks.shape[1:])):
        raise RuntimeError("Malformed segmentation mask dimensions/count")
    height, width = result.orig_shape
    objects = []
    for index, (mask, label, score) in enumerate(zip(masks, labels, scores)):
        if (not math.isfinite(float(label)) or int(label) != label or int(label) not in names
                or not math.isfinite(float(score)) or not 0 <= score <= 1):
            raise RuntimeError("Malformed segmentation class/confidence")
        cleaned, original, evidence = clean_mask(mask, cv2, np)
        polygon = _polygon(cleaned, result.orig_shape, cv2, np)
        boundary = _polygon(original, result.orig_shape, cv2, np)
        evidence["source_touches_image_edge"] = bool(len(boundary) and (
            np.any(boundary[:, 0] <= 0) or np.any(boundary[:, 0] >= width - 1)
            or np.any(boundary[:, 1] <= 0) or np.any(boundary[:, 1] >= height - 1)))
        if len(polygon) < 3 or abs(cv2.contourArea(polygon)) < 1:
            evidence.update(status="rejected", reason="No measurable cleaned mask region")
            polygon = np.empty((0, 2), np.float32)
        if evidence["status"] == "rejected":
            polygon = np.empty((0, 2), np.float32)  # No guessed pose/axes for ambiguous masks.
        objects.append({"index": index, "class_id": int(label), "class_name": names[int(label)],
                        "confidence": float(score), "polygon": polygon, "mask_clean": evidence})
    return objects
