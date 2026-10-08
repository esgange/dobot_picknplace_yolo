"""Read-only preview validation, passive camera display and completed capture feedback."""

import base64
import math
import re
import threading
import time

import numpy as np

from .preview_protocol import MAX_IMAGE_BYTES

FRAME_MAX_AGE_SEC = 0.5
CAPTURE_HOLD_SEC = 5.0
PASSIVE_RESULT_MAX_AGE_SEC = 5.0


def validate_prefix(prefix):
    if type(prefix) is not str or re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", prefix) is None:
        raise ValueError("Enter an exact camera prefix without slashes, e.g. bin_camera")
    return prefix


def validate_preview_settings(task, yolo):
    from .item_teach_core import validate_yolo_settings
    validate_yolo_settings(task, yolo)


def frame_from_message(message, prefix, now_ns):
    if message.header.frame_id != f"{prefix}_color_optical_frame":
        raise ValueError("RGB frame_id does not match the explicitly connected camera prefix")
    sec, nano = message.header.stamp.sec, message.header.stamp.nanosec
    if sec < 0 or not 0 <= nano < 1_000_000_000 or sec * 1_000_000_000 + nano <= 0:
        raise ValueError("RGB timestamp must be valid and non-zero")
    age = (now_ns - (sec * 1_000_000_000 + nano)) / 1e9
    if not 0 <= age <= FRAME_MAX_AGE_SEC:
        raise ValueError(f"RGB timestamp is stale or future-dated: age={age:.3f}s")
    width, height = message.width, message.height
    if (message.encoding != "rgb8" or message.is_bigendian != 0
            or not 0 < width <= 4096 or not 0 < height <= 4096
            or message.step != width * 3 or len(message.data) != width * height * 3
            or len(message.data) > MAX_IMAGE_BYTES):
        raise ValueError("RGB must be tightly packed little-endian rgb8 with valid dimensions")
    return {
        "width": width, "height": height, "stamp_ns": sec * 1_000_000_000 + nano,
        "received_at": time.monotonic(), "rgb": bytes(message.data),
    }


# OpenCV 4.10 COLORMAP_TURBO in RGB order. The GUI never imports native OpenCV.
_TURBO = np.frombuffer(base64.b64decode(
    "MBI7MhVDMxhKNBtRNR5YNiFfNyRmOCdtOSpzOi15Oy+APDKGPTWLPjiRPzuXPz6cQECiQUOnQUasQkmxQku1"
    "Q066RFG/RFTDRFbHRVnLRVzPRV7TRmHWRmTaRmbdRmngRmvjR27mR3HpR3PrR3buR3jwR3vyRn30RoD2RoL4"
    "RoX6Rof7RYr8RYz9RI/+Q5H+QpT/QZb/QJn/Ppv+PZ7+O6D9OqP8OKX7N6j6Nav4M633Ma/1L7L0LrTyLLfw"
    "KrnuKLzrJ77pJcDnI8PkIsXiIMffH8ndHsvaHM3YG9DVGtLSGtTQGdXNGNfKGNnIGNvFGN3CGN7AGOC9GeK7"
    "GeO5GuS2HOa0HeeyH+mvIOqsIuuqJeynJ+6kKu+hLPCeL/GbMvKYNfOUOPSRPPWOP/aKQ/eHRviESviATvl9"
    "Uvp6Vfp2WftzXfxvYfxsZf1paf1mbf5icf5fdf5cef5Zff9WgP9ThP9RiP9Oi/9Lj/9Jkv9Hlv5Emf5CnP5A"
    "n/0/of09pPw8p/w6qfs5rPs4r/o3sfk2tPg2t/c1ufY1vPU0vvQ0wfM0w/E0xvA0yO80y+00zew00Oo00uk1"
    "1Oc11+U12eQ22+I23eA339834d0349s45dk459c56dU569M57NE67s8678068cs68sk69Mc69cU69sM698E6+L45"
    "+bw5+ro5+7g4+7Y3/LM2/LE2/a41/aw0/qkz/qcy/qQx/qEw/p4v/pst/pks/pYr/pMq/pAp/Y0n/Yom/Icl"
    "/IQj+4Ei+34h+nsf+Xge+XUd+HIc928a9mwZ9WkY9GYX82MV8mAU8V0T8FsS71gR7VUQ7FMP61AO6k4N6EsM"
    "50kM5UcL5EUK4kMK4UEJ3z8I3T0I3DsH2jkH2DcG1jUG1DMF0jEF0C8Fzi0EzCsEyioEyCgDxSYDwyUDwSMC"
    "viECvCACuR4Ctx0CtBsBshoBrxgBrBcBqRYBpxQBpBMBoRIBnhABmw8BmA4BlQ0BkgsBjgoBiwkCiAgChQcC"
    "gQYCfgUCegQD"), dtype=np.uint8).reshape(256, 3)


class CaptureMailbox:
    """Keep at most one completed service result for the GUI thread to consume."""

    def __init__(self):
        self.lock = threading.Lock()
        self.pending = None

    def put(self, response, view):
        with self.lock:
            self.pending = {"response": response, "view": view}

    def take(self):
        with self.lock:
            pending, self.pending = self.pending, None
            return pending


class PassiveCameraView:
    """Color raw depth once per frame, without inference, calibration or pose reuse."""

    def __init__(self):
        self.depth_key = self.depth_pixels = None

    def make(self, rgb, depth, now_ns):
        if rgb is None:
            return None
        result = {**rgb, "preview_mode": "passive", "metadata": {}, "depth_rgb": None,
                  "depth_error": "Waiting for synchronized registered depth"}
        if (depth is None or (depth["width"], depth["height"]) != (rgb["width"], rgb["height"])
                or not 0 <= (now_ns - depth["stamp_ns"]) / 1e9 <= .5
                or abs(depth["stamp_ns"] - rgb["stamp_ns"]) > 100_000_000):
            return result
        key = depth["stamp_ns"], id(depth["depth"]), depth["width"], depth["height"]
        if key != self.depth_key:
            values = np.frombuffer(depth["depth"], "<u2")
            if values.size != rgb["width"] * rgb["height"]:
                return result
            # Same 200–1000 mm display range as the native tray depth view.
            scaled = np.clip((values.astype(float) - 200) * 255 / 800, 0, 255).astype(np.uint8)
            pixels = _TURBO[scaled].copy()
            pixels[(values < 200) | (values > 1000)] = 0
            self.depth_key, self.depth_pixels = key, pixels.tobytes()
        result.update(depth_rgb=self.depth_pixels, depth_stamp_ns=depth["stamp_ns"], depth_error="")
        return result


def passive_inference_view(raw, completed, enabled, now_ns, *, inspect_detections=False):
    """Hold one matched annotated pair between inferences; never paint on newer raw pixels."""
    if (raw is None or not enabled or completed is None
            or not completed.get("passive_overlay")
            or not 0 <= (now_ns - completed["stamp_ns"]) / 1e9 <= PASSIVE_RESULT_MAX_AGE_SEC):
        return raw
    return {**completed, "preview_mode": "passive", "passive_overlay": True,
            "metadata": completed.get("metadata", {}) if inspect_detections else {},
            "passive_inspectable": inspect_detections}


def passive_item_details(view, detection):
    """Describe the same validated evidence used to color one displayed Item snapshot."""
    from .item_geometry import passive_detection_color

    snapshot = view.get("rviz") or {}
    rejected = snapshot.get("rejected", [])
    color = passive_detection_color(detection, snapshot.get("candidates", []), rejected)
    name, meaning = {
        (255, 50, 50): ("Red", "failed size check"),
        (255, 220, 0): ("Yellow", "failed height check"),
        (0, 220, 0): ("Green", "valid candidate"),
        (160, 160, 160): ("Gray", "unchecked or another rejection"),
    }[color]
    rejection = next((r for r in rejected if r["source_index"] == detection["source_index"]), {})
    if name == "Red":
        reason = detection.get("size_reason") or "Measured size is outside tolerance"
    elif name == "Green":
        reason = "All candidate checks passed in this snapshot"
    else:
        reason = (rejection.get("reason") or detection.get("measurement_error")
                  or snapshot.get("error") or snapshot.get("pose_error")
                  or (detection.get("size_reason") if detection.get("size_valid") is None else "")
                  or "Candidate checks unavailable for this snapshot")
    measured = detection.get("measurement")
    size = (f"{measured['length_mm']:.1f} × {measured['width_mm']:.1f} mm"
            if measured is not None else "Size unavailable")
    expected = view.get("measurement_geometry")
    target = (f"Taught {expected['height']:g} × {expected['width']:g} mm "
              f"± {expected['tolerance']:g} mm" if expected is not None else "Size target unset")
    return (f"#{detection['source_index']} {detection['class_name']} · {size} · {target}\n"
            f"{name}: {meaning} · {reason}")


def configure_status_band(label):
    from python_qt_binding import QtCore, QtWidgets

    label.setTextFormat(QtCore.Qt.PlainText)
    label.setWordWrap(False)
    label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
    label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)
    label.setStyleSheet(
        "background: #111; color: white; padding: 8px 12px; font-weight: bold;")
    label.ensurePolished()
    label.setFixedHeight(label.fontMetrics().lineSpacing() * 3 + 20)


def capture_status(source, status, count, age, processing_ms, remaining, reason=""):
    """Use identical concise result/age/countdown wording in both teaching GUIs."""
    title = f"{source} · {status} · {count} {'pose' if count == 1 else 'poses'}"
    timing = "Depth unavailable" if age is None else f"Age {max(0., age):.2f}s"
    timing += f" · {processing_ms:.0f}ms · live in {max(0, math.ceil(remaining))}s"
    return "\n".join((title, timing, reason))
