"""Strict RGB and preview-setting validators; no native runtime imports."""

import re
import time


from .preview_protocol import MAX_IMAGE_BYTES

FRAME_MAX_AGE_SEC = 0.5


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
