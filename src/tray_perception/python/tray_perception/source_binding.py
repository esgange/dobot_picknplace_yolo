"""File identity fences for explicitly loaded, fully validated tray sources."""

from item_perception_yolo.source_binding import fingerprints, check as _check


def check(binding):
    _check(binding, "tray")
