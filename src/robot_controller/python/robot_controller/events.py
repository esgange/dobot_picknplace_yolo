"""Bounded controller/preview event logging, independent of hardware authority."""

import fcntl
import json
import os
from pathlib import Path
import threading

from item_perception_yolo.item_teach_core import utc_now


class PackageEventLogger:
    """Package-owned bounded JSONL event recorder."""

    def __init__(self, root, node_name="robot_controller"):
        self.path = Path(root) / "logs/robot_controller/events.jsonl"
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
        self.node_name = node_name

    def record(self, level, event, message, **fields):
        payload = {
            "timestamp_utc": utc_now(), "package": "robot_controller",
            "node": self.node_name, "level": level, "event": event,
            "message": message, **fields,
        }
        with self.lock:
            with self.path.open("a+", encoding="utf-8") as stream:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
                stream.seek(0)
                count = sum(bool(line.strip()) for line in stream)
                if count >= 1000:
                    stream.seek(0)
                    stream.truncate()
                else:
                    stream.seek(0, os.SEEK_END)
                stream.write(json.dumps(payload, sort_keys=True, allow_nan=False) + "\n")
                stream.flush()
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
