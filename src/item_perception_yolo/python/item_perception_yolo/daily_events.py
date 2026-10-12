"""Package-owned daily JSONL archives; standard library only, no ROS authority."""

from contextlib import contextmanager
from contextvars import ContextVar
from datetime import datetime, timedelta, timezone
import fcntl
import json
import os
from pathlib import Path
import re
import sys
import threading
import time
import uuid
from zoneinfo import ZoneInfo


LOG_TIMEZONE = ZoneInfo("Asia/Dubai")
RETENTION_DAYS = 30
_CONTEXT = ContextVar("event_context", default={})


def current_event_context():
    return dict(_CONTEXT.get())


@contextmanager
def event_context(**fields):
    token = _CONTEXT.set({**_CONTEXT.get(), **fields})
    try:
        yield
    finally:
        _CONTEXT.reset(token)


def log_day(now=None):
    return (now or datetime.now(timezone.utc)).astimezone(LOG_TIMEZONE).date()


def prune_daily_files(directory, today):
    """Only our exact dated regular files; preserve legacy/operator/symlink files."""
    cutoff = today - timedelta(days=RETENTION_DAYS - 1)
    for path in directory.glob("events-????-??-??.jsonl"):
        if path.is_symlink() or not path.is_file():
            continue
        match = re.fullmatch(r"events-(\d{4}-\d{2}-\d{2})\.jsonl", path.name)
        try:
            day = datetime.strptime(match[1], "%Y-%m-%d").date() if match else today
        except ValueError:
            continue
        if day < cutoff:
            path.unlink(missing_ok=True)


class DailyEventLogger:
    def __init__(self, root, package, node, *, clock=None):
        self.root = Path(root)
        self.directory = self.root / "logs" / package
        self.package, self.node = package, node
        self.clock = clock or (lambda: datetime.now(timezone.utc))
        self.lock = threading.RLock()
        self.session_id = uuid.uuid4().hex
        self.sequence = 0
        self.context = {}
        self.last_pruned = None
        self.observer = None
        self.last_error = ""
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
        except OSError:
            pass  # Report once on the first attempted record.

    @property
    def path(self):
        return self.directory / f"events-{log_day(self.clock()).isoformat()}.jsonl"

    def record(self, level, event, message, **fields):
        with self.lock:
            now = self.clock()
            self.sequence += 1
            payload = {**self.context, **_CONTEXT.get(), **fields,
                       "timestamp_utc": now.astimezone(timezone.utc).isoformat().replace(
                           "+00:00", "Z"),
                       "monotonic_ns": time.monotonic_ns(), "session_id": self.session_id,
                       "event_sequence": self.sequence, "package": self.package,
                       "node": self.node, "level": str(level), "event": str(event),
                       "message": str(message)}
            try:
                self.directory.mkdir(parents=True, exist_ok=True)
                today = log_day(now)
                if self.last_pruned != today:
                    prune_daily_files(self.directory, today)
                    self.last_pruned = today
                path = self.directory / f"events-{today.isoformat()}.jsonl"
                data = json.dumps(payload, sort_keys=True, allow_nan=False) + "\n"
                fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
                with os.fdopen(fd, "a", encoding="utf-8") as stream:
                    fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
                    stream.write(data)
                    stream.flush()
                    fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
                self.last_error = ""
            except (OSError, ValueError, TypeError) as exc:
                # Diagnostics never change command admission or safety handling.
                error = f"{self.package} daily log failed: {exc}"
                if error != self.last_error:
                    print(error, file=sys.stderr)
                self.last_error = error
            if self.observer is not None:
                try:
                    self.observer(payload)
                except (ValueError, TypeError, KeyError) as exc:
                    print(f"Timing observer failed: {exc}", file=sys.stderr)
                    self.observer = None
            return payload
