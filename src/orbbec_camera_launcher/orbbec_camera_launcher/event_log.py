from __future__ import annotations

import fcntl
import json
import threading
import time
from datetime import datetime, timezone
from pathlib import Path


PACKAGE_NAME = 'orbbec_camera_launcher'
MAX_EVENTS = 1000


class PackageEventLogger:
    def __init__(self, project_root: Path) -> None:
        log_dir = project_root / 'logs' / PACKAGE_NAME
        log_dir.mkdir(parents=True, exist_ok=True)
        self.path = log_dir / 'events.jsonl'
        self.path.touch(exist_ok=True)
        self._started = time.monotonic()
        self._thread_lock = threading.Lock()

    def record(self, level: str, event: str, message: str, **details: object) -> None:
        if level not in {'INFO', 'WARNING', 'ERROR'}:
            raise ValueError(f'Unsupported event level: {level}')
        record = {
            'timestamp': (
                datetime.now(timezone.utc)
                .isoformat(timespec='milliseconds')
                .replace('+00:00', 'Z')
            ),
            'package': PACKAGE_NAME,
            'level': level,
            'event': event,
            'message': message,
            'elapsed_sec': round(time.monotonic() - self._started, 3),
        }
        record.update(details)
        encoded = json.dumps(record, separators=(',', ':'), sort_keys=True) + '\n'

        with self._thread_lock:
            with self.path.open('a+', encoding='utf-8') as log_file:
                fcntl.flock(log_file.fileno(), fcntl.LOCK_EX)
                log_file.seek(0)
                event_count = sum(1 for _ in log_file)
                if event_count >= MAX_EVENTS:
                    log_file.seek(0)
                    log_file.truncate()
                else:
                    log_file.seek(0, 2)
                log_file.write(encoded)
                log_file.flush()
                fcntl.flock(log_file.fileno(), fcntl.LOCK_UN)

    def cursor(self) -> int:
        """Return a process-safe byte cursor for later supervisor-result reads."""
        with self._thread_lock:
            with self.path.open('r', encoding='utf-8') as log_file:
                fcntl.flock(log_file.fileno(), fcntl.LOCK_SH)
                log_file.seek(0, 2)
                offset = log_file.tell()
                fcntl.flock(log_file.fileno(), fcntl.LOCK_UN)
        return offset

    def records_since(self, offset: int) -> list[dict[str, object]]:
        """Read complete JSONL records written since a previously captured cursor."""
        if isinstance(offset, bool) or not isinstance(offset, int) or offset < 0:
            raise ValueError('Event-log cursor must be a non-negative integer')
        with self._thread_lock:
            with self.path.open('r', encoding='utf-8') as log_file:
                fcntl.flock(log_file.fileno(), fcntl.LOCK_SH)
                log_file.seek(0, 2)
                size = log_file.tell()
                # A smaller file means the 1,000-event cap replaced the old
                # contents. The active run's records then begin at byte zero.
                log_file.seek(0 if size < offset else offset)
                lines = log_file.readlines()
                fcntl.flock(log_file.fileno(), fcntl.LOCK_UN)
        records = []
        for line_number, line in enumerate(lines, start=1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise RuntimeError(
                    f'Malformed package event record {line_number}: {exc}'
                ) from exc
            if not isinstance(record, dict):
                raise RuntimeError(
                    f'Package event record {line_number} must be a JSON object'
                )
            records.append(record)
        return records
