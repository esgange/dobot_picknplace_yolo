"""Bounded package events. Callers provide only sanitized outcomes, never settings."""
from datetime import datetime, timezone
import fcntl
import json


def _record(root, event, message):
    path = root / 'logs/remote_setup/events.jsonl'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a+', encoding='utf-8') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        stream.seek(0)
        if sum(1 for _ in stream) >= 1000:
            stream.seek(0)
            stream.truncate()
        stream.write(json.dumps({'timestamp': datetime.now(timezone.utc).isoformat(),
                                 'package': 'remote_setup', 'event': event,
                                 'message': message}) + '\n')


def record(root, event, message):
    try:
        _record(root, event, message)
    except OSError:
        # A diagnostics failure must not crash the GUI or change a test outcome.
        import sys
        print('Remote Setup: package event log is unavailable.', file=sys.stderr)
