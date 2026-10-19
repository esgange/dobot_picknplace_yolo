"""Read-only camera interlock; reconnection never acknowledges a robot interruption."""

import threading
import time
from diagnostic_msgs.msg import DiagnosticStatus

from .errors import CommandRejected


TOPIC = '/camera_watchdog/status'
MAX_AGE_SEC = 5.
ALARM = 'CAMERA ALARM: '
RESTORED = 'CAMERA RECOVERY: '


class CameraHealth:
    def __init__(self, expected, *, required=False, clock=time.monotonic):
        self.expected = dict(expected)
        self.required = required
        self.clock = clock
        self.lock = threading.RLock()
        self.seen = False
        self.received = None
        self.stamp_ns = 0
        self.healthy = False
        self.detail = 'Waiting for camera supervisor status'
        self.latched = False

    def update(self, message, publishers, now_ns):
        with self.lock:
            self.seen = True
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            owners = [(p.node_name, p.node_namespace) for p in publishers]
            if owners != [('supervisor', '/camera_watchdog')]:
                self.healthy = False
                self.detail = 'Camera supervisor is missing or has competing publishers'
                return
            if not 0 <= (now_ns - stamp) / 1e9 <= MAX_AGE_SEC or stamp <= self.stamp_ns:
                self.healthy = False
                self.detail = 'Camera supervisor status is stale or not advancing'
                return
            self.received, self.stamp_ns = self.clock(), stamp
            statuses = message.status
            identities = {s.name: s.hardware_id for s in statuses}
            if len(statuses) != len(self.expected) or identities != self.expected:
                self.healthy = False
                self.detail = 'Camera supervisor does not match the configured camera names/serials'
                return
            self.healthy = all(s.level == DiagnosticStatus.OK and dict(
                (v.key, v.value) for v in s.values).get('phase') == 'healthy' for s in statuses)
            details = [dict((v.key, v.value) for v in s.values).get('last_error')
                       or s.message for s in statuses]
            self.detail = '; '.join(dict.fromkeys(details))

    def unavailable(self):
        with self.lock:
            if not (self.required or self.seen):
                return ''
            if self.received is None or self.clock() - self.received > MAX_AGE_SEC:
                detail = 'Camera supervisor status unavailable or older than five seconds'
            elif self.healthy:
                return ''
            else:
                detail = self.detail
            return ALARM + 'Check camera connection, cable and power. ' + detail

    def trip(self):
        with self.lock:
            if not self.unavailable() or self.latched:
                return False
            self.latched = True
            return True

    def message(self, configured):
        with self.lock:
            return self.unavailable() or (
                RESTORED + 'Camera streams restored. ' +
                ('Click Recover before starting a new operation.' if configured else
                 'Load teach configuration to prepare the robot.') if self.latched else '')

    def admit(self, operation, *, acknowledge=False):
        with self.lock:
            reason = self.unavailable()
            if reason:
                raise CommandRejected(reason)
            if self.latched and operation not in ('configure', 'startup', 'recover'):
                raise CommandRejected(RESTORED + 'Explicit preparation or Recover is required')
            if acknowledge:
                self.latched = False


def configured_cameras(env):
    return {f'orbbec_camera_launcher/{env[f"ORBBEC_CAMERA_{slot}_NAME"]}':
            env[f'ORBBEC_CAMERA_{slot}_SERIAL'] for slot in (1, 2)}
