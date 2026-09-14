#!/usr/bin/env python3

import fcntl
import json
import math
import os
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


PACKAGE_NAME = 'dobot_rviz'
JOINT_STATE_TOPIC = '/joint_states'
EXPECTED_JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')
STARTUP_TIMEOUT_SEC = 5.0
STALE_TIMEOUT_SEC = 1.0
CHECK_PERIOD_SEC = 0.1
MAX_EVENTS = 1000


def validate_joint_state_message(message: JointState) -> str:
    if tuple(message.name) != EXPECTED_JOINT_NAMES:
        return (
            f'joint names must be exactly {list(EXPECTED_JOINT_NAMES)}, '
            f'received {list(message.name)}'
        )
    if len(message.position) != len(EXPECTED_JOINT_NAMES):
        return f'position must contain exactly 6 values, received {len(message.position)}'
    if not all(math.isfinite(value) for value in message.position):
        return 'position contains a non-finite value'
    if message.velocity and len(message.velocity) != len(EXPECTED_JOINT_NAMES):
        return (
            'velocity must be empty or contain exactly 6 values, '
            f'received {len(message.velocity)}'
        )
    if message.velocity and not all(math.isfinite(value) for value in message.velocity):
        return 'velocity contains a non-finite value'
    if message.effort and len(message.effort) != len(EXPECTED_JOINT_NAMES):
        return f'effort must be empty or contain exactly 6 values, received {len(message.effort)}'
    if message.effort and not all(math.isfinite(value) for value in message.effort):
        return 'effort contains a non-finite value'
    if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
        return 'header timestamp is zero'
    return ''


def validate_publishers(publishers, expected_node_name: str) -> str:
    if not publishers:
        return 'no publisher exists'
    endpoints = sorted(
        f'{publisher.node_namespace.rstrip("/")}/{publisher.node_name}'
        for publisher in publishers
    )
    expected_full_name = f'/{expected_node_name}'
    if len(publishers) != 1:
        return f'exactly one publisher is required, found {len(publishers)}: {endpoints}'
    if endpoints[0] != expected_full_name:
        return f'publisher must be exactly {expected_full_name}, found {endpoints[0]}'
    return ''


class PackageEventLogger:
    def __init__(self, project_root: Path) -> None:
        if not (project_root / '.env.example').is_file() or not (project_root / 'src').is_dir():
            raise RuntimeError(f'invalid project root: {project_root}')
        log_dir = project_root / 'logs' / PACKAGE_NAME
        log_dir.mkdir(parents=True, exist_ok=True)
        self.path = log_dir / 'events.jsonl'
        self.path.touch(exist_ok=True)
        self._started = time.monotonic()
        self._thread_lock = threading.Lock()

    def record(self, level: str, event: str, message: str, **details: object) -> None:
        if level not in {'INFO', 'WARNING', 'ERROR'}:
            raise ValueError(f'unsupported event level: {level}')
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
                    log_file.seek(0, os.SEEK_END)
                log_file.write(encoded)
                log_file.flush()
                fcntl.flock(log_file.fileno(), fcntl.LOCK_UN)


class ActualJointStateMonitor(Node):
    def __init__(self) -> None:
        super().__init__('actual_joint_state_monitor')
        expected_publisher = self.declare_parameter(
            'expected_publisher_node', '__REQUIRED__'
        ).value
        project_root_text = self.declare_parameter('project_root', '__REQUIRED__').value

        if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('ROS_LOCALHOST_ONLY must be exactly 1')
        if not isinstance(expected_publisher, str) or expected_publisher == '__REQUIRED__':
            raise RuntimeError('expected_publisher_node is required')
        if not isinstance(project_root_text, str) or project_root_text == '__REQUIRED__':
            raise RuntimeError('project_root is required')

        self._expected_publisher = expected_publisher
        self._event_logger = PackageEventLogger(Path(project_root_text).resolve())
        self._startup_deadline = time.monotonic() + STARTUP_TIMEOUT_SEC
        self._last_valid_message_monotonic = None
        self._healthy = False
        self.exit_code = None

        self.create_subscription(JointState, JOINT_STATE_TOPIC, self._joint_state_callback, 10)
        self.create_timer(CHECK_PERIOD_SEC, self._check_stream)
        self._event_logger.record(
            'INFO',
            'viewer_startup',
            f'waiting for canonical actual joint states on {JOINT_STATE_TOPIC}',
            expected_publisher=f'/{self._expected_publisher}',
            startup_timeout_sec=STARTUP_TIMEOUT_SEC,
            stale_timeout_sec=STALE_TIMEOUT_SEC,
        )
        self.get_logger().info(
            f'Waiting up to {STARTUP_TIMEOUT_SEC:.1f}s for {JOINT_STATE_TOPIC} from '
            f'/{self._expected_publisher}; stale timeout is {STALE_TIMEOUT_SEC:.1f}s.'
        )

    @property
    def event_logger(self) -> PackageEventLogger:
        return self._event_logger

    def _fail(self, event: str, message: str) -> None:
        if self.exit_code is not None:
            return
        self.exit_code = 1
        self.get_logger().fatal(message)
        self._event_logger.record('ERROR', event, message)
        rclpy.shutdown()

    def _joint_state_callback(self, message: JointState) -> None:
        validation_error = validate_joint_state_message(message)
        if validation_error:
            self._fail(
                'joint_state_invalid',
                f'Rejected {JOINT_STATE_TOPIC}: {validation_error}',
            )
            return
        self._last_valid_message_monotonic = time.monotonic()

    def _check_stream(self) -> None:
        now = time.monotonic()
        publishers = self.get_publishers_info_by_topic(JOINT_STATE_TOPIC)
        publisher_error = validate_publishers(publishers, self._expected_publisher)

        if publisher_error and publishers:
            self._fail(
                'joint_state_publisher_invalid',
                f'Rejected {JOINT_STATE_TOPIC}: {publisher_error}',
            )
            return

        if not self._healthy:
            if not publisher_error and self._last_valid_message_monotonic is not None:
                age = now - self._last_valid_message_monotonic
                if age <= STALE_TIMEOUT_SEC:
                    self._healthy = True
                    message = (
                        f'Using actual CR10 joint states only from /{self._expected_publisher} '
                        f'on {JOINT_STATE_TOPIC}'
                    )
                    self.get_logger().info(message)
                    self._event_logger.record('INFO', 'joint_state_stream_healthy', message)
                    return
            if now >= self._startup_deadline:
                detail = publisher_error
                if not detail and self._last_valid_message_monotonic is None:
                    detail = 'publisher exists but no valid message was received'
                self._fail(
                    'joint_state_startup_failed',
                    f'No valid actual joint-state stream within '
                    f'{STARTUP_TIMEOUT_SEC:.1f}s: {detail}',
                )
            return

        if publisher_error:
            self._fail(
                'joint_state_publisher_lost',
                f'Canonical joint-state publisher contract failed: {publisher_error}',
            )
            return

        if self._last_valid_message_monotonic is None:
            self._fail('joint_state_stream_lost', 'No valid joint-state message is available')
            return

        age = now - self._last_valid_message_monotonic
        if age > STALE_TIMEOUT_SEC:
            self._fail(
                'joint_state_stream_stale',
                f'Actual joint-state stream is stale: age={age:.3f}s '
                f'limit={STALE_TIMEOUT_SEC:.1f}s',
            )


def main() -> None:
    rclpy.init()
    node = None
    exit_code = 1
    try:
        node = ActualJointStateMonitor()
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=CHECK_PERIOD_SEC)
        exit_code = node.exit_code if node.exit_code is not None else 0
    except KeyboardInterrupt:
        exit_code = 0
    except Exception as error:
        print(f'[DOBOT RVIZ] Actual joint-state monitor failed: {error}', file=sys.stderr)
        if node is not None:
            try:
                node.event_logger.record('ERROR', 'monitor_failed', str(error))
            except Exception:
                pass
        exit_code = 1
    finally:
        if node is not None:
            if exit_code == 0:
                try:
                    node.event_logger.record('INFO', 'viewer_shutdown', 'RViz viewer stopped')
                except Exception:
                    pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
