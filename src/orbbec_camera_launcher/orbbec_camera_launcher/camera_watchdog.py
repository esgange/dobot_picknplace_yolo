from __future__ import annotations

import ctypes
import json
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from .device_scan import scan_connected_serials
from .event_log import PackageEventLogger
from .project_config import FIXED_CAMERA_COUNT, ORBBEC_LAUNCH_FILE


PR_SET_PDEATHSIG = 1
_LIBC = ctypes.CDLL(None)
_REQUIRED_LAUNCH_ARGUMENTS = {
    'device_preset',
    'enable_color',
    'enable_depth',
    'depth_registration',
    'align_target_stream',
    'align_mode',
    'enable_frame_sync',
    'enable_temporal_filter',
    'color_width',
    'color_height',
    'color_fps',
    'depth_width',
    'depth_height',
    'depth_fps',
    'enable_point_cloud',
    'enumerate_net_device',
}


def _camera_child_preexec() -> None:
    if _LIBC.prctl(PR_SET_PDEATHSIG, int(signal.SIGTERM)) != 0:
        os._exit(126)
    if os.getppid() == 1:
        os._exit(126)


@dataclass
class CameraState:
    name: str
    serial_number: str
    process: subprocess.Popen | None = None
    process_group_id: int | None = None
    startup_started_at: float | None = None
    startup_color_at: float | None = None
    startup_depth_at: float | None = None
    ready_at: float | None = None
    last_color_at: float | None = None
    last_depth_at: float | None = None
    owned_endpoint_gids: dict[str, set[bytes]] = field(
        default_factory=lambda: {'color': set(), 'depth': set()}
    )
    endpoint_baseline_gids: dict[str, set[bytes]] = field(
        default_factory=lambda: {'color': set(), 'depth': set()}
    )


class CameraWatchdog(Node):
    def __init__(self, *, parameter_overrides=None, context=None) -> None:
        super().__init__(
            'supervisor',
            parameter_overrides=parameter_overrides,
            context=context,
        )
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('ROS_LOCALHOST_ONLY must be exactly 1')

        self.declare_parameter('camera_names', ['__REQUIRED__'])
        self.declare_parameter('serial_numbers', ['__REQUIRED__'])
        self.declare_parameter('orbbec_launch_file', '__REQUIRED__')
        self.declare_parameter('device_num', 0)
        self.declare_parameter('launch_args_json', '__REQUIRED__')
        self.declare_parameter('workspace_root', '__REQUIRED__')
        self.declare_parameter('scan_timeout_sec', 0.0)
        self.declare_parameter('startup_timeout_sec', 0.0)
        self.declare_parameter('health_timeout_sec', 0.0)
        self.declare_parameter('check_period_sec', 0.0)
        self.declare_parameter('max_attempts', 0)
        self.declare_parameter('retry_delay_sec', 0.0)
        self.declare_parameter('shutdown_timeout_sec', 0.0)

        camera_names = self._required_string_list('camera_names')
        serial_numbers = self._required_string_list('serial_numbers')
        if len(camera_names) != len(serial_numbers):
            raise RuntimeError('camera_names and serial_numbers must have matching lengths')
        if len(camera_names) != FIXED_CAMERA_COUNT:
            raise RuntimeError(
                f'Exactly {FIXED_CAMERA_COUNT} configured Orbbec cameras are required'
            )
        if len(set(camera_names)) != len(camera_names):
            raise RuntimeError('Configured camera names must be unique')
        if len(set(serial_numbers)) != len(serial_numbers):
            raise RuntimeError('Configured camera serial numbers must be unique')

        self._orbbec_launch_file = self._required_text('orbbec_launch_file')
        if self._orbbec_launch_file != ORBBEC_LAUNCH_FILE:
            raise RuntimeError(
                f'orbbec_launch_file must be exactly {ORBBEC_LAUNCH_FILE}'
            )
        self._device_num = self._required_integer('device_num')
        if self._device_num != FIXED_CAMERA_COUNT:
            raise RuntimeError(f'device_num must be exactly {FIXED_CAMERA_COUNT}')

        self._workspace_root = Path(self._required_text('workspace_root')).resolve()
        if not (
            (self._workspace_root / '.env.example').is_file()
            and (self._workspace_root / 'src').is_dir()
        ):
            raise RuntimeError('workspace_root is not the PicknPlace repository root')

        self._scan_timeout_sec = self._required_positive_number('scan_timeout_sec')
        self._startup_timeout_sec = self._required_positive_number('startup_timeout_sec')
        self._health_timeout_sec = self._required_positive_number('health_timeout_sec')
        self._check_period_sec = self._required_positive_number('check_period_sec')
        self._max_attempts = self._required_integer('max_attempts')
        self._retry_delay_sec = self._required_positive_number('retry_delay_sec')
        self._shutdown_timeout_sec = self._required_positive_number('shutdown_timeout_sec')
        if self._max_attempts != 3:
            raise RuntimeError('max_attempts must be exactly 3')
        if self._startup_timeout_sec != 5.0:
            raise RuntimeError('startup_timeout_sec must be exactly 5 seconds')
        if self._retry_delay_sec != 3.0:
            raise RuntimeError('retry_delay_sec must be exactly 3 seconds')

        self._launch_args = self._required_launch_arguments()
        if self._launch_args['enumerate_net_device'] != 'false':
            raise RuntimeError('enumerate_net_device must be exactly false')
        if self._launch_args['enable_color'] != 'true':
            raise RuntimeError('enable_color must be exactly true')
        if self._launch_args['enable_depth'] != 'true':
            raise RuntimeError('enable_depth must be exactly true')

        self._event_logger = PackageEventLogger(self._workspace_root)
        self._states = {
            name: CameraState(name=name, serial_number=serial)
            for name, serial in zip(camera_names, serial_numbers)
        }
        self._camera_order = list(self._states)
        self._startup_index: int | None = None
        self._retired_owned_endpoint_gids: dict[str, set[bytes]] = {
            self._topic(state, stream): set()
            for state in self._states.values()
            for stream in ('color', 'depth')
        }
        self._attempt_count = 0
        self._launch_parent_pid = os.getppid()
        self._next_attempt_at = time.monotonic()
        self._phase = 'waiting'
        self._last_error = ''
        self._terminal_failure = False
        self._shutting_down = False

        self._control_group = MutuallyExclusiveCallbackGroup()
        self._subscription_group = ReentrantCallbackGroup()
        self._image_subscriptions = []
        for camera_name in camera_names:
            self._image_subscriptions.extend([
                self.create_subscription(
                    Image,
                    f'/{camera_name}/color/image_raw',
                    lambda _message, name=camera_name: self._record_image(name, 'color'),
                    qos_profile_sensor_data,
                    callback_group=self._subscription_group,
                ),
                self.create_subscription(
                    Image,
                    f'/{camera_name}/depth/image_raw',
                    lambda _message, name=camera_name: self._record_image(name, 'depth'),
                    qos_profile_sensor_data,
                    callback_group=self._subscription_group,
                ),
            ])

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(DiagnosticArray, 'status', status_qos)
        self._healthy_publisher = self.create_publisher(Bool, 'healthy', status_qos)
        self._timer = self.create_timer(
            self._check_period_sec,
            self._check_cameras,
            callback_group=self._control_group,
        )
        configured = ', '.join(
            f'{state.name}({state.serial_number})' for state in self._states.values()
        )
        self._event_logger.record(
            'INFO',
            'supervisor_started',
            f'configured={configured} max_attempts=3 retry_delay_sec=3 local_only=1',
            launch_parent_pid=self._launch_parent_pid,
        )
        self.get_logger().info(
            f'Supervising complete camera set: {configured}; '
            'strict slot-order startup, maximum three total attempts with '
            '3-second rests.'
        )

    @property
    def terminal_failure(self) -> bool:
        return self._terminal_failure

    def _required_text(self, name: str) -> str:
        value = str(self.get_parameter(name).value).strip()
        if not value or value == '__REQUIRED__':
            raise RuntimeError(f'Required parameter {name} is missing')
        return value

    def _required_string_list(self, name: str) -> list[str]:
        raw = self.get_parameter(name).value
        if not isinstance(raw, (list, tuple)):
            raise RuntimeError(f'Required parameter {name} must be a string list')
        values = [str(value).strip() for value in raw]
        if not values or any(not value or value == '__REQUIRED__' for value in values):
            raise RuntimeError(f'Required parameter {name} contains an empty value')
        return values

    def _required_integer(self, name: str) -> int:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            raise RuntimeError(f'Required parameter {name} must be an integer')
        try:
            parsed = int(value)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f'Required parameter {name} must be an integer') from exc
        if parsed <= 0 or str(parsed) != str(value):
            raise RuntimeError(f'Required parameter {name} must be a canonical positive integer')
        return parsed

    def _required_positive_number(self, name: str) -> float:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            raise RuntimeError(f'Required parameter {name} must be a positive number')
        try:
            parsed = float(value)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f'Required parameter {name} must be a positive number') from exc
        if not 0.0 < parsed < float('inf'):
            raise RuntimeError(f'Required parameter {name} must be a positive finite number')
        return parsed

    def _required_launch_arguments(self) -> dict[str, str]:
        raw = self._required_text('launch_args_json')
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f'launch_args_json is invalid: {exc}') from exc
        if not isinstance(payload, dict) or set(payload) != _REQUIRED_LAUNCH_ARGUMENTS:
            raise RuntimeError(
                'launch_args_json must contain the exact canonical Orbbec argument set'
            )
        values = {str(key): str(value) for key, value in payload.items()}
        if any(not value for value in values.values()):
            raise RuntimeError('launch_args_json cannot contain empty values')
        return values

    def _record_image(self, camera_name: str, stream: str) -> None:
        if self._phase not in {'starting', 'healthy'}:
            return
        state = self._states.get(camera_name)
        if state is None or state.process is None:
            return
        now = time.monotonic()
        if stream == 'color':
            if state.startup_color_at is None:
                state.startup_color_at = now
            state.last_color_at = now
        else:
            if state.startup_depth_at is None:
                state.startup_depth_at = now
            state.last_depth_at = now

    def _check_cameras(self) -> None:
        if self._shutting_down or self._terminal_failure:
            return
        now = time.monotonic()
        if self._phase == 'waiting':
            if now >= self._next_attempt_at:
                self._begin_attempt()
            if self._terminal_failure:
                return
            self._publish_status(time.monotonic())
            return

        if self._phase not in {'starting', 'healthy'}:
            self._publish_status(now)
            return

        try:
            collision = self._track_owned_endpoints()
        except RuntimeError as exc:
            self._fail_attempt(f'DDS endpoint inspection failed: {exc}')
            return
        if collision:
            self._fail_attempt(collision)
            return

        launched_states = [
            state for state in self._states.values() if state.process is not None
        ]
        for state in launched_states:
            return_code = state.process.poll()
            if return_code is not None:
                self._event_logger.record(
                    'ERROR',
                    'camera_runtime_failure',
                    f'attempt={self._attempt_count} camera={state.name} '
                    f'process_exit={return_code} phase={self._phase}',
                    attempt=self._attempt_count,
                    camera=state.name,
                    failure='process_exit',
                    return_code=return_code,
                    phase=self._phase,
                )
                self._fail_attempt(
                    f'{state.name} launch process exited with code {return_code}'
                )
                return

        if self._phase == 'starting':
            if self._startup_index is None:
                self._fail_attempt('Sequential startup index is unavailable')
                return

            for ready_name in self._camera_order[:self._startup_index]:
                ready_state = self._states[ready_name]
                stale = self._stale_streams(ready_state, now)
                if stale:
                    reason = (
                        f'{ready_state.name} became unhealthy while starting '
                        f'{self._camera_order[self._startup_index]}: '
                        + ', '.join(stale)
                    )
                    self._event_logger.record(
                        'ERROR',
                        'camera_runtime_failure',
                        reason,
                        attempt=self._attempt_count,
                        camera=ready_state.name,
                        failure='stale_stream',
                        phase='sequential_start',
                    )
                    self._fail_attempt(reason)
                    return

            current = self._states[self._camera_order[self._startup_index]]
            if current.process is None or current.startup_started_at is None:
                self._fail_attempt(
                    f'{current.name} has no owned sequential launch process'
                )
                return
            deadline = current.startup_started_at + self._startup_timeout_sec
            ready = (
                current.startup_color_at is not None
                and current.startup_depth_at is not None
                and current.startup_color_at <= deadline
                and current.startup_depth_at <= deadline
            )
            if ready:
                current.ready_at = now
                self._event_logger.record(
                    'INFO',
                    'camera_streams_ready',
                    f'attempt={self._attempt_count} camera={current.name} '
                    'color/depth ready',
                    attempt=self._attempt_count,
                    camera=current.name,
                    startup_order=self._startup_index + 1,
                    startup_elapsed_sec=round(now - current.startup_started_at, 3),
                )
                if self._startup_index + 1 < len(self._camera_order):
                    previous_name = current.name
                    self._startup_index += 1
                    next_state = self._states[self._camera_order[self._startup_index]]
                    self._event_logger.record(
                        'INFO',
                        'next_camera_starting',
                        f'attempt={self._attempt_count} {previous_name} ready; '
                        f'launching {next_state.name}',
                        attempt=self._attempt_count,
                        ready_camera=previous_name,
                        next_camera=next_state.name,
                        startup_order=self._startup_index + 1,
                    )
                    if not self._start_camera(next_state, self._startup_index):
                        return
                else:
                    self._phase = 'healthy'
                    self._last_error = ''
                    self._event_logger.record(
                        'INFO',
                        'camera_set_healthy',
                        f'attempt={self._attempt_count} all color/depth streams ready',
                        attempt=self._attempt_count,
                        cameras=list(self._camera_order),
                    )
                    self.get_logger().info(
                        f'Complete camera set is healthy on attempt {self._attempt_count}.'
                    )
            elif now >= deadline:
                missing = self._missing_streams(current, deadline)
                reason = (
                    f'{current.name} startup timeout after '
                    f'{self._startup_timeout_sec:g} seconds waiting for: '
                    + ', '.join(missing)
                )
                self._event_logger.record(
                    'ERROR',
                    'camera_startup_timeout',
                    reason,
                    attempt=self._attempt_count,
                    camera=current.name,
                    startup_order=self._startup_index + 1,
                    timeout_sec=self._startup_timeout_sec,
                    missing_streams=missing,
                )
                self._fail_attempt(reason)
                return
        else:
            stale = []
            for state in self._states.values():
                stale.extend(self._stale_streams(state, now))
            if stale:
                reason = 'Stale stream(s): ' + ', '.join(stale)
                self._event_logger.record(
                    'ERROR',
                    'camera_runtime_failure',
                    reason,
                    attempt=self._attempt_count,
                    failure='stale_stream',
                    phase='healthy',
                    stale_streams=stale,
                )
                self._fail_attempt(reason)
                return
        self._publish_status(time.monotonic())

    def _begin_attempt(self) -> None:
        self._attempt_count += 1
        self._phase = 'scanning'
        self._last_error = ''
        self._event_logger.record(
            'INFO',
            'connection_attempt_started',
            f'attempt={self._attempt_count}/3 scanning exact configured serials',
            attempt=self._attempt_count,
            maximum_attempts=self._max_attempts,
        )
        self.get_logger().info(
            f'Orbbec connection attempt {self._attempt_count}/{self._max_attempts}.'
        )

        return_code, detected, output = scan_connected_serials(self._scan_timeout_sec)
        configured = {state.serial_number for state in self._states.values()}
        missing = sorted(configured - set(detected))
        self._event_logger.record(
            'INFO' if return_code == 0 and not missing else 'ERROR',
            'device_scan_result',
            f'attempt={self._attempt_count} exit_code={return_code} '
            f'detected={detected} missing={missing}',
            attempt=self._attempt_count,
            exit_code=return_code,
            detected_serials=detected,
            missing_serials=missing,
            scan_output=output,
        )
        if return_code != 0:
            self._fail_attempt(f'Device scan exited with code {return_code}: {output}')
            return
        if missing:
            self._fail_attempt('Configured camera serial(s) not detected: ' + ', '.join(missing))
            return

        topic_collisions = []
        retired_filtered: dict[str, list[str]] = {}
        try:
            for state in self._states.values():
                for stream in ('color', 'depth'):
                    topic = self._topic(state, stream)
                    unknown = []
                    ignored = []
                    for endpoint in self.get_publishers_info_by_topic(topic):
                        gid = self._endpoint_gid(endpoint)
                        if gid in self._retired_owned_endpoint_gids[topic]:
                            ignored.append(gid.hex())
                        else:
                            unknown.append(gid.hex())
                    if ignored:
                        retired_filtered[topic] = ignored
                    if unknown:
                        topic_collisions.append(f'{topic} gids={unknown}')
        except RuntimeError as exc:
            self._fail_attempt(f'DDS endpoint inspection failed: {exc}')
            return
        if retired_filtered:
            self._event_logger.record(
                'INFO',
                'retired_owned_endpoints_ignored',
                f'attempt={self._attempt_count} ignored exact retired endpoint GIDs',
                attempt=self._attempt_count,
                endpoints=retired_filtered,
            )
        if topic_collisions:
            self._fail_attempt(
                'Required topic(s) already have unowned publishers: '
                + ', '.join(topic_collisions)
            )
            return

        for state in self._states.values():
            self._clear_runtime_state(state)
        self._startup_index = 0
        self._phase = 'starting'
        self._start_camera(self._states[self._camera_order[0]], 0)

    def _start_camera(self, state: CameraState, startup_index: int) -> bool:
        try:
            for stream in ('color', 'depth'):
                topic = self._topic(state, stream)
                baseline = set()
                unknown = []
                for endpoint in self.get_publishers_info_by_topic(topic):
                    gid = self._endpoint_gid(endpoint)
                    baseline.add(gid)
                    if gid not in self._retired_owned_endpoint_gids[topic]:
                        unknown.append(gid.hex())
                if unknown:
                    self._fail_attempt(
                        f'Required topic already has unowned publishers before '
                        f'{state.name} launch: {topic} gids={unknown}'
                    )
                    return False
                state.endpoint_baseline_gids[stream] = baseline
        except RuntimeError as exc:
            self._fail_attempt(f'DDS endpoint inspection failed: {exc}')
            return False
        command = [
            'ros2',
            'launch',
            'orbbec_camera',
            self._orbbec_launch_file,
            f'camera_name:={state.name}',
            f'serial_number:={state.serial_number}',
            f'device_num:={self._device_num}',
            *[f'{key}:={value}' for key, value in self._launch_args.items()],
        ]
        try:
            process = subprocess.Popen(
                command,
                cwd=str(self._workspace_root),
                env=os.environ.copy(),
                start_new_session=True,
                preexec_fn=_camera_child_preexec,
            )
        except OSError as exc:
            self._fail_attempt(f'Failed to start {state.name}: {exc}')
            return False
        state.process = process
        state.process_group_id = process.pid
        state.startup_started_at = time.monotonic()
        self._event_logger.record(
            'INFO',
            'camera_process_started',
            f'attempt={self._attempt_count} camera={state.name} '
            f'serial={state.serial_number} pid={process.pid} '
            f'startup_order={startup_index + 1}',
            attempt=self._attempt_count,
            camera=state.name,
            serial_number=state.serial_number,
            pid=process.pid,
            startup_order=startup_index + 1,
            startup_timeout_sec=self._startup_timeout_sec,
            device_num=self._device_num,
        )
        self.get_logger().info(
            f'Started camera {startup_index + 1}/{len(self._camera_order)} '
            f'{state.name}; waiting for its color/depth streams.'
        )
        return True

    def _missing_streams(
        self,
        state: CameraState,
        deadline: float | None = None,
    ) -> list[str]:
        missing = []
        if state.startup_color_at is None or (
            deadline is not None and state.startup_color_at > deadline
        ):
            missing.append(f'{state.name}/color')
        if state.startup_depth_at is None or (
            deadline is not None and state.startup_depth_at > deadline
        ):
            missing.append(f'{state.name}/depth')
        return missing

    def _stale_streams(self, state: CameraState, now: float) -> list[str]:
        stale = []
        for stream, timestamp in (
            ('color', state.last_color_at),
            ('depth', state.last_depth_at),
        ):
            age = float('inf') if timestamp is None else max(0.0, now - timestamp)
            if age > self._health_timeout_sec:
                age_text = 'never' if age == float('inf') else f'{age:.2f}s'
                stale.append(f'{state.name}/{stream} age={age_text}')
        return stale

    @staticmethod
    def _topic(state: CameraState, stream: str) -> str:
        return f'/{state.name}/{stream}/image_raw'

    @staticmethod
    def _endpoint_gid(endpoint) -> bytes:
        try:
            gid = bytes(endpoint.endpoint_gid)
        except (AttributeError, TypeError, ValueError) as exc:
            raise RuntimeError('DDS publisher endpoint has an invalid GID') from exc
        if not gid:
            raise RuntimeError('DDS publisher endpoint has an empty GID')
        return gid

    @staticmethod
    def _endpoint_matches_camera(endpoint, state: CameraState) -> bool:
        namespace = str(getattr(endpoint, 'node_namespace', '')).rstrip('/') or '/'
        return (
            str(getattr(endpoint, 'node_name', '')) == state.name
            and namespace == f'/{state.name}'
        )

    def _track_owned_endpoints(self) -> str | None:
        unknown: list[str] = []
        for state in self._states.values():
            if state.process is None:
                continue
            for stream in ('color', 'depth'):
                topic = self._topic(state, stream)
                for endpoint in self.get_publishers_info_by_topic(topic):
                    gid = self._endpoint_gid(endpoint)
                    if gid in self._retired_owned_endpoint_gids[topic]:
                        continue
                    if gid in state.owned_endpoint_gids[stream]:
                        continue
                    if (
                        gid not in state.endpoint_baseline_gids[stream]
                        and self._endpoint_matches_camera(endpoint, state)
                    ):
                        state.owned_endpoint_gids[stream].add(gid)
                    else:
                        unknown.append(f'{topic} gid={gid.hex()}')
        if unknown:
            return 'Unknown publisher endpoint collision(s): ' + ', '.join(unknown)
        return None

    @staticmethod
    def _clear_runtime_state(state: CameraState) -> None:
        state.process = None
        state.process_group_id = None
        state.startup_started_at = None
        state.startup_color_at = None
        state.startup_depth_at = None
        state.ready_at = None
        state.last_color_at = None
        state.last_depth_at = None
        state.owned_endpoint_gids = {'color': set(), 'depth': set()}
        state.endpoint_baseline_gids = {'color': set(), 'depth': set()}

    def _fail_attempt(self, reason: str) -> None:
        attempt = self._attempt_count
        self._last_error = reason
        self._phase = 'stopping'
        self._event_logger.record(
            'ERROR',
            'connection_attempt_failed',
            f'attempt={attempt}/3 {reason}',
            attempt=attempt,
            maximum_attempts=self._max_attempts,
        )
        self.get_logger().error(f'Orbbec attempt {attempt}/{self._max_attempts} failed: {reason}')
        owned_set_stopped = self._stop_owned_processes()

        if not owned_set_stopped:
            self._last_error = reason + '; owned camera set did not stop completely'
            self._phase = 'failed'
            self._terminal_failure = True
            self._event_logger.record(
                'ERROR',
                'launcher_failed',
                self._last_error,
                attempt=attempt,
                maximum_attempts=self._max_attempts,
                failure='owned_set_shutdown_incomplete',
            )
            self._publish_status(time.monotonic())
            self.get_logger().fatal(
                'Owned camera shutdown did not complete; retry is unsafe and '
                'the launcher is terminating.'
            )
            self.context.try_shutdown()
            return

        if attempt >= self._max_attempts:
            self._phase = 'failed'
            self._terminal_failure = True
            self._event_logger.record(
                'ERROR',
                'launcher_failed',
                f'all {self._max_attempts} attempts exhausted: {reason}',
                attempt=attempt,
                maximum_attempts=self._max_attempts,
            )
            self._publish_status(time.monotonic())
            self.get_logger().fatal('All three Orbbec attempts failed; terminating launcher.')
            self.context.try_shutdown()
            return

        self._phase = 'waiting'
        self._next_attempt_at = time.monotonic() + self._retry_delay_sec
        self._event_logger.record(
            'WARNING',
            'complete_set_recovery_scheduled',
            f'next_attempt={attempt + 1}/3 rest_sec={self._retry_delay_sec:g}',
            completed_attempt=attempt,
            next_attempt=attempt + 1,
            rest_sec=self._retry_delay_sec,
            restart_order=list(self._camera_order),
        )
        self.get_logger().warning(
            f'Resting {self._retry_delay_sec:g} seconds before attempt {attempt + 1}/3.'
        )

    @staticmethod
    def _process_group_alive(process_group_id: int | None) -> bool:
        if process_group_id is None:
            return False
        try:
            os.killpg(process_group_id, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def _state_stopped(self, state: CameraState) -> bool:
        process_done = state.process is None or state.process.poll() is not None
        return process_done and not self._process_group_alive(state.process_group_id)

    def _stop_owned_processes(self) -> bool:
        owned = [state for state in self._states.values() if state.process is not None]
        try:
            self._track_owned_endpoints()
        except RuntimeError as exc:
            self._event_logger.record(
                'ERROR',
                'endpoint_retirement_failed',
                f'owned process shutdown continues after endpoint error: {exc}',
                attempt=self._attempt_count,
            )
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            active = [state for state in owned if not self._state_stopped(state)]
            if not active:
                break
            for state in active:
                try:
                    if self._process_group_alive(state.process_group_id):
                        os.killpg(int(state.process_group_id), sig)
                    elif state.process is not None and state.process.poll() is None:
                        state.process.send_signal(sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + self._shutdown_timeout_sec
            while time.monotonic() < deadline:
                if all(self._state_stopped(state) for state in active):
                    break
                time.sleep(0.05)

        all_stopped = True
        for state in owned:
            return_code = state.process.poll() if state.process is not None else None
            stopped = self._state_stopped(state)
            all_stopped = all_stopped and stopped
            self._event_logger.record(
                'INFO' if stopped else 'ERROR',
                'camera_process_stopped',
                f'camera={state.name} stopped={stopped} return_code={return_code}',
                camera=state.name,
                stopped=stopped,
                return_code=return_code,
            )
            for stream in ('color', 'depth'):
                topic = self._topic(state, stream)
                self._retired_owned_endpoint_gids[topic].update(
                    state.owned_endpoint_gids[stream]
                )
        if all_stopped:
            for state in self._states.values():
                self._clear_runtime_state(state)
            self._startup_index = None
        return all_stopped

    def _publish_status(self, now: float) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        for state in self._states.values():
            status = DiagnosticStatus()
            status.name = f'orbbec_camera_launcher/{state.name}'
            status.hardware_id = state.serial_number
            if self._phase == 'healthy':
                status.level = DiagnosticStatus.OK
                status.message = 'Camera streams healthy'
            elif self._phase == 'failed':
                status.level = DiagnosticStatus.ERROR
                status.message = self._last_error
            else:
                status.level = DiagnosticStatus.WARN
                status.message = self._phase
            status.values = [
                KeyValue(key='phase', value=self._phase),
                KeyValue(key='camera_phase', value=self._camera_phase(state)),
                KeyValue(key='attempt', value=str(self._attempt_count)),
                KeyValue(key='maximum_attempts', value=str(self._max_attempts)),
                KeyValue(key='serial_number', value=state.serial_number),
                KeyValue(
                    key='color_age_sec',
                    value=self._age_text(now, state.last_color_at),
                ),
                KeyValue(
                    key='depth_age_sec',
                    value=self._age_text(now, state.last_depth_at),
                ),
                KeyValue(
                    key='pid',
                    value=str(state.process.pid) if state.process is not None else '',
                ),
                KeyValue(key='last_error', value=self._last_error),
            ]
            message.status.append(status)
        self._status_publisher.publish(message)
        self._healthy_publisher.publish(Bool(data=self._phase == 'healthy'))

    def _camera_phase(self, state: CameraState) -> str:
        if self._phase == 'failed':
            return 'failed'
        if self._phase == 'healthy':
            return 'healthy'
        if state.process is None:
            return 'pending'
        if state.ready_at is not None:
            return 'ready'
        return 'starting'

    @staticmethod
    def _age_text(now: float, timestamp: float | None) -> str:
        return 'never' if timestamp is None else f'{max(0.0, now - timestamp):.2f}'

    def shutdown(self) -> None:
        if self._shutting_down:
            return
        self._shutting_down = True
        owned_set_stopped = self._stop_owned_processes()
        if not owned_set_stopped:
            self._terminal_failure = True
            self._last_error = 'Owned camera set did not stop completely during shutdown'
        self._phase = 'stopped' if not self._terminal_failure else 'failed'
        self._event_logger.record(
            'INFO' if not self._terminal_failure else 'ERROR',
            'supervisor_stopped',
            f'terminal_failure={self._terminal_failure} attempts={self._attempt_count}',
            terminal_failure=self._terminal_failure,
            attempts=self._attempt_count,
            owned_set_stopped=owned_set_stopped,
            launch_parent_pid=self._launch_parent_pid,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node: CameraWatchdog | None = None
    executor: MultiThreadedExecutor | None = None
    terminal_failure = False
    try:
        node = CameraWatchdog()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            terminal_failure = node.terminal_failure
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if terminal_failure:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
