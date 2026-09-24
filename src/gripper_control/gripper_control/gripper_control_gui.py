import json
import os
import queue
import sys
import threading
import time
import tkinter as tk
from datetime import datetime, timezone
from pathlib import Path
from tkinter import ttk

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from dobot_msgs_v4.msg import RobotStatus
from dobot_msgs_v4.srv import DO
from std_msgs.msg import String


PACKAGE_NAME = 'gripper_control'
DO_SERVICE_NAME = '/dobot_bringup_ros2/srv/DO'
ROBOT_STATUS_TOPIC = '/dobot_msgs_v4/msg/RobotStatus'
FEED_INFO_TOPIC = '/dobot_bringup_ros2/msg/FeedInfo'
STARTUP_TIMEOUT_SEC = 5.0
SERVICE_RESPONSE_TIMEOUT_SEC = 3.0
OUTPUT_CONFIRM_TIMEOUT_SEC = 1.5
FEEDBACK_STALE_SEC = 1.5
MAX_LOG_EVENTS = 1000
SUCTION_EXHAUST_DO = 1
GRIPPER_CLOSE_DO = 2
FINGER_CLOSE_DO = 13
GRIPPER_OPEN_DO = 14
OUTPUT_CHANNELS = (
    SUCTION_EXHAUST_DO,
    GRIPPER_CLOSE_DO,
    FINGER_CLOSE_DO,
    GRIPPER_OPEN_DO,
)
DI_CHANNELS = (1, 2)
SHUTDOWN_TIMEOUT_MS = 5000


def workspace_root() -> Path:
    for start in (Path.cwd(), Path(__file__).resolve()):
        path = start if start.is_dir() else start.parent
        for candidate in (path, *path.parents):
            if (candidate / '.env.example').is_file() and (candidate / 'src').is_dir():
                return candidate
    raise RuntimeError('Cannot locate workspace root containing .env.example and src/')


class PackageEventLogger:
    def __init__(self) -> None:
        log_dir = workspace_root() / 'logs' / PACKAGE_NAME
        log_dir.mkdir(parents=True, exist_ok=True)
        self.path = log_dir / 'events.jsonl'
        self.path.touch(exist_ok=True)
        self._lock = threading.Lock()
        self._started = time.monotonic()

    def record(self, level: str, event: str, message: str) -> None:
        timestamp = (
            datetime.now(timezone.utc)
            .isoformat(timespec='milliseconds')
            .replace('+00:00', 'Z')
        )
        record = {
            'timestamp': timestamp,
            'package': PACKAGE_NAME,
            'level': level,
            'event': event,
            'message': message,
            'elapsed_sec': round(time.monotonic() - self._started, 3),
        }
        with self._lock:
            with self.path.open('r', encoding='utf-8') as existing:
                event_count = sum(1 for _ in existing)
            mode = 'w' if event_count >= MAX_LOG_EVENTS else 'a'
            with self.path.open(mode, encoding='utf-8') as log_file:
                log_file.write(json.dumps(record, separators=(',', ':')) + '\n')


class GripperControlNode(Node):
    def __init__(self, event_logger: PackageEventLogger) -> None:
        super().__init__('gripper_control_gui')
        self._event_logger = event_logger
        self._feedback_lock = threading.Lock()
        self._robot_status_received = False
        self._feed_info_received = False
        self._robot_connected = False
        self._robot_enabled = False
        self._robot_status_monotonic: float | None = None
        self._feed_info_monotonic: float | None = None
        self._digital_input_bits = 0
        self._digital_output_bits = 0

        self._do_client = self.create_client(DO, DO_SERVICE_NAME)
        self.create_subscription(
            RobotStatus,
            ROBOT_STATUS_TOPIC,
            self._robot_status_callback,
            10,
        )
        self.create_subscription(String, FEED_INFO_TOPIC, self._feed_info_callback, 10)
        self._event_logger.record(
            'INFO',
            'startup',
            f'created canonical interfaces service={DO_SERVICE_NAME} '
            f'robot_status={ROBOT_STATUS_TOPIC} feed_info={FEED_INFO_TOPIC}',
        )

    @property
    def event_logger(self) -> PackageEventLogger:
        return self._event_logger

    def _robot_status_callback(self, msg: RobotStatus) -> None:
        now = time.monotonic()
        with self._feedback_lock:
            previous_connected = self._robot_connected if self._robot_status_received else None
            self._robot_status_received = True
            self._robot_connected = bool(msg.is_connected)
            self._robot_enabled = bool(msg.is_enable)
            self._robot_status_monotonic = now
        if previous_connected is None or previous_connected != bool(msg.is_connected):
            self._event_logger.record(
                'INFO' if msg.is_connected else 'ERROR',
                'robot_connection_state',
                f'connected={bool(msg.is_connected)} enabled={bool(msg.is_enable)}',
            )

    def _feed_info_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('payload must be a JSON object')
            input_bits = payload['digital_input_bits']
            output_bits = payload['digital_outputs']
            if (
                isinstance(input_bits, bool)
                or not isinstance(input_bits, int)
                or input_bits < 0
                or isinstance(output_bits, bool)
                or not isinstance(output_bits, int)
                or output_bits < 0
            ):
                raise ValueError(
                    'digital_input_bits and digital_outputs must be non-negative integers'
                )
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self._event_logger.record('ERROR', 'feed_info_invalid', str(exc))
            self.get_logger().error(f'Invalid canonical FeedInfo payload: {exc}')
            return

        now = time.monotonic()
        with self._feedback_lock:
            previous = (
                (self._digital_input_bits, self._digital_output_bits)
                if self._feed_info_received
                else None
            )
            self._feed_info_received = True
            self._feed_info_monotonic = now
            self._digital_input_bits = input_bits
            self._digital_output_bits = output_bits
        current = (input_bits, output_bits)
        if previous != current:
            self._event_logger.record(
                'INFO',
                'io_feedback',
                f'digital_input_bits={input_bits} digital_outputs={output_bits}',
            )

    def startup_missing_requirements(self) -> list[str]:
        missing = []
        if not self._do_client.service_is_ready():
            missing.append(f'DO service {DO_SERVICE_NAME}')
        with self._feedback_lock:
            if not self._robot_status_received:
                missing.append(f'RobotStatus {ROBOT_STATUS_TOPIC}')
            elif not self._robot_connected:
                missing.append('connected robot state')
            if not self._feed_info_received:
                missing.append(f'FeedInfo {FEED_INFO_TOPIC}')
        return missing

    def io_snapshot(self) -> dict[str, object]:
        now = time.monotonic()
        with self._feedback_lock:
            status_fresh = (
                self._robot_status_monotonic is not None
                and now - self._robot_status_monotonic <= FEEDBACK_STALE_SEC
            )
            feed_fresh = (
                self._feed_info_monotonic is not None
                and now - self._feed_info_monotonic <= FEEDBACK_STALE_SEC
            )
            connected = self._robot_connected
            enabled = self._robot_enabled
            input_bits = self._digital_input_bits
            output_bits = self._digital_output_bits
        service_ready = self._do_client.service_is_ready()
        ready = service_ready and status_fresh and feed_fresh and connected
        missing = []
        if not service_ready:
            missing.append('DO service unavailable')
        if not status_fresh:
            missing.append('RobotStatus stale')
        elif not connected:
            missing.append('robot disconnected')
        if not feed_fresh:
            missing.append('FeedInfo stale')
        return {
            'ready': ready,
            'health_text': 'ready' if ready else ', '.join(missing),
            'connected': connected,
            'enabled': enabled,
            'digital_input_bits': input_bits,
            'digital_output_bits': output_bits,
        }

    def output_is_active(self, index: int) -> bool:
        snapshot = self.io_snapshot()
        return bool(int(snapshot['digital_output_bits']) & (1 << (index - 1)))

    def send_do(self, index: int, status: int, on_complete) -> None:
        if index not in OUTPUT_CHANNELS:
            on_complete(False, -1, f'invalid DO index {index}')
            return
        if status not in {0, 1}:
            on_complete(False, -1, f'invalid DO status {status}')
            return
        if not self._do_client.service_is_ready():
            detail = f'service unavailable: {DO_SERVICE_NAME}'
            self._event_logger.record('ERROR', 'do_result', detail)
            on_complete(False, -1, detail)
            return

        request = DO.Request()
        request.index = index
        request.status = status
        request.time = 0
        call_text = f'DO(index={index},status={status},time=0)'
        self._event_logger.record('INFO', 'do_dispatch', call_text)
        future = self._do_client.call_async(request)
        completion_lock = threading.Lock()
        completion = {'done': False}

        def _complete_once(success: bool, result_code: int, detail: str) -> None:
            with completion_lock:
                if completion['done']:
                    return
                completion['done'] = True
            self._event_logger.record(
                'INFO' if success else 'ERROR',
                'do_result',
                f'{call_text} {detail}',
            )
            on_complete(success, result_code, detail)

        def _timeout() -> None:
            _complete_once(
                False,
                -1,
                f'no response within {SERVICE_RESPONSE_TIMEOUT_SEC:.1f} seconds',
            )

        response_timer = threading.Timer(SERVICE_RESPONSE_TIMEOUT_SEC, _timeout)
        response_timer.daemon = True
        response_timer.start()

        def _done(done_future) -> None:
            try:
                response = done_future.result()
                if response is None:
                    raise RuntimeError('empty response')
                result_code = int(response.res)
            except Exception as exc:  # pragma: no cover - defensive ROS path
                response_timer.cancel()
                _complete_once(False, -1, f'call failed: {exc}')
                return
            response_timer.cancel()
            _complete_once(
                result_code != -1,
                result_code,
                f'res={result_code}',
            )

        future.add_done_callback(_done)


class GripperControlApp:
    def __init__(
        self,
        root: tk.Tk,
        node: GripperControlNode,
        executor: SingleThreadedExecutor,
        spin_thread: threading.Thread,
        stop_event: threading.Event,
    ) -> None:
        self._root = root
        self._node = node
        self._executor = executor
        self._spin_thread = spin_thread
        self._stop_event = stop_event
        self._ui_queue: queue.Queue = queue.Queue()
        self._closing = False
        self._finalized = False
        self._operation_name: str | None = None
        self._operation_token = 0
        self._last_runtime_ready: bool | None = None
        self._last_input_bits: int | None = None
        self._last_output_bits: int | None = None
        self._channels: dict[int, dict[str, object]] = {}
        self._digital_input_status_vars: dict[int, tk.StringVar] = {}
        self._digital_input_detail_vars: dict[int, tk.StringVar] = {}
        self._digital_input_led_canvases: dict[int, tk.Canvas] = {}
        self._digital_input_led_items: dict[int, int] = {}
        self._digital_input_status_labels: dict[int, tk.Label] = {}

        self._status_var = tk.StringVar(value='Ready')
        self._service_var = tk.StringVar(value='Bringup: ready')

        self._build_ui()
        self._root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._root.after(20, self._drain_ui_queue)
        self._root.after(50, self._refresh_feedback)

    def _build_ui(self) -> None:
        self._root.title('Dobot Gripper Diagnostics')
        self._root.minsize(640, 0)
        self._root.resizable(False, False)

        outer = ttk.Frame(self._root, padding=14)
        outer.pack(fill='both', expand=True)

        ttk.Label(
            outer,
            text='Dobot Gripper Diagnostics',
            font=('TkDefaultFont', 12, 'bold'),
        ).grid(row=0, column=0, sticky='w')
        ttk.Label(outer, textvariable=self._service_var).grid(
            row=1, column=0, sticky='w', pady=(4, 10)
        )

        output_panel = ttk.LabelFrame(outer, text='Outputs', padding=10)
        output_panel.grid(row=2, column=0, sticky='ew')
        for row, do_index in enumerate(OUTPUT_CHANNELS):
            self._create_channel_row(output_panel, do_index, row)

        input_panel = ttk.LabelFrame(outer, text='Digital Inputs', padding=10)
        input_panel.grid(row=3, column=0, sticky='ew', pady=(10, 0))
        input_panel.columnconfigure(2, weight=1)
        for row, di_index in enumerate(DI_CHANNELS):
            self._create_digital_input_row(input_panel, di_index, row)

        ttk.Label(outer, textvariable=self._status_var, foreground='#204a87').grid(
            row=4, column=0, sticky='w', pady=(10, 0)
        )
        ttk.Label(
            outer,
            text='0 ms keeps an output ON; a positive value schedules an explicit OFF call.',
        ).grid(row=5, column=0, sticky='w', pady=(6, 0))

    def _create_channel_row(self, parent: ttk.LabelFrame, do_index: int, row: int) -> None:
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=0, sticky='ew', pady=4)
        button = tk.Button(
            frame,
            text=f'DO{do_index} OFF',
            width=13,
            bg='#8b1a1a',
            fg='white',
            activebackground='#b22222',
            command=lambda index=do_index: self._on_toggle(index),
        )
        button.grid(row=0, column=0, padx=(0, 12))
        ttk.Label(frame, text='Auto OFF (ms)').grid(row=0, column=1, padx=(0, 6))
        duration_var = tk.StringVar(value='0')
        duration_entry = ttk.Entry(frame, textvariable=duration_var, width=10)
        duration_entry.grid(row=0, column=2, padx=(0, 10))
        state_label = tk.Label(frame, text='OFF', width=12, fg='#8b1a1a')
        state_label.grid(row=0, column=3, sticky='w')
        self._channels[do_index] = {
            'button': button,
            'duration_var': duration_var,
            'duration_entry': duration_entry,
            'state_label': state_label,
            'on': False,
            'pending': False,
            'timer_after_id': None,
        }

    def _create_digital_input_row(
        self, parent: ttk.LabelFrame, di_index: int, row: int
    ) -> None:
        led_canvas = tk.Canvas(
            parent, width=18, height=18, highlightthickness=0, bd=0
        )
        led_canvas.grid(row=row, column=0, sticky='w', padx=(0, 8), pady=3)
        led_item = led_canvas.create_oval(
            3, 3, 15, 15, fill='#9a9a9a', outline='#666666', width=1
        )
        status_var = tk.StringVar(value=f'DI{di_index}: UNKNOWN')
        status_label = tk.Label(
            parent,
            textvariable=status_var,
            width=14,
            fg='#8a6d1d',
        )
        status_label.grid(row=row, column=1, sticky='w', padx=(0, 12))
        detail_var = tk.StringVar(value=f'DI{di_index} bit {di_index - 1}: waiting')
        ttk.Label(parent, textvariable=detail_var).grid(
            row=row, column=2, sticky='w', pady=3
        )
        self._digital_input_status_vars[di_index] = status_var
        self._digital_input_detail_vars[di_index] = detail_var
        self._digital_input_led_canvases[di_index] = led_canvas
        self._digital_input_led_items[di_index] = led_item
        self._digital_input_status_labels[di_index] = status_label

    def _enqueue_ui(self, callback) -> None:
        self._ui_queue.put(callback)

    def _drain_ui_queue(self) -> None:
        if self._finalized:
            return
        while True:
            try:
                callback = self._ui_queue.get_nowait()
            except queue.Empty:
                break
            callback()
        self._root.after(20, self._drain_ui_queue)

    def _refresh_feedback(self) -> None:
        if self._finalized:
            return
        snapshot = self._node.io_snapshot()
        ready = bool(snapshot['ready'])
        input_bits = int(snapshot['digital_input_bits'])
        output_bits = int(snapshot['digital_output_bits'])
        self._service_var.set(
            f'Bringup: {snapshot["health_text"]}\n'
            f'DO: {DO_SERVICE_NAME} | Feedback: {FEED_INFO_TOPIC}'
        )

        if ready != self._last_runtime_ready:
            self._node.event_logger.record(
                'INFO' if ready else 'ERROR',
                'runtime_health',
                str(snapshot['health_text']),
            )
            if not ready and not self._closing:
                self._status_var.set(f'Controls disabled: {snapshot["health_text"]}')
            self._last_runtime_ready = ready

        if output_bits != self._last_output_bits:
            self._last_output_bits = output_bits
            for index, channel in self._channels.items():
                channel['on'] = bool(output_bits & (1 << (index - 1)))
                self._set_channel_ui(index)
        if input_bits != self._last_input_bits:
            self._last_input_bits = input_bits
            self._set_digital_input_status(input_bits)
        self._update_control_states()
        self._root.after(100, self._refresh_feedback)

    def _set_digital_input_status(self, digital_input_bits: int) -> None:
        for di_index in DI_CHANNELS:
            bit_index = di_index - 1
            active = bool(digital_input_bits & (1 << bit_index))
            fill, outline, text, foreground = (
                ('#20d060', '#0f8a3f', 'HIGH', '#1f7a1f')
                if active
                else ('#303030', '#777777', 'LOW', '#555555')
            )
            self._digital_input_status_vars[di_index].set(
                f'DI{di_index}: {text}'
            )
            self._digital_input_detail_vars[di_index].set(
                f'bit {bit_index}, digital_input_bits={digital_input_bits}'
            )
            self._digital_input_led_canvases[di_index].itemconfigure(
                self._digital_input_led_items[di_index], fill=fill, outline=outline
            )
            self._digital_input_status_labels[di_index].configure(fg=foreground)

    def _set_channel_ui(self, do_index: int) -> None:
        channel = self._channels[do_index]
        active = bool(channel['on'])
        pending = bool(channel['pending'])
        button = channel['button']
        state_label = channel['state_label']
        if active:
            button.configure(
                text=f'DO{do_index} ON', bg='#1f7a1f', activebackground='#2e8b57'
            )
            state_label.configure(text='ON', fg='#1f7a1f')
        else:
            button.configure(
                text=f'DO{do_index} OFF', bg='#8b1a1a', activebackground='#b22222'
            )
            state_label.configure(text='OFF', fg='#8b1a1a')
        if pending:
            state_label.configure(text='PENDING', fg='#8a6d1d')

    def _update_control_states(self) -> None:
        ready = bool(self._node.io_snapshot()['ready'])
        enabled = ready and not self._closing and self._operation_name is None
        state = tk.NORMAL if enabled else tk.DISABLED
        for channel in self._channels.values():
            channel['button'].configure(state=state)
            channel['duration_entry'].configure(state=state)

    def _parse_duration_ms(self, do_index: int) -> int | None:
        text = str(self._channels[do_index]['duration_var'].get()).strip()
        if text == '':
            self._status_var.set(f'DO{do_index}: auto-off time is required')
            return None
        if not text.isascii() or not text.isdecimal() or str(int(text)) != text:
            self._status_var.set(f'DO{do_index}: auto-off must be a canonical integer')
            return None
        return int(text)

    def _cancel_auto_off_timer(self, do_index: int) -> None:
        channel = self._channels[do_index]
        timer_after_id = channel['timer_after_id']
        if timer_after_id is not None:
            self._root.after_cancel(timer_after_id)
            channel['timer_after_id'] = None

    def _on_toggle(self, do_index: int) -> None:
        channel = self._channels[do_index]
        if bool(channel['on']):
            self._cancel_auto_off_timer(do_index)
            self._run_sequence(
                f'Manual DO{do_index} OFF',
                [('set', do_index, 0)],
                f'DO{do_index}: OFF confirmed',
            )
            return
        duration_ms = self._parse_duration_ms(do_index)
        if duration_ms is None:
            return

        def _schedule_auto_off() -> None:
            if duration_ms > 0:
                channel['timer_after_id'] = self._root.after(
                    duration_ms,
                    lambda index=do_index: self._auto_off(index),
                )

        self._run_sequence(
            f'Manual DO{do_index} ON',
            [('set', do_index, 1)],
            f'DO{do_index}: ON confirmed',
            on_success=_schedule_auto_off,
        )

    def _auto_off(self, do_index: int) -> None:
        channel = self._channels[do_index]
        channel['timer_after_id'] = None
        if self._closing or not bool(channel['on']):
            return
        if self._operation_name is not None:
            channel['timer_after_id'] = self._root.after(
                50, lambda index=do_index: self._auto_off(index)
            )
            return
        self._run_sequence(
            f'Automatic DO{do_index} OFF',
            [('set', do_index, 0)],
            f'DO{do_index}: automatic OFF confirmed',
        )

    def _run_sequence(
        self,
        action_name: str,
        steps: list[tuple],
        success_text: str,
        on_success=None,
    ) -> bool:
        if self._closing or self._operation_name is not None:
            return False
        snapshot = self._node.io_snapshot()
        if not snapshot['ready']:
            self._status_var.set(f'{action_name} rejected: {snapshot["health_text"]}')
            self._node.event_logger.record(
                'ERROR', 'action_rejected', f'{action_name}: {snapshot["health_text"]}'
            )
            return False

        self._operation_name = action_name
        self._operation_token += 1
        token = self._operation_token
        self._status_var.set(f'{action_name}: starting')
        self._node.event_logger.record('INFO', 'action_started', action_name)
        self._update_control_states()

        def _finish(success: bool, detail: str) -> None:
            if token != self._operation_token or self._closing:
                return
            self._operation_name = None
            self._status_var.set(detail)
            self._node.event_logger.record(
                'INFO' if success else 'ERROR',
                'action_result',
                f'{action_name}: {detail}',
            )
            self._update_control_states()
            if success and on_success is not None:
                on_success()

        def _run_step(step_index: int) -> None:
            if token != self._operation_token or self._closing:
                return
            if step_index >= len(steps):
                _finish(True, success_text)
                return
            step = steps[step_index]
            if step[0] == 'delay':
                delay_ms = int(step[1])
                self._status_var.set(f'{action_name}: waiting {delay_ms} ms')
                self._root.after(delay_ms, lambda: _run_step(step_index + 1))
                return
            _, do_index, status = step
            self._status_var.set(
                f'{action_name}: DO{do_index} -> {"ON" if status else "OFF"}'
            )

            def _step_complete(success: bool, detail: str) -> None:
                if not success:
                    _finish(False, f'failed at DO{do_index}={status}: {detail}')
                    return
                _run_step(step_index + 1)

            self._send_do_confirmed(do_index, status, token, _step_complete)

        _run_step(0)
        return True

    def _send_do_confirmed(self, do_index: int, status: int, token: int, on_complete) -> None:
        channel = self._channels[do_index]
        channel['pending'] = True
        self._set_channel_ui(do_index)

        def _service_complete(success: bool, result_code: int, detail: str) -> None:
            def _handle_service_result() -> None:
                if token != self._operation_token or self._finalized:
                    return
                if not success:
                    channel['pending'] = False
                    self._set_channel_ui(do_index)
                    on_complete(False, detail)
                    return
                self._wait_for_output_confirmation(
                    do_index,
                    status,
                    token,
                    time.monotonic() + OUTPUT_CONFIRM_TIMEOUT_SEC,
                    on_complete,
                )

            self._enqueue_ui(_handle_service_result)

        self._node.send_do(do_index, status, _service_complete)

    def _wait_for_output_confirmation(
        self,
        do_index: int,
        status: int,
        token: int,
        deadline: float,
        on_complete,
    ) -> None:
        channel = self._channels[do_index]
        if token != self._operation_token or self._finalized:
            return
        snapshot = self._node.io_snapshot()
        actual = bool(int(snapshot['digital_output_bits']) & (1 << (do_index - 1)))
        if snapshot['ready'] and actual == bool(status):
            channel['pending'] = False
            self._set_channel_ui(do_index)
            self._node.event_logger.record(
                'INFO',
                'do_confirmed',
                f'DO{do_index}={status} confirmed by {FEED_INFO_TOPIC}',
            )
            on_complete(True, 'feedback confirmed')
            return
        if time.monotonic() >= deadline:
            channel['pending'] = False
            self._set_channel_ui(do_index)
            detail = (
                f'FeedInfo did not confirm DO{do_index}={status} within '
                f'{OUTPUT_CONFIRM_TIMEOUT_SEC:.1f} seconds'
            )
            self._node.event_logger.record('ERROR', 'do_confirmation_failed', detail)
            on_complete(False, detail)
            return
        self._root.after(
            50,
            lambda: self._wait_for_output_confirmation(
                do_index, status, token, deadline, on_complete
            ),
        )

    def _on_close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self._operation_token += 1
        self._operation_name = 'Shutdown'
        for index in OUTPUT_CHANNELS:
            self._cancel_auto_off_timer(index)
            self._channels[index]['pending'] = False
        self._status_var.set('Closing: requesting DO1, DO2, DO13, and DO14 OFF...')
        self._node.event_logger.record(
            'INFO',
            'shutdown_started',
            'requesting DO1, DO2, DO13, and DO14 OFF regardless of cached state',
        )
        self._update_control_states()
        self._root.after(SHUTDOWN_TIMEOUT_MS, self._shutdown_timeout)
        self._shutdown_output(0)

    def _shutdown_output(self, channel_position: int) -> None:
        if self._finalized:
            return
        if channel_position >= len(OUTPUT_CHANNELS):
            self._finalize_shutdown()
            return
        do_index = OUTPUT_CHANNELS[channel_position]
        token = self._operation_token

        def _complete(success: bool, detail: str) -> None:
            if not success:
                self._node.event_logger.record(
                    'ERROR', 'shutdown_output_failed', f'DO{do_index}: {detail}'
                )
            self._shutdown_output(channel_position + 1)

        self._send_do_confirmed(do_index, 0, token, _complete)

    def _shutdown_timeout(self) -> None:
        if self._finalized:
            return
        self._node.event_logger.record(
            'ERROR',
            'shutdown_timeout',
            f'DO shutdown sequence exceeded {SHUTDOWN_TIMEOUT_MS} ms',
        )
        self._finalize_shutdown()

    def _finalize_shutdown(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        self._node.event_logger.record('INFO', 'shutdown_complete', 'GUI shutdown complete')
        self._stop_event.set()
        try:
            self._executor.shutdown()
        except Exception:  # pragma: no cover - defensive shutdown path
            pass
        try:
            self._node.destroy_node()
        except Exception:  # pragma: no cover - defensive shutdown path
            pass
        if rclpy.ok():
            rclpy.shutdown()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        self._root.destroy()


def _wait_for_live_bringup(
    node: GripperControlNode,
    executor: SingleThreadedExecutor,
) -> tuple[bool, list[str]]:
    deadline = time.monotonic() + STARTUP_TIMEOUT_SEC
    while rclpy.ok() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
        missing = node.startup_missing_requirements()
        if not missing:
            return True, []
    return False, node.startup_missing_requirements()


def _spin_executor(executor: SingleThreadedExecutor, stop_event: threading.Event) -> None:
    while rclpy.ok() and not stop_event.is_set():
        executor.spin_once(timeout_sec=0.1)


def main() -> None:
    event_logger = PackageEventLogger()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        message = 'ROS_LOCALHOST_ONLY must be exactly 1 before starting gripper_control'
        event_logger.record('ERROR', 'startup_failed', message)
        print(f'[GRIPPER CONTROL] {message}', file=sys.stderr)
        raise SystemExit(1)

    rclpy.init()
    node = GripperControlNode(event_logger)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ready, missing = _wait_for_live_bringup(node, executor)
    if not ready:
        message = (
            f'live Dobot bringup precondition failed after {STARTUP_TIMEOUT_SEC:.0f} seconds: '
            + ', '.join(missing)
        )
        event_logger.record('ERROR', 'startup_failed', message)
        node.get_logger().error(message)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'[GRIPPER CONTROL] {message}', file=sys.stderr)
        raise SystemExit(1)

    event_logger.record(
        'INFO',
        'startup_ready',
        'canonical DO service, connected RobotStatus, and FeedInfo are live',
    )
    stop_event = threading.Event()
    spin_thread = threading.Thread(
        target=_spin_executor,
        args=(executor, stop_event),
        daemon=True,
    )
    spin_thread.start()

    root = tk.Tk()
    GripperControlApp(root, node, executor, spin_thread, stop_event)
    root.mainloop()


if __name__ == '__main__':
    main()
