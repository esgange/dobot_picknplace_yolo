from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, scrolledtext, ttk

from .device_scan import scan_connected_serials
from .event_log import PackageEventLogger
from .project_config import (
    FIXED_CAMERA_COUNT,
    ORBBEC_CAMERA_SLOTS,
    ORBBEC_ENV_KEYS,
    ORBBEC_LAUNCH_FILE,
    camera_specs,
    load_project_config,
    orbbec_launch_arguments,
    validate_project_config,
    write_orbbec_config,
)


PACKAGE_NAME = 'orbbec_camera_launcher'
HEADLESS_LAUNCH = 'camera_headless.launch.py'
ORBBEC_PACKAGE_NAME = 'orbbec_camera'
TERMINAL_EXECUTABLE = Path('/usr/bin/gnome-terminal')


def single_camera_ros_command(values: dict[str, str], slot: int) -> list[str]:
    """Build the canonical direct-launch command for one configured camera slot."""
    validate_project_config(values, require_camera_ready=True)
    if slot not in ORBBEC_CAMERA_SLOTS:
        raise RuntimeError(f'Camera slot must be one of {ORBBEC_CAMERA_SLOTS}')
    camera_by_slot = {camera.slot: camera for camera in camera_specs(values)}
    camera = camera_by_slot[slot]
    launch_arguments = orbbec_launch_arguments(values)
    return [
        'ros2',
        'launch',
        ORBBEC_PACKAGE_NAME,
        ORBBEC_LAUNCH_FILE,
        f'camera_name:={camera.name}',
        f'serial_number:={camera.serial_number}',
        'device_num:=1',
        *[f'{key}:={value}' for key, value in launch_arguments.items()],
    ]


class CameraLauncherApp:
    def __init__(
        self,
        root: tk.Tk,
        project_root: Path,
        project_values: dict[str, str],
        event_logger: PackageEventLogger,
    ) -> None:
        self.root = root
        self.project_root = project_root
        self.project_values = dict(project_values)
        self.event_logger = event_logger
        self.env_path = project_root / '.env'

        self.string_vars = {
            key: tk.StringVar(value=project_values[key])
            for key in ORBBEC_ENV_KEYS
        }
        self.boolean_vars = {
            key: tk.BooleanVar(value=project_values[key] == 'true')
            for key in (
                'ORBBEC_DEPTH_REGISTRATION',
                'ORBBEC_ENABLE_FRAME_SYNC',
                'ORBBEC_ENABLE_TEMPORAL_FILTER',
                'ORBBEC_ENABLE_POINT_CLOUD',
            )
        }
        self.status_var = tk.StringVar(value='Configuration loaded; cameras are stopped')
        self.scan_status_var = tk.StringVar(value='No device scan has run')
        self.process_var = tk.StringVar(value='No camera process running')
        self.selected_serial_var = tk.StringVar(value='')
        self.detected_serials: list[str] = []
        self.single_launch_buttons: dict[int, ttk.Button] = {}
        self.single_camera_processes: dict[int, subprocess.Popen] = {}
        self.supervisor_process: subprocess.Popen | None = None
        self.supervisor_process_group: int | None = None
        self.supervisor_event_cursor: int | None = None
        self.scanning = False
        self.stopping = False
        self.closing = False

        self._build_ui()
        self._bind_updates()
        self._update_button_state()
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.event_logger.record(
            'INFO',
            'gui_started',
            f'config={self.env_path} local_only=1 automatic_launch=false',
        )

    def _build_ui(self) -> None:
        self.root.title('Orbbec Gemini 335 Camera Launcher')
        self.root.geometry('900x780')
        self.root.minsize(820, 700)

        outer = ttk.Frame(self.root, padding=14)
        outer.pack(fill='both', expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(4, weight=1)

        header = ttk.Frame(outer)
        header.grid(row=0, column=0, sticky='ew')
        header.columnconfigure(0, weight=1)
        ttk.Label(
            header,
            text='Orbbec Gemini 335 Camera Launcher',
            font=('TkDefaultFont', 14, 'bold'),
        ).grid(row=0, column=0, sticky='w')
        ttk.Label(header, textvariable=self.status_var).grid(row=1, column=0, sticky='w')
        ttk.Label(header, text=f'Canonical config: {self.env_path}').grid(
            row=2, column=0, sticky='w', pady=(2, 10)
        )

        mapping = ttk.LabelFrame(outer, text='Exact camera mapping', padding=10)
        mapping.grid(row=1, column=0, sticky='ew')
        mapping.columnconfigure(2, weight=1)
        mapping.columnconfigure(3, weight=1)
        ttk.Label(mapping, text='Camera').grid(row=0, column=0, sticky='w')
        ttk.Label(mapping, text='Diagnostic launch').grid(row=0, column=1, sticky='w')
        ttk.Label(mapping, text='ROS camera name').grid(row=0, column=2, sticky='w')
        ttk.Label(mapping, text='Exact serial number').grid(row=0, column=3, sticky='w')

        for slot in ORBBEC_CAMERA_SLOTS:
            row = slot
            ttk.Label(mapping, text=f'Camera {slot}').grid(row=row, column=0, sticky='w', pady=4)
            launch_button = ttk.Button(
                mapping,
                text=f'Launch Camera {slot}',
                command=lambda selected_slot=slot: self._launch_single_camera(selected_slot),
            )
            launch_button.grid(row=row, column=1, sticky='w', padx=(8, 20), pady=4)
            name_entry = ttk.Entry(
                mapping,
                textvariable=self.string_vars[f'ORBBEC_CAMERA_{slot}_NAME'],
            )
            name_entry.grid(row=row, column=2, sticky='ew', padx=(0, 10), pady=4)
            serial_entry = ttk.Entry(
                mapping,
                textvariable=self.string_vars[f'ORBBEC_CAMERA_{slot}_SERIAL'],
            )
            serial_entry.grid(row=row, column=3, sticky='ew', pady=4)
            self.single_launch_buttons[slot] = launch_button

        ttk.Label(
            mapping,
            text=(
                'Single-camera buttons open one direct vendor launch in a separate '
                'terminal without watchdog supervision.'
            ),
        ).grid(row=3, column=0, columnspan=4, sticky='w', pady=(8, 0))

        profile = ttk.LabelFrame(outer, text='Absolute Gemini 335 stream profile', padding=10)
        profile.grid(row=2, column=0, sticky='ew', pady=(10, 0))
        for column in range(8):
            profile.columnconfigure(column, weight=1 if column in {1, 3, 5, 7} else 0)
        self._entry_row(
            profile,
            0,
            (
                ('Preset', 'ORBBEC_DEVICE_PRESET'),
                ('Color width', 'ORBBEC_COLOR_WIDTH'),
                ('Color height', 'ORBBEC_COLOR_HEIGHT'),
                ('Color FPS', 'ORBBEC_COLOR_FPS'),
            ),
        )
        self._entry_row(
            profile,
            1,
            (
                ('Align target', 'ORBBEC_ALIGN_TARGET_STREAM'),
                ('Align mode', 'ORBBEC_ALIGN_MODE'),
                ('Depth width', 'ORBBEC_DEPTH_WIDTH'),
                ('Depth height', 'ORBBEC_DEPTH_HEIGHT'),
            ),
        )
        self._entry_row(
            profile,
            2,
            (
                ('Depth FPS', 'ORBBEC_DEPTH_FPS'),
                ('Scan timeout', 'ORBBEC_SCAN_TIMEOUT_SEC'),
                ('Startup timeout', 'ORBBEC_STARTUP_TIMEOUT_SEC'),
                ('Health timeout', 'ORBBEC_HEALTH_TIMEOUT_SEC'),
            ),
        )
        self._entry_row(
            profile,
            3,
            (
                ('Check period', 'ORBBEC_CHECK_PERIOD_SEC'),
                ('Shutdown timeout', 'ORBBEC_SHUTDOWN_TIMEOUT_SEC'),
            ),
        )

        flags = ttk.Frame(profile)
        flags.grid(row=4, column=0, columnspan=8, sticky='ew', pady=(8, 0))
        for column, (label, key) in enumerate(
            (
                ('Depth registration', 'ORBBEC_DEPTH_REGISTRATION'),
                ('Frame sync', 'ORBBEC_ENABLE_FRAME_SYNC'),
                ('Temporal filter', 'ORBBEC_ENABLE_TEMPORAL_FILTER'),
                ('Point cloud', 'ORBBEC_ENABLE_POINT_CLOUD'),
            )
        ):
            ttk.Checkbutton(flags, text=label, variable=self.boolean_vars[key]).grid(
                row=0, column=column, sticky='w', padx=(0, 18)
            )
        ttk.Label(
            flags,
            text='Color=true  Depth=true  Network enumeration=false  Attempts=3  Rest=3s',
        ).grid(row=1, column=0, columnspan=4, sticky='w', pady=(6, 0))

        actions = ttk.Frame(outer)
        actions.grid(row=3, column=0, sticky='ew', pady=(10, 0))
        actions.columnconfigure(4, weight=1)
        self.scan_button = ttk.Button(actions, text='Scan Devices', command=self._scan_devices)
        self.scan_button.grid(row=0, column=0, padx=(0, 8))
        self.save_button = ttk.Button(actions, text='Save to .env', command=self._save_config)
        self.save_button.grid(row=0, column=1, padx=(0, 8))
        self.launch_button = ttk.Button(
            actions,
            text='Launch Both (Watchdog)',
            command=self._launch_cameras,
        )
        self.launch_button.grid(row=0, column=2, padx=(0, 8))
        self.stop_button = ttk.Button(
            actions,
            text='Stop Watchdog',
            command=self._stop_cameras,
        )
        self.stop_button.grid(row=0, column=3, padx=(0, 8))
        ttk.Label(actions, textvariable=self.process_var).grid(row=0, column=4, sticky='e')

        scan_frame = ttk.LabelFrame(outer, text='Camera launcher log', padding=10)
        scan_frame.grid(row=4, column=0, sticky='nsew', pady=(10, 0))
        scan_frame.columnconfigure(0, weight=1)
        scan_frame.rowconfigure(2, weight=1)
        ttk.Label(scan_frame, textvariable=self.scan_status_var).grid(
            row=0,
            column=0,
            sticky='w',
        )
        self.copy_log_button = ttk.Button(
            scan_frame,
            text='Copy Log',
            command=self._copy_log,
        )
        self.copy_log_button.grid(row=0, column=1, sticky='e')
        serial_selector = ttk.Frame(scan_frame)
        serial_selector.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(6, 8))
        serial_selector.columnconfigure(1, weight=1)
        ttk.Label(serial_selector, text='Detected serial (scan result)').grid(
            row=0,
            column=0,
            sticky='w',
        )
        self.serial_selector = ttk.Combobox(
            serial_selector,
            textvariable=self.selected_serial_var,
            values=(),
            state='readonly',
        )
        self.serial_selector.grid(row=0, column=1, sticky='ew', padx=(8, 0))
        self.output_text = scrolledtext.ScrolledText(scan_frame, height=14, wrap='word')
        self.output_text.grid(row=2, column=0, columnspan=2, sticky='nsew')
        self.output_text.bind('<Control-c>', self._copy_log_selection)
        self._set_output(
            'No automatic camera launch occurs. Scan, enter exact serials, save, then launch.\n'
            'Single-camera launches run in their own terminal; stop them there with Ctrl+C.\n'
        )

    def _entry_row(
        self,
        parent: ttk.LabelFrame,
        row: int,
        fields: tuple[tuple[str, str], ...],
    ) -> None:
        for field_index, (label, key) in enumerate(fields):
            column = field_index * 2
            ttk.Label(parent, text=label).grid(row=row, column=column, sticky='w', pady=3)
            ttk.Entry(parent, textvariable=self.string_vars[key], width=16).grid(
                row=row,
                column=column + 1,
                sticky='ew',
                padx=(6, 12),
                pady=3,
            )

    def _bind_updates(self) -> None:
        for variable in self.string_vars.values():
            variable.trace_add('write', lambda *_args: self._update_button_state())
        for variable in self.boolean_vars.values():
            variable.trace_add('write', lambda *_args: self._update_button_state())

    def _collect_orbbec_values(self) -> dict[str, str]:
        updates = {key: variable.get().strip() for key, variable in self.string_vars.items()}
        for key, variable in self.boolean_vars.items():
            updates[key] = 'true' if variable.get() else 'false'
        updates['ORBBEC_ENABLE_COLOR'] = 'true'
        updates['ORBBEC_ENABLE_DEPTH'] = 'true'
        updates['ORBBEC_ENUMERATE_NET_DEVICE'] = 'false'
        updates['ORBBEC_MAX_ATTEMPTS'] = '3'
        updates['ORBBEC_RETRY_DELAY_SEC'] = '3'
        return updates

    def _configuration_error(self) -> str | None:
        merged = dict(self.project_values)
        merged.update(self._collect_orbbec_values())
        try:
            validate_project_config(merged, require_camera_ready=True)
        except RuntimeError as exc:
            return str(exc)
        return None

    def _process_alive(self) -> bool:
        process = self.supervisor_process
        if process is None:
            return False
        if process.poll() is None:
            return True
        return self._process_group_alive(self.supervisor_process_group)

    def _running_single_camera_slot(self) -> int | None:
        for slot, process in self.single_camera_processes.items():
            if process.poll() is None:
                return slot
        return None

    @staticmethod
    def _process_group_alive(process_group: int | None) -> bool:
        if process_group is None:
            return False
        try:
            os.killpg(process_group, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def _update_button_state(self) -> None:
        if not hasattr(self, 'launch_button'):
            return
        supervisor_running = self._process_alive()
        single_camera_slot = self._running_single_camera_slot()
        running = supervisor_running or single_camera_slot is not None
        configuration_ready = self._configuration_error() is None
        self.scan_button.configure(state='disabled' if self.scanning or running else 'normal')
        self.save_button.configure(state='disabled' if running or self.stopping else 'normal')
        self.launch_button.configure(
            state=(
                'normal'
                if configuration_ready and not running and not self.stopping
                else 'disabled'
            )
        )
        self.stop_button.configure(
            state='normal' if supervisor_running and not self.stopping else 'disabled'
        )
        self.launch_button.configure(text='Launch Both (Watchdog)')
        for slot, button in self.single_launch_buttons.items():
            button.configure(
                text=(
                    f'Camera {slot} Running'
                    if single_camera_slot == slot
                    else f'Launch Camera {slot}'
                )
            )
            button.configure(
                state=(
                    'normal'
                    if configuration_ready
                    and not running
                    and not self.stopping
                    else 'disabled'
                )
            )

    def _set_output(self, text: str) -> None:
        self.output_text.configure(state='normal')
        self.output_text.delete('1.0', tk.END)
        self.output_text.insert(tk.END, text)
        self.output_text.configure(state='disabled')

    def _append_output(self, text: str) -> None:
        self.output_text.configure(state='normal')
        self.output_text.insert(tk.END, text)
        self.output_text.see(tk.END)
        self.output_text.configure(state='disabled')

    def _copy_text(self, text: str, description: str) -> bool:
        try:
            self.root.clipboard_clear()
            self.root.clipboard_append(text)
        except tk.TclError as exc:
            self.event_logger.record(
                'ERROR',
                'gui_log_copy_failed',
                f'description={description} error={exc}',
            )
            self.scan_status_var.set(f'Could not copy {description}')
            return False
        self.event_logger.record(
            'INFO',
            'gui_log_copied',
            f'description={description} characters={len(text)}',
            copied_characters=len(text),
        )
        self.scan_status_var.set(f'Copied {description} to clipboard')
        return True

    def _copy_log(self) -> None:
        text = self.output_text.get('1.0', 'end-1c')
        self._copy_text(text, 'complete camera launcher log')

    def _copy_log_selection(self, _event=None):
        try:
            text = self.output_text.get(tk.SEL_FIRST, tk.SEL_LAST)
        except tk.TclError:
            return 'break'
        self._copy_text(text, 'selected camera launcher log text')
        return 'break'

    def _scan_devices(self) -> None:
        if self.scanning or self._process_alive() or self._running_single_camera_slot():
            return
        try:
            timeout = float(self.string_vars['ORBBEC_SCAN_TIMEOUT_SEC'].get())
            if not 0.0 < timeout < float('inf'):
                raise ValueError
        except ValueError:
            messagebox.showerror(
                'Invalid scan timeout',
                'ORBBEC_SCAN_TIMEOUT_SEC must be positive.',
            )
            return
        self.scanning = True
        self.scan_status_var.set('Scanning connected Orbbec USB devices...')
        self._append_output('\n[SCAN] Running the canonical Orbbec device scan.\n')
        self.event_logger.record('INFO', 'gui_scan_started', f'timeout_sec={timeout:g}')
        self._update_button_state()
        threading.Thread(target=self._scan_worker, args=(timeout,), daemon=True).start()

    def _scan_worker(self, timeout: float) -> None:
        result = scan_connected_serials(timeout)
        self.root.after(0, lambda: self._finish_scan(*result))

    def _finish_scan(self, return_code: int, serials: list[str], output: str) -> None:
        self.scanning = False
        self.detected_serials = serials
        self.serial_selector.configure(values=tuple(serials))
        self.selected_serial_var.set(serials[0] if serials else '')
        self.scan_status_var.set(
            f'Detected {len(serials)} serial(s); exit code {return_code}'
        )
        text = f'\n[SCAN] Completed with exit code {return_code}.\nDetected serial numbers:\n'
        text += ''.join(f'  {serial}\n' for serial in serials) if serials else '  none\n'
        if output:
            text += f'\nRaw scan output:\n{output}\n'
        self._append_output(text)
        self.event_logger.record(
            'INFO' if return_code == 0 else 'ERROR',
            'gui_scan_finished',
            f'exit_code={return_code} detected={serials}',
            exit_code=return_code,
            detected_serials=serials,
            scan_output=output,
        )
        self._update_button_state()

    def _save_config(self) -> bool:
        updates = self._collect_orbbec_values()
        merged = dict(self.project_values)
        merged.update(updates)
        try:
            validate_project_config(merged, require_camera_ready=True)
            _root, saved = write_orbbec_config(updates)
        except (OSError, RuntimeError) as exc:
            self.event_logger.record('ERROR', 'config_save_failed', str(exc))
            messagebox.showerror('Configuration rejected', str(exc))
            self.status_var.set('Configuration was not saved')
            self._update_button_state()
            return False

        self.project_values = saved
        for key in ORBBEC_ENV_KEYS:
            if key in self.boolean_vars:
                self.boolean_vars[key].set(saved[key] == 'true')
            else:
                self.string_vars[key].set(saved[key])
        self.event_logger.record(
            'INFO',
            'config_saved',
            'GUI atomically updated canonical Orbbec keys in root .env',
            camera_count=FIXED_CAMERA_COUNT,
            camera_names=[
                saved[f'ORBBEC_CAMERA_{slot}_NAME']
                for slot in ORBBEC_CAMERA_SLOTS
            ],
            serial_numbers=[
                saved[f'ORBBEC_CAMERA_{slot}_SERIAL']
                for slot in ORBBEC_CAMERA_SLOTS
            ],
        )
        self.status_var.set(f'Saved canonical camera configuration to {self.env_path}')
        self._append_output('\n[CONFIG] Configuration saved atomically to root .env.\n')
        self._update_button_state()
        return True

    def _launch_single_camera(self, slot: int) -> None:
        if (
            self._process_alive()
            or self._running_single_camera_slot() is not None
            or self.stopping
        ):
            return
        if not TERMINAL_EXECUTABLE.is_file() or not os.access(
            TERMINAL_EXECUTABLE,
            os.X_OK,
        ):
            error = f'Required terminal executable is missing: {TERMINAL_EXECUTABLE}'
            self.event_logger.record('ERROR', 'single_camera_terminal_missing', error)
            messagebox.showerror('Launch failed', error)
            return
        if not self._save_config():
            return
        try:
            camera_command = single_camera_ros_command(self.project_values, slot)
        except RuntimeError as exc:
            self.event_logger.record(
                'ERROR',
                'single_camera_config_rejected',
                f'camera_slot={slot} error={exc}',
            )
            messagebox.showerror('Launch failed', str(exc))
            return

        camera = next(
            camera
            for camera in camera_specs(self.project_values)
            if camera.slot == slot
        )
        terminal_command = [
            str(TERMINAL_EXECUTABLE),
            '--wait',
            f'--title=Orbbec Camera {slot}: {camera.name}',
            f'--working-directory={self.project_root}',
            '--',
            *camera_command,
        ]
        try:
            process = subprocess.Popen(
                terminal_command,
                cwd=str(self.project_root),
                env=os.environ.copy(),
                start_new_session=True,
            )
        except OSError as exc:
            self.event_logger.record(
                'ERROR',
                'single_camera_terminal_start_failed',
                f'camera_slot={slot} error={exc}',
            )
            messagebox.showerror('Launch failed', str(exc))
            return

        self.single_camera_processes[slot] = process
        self.process_var.set(f'Camera {slot} terminal running (PID {process.pid})')
        self.status_var.set(
            f'Camera {slot} direct launch is running without watchdog supervision'
        )
        self._append_output(
            f'\n[CAMERA {slot}] Started direct launch for {camera.name} '
            f'(serial {camera.serial_number}) in terminal PID {process.pid}.\n'
            f'[CAMERA {slot}] Stop it with Ctrl+C in that terminal; the watchdog is disabled.\n'
        )
        self.event_logger.record(
            'INFO',
            'single_camera_terminal_started',
            f'camera_slot={slot} camera={camera.name} serial={camera.serial_number} '
            f'pid={process.pid} watchdog=false',
            camera_slot=slot,
            camera=camera.name,
            serial_number=camera.serial_number,
            pid=process.pid,
            watchdog=False,
        )
        self._update_button_state()
        self.root.after(500, lambda: self._poll_single_camera(slot, process))

    def _poll_single_camera(self, slot: int, process: subprocess.Popen) -> None:
        if self.closing or self.single_camera_processes.get(slot) is not process:
            return
        return_code = process.poll()
        if return_code is None:
            self.root.after(500, lambda: self._poll_single_camera(slot, process))
            return
        del self.single_camera_processes[slot]
        self.event_logger.record(
            'INFO' if return_code == 0 else 'ERROR',
            'single_camera_terminal_exited',
            f'camera_slot={slot} pid={process.pid} return_code={return_code}',
            camera_slot=slot,
            pid=process.pid,
            return_code=return_code,
        )
        self._append_output(
            f'\n[CAMERA {slot}] Direct-launch terminal exited with code {return_code}.\n'
        )
        self.status_var.set(
            f'Camera {slot} direct launch stopped normally'
            if return_code == 0
            else f'Camera {slot} direct launch failed; inspect its terminal output'
        )
        self.process_var.set('No camera process running')
        self._update_button_state()

    def _launch_cameras(self) -> None:
        if (
            self._process_alive()
            or self._running_single_camera_slot() is not None
            or self.stopping
        ):
            return
        if not self._save_config():
            return
        command = ['ros2', 'launch', PACKAGE_NAME, HEADLESS_LAUNCH]
        event_cursor = self.event_logger.cursor()
        try:
            process = subprocess.Popen(
                command,
                cwd=str(self.project_root),
                env=os.environ.copy(),
                start_new_session=True,
            )
        except OSError as exc:
            self.event_logger.record('ERROR', 'headless_start_failed', str(exc))
            messagebox.showerror('Launch failed', str(exc))
            return
        self.supervisor_process = process
        self.supervisor_process_group = process.pid
        self.supervisor_event_cursor = event_cursor
        self.process_var.set(f'Supervisor running (PID {process.pid})')
        self.status_var.set('Camera supervisor is executing attempt 1 of 3')
        self._append_output(
            f'\n[SUPERVISOR] Started owned headless supervisor PID {process.pid}. '
            'See this package log for attempt details.\n'
        )
        self.event_logger.record(
            'INFO',
            'headless_process_started',
            f'pid={process.pid} command={command}',
            pid=process.pid,
        )
        self._update_button_state()
        self.root.after(500, self._poll_supervisor)

    def _poll_supervisor(self) -> None:
        process = self.supervisor_process
        if process is None or self.closing:
            return
        return_code = process.poll()
        if return_code is None or self._process_group_alive(self.supervisor_process_group):
            self.root.after(500, self._poll_supervisor)
            return
        evidence_error = ''
        terminal_failure = True
        try:
            if self.supervisor_event_cursor is None:
                raise RuntimeError('Supervisor event cursor is unavailable')
            events = self.event_logger.records_since(self.supervisor_event_cursor)
            stopped_events = [
                event
                for event in events
                if event.get('event') == 'supervisor_stopped'
                and event.get('launch_parent_pid') == process.pid
            ]
            if not stopped_events:
                raise RuntimeError(
                    f'Supervisor terminal event for launch PID {process.pid} is missing'
                )
            recorded = stopped_events[-1].get('terminal_failure')
            if type(recorded) is not bool:
                raise RuntimeError('Supervisor terminal-failure value is invalid')
            terminal_failure = recorded
        except (OSError, RuntimeError, ValueError) as exc:
            evidence_error = str(exc)
            terminal_failure = True
        failed = return_code != 0 or terminal_failure
        self.event_logger.record(
            'ERROR' if failed else 'INFO',
            'headless_process_exited',
            f'pid={process.pid} return_code={return_code} '
            f'terminal_failure={terminal_failure} evidence_error={evidence_error}',
            pid=process.pid,
            return_code=return_code,
            supervisor_terminal_failure=terminal_failure,
            evidence_error=evidence_error,
        )
        self._append_output(
            f'\n[SUPERVISOR] Headless supervisor '
            f'{"failed" if failed else "stopped normally"}; '
            f'wrapper exit code {return_code}.\n'
        )
        self.status_var.set(
            'Camera supervisor failed; inspect package events.jsonl'
            if failed
            else 'Camera supervisor stopped normally'
        )
        self.process_var.set('No camera process running')
        self.supervisor_process = None
        self.supervisor_process_group = None
        self.supervisor_event_cursor = None
        self._update_button_state()

    def _stop_cameras(self) -> None:
        if not self._process_alive() or self.stopping:
            return
        self.stopping = True
        self.status_var.set('Stopping owned camera supervisor and camera processes...')
        self.process_var.set('Stopping supervisor...')
        self.event_logger.record('INFO', 'stop_requested', 'operator requested camera stop')
        self._update_button_state()
        process = self.supervisor_process
        process_group = self.supervisor_process_group
        threading.Thread(
            target=self._stop_worker,
            args=(process, process_group),
            daemon=True,
        ).start()

    def _stop_worker(
        self,
        process: subprocess.Popen | None,
        process_group: int | None,
    ) -> None:
        stopped = self._terminate_owned_process(process, process_group)
        self.root.after(0, lambda: self._finish_stop(stopped))

    def _terminate_owned_process(
        self,
        process: subprocess.Popen | None,
        process_group: int | None,
    ) -> bool:
        if process is None:
            return True
        shutdown_timeout = float(self.project_values['ORBBEC_SHUTDOWN_TIMEOUT_SEC'])
        stages = (
            (signal.SIGINT, shutdown_timeout * 3.0 + 2.0),
            (signal.SIGTERM, shutdown_timeout),
            (signal.SIGKILL, 1.0),
        )
        for sig, timeout in stages:
            if process.poll() is not None and not self._process_group_alive(process_group):
                return True
            try:
                if self._process_group_alive(process_group):
                    os.killpg(int(process_group), sig)
                elif process.poll() is None:
                    process.send_signal(sig)
            except ProcessLookupError:
                return True
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if process.poll() is not None and not self._process_group_alive(process_group):
                    return True
                time.sleep(0.05)
        return process.poll() is not None and not self._process_group_alive(process_group)

    def _finish_stop(self, stopped: bool) -> None:
        process = self.supervisor_process
        self.event_logger.record(
            'INFO' if stopped else 'ERROR',
            'stop_finished',
            f'owned_supervisor_stopped={stopped}',
            pid=process.pid if process is not None else None,
        )
        if stopped:
            self.supervisor_process = None
            self.supervisor_process_group = None
            self.supervisor_event_cursor = None
        self.stopping = False
        self.process_var.set(
            'No camera process running' if stopped else 'Supervisor stop failed'
        )
        self.status_var.set(
            'Owned camera processes stopped'
            if stopped
            else 'Camera shutdown failed; inspect package log'
        )
        self._append_output(
            '\n[SUPERVISOR] Owned camera supervisor stopped.\n'
            if stopped
            else '\n[SUPERVISOR] Owned camera supervisor did not stop cleanly.\n'
        )
        self._update_button_state()

    def shutdown(self) -> None:
        if self.closing:
            return
        self.closing = True
        stopped = self._terminate_owned_process(
            self.supervisor_process,
            self.supervisor_process_group,
        )
        self.event_logger.record(
            'INFO' if stopped else 'ERROR',
            'gui_stopped',
            f'owned_supervisor_stopped={stopped} '
            f'operator_terminal_cameras={list(self.single_camera_processes)}',
            operator_terminal_camera_slots=list(self.single_camera_processes),
        )
        self.supervisor_process = None
        self.supervisor_process_group = None
        self.supervisor_event_cursor = None

    def _on_close(self) -> None:
        self.shutdown()
        self.root.destroy()


def main() -> None:
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError(
            'Source scripts/source_ros_workspace.bash first; '
            'ROS_LOCALHOST_ONLY must be exactly 1'
        )
    project_root, values = load_project_config(require_camera_ready=False)
    event_logger = PackageEventLogger(project_root)
    root = tk.Tk()
    app = CameraLauncherApp(root, project_root, values, event_logger)

    def request_shutdown(_signum=None, _frame=None) -> None:
        try:
            root.after(0, app._on_close)
        except tk.TclError:
            app.shutdown()

    for signal_name in ('SIGINT', 'SIGTERM', 'SIGHUP'):
        sig = getattr(signal, signal_name, None)
        if sig is not None:
            signal.signal(sig, request_shutdown)
    try:
        root.mainloop()
    finally:
        app.shutdown()


if __name__ == '__main__':
    main()
