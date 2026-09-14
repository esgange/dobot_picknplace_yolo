import json
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter

from orbbec_camera_launcher import camera_launcher_gui, camera_watchdog, project_config
from orbbec_camera_launcher.event_log import PackageEventLogger


LAUNCH_ARGUMENTS = {
    'device_preset': 'High Accuracy',
    'enable_color': 'true',
    'enable_depth': 'true',
    'depth_registration': 'true',
    'align_target_stream': 'COLOR',
    'align_mode': 'SW',
    'enable_frame_sync': 'true',
    'enable_temporal_filter': 'true',
    'color_width': '848',
    'color_height': '480',
    'color_fps': '30',
    'depth_width': '848',
    'depth_height': '480',
    'depth_fps': '30',
    'enable_point_cloud': 'false',
    'enumerate_net_device': 'false',
}


class FakeProcess:
    next_pid = 43000
    popen_calls = []

    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.pid = FakeProcess.next_pid
        FakeProcess.next_pid += 1
        FakeProcess.popen_calls.append(self)
        self.returncode = None

    def poll(self):
        return self.returncode

    def send_signal(self, _signal):
        self.returncode = 0


class FakeClipboardRoot:
    def __init__(self):
        self.clipboard = ''

    def clipboard_clear(self):
        self.clipboard = ''

    def clipboard_append(self, text):
        self.clipboard = text


class FakeAfterRoot:
    def __init__(self):
        self.after_calls = []

    def after(self, delay_ms, callback):
        self.after_calls.append((delay_ms, callback))


class FakeStringVar:
    def __init__(self):
        self.value = ''

    def set(self, value):
        self.value = value

    def get(self):
        return self.value


class FakeOutputText:
    def __init__(self, text):
        self.text = text

    def get(self, _start, _end):
        return self.text


class FakeEventLogger:
    def __init__(self):
        self.records = []
        self.records_after_cursor = []

    def record(self, level, event, message, **details):
        self.records.append((level, event, message, details))

    def cursor(self):
        return 0

    def records_since(self, _offset):
        return list(self.records_after_cursor)


def parse_example() -> dict[str, str]:
    values = {}
    example = project_config.project_root() / '.env.example'
    for raw_line in example.read_text(encoding='utf-8').splitlines():
        if not raw_line or raw_line.startswith('#'):
            continue
        key, value = raw_line.split('=', 1)
        values[key] = value
    return values


def create_workspace(path: Path) -> Path:
    path.joinpath('.env.example').write_text('# test marker\n', encoding='utf-8')
    path.joinpath('src').mkdir()
    return path


def parameters(
    path: Path,
    names=('test_camera_1', 'test_camera_2'),
    serials=('TEST123', 'TEST456'),
    *,
    startup_timeout=5.0,
    health_timeout=1.0,
):
    return [
        Parameter('camera_names', value=list(names)),
        Parameter('serial_numbers', value=list(serials)),
        Parameter('orbbec_launch_file', value='gemini_330_series.launch.py'),
        Parameter('device_num', value=len(names)),
        Parameter('launch_args_json', value=json.dumps(LAUNCH_ARGUMENTS)),
        Parameter('workspace_root', value=str(path)),
        Parameter('scan_timeout_sec', value=0.1),
        Parameter('startup_timeout_sec', value=startup_timeout),
        Parameter('health_timeout_sec', value=health_timeout),
        Parameter('check_period_sec', value=0.1),
        Parameter('max_attempts', value=3),
        Parameter('retry_delay_sec', value=3.0),
        Parameter('shutdown_timeout_sec', value=0.1),
    ]


class CameraLauncherTests(unittest.TestCase):
    def test_single_camera_button_opens_exact_vendor_launch_in_terminal(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary)
            terminal = workspace / 'gnome-terminal'
            terminal.write_text('#!/bin/sh\n', encoding='utf-8')
            terminal.chmod(0o755)
            values = parse_example()
            values['ORBBEC_CAMERA_1_SERIAL'] = 'CAMERA001'
            values['ORBBEC_CAMERA_2_SERIAL'] = 'CAMERA002'

            app = camera_launcher_gui.CameraLauncherApp.__new__(
                camera_launcher_gui.CameraLauncherApp
            )
            app.root = FakeAfterRoot()
            app.project_root = workspace
            app.project_values = values
            app.event_logger = FakeEventLogger()
            app.process_var = FakeStringVar()
            app.status_var = FakeStringVar()
            app.supervisor_process = None
            app.supervisor_process_group = None
            app.single_camera_processes = {}
            app.stopping = False
            app.closing = False
            displayed_output = []
            app._append_output = displayed_output.append
            app._save_config = Mock(return_value=True)
            app._update_button_state = Mock()
            FakeProcess.popen_calls = []

            with patch.object(
                camera_launcher_gui,
                'TERMINAL_EXECUTABLE',
                terminal,
            ), patch.object(camera_launcher_gui.subprocess, 'Popen', FakeProcess):
                app._launch_single_camera(2)

            self.assertEqual(len(FakeProcess.popen_calls), 1)
            process = FakeProcess.popen_calls[0]
            terminal_command = process.args[0]
            self.assertEqual(terminal_command[0], str(terminal))
            self.assertIn('--wait', terminal_command)
            self.assertIn(
                f'--title=Orbbec Camera 2: {values["ORBBEC_CAMERA_2_NAME"]}',
                terminal_command,
            )
            self.assertEqual(process.kwargs['cwd'], str(workspace))
            self.assertTrue(process.kwargs['start_new_session'])
            separator = terminal_command.index('--')
            ros_command = terminal_command[separator + 1:]
            self.assertEqual(
                ros_command,
                camera_launcher_gui.single_camera_ros_command(values, 2),
            )
            self.assertIn('device_num:=1', ros_command)
            self.assertNotIn('camera_headless.launch.py', ros_command)
            for key, value in project_config.orbbec_launch_arguments(values).items():
                self.assertIn(f'{key}:={value}', ros_command)
            self.assertEqual(app.single_camera_processes, {2: process})
            self.assertEqual(app.root.after_calls[0][0], 500)
            self.assertEqual(
                app.event_logger.records[-1][1],
                'single_camera_terminal_started',
            )
            self.assertIn('watchdog is disabled', displayed_output[-1])

            process.returncode = 0
            app._poll_single_camera(2, process)
            self.assertEqual(app.single_camera_processes, {})
            self.assertEqual(
                app.event_logger.records[-1][1],
                'single_camera_terminal_exited',
            )

    def test_removed_camera_count_key_is_rejected(self):
        values = parse_example()
        values['ORBBEC_CAMERA_COUNT'] = '1'

        with self.assertRaisesRegex(RuntimeError, 'unsupported=ORBBEC_CAMERA_COUNT'):
            project_config.validate_project_config(
                values,
                require_camera_ready=False,
            )

    def test_camera_specs_are_always_exactly_two(self):
        values = parse_example()

        cameras = project_config.camera_specs(values)

        self.assertEqual([camera.slot for camera in cameras], [1, 2])
        self.assertEqual(len(cameras), project_config.FIXED_CAMERA_COUNT)

    def test_complete_gui_log_copies_exact_text(self):
        app = camera_launcher_gui.CameraLauncherApp.__new__(
            camera_launcher_gui.CameraLauncherApp
        )
        app.root = FakeClipboardRoot()
        app.output_text = FakeOutputText('scan result\nsupervisor result')
        app.scan_status_var = FakeStringVar()
        app.event_logger = FakeEventLogger()

        app._copy_log()

        self.assertEqual(app.root.clipboard, 'scan result\nsupervisor result')
        self.assertEqual(
            app.scan_status_var.value,
            'Copied complete camera launcher log to clipboard',
        )
        self.assertEqual(app.event_logger.records[0][1], 'gui_log_copied')
        self.assertEqual(
            app.event_logger.records[0][3]['copied_characters'],
            len('scan result\nsupervisor result'),
        )

    def test_selected_gui_log_text_copies_exact_selection(self):
        app = camera_launcher_gui.CameraLauncherApp.__new__(
            camera_launcher_gui.CameraLauncherApp
        )
        app.root = FakeClipboardRoot()
        app.output_text = FakeOutputText('selected text')
        app.scan_status_var = FakeStringVar()
        app.event_logger = FakeEventLogger()

        result = app._copy_log_selection()

        self.assertEqual(result, 'break')
        self.assertEqual(app.root.clipboard, 'selected text')
        self.assertEqual(
            app.scan_status_var.value,
            'Copied selected camera launcher log text to clipboard',
        )
        self.assertEqual(app.event_logger.records[0][1], 'gui_log_copied')

    def test_project_config_requires_both_camera_slots_before_launch(self):
        values = parse_example()
        project_config.validate_project_config(values, require_camera_ready=False)
        with self.assertRaisesRegex(RuntimeError, 'SERIAL'):
            project_config.validate_project_config(values, require_camera_ready=True)

        values['ORBBEC_CAMERA_1_SERIAL'] = 'CAMERA001'
        values['ORBBEC_CAMERA_2_SERIAL'] = 'CAMERA002'
        project_config.validate_project_config(values, require_camera_ready=True)

    def test_event_log_overwrites_before_record_1001(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            logger = PackageEventLogger(root)
            logger.path.write_text('{}\n' * 1000, encoding='utf-8')
            logger.record('INFO', 'bounded', 'newest event')
            lines = logger.path.read_text(encoding='utf-8').splitlines()
            self.assertEqual(len(lines), 1)
            self.assertEqual(json.loads(lines[0])['event'], 'bounded')

    def test_event_log_cursor_reads_only_new_complete_records(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            logger = PackageEventLogger(root)
            logger.record('INFO', 'before', 'before cursor')
            cursor = logger.cursor()
            logger.record('ERROR', 'after', 'after cursor', terminal_failure=True)

            records = logger.records_since(cursor)

            self.assertEqual([record['event'] for record in records], ['after'])
            self.assertTrue(records[0]['terminal_failure'])

    def test_gui_writer_changes_only_complete_orbbec_key_set(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            example_text = (
                project_config.project_root()
                .joinpath('.env.example')
                .read_text(encoding='utf-8')
            )
            root.joinpath('.env').write_text(example_text, encoding='utf-8')
            values = parse_example()
            original_dobot = {
                key: values[key] for key in project_config.DOBOT_ENV_KEYS
            }
            updates = {
                key: values[key] for key in project_config.ORBBEC_ENV_KEYS
            }
            updates['ORBBEC_CAMERA_1_SERIAL'] = 'CAMERA001'
            updates['ORBBEC_CAMERA_2_SERIAL'] = 'CAMERA002'

            with patch.object(project_config, 'project_root', return_value=root):
                returned_root, saved = project_config.write_orbbec_config(updates)
                _loaded_root, loaded = project_config.load_project_config(
                    require_camera_ready=True
                )

            self.assertEqual(returned_root, root)
            self.assertEqual(saved, loaded)
            self.assertEqual(
                {key: loaded[key] for key in project_config.DOBOT_ENV_KEYS},
                original_dobot,
            )
            self.assertIn(
                '# Project-wide runtime configuration.',
                root.joinpath('.env').read_text(encoding='utf-8'),
            )

    def test_project_config_requires_exact_five_second_startup_timeout(self):
        values = parse_example()
        values['ORBBEC_STARTUP_TIMEOUT_SEC'] = '5.1'

        with self.assertRaisesRegex(RuntimeError, 'must be exactly 5'):
            project_config.validate_project_config(
                values,
                require_camera_ready=False,
            )

    def test_watchdog_hard_fails_after_three_complete_set_attempts(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = create_workspace(Path(temporary))
            context = Context()
            rclpy.init(context=context)
            node = None
            try:
                with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}), patch.object(
                    camera_watchdog,
                    'scan_connected_serials',
                    return_value=(0, [], 'no connected devices'),
                ):
                    node = camera_watchdog.CameraWatchdog(
                        parameter_overrides=parameters(workspace),
                        context=context,
                    )
                    for expected_attempt in (1, 2, 3):
                        node._next_attempt_at = 0.0
                        node._check_cameras()
                        self.assertEqual(node._attempt_count, expected_attempt)
                    self.assertTrue(node.terminal_failure)
                    self.assertEqual(node._phase, 'failed')
                    events = [
                        json.loads(line)
                        for line in workspace.joinpath(
                            'logs/orbbec_camera_launcher/events.jsonl'
                        ).read_text(encoding='utf-8').splitlines()
                    ]
                    self.assertEqual(
                        sum(
                            event['event'] == 'connection_attempt_failed'
                            for event in events
                        ),
                        3,
                    )
                    self.assertEqual(events[-1]['event'], 'launcher_failed')
            finally:
                if node is not None:
                    node.shutdown()
                    node.destroy_node()
                context.try_shutdown()

    def test_watchdog_uses_owned_process_and_lifetime_retry_budget(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = create_workspace(Path(temporary))
            FakeProcess.popen_calls = []
            context = Context()
            rclpy.init(context=context)
            node = None
            try:
                with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}), patch.object(
                    camera_watchdog,
                    'scan_connected_serials',
                    return_value=(
                        0,
                        ['TEST123', 'TEST456'],
                        'serials: TEST123 TEST456',
                    ),
                ), patch.object(camera_watchdog.subprocess, 'Popen', FakeProcess):
                    node = camera_watchdog.CameraWatchdog(
                        parameter_overrides=parameters(workspace),
                        context=context,
                    )
                    node._check_cameras()
                    first = node._states['test_camera_1']
                    self.assertEqual(node._attempt_count, 1)
                    self.assertEqual(node._phase, 'starting')
                    self.assertIsNotNone(first.process)
                    self.assertTrue(first.process.kwargs['start_new_session'])
                    self.assertTrue(callable(first.process.kwargs['preexec_fn']))

                    node._record_image('test_camera_1', 'color')
                    node._record_image('test_camera_1', 'depth')
                    node._check_cameras()
                    second = node._states['test_camera_2']
                    self.assertIsNotNone(second.process)
                    node._record_image('test_camera_2', 'color')
                    node._record_image('test_camera_2', 'depth')
                    node._check_cameras()
                    self.assertEqual(node._phase, 'healthy')

                    first.last_depth_at = time.monotonic() - 2.0
                    node._check_cameras()
                    self.assertEqual(node._phase, 'waiting')
                    self.assertEqual(node._attempt_count, 1)
                    self.assertIsNone(first.process)
                    self.assertIsNone(second.process)
                    self.assertGreater(node._next_attempt_at, time.monotonic())
            finally:
                if node is not None:
                    node.shutdown()
                    node.destroy_node()
                context.try_shutdown()

    def test_two_cameras_start_strictly_in_configured_order(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = create_workspace(Path(temporary))
            FakeProcess.popen_calls = []
            context = Context()
            rclpy.init(context=context)
            node = None
            try:
                with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}), patch.object(
                    camera_watchdog,
                    'scan_connected_serials',
                    return_value=(0, ['SERIAL1', 'SERIAL2'], 'both connected'),
                ), patch.object(camera_watchdog.subprocess, 'Popen', FakeProcess):
                    node = camera_watchdog.CameraWatchdog(
                        parameter_overrides=parameters(
                            workspace,
                            names=('camera_1', 'camera_2'),
                            serials=('SERIAL1', 'SERIAL2'),
                        ),
                        context=context,
                    )
                    node._check_cameras()
                    self.assertEqual(len(FakeProcess.popen_calls), 1)
                    self.assertIn('camera_name:=camera_1', FakeProcess.popen_calls[0].args[0])
                    self.assertIsNone(node._states['camera_2'].process)

                    node._record_image('camera_1', 'color')
                    node._check_cameras()
                    self.assertEqual(len(FakeProcess.popen_calls), 1)
                    self.assertIsNone(node._states['camera_2'].process)

                    node._record_image('camera_1', 'depth')
                    node._check_cameras()
                    self.assertEqual(len(FakeProcess.popen_calls), 2)
                    self.assertIn('camera_name:=camera_2', FakeProcess.popen_calls[1].args[0])
                    self.assertIsNotNone(node._states['camera_2'].process)

                    node._record_image('camera_2', 'color')
                    node._record_image('camera_2', 'depth')
                    node._check_cameras()
                    self.assertEqual(node._phase, 'healthy')
                    events = [
                        json.loads(line)['event']
                        for line in workspace.joinpath(
                            'logs/orbbec_camera_launcher/events.jsonl'
                        ).read_text(encoding='utf-8').splitlines()
                    ]
                    self.assertEqual(events.count('camera_process_started'), 2)
                    self.assertEqual(events.count('camera_streams_ready'), 2)
                    self.assertIn('next_camera_starting', events)
                    self.assertIn('camera_set_healthy', events)
            finally:
                if node is not None:
                    node.shutdown()
                    node.destroy_node()
                context.try_shutdown()

    def test_camera_two_never_starts_when_camera_one_times_out(self):
        for received_stream in (None, 'color', 'depth', 'both'):
            with self.subTest(received_stream=received_stream):
                temporary_directory = tempfile.TemporaryDirectory()
                self.addCleanup(temporary_directory.cleanup)
                workspace = create_workspace(Path(temporary_directory.name))
                FakeProcess.popen_calls = []
                context = Context()
                rclpy.init(context=context)
                node = None
                try:
                    with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}), patch.object(
                        camera_watchdog,
                        'scan_connected_serials',
                        return_value=(0, ['SERIAL1', 'SERIAL2'], 'both connected'),
                    ), patch.object(camera_watchdog.subprocess, 'Popen', FakeProcess):
                        node = camera_watchdog.CameraWatchdog(
                            parameter_overrides=parameters(
                                workspace,
                                names=('camera_1', 'camera_2'),
                                serials=('SERIAL1', 'SERIAL2'),
                            ),
                            context=context,
                        )
                        node._check_cameras()
                        if received_stream == 'both':
                            node._record_image('camera_1', 'color')
                            node._record_image('camera_1', 'depth')
                        elif received_stream is not None:
                            node._record_image('camera_1', received_stream)
                        node._states['camera_1'].startup_started_at = time.monotonic() - 6.0
                        node._check_cameras()
                        self.assertEqual(len(FakeProcess.popen_calls), 1)
                        self.assertEqual(node._phase, 'waiting')
                finally:
                    if node is not None:
                        node.shutdown()
                        node.destroy_node()
                    context.try_shutdown()

    def test_camera_two_receives_an_independent_startup_deadline(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = create_workspace(Path(temporary))
            FakeProcess.popen_calls = []
            context = Context()
            rclpy.init(context=context)
            node = None
            try:
                with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}), patch.object(
                    camera_watchdog,
                    'scan_connected_serials',
                    return_value=(0, ['SERIAL1', 'SERIAL2'], 'both connected'),
                ), patch.object(camera_watchdog.subprocess, 'Popen', FakeProcess):
                    node = camera_watchdog.CameraWatchdog(
                        parameter_overrides=parameters(
                            workspace,
                            names=('camera_1', 'camera_2'),
                            serials=('SERIAL1', 'SERIAL2'),
                            health_timeout=10.0,
                        ),
                        context=context,
                    )
                    node._check_cameras()
                    first = node._states['camera_1']
                    first.startup_started_at = time.monotonic() - 100.0
                    node._record_image('camera_1', 'color')
                    node._record_image('camera_1', 'depth')
                    # Preserve valid first-arrival timestamps while proving the
                    # complete-set start time is not reused for camera 2.
                    first.startup_color_at = first.startup_started_at + 0.1
                    first.startup_depth_at = first.startup_started_at + 0.2
                    before_second = time.monotonic()
                    node._check_cameras()
                    second = node._states['camera_2']
                    self.assertIsNotNone(second.startup_started_at)
                    self.assertGreaterEqual(second.startup_started_at, before_second)
                    self.assertEqual(node._phase, 'starting')
            finally:
                if node is not None:
                    node.shutdown()
                    node.destroy_node()
                context.try_shutdown()

    def test_camera_one_remains_supervised_while_camera_two_starts(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = create_workspace(Path(temporary))
            FakeProcess.popen_calls = []
            context = Context()
            rclpy.init(context=context)
            node = None
            try:
                with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}), patch.object(
                    camera_watchdog,
                    'scan_connected_serials',
                    return_value=(0, ['SERIAL1', 'SERIAL2'], 'both connected'),
                ), patch.object(camera_watchdog.subprocess, 'Popen', FakeProcess):
                    node = camera_watchdog.CameraWatchdog(
                        parameter_overrides=parameters(
                            workspace,
                            names=('camera_1', 'camera_2'),
                            serials=('SERIAL1', 'SERIAL2'),
                        ),
                        context=context,
                    )
                    node._check_cameras()
                    node._record_image('camera_1', 'color')
                    node._record_image('camera_1', 'depth')
                    node._check_cameras()
                    first = node._states['camera_1']
                    first.last_depth_at = time.monotonic() - 2.0
                    node._check_cameras()
                    self.assertEqual(node._phase, 'waiting')
                    self.assertIsNone(node._states['camera_1'].process)
                    self.assertIsNone(node._states['camera_2'].process)
            finally:
                if node is not None:
                    node.shutdown()
                    node.destroy_node()
                context.try_shutdown()

    def test_camera_two_startup_timeout_stops_the_complete_set(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = create_workspace(Path(temporary))
            FakeProcess.popen_calls = []
            context = Context()
            rclpy.init(context=context)
            node = None
            try:
                with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}), patch.object(
                    camera_watchdog,
                    'scan_connected_serials',
                    return_value=(0, ['SERIAL1', 'SERIAL2'], 'both connected'),
                ), patch.object(camera_watchdog.subprocess, 'Popen', FakeProcess):
                    node = camera_watchdog.CameraWatchdog(
                        parameter_overrides=parameters(
                            workspace,
                            names=('camera_1', 'camera_2'),
                            serials=('SERIAL1', 'SERIAL2'),
                        ),
                        context=context,
                    )
                    node._check_cameras()
                    node._record_image('camera_1', 'color')
                    node._record_image('camera_1', 'depth')
                    node._check_cameras()
                    node._states['camera_2'].startup_started_at = time.monotonic() - 6.0
                    node._check_cameras()

                    self.assertEqual(node._phase, 'waiting')
                    self.assertTrue(all(state.process is None for state in node._states.values()))
                    events = [
                        json.loads(line)
                        for line in workspace.joinpath(
                            'logs/orbbec_camera_launcher/events.jsonl'
                        ).read_text(encoding='utf-8').splitlines()
                    ]
                    timeout = next(
                        event for event in events
                        if event['event'] == 'camera_startup_timeout'
                    )
                    self.assertEqual(timeout['camera'], 'camera_2')
                    self.assertEqual(timeout['startup_order'], 2)
            finally:
                if node is not None:
                    node.shutdown()
                    node.destroy_node()
                context.try_shutdown()

    def test_process_and_stream_failures_always_stop_both_owned_cameras(self):
        faults = (
            ('starting_camera_1_process', False),
            ('starting_camera_2_process', False),
            ('starting_camera_1_color', False),
            ('starting_camera_1_depth', False),
            ('starting_camera_1_both_streams', False),
            ('starting_both_processes', False),
            ('healthy_camera_1_process', True),
            ('healthy_camera_2_process', True),
            ('healthy_camera_1_color', True),
            ('healthy_camera_1_depth', True),
            ('healthy_camera_2_color', True),
            ('healthy_camera_2_depth', True),
            ('healthy_both_streams', True),
            ('healthy_both_processes', True),
        )
        for fault, enter_healthy in faults:
            with self.subTest(fault=fault):
                temporary_directory = tempfile.TemporaryDirectory()
                self.addCleanup(temporary_directory.cleanup)
                workspace = create_workspace(Path(temporary_directory.name))
                FakeProcess.popen_calls = []
                context = Context()
                rclpy.init(context=context)
                node = None
                try:
                    with patch.dict(
                        os.environ,
                        {'ROS_LOCALHOST_ONLY': '1'},
                    ), patch.object(
                        camera_watchdog,
                        'scan_connected_serials',
                        return_value=(0, ['SERIAL1', 'SERIAL2'], 'both connected'),
                    ), patch.object(camera_watchdog.subprocess, 'Popen', FakeProcess):
                        node = camera_watchdog.CameraWatchdog(
                            parameter_overrides=parameters(
                                workspace,
                                names=('camera_1', 'camera_2'),
                                serials=('SERIAL1', 'SERIAL2'),
                            ),
                            context=context,
                        )
                        node._check_cameras()
                        node._record_image('camera_1', 'color')
                        node._record_image('camera_1', 'depth')
                        node._check_cameras()
                        if enter_healthy:
                            node._record_image('camera_2', 'color')
                            node._record_image('camera_2', 'depth')
                            node._check_cameras()
                            self.assertEqual(node._phase, 'healthy')

                        if 'both_processes' in fault:
                            node._states['camera_1'].process.returncode = 8
                            node._states['camera_2'].process.returncode = 9
                        elif 'camera_1_process' in fault:
                            node._states['camera_1'].process.returncode = 8
                        elif 'camera_2_process' in fault:
                            node._states['camera_2'].process.returncode = 9
                        elif 'both_streams' in fault:
                            names = (
                                ('camera_1',)
                                if 'camera_1' in fault
                                else ('camera_1', 'camera_2')
                            )
                            for camera_name in names:
                                node._states[camera_name].last_color_at = (
                                    time.monotonic() - 2.0
                                )
                                node._states[camera_name].last_depth_at = (
                                    time.monotonic() - 2.0
                                )
                        else:
                            camera_name = 'camera_1' if 'camera_1' in fault else 'camera_2'
                            stream = 'color' if fault.endswith('color') else 'depth'
                            setattr(
                                node._states[camera_name],
                                f'last_{stream}_at',
                                time.monotonic() - 2.0,
                            )
                        node._check_cameras()

                        self.assertEqual(node._phase, 'waiting')
                        self.assertTrue(
                            all(state.process is None for state in node._states.values())
                        )
                finally:
                    if node is not None:
                        node.shutdown()
                        node.destroy_node()
                    context.try_shutdown()

    def test_failure_stops_both_then_retries_from_camera_one_after_three_seconds(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = create_workspace(Path(temporary))
            FakeProcess.popen_calls = []
            context = Context()
            rclpy.init(context=context)
            node = None
            try:
                with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}), patch.object(
                    camera_watchdog,
                    'scan_connected_serials',
                    return_value=(0, ['SERIAL1', 'SERIAL2'], 'both connected'),
                ), patch.object(camera_watchdog.subprocess, 'Popen', FakeProcess):
                    node = camera_watchdog.CameraWatchdog(
                        parameter_overrides=parameters(
                            workspace,
                            names=('camera_1', 'camera_2'),
                            serials=('SERIAL1', 'SERIAL2'),
                        ),
                        context=context,
                    )
                    node._check_cameras()
                    node._record_image('camera_1', 'color')
                    node._record_image('camera_1', 'depth')
                    node._check_cameras()
                    self.assertEqual(len(FakeProcess.popen_calls), 2)

                    failure_time = time.monotonic()
                    FakeProcess.popen_calls[1].returncode = 7
                    node._check_cameras()
                    self.assertEqual(node._phase, 'waiting')
                    self.assertAlmostEqual(
                        node._next_attempt_at - failure_time,
                        3.0,
                        delta=0.1,
                    )
                    self.assertEqual(FakeProcess.popen_calls[0].returncode, 0)
                    self.assertEqual(FakeProcess.popen_calls[1].returncode, 7)

                    node._check_cameras()
                    self.assertEqual(len(FakeProcess.popen_calls), 2)
                    node._next_attempt_at = 0.0
                    node._check_cameras()
                    self.assertEqual(len(FakeProcess.popen_calls), 3)
                    self.assertIn(
                        'camera_name:=camera_1',
                        FakeProcess.popen_calls[2].args[0],
                    )
                    self.assertEqual(node._attempt_count, 2)
            finally:
                if node is not None:
                    node.shutdown()
                    node.destroy_node()
                context.try_shutdown()

    def test_retired_owned_endpoints_are_ignored_but_unknown_endpoints_fail(self):
        class FakeEndpoint:
            def __init__(self, gid, name='camera_1', namespace='/camera_1'):
                self.endpoint_gid = list(gid)
                self.node_name = name
                self.node_namespace = namespace

        with tempfile.TemporaryDirectory() as temporary:
            workspace = create_workspace(Path(temporary))
            FakeProcess.popen_calls = []
            context = Context()
            rclpy.init(context=context)
            node = None
            graph = {}
            try:
                with patch.dict(os.environ, {'ROS_LOCALHOST_ONLY': '1'}), patch.object(
                    camera_watchdog,
                    'scan_connected_serials',
                    return_value=(0, ['SERIAL1', 'SERIAL2'], 'connected'),
                ), patch.object(camera_watchdog.subprocess, 'Popen', FakeProcess):
                    node = camera_watchdog.CameraWatchdog(
                        parameter_overrides=parameters(
                            workspace,
                            names=('camera_1', 'camera_2'),
                            serials=('SERIAL1', 'SERIAL2'),
                        ),
                        context=context,
                    )
                    node.get_publishers_info_by_topic = lambda topic: graph.get(topic, [])
                    node._check_cameras()
                    owned_gid = bytes(range(1, 25))
                    graph['/camera_1/color/image_raw'] = [FakeEndpoint(owned_gid)]
                    node._check_cameras()
                    node._fail_attempt('injected complete-set failure')
                    node._next_attempt_at = 0.0
                    node._check_cameras()
                    self.assertEqual(node._attempt_count, 2)
                    self.assertEqual(node._phase, 'starting')

                    unknown_gid = bytes(range(25, 49))
                    graph['/camera_1/depth/image_raw'] = [
                        FakeEndpoint(unknown_gid, 'external', '/external')
                    ]
                    node._fail_attempt('prepare unknown endpoint attempt')
                    node._next_attempt_at = 0.0
                    node._check_cameras()
                    self.assertTrue(node.terminal_failure)
                    self.assertIn('unowned publishers', node._last_error)
            finally:
                if node is not None:
                    node.shutdown()
                    node.destroy_node()
                context.try_shutdown()

    def test_gui_reports_supervisor_terminal_failure_even_with_zero_wrapper_exit(self):
        app = camera_launcher_gui.CameraLauncherApp.__new__(
            camera_launcher_gui.CameraLauncherApp
        )
        app.root = FakeAfterRoot()
        app.event_logger = FakeEventLogger()
        app.status_var = FakeStringVar()
        app.process_var = FakeStringVar()
        app.supervisor_process = FakeProcess()
        app.supervisor_process.returncode = 0
        app.supervisor_process_group = app.supervisor_process.pid
        app.supervisor_event_cursor = 0
        app.event_logger.records_after_cursor = [{
            'event': 'supervisor_stopped',
            'terminal_failure': True,
            'launch_parent_pid': app.supervisor_process.pid,
        }]
        app.closing = False
        app._append_output = Mock()
        app._update_button_state = Mock()

        with patch.object(app, '_process_group_alive', return_value=False):
            app._poll_supervisor()

        self.assertIn('failed', app.status_var.value)
        self.assertEqual(app.event_logger.records[-1][0], 'ERROR')
        self.assertTrue(
            app.event_logger.records[-1][3]['supervisor_terminal_failure']
        )


if __name__ == '__main__':
    unittest.main()
