import asyncio
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

import pytest
from PyQt5 import QtCore, QtWidgets

from remote_setup import config, probes, runner
from remote_setup.gui import Window
from orbbec_camera_launcher import project_config

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def values():
    return dict(zip(config.KEYS, (
        '192.0.2.1', '5672', 'false', '/', 'test-user', 'private=pass&word',
        'http://192.0.2.1:8000', '', '', 'belt_1', '3', '1')))


@pytest.fixture
def root(tmp_path):
    text = (ROOT / '.env.example').read_text()
    text = '\n'.join(line for line in text.splitlines() if not line.startswith('REMOTE_')) + '\n'
    (tmp_path / '.env.example').write_text((ROOT / '.env.example').read_text())
    (tmp_path / '.env').write_text(text)
    return tmp_path


def test_legacy_offline_env_accepts_optional_remote_group(root, values):
    previous = (root / '.env').read_bytes()
    project_config.validate_project_config(project_config._parse_env_file(root / '.env'),
                                           require_camera_ready=False)
    assert set(config.read(root).values()) == {''}
    with pytest.raises(ValueError, match='required'):
        config.load(root)
    settings = config.save(values, root)
    assert (root / '.env').read_bytes().startswith(previous)
    assert config.load(root) == settings
    assert (root / '.env').stat().st_mode & 0o777 == 0o600
    project_config.validate_project_config(project_config._parse_env_file(root / '.env'),
                                           require_camera_ready=False)
    assert values['REMOTE_AMQP_PASSWORD'] not in repr(settings)


@pytest.mark.parametrize('key,value', [
    ('SERVER_HOST', 'http://host'), ('SERVER_HOST', '-bad'), ('AMQP_PORT', '0'),
    ('AMQP_PORT', '65536'), ('AMQP_PORT', '05672'), ('AMQP_TLS', 'yes'),
    ('ARM_NUMBER', '9'), ('ARM_NUMBER', '1.0'), ('BELT_ID', 'belt.#'),
    ('CONNECT_TIMEOUT_SEC', 'nan'), ('CONNECT_TIMEOUT_SEC', '0'),
    ('CONNECT_TIMEOUT_SEC', '61'), ('CONVEYOR_API_URL', 'ftp://host'),
    ('CONVEYOR_API_URL', 'https://user:secret@host'), ('CONVEYOR_API_URL', 'http://host?key=x'),
    ('CONVEYOR_API_URL', 'http://host:99999'), ('AMQP_PASSWORD', ' secret'),
    ('AMQP_PASSWORD', 'secret\nNEW_KEY=yes'), ('AMQP_PASSWORD', 'secret\u2028NEW_KEY=yes'),
    ('AMQP_PASSWORD', '\x00secret'),
])
def test_invalid_save_never_changes_env_or_echoes_secret(root, values, key, value):
    previous = (root / '.env').read_bytes()
    values['REMOTE_' + key] = value
    with pytest.raises(ValueError) as exc:
        config.save(values, root)
    assert value not in str(exc.value) if len(value) > 5 else True
    assert (root / '.env').read_bytes() == previous


def test_save_replaces_only_remote_lines_and_cleans_failed_temp(root, values, monkeypatch):
    config.save(values, root)
    path = root / '.env'
    previous = path.read_bytes()
    values['REMOTE_ARM_NUMBER'] = '8'

    def conflict(_fd):
        path.write_bytes(previous + b'# concurrent camera edit\n')
    monkeypatch.setattr(config.os, 'fsync', conflict)
    with pytest.raises(ValueError, match='changed while saving'):
        config.save(values, root)
    assert path.read_bytes() == previous + b'# concurrent camera edit\n'
    assert not list(root.glob('.remote_setup-*'))


def test_save_preserves_comments_and_other_values(root, values):
    config.save(values, root)
    path = root / '.env'
    before = [line for line in path.read_bytes().splitlines() if not line.startswith(b'REMOTE_')]
    values['REMOTE_AMQP_PASSWORD'] = 'changed=x=y'
    config.save(values, root)
    assert config.load(root)['AMQP_PASSWORD'] == 'changed=x=y'
    assert [line for line in path.read_bytes().splitlines() if not line.startswith(b'REMOTE_')] == before


def test_broker_auth_passive_only_and_cleanup(values):
    channel = SimpleNamespace(declare_exchange=AsyncMock())
    connection = SimpleNamespace(channel=AsyncMock(return_value=channel), close=AsyncMock())
    connect = AsyncMock(return_value=connection)
    result = asyncio.run(probes.broker(config.validate(values), connect))
    assert result['ok']
    connect.assert_awaited_once()
    assert connect.call_args.kwargs['password'] == values['REMOTE_AMQP_PASSWORD']
    assert connect.call_args.kwargs['virtualhost'] == '/'
    channel.declare_exchange.assert_awaited_once_with('catarm.events', passive=True, timeout=1.)
    connection.close.assert_awaited_once()


def test_broker_failure_closes_and_never_exposes_server_text(values):
    channel = SimpleNamespace(declare_exchange=AsyncMock(side_effect=RuntimeError('private=pass&word')))
    connection = SimpleNamespace(channel=AsyncMock(return_value=channel), close=AsyncMock())
    result = asyncio.run(probes.broker(config.validate(values), AsyncMock(return_value=connection)))
    assert not result['ok'] and 'exchange' in result['message']
    assert 'private' not in json.dumps(result)
    connection.close.assert_awaited_once()


def test_broker_tls_verifies_and_timeout_has_no_retry(values):
    values['REMOTE_AMQP_TLS'] = 'true'
    connect = AsyncMock(side_effect=TimeoutError('private=pass&word'))
    result = asyncio.run(probes.broker(config.validate(values), connect))
    assert not result['ok'] and 'timeout' in result['message']
    context = connect.call_args.kwargs['ssl_context']
    assert context.check_hostname and context.verify_mode == 2
    assert connect.await_count == 1


def test_broker_cancellation_closes_connection(values):
    channel = SimpleNamespace(declare_exchange=AsyncMock(side_effect=asyncio.CancelledError()))
    connection = SimpleNamespace(channel=AsyncMock(return_value=channel), close=AsyncMock())
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(probes.broker(config.validate(values), AsyncMock(return_value=connection)))
    connection.close.assert_awaited_once()


@pytest.mark.parametrize('body,ok', [
    ({'status': 'ok', 'service': 'conveyor-server', 'belt_id': 'belt_1'}, True),
    ({'status': 'ok', 'service': 'conveyor-server', 'belt_id': 'other'}, False),
    ({'status': 'ok', 'service': 'other', 'belt_id': 'belt_1'}, False),
    ({'status': 'failed', 'service': 'conveyor-server', 'belt_id': 'belt_1'}, False),
    ([], False),
])
def test_http_checks_identity_without_sending_secrets(values, body, ok):
    response = Mock(status=200, read=Mock(return_value=json.dumps(body).encode()))
    context = Mock(__enter__=Mock(return_value=response), __exit__=Mock(return_value=False))
    opener = SimpleNamespace(open=Mock(return_value=context))
    result = probes.http_health(config.validate(values), opener)
    assert result['ok'] is ok
    req = opener.open.call_args.args[0]
    assert req.full_url == 'http://192.0.2.1:8000/api/health'
    assert req.get_method() == 'GET'
    assert req.headers == {'Accept': 'application/json'}
    response.read.assert_called_once_with(65537)


@pytest.mark.parametrize('data', [b'{broken', b'\xff', b'x' * 65537])
def test_http_rejects_bad_or_large_response(values, data):
    response = Mock(status=200, read=Mock(return_value=data))
    opener = SimpleNamespace(open=Mock(return_value=Mock(
        __enter__=Mock(return_value=response), __exit__=Mock(return_value=False))))
    assert not probes.http_health(config.validate(values), opener)['ok']


def test_http_redirect_is_refused():
    assert probes.NoRedirect().redirect_request(None, None, 302, '', {}, 'http://other') is None


def test_runner_bound_and_secrets_only_on_stdin(values, monkeypatch):
    run = Mock(side_effect=subprocess.TimeoutExpired('test', 10))
    monkeypatch.setattr(runner.subprocess, 'run', run)
    ok, message = runner.run(config.validate(values))
    assert not ok and 'deadline' in message
    assert values['REMOTE_AMQP_PASSWORD'] not in repr(run.call_args.args)
    assert run.call_args.kwargs['timeout'] == 10
    assert json.loads(run.call_args.kwargs['input']) == values


@pytest.fixture
def window(root):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = Window(root)
    yield widget
    widget.close()
    app.processEvents()


def test_ui_first_open_never_connects_masks_secrets_and_requires_saved_config(window, values):
    assert window.process is None
    for key in config.SECRETS:
        assert window.fields[key].echoMode() == QtWidgets.QLineEdit.Password
    window.test()
    assert 'required' in window.output.toPlainText() and window.process is None
    for key, value in values.items():
        window.fields[key].setText(value)
    window.save()
    assert 'saved' in window.output.toPlainText()
    window.fields['REMOTE_ARM_NUMBER'].setText('4')
    window.test()
    assert 'Unsaved edits' in window.output.toPlainText() and window.process is None


def test_ui_test_responsive_cancel_kills_only_owned_child(window, values, monkeypatch):
    config.save(values, window.root)
    window.reload()
    monkeypatch.setattr(runner, 'COMMAND', [
        os.sys.executable, '-c', 'import sys,time; sys.stdin.read(); time.sleep(30)'])
    window.test()
    process = window.process
    assert process.waitForStarted(2000)
    assert window.cancel_button.isEnabled() and not window.save_button.isEnabled()
    window.cancel('Canceled fixture')
    assert process.state() == QtCore.QProcess.NotRunning and window.process is None
    assert window.save_button.isEnabled()


def test_ui_harmless_result_is_logged_without_settings(window, values, monkeypatch):
    config.save(values, window.root)
    window.reload()
    output = json.dumps({'ok': True, 'results': [{'check': 'fixture', 'ok': True, 'message': 'verified'}]})
    monkeypatch.setattr(runner, 'COMMAND', [
        os.sys.executable, '-c', 'import sys; sys.stdin.read(); print(' + repr(output) + ')'])
    window.test()
    process = window.process
    assert process.waitForFinished(2000)
    assert window.process is None and 'PASS' in window.output.toPlainText()
    log = (window.root / 'logs/remote_setup/events.jsonl').read_text()
    assert 'test_passed' in log and values['REMOTE_AMQP_PASSWORD'] not in log


def test_all_canonical_readers_accept_remote_keys_without_exporting_secrets(root, values):
    import runpy
    import shutil
    scripts = root / 'scripts'
    scripts.mkdir()
    shutil.copyfile(ROOT / 'scripts/source_ros_workspace.bash', scripts / 'source_ros_workspace.bash')
    (root / 'install').mkdir()
    (root / 'install/setup.bash').write_text('# fixture only; never launch hardware\n')
    launch = root / 'src' / 'dobot_bringup_ros2.launch.py'
    launch.parent.mkdir()
    (launch.parent / 'package.xml').write_text('<package><name>fixture</name></package>')
    source = ROOT / 'src/DOBOT_6Axis_ROS2_V4/dobot_bringup_v4/launch/dobot_bringup_ros2.launch.py'
    shutil.copyfile(source, launch)
    module = runpy.run_path(str(launch))
    assert set(config.KEYS).issubset(module['_SUPPORTED_ENV_KEYS'])
    for deploy in (False, True):
        if deploy:
            config.save(values, root)
        module['_load_project_env']()  # Validation only; never generate or run a launch.
        assert 'REMOTE_AMQP_PASSWORD' not in os.environ
        result = subprocess.run(
            ['bash', '-c', 'source scripts/source_ros_workspace.bash && test -z "${REMOTE_AMQP_PASSWORD+x}"'],
            cwd=root, capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        assert values['REMOTE_AMQP_PASSWORD'] not in result.stdout + result.stderr
    path = root / '.env'
    path.write_text(path.read_text() + 'REMOTE_UNKNOWN_KEY=bad\n')
    with pytest.raises(RuntimeError, match='Unsupported'):
        module['_load_project_env']()


def test_headless_uses_saved_settings_and_exit_code_without_gui(root, values, monkeypatch, capsys):
    import rclpy
    from remote_setup import main
    config.save(values, root)
    monkeypatch.setattr(config, 'project_root', lambda: root)
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setattr(rclpy, 'init', Mock())
    monkeypatch.setattr(rclpy, 'create_node', Mock(return_value=Mock()))
    monkeypatch.setattr(rclpy, 'ok', lambda: False)
    run = Mock(return_value=(False, 'fixture failure'))
    monkeypatch.setattr(runner, 'run', run)
    assert main.main(['--headless']) == 1
    assert run.call_args.args[0].values == values
    assert values['REMOTE_AMQP_PASSWORD'] not in capsys.readouterr().out
    run.return_value = (True, 'fixture pass')
    assert main.main(['--headless']) == 0
    path = root / '.env'
    path.write_text(path.read_text().replace('REMOTE_ARM_NUMBER=3', 'REMOTE_ARM_NUMBER=9'))
    assert main.main(['--headless']) == 2
    assert run.call_count == 2


def test_ui_deadline_and_close_stop_owned_test(window, values, monkeypatch):
    import time
    config.save(values, window.root)
    window.reload()
    monkeypatch.setattr(runner, 'COMMAND', [
        os.sys.executable, '-c', 'import sys,time; sys.stdin.read(); time.sleep(30)'])
    window.test()
    process = window.process
    assert process.waitForStarted(2000)
    window.timer.start(1)
    deadline = time.monotonic() + 2
    while window.process is not None and time.monotonic() < deadline:
        QtWidgets.QApplication.processEvents()
    assert window.process is None and 'timed out' in window.output.toPlainText()
    window.test()
    process = window.process
    assert process.waitForStarted(2000)
    window.close()
    assert process.state() == QtCore.QProcess.NotRunning


def test_log_failure_does_not_change_test_outcome(root, monkeypatch, capsys):
    from remote_setup import events
    monkeypatch.setattr(events, '_record', Mock(side_effect=OSError('do not expose details')))
    events.record(root, 'test_passed', 'fixture')
    assert capsys.readouterr().err == 'Remote Setup: package event log is unavailable.\n'


def test_worker_timeout_still_reports_independent_http_result(values, monkeypatch, capsys):
    import io
    import logging
    old_disable = logging.root.manager.disable
    monkeypatch.setattr(probes.sys, 'stdin', io.StringIO(json.dumps(values)))
    monkeypatch.setattr(probes, 'broker', AsyncMock(side_effect=asyncio.TimeoutError()))
    health = Mock(return_value={'check': 'Conveyor API', 'ok': True, 'message': 'fixture healthy'})
    monkeypatch.setattr(probes, 'http_health', health)
    try:
        assert probes.main() == 1
        result = json.loads(capsys.readouterr().out)
        assert not result['ok'] and result['results'][1]['ok']
        assert 'timed out' in result['results'][0]['message']
        health.assert_called_once()
    finally:
        logging.disable(old_disable)
