"""Tray reception failure is visible and logged instead of leaving a live-looking UI."""

import json
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tray_perception.core import EventLogger
from tray_perception.execution import spin_checked


def test_real_callback_exception_disarms_and_logs_traceback(tmp_path):
    context = Context()
    rclpy.init(context=context)
    node = Node('tray_executor_failure_test', context=context)
    node.shutdown_requested = threading.Event()
    node.fatal_error = ''
    node.requests = SimpleNamespace(disarm=Mock())
    node.events = EventLogger(tmp_path)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(node)

    def broken_callback():
        raise ValueError('synthetic callback failure')

    node.create_timer(.02, broken_callback)
    thread = threading.Thread(target=spin_checked, args=(node, executor), daemon=True)
    try:
        thread.start()
        thread.join(2)
        assert not thread.is_alive()
        assert 'synthetic callback failure' in node.fatal_error
        node.requests.disarm.assert_called_once_with(node.fatal_error)
        event = json.loads(node.events.path.read_text().splitlines()[-1])
        assert event['event'] == 'tray_executor_failed' and event['level'] == 'FATAL'
        assert 'broken_callback' in event['traceback']
        assert 'ValueError: synthetic callback failure' in event['traceback']
    finally:
        node.shutdown_requested.set()
        executor.shutdown(timeout_sec=2)
        thread.join(2)
        node.destroy_node()
        rclpy.shutdown(context=context)


@pytest.mark.parametrize('stopping,context_ok,fails', [
    (True, True, False), (True, True, True), (False, False, True),
])
def test_intentional_shutdown_does_not_report_a_reception_failure(stopping, context_ok, fails):
    node = SimpleNamespace(shutdown_requested=threading.Event(),
                           context=SimpleNamespace(ok=lambda: context_ok),
                           fatal_error='', requests=Mock(), events=Mock())
    if stopping:
        node.shutdown_requested.set()
    executor = SimpleNamespace(spin=Mock(side_effect=RuntimeError('shutdown') if fails else None))
    spin_checked(node, executor)
    assert not node.fatal_error
    node.requests.disarm.assert_not_called()
    node.events.record.assert_not_called()


def test_unexpected_executor_return_is_reported():
    node = SimpleNamespace(shutdown_requested=threading.Event(),
                           context=SimpleNamespace(ok=lambda: True),
                           fatal_error='', requests=Mock(), events=Mock())
    spin_checked(node, SimpleNamespace(spin=lambda: None))
    assert 'executor exited unexpectedly' in node.fatal_error
    node.requests.disarm.assert_called_once()
