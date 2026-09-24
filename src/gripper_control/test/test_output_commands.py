"""Exercise diagnostic output commands without ROS nodes, a display or hardware."""

from concurrent.futures import Future
import queue
import threading
from types import MethodType, SimpleNamespace

import pytest

from gripper_control import gripper_control_gui as gui


class Widget:
    def __init__(self, value=''):
        self.value = value
        self.options = {}

    def get(self):
        return self.value

    def set(self, value):
        self.value = value

    def configure(self, **options):
        self.options.update(options)


class Clock:
    def __init__(self):
        self.now = 0.0
        self.sequence = 0
        self.pending = {}

    def after(self, milliseconds, callback):
        self.sequence += 1
        self.pending[self.sequence] = self.now + milliseconds / 1000, callback
        return self.sequence

    def after_cancel(self, identifier):
        self.pending.pop(identifier, None)

    def advance(self, milliseconds):
        end = self.now + milliseconds / 1000
        while self.pending:
            identifier = min(self.pending, key=lambda key: self.pending[key][0])
            when, callback = self.pending[identifier]
            if when > end:
                break
            self.now = when
            del self.pending[identifier]
            callback()
        self.now = end

    def destroy(self):
        self.pending.clear()


@pytest.fixture
def app(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(gui, 'time', SimpleNamespace(monotonic=lambda: clock.now))
    timers = []

    class ResponseTimer:
        def __init__(self, interval, callback):
            self.interval = interval
            self.callback = callback
            self.cancelled = False

        def start(self):
            timers.append(self)

        def cancel(self):
            self.cancelled = True

    monkeypatch.setattr(gui.threading, 'Timer', ResponseTimer)
    calls = []
    events = []

    def dispatch(request):
        future = Future()
        calls.append((request, future))
        return future

    snapshot = dict(ready=True, health_text='ready', digital_input_bits=0,
                    digital_output_bits=0)
    logger = SimpleNamespace(record=lambda *event: events.append(event))
    node = SimpleNamespace(
        _do_client=SimpleNamespace(service_is_ready=lambda: True, call_async=dispatch),
        _event_logger=logger, event_logger=logger, io_snapshot=lambda: snapshot,
        destroy_node=lambda: None)
    node.send_do = MethodType(gui.GripperControlNode.send_do, node)
    instance = gui.GripperControlApp.__new__(gui.GripperControlApp)
    instance._root = clock
    instance._node = node
    instance._executor = SimpleNamespace(shutdown=lambda: None)
    instance._spin_thread = SimpleNamespace(is_alive=lambda: False)
    instance._stop_event = threading.Event()
    instance._ui_queue = queue.Queue()
    instance._closing = instance._finalized = False
    instance._operation_name = None
    instance._operation_token = 0
    instance._last_runtime_ready = instance._last_output_bits = None
    instance._last_input_bits = 0
    instance._status_var = Widget()
    instance._service_var = Widget()
    instance._channels = {
        index: dict(button=Widget(), duration_var=Widget('0'), duration_entry=Widget(),
                    state_label=Widget(), on=False, pending=False, timer_after_id=None)
        for index in gui.OUTPUT_CHANNELS
    }
    instance.calls = calls
    instance.events = events
    instance.snapshot = snapshot
    instance.timers = timers
    return instance


def reply(app, result=0, index=-1):
    app.calls[index][1].set_result(SimpleNamespace(res=result))
    while not app._ui_queue.empty():
        app._ui_queue.get_nowait()()


@pytest.mark.parametrize('result', [0, -1, -2, 1])
def test_only_zero_service_result_is_accepted(app, result):
    completed = []
    app._node.send_do(13, 1, lambda *args: completed.append(args))
    request = app.calls[0][0]
    assert (request.index, request.status, request.time) == (13, 1, 0)
    reply(app, result)
    assert completed == [(result == 0, result, f'res={result}')]
    assert app.timers[0].cancelled


def test_timeout_completes_once_even_with_late_success(app):
    completed = []
    app._node.send_do(1, 1, lambda *args: completed.append(args))
    assert app.timers[0].interval == 3.0
    app.timers[0].callback()
    reply(app)
    assert completed == [(False, -1, 'no response within 3.0 seconds')]


@pytest.mark.parametrize('response', [None, RuntimeError('transport failed')])
def test_empty_or_failed_response_is_rejected(app, response):
    completed = []
    app._node.send_do(1, 1, lambda *args: completed.append(args))
    future = app.calls[0][1]
    if isinstance(response, Exception):
        future.set_exception(response)
    else:
        future.set_result(response)
    assert len(completed) == 1 and completed[0][0] is False
    assert app.timers[0].cancelled


def test_auto_off_starts_after_feedback_confirmation(app):
    app._channels[1]['duration_var'].set('250')
    app._on_toggle(1)
    app._on_toggle(2)
    assert len(app.calls) == 1  # An active command blocks another output action.
    reply(app)
    assert app._channels[1]['timer_after_id'] is None
    assert app._operation_name is not None
    app.snapshot['digital_output_bits'] = 1
    app._refresh_feedback()
    app._root.advance(50)
    assert app._operation_name is None
    assert app._status_var.get() == 'DO1: ON confirmed'
    app._root.advance(249)
    assert len(app.calls) == 1
    app._root.advance(1)
    assert [(request.index, request.status) for request, _ in app.calls] == [(1, 1), (1, 0)]
    app.snapshot['digital_output_bits'] = 0
    reply(app)
    assert app._status_var.get() == 'DO1: automatic OFF confirmed'


def test_service_rejection_never_arms_auto_off(app):
    app._channels[1]['duration_var'].set('250')
    app._on_toggle(1)
    reply(app, -2)
    assert app._operation_name is None
    assert app._channels[1]['timer_after_id'] is None
    assert 'failed at DO1=1: res=-2' in app._status_var.get()
    app._root.advance(2000)
    assert len(app.calls) == 1


@pytest.mark.parametrize('stale', [False, True])
def test_missing_or_stale_feedback_never_confirms_output(app, stale):
    app._channels[1]['duration_var'].set('250')
    app._on_toggle(1)
    if stale:
        app.snapshot.update(ready=False, digital_output_bits=1)
    reply(app)
    app._root.advance(1600)
    assert app._operation_name is None
    assert 'FeedInfo did not confirm DO1=1' in app._status_var.get()
    assert app._channels[1]['timer_after_id'] is None
    assert len(app.calls) == 1


def test_unavailable_feedback_blocks_dispatch(app):
    app.snapshot.update(ready=False, health_text='FeedInfo stale')
    app._on_toggle(1)
    assert not app.calls
    assert 'rejected: FeedInfo stale' in app._status_var.get()


@pytest.mark.parametrize('result', [0, -2])
def test_shutdown_invalidates_pending_action_and_attempts_every_output(app, result):
    app._channels[2]['duration_var'].set('250')
    app._on_toggle(2)
    app._on_close()
    reply(app, index=0)  # The old ON completion cannot arm auto-off after close.
    assert app._channels[2]['timer_after_id'] is None
    for index in range(1, 5):
        reply(app, result, index=index)
    assert [(request.index, request.status) for request, _ in app.calls[1:]] == [
        (1, 0), (2, 0), (13, 0), (14, 0)]
    assert app._finalized and app._stop_event.is_set()
