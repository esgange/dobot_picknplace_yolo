"""A tray request can expire without releasing the single native transport owner."""
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from item_perception_yolo import item_native_client as module


def client(monkeypatch):
    gate = threading.Event()
    sent = []
    value = module.NativeClient(MagicMock(), torch_threads=8)
    value.ready = True
    value._start = lambda: None
    value.process = SimpleNamespace(pid=42, stdin=object(), stdout=object(),
                                    poll=lambda: None, terminate=gate.set, wait=lambda **_kw: None)
    monkeypatch.setattr(module, 'send_packet', lambda _stream, header, _data: sent.append(header))

    def receive(_stream):
        if len(sent) == 1:
            assert gate.wait(2.)
        return {'state': 'ok', 'id': len(sent)}, b'pixels'
    monkeypatch.setattr(module, 'receive_packet', receive)
    return value, gate, sent


def test_expired_prediction_retires_without_replay_or_parallel_request(monkeypatch):
    value, gate, sent = client(monkeypatch)
    started = time.monotonic()
    with pytest.raises(TimeoutError, match='retiring'):
        value.call({'operation': 'tray_preview'}, timeout=1., request_timeout=.02)
    assert time.monotonic() - started < .3
    assert value.lock.locked() and not value.failed and not value.closed
    with pytest.raises(TimeoutError, match='busy'):
        value.call({'operation': 'tray_preview'}, timeout=1., request_timeout=.01)
    assert len(sent) == 1
    gate.set()
    assert value.lock.acquire(timeout=1.)
    value.lock.release()
    header, _ = value.call({'operation': 'tray_preview'}, timeout=1., request_timeout=.5)
    assert header['id'] == 2 and len(sent) == 2 and not value.failed


def test_retirement_keeps_a_hard_worker_deadline(monkeypatch):
    value, gate, sent = client(monkeypatch)
    with pytest.raises(TimeoutError, match='retiring'):
        value.call({'operation': 'tray_preview'}, timeout=.08, request_timeout=.01)
    assert value.lock.acquire(timeout=1.)
    value.lock.release()
    assert value.failed and value.closed and gate.is_set() and len(sent) == 1


def test_default_item_timeout_remains_terminal(monkeypatch):
    value, gate, _ = client(monkeypatch)
    with pytest.raises(RuntimeError, match='Terminal item worker failure'):
        value.call({'operation': 'detect'}, timeout=.02)
    assert value.failed and value.closed and gate.is_set()
