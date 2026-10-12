from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
import importlib.util
import json
from pathlib import Path

from item_perception_yolo.daily_events import DailyEventLogger, event_context


def test_dubai_rollover_restarts_retention_and_legacy_preservation(tmp_path):
    now = [datetime(2026, 10, 12, 19, 59, 59, tzinfo=timezone.utc)]
    logger = DailyEventLogger(tmp_path, 'fixture', 'first', clock=lambda: now[0])
    old = logger.directory / 'events-2026-09-12.jsonl'
    keep = logger.directory / 'events-2026-09-13.jsonl'
    old.write_text('old\n')
    keep.write_text('retained\n')
    legacy = logger.directory / 'events.jsonl'
    legacy.write_text('legacy\n')
    unrelated = logger.directory / 'events-operator.jsonl'
    unrelated.write_text('operator\n')
    target = tmp_path / 'protected'
    target.write_text('protected\n')
    (logger.directory / 'events-2020-01-01.jsonl').symlink_to(target)
    for i in range(1002):
        logger.record('INFO', 'fixture', 'test', index=i)
    first = logger.path
    assert first.name == 'events-2026-10-12.jsonl'
    assert len(first.read_text().splitlines()) == 1002
    assert not old.exists() and keep.exists()
    now[0] += timedelta(seconds=2)
    logger.record('INFO', 'fixture', 'new Dubai day')
    assert logger.path.name == 'events-2026-10-13.jsonl'
    assert len(logger.path.read_text().splitlines()) == 1
    assert not keep.exists()
    restarted = DailyEventLogger(tmp_path, 'fixture', 'second', clock=lambda: now[0])
    restarted.record('INFO', 'fixture', 'append after restart')
    lines = [json.loads(line) for line in logger.path.read_text().splitlines()]
    assert len(lines) == 2 and lines[0]['session_id'] != lines[1]['session_id']
    assert lines[0]['timestamp_utc'].startswith('2026-10-12T20:00:01')
    assert legacy.read_text() == 'legacy\n' and unrelated.read_text() == 'operator\n'
    assert target.read_text() == 'protected\n'


def test_concurrent_writers_and_isolated_context(tmp_path):
    writers = [DailyEventLogger(tmp_path, 'fixture', str(i)) for i in range(4)]

    def write(i):
        with event_context(run_id='run', cycle_index=i):
            for n in range(100):
                writers[i].record('INFO', 'parallel', 'test', index=n)
        writers[i].record('INFO', 'outside', 'test')

    with ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(write, range(4)))
    records = [json.loads(line) for line in writers[0].path.read_text().splitlines()]
    assert len(records) == 404
    assert all(r['cycle_index'] == int(r['node']) for r in records if r['event'] == 'parallel')
    assert all('cycle_index' not in r for r in records if r['event'] == 'outside')
    assert len({(r['session_id'], r['event_sequence']) for r in records}) == 404


def test_daily_write_failure_is_visible_and_does_not_raise(tmp_path, capsys):
    logger = DailyEventLogger(tmp_path, 'fixture', 'test')
    logger.path.symlink_to(tmp_path / 'not_owned')
    record = logger.record('ERROR', 'fault', 'keep safety handling running')
    assert record['event'] == 'fault'
    assert logger.last_error and 'daily log failed' in capsys.readouterr().err
    assert not (tmp_path / 'not_owned').exists()


def test_compile_legacy_and_daily_logs_by_dubai_date(tmp_path):
    script = Path(__file__).resolve().parents[3] / 'scripts/compile_logs.py'
    spec = importlib.util.spec_from_file_location('compile_daily_logs', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    root = tmp_path / 'logs/fixture'
    root.mkdir(parents=True)
    (root/'events.jsonl').write_text(json.dumps({
        'timestamp': '2026-10-12T19:59:59Z', 'package': 'fixture', 'event': 'before'})+'\n')
    (root/'events-2026-10-13.jsonl').write_text(json.dumps({
        'timestamp_utc': '2026-10-12T20:00:00Z', 'package': 'fixture', 'event': 'after'})+'\n')
    out = tmp_path / 'compiled.jsonl'
    assert module.compile_logs(tmp_path, out) == 2
    assert module.compile_logs(tmp_path, out, '2026-10-13') == 1
    assert json.loads(out.read_text())['event'] == 'after'
