import json
from datetime import date
from types import SimpleNamespace

import pytest

from robot_controller.events import PackageEventLogger
from robot_controller.time_study import RunStudy
from item_perception_yolo.daily_events import event_context
from item_perception_yolo.daily_events import current_event_context
from robot_controller.autorun import CandidatePrefetch


def emit(study, seconds, name, **fields):
    study.observe({'event': name, 'monotonic_ns': int(seconds*1e9), **fields})


def test_50_cycles_overlap_retries_and_exact_exclusive_total(tmp_path):
    study = RunStudy('a'*32, 50)
    emit(study, 0, 'auto_run_started')
    for i in range(50):
        t = i*10
        emit(study, t+1, 'motion_batch_dispatch_started', batch='candidate_1_home_to_pick')
        emit(study, t+3, 'pickup_detected')
        emit(study, t+4, 'motion_batch_dispatch_started', batch='candidate_1_pick_to_tray')
        emit(study, t+6, 'motion_batch_completed', terminal_target='tray_detect_position')
        emit(study, t+6.2, 'operation_phase', phase='TRAY_DEPTH', message='requesting fresh tray')
        emit(study, t+6.7, 'tray_request_timing', cycle_index=i+1, attempt=1,
             status='NO_VALID_TRAY', usable=False, total_ms=500.,
             worker_timings_ms={'yolo_predict_ms': 80.})
        emit(study, t+7.2, 'tray_request_timing', cycle_index=i+1, attempt=2,
             status='OK', usable=True, total_ms=500., worker_timings_ms={'yolo_predict_ms': 70.})
        emit(study, t+7.2, 'placement_depth')
        emit(study, t+7.5, 'motion_batch_dispatch_started', batch='place_to_retract')
        if i < 49:
            emit(study, t+8, 'candidate_timing', cycle_index=i+2,
                 total_ms=400., service_wait_ms=300., batch_id=str(i), detector_timings_ms={})
            emit(study, t+8.1, 'motion_batch_dispatch_started', batch='candidate_1_place_to_pick')
        emit(study, t+10, 'auto_run_placement_completed', completed=i+1, boundary_command_id=20)
    emit(study, 501, 'auto_run_result')
    paths = study.finish(tmp_path, {'completed': 50, 'elapsed_sec': 501})
    data = json.loads(open(paths['json_path']).read())
    assert data['observed_elapsed_sec'] == pytest.approx(501)
    assert sum(data['phase_totals_sec'].values()) == pytest.approx(501)
    assert data['phase_totals_sec']['placement_and_handoff'] == pytest.approx(125)
    assert data['phase_totals_sec']['final_home'] == pytest.approx(1)
    assert data['tray_request_ms']['count'] == 100
    assert data['tray_failures_by_status'] == {'NO_VALID_TRAY': 50}
    assert data['tray_first_request_usable'] == 0
    assert data['tray_failed_request_time_ms'] == 25000
    assert data['tray_worker_timings_ms']['yolo_predict_ms']['median'] == 75
    assert len(data['item_requests']) == 49
    assert data['cycle_duration_sec']['count'] == 50


def test_interrupted_run_reports_incomplete_cycle_and_pause(tmp_path):
    study = RunStudy('b'*32, 50)
    emit(study, 0, 'auto_run_started')
    emit(study, 2, 'state_transition', state='PAUSED')
    emit(study, 12, 'state_transition', previous='PAUSED', state='PICKING')
    emit(study, 14, 'auto_run_result')
    paths = study.finish(tmp_path, {'completed': 0, 'outcome': 'canceled'})
    data = json.loads(open(paths['json_path']).read())
    assert data['phase_totals_sec']['paused'] == 10
    assert not data['cycles'][0]['completed']
    assert data['observed_elapsed_sec'] == 14


def test_controller_logger_keeps_context_and_writes_reports(tmp_path):
    logger = PackageEventLogger(tmp_path)
    logger.start_run('c'*32, 1, global_speed_percent=100)
    with event_context(cycle_index=1):
        logger.record('INFO', 'pickup_detected', 'suction')
    logger.record('INFO', 'auto_run_placement_completed', 'placed',
                  completed=1, boundary_command_id=1)
    logger.record('INFO', 'auto_run_result', 'done', elapsed_sec=1.)
    logger.finish_run(completed=1, elapsed_sec=1.)
    logger.record('INFO', 'idle', 'outside run')
    records = [json.loads(s) for s in logger.path.read_text().splitlines()]
    assert records[1]['run_id'] == 'c'*32 and records[1]['cycle_index'] == 1
    assert records[-2]['event'] == 'auto_run_time_study'
    assert 'run_id' not in records[-1]
    assert len(list(logger.directory.glob('run-*.csv'))) == 1


def test_prefetch_retains_next_cycle_context_in_its_own_thread():
    captured = []

    def acquire(*_, **__):
        captured.append(current_event_context())
        return 'batch'

    node = SimpleNamespace(candidates=SimpleNamespace(request=acquire),
                           cancel_requested=lambda: False)
    with event_context(run_id='run', cycle_index=2, acquisition_mode='prefetch'):
        prefetch = CandidatePrefetch(node, object(), False)
    assert prefetch.future.result(timeout=1) == 'batch'
    prefetch.close()
    assert captured == [{'run_id': 'run', 'cycle_index': 2, 'acquisition_mode': 'prefetch'}]
    assert current_event_context() == {}


def test_report_retention_only_removes_owned_expired_files(tmp_path, monkeypatch):
    import robot_controller.time_study as module
    monkeypatch.setattr(module, 'log_day', lambda: date(2026, 10, 12))
    paths = []
    for day in ('2026-09-12', '2026-09-13'):
        for suffix in ('json', 'csv'):
            path = tmp_path / f"run-{day}-{'a'*32}.{suffix}"
            path.write_text('fixture')
            paths.append(path)
    operator = tmp_path / 'run-2020-01-01-operator.json'
    operator.write_text('preserve')
    link = tmp_path / f"run-2020-01-01-{'b'*32}.json"
    link.symlink_to(operator)
    study = RunStudy('c'*32, 1)
    emit(study, 0, 'auto_run_started')
    emit(study, 1, 'auto_run_result')
    study.finish(tmp_path, {})
    assert not paths[0].exists() and not paths[1].exists()
    assert paths[2].exists() and paths[3].exists()
    assert operator.read_text() == 'preserve' and link.is_symlink()
