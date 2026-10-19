"""Hardware budgeting, namespace quotas and workload-bound offline measurements."""
import copy
import importlib.util
import json
from pathlib import Path
from unittest.mock import MagicMock

import pytest
from item_perception_yolo import cpu_budget as budget
from item_perception_yolo.item_native_client import NativeClient


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value)


def hardware(tmp_path, *, cpus=16, smt=2, quota='max 100000', parent='max 100000'):
    proc, sys, fs = tmp_path/'proc', tmp_path/'sys', tmp_path/'fs'
    write(proc/'self/cgroup', '0::/group/child\n')
    write(proc/'self/mountinfo', '34 25 0:29 / /cgroup rw - cgroup2 cgroup2 rw\n')
    write(proc/'cpuinfo', 'model name : Fake CPU\nflags : sse avx\n')
    write(fs/'cgroup/group/child/cpu.max', quota)
    write(fs/'cgroup/group/cpu.max', parent)
    for cpu in range(cpus):
        root = sys/f'devices/system/cpu/cpu{cpu}/topology'
        write(root/'physical_package_id', '0')
        write(root/'core_id', str(cpu//smt))
    return {'proc': proc, 'sys': sys, 'fs_root': fs, 'affinity': range(cpus)}


@pytest.mark.parametrize('cpus,smt,expected', [(16, 2, (7, 7)), (32, 2, (14, 14)),
                                           (8, 1, (4, 3)), (4, 1, (2, 1)),
                                           (2, 1, (1, 1)), (1, 1, (1, 1))])
def test_hardware_scaling_and_control_headroom(tmp_path, cpus, smt, expected):
    result = budget.discover(**hardware(tmp_path, cpus=cpus, smt=smt))
    allocation = budget.allocate(result)
    assert tuple(allocation['limits'][role] for role in budget.ROLES) == expected
    assert result['physical_cores'] == cpus//smt
    assert allocation['limited_capacity'] == (cpus < 3)


def test_affinity_and_tightest_fractional_ancestor_quota(tmp_path):
    args = hardware(tmp_path, quota='350000 100000', parent='250000 100000')
    args['affinity'] = [0, 2, 4, 6]
    result = budget.discover(**args)
    assert result['effective_cpus'] == 2.5 and result['physical_cores'] == 4
    assert budget.allocate(result)['limits'] == {'item': 1, 'tray': 1}
    write(args['fs_root']/'cgroup/group/cpu.max', '50000 100000')
    assert budget.discover(**args)['effective_cpus'] == .5


@pytest.mark.parametrize('namespace', [False, True])
def test_v1_and_namespaced_mount_root(tmp_path, namespace):
    args = hardware(tmp_path)
    write(args['proc']/'self/cgroup', '2:cpu,cpuacct:' + ('/' if namespace else '/host/team/child') + '\n')
    write(args['proc']/'self/mountinfo', '34 25 0:29 /host/team /cgroup rw - cgroup cgroup rw,cpu,cpuacct\n')
    write(args['fs_root']/'cgroup/cpu.cfs_quota_us', '600000')
    write(args['fs_root']/'cgroup/cpu.cfs_period_us', '100000')
    write(args['fs_root']/'cgroup/child/cpu.cfs_quota_us', '-1')
    write(args['fs_root']/'cgroup/child/cpu.cfs_period_us', '100000')
    assert budget.discover(**args)['effective_cpus'] == 6


def test_unknown_topology_is_reported_not_assumed_physical(tmp_path):
    args = hardware(tmp_path)
    (args['sys']/'devices/system/cpu/cpu0/topology/core_id').unlink()
    assert not budget.discover(**args)['topology_known']


def test_cpu_identity_includes_arm_part_numbers(tmp_path):
    args = hardware(tmp_path)
    write(args['proc']/'cpuinfo', 'CPU implementer : 0x41\nCPU part : 0xd05\n')
    first = budget.discover(**args)
    write(args['proc']/'cpuinfo', 'CPU implementer : 0x41\nCPU part : 0xd0b\n')
    assert budget.discover(**args) != first


def test_missing_peer_package_invalidates_measurements_without_blocking_item(monkeypatch, tmp_path):
    import importlib.util
    write(tmp_path/'runtime-lock.json', '{}')
    identity = budget.runtime_identity(tmp_path)
    find = importlib.util.find_spec
    monkeypatch.setattr(importlib.util, 'find_spec',
                        lambda name: None if name == 'tray_perception' else find(name))
    without_tray = budget.runtime_identity(tmp_path)
    assert without_tray != identity


@pytest.mark.parametrize('value', ['', '0', '-1', 'Auto', ' 8', '8.0', '08', True])
def test_bad_override_rejected(value):
    with pytest.raises(ValueError, match='auto or a positive integer'):
        budget.settings({'ITEM_TORCH_THREADS': value})


def test_explicit_overrides_and_combined_budget(tmp_path):
    h = budget.discover(**hardware(tmp_path))
    assert budget.allocate(h, budget.settings({'ITEM_TORCH_THREADS': '8'}))['limits'] == {
        'item': 8, 'tray': 6}
    assert budget.allocate(h, budget.settings({'ITEM_TORCH_THREADS': '8', 'TRAY_TORCH_THREADS': '8'}))[
        'reserved_capacity'] == 0
    with pytest.raises(ValueError, match='Combined'):
        budget.allocate(h, {'item': 9, 'tray': 8})
    with pytest.raises(ValueError, match='capacity'):
        budget.allocate(h, {'item': 17, 'tray': None})


def request():
    return {'operation': 'detect', 'width': 640, 'height': 480, 'settings': {'geometry_source': 'mask'},
            'model': {'path': '/model.pt', 'sha256': 'a'*64, 'task': 'segment', 'yolo': {'image_size': 448}}}


def test_workload_identity_binds_model_settings_resolution_role():
    header = request()
    key = budget.workload_key('item', header)
    for changed in ({'width': 848}, {'settings': {}}, {'render_images': False}, {'candidate_limit': 2}):
        assert budget.workload_key('item', {**header, **changed}) != key
    assert budget.workload_key('tray', header) != key
    for field, value in [('sha256', 'b'*64), ('task', 'obb'), ('yolo', {'image_size': 640})]:
        changed = copy.deepcopy(header)
        changed['model'][field] = value
        assert budget.workload_key('item', changed) != key
    changed = copy.deepcopy(header)
    changed['model']['path'] = '/relocated/model.pt'
    assert budget.workload_key('item', changed) == key
    assert budget.workload_key('item', {'operation': 'overlay_roi'}) is None


def test_cache_hardware_runtime_invalidation_and_corruption(tmp_path):
    path = tmp_path/'tuning.json'
    identity = {'hardware': {'effective': 8}, 'runtime': 'old', 'allocation': {'item': 3, 'tray': 4}}
    key = budget.workload_key('item', request())
    write(path, json.dumps({'version': 1, 'identity': identity, 'entries': {key: 2}}))
    assert budget.read_tuning(path, identity) == ({key: 2}, 'matched')
    for field in identity:
        assert budget.read_tuning(path, {**identity, field: 'changed'})[0] == {}
    for entries in ({key: True}, {key: 0}, {'bad': 2}, [], None):
        write(path, json.dumps({'version': 1, 'identity': identity, 'entries': entries}))
        assert budget.read_tuning(path, identity)[0] == {}
    write(path, '[')
    assert budget.read_tuning(path, identity)[0] == {}
    path.unlink()
    assert budget.read_tuning(path, identity)[1] == 'no measurements'


def test_client_applies_and_restores_measured_budget_without_model_replay(monkeypatch):
    from item_perception_yolo import item_native_client as module
    client = NativeClient(MagicMock(), torch_threads=7)
    client.process = MagicMock()
    header = request()
    client.tuning = {budget.workload_key('item', header): 2}
    sent = []
    monkeypatch.setattr(module, 'send_packet', lambda _stream, h: sent.append(h))
    monkeypatch.setattr(module, 'receive_packet', lambda _stream: (
        {'state': 'ok', 'torch_threads': sent[-1]['threads']}, b''))
    client._select_threads(header)
    client._select_threads(header)
    client._select_threads({**header, 'width': 848})
    assert sent == [{'operation': 'thread_budget', 'threads': 2},
                    {'operation': 'thread_budget', 'threads': 7}]
    client.tuning = {budget.workload_key('item', header): 999}
    client._select_threads(header)
    assert len(sent) == 2


def test_private_worker_thread_argument():
    assert budget.worker_threads('tray', ['worker', 'runtime', '3']) == 3
    for value in ('0', '-1', 'auto', '3.0'):
        with pytest.raises(ValueError):
            budget.worker_threads('item', ['worker', 'runtime', value])


def tuner():
    path = Path(__file__).resolve().parents[3]/'scripts/tune_perception_cpu.py'
    spec = importlib.util.spec_from_file_location('tuner', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_offline_selection_uses_tail_latency_and_preserves_validity():
    module = tuner()
    trials = [
        {'threads': {'item': 1, 'tray': 1}, 'item_p95_ms': 5, 'tray_p95_ms': 5, 'outcomes': [0]},
        {'threads': {'item': 2, 'tray': 2}, 'item_p95_ms': 20, 'tray_p95_ms': 30, 'outcomes': [1]},
        {'threads': {'item': 7, 'tray': 7}, 'item_p95_ms': 10, 'tray_p95_ms': 45, 'outcomes': [1]}]
    assert module.choose(trials) is trials[1]
    assert module.counts(7) == [1, 2, 4, 7]
    assert module.percentile95(list(range(1, 21))) == 19


def test_readonly_replay_validation_and_atomic_result(tmp_path):
    module = tuner()
    path = tmp_path/'item.json'
    h = {**request(), 'context': {}}
    write(tmp_path/'frame.bin', 'pixels')
    data = {'version': 1, 'role': 'item', 'requests': [{'header': h, 'payload': 'frame.bin'}]}
    module.atomic_json(path, data)
    requests, key = module.load_bundle(path, 'item')
    assert requests == [(h, b'pixels')] and key == budget.workload_key('item', h)
    del data['requests'][0]['header']['context']
    module.atomic_json(path, data)
    with pytest.raises(ValueError, match='complete native'):
        module.load_bundle(path, 'item')


def test_tuner_refuses_running_provider_without_touching_it(tmp_path):
    module = tuner()
    write(tmp_path/'123/cmdline', 'python3\0/install/tray_detect\0')
    with pytest.raises(RuntimeError, match='123 is running'):
        module.require_idle_perception(tmp_path)
    write(tmp_path/'123/cmdline', 'unrelated\0')
    module.require_idle_perception(tmp_path)
