#!/usr/bin/env python3
"""Offline item/tray native-request replay and CPU tuning; no ROS nodes or hardware.

Requires explicitly supplied saved request/payload bundles; never captures live data.
Each trial uses fresh native processes, normal pinned inference/geometry paths and
overlapping item/tray work. --save retains measured choices for matching workloads.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import itertools
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix
from camera_calibration_gui.calibration_core import workspace_root
from item_perception_yolo import cpu_budget
from item_perception_yolo.item_native_client import NativeClient
from orbbec_camera_launcher.project_config import _parse_env_file


class Events:
    def record(self, *_args, **_kwargs):
        pass


def require_idle_perception(proc=Path('/proc')):
    names = {'item_detect', 'item_teach', 'tray_detect', 'tray_teach',
             'item_preview_worker', 'tray_worker'}
    for path in proc.glob('[0-9]*/cmdline'):
        try:
            arguments = path.read_bytes().decode(errors='replace').split('\0')
        except (FileNotFoundError, PermissionError):
            continue
        if any(Path(argument).name in names for argument in arguments):
            raise RuntimeError(f'Perception process {path.parent.name} is running; stop providers before tuning')


def load_bundle(path, role):
    data = json.loads(path.read_text())
    if data.get('version') != 1 or data.get('role') != role:
        raise ValueError(f'{path}: expected version 1 {role} replay')
    requests = []
    for request in data['requests']:
        header = request['header']
        required = ('context', 'settings') if role == 'item' else ('plane', 'camera_context', 'settings')
        if (header.get('operation') != ('detect' if role == 'item' else 'tray_preview')
                or any(not isinstance(header.get(k), dict) for k in required)
                or cpu_budget.workload_key(role, header) is None):
            raise ValueError(f'{path}: replay needs the complete native {role} pose request')
        requests.append((header, (path.parent / request['payload']).read_bytes()))
    if len(requests) < (1 if role == 'item' else 2) or (role == 'tray' and len(requests) % 2):
        raise ValueError(f'{path}: supply item observations / complete consecutive tray pairs')
    keys = {cpu_budget.workload_key(role, header) for header, _ in requests}
    if len(keys) != 1:
        raise ValueError('Each replay bundle must have one model/settings/resolution identity')
    return requests, keys.pop()


def counts(limit):
    return sorted({1, limit, *(n for n in (2, 4, 8, 16, 32, 64) if n <= limit)})


def percentile95(values):
    return sorted(values)[max(0, math.ceil(.95 * len(values)) - 1)]


def choose(trials):
    # Ignore trials whose output validity changed: timing never overrides pose validity.
    baseline = trials[-1]['outcomes']
    valid = [t for t in trials if t['outcomes'] == baseline]
    return min(valid, key=lambda t: (max(t['item_p95_ms'], t['tray_p95_ms']),
                                    t['item_p95_ms'] + t['tray_p95_ms'], sum(t['threads'].values())))


def cpu_seconds(pid):
    # Field 3 starts after the final ')' so spaces in the process name are harmless.
    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')


def trial(bundles, pair, warmups, rounds):
    from tray_perception.stability import compare, sample
    clients = {role: NativeClient(Events(), worker_package=(
        'item_perception_yolo' if role == 'item' else 'tray_perception'),
        worker_executable=('item_preview_worker' if role == 'item' else 'tray_worker'),
        torch_threads=pair[role]) for role in cpu_budget.ROLES}
    barrier = threading.Barrier(2, timeout=60.)

    def step(role, index):
        replay = bundles[role]
        size = 1 if role == 'item' else 2
        selected = [replay[(index * size + j) % len(replay)] for j in range(size)]
        barrier.wait()
        started = time.perf_counter()
        results = [clients[role].call(header, payload, timeout=60.)[0] for header, payload in selected]
        if role == 'item':
            outcome = len(results[0].get('candidates', []))
        else:
            first, second = (result['selected'] for result in results)
            outcome = bool(first is not None and second is not None and compare(
                sample(first, index*2+1), sample(second, index*2+2),
                selected[1][0]['settings']['geometry'])['accepted'])
        return (time.perf_counter() - started) * 1000., outcome

    durations = {role: [] for role in cpu_budget.ROLES}
    outcomes = {role: [] for role in cpu_budget.ROLES}
    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            for index in range(warmups + rounds):
                if index == warmups:
                    started = time.perf_counter()
                    cpu_start = sum(cpu_seconds(client.process.pid) for client in clients.values())
                pending = {role: pool.submit(step, role, index % rounds) for role in cpu_budget.ROLES}
                for role, future in pending.items():
                    duration, outcome = future.result(timeout=130.)
                    if index >= warmups:
                        durations[role].append(duration)
                        outcomes[role].append(outcome)
            elapsed = time.perf_counter() - started
            cpu = sum(cpu_seconds(client.process.pid) for client in clients.values()) - cpu_start
        result = {'threads': pair, 'outcomes': outcomes, 'cpu_core_percent': 100 * cpu / elapsed}
        for role, values in durations.items():
            result[f'{role}_median_ms'] = statistics.median(values)
            result[f'{role}_p95_ms'] = percentile95(values)
        return result
    finally:
        for client in clients.values():
            client.close()


def atomic_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        try:
            json.dump(value, stream, sort_keys=True, allow_nan=False, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--item-replay', type=Path, required=True)
    parser.add_argument('--tray-replay', type=Path, required=True)
    parser.add_argument('--warmups', type=int, default=4)
    parser.add_argument('--rounds', type=int, default=20)
    parser.add_argument('--save', action='store_true', help='Save matched derived measurements for auto mode')
    args = parser.parse_args()
    if args.warmups < 1 or args.rounds < 5:
        parser.error('Use at least one warmup and five measured rounds')
    require_idle_perception()
    root = workspace_root()
    hardware = cpu_budget.discover()
    allocation = cpu_budget.allocate(hardware, cpu_budget.settings(_parse_env_file(root/'.env')))
    if any(value is not None for value in allocation['overrides'].values()):
        parser.error('Set both optional thread overrides to auto before tuning')
    runtime = Path(get_package_prefix('item_perception_yolo'))/'lib/item_perception_yolo/yolo_runtime'
    identity = {'hardware': hardware, 'allocation': allocation,
                'runtime': cpu_budget.runtime_identity(runtime)}
    bundles, keys = {}, {}
    for role in cpu_budget.ROLES:
        bundles[role], keys[role] = load_bundle(getattr(args, f'{role}_replay'), role)
    trials = []
    for item, tray in itertools.product(counts(allocation['limits']['item']), counts(allocation['limits']['tray'])):
        result = trial(bundles, {'item': item, 'tray': tray}, args.warmups, args.rounds)
        trials.append(result)
        print(json.dumps(result), flush=True)
    best = choose(trials)
    if hardware != cpu_budget.discover() or identity['runtime'] != cpu_budget.runtime_identity(runtime):
        raise RuntimeError('Hardware/runtime changed during benchmark; measurements not saved')
    report = {'version': 1, 'identity': identity, 'trials': trials, 'selected': best,
              'workloads': keys, 'warmups': args.warmups, 'rounds': args.rounds,
              'inputs': {role: [{'header_sha256': cpu_budget.digest(header),
                                'payload_sha256': hashlib.sha256(payload).hexdigest()}
                               for header, payload in bundle] for role, bundle in bundles.items()},
              'limitations': 'Archived native compute + IPC and tray pair check only; synthetic stamps. '
              'Excludes live camera/TF waits, controller guards, ROS transport and physical operation. '
              'Choose representative scenes; empty/rejected frames can underrepresent processing cost.'}
    atomic_json(root/'logs/perception_cpu_benchmark.json', report)
    if args.save:
        path = root/'logs/perception_cpu_tuning.json'
        entries, _ = cpu_budget.read_tuning(path, identity)
        entries.update({keys[role]: best['threads'][role] for role in cpu_budget.ROLES})
        atomic_json(path, {'version': 1, 'identity': identity, 'entries': entries})
    print('Selected:', json.dumps(best['threads']), '; saved:', args.save)


if __name__ == '__main__':
    main()
