"""CPU-only resource discovery and deterministic budgets; no native/ROS imports."""
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re


ROLES = ('item', 'tray')
ENV_KEYS = {role: f'{role.upper()}_TORCH_THREADS' for role in ROLES}
POLICY_VERSION = 1


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False,
                                     separators=(',', ':')).encode()).hexdigest()


def _unescape(value):
    return re.sub(r'\\([0-7]{3})', lambda m: chr(int(m[1], 8)), value)


def quota_limits(proc=Path('/proc'), fs_root=Path('/')):
    """Read every ancestor quota in the current cgroup v1/v2 CPU hierarchy."""
    groups = {}
    for line in (proc/'self/cgroup').read_text().splitlines():
        _, controllers, name = line.split(':', 2)
        for controller in controllers.split(','):
            groups[controller] = name
    limits = []
    for line in (proc/'self/mountinfo').read_text().splitlines():
        left, right = line.split(' - ', 1)
        fields, mounted = left.split(), right.split()
        kind = mounted[0]
        if kind == 'cgroup2' and '' in groups:
            group, v2 = groups[''], True
        elif kind == 'cgroup' and 'cpu' in mounted[2].split(',') and 'cpu' in groups:
            group, v2 = groups['cpu'], False
        else:
            continue
        root = Path(_unescape(fields[3]))
        mount = fs_root / _unescape(fields[4]).lstrip('/')
        group = Path(group)
        # A cgroup namespace can report '/' while the mount root names the host group.
        relative = Path('.') if group == Path('/') else group.relative_to(root)
        leaf = mount / relative
        for directory in (leaf, *leaf.parents):
            if not directory.is_relative_to(mount):
                break
            if v2:
                path = directory/'cpu.max'
                if not path.exists():
                    continue
                quota, period = path.read_text().split()
                if quota == 'max':
                    continue
                quota, period = int(quota), int(period)
            else:
                path = directory/'cpu.cfs_quota_us'
                if not path.exists():
                    continue
                quota = int(path.read_text())
                period = int((directory/'cpu.cfs_period_us').read_text())
                if quota == -1:
                    continue
            if quota <= 0 or period <= 0:
                raise ValueError(f'Invalid CPU quota at {path}')
            limits.append(quota / period)
    return limits


def discover(*, affinity=None, proc=Path('/proc'), sys=Path('/sys'), fs_root=Path('/')):
    cpus = sorted(os.sched_getaffinity(0) if affinity is None else affinity)
    if not cpus:
        raise ValueError('No CPUs available to perception')
    cores, topology_known = set(), True
    for cpu in cpus:
        topology = sys/f'devices/system/cpu/cpu{cpu}/topology'
        try:
            cores.add((int((topology/'physical_package_id').read_text()),
                       int((topology/'core_id').read_text())))
        except FileNotFoundError:
            topology_known = False
            cores.add(('unknown', cpu))
    quotas = quota_limits(proc, fs_root)
    model = sorted({line.strip() for line in (proc/'cpuinfo').read_text().splitlines()
                    if line.split(':', 1)[0].strip() in
                    ('model name', 'vendor_id', 'cpu family', 'model', 'stepping', 'microcode', 'flags',
                     'CPU implementer', 'CPU architecture', 'CPU variant', 'CPU part',
                     'CPU revision', 'Hardware', 'Revision', 'uarch', 'isa')})
    effective = min([float(len(cpus)), *quotas])
    return {'policy': POLICY_VERSION, 'architecture': platform.machine(), 'model': model,
            'allowed_cpus': cpus, 'physical_cores': len(cores),
            'topology_known': topology_known, 'effective_cpus': effective}


def settings(values):
    result = {}
    for role, key in ENV_KEYS.items():
        value = values.get(key, 'auto')  # Optional override; absence selects the auto policy.
        if value == 'auto':
            result[role] = None
        elif isinstance(value, str) and re.fullmatch(r'[1-9][0-9]*', value):
            result[role] = int(value)
        else:
            raise ValueError(f'{key} must be auto or a positive integer')
    return result


def allocate(hardware, overrides=None):
    """Budget two potentially overlapping workers, reserving ~1/8 CPU capacity."""
    overrides = overrides or dict.fromkeys(ROLES)
    capacity = max(1, math.floor(hardware['effective_cpus']))
    physical = hardware['physical_cores']
    reserve = max(1, math.ceil(capacity / 8))
    total = max(2, capacity - reserve)
    fixed = {role: count for role, count in overrides.items() if count is not None}
    if any(type(n) is not int or not 1 <= n <= capacity for n in fixed.values()):
        raise ValueError(f'Perception thread override exceeds usable CPU capacity {capacity}')
    automatic = [role for role in ROLES if role not in fixed]
    if sum(fixed.values()) + len(automatic) > max(2, capacity):
        raise ValueError('Combined item/tray thread overrides exceed usable CPU capacity')
    remaining = max(len(automatic), total - sum(fixed.values()))
    limits = dict(fixed)
    for index, role in enumerate(automatic):
        count = remaining // len(automatic) + int(index < remaining % len(automatic))
        limits[role] = min(physical, count)
    return {'limits': limits, 'overrides': overrides,
            'reserved_capacity': max(0, hardware['effective_cpus'] - sum(limits.values())),
            'limited_capacity': hardware['effective_cpus'] < 3}


def automatic_threads(role='item'):
    return allocate(discover())['limits'][role]


def checked_threads(value):
    if type(value) is not int or value < 1:
        raise ValueError('Torch thread count must be a positive integer')
    return value


def worker_threads(role, argv):
    # Explicit private transport argument, never a second user configuration source.
    if len(argv) == 2:
        return automatic_threads(role)
    if len(argv) != 3 or not re.fullmatch(r'[1-9][0-9]*', argv[2]):
        raise ValueError('Expected runtime path and optional private thread budget')
    return checked_threads(int(argv[2]))


def runtime_identity(runtime):
    """Tie measurements to pinned dependencies and the installed processing code."""
    import importlib.util
    hashes = {'runtime-lock': hashlib.sha256((runtime/'runtime-lock.json').read_bytes()).hexdigest()}
    for package in ('item_perception_yolo', 'tray_perception'):
        spec = importlib.util.find_spec(package)
        if spec is None or not spec.submodule_search_locations:
            hashes[package] = None
            continue
        for path in sorted(Path(next(iter(spec.submodule_search_locations))).glob('*.py')):
            hashes[f'{package}/{path.name}'] = hashlib.sha256(path.read_bytes()).hexdigest()
    return digest(hashes)


def workload_key(role, header):
    model = header.get('model')
    if not isinstance(model, dict) or not re.fullmatch(r'[0-9a-f]{64}', model.get('sha256', '')):
        return None
    if header.get('operation') not in ('detect', 'tray_preview'):
        return None
    return digest({'role': role, 'model': {k: v for k, v in model.items() if k != 'path'},
                   'width': header.get('width'), 'height': header.get('height'),
                   'settings': header.get('settings'), 'render_images': header.get('render_images', True),
                   'candidate_limit': header.get('candidate_limit')})


def read_tuning(path, identity):
    """A stale/unusable measurement is ignored; it never changes perception validity."""
    try:
        if path.stat().st_size > 1024 * 1024:
            return {}, 'measurement file exceeds 1 MiB'
        data = json.loads(path.read_text())
        if data['version'] != 1 or data['identity'] != identity:
            return {}, 'hardware/runtime/budget changed'
        entries = data['entries']
        if not isinstance(entries, dict):
            raise ValueError('entries must be a dictionary')
        for key, value in entries.items():
            if not re.fullmatch(r'[0-9a-f]{64}', key):
                raise ValueError('invalid workload hash')
            checked_threads(value)
        return entries, 'matched'
    except FileNotFoundError:
        return {}, 'no measurements'
    except (OSError, ValueError, TypeError, KeyError) as exc:
        return {}, f'invalid measurement: {exc}'
