from __future__ import annotations

import ipaddress
import math
import os
import re
import stat
import tempfile
from dataclasses import dataclass
from pathlib import Path


_ENV_KEY_RE = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')
_ROS_NAME_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')
_SERIAL_RE = re.compile(r'^[A-Za-z0-9_.:-]+$')

DOBOT_ENV_KEYS = (
    'ROS_LOCALHOST_ONLY',
    'DOBOT_ROBOT_LAN1_IP',
    'DOBOT_ROBOT_LAN2_IP',
    'DOBOT_CONNECTION_TIMEOUT_MS',
    'DOBOT_ROBOT_TYPE',
    'DOBOT_ROBOT_NUMBER',
    'DOBOT_TRAJECTORY_DURATION',
    'DOBOT_ROBOT_NODE_NAME',
)

ORBBEC_ENV_KEYS = (
    'ORBBEC_CAMERA_1_NAME',
    'ORBBEC_CAMERA_1_SERIAL',
    'ORBBEC_CAMERA_2_NAME',
    'ORBBEC_CAMERA_2_SERIAL',
    'ORBBEC_DEVICE_PRESET',
    'ORBBEC_ENABLE_COLOR',
    'ORBBEC_ENABLE_DEPTH',
    'ORBBEC_DEPTH_REGISTRATION',
    'ORBBEC_ALIGN_TARGET_STREAM',
    'ORBBEC_ALIGN_MODE',
    'ORBBEC_ENABLE_FRAME_SYNC',
    'ORBBEC_ENABLE_TEMPORAL_FILTER',
    'ORBBEC_COLOR_WIDTH',
    'ORBBEC_COLOR_HEIGHT',
    'ORBBEC_COLOR_FPS',
    'ORBBEC_DEPTH_WIDTH',
    'ORBBEC_DEPTH_HEIGHT',
    'ORBBEC_DEPTH_FPS',
    'ORBBEC_ENABLE_POINT_CLOUD',
    'ORBBEC_ENUMERATE_NET_DEVICE',
    'ORBBEC_SCAN_TIMEOUT_SEC',
    'ORBBEC_STARTUP_TIMEOUT_SEC',
    'ORBBEC_HEALTH_TIMEOUT_SEC',
    'ORBBEC_CHECK_PERIOD_SEC',
    'ORBBEC_MAX_ATTEMPTS',
    'ORBBEC_RETRY_DELAY_SEC',
    'ORBBEC_SHUTDOWN_TIMEOUT_SEC',
)

SUPPORTED_ENV_KEYS = frozenset((*DOBOT_ENV_KEYS, *ORBBEC_ENV_KEYS))
REQUIRED_ENV_KEYS = SUPPORTED_ENV_KEYS
ORBBEC_LAUNCH_FILE = 'gemini_330_series.launch.py'
FIXED_CAMERA_COUNT = 2
ORBBEC_CAMERA_SLOTS = tuple(range(1, FIXED_CAMERA_COUNT + 1))


@dataclass(frozen=True)
class CameraSpec:
    slot: int
    name: str
    serial_number: str


def project_root() -> Path:
    """Find the one repository root above this installed or source module."""
    module_path = Path(__file__).resolve()
    for candidate in (module_path.parent, *module_path.parents):
        if (
            (candidate / '.env.example').is_file()
            and (candidate / 'src').is_dir()
            and (candidate / 'docs' / 'WORKFLOW_RULES_BLUEPRINT_DIARY.md').is_file()
        ):
            return candidate
    raise RuntimeError(
        'Cannot locate the PicknPlace repository root above orbbec_camera_launcher'
    )


def _parse_env_file(env_path: Path) -> dict[str, str]:
    if not env_path.is_file():
        raise RuntimeError(f'Required project configuration is missing: {env_path}')

    values: dict[str, str] = {}
    with env_path.open('r', encoding='utf-8') as env_file:
        for line_number, raw_line in enumerate(env_file, start=1):
            line = raw_line.rstrip('\r\n')
            if not line or line.startswith('#'):
                continue
            if line.startswith('export ') or '=' not in line:
                raise RuntimeError(
                    f'Invalid project .env syntax at {env_path}:{line_number}'
                )
            key, value = line.split('=', 1)
            if key != key.strip() or value != value.strip():
                raise RuntimeError(
                    f'Project .env cannot contain spaces around values at '
                    f'{env_path}:{line_number}'
                )
            if not _ENV_KEY_RE.fullmatch(key):
                raise RuntimeError(
                    f'Invalid project .env key at {env_path}:{line_number}'
                )
            if key not in SUPPORTED_ENV_KEYS:
                raise RuntimeError(
                    f'Unsupported project .env key {key!r} at {env_path}:{line_number}'
                )
            if key in values:
                raise RuntimeError(
                    f'Duplicate project .env key {key!r} at {env_path}:{line_number}'
                )
            values[key] = value

    missing = sorted(REQUIRED_ENV_KEYS - values.keys())
    if missing:
        raise RuntimeError(
            'Project .env is missing required key(s): ' + ', '.join(missing)
        )
    return values


def _canonical_integer(values: dict[str, str], key: str, minimum: int) -> int:
    text = values[key]
    if (
        not text.isascii()
        or not text.isdecimal()
        or str(int(text)) != text
        or int(text) < minimum
    ):
        raise RuntimeError(f'{key} must be a canonical integer >= {minimum}')
    return int(text)


def _positive_number(values: dict[str, str], key: str) -> float:
    text = values[key]
    try:
        value = float(text)
    except ValueError as exc:
        raise RuntimeError(f'{key} must be a finite number greater than zero') from exc
    if not math.isfinite(value) or value <= 0.0:
        raise RuntimeError(f'{key} must be a finite number greater than zero')
    return value


def _boolean(values: dict[str, str], key: str) -> bool:
    text = values[key]
    if text not in {'true', 'false'}:
        raise RuntimeError(f'{key} must be exactly true or false')
    return text == 'true'


def validate_project_config(
    values: dict[str, str],
    *,
    require_camera_ready: bool,
) -> None:
    if set(values) != REQUIRED_ENV_KEYS:
        missing = sorted(REQUIRED_ENV_KEYS - values.keys())
        extra = sorted(values.keys() - REQUIRED_ENV_KEYS)
        detail = []
        if missing:
            detail.append('missing=' + ','.join(missing))
        if extra:
            detail.append('unsupported=' + ','.join(extra))
        raise RuntimeError('Invalid project configuration key set: ' + ' '.join(detail))

    if values['ROS_LOCALHOST_ONLY'] != '1':
        raise RuntimeError('ROS_LOCALHOST_ONLY must be exactly 1')

    for key in ('DOBOT_ROBOT_LAN1_IP', 'DOBOT_ROBOT_LAN2_IP'):
        try:
            ipaddress.IPv4Address(values[key])
        except ipaddress.AddressValueError as exc:
            raise RuntimeError(f'{key} must be a valid IPv4 address') from exc
    if values['DOBOT_ROBOT_LAN1_IP'] == values['DOBOT_ROBOT_LAN2_IP']:
        raise RuntimeError('Dobot LAN1 and LAN2 addresses must be different')
    connection_timeout = _canonical_integer(values, 'DOBOT_CONNECTION_TIMEOUT_MS', 100)
    if connection_timeout > 60000:
        raise RuntimeError('DOBOT_CONNECTION_TIMEOUT_MS must not exceed 60000')
    if values['DOBOT_ROBOT_TYPE'] != 'cr10':
        raise RuntimeError('DOBOT_ROBOT_TYPE must be exactly cr10')
    if values['DOBOT_ROBOT_NUMBER'] != '1':
        raise RuntimeError('DOBOT_ROBOT_NUMBER must be exactly 1')
    _positive_number(values, 'DOBOT_TRAJECTORY_DURATION')
    if not _ROS_NAME_RE.fullmatch(values['DOBOT_ROBOT_NODE_NAME']):
        raise RuntimeError('DOBOT_ROBOT_NODE_NAME must be a valid ROS node name')

    active_names: list[str] = []
    active_serials: list[str] = []
    for slot in ORBBEC_CAMERA_SLOTS:
        name_key = f'ORBBEC_CAMERA_{slot}_NAME'
        serial_key = f'ORBBEC_CAMERA_{slot}_SERIAL'
        name = values[name_key]
        serial = values[serial_key]
        if name and not _ROS_NAME_RE.fullmatch(name):
            raise RuntimeError(f'{name_key} must be a valid ROS name')
        if serial and not _SERIAL_RE.fullmatch(serial):
            raise RuntimeError(f'{serial_key} contains invalid characters')
        if require_camera_ready and not name:
            raise RuntimeError(f'{name_key} must not be empty before camera launch')
        if require_camera_ready and not serial:
            raise RuntimeError(f'{serial_key} must not be empty before camera launch')
        if name:
            active_names.append(name)
        if serial:
            active_serials.append(serial)
    if len(active_names) != len(set(active_names)):
        raise RuntimeError('Configured Orbbec camera names must be unique')
    if len(active_serials) != len(set(active_serials)):
        raise RuntimeError('Configured Orbbec serial numbers must be unique')

    if values['ORBBEC_DEVICE_PRESET'] != 'High Accuracy':
        raise RuntimeError('ORBBEC_DEVICE_PRESET must be exactly High Accuracy')
    for key in (
        'ORBBEC_ENABLE_COLOR',
        'ORBBEC_ENABLE_DEPTH',
        'ORBBEC_DEPTH_REGISTRATION',
        'ORBBEC_ENABLE_FRAME_SYNC',
        'ORBBEC_ENABLE_TEMPORAL_FILTER',
        'ORBBEC_ENABLE_POINT_CLOUD',
        'ORBBEC_ENUMERATE_NET_DEVICE',
    ):
        _boolean(values, key)
    if values['ORBBEC_ENABLE_COLOR'] != 'true':
        raise RuntimeError('ORBBEC_ENABLE_COLOR must be exactly true')
    if values['ORBBEC_ENABLE_DEPTH'] != 'true':
        raise RuntimeError('ORBBEC_ENABLE_DEPTH must be exactly true')
    if values['ORBBEC_ENUMERATE_NET_DEVICE'] != 'false':
        raise RuntimeError('ORBBEC_ENUMERATE_NET_DEVICE must be exactly false')
    if values['ORBBEC_ALIGN_TARGET_STREAM'] != 'COLOR':
        raise RuntimeError('ORBBEC_ALIGN_TARGET_STREAM must be exactly COLOR')
    if values['ORBBEC_ALIGN_MODE'] != 'SW':
        raise RuntimeError('ORBBEC_ALIGN_MODE must be exactly SW')

    for key in (
        'ORBBEC_COLOR_WIDTH',
        'ORBBEC_COLOR_HEIGHT',
        'ORBBEC_COLOR_FPS',
        'ORBBEC_DEPTH_WIDTH',
        'ORBBEC_DEPTH_HEIGHT',
        'ORBBEC_DEPTH_FPS',
    ):
        _canonical_integer(values, key, 1)
    for key in (
        'ORBBEC_SCAN_TIMEOUT_SEC',
        'ORBBEC_STARTUP_TIMEOUT_SEC',
        'ORBBEC_HEALTH_TIMEOUT_SEC',
        'ORBBEC_CHECK_PERIOD_SEC',
        'ORBBEC_SHUTDOWN_TIMEOUT_SEC',
    ):
        _positive_number(values, key)
    if values['ORBBEC_STARTUP_TIMEOUT_SEC'] != '5':
        raise RuntimeError('ORBBEC_STARTUP_TIMEOUT_SEC must be exactly 5')
    if values['ORBBEC_MAX_ATTEMPTS'] != '3':
        raise RuntimeError('ORBBEC_MAX_ATTEMPTS must be exactly 3')
    if values['ORBBEC_RETRY_DELAY_SEC'] != '3':
        raise RuntimeError('ORBBEC_RETRY_DELAY_SEC must be exactly 3')


def load_project_config(*, require_camera_ready: bool) -> tuple[Path, dict[str, str]]:
    root = project_root()
    values = _parse_env_file(root / '.env')
    validate_project_config(values, require_camera_ready=require_camera_ready)
    return root, values


def camera_specs(values: dict[str, str]) -> list[CameraSpec]:
    return [
        CameraSpec(
            slot=slot,
            name=values[f'ORBBEC_CAMERA_{slot}_NAME'],
            serial_number=values[f'ORBBEC_CAMERA_{slot}_SERIAL'],
        )
        for slot in ORBBEC_CAMERA_SLOTS
    ]


def orbbec_launch_arguments(values: dict[str, str]) -> dict[str, str]:
    return {
        'device_preset': values['ORBBEC_DEVICE_PRESET'],
        'enable_color': values['ORBBEC_ENABLE_COLOR'],
        'enable_depth': values['ORBBEC_ENABLE_DEPTH'],
        'depth_registration': values['ORBBEC_DEPTH_REGISTRATION'],
        'align_target_stream': values['ORBBEC_ALIGN_TARGET_STREAM'],
        'align_mode': values['ORBBEC_ALIGN_MODE'],
        'enable_frame_sync': values['ORBBEC_ENABLE_FRAME_SYNC'],
        'enable_temporal_filter': values['ORBBEC_ENABLE_TEMPORAL_FILTER'],
        'color_width': values['ORBBEC_COLOR_WIDTH'],
        'color_height': values['ORBBEC_COLOR_HEIGHT'],
        'color_fps': values['ORBBEC_COLOR_FPS'],
        'depth_width': values['ORBBEC_DEPTH_WIDTH'],
        'depth_height': values['ORBBEC_DEPTH_HEIGHT'],
        'depth_fps': values['ORBBEC_DEPTH_FPS'],
        'enable_point_cloud': values['ORBBEC_ENABLE_POINT_CLOUD'],
        'enumerate_net_device': values['ORBBEC_ENUMERATE_NET_DEVICE'],
    }


def write_orbbec_config(updates: dict[str, str]) -> tuple[Path, dict[str, str]]:
    if set(updates) != set(ORBBEC_ENV_KEYS):
        missing = sorted(set(ORBBEC_ENV_KEYS) - updates.keys())
        extra = sorted(updates.keys() - set(ORBBEC_ENV_KEYS))
        raise RuntimeError(
            'GUI must write the complete Orbbec configuration; '
            f'missing={missing} unsupported={extra}'
        )
    if any('\n' in value or '\r' in value for value in updates.values()):
        raise RuntimeError('Orbbec configuration values cannot contain newlines')

    root, current = load_project_config(require_camera_ready=False)
    merged = dict(current)
    merged.update(updates)
    validate_project_config(merged, require_camera_ready=True)

    env_path = root / '.env'
    output_lines: list[str] = []
    replaced: set[str] = set()
    with env_path.open('r', encoding='utf-8') as env_file:
        for raw_line in env_file:
            line = raw_line.rstrip('\r\n')
            if line and not line.startswith('#') and '=' in line:
                key = line.split('=', 1)[0]
                if key in ORBBEC_ENV_KEYS:
                    output_lines.append(f'{key}={updates[key]}\n')
                    replaced.add(key)
                    continue
            output_lines.append(raw_line)
    missing_lines = sorted(set(ORBBEC_ENV_KEYS) - replaced)
    if missing_lines:
        raise RuntimeError(
            'Root .env is missing Orbbec lines required for atomic update: '
            + ', '.join(missing_lines)
        )

    original_mode = stat.S_IMODE(env_path.stat().st_mode)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            dir=env_path.parent,
            prefix='.env.orbbec.',
            delete=False,
        ) as temporary:
            temporary.writelines(output_lines)
            temporary.flush()
            os.fsync(temporary.fileno())
            temporary_path = Path(temporary.name)
        temporary_path.chmod(original_mode)
        os.replace(temporary_path, env_path)
        temporary_path = None
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)

    return root, merged
