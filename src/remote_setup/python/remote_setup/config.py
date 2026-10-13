"""Explicit remote settings in the canonical root .env; never load shell code."""
from dataclasses import dataclass, field
import ipaddress
import math
import os
from pathlib import Path
import re
import tempfile
from urllib.parse import urlsplit

from orbbec_camera_launcher.project_config import (
    REMOTE_ENV_KEYS, _parse_env_file, project_root,
)

KEYS = REMOTE_ENV_KEYS
SECRETS = {'REMOTE_AMQP_PASSWORD', 'REMOTE_INTERNAL_API_KEY', 'REMOTE_MESSAGE_SIGNING_KEY'}


@dataclass(frozen=True)
class Settings:
    values: dict = field(repr=False)

    def __getitem__(self, name):
        return self.values['REMOTE_' + name]

    @property
    def timeout(self):
        return float(self['CONNECT_TIMEOUT_SEC'])

    @property
    def deadline(self):
        # Two probes, bounded AMQP cleanup, and child startup overhead.
        return 2 * self.timeout + 8

    @property
    def route(self):
        return f"belt.{self['BELT_ID']}.arm.{self['ARM_NUMBER']}"


def validate(values):
    if set(values) != set(KEYS):
        raise ValueError('Remote settings require the complete REMOTE_* key group')
    for key, value in values.items():
        if not isinstance(value, str) or value != value.strip() or any(
                ord(char) < 32 or ord(char) == 127 or char in '\u0085\u2028\u2029' for char in value):
            raise ValueError(f'{key}: use one line without surrounding whitespace')
        if not value and key not in {'REMOTE_INTERNAL_API_KEY', 'REMOTE_MESSAGE_SIGNING_KEY'}:
            raise ValueError(f'{key}: required')
        if len(value) > 4096:
            raise ValueError(f'{key}: value too long')
    host = values['REMOTE_SERVER_HOST']
    try:
        ipaddress.ip_address(host)
    except ValueError:
        if len(host) > 253 or any(not re.fullmatch(
                r'[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?', part)
                for part in host.split('.')):
            raise ValueError('REMOTE_SERVER_HOST: enter an IP address or DNS hostname') from None
    for key, low, high in [('REMOTE_AMQP_PORT', 1, 65535), ('REMOTE_ARM_NUMBER', 1, 8)]:
        value = values[key]
        if not value.isascii() or not value.isdecimal() or str(int(value)) != value \
                or not low <= int(value) <= high:
            raise ValueError(f'{key}: enter an integer from {low} to {high}')
    if values['REMOTE_AMQP_TLS'] not in ('true', 'false'):
        raise ValueError('REMOTE_AMQP_TLS: must be true or false')
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', values['REMOTE_BELT_ID']):
        raise ValueError('REMOTE_BELT_ID: use 1–64 letters, digits, underscores or hyphens')
    try:
        timeout = float(values['REMOTE_CONNECT_TIMEOUT_SEC'])
    except ValueError:
        timeout = math.nan
    if not math.isfinite(timeout) or not 1 <= timeout <= 60:
        raise ValueError('REMOTE_CONNECT_TIMEOUT_SEC: enter 1–60 seconds')
    try:
        url = urlsplit(values['REMOTE_CONVEYOR_API_URL'])
        port = url.port
        valid = (url.scheme in ('http', 'https') and url.hostname and not url.username
                 and not url.password and not url.query and not url.fragment
                 and (port is None or 1 <= port <= 65535)
                 and not any(c.isspace() for c in values['REMOTE_CONVEYOR_API_URL']))
    except ValueError:
        valid = False
    if not valid:
        raise ValueError(
            'REMOTE_CONVEYOR_API_URL: use an HTTP(S) base URL without credentials, query or fragment')
    return Settings(dict(values))


def read(root=None):
    root = project_root() if root is None else Path(root)
    values = _parse_env_file(root / '.env')
    return {key: values.get(key, '') for key in KEYS}


def load(root=None):
    return validate(read(root))


def save(values, root=None):
    settings = validate(values)
    root = project_root() if root is None else Path(root)
    path = root / '.env'
    _parse_env_file(path)  # Reject malformed/unknown/duplicate keys before writing.
    before = path.read_bytes()
    lines = before.decode('utf-8').splitlines(keepends=True)
    seen = set()
    output = []
    for line in lines:
        key = line.split('=', 1)[0]
        if key in KEYS:
            output.append(f'{key}={settings.values[key]}\n')
            seen.add(key)
        else:
            output.append(line)
    if output and not output[-1].endswith('\n'):
        output[-1] += '\n'
    output.extend(f'{key}={settings.values[key]}\n' for key in KEYS if key not in seen)
    descriptor, name = tempfile.mkstemp(prefix='.remote_setup-', dir=root)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            # Remote credentials are private to this station user.
            os.fchmod(stream.fileno(), 0o600)
            stream.write(''.join(output).encode('utf-8'))
            stream.flush()
            os.fsync(stream.fileno())
        if path.read_bytes() != before:
            raise ValueError('Project settings changed while saving; reload and try again')
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)
    return settings
