from __future__ import annotations

import re
import subprocess


SERIAL_LABEL_RE = re.compile(
    r'(?:serial(?:\s+number)?|serial_number|sn)\s*[:=]\s*([A-Za-z0-9_.:-]+)',
    re.IGNORECASE,
)
GENERIC_SERIAL_RE = re.compile(r'\b[A-Za-z0-9][A-Za-z0-9_.:-]{5,}\b')


def extract_serial_numbers(text: str) -> list[str]:
    found: list[str] = []
    for match in SERIAL_LABEL_RE.finditer(text):
        candidate = match.group(1).strip().strip(',;')
        if candidate and candidate not in found:
            found.append(candidate)
    if found:
        return found

    stop_words = {
        'orbbec',
        'camera',
        'device',
        'serial',
        'number',
        'version',
        'firmware',
        'connected',
        'product',
    }
    for match in GENERIC_SERIAL_RE.finditer(text):
        candidate = match.group(0).strip().strip(',;')
        if candidate.lower() in stop_words:
            continue
        if candidate not in found:
            found.append(candidate)
    return found


def scan_connected_serials(timeout_sec: float) -> tuple[int, list[str], str]:
    command = ['ros2', 'run', 'orbbec_camera', 'list_devices_node']
    try:
        result = subprocess.run(
            command,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_sec,
        )
    except subprocess.TimeoutExpired as exc:
        combined = '\n'.join(
            part.decode(errors='replace') if isinstance(part, bytes) else part
            for part in (exc.stdout, exc.stderr)
            if part
        ).strip()
        return -1, extract_serial_numbers(combined), (combined + '\nScan timed out.').strip()
    except OSError as exc:
        return -1, [], f'Failed to execute Orbbec device scan: {exc}'

    output = (result.stdout or '').strip()
    error = (result.stderr or '').strip()
    combined = '\n'.join(part for part in (output, error) if part).strip()
    return result.returncode, extract_serial_numbers(combined), combined
