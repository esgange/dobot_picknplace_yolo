"""Single-file teach io helpers for item_pick.

Mirrors the C++ helpers in
``item_perception_eyetohand/include/item_perception_eyetohand/teach_io.hpp``.

Layout contract:
    teach_files/<kind>/<id>.yaml          # exactly one yaml per subject dir
    schema_version: 1
    <kind>: { id: <id>, ... }             # subject identity block
    teach:  { ros__parameters: { ... } }  # owned by the teach writer
    detect: { ros__parameters: { ... } }  # owned by the detect writer (optional)
    pick:   { ros__parameters: { ... } }  # owned by the pick writer (optional)

Each writer must replace only its own top-level section and round-trip sibling
sections verbatim. CATARM swaps the file atomically; nodes do not hot-reload.
Calibration files are handled separately and are not the responsibility of
these helpers.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path
from typing import Any, Iterable

import yaml


CURRENT_SCHEMA_VERSION = 1


class LayoutError(RuntimeError):
    """Raised when the on-disk layout violates the single-file-per-dir rule."""


class SchemaError(RuntimeError):
    """Raised when a subject yaml is missing the expected schema metadata."""


def _yaml_extensions() -> Iterable[str]:
    return ('.yaml', '.yml')


def resolve_single_teach_file(directory: Path, kind: str) -> Path | None:
    """Return the lone yaml in ``directory``.

    Returns ``None`` when the directory is missing or contains zero yamls.
    Raises :class:`LayoutError` when more than one yaml is present.
    """

    if not directory.exists() or not directory.is_dir():
        return None

    candidates = sorted(
        path for path in directory.iterdir()
        if path.is_file() and path.suffix in _yaml_extensions()
    )

    if not candidates:
        return None

    if len(candidates) > 1:
        listing = '\n'.join(f'  - {path.name}' for path in candidates)
        raise LayoutError(
            f'Expected exactly one {kind} yaml in {directory!s}, '
            f'found {len(candidates)}:\n{listing}'
        )

    return candidates[0]


def load_subject_yaml(file: Path, subject_kind: str) -> dict:
    """Load and validate a subject yaml. Returns the full root mapping."""

    if not file.exists():
        raise SchemaError(f'Subject yaml does not exist: {file!s}')

    try:
        with file.open('r', encoding='utf-8') as infile:
            root = yaml.safe_load(infile)
    except Exception as ex:
        raise SchemaError(f'Failed to parse {file!s}: {ex}') from ex

    if not isinstance(root, dict):
        raise SchemaError(f'Subject yaml root is not a mapping: {file!s}')

    version = root.get('schema_version')
    if version is None:
        raise SchemaError(
            f'Missing schema_version in {file!s}. '
            f'Expected schema_version: {CURRENT_SCHEMA_VERSION}'
        )
    if not isinstance(version, int):
        raise SchemaError(f'schema_version is not an integer in {file!s}')
    if version != CURRENT_SCHEMA_VERSION:
        raise SchemaError(
            f'Unsupported schema_version {version} in {file!s}. '
            f'Expected {CURRENT_SCHEMA_VERSION}'
        )

    subject = root.get(subject_kind)
    if not isinstance(subject, dict):
        raise SchemaError(
            f"Missing or invalid '{subject_kind}:' block in {file!s}"
        )

    return root


def make_fresh_subject_root(
    subject_kind: str,
    subject_id: str,
    display_name: str | None = None,
) -> dict:
    root: dict[str, Any] = {
        'schema_version': CURRENT_SCHEMA_VERSION,
        subject_kind: {'id': subject_id},
    }
    if display_name:
        root[subject_kind]['display_name'] = display_name
    return root


def write_subject_yaml(file: Path, root: dict) -> None:
    """Atomically write ``root`` to ``file`` via tmp+rename."""

    file = Path(file)
    parent = file.parent
    parent.mkdir(parents=True, exist_ok=True)

    fd, tmp_name = tempfile.mkstemp(
        prefix=file.name + '.', suffix='.tmp', dir=str(parent)
    )
    tmp_path = Path(tmp_name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as out:
            yaml.safe_dump(root, out, sort_keys=False)
        os.replace(tmp_path, file)
    except Exception:
        tmp_path.unlink(missing_ok=True)
        raise


def write_subject_section(
    file: Path,
    section_key: str,
    section_value: dict,
    *,
    subject_kind: str,
    subject_id: str,
    display_name: str | None = None,
) -> None:
    """Replace a single top-level section in ``file`` and write atomically."""

    file = Path(file)
    if file.exists():
        root = load_subject_yaml(file, subject_kind)
    else:
        if not subject_kind or not subject_id:
            raise LayoutError(
                f'Cannot seed subject yaml {file!s} without subject_kind and subject_id'
            )
        root = make_fresh_subject_root(subject_kind, subject_id, display_name)

    if display_name:
        subject = root.setdefault(subject_kind, {'id': subject_id})
        subject['display_name'] = display_name

    root[section_key] = section_value
    write_subject_yaml(file, root)


def read_subject_section(root: dict, section_key: str) -> dict | None:
    section = root.get(section_key)
    if not isinstance(section, dict):
        return None
    return section


def read_subject_params(root: dict, section_key: str) -> dict | None:
    section = read_subject_section(root, section_key)
    if section is None:
        return None
    params = section.get('ros__parameters')
    if not isinstance(params, dict):
        return None
    return params
