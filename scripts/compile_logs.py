#!/usr/bin/env python3
"""Merge isolated package JSONL logs into one timestamp-ordered file."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path


def _timestamp(value: object, source: Path) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{source}: every event must have a non-empty timestamp")
    try:
        datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError(f"{source}: invalid event timestamp: {value!r}") from exc
    return value


def compile_logs(workspace_root: Path, output: Path) -> int:
    log_root = workspace_root / "logs"
    if not log_root.is_dir():
        raise FileNotFoundError(f"package log directory is missing: {log_root}")

    output = output.resolve()
    records = []
    input_files = sorted(log_root.glob("*/events.jsonl"))
    if not input_files:
        raise FileNotFoundError(f"no package event logs found below: {log_root}")

    for source in input_files:
        if source.resolve() == output:
            continue
        with source.open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, start=1):
                try:
                    record = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{source}:{line_number}: invalid JSON event") from exc
                if not isinstance(record, dict):
                    raise ValueError(f"{source}:{line_number}: event must be a JSON object")
                record["timestamp"] = _timestamp(record.get("timestamp"), source)
                package = record.get("package")
                if not isinstance(package, str) or not package:
                    raise ValueError(f"{source}:{line_number}: event has no package name")
                records.append(record)

    records.sort(key=lambda record: (record["timestamp"], record["package"], record.get("event", "")))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        for record in records:
            stream.write(json.dumps(record, separators=(",", ":")) + "\n")
    return len(records)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args()
    workspace_root = arguments.workspace_root.resolve()
    output = arguments.output
    if not output.is_absolute():
        output = workspace_root / output
    count = compile_logs(workspace_root, output)
    print(f"compiled {count} events into {output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
