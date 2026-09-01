# Runtime datalogs

The project writes isolated package logs under this directory:

```text
logs/<package-name>/events.jsonl
```

Records are timestamped JSONL events. Each package file is capped at 1,000 records; record 1,001 truncates that package file before writing the newest event. Runtime logs are ignored by Git. Use `scripts/compile_logs.py` to merge package files into an explicitly selected universal output when needed; no logger-only ROS package owns this directory.
