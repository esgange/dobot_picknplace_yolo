"""Observed cycle phases and overlapping acquisition timings; never a motion gate."""

from collections import Counter, defaultdict
from datetime import datetime, timedelta
import csv
import json
import math
from pathlib import Path
import statistics

from item_perception_yolo.daily_events import RETENTION_DAYS, log_day


def distribution(values):
    values = sorted(float(v) for v in values if isinstance(v, (float, int))
                    and math.isfinite(v) and v >= 0)
    if not values:
        return {"count": 0}
    return {"count": len(values), "mean": statistics.mean(values),
            "median": statistics.median(values),
            "p95": values[max(0, math.ceil(.95 * len(values)) - 1)],
            "min": values[0], "max": values[-1], "sum": sum(values)}


class RunStudy:
    def __init__(self, run_id, quantity):
        self.run_id, self.quantity = run_id, quantity
        self.cycle = 1
        self.phase = "initial_preparation"
        self.last_ns = self.start_ns = None
        self.rows = defaultdict(lambda: defaultdict(float))
        self.completed = set()
        self.item_requests = []
        self.tray_requests = []
        self.tray_failures = Counter()
        self.counts = Counter()
        self.paused_phase = None
        self.finished = False

    def _advance(self, stamp, phase=None):
        if self.last_ns is None:
            self.start_ns = self.last_ns = stamp
        seconds = max(0., (stamp - self.last_ns) / 1e9)
        self.rows[self.cycle][self.phase] += seconds
        self.last_ns = max(self.last_ns, stamp)
        if phase is not None:
            self.phase = phase

    def observe(self, event):
        if self.finished:
            return
        name, stamp = event["event"], event["monotonic_ns"]
        self._advance(stamp)
        if name == "candidate_timing":
            self.item_requests.append({key: event.get(key) for key in (
                "cycle_index", "batch_id", "total_ms", "service_wait_ms", "detector_timings_ms")})
        if name == "tray_request_timing":
            self.tray_requests.append({key: event.get(key) for key in (
                "cycle_index", "attempt", "status", "usable", "total_ms", "provider_timings_ms",
                "batch_id", "request_id", "reason", "worker_timings_ms", "rejection_category")})
            if not event.get("usable"):
                self.tray_failures[event.get("status", "UNKNOWN")] += 1
        if name in ("repick_attempt", "pick_empty_pose_retry", "pick_batch_exhausted",
                    "tray_observation_attempt_failed", "held_suction_lost"):
            self.counts[name] += 1
        if name == "auto_run_placement_completed":
            self.completed.add(event["completed"])
            self.cycle = event["completed"] + 1
            self.phase = ("final_home" if event["completed"] == self.quantity else
                          "pick_approach" if event.get("boundary_command_id") is not None
                          else "item_wait")
        elif name == "pickup_detected":
            self.phase = "pickup_confirmation_gripper"
        elif name == "placement_depth":
            self.phase = "placement_preparation"
        elif name == "motion_batch_dispatch_started":
            batch = event.get("batch", "")
            if batch == "place_to_retract":
                self.phase = "placement_and_handoff"
            elif "pick_to_tray" in batch:
                self.phase = "lift_and_tray_transport"
            elif self.phase != "placement_and_handoff":
                if "home_to_pick" in batch or "place_to_pick" in batch:
                    self.phase = "pick_approach"
                elif any(word in batch for word in (
                        "retry", "return", "put_back", "miss_retract", "pick_to_home")):
                    self.phase = "retry_return"
        elif name == "motion_batch_completed" and event.get("terminal_target") == \
                "tray_detect_position":
            self.phase = "tray_preparation"
        elif name == "operation_phase":
            phase, message = event.get("phase"), event.get("message", "")
            if phase == "TRAY_DEPTH" and "requesting" in message:
                self.phase = "tray_acquisition"
            elif phase == "DETECT" and self.phase != "placement_and_handoff":
                self.phase = "item_wait"
        elif name == "state_transition":
            state = event.get("state")
            if state == "PAUSED":
                self.paused_phase, self.phase = self.phase, "paused"
            elif event.get("previous") == "PAUSED":
                self.phase = self.paused_phase or "recovery"
            elif state in ("STOPPING", "RECOVERING", "RETURNING_ITEM"):
                self.phase = "recovery"

    def finish(self, directory, result):
        self.finished = True
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        day = log_day()
        cutoff = day - timedelta(days=RETENTION_DAYS - 1)
        for path in directory.glob("run-????-??-??-*.json"):
            identifier = path.stem[15:]
            if len(identifier) != 32 or any(c not in "0123456789abcdef" for c in identifier):
                continue
            try:
                expired = datetime.strptime(path.name[4:14], "%Y-%m-%d").date() < cutoff
            except ValueError:
                continue
            if expired:
                for target in (path, path.with_suffix(".csv")):
                    if target.is_file() and not target.is_symlink():
                        target.unlink(missing_ok=True)
        base = directory / f"run-{day}-{self.run_id}"
        columns = sorted({key for row in self.rows.values() for key in row})
        cycles = [{"cycle_index": index, "completed": index in self.completed,
                   "kind": "final_home" if index > self.quantity else "cycle",
                   "total_sec": sum(row.values()), **dict(row)}
                  for index, row in sorted(self.rows.items())]
        grouped = defaultdict(list)
        for request in self.tray_requests:
            grouped[request["cycle_index"]].append(request)
        summary = {
            "schema_version": 1, "run_id": self.run_id, "result": result,
            "timing_basis": "controller-observed group boundaries; includes dispatch/feedback lag",
            "placement_boundary": "confirmed handoff into next pre-pick or Home, or idle retract",
            "overlap_policy": "acquisition durations are separate; phase seconds are exclusive",
            "observed_elapsed_sec": sum(row["total_sec"] for row in cycles),
            "cycles": cycles, "phase_totals_sec": {
                key: sum(row.get(key, 0.) for row in cycles) for key in columns},
            "cycle_duration_sec": distribution([r["total_sec"] for r in cycles if r["completed"]]),
            "item_requests": self.item_requests, "tray_requests": self.tray_requests,
            "tray_request_ms": distribution([r["total_ms"] for r in self.tray_requests]),
            "tray_acquisition_ms": distribution([
                sum(r["total_ms"] or 0 for r in requests) for requests in grouped.values()]),
            "tray_cycles_observed": len(grouped),
            "tray_first_request_usable": sum(bool(v[0]["usable"]) for v in grouped.values()),
            "tray_failures_by_status": dict(self.tray_failures), "counts": dict(self.counts),
            "tray_failed_request_time_ms": sum(r["total_ms"] or 0 for r in self.tray_requests
                                               if not r["usable"]),
            "tray_worker_timings_ms": {key: distribution([
                r.get("worker_timings_ms", {}).get(key) for r in self.tray_requests
                if isinstance(r.get("worker_timings_ms"), dict)])
                for key in sorted({key for r in self.tray_requests
                                   for key in (r.get("worker_timings_ms") or {})})},
            "tray_model_accuracy": "unlabelled observations; usable rate is not model accuracy",
        }
        for target, content in [(base.with_suffix('.json'), json.dumps(summary, indent=2))]:
            with target.open("x", encoding="utf-8") as stream:
                stream.write(content + "\n")
        with base.with_suffix('.csv').open("x", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=[
                "cycle_index", "completed", "kind", "total_sec", *columns])
            writer.writeheader()
            writer.writerows(cycles)
        return {"json_path": str(base.with_suffix('.json')),
                "csv_path": str(base.with_suffix('.csv'))}
