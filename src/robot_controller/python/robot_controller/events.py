"""Controller-owned daily audit and observed Auto Run time studies."""

import sys

from item_perception_yolo.daily_events import DailyEventLogger
from .time_study import RunStudy


class PackageEventLogger(DailyEventLogger):
    def __init__(self, root, node_name="robot_controller"):
        super().__init__(root, "robot_controller", node_name)
        self.study = None

    def start_run(self, run_id, quantity, **metadata):
        with self.lock:
            self.context = {"run_id": run_id}
            self.study = RunStudy(run_id, quantity)
            self.observer = self.study.observe
            self.record("INFO", "auto_run_started", "Auto Run time study started",
                        requested=quantity, **metadata)

    def finish_run(self, **result):
        with self.lock:
            study, self.study = self.study, None
            self.observer = None
            if study is not None:
                try:
                    paths = study.finish(self.directory, result)
                    self.record("INFO", "auto_run_time_study", "Run timing report saved", **paths)
                except (OSError, ValueError, TypeError) as exc:
                    print(f"Run timing report failed: {exc}", file=sys.stderr)
                    self.record("WARNING", "auto_run_time_study_failed", str(exc))
            self.context = {}
