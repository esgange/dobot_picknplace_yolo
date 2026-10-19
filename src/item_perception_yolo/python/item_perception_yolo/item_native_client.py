"""Single lifetime native child with serialized, bounded requests. No native imports."""

import os
from pathlib import Path
import queue
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix

from .preview_protocol import receive_packet, send_packet
from . import cpu_budget


class NativeClient:
    def __init__(self, events, *, worker_package="item_perception_yolo",
                 worker_executable="item_preview_worker", torch_threads=None, root=None):
        self.events = events
        self.worker_package = worker_package
        self.worker_executable = worker_executable
        self.role = 'tray' if worker_package == 'tray_perception' else 'item'
        self.thread_policy = None
        self.tuning = {}
        if torch_threads is None:
            from camera_calibration_gui.calibration_core import workspace_root
            from orbbec_camera_launcher.project_config import _parse_env_file
            self.root = workspace_root() if root is None else Path(root)
            hardware = cpu_budget.discover()
            allocation = cpu_budget.allocate(hardware, cpu_budget.settings(
                _parse_env_file(self.root / '.env')))
            self.thread_policy = {'hardware': hardware, 'allocation': allocation}
            torch_threads = allocation['limits'][self.role]
            self.events.record('INFO', 'perception_cpu_budget', 'Selected CPU budget',
                               role=self.role, threads=torch_threads, **self.thread_policy)
        self.thread_limit = self.torch_threads = cpu_budget.checked_threads(torch_threads)
        self.lock = threading.Lock()
        self.process_lock = threading.Lock()
        self.process = None
        self.failed = False
        self.closed = False
        self.ready = False
        self.retiring = False
        self.stderr_tail = bytearray()

    def _start(self):
        with self.process_lock:
            if self.closed or self.failed:
                raise RuntimeError("Native worker closed/failed; no replacement allowed")
            if self.process is not None:
                return
            directory = Path(get_package_prefix("item_perception_yolo")) / \
                "lib/item_perception_yolo"
            executable = Path(get_package_prefix(self.worker_package)) / \
                "lib" / self.worker_package / self.worker_executable
            if self.thread_policy is not None:
                path = self.root / 'logs/perception_cpu_tuning.json'
                if self.thread_policy['allocation']['overrides'][self.role] is not None:
                    self.tuning, reason = {}, 'explicit thread override'
                elif not path.exists():
                    self.tuning, reason = {}, 'no measurements'
                else:
                    identity = {**self.thread_policy,
                                'runtime': cpu_budget.runtime_identity(directory / 'yolo_runtime')}
                    self.tuning, reason = cpu_budget.read_tuning(path, identity)
                self.events.record('INFO', 'perception_cpu_tuning', reason, role=self.role)
            environment = dict(os.environ, YOLO_AUTOINSTALL="false", YOLO_OFFLINE="true",
                               QT_QPA_PLATFORM="offscreen", MPLBACKEND="Agg",
                               OMP_NUM_THREADS=str(self.torch_threads),
                               MKL_NUM_THREADS=str(self.torch_threads), OPENBLAS_NUM_THREADS="1")
            self.process = subprocess.Popen(
                ["/usr/bin/python3", str(executable),
                 str(directory / "yolo_runtime"), str(self.thread_limit)], stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=environment)
            process = self.process

        def drain():
            while True:
                block = process.stderr.read(1024)
                if not block:
                    return
                self.stderr_tail.extend(block)
                del self.stderr_tail[:-8192]
        threading.Thread(target=drain, daemon=True).start()

    def _select_threads(self, header):
        key = cpu_budget.workload_key(self.role, header)
        selected = self.tuning.get(key, self.thread_limit)
        if not 1 <= selected <= self.thread_limit:
            selected = self.thread_limit
        if selected == self.torch_threads:
            return
        send_packet(self.process.stdin, {'operation': 'thread_budget', 'threads': selected})
        reply, payload = receive_packet(self.process.stdout)
        if reply != {'state': 'ok', 'torch_threads': selected} or payload:
            raise RuntimeError(f'Invalid native thread budget reply: {reply}')
        self.torch_threads = selected
        self.events.record('INFO', 'perception_cpu_selected', 'Applied measured/default CPU budget',
                           role=self.role, threads=selected, workload=key,
                           measured=key in self.tuning)

    def _failure(self, header, exc):
        if self.closed and not self.failed:
            return RuntimeError("Native operation cancelled by shutdown")
        self.failed = True
        process = self.process
        self.events.record("FATAL", "item_worker_failed", str(exc),
                           pid=None if process is None else process.pid,
                           exit_code=None if process is None else process.poll(),
                           operation=header.get("operation"), width=header.get("width"),
                           height=header.get("height"),
                           stderr_tail=bytes(self.stderr_tail).decode("utf-8", errors="replace"))
        self.close()
        return RuntimeError(f"Terminal item worker failure: {exc}")

    def call(self, header, data=b"", timeout=30.0, *, request_timeout=None):
        """An optional shorter caller deadline discards/drains, retaining native ownership."""
        deadline = time.monotonic() + timeout
        wait_deadline = (deadline if request_timeout is None else
                         min(deadline, time.monotonic() + max(0., request_timeout)))
        if not self.lock.acquire(timeout=max(0., wait_deadline - time.monotonic())):
            raise TimeoutError("Native worker busy until request deadline")
        retiring = False
        try:
            if time.monotonic() >= wait_deadline:
                raise TimeoutError("Native request expired before dispatch")
            if self.failed or self.closed:
                raise RuntimeError("Native worker closed/failed; no replacement allowed")
            result = queue.Queue(maxsize=1)

            def exchange():
                try:
                    self._start()
                    if not self.ready:
                        startup, payload = receive_packet(self.process.stdout)
                        if (startup.get("state") != "ready" or payload
                                or startup.get("torch_threads") != self.torch_threads
                                or startup.get("torch_interop_threads") != 1
                                or startup.get("opencv_threads") != 1
                                or startup.get("opencl") is not False):
                            raise RuntimeError(f"Invalid native startup: {startup}")
                        self.events.record("INFO", "item_runtime_ready", "Private CPU runtime",
                                           **startup)
                        self.ready = True
                    self._select_threads(header)
                    send_packet(self.process.stdin, header, data)
                    metadata, payload = receive_packet(self.process.stdout)
                    if metadata.get("state") != "ok":
                        raise RuntimeError(metadata.get("error", "Malformed native reply"))
                    result.put((metadata, payload, None))
                except Exception as exc:
                    result.put((None, None, exc))
            threading.Thread(target=exchange, daemon=True).start()
            try:
                try:
                    metadata, payload, error = result.get(
                        timeout=max(0., wait_deadline - time.monotonic()))
                except queue.Empty:
                    if request_timeout is None or time.monotonic() >= deadline:
                        raise
                    # The caller can return, but no other request can touch the
                    # pipe/model until this exact response drains or hard-fails.
                    def retire():
                        try:
                            _, _, error = result.get(timeout=max(0., deadline-time.monotonic()))
                            if error is not None:
                                raise error
                        except Exception as exc:
                            self._failure(header, exc)
                        finally:
                            self.lock.release()
                            self.retiring = False
                    retiring = True
                    self.retiring = True
                    threading.Thread(target=retire, daemon=True).start()
                    raise TimeoutError("Native request expired; previous operation is retiring")
                if error is not None:
                    raise error
                return metadata, payload
            except Exception as exc:
                if retiring:
                    raise
                raise self._failure(header, exc) from exc
        finally:
            if not retiring:
                self.lock.release()

    def close(self):
        with self.process_lock:
            self.closed = True
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
