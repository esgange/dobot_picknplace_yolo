"""Single lifetime native child with serialized, bounded requests. No native imports."""

import os
from pathlib import Path
import queue
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix

from .preview_protocol import receive_packet, send_packet


class NativeClient:
    def __init__(self, events):
        self.events = events
        self.lock = threading.Lock()
        self.process_lock = threading.Lock()
        self.process = None
        self.failed = False
        self.closed = False
        self.ready = False
        self.stderr_tail = bytearray()

    def _start(self):
        with self.process_lock:
            if self.closed or self.failed:
                raise RuntimeError("Native worker closed/failed; no replacement allowed")
            if self.process is not None:
                return
            directory = Path(get_package_prefix("item_perception_yolo")) / \
                "lib/item_perception_yolo"
            environment = dict(os.environ, YOLO_AUTOINSTALL="false", YOLO_OFFLINE="true",
                               QT_QPA_PLATFORM="offscreen", MPLBACKEND="Agg", OMP_NUM_THREADS="4",
                               MKL_NUM_THREADS="4", OPENBLAS_NUM_THREADS="1")
            self.process = subprocess.Popen(
                ["/usr/bin/python3", str(directory / "item_preview_worker"),
                 str(directory / "yolo_runtime")], stdin=subprocess.PIPE,
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

    def call(self, header, data=b"", timeout=30.0):
        deadline = time.monotonic() + timeout
        if not self.lock.acquire(timeout=timeout):
            raise TimeoutError("Native worker busy until request deadline")
        try:
            if self.failed or self.closed:
                raise RuntimeError("Native worker closed/failed; no replacement allowed")
            result = queue.Queue(maxsize=1)

            def exchange():
                try:
                    self._start()
                    if not self.ready:
                        startup, payload = receive_packet(self.process.stdout)
                        if startup.get("state") != "ready" or payload:
                            raise RuntimeError(f"Invalid native startup: {startup}")
                        self.events.record("INFO", "item_runtime_ready", "Private CPU runtime",
                                           **startup)
                        self.ready = True
                    send_packet(self.process.stdin, header, data)
                    metadata, payload = receive_packet(self.process.stdout)
                    if metadata.get("state") != "ok":
                        raise RuntimeError(metadata.get("error", "Malformed native reply"))
                    result.put((metadata, payload, None))
                except Exception as exc:
                    result.put((None, None, exc))
            threading.Thread(target=exchange, daemon=True).start()
            try:
                metadata, payload, error = result.get(
                    timeout=max(0.001, deadline - time.monotonic()))
                if error is not None:
                    raise error
                return metadata, payload
            except Exception as exc:
                if self.closed and not self.failed:
                    raise RuntimeError("Native operation cancelled by shutdown") from exc
                self.failed = True
                process = self.process
                self.events.record("FATAL", "item_worker_failed", str(exc),
                                   pid=None if process is None else process.pid,
                                   exit_code=None if process is None else process.poll(),
                                   operation=header.get("operation"), width=header.get("width"),
                                   height=header.get("height"),
                                   stderr_tail=bytes(self.stderr_tail).decode(
                                       "utf-8", errors="replace"))
                self.close()
                raise RuntimeError(f"Terminal item worker failure: {exc}") from exc
        finally:
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
