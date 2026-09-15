"""Incremental, bounded recording for the T-block pushing experiment.

The control loop only copies/serializes data and enqueues work. A single worker
writes native-resolution camera images and JSONL records in submission order.
Queue exhaustion or a disk error is fatal and is exposed by ``check()``; records
are never silently dropped. Call ``check()`` every control iteration and stop
motion before ``finish()`` drains the queue.

Camera timestamps describe PyCAAS's host read-completion clock, not camera
exposure time. Keep robot and command timestamps in the supplied state records;
this writer does not manufacture synchronized camera/robot measurements.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import queue
import tempfile
import threading
import time

import cv2
import numpy as np


class RecordingError(RuntimeError):
    """The episode can no longer be recorded without losing data."""


def json_value(value):
    """Return a detached JSON value, rejecting non-finite measurements."""
    if isinstance(value, np.ndarray):
        return json_value(value.tolist())
    if isinstance(value, np.generic):
        return json_value(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        if not all(isinstance(key, str) for key in value):
            raise TypeError("Recorded JSON object keys must be strings")
        return {key: json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item) for item in value]
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("Cannot record a non-finite measurement")
        return value
    if value is None or isinstance(value, (str, int, bool)):
        return value
    raise TypeError(f"Cannot record a {type(value).__name__} as JSON")


def _json_line(value):
    return json.dumps(json_value(value), allow_nan=False, separators=(",", ":")) + "\n"


def atomic_json(path, value):
    """Atomically replace one JSON document, flushing its contents to disk."""
    path = Path(path)
    payload = json.dumps(json_value(value), allow_nan=False, indent=2) + "\n"
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=f".{path.name}.", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


class EpisodeWriter:
    """Create an exclusive episode directory and asynchronously save its data.

    ``record_frame`` requires ``camera_timestamp_s`` (or ``timestamp``), accepts
    an unannotated uint8 BGR image, and preserves all supplied detector data.
    Record images even when detection fails, with a null pose/failed detection
    in ``frame_record``. ``image_format="png"`` preserves exact pixels;
    ``image_format="jpeg"`` saves disk space with JPEG quality 95. Equal
    consecutive timestamps return False; decreasing timestamps are rejected.
    No set of historical frames is retained in RAM.

    JSONL files are flushed after each record and fsynced at least once per
    ``sync_interval`` while running and again at finish. Every image is fsynced
    before its camera record is appended, so a saved record refers to a complete
    image. ``episode.json`` starts with status ``recording`` and is atomically
    replaced on finish, making unfinished processes identifiable on recovery.
    """

    def __init__(self, episode_dir, metadata, *, queue_size=256, sync_interval=1.0, image_format="png"):
        if isinstance(queue_size, bool) or int(queue_size) != queue_size or queue_size < 1:
            raise ValueError("queue_size must be a positive integer")
        if not math.isfinite(sync_interval) or sync_interval <= 0:
            raise ValueError("sync_interval must be positive and finite")
        if image_format not in ("png", "jpeg"):
            raise ValueError("image_format must be 'png' or 'jpeg'")
        self.image_format = image_format
        self._extension = ".png" if image_format == "png" else ".jpg"
        self.directory = Path(episode_dir)
        self.metadata = json_value(metadata)
        if not isinstance(self.metadata, dict):
            raise TypeError("Episode metadata must be a dictionary")
        self.started_at = time.time()
        self._queue = queue.Queue(maxsize=int(queue_size))
        self._sync_interval = float(sync_interval)
        self._closing = threading.Event()
        self._lock = threading.Lock()
        self._worker_error = None
        self._producer_error = None
        self._summary = None
        self._last_timestamp = None
        self._accepted_states = 0
        self._accepted_frames = 0
        self._saved_states = 0
        self._saved_frames = 0
        self._duplicates = 0
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory / "rgb").mkdir()
        atomic_json(self.directory / "episode.json", self._manifest("recording", None))
        self._worker = threading.Thread(target=self._write_loop, name="t-pushing-recorder", daemon=True)
        self._worker.start()

    def _manifest(self, outcome, reason):
        return {
            "schema_version": 1,
            "status": outcome,
            "reason": reason,
            "started_at_unix_s": self.started_at,
            "finished_at_unix_s": None if outcome == "recording" else time.time(),
            "metadata": self.metadata,
            "files": {"states": "states.jsonl", "camera": "camera.jsonl", "rgb": "rgb"},
            "image_format": f"{self.image_format.upper()}, native resolution; input arrays are BGR8",
            "image_codec": self.image_format,
            "image_lossless": self.image_format == "png",
            "png_compression_level": 0 if self.image_format == "png" else None,
            "jpeg_quality": 95 if self.image_format == "jpeg" else None,
            "camera_clock": "PyCAAS host Unix read-completion time, not exposure time",
            "synchronization": "Camera and robot timestamps are recorded separately",
            "state_count": self._saved_states,
            "frame_count": self._saved_frames,
            "accepted_state_count": self._accepted_states,
            "accepted_frame_count": self._accepted_frames,
            "duplicate_frame_count": self._duplicates,
            "unwritten_state_count": self._accepted_states - self._saved_states,
            "unwritten_frame_count": self._accepted_frames - self._saved_frames,
        }

    def check(self):
        """Raise immediately if queue exhaustion or a worker failure occurred."""
        error = self._producer_error or self._worker_error
        if error is not None:
            raise RecordingError(f"Recording failed in {self.directory}: {error}") from error

    def _enqueue(self, item):
        # Caller holds _lock, keeping close, counters and timestamp checks ordered.
        self.check()
        if self._closing.is_set():
            raise RecordingError("Cannot append to a finished episode")
        try:
            self._queue.put_nowait(item)
        except queue.Full as exc:
            self._producer_error = RecordingError("Recording queue is full; stop the episode to avoid missing data")
            raise self._producer_error from exc

    def record_state(self, record):
        if not isinstance(record, dict):
            raise TypeError("A state record must be a dictionary")
        line = _json_line(record)
        with self._lock:
            self._enqueue(("state", line))
            self._accepted_states += 1

    def record_frame(self, image_bgr, frame_record):
        if not isinstance(frame_record, dict):
            raise TypeError("A camera record must be a dictionary")
        record = json_value(frame_record)
        timestamp = record.get("camera_timestamp_s", record.get("timestamp"))
        if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
            raise ValueError("Camera records require a finite camera_timestamp_s or timestamp")
        image = np.asarray(image_bgr)
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or min(image.shape[:2]) < 1:
            raise ValueError("Camera image must be a nonempty native uint8 BGR image")
        if any(key in record for key in ("image", "image_shape", "frame_index")):
            raise ValueError("Camera fields image, image_shape, and frame_index are assigned by the recorder")
        with self._lock:
            self.check()
            if self._closing.is_set():
                raise RecordingError("Cannot append to a finished episode")
            if self._last_timestamp is not None:
                if timestamp == self._last_timestamp:
                    self._duplicates += 1
                    return False
                if timestamp < self._last_timestamp:
                    raise ValueError("Camera timestamp moved backwards; do not record stale frames")
            index = self._accepted_frames
            record.update(frame_index=index, image=f"rgb/{index:06d}{self._extension}", image_shape=list(image.shape))
            line = _json_line(record)
            self._enqueue(("frame", image.copy(), record["image"], line))
            self._accepted_frames += 1
            self._last_timestamp = timestamp
            return True

    def _save_image(self, relative_path, image):
        # Deflate compression bottlenecks 1280x720 at 30 Hz on the recording
        # computer. Level 0 preserves exact pixels and prioritizes capture rate.
        parameters = ([cv2.IMWRITE_PNG_COMPRESSION, 0] if self.image_format == "png"
                      else [cv2.IMWRITE_JPEG_QUALITY, 95])
        success, encoded = cv2.imencode(self._extension, image, parameters)
        if not success:
            raise OSError("OpenCV could not encode a recorded camera frame")
        destination = self.directory / relative_path
        temporary = destination.with_name(destination.name + ".tmp")
        try:
            with temporary.open("xb") as stream:
                stream.write(encoded.tobytes())
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, destination)
        finally:
            temporary.unlink(missing_ok=True)

    def _write_loop(self):
        try:
            with (self.directory / "states.jsonl").open("x", encoding="utf-8", buffering=1) as states, \
                    (self.directory / "camera.jsonl").open("x", encoding="utf-8", buffering=1) as camera:
                last_sync = time.monotonic()
                try:
                    while not self._closing.is_set() or not self._queue.empty():
                        try:
                            item = self._queue.get(timeout=0.05)
                        except queue.Empty:
                            if time.monotonic() - last_sync >= self._sync_interval:
                                self._sync(states, camera)
                                last_sync = time.monotonic()
                            continue
                        try:
                            if item[0] == "state":
                                states.write(item[1])
                                self._saved_states += 1
                            else:
                                _, image, relative_path, line = item
                                self._save_image(relative_path, image)
                                camera.write(line)
                                self._saved_frames += 1
                            if time.monotonic() - last_sync >= self._sync_interval:
                                self._sync(states, camera)
                                last_sync = time.monotonic()
                        finally:
                            self._queue.task_done()
                finally:
                    self._sync(states, camera)
        except Exception as exc:
            self._worker_error = exc

    @staticmethod
    def _sync(*streams):
        for stream in streams:
            stream.flush()
            os.fsync(stream.fileno())

    def finish(self, outcome, reason=None):
        """Drain after motion stops, save final counts, and expose any write error."""
        if not isinstance(outcome, str) or not outcome or outcome == "recording":
            raise ValueError("A nonempty final outcome other than 'recording' is required")
        if reason is not None and not isinstance(reason, str):
            raise TypeError("Episode finish reason must be a string or None")
        with self._lock:
            if self._summary is not None:
                self.check()
                return self._summary
            self._closing.set()
        self._worker.join()
        error = self._producer_error or self._worker_error
        summary = self._manifest("error" if error else outcome, reason)
        if error:
            summary["recording_error"] = str(error)
            summary["requested_outcome"] = outcome
        atomic_json(self.directory / "episode.json", summary)
        self._summary = summary
        self.check()
        return summary

    close = finish

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        if exc is None:
            self.finish("completed")
        else:
            outcome = "interrupted" if isinstance(exc, (KeyboardInterrupt, SystemExit)) else "error"
            try:
                self.finish(outcome, f"{type(exc).__name__}: {exc}")
            except Exception:
                # Preserve the exception that stopped the experiment. Any worker
                # error is also persisted in episode.json when storage permits.
                pass
        return False
