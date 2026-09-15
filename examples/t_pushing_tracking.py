"""Independent camera/SpaceMouse readers; importing this module opens no devices."""

import importlib.util
from pathlib import Path
import threading
import time

import numpy as np


class _MarkerArrayShapes:
    """Keep OpenCV 5 marker arrays in the shapes AprilCube 0.3 expects."""

    def __init__(self, detector):
        self.detector = detector

    def detectMarkers(self, image):
        corners, ids, rejected = self.detector.detectMarkers(image)
        return tuple(corners), None if ids is None else np.asarray(ids).reshape(-1, 1), tuple(rejected)


def create_detector(config_path, metadata, T_base_camera):
    import aprilcube

    detector = aprilcube.detector(
        config_path, np.asarray(metadata["camera_matrix"]),
        dist_coeffs=np.asarray(metadata["dist_coeffs"]), extrinsic=np.asarray(T_base_camera))
    detector.detector = _MarkerArrayShapes(detector.detector)
    detector.fallback_detector = _MarkerArrayShapes(detector.fallback_detector)
    return detector


class CameraTracker:
    """Process PyCAAS frames with AprilCube on one thread and record native results."""

    def __init__(self, camera_factory, detector_factory):
        self.camera_factory, self.detector_factory = camera_factory, detector_factory
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.latest = self.error = self.writer = self.metadata = None

    def __enter__(self):
        self.thread = threading.Thread(target=self._run, name="T-block camera", daemon=True)
        self.thread.start()
        if not self.ready.wait(5):
            self.close()
            raise RuntimeError("Camera did not become ready within five seconds")
        self.check()
        return self

    def _run(self):
        try:
            with self.camera_factory() as camera:
                detector = self.detector_factory(camera.metadata)
                self.metadata = camera.metadata
                self.ready.set()
                sequence = 0
                while not self.stop_event.is_set():
                    image, timing = camera.read()
                    result = detector.process_frame(image, timestamp=time.monotonic(), store_latest=False)
                    pose = detector.world_pose(result)  # Native conversion: object -> base, meters.
                    error = float(result["reproj_error"])
                    record = {**timing, "sequence": sequence, "valid": pose is not None,
                              "T_base_object": pose.tolist() if pose is not None else None,
                              "tracking_error": None if pose is not None else "AprilCube returned no pose",
                              "tag_ids": result["tag_ids"], "n_tags": result["n_tags"],
                              "n_inliers": result["n_inliers"], "predicted": result["predicted"],
                              "aprilcube_detections": result["detections"],
                              "aprilcube_visible_faces": sorted(result["visible_faces"]),
                              "reprojection_rms_px": error if np.isfinite(error) else None,
                              "aprilcube_T_camera_object_mm": result["T"]}
                    record["processed_timestamp_s"] = time.time()
                    with self.lock:
                        self.latest = {"image": image, "record": record}
                        if self.writer is not None:
                            self.writer.record_frame(image, record)
                    sequence += 1
        except BaseException as exc:
            with self.lock:
                self.error = exc
            self.ready.set()

    def check(self):
        with self.lock:
            error = self.error
        if error is not None:
            raise RuntimeError(f"Camera tracking failed: {error}") from error

    def snapshot(self):
        self.check()
        with self.lock:
            return self.latest

    def attach(self, writer):
        # Detaching under the same lock waits for any in-progress enqueue.
        with self.lock:
            self.writer = writer
            if writer is not None and self.latest is not None:
                writer.record_frame(self.latest["image"], self.latest["record"])

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=3)
        if self.thread.is_alive():
            raise RuntimeError("Camera reader did not shut down")

    def __exit__(self, *_):
        self.close()


def native_record(snapshot):
    """Use AprilCube's validity decision without an additional pose-age cutoff."""
    if snapshot is None:
        return None
    record = snapshot["record"]
    return record if record["valid"] else None


def capture_target(tracker, timeout=12.0):
    """Take the next valid native AprilCube pose after placement is confirmed."""
    previous = tracker.snapshot()
    sequence = previous["record"]["sequence"] if previous is not None else None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snapshot = tracker.snapshot()
        if snapshot is not None:
            record = snapshot["record"]
            if record["sequence"] != sequence and record["valid"]:
                return np.asarray(record["T_base_object"]), snapshot
        time.sleep(0.01)
    raise ValueError(f"No new valid AprilCube pose within {timeout:g} seconds after confirmation")


class MouseReader:
    """Drain queued SpaceMouse reports before publishing each input snapshot."""

    def __init__(self, factory=None):
        self.factory = factory
        self.lock = threading.Lock()
        self.stop_event, self.ready = threading.Event(), threading.Event()
        self.latest, self.error = None, None

    def __enter__(self):
        self.thread = threading.Thread(target=self._run, name="SpaceMouse reader", daemon=True)
        self.thread.start()
        if not self.ready.wait(5):
            self.close()
            raise RuntimeError("SpaceMouse did not open within five seconds")
        self.read()
        return self

    def _run(self):
        try:
            # Load this pure helper without importing aiofranka.__init__, which
            # imports robot backends. Offline tracking tests need no robot SDK.
            spec = importlib.util.spec_from_file_location(
                "_t_pushing_spacemouse", Path(__file__).resolve().parents[1]
                / "aiofranka/utils/spacemouse.py")
            utility = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(utility)
            with (self.factory or utility.open_spacemouse)() as device:
                last_t, report_received = None, float('-inf')
                self.ready.set()
                while not self.stop_event.is_set():
                    state = utility.read_latest_state(device)
                    now = time.monotonic()
                    if state.t != last_t and state.t >= 0:
                        last_t, report_received = state.t, now
                    axes = np.array([state.x, state.y, state.z], dtype=float)
                    if not np.isfinite(axes).all():
                        raise ValueError("SpaceMouse sent nonfinite axes")
                    with self.lock:
                        self.latest = (axes, list(state.buttons), report_received, now)
                    self.stop_event.wait(0.001)
        except BaseException as exc:
            with self.lock:
                self.error = exc
            self.ready.set()

    def read(self):
        with self.lock:
            latest, error = self.latest, self.error
        if error is not None:
            raise RuntimeError(f"SpaceMouse failed: {error}") from error
        if latest is None:
            return np.zeros(3), [], None
        axes, buttons, report_received, polled = latest
        if time.monotonic() - polled > 0.1:
            raise RuntimeError("SpaceMouse reader stopped responding")
        age = time.monotonic() - report_received
        # A disconnected device may return its last nonzero state indefinitely.
        return (axes.copy() if age <= 0.2 else np.zeros(3)), buttons, age if np.isfinite(age) else None

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError("SpaceMouse reader did not shut down")

    def __exit__(self, *_):
        self.close()
