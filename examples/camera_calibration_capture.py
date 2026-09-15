"""Read native RealSense color frames from an already-running local PyCAAS daemon.

PyCAAS timestamps driver-read completion, not sensor exposure. Settled robot
captures reduce timing error but cannot measure hidden camera/driver latency.
This client never starts, stops, or reconfigures the shared camera service.
"""

import json
import time

import numpy as np
import zmq


class _FrameMetadata:
    """Retain PyCAAS's existing wire timestamp, discarded by public get_frame().

    Uses the upstream get_frame protocol and its existing REQ socket. No resize
    parameter is sent, regardless of the client's optional resolution setting.
    """

    def get_frame_with_metadata(self, stream):
        self._socket.send_json({"cmd": "get_frame", "camera_id": stream})
        parts = self._socket.recv_multipart()
        metadata = json.loads(parts[0])
        if "error" in metadata:
            raise RuntimeError(f"PyCAAS frame error: {metadata['error']}")
        if len(parts) != 2 or metadata.get("camera_id") != stream:
            raise RuntimeError("Unexpected PyCAAS frame response")
        try:
            shape, dtype = tuple(metadata["shape"]), np.dtype(metadata["dtype"])
            if len(shape) != 3 or shape[2] != 3 or dtype != np.uint8:
                raise ValueError("Expected native uint8 three-channel color")
            pixels = np.frombuffer(parts[1], dtype=dtype).reshape(shape).copy()
            metadata["timestamp"] = float(metadata["timestamp"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"Invalid PyCAAS color frame: {exc}") from exc
        return pixels, metadata


def _client(args):
    endpoint = getattr(args, "pycaas_endpoint", "ipc:///tmp/pycaas.sock")
    if not endpoint.startswith("ipc:///"):
        raise ValueError("Calibration requires local PyCAAS IPC so robot/camera receipt clocks match")
    try:
        from pycaas import PycaasClient
    except ImportError as exc:
        raise RuntimeError("Install PyCAAS in this Python environment (python -m pip install -e /path/to/pycaas)") from exc

    class TimestampedPycaasClient(_FrameMetadata, PycaasClient):
        pass

    return _request(TimestampedPycaasClient, endpoint=endpoint, timeout_ms=1000)


def _request(method, *args, **kwargs):
    try:
        return method(*args, **kwargs)
    except zmq.ZMQError as exc:
        raise RuntimeError("Cannot communicate with PyCAAS. Check the endpoint and run 'pycaas start'.") from exc


def _active_color_streams(client):
    cameras, status = _request(client.list_cameras), _request(client.status)
    if not status.get("running"):
        raise RuntimeError("PyCAAS is not running; start the daemon with 'pycaas start'")
    active = set(status.get("active_streams", []))
    choices = [(camera, camera["id"] + "_color") for camera in cameras
               if camera.get("type") == "realsense" and camera["id"] + "_color" in active]
    return choices, status


def list_cameras(args):
    """List active RealSense color streams without touching capture settings."""
    client = _client(args)
    try:
        choices, status = _active_color_streams(client)
        if not choices:
            raise RuntimeError("PyCAAS has no active RealSense color streams")
        for camera, stream in choices:
            size = status.get("streams", {}).get(stream, {}).get("resolution")
            print(f"{stream}  {camera.get('name', 'RealSense')}  native resolution: {size}")
    finally:
        client.close()


def _intrinsics(client, camera_id):
    intr = _request(client.get_intrinsics, camera_id)
    try:
        width, height = int(intr["width"]), int(intr["height"])
        K = np.array([[intr["fx"], 0, intr["cx"]], [0, intr["fy"], intr["cy"]], [0, 0, 1]], dtype=float)
        distortion = np.asarray(intr["dist_coeffs"], dtype=float)
        if (min(width, height) <= 0 or not np.isfinite(K).all() or min(K[0, 0], K[1, 1]) <= 0
                or distortion.shape != (5,) or not np.isfinite(distortion).all()):
            raise ValueError("Invalid dimensions, intrinsics, or distortion coefficients")
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"PyCAAS must provide native color intrinsics: {exc}") from exc
    model = intr.get("distortion_model")
    normalized = "".join(c for c in str(model).lower() if c.isalnum())
    if np.any(distortion != 0) and normalized not in ("brownconrady", "distortionbrownconrady"):
        raise ValueError("PyCAAS has nonzero distortion without a verified forward Brown-Conrady model")
    signature = (width, height, tuple(K.ravel()), tuple(distortion), model)
    return intr, K, distortion, signature


class PycaasCamera:
    """Context/read API yielding owned BGR pixels and honest daemon timestamps."""

    def __init__(self, args):
        self.args, self.client = args, None
        self._last_timestamp = None
        self._frame_count = 0

    def __enter__(self):
        self.client = _client(self.args)
        try:
            choices, status = _active_color_streams(self.client)
            stream = getattr(self.args, "stream", None)
            serial = getattr(self.args, "serial", None)
            if serial:
                selected = f"rs_{serial}_color"
                if stream and stream != selected:
                    raise ValueError("--stream and --serial select different cameras")
                stream = selected
            candidates = [(camera, candidate) for camera, candidate in choices if not stream or candidate == stream]
            if len(candidates) != 1:
                raise ValueError(f"Select one active RealSense color --stream; available: {[s for _, s in choices]}")
            info, self.stream = candidates[0]
            self.camera_id = info["id"]
            intr, K, distortion, self._profile = _intrinsics(self.client, self.camera_id)
            width, height = self._profile[:2]
            stream_info = status.get("streams", {}).get(self.stream, {})
            if stream_info.get("resolution") != [width, height]:
                raise ValueError("PyCAAS native frame dimensions disagree with its camera intrinsics")
            fps = stream_info.get("fps", info.get("fps", intr.get("fps")))
            for name, actual in (("width", width), ("height", height), ("fps", fps)):
                expected = getattr(self.args, name, None)
                if expected is not None and expected != actual:
                    detail = "this PyCAAS version does not report FPS; omit --fps" if actual is None else f"daemon uses {actual}"
                    raise ValueError(f"Cannot satisfy --{name} {expected}: {detail}. Camera settings are unchanged.")
            self.metadata = {
                "backend": "pycaas", "endpoint": getattr(self.args, "pycaas_endpoint", "ipc:///tmp/pycaas.sock"),
                "serial": info.get("serial", ""), "name": info.get("name", "RealSense"),
                "stream": self.stream, "camera_id": self.camera_id,
                "width": width, "height": height, "fps": fps, "format": "bgr8",
                "source_format": "rgb8 (PyCAAS RealSense driver)", "camera_matrix": K.tolist(),
                "dist_coeffs": distortion.tolist(),
                "distortion_model": intr.get("distortion_model") or "none (zero coefficients; source model unavailable)",
                "intrinsics_source": "PyCAAS active native RealSense color stream",
                "timestamp_meaning": "Same-host PyCAAS driver-read completion; not hardware exposure time",
                "timing_limit": "Camera/driver latency before daemon receipt is unknown; capture only while stationary",
            }
            self.read()  # Validate a native frame before any robot is connected.
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def read(self):
        if self.client is None:
            raise RuntimeError("Connect PyCAAS before reading frames")
        deadline = time.monotonic() + 0.3
        while True:
            pixels, header = _request(self.client.get_frame_with_metadata, self.stream)
            timestamp, received = header["timestamp"], time.time()
            if not np.isfinite(timestamp) or not -0.05 <= received - timestamp <= 0.25:
                raise RuntimeError("PyCAAS frame is stale or its server clock is invalid")
            if self._last_timestamp is not None and timestamp <= self._last_timestamp:
                if timestamp < self._last_timestamp or time.monotonic() >= deadline:
                    raise RuntimeError("PyCAAS stopped delivering new frames or its timestamp moved backwards")
                time.sleep(0.005)
                continue
            if pixels.dtype != np.uint8 or pixels.shape != (self.metadata["height"], self.metadata["width"], 3):
                raise RuntimeError("PyCAAS native color dimensions/format changed; restart calibration")
            # Factory intrinsics are cheap to read and catch daemon reconfiguration.
            if _intrinsics(self.client, self.camera_id)[3] != self._profile:
                raise RuntimeError("PyCAAS camera profile/intrinsics changed; restart calibration")
            if time.time() - timestamp > 0.25:
                raise RuntimeError("PyCAAS frame became stale while checking its camera profile")
            self._last_timestamp = timestamp
            self._frame_count += 1
            return pixels[:, :, ::-1].copy(), {
                "frame_number": self._frame_count, "frame_number_meaning": "Client count of unique daemon frames",
                "camera_timestamp_s": timestamp, "pycaas_server_timestamp_s": timestamp,
                "received_timestamp_s": received, "timestamp_domain": "pycaas_same_host_wall_clock",
                "timestamp_meaning": "Driver-read completion, not sensor exposure", "stream": self.stream,
            }

    def __exit__(self, *_):
        if self.client is not None:
            self.client.close()
            self.client = None
