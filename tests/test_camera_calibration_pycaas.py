"""Native PyCAAS camera reads using fake clients; no service or hardware needed."""

import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import zmq


SOURCE = Path(__file__).parents[1] / "examples/camera_calibration_capture.py"
SPEC = importlib.util.spec_from_file_location("pycaas_calibration_capture", SOURCE)
capture = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture)


class PycaasCaptureTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        clock = SimpleNamespace(time=lambda: self.now, monotonic=lambda: self.now, sleep=self.advance)
        self.clock_patch = patch.object(capture, "time", clock)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.args = SimpleNamespace(stream=None, serial=None, width=None, height=None, fps=None,
                                    pycaas_endpoint="ipc:///tmp/pycaas.sock")
        self.intrinsics = {"fx": 100., "fy": 101., "cx": 2., "cy": 1.,
                           "width": 4, "height": 3, "dist_coeffs": [0.] * 5}
        self.camera_info = {"id": "rs_123", "serial": "123", "type": "realsense", "name": "D435"}
        self.status = {"running": True, "active_streams": ["rs_123_color", "rs_123_depth"],
                       "streams": {"rs_123_color": {"resolution": [4, 3]}}}
        self.rgb = np.full((3, 4, 3), [11, 22, 33], dtype=np.uint8)
        self.client = Mock()
        self.client.list_cameras.return_value = [self.camera_info]
        self.client.status.side_effect = lambda: copy.deepcopy(self.status)
        self.client.get_intrinsics.side_effect = lambda _: copy.deepcopy(self.intrinsics)
        self.client.get_frame_with_metadata.side_effect = self.frame
        factory = patch.object(capture, "_client", return_value=self.client)
        factory.start()
        self.addCleanup(factory.stop)

    def advance(self, seconds):
        self.now += seconds

    def frame(self, stream):
        self.advance(.04)
        return self.rgb, {"camera_id": stream, "timestamp": self.now - .01}

    def test_native_rgb_converted_and_timestamp_preserved_without_service_mutation(self):
        self.args.serial, self.args.width, self.args.height = "123", 4, 3
        with capture.PycaasCamera(self.args) as camera:
            image, timing = camera.read()
            self.assertEqual(camera.metadata["stream"], "rs_123_color")
            self.assertIsNone(camera.metadata["fps"])
            self.assertIn("not hardware exposure", camera.metadata["timestamp_meaning"])
            self.assertAlmostEqual(timing["camera_timestamp_s"], self.now - .01)
            self.assertEqual(timing["camera_timestamp_s"], timing["pycaas_server_timestamp_s"])
            self.rgb[:] = 0
            np.testing.assert_array_equal(image[0, 0], [33, 22, 11])
        self.client.get_intrinsics.assert_called_with("rs_123")
        self.client.close.assert_called_once()
        self.client.stop.assert_not_called()
        self.client.set_config.assert_not_called()
        self.client.set_resolution.assert_not_called()

    def test_duplicate_frame_is_skipped_until_new_timestamp(self):
        with capture.PycaasCamera(self.args) as camera:
            first = camera._last_timestamp
            calls = [0]

            def next_frame(stream):
                calls[0] += 1
                return (self.rgb, {"timestamp": first}) if calls[0] == 1 else self.frame(stream)

            self.client.get_frame_with_metadata.side_effect = next_frame
            _, timing = camera.read()
            self.assertGreater(timing["camera_timestamp_s"], first)
            self.assertEqual(calls[0], 2)
            self.assertEqual(timing["frame_number"], 2)

    def test_stale_frozen_backward_or_future_frames_are_rejected(self):
        for mode in ("stale", "frozen", "backward", "future"):
            with self.subTest(mode=mode):
                self.client.get_frame_with_metadata.side_effect = self.frame
                with capture.PycaasCamera(self.args) as camera:
                    timestamp = {"stale": self.now - 1, "frozen": camera._last_timestamp,
                                 "backward": camera._last_timestamp - .01, "future": self.now + 1}[mode]
                    self.client.get_frame_with_metadata.side_effect = lambda _: (self.rgb, {"timestamp": timestamp})
                    with self.assertRaises(RuntimeError):
                        camera.read()

    def test_changed_native_frame_shape_or_intrinsics_rejected(self):
        for mode in ("shape", "intrinsics"):
            with self.subTest(mode=mode):
                self.rgb = np.zeros((3, 4, 3), dtype=np.uint8)
                with capture.PycaasCamera(self.args) as camera:
                    if mode == "shape":
                        self.rgb = np.zeros((6, 8, 3), dtype=np.uint8)
                    else:
                        self.intrinsics["fx"] += 1
                    with self.assertRaisesRegex(RuntimeError, "changed"):
                        camera.read()

    def test_ambiguous_stream_requires_selection_and_closes_failed_client(self):
        second = {**self.camera_info, "id": "rs_456", "serial": "456"}
        self.client.list_cameras.return_value.append(second)
        self.status["active_streams"].append("rs_456_color")
        self.status["streams"]["rs_456_color"] = {"resolution": [4, 3]}
        with self.assertRaisesRegex(ValueError, "Select one"):
            with capture.PycaasCamera(self.args):
                pass
        self.client.close.assert_called_once()
        self.args.stream = "rs_456_color"
        with capture.PycaasCamera(self.args) as camera:
            self.assertEqual(camera.camera_id, "rs_456")

    def test_missing_intrinsics_nonzero_unknown_distortion_and_fps_assertion_rejected(self):
        for mode in ("missing", "distortion", "fps"):
            with self.subTest(mode=mode):
                original = copy.deepcopy(self.intrinsics)
                if mode == "missing":
                    self.intrinsics.pop("fx")
                elif mode == "distortion":
                    self.intrinsics["dist_coeffs"][0] = .1
                else:
                    self.args.fps = 30
                with self.assertRaises(ValueError):
                    with capture.PycaasCamera(self.args):
                        pass
                self.intrinsics = original
                self.args.fps = None
        self.assertEqual(self.client.close.call_count, 3)

    def test_timeout_error_is_actionable_and_closes_client(self):
        self.client.status.side_effect = zmq.Again()
        with self.assertRaisesRegex(RuntimeError, "pycaas start"):
            with capture.PycaasCamera(self.args):
                pass
        self.client.close.assert_called_once()

    def test_slow_profile_response_cannot_return_an_old_frame_as_fresh(self):
        with capture.PycaasCamera(self.args) as camera:
            def slow_intrinsics(_):
                self.advance(.3)
                return copy.deepcopy(self.intrinsics)
            self.client.get_intrinsics.side_effect = slow_intrinsics
            with self.assertRaisesRegex(RuntimeError, "became stale"):
                camera.read()


class PycaasWireTest(unittest.TestCase):
    def test_wire_timestamp_and_native_request_are_retained(self):
        client = capture._FrameMetadata()
        client._socket = Mock()
        client._resolution = (1, 1)  # Must not invoke server-side resizing.
        original = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        metadata = {"camera_id": "rs_123_color", "shape": [2, 3, 3], "dtype": "uint8", "timestamp": 123.456}
        client._socket.recv_multipart.return_value = [json.dumps(metadata).encode(), original.tobytes()]
        image, header = client.get_frame_with_metadata("rs_123_color")
        client._socket.send_json.assert_called_once_with({"cmd": "get_frame", "camera_id": "rs_123_color"})
        self.assertEqual(header["timestamp"], 123.456)
        np.testing.assert_array_equal(image, original)
        self.assertTrue(image.flags.owndata)

    def test_nonlocal_endpoint_is_rejected_before_import_or_connection(self):
        with self.assertRaisesRegex(ValueError, "local PyCAAS IPC"):
            capture._client(SimpleNamespace(pycaas_endpoint="tcp://localhost:5555"))


if __name__ == "__main__":
    unittest.main()
