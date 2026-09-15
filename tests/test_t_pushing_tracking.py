"""Freshness and loss handling with fake camera/HID inputs; opens no devices."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np


script = Path(__file__).resolve().parents[1] / "examples/t_pushing_tracking.py"
spec = importlib.util.spec_from_file_location("t_pushing_tracking_test", script)
tracking = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracking)


def snapshot(sequence=0, timestamp=100.0, valid=True):
    pose = np.eye(4)
    pose[0, 3] = sequence * .0001
    return {"image": np.zeros((2, 3, 3), np.uint8), "record": {
        "sequence": sequence, "camera_timestamp_s": timestamp, "valid": valid,
        "T_base_object": pose.tolist() if valid else None,
        "tracking_error": None if valid else "target occluded",
    }}


class FakeClock:
    def __init__(self):
        self.elapsed = 0.0

    def time(self):
        return 100.0 + self.elapsed

    def monotonic(self):
        return self.elapsed

    def sleep(self, seconds):
        self.elapsed += seconds


class FreshTrackingTest(unittest.TestCase):
    def test_native_aprilcube_single_tag_and_world_pose_units(self):
        try:
            import cv2
            import aprilcube
        except ImportError:
            self.skipTest("Native detector test requires AprilCube and OpenCV")
        metadata = {"camera_matrix": [[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]],
                    "dist_coeffs": [0.] * 5}
        base = np.eye(4)
        base[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        base[:3, 3] = [.1, .2, .3]
        detector = tracking.create_detector(
            script.parents[1] / "assets/t_shape_target_xl/config.json", metadata, base)
        self.assertIsInstance(detector, aprilcube.CubePoseEstimator)
        self.assertIsNotNone(detector.pose_filter)
        image = np.full((480, 640), 255, np.uint8)
        image[160:320, 240:400] = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 13, 160)
        result = detector.process_frame(image, timestamp=1., store_latest=False)
        self.assertTrue(result["success"])
        self.assertFalse(result["predicted"])
        self.assertEqual(result["n_tags"], 1)
        self.assertEqual(result["tag_ids"], [13])
        camera_pose_m = result["T"].copy()
        camera_pose_m[:3, 3] /= 1000.
        np.testing.assert_allclose(detector.world_pose(result), base @ camera_pose_m, atol=1e-12)

    def test_opencv_marker_adapter_only_normalizes_array_shapes(self):
        corners = (np.arange(8).reshape(1, 4, 2),)
        rejected = (np.zeros((1, 4, 2)),)
        native = mock.Mock()
        for ids in (np.array([13]), np.array([[13]])):
            native.detectMarkers.return_value = corners, ids, rejected
            actual_corners, actual_ids, actual_rejected = tracking._MarkerArrayShapes(native).detectMarkers(None)
            np.testing.assert_array_equal(actual_ids, [[13]])
            self.assertIs(actual_corners[0], corners[0])
            self.assertIs(actual_rejected[0], rejected[0])

    def test_native_record_uses_library_validity_without_custom_age_gate(self):
        self.assertIsNone(tracking.native_record(None))
        for timestamp in (99., 99.9, 100.):
            reading = snapshot(timestamp=timestamp)
            self.assertIs(tracking.native_record(reading), reading["record"])
        self.assertIsNone(tracking.native_record(snapshot(valid=False)))

    def test_capture_uses_next_native_pose_without_stationarity_or_latency_gate(self):
        old = snapshot(1)
        new = snapshot(2, timestamp=99.5)
        # Greater than the removed 3 mm / 2 degree thresholds. Even a native
        # prediction with no tags is accepted when AprilCube supplies a pose.
        new["record"]["T_base_object"][0][3] = .02
        new["record"]["predicted"] = True
        new["record"]["n_tags"] = 0
        new["record"]["T_base_object"] = (
            np.array([[1, 0, 0, .02], [0, 0, -1, .1],
                      [0, 1, 0, .432], [0, 0, 0, 1]]).tolist())
        tracker = mock.Mock()
        tracker.snapshot.side_effect = [old, old, snapshot(2, valid=False), new]
        with mock.patch.object(tracking, "time", FakeClock()):
            pose, frame = tracking.capture_target(tracker)
        self.assertIs(frame, new)
        np.testing.assert_array_equal(pose, new["record"]["T_base_object"])

    def test_capture_waits_for_initial_native_detection(self):
        detected = snapshot(1)
        tracker = mock.Mock()
        tracker.snapshot.side_effect = [None, None, snapshot(0, valid=False), detected]
        with mock.patch.object(tracking, "time", FakeClock()):
            _, frame = tracking.capture_target(tracker)
        self.assertIs(frame, detected)

    def test_capture_cannot_use_a_frozen_or_missing_native_pose(self):
        for reading in (None, snapshot(7), snapshot(7, valid=False)):
            tracker = mock.Mock()
            tracker.snapshot.return_value = reading
            with self.subTest(reading=reading):
                with mock.patch.object(tracking, "time", FakeClock()):
                    with self.assertRaisesRegex(ValueError, "No new valid AprilCube pose"):
                        tracking.capture_target(tracker, timeout=.1)

    def test_camera_uses_native_pose_and_records_single_tag_prediction_and_loss(self):
        camera, detector, writer = mock.MagicMock(), mock.Mock(), mock.Mock()
        camera.__enter__.return_value = camera
        camera.metadata = {"width": 3, "height": 2}
        tracker = tracking.CameraTracker(lambda: camera, lambda _: detector)
        tracker.attach(writer)
        pose = np.eye(4)
        pose[:3, 3] = [.5, .1, .2]
        predicted = pose.copy()
        predicted[0, 3] += .01
        outputs = [dict(success=success, T=np.eye(4) if success else None,
                        predicted=prediction, n_tags=tags, n_inliers=4 if tags else 0,
                        tag_ids=[17] if tags else [], reproj_error=1. if tags else float("inf"),
                        detections=[(17, np.arange(8).reshape(4, 2))] if tags else [],
                        visible_faces={"-Y"} if tags else set())
                   for success, prediction, tags in ((True, False, 1), (True, True, 0), (False, False, 0))]
        detector.process_frame.side_effect = outputs
        detector.world_pose.side_effect = [pose, predicted, None]
        count = 0

        def read():
            nonlocal count
            count += 1
            if count == 3:
                tracker.stop_event.set()
            return np.full((2, 3, 3), count, np.uint8), {"camera_timestamp_s": 100 + count * .03}

        camera.read.side_effect = read
        tracker._run()
        tracker.check()
        records = [call.args[1] for call in writer.record_frame.call_args_list]
        self.assertEqual([r["valid"] for r in records], [True, True, False])
        self.assertEqual([r["predicted"] for r in records], [False, True, False])
        self.assertEqual([r["n_tags"] for r in records], [1, 0, 0])
        self.assertIs(records[0]["aprilcube_detections"], outputs[0]["detections"])
        self.assertEqual(records[0]["aprilcube_visible_faces"], ["-Y"])
        self.assertEqual(records[1]["aprilcube_detections"], [])
        np.testing.assert_array_equal(records[0]["T_base_object"], pose)
        np.testing.assert_array_equal(records[1]["T_base_object"], predicted)
        self.assertIsNone(records[2]["T_base_object"])
        self.assertIsNone(records[1]["reprojection_rms_px"])
        self.assertIsNone(tracking.native_record({"record": records[2]}))
        self.assertEqual(detector.world_pose.call_args_list, [mock.call(result) for result in outputs])
        for call in detector.process_frame.call_args_list:
            self.assertFalse(call.kwargs["store_latest"])
            self.assertIn("timestamp", call.kwargs)
        camera.__exit__.assert_called_once()

    def test_camera_backend_failure_is_fatal_instead_of_reusing_last_frame(self):
        camera = mock.MagicMock()
        camera.__enter__.return_value = camera
        camera.read.side_effect = RuntimeError("camera disconnected")
        tracker = tracking.CameraTracker(lambda: camera, lambda _: mock.Mock())
        tracker.latest = snapshot()
        tracker._run()
        with self.assertRaisesRegex(RuntimeError, "camera disconnected"):
            tracker.snapshot()
        camera.__exit__.assert_called_once()


class MouseFreshnessTest(unittest.TestCase):
    def run_reader(self, reports, poll_times):
        device = mock.MagicMock()
        device.__enter__.return_value = device
        # Each polling batch ends with a repeated timestamp: public read() found
        # no further HID report. A batch can contain several queued reports.
        batches = [report if isinstance(report, list) else [report] for report in reports]
        device.read.side_effect = [state for batch in batches for state in [*batch, batch[-1]]]
        reader = tracking.MouseReader(lambda: device)
        count = 0

        def waited(_):
            nonlocal count
            count += 1
            if count == len(reports):
                reader.stop_event.set()

        with mock.patch.object(reader.stop_event, "wait", side_effect=waited), \
                mock.patch.object(tracking.time, "monotonic", side_effect=poll_times):
            reader._run()
        device.__exit__.assert_called_once()
        return reader

    @staticmethod
    def report(stamp, x=.7, y=-.2, z=.1, buttons=(False, True)):
        return SimpleNamespace(t=stamp, x=x, y=y, z=z, roll=.2, pitch=-.3, yaw=.4,
                               buttons=list(buttons))

    def test_queued_motion_and_final_release_are_drained_before_publish(self):
        reports = [self.report(1), self.report(2, -.5, buttons=(True, True)),
                   self.report(3, 0, 0, 0, buttons=(False, False))]
        reader = self.run_reader([reports], [10.])
        with mock.patch.object(tracking.time, "monotonic", return_value=10.01):
            axes, buttons, age = reader.read()
        np.testing.assert_array_equal(axes, np.zeros(3))
        self.assertEqual(buttons, [False, False])
        self.assertAlmostEqual(age, .01)

    def test_continuous_queue_is_fatal_and_never_publishes_partial_motion(self):
        device = mock.MagicMock()
        device.__enter__.return_value = device
        state = self.report(0)

        def read():
            state.t += 1  # Same mutable state object, as in PySpaceMouse 2.x.
            return state

        device.read.side_effect = read
        reader = tracking.MouseReader(lambda: device)
        reader._run()
        with self.assertRaisesRegex(RuntimeError, "queue did not drain"):
            reader.read()
        self.assertIsNone(reader.latest)
        self.assertEqual(device.read.call_count, 256)
        device.__exit__.assert_called_once()

    def test_repeated_hid_report_zeros_axes_even_while_polling_continues(self):
        reader = self.run_reader([self.report(1)] * 3, [10., 10.11, 10.22])
        with mock.patch.object(tracking.time, "monotonic", return_value=10.23):
            axes, _, age = reader.read()
        np.testing.assert_array_equal(axes, np.zeros(3))
        self.assertAlmostEqual(age, .23)

    def test_fresh_report_restores_axes_and_returns_a_copy(self):
        reader = self.run_reader([self.report(1), self.report(1), self.report(2, -.5)],
                                 [10., 10.21, 10.22])
        with mock.patch.object(tracking.time, "monotonic", return_value=10.23):
            axes, buttons, age = reader.read()
            np.testing.assert_allclose(axes, [-.5, -.2, .1])
            self.assertEqual(buttons, [False, True])
            self.assertAlmostEqual(age, .01)
            axes[:] = 0
            np.testing.assert_allclose(reader.read()[0], [-.5, -.2, .1])

    def test_reader_stall_and_nonfinite_report_raise(self):
        reader = self.run_reader([self.report(1)], [10.])
        with mock.patch.object(tracking.time, "monotonic", return_value=10.11):
            with self.assertRaisesRegex(RuntimeError, "stopped responding"):
                reader.read()
        reader = self.run_reader([self.report(2, float("nan"))], [10.])
        with self.assertRaisesRegex(RuntimeError, "nonfinite axes"):
            reader.read()

    def test_no_hid_report_yet_produces_zero_command(self):
        reader = self.run_reader([self.report(-1)], [10.])
        with mock.patch.object(tracking.time, "monotonic", return_value=10.):
            axes, _, age = reader.read()
        np.testing.assert_array_equal(axes, np.zeros(3))
        self.assertIsNone(age)


if __name__ == "__main__":
    unittest.main()
