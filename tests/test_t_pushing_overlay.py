"""Offline replay uses saved AprilCube outputs and never reruns detection."""

import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None


ROOT = Path(__file__).resolve().parents[1]
if cv2 is not None:
    spec = importlib.util.spec_from_file_location("t_pushing_overlay_test", ROOT / "examples/t_pushing_overlay.py")
    overlay_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(overlay_module)


@unittest.skipIf(cv2 is None, "AprilCube overlays require optional OpenCV")
class AprilCubeOverlayTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.T_base_camera = np.array([[0., -1., 0., .2], [1., 0., 0., -.4],
                                       [0., 0., 1., .3], [0., 0., 0., 1.]])
        self.calibration = {"T_base_camera": self.T_base_camera.tolist(),
                            "camera_matrix": [[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]],
                            "dist_coeffs": [0.] * 5}
        (self.directory / "calibration.json").write_text(json.dumps(self.calibration))
        shutil.copyfile(ROOT / "assets/t_shape_target_xl/config.json", self.directory / "target_config.json")
        self.native = np.eye(4)
        self.native[:3, :3] = cv2.Rodrigues(np.array([.2, -.1, .3]))[0]
        self.native[:3, 3] = [50., -80., 800.]
        self.record = {"valid": True, "aprilcube_T_camera_object_mm": self.native.tolist(),
                       "tag_ids": [13], "n_tags": 1, "n_inliers": 4,
                       "reprojection_rms_px": .75, "predicted": False}
        self.image = np.full((480, 640, 3), 127, dtype=np.uint8)

    def fake_overlay(self):
        estimator = mock.Mock()
        estimator.face_id_sets = {"+X": {13}, "-Y": {17}}
        estimator.draw_result.side_effect = lambda image, result: image.copy()
        for name in ("process_frame", "detect_markers", "process_marker_detections", "world_pose"):
            getattr(estimator, name).side_effect = AssertionError("Detection/pose processing is forbidden")
        factory = mock.Mock(return_value=estimator)
        with mock.patch.dict("sys.modules", {"aprilcube": SimpleNamespace(detector=factory)}):
            overlay = overlay_module.AprilCubeOverlay(self.directory)
        return overlay, estimator, factory

    def test_native_saved_pose_goes_directly_to_native_renderer_with_mm_units(self):
        overlay, estimator, factory = self.fake_overlay()
        before = self.image.copy()
        rendered = overlay(self.image, self.record)
        result = estimator.draw_result.call_args.args[1]
        np.testing.assert_allclose(cv2.Rodrigues(result["rvec"])[0], self.native[:3, :3], atol=1e-12)
        np.testing.assert_array_equal(result["tvec"], [[50.], [-80.], [800.]])
        self.assertEqual(result["visible_faces"], {"+X"})
        self.assertEqual(result["detections"], [])
        self.assertEqual(result["reproj_error"], .75)
        np.testing.assert_array_equal(self.image, before)
        self.assertFalse(np.array_equal(rendered, self.image))
        self.assertEqual(rendered.shape, self.image.shape)
        self.assertFalse(overlay.metadata["detector_rerun"])
        np.testing.assert_array_equal(factory.call_args.args[1], self.calibration["camera_matrix"])
        np.testing.assert_array_equal(factory.call_args.kwargs["extrinsic"], self.T_base_camera)

    def test_saved_base_pose_fallback_preserves_camera_pose_and_mm_units(self):
        overlay, estimator, _ = self.fake_overlay()
        camera_m = self.native.copy()
        camera_m[:3, 3] /= 1000.
        record = {**self.record, "T_base_object": (self.T_base_camera @ camera_m).tolist()}
        del record["aprilcube_T_camera_object_mm"]
        overlay(self.image, record)
        result = estimator.draw_result.call_args.args[1]
        np.testing.assert_allclose(result["T"], self.native, atol=1e-12)

    def test_observed_corners_and_faces_are_preserved_in_native_shapes(self):
        overlay, estimator, _ = self.fake_overlay()
        corners = [[11.1, 12.2], [31.3, 12.4], [31.5, 32.6], [11.7, 32.8]]
        record = {**self.record, "aprilcube_detections": [[13, corners]], "aprilcube_visible_faces": ["-Y"]}
        with mock.patch.object(overlay_module.cv2, "putText", wraps=cv2.putText) as text:
            overlay(self.image, record)
        result = estimator.draw_result.call_args.args[1]
        self.assertEqual(result["visible_faces"], {"-Y"})
        self.assertEqual(result["detections"][0][0], 13)
        np.testing.assert_array_equal(result["detections"][0][1], corners)
        self.assertIn("TRACKED", [call.args[1] for call in text.call_args_list])
        self.assertNotIn("Observed tag corners were not recorded", [call.args[1] for call in text.call_args_list])

    def test_prediction_and_loss_are_labeled_without_carrying_forward_a_pose(self):
        overlay, estimator, _ = self.fake_overlay()
        with mock.patch.object(overlay_module.cv2, "putText", wraps=cv2.putText) as text:
            overlay(self.image, {**self.record, "predicted": True, "n_tags": 0,
                                 "tag_ids": [], "reprojection_rms_px": None})
            prediction = estimator.draw_result.call_args.args[1]
            self.assertTrue(prediction["success"])
            self.assertTrue(prediction["predicted"])
            self.assertTrue(np.isinf(prediction["reproj_error"]))
            # An invalid record must ignore even a stale pose stored on that row.
            overlay(self.image, {**self.record, "valid": False})
            invalid = estimator.draw_result.call_args.args[1]
        self.assertFalse(invalid["success"])
        self.assertIsNone(invalid["rvec"])
        self.assertIsNone(invalid["tvec"])
        self.assertIsNone(invalid["T"])
        labels = [call.args[1] for call in text.call_args_list]
        self.assertIn("PREDICTED", labels)
        self.assertIn("NO POSE", labels)
        self.assertIn("Observed tag corners were not recorded", labels)

    def test_missing_saved_valid_pose_is_reported_without_estimating_one(self):
        overlay, estimator, _ = self.fake_overlay()
        record = {**self.record, "aprilcube_T_camera_object_mm": None}
        with self.assertRaisesRegex(ValueError, "no saved AprilCube pose"):
            overlay(self.image, record)
        estimator.draw_result.assert_not_called()

    def test_installed_native_renderer_works_with_all_detection_entrypoints_forbidden(self):
        try:
            import aprilcube
        except ImportError:
            self.skipTest("Native renderer check requires installed AprilCube")
        overlay = overlay_module.AprilCubeOverlay(self.directory)
        for name in ("process_frame", "detect_markers", "process_marker_detections", "world_pose"):
            setattr(overlay.estimator, name, mock.Mock(side_effect=AssertionError("No detection")))
        overlay.estimator.detector = mock.Mock()
        overlay.estimator.detector.detectMarkers.side_effect = AssertionError("No marker detection")
        overlay.estimator.fallback_detector = overlay.estimator.detector
        with mock.patch.object(overlay.estimator, "draw_result", wraps=overlay.estimator.draw_result) as draw:
            rendered = overlay(self.image, self.record)
        draw.assert_called_once()
        self.assertEqual(rendered.shape, self.image.shape)
        self.assertTrue(np.any(rendered[100:] != self.image[100:]))


if __name__ == "__main__":
    unittest.main()
