"""Synthetic camera calibration checks without a robot, RealSense, or AprilCube."""

import copy
import importlib.util
import json
from pathlib import Path
import unittest
from unittest import mock

import numpy as np
from scipy.spatial.transform import Rotation

try:
    import cv2
except ImportError:
    cv2 = None


if cv2 is not None:
    script = Path(__file__).resolve().parents[1] / "examples" / "12_camera_calibration.py"
    spec = importlib.util.spec_from_file_location("camera_calibration", script)
    calibration = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(calibration)


def make_dataset(noise=0.0, mode="diverse", count=20):
    rng = np.random.default_rng(100)
    K = np.array([[620., 0, 320], [0, 615., 240], [0, 0, 1]])
    D = np.array([0.015, -0.02, 0.001, -0.0003, 0.01])
    X = calibration._calib_transform(
        Rotation.from_euler("xyz", [20, -30, 45], degrees=True).as_matrix(),
        [0.4, -0.3, 0.8],
    )
    Y = calibration._calib_transform(
        Rotation.from_euler("xyz", [15, -10, 30], degrees=True).as_matrix(),
        [0.025, -0.01, 0.12],
    )
    points = np.array([[x, y, z] for x in [-0.035, 0.035]
                       for y in [-0.035, 0.035] for z in [-0.035, 0.035]])
    views = []
    for _ in range(count):
        angles = rng.uniform(-40, 40, 3)
        if mode == "translation":
            angles[:] = 0
        elif mode == "single_axis":
            angles[:2] = 0
        C = calibration._calib_transform(
            Rotation.from_euler("xyz", angles, degrees=True).as_matrix(),
            rng.uniform([-.15, -.1, .5], [.15, .1, .9]),
        )
        G = X @ C @ np.linalg.inv(Y)
        pixels, _ = cv2.projectPoints(
            points, Rotation.from_matrix(C[:3, :3]).as_rotvec(), C[:3, 3], K, D,
        )
        pixels = pixels.reshape(-1, 2) + rng.normal(0, noise, (len(points), 2))
        success, rvec, tvec = cv2.solvePnP(points, pixels, K, D)
        if not success:
            raise AssertionError("Synthetic PnP initialization failed")
        estimate = calibration._calib_transform(
            Rotation.from_rotvec(rvec.ravel()).as_matrix(), tvec.ravel(),
        )
        views.append({
            "T_base_ee": G.tolist(), "T_camera_cube": estimate.tolist(),
            "object_points_m": points.tolist(), "image_points_px": pixels.tolist(),
            "tag_ids": [0, 1],
        })
    return {
        "camera": {"camera_matrix": K.tolist(), "dist_coeffs": D.tolist(),
                   "width": 640, "height": 480},
        "views": views,
    }, X, Y


@unittest.skipIf(cv2 is None, "Camera calibration requires optional OpenCV")
class CameraCalibrationTest(unittest.TestCase):
    def assert_pose_close(self, actual, expected, meters, degrees):
        difference = np.linalg.inv(expected) @ np.asarray(actual)
        self.assertLess(np.linalg.norm(difference[:3, 3]), meters)
        angle = Rotation.from_matrix(difference[:3, :3]).magnitude()
        self.assertLess(np.rad2deg(angle), degrees)

    def test_exact_recovery_and_transform_direction(self):
        data, X, Y = make_dataset()
        result = calibration.fit_calibration(data)
        self.assert_pose_close(result["T_base_camera"], X, 1e-7, 1e-5)
        self.assert_pose_close(result["T_ee_cube"], Y, 1e-7, 1e-5)
        np.testing.assert_allclose(
            np.asarray(result["T_base_camera"]) @ result["T_camera_base"],
            np.eye(4), atol=1e-10,
        )
        self.assertLess(result["metrics"]["all_views"]["rms_px"], 1e-6)
        self.assertLess(result["metrics"]["held_out"]["rms_px"], 1e-6)
        json.dumps(result, allow_nan=False)

    def test_noisy_recovery_with_fixed_intrinsics(self):
        data, X, Y = make_dataset(noise=0.35, count=25)
        result = calibration.fit_calibration(data)
        self.assert_pose_close(result["T_base_camera"], X, 0.004, 0.4)
        self.assert_pose_close(result["T_ee_cube"], Y, 0.003, 0.4)
        self.assertLess(result["metrics"]["held_out"]["rms_px"], 0.8)
        self.assertFalse(result["metrics"]["intrinsics_refined"])
        np.testing.assert_array_equal(result["camera_matrix"], data["camera"]["camera_matrix"])
        np.testing.assert_array_equal(result["dist_coeffs"], data["camera"]["dist_coeffs"])

    def test_park_fallback_without_opencv_handeye_recovers_exact_and_noisy_data(self):
        for noise, tolerance in ((0.0, 1e-7), (0.35, 0.004)):
            with self.subTest(noise=noise):
                data, X, Y = make_dataset(noise=noise, count=25)
                with mock.patch.object(cv2, "calibrateHandEye", None, create=True):
                    result = calibration.fit_calibration(data)
                self.assert_pose_close(result["T_base_camera"], X, tolerance, 0.4 if noise else 1e-5)
                self.assert_pose_close(result["T_ee_cube"], Y, tolerance, 0.4 if noise else 1e-5)
                self.assertLess(result["metrics"]["held_out"]["rms_px"], 0.8 if noise else 1e-6)

    def test_park_fallback_initializer_matches_opencv_when_available(self):
        if not callable(getattr(cv2, "calibrateHandEye", None)):
            self.skipTest("This OpenCV wheel omits the reference hand-eye implementation")
        for noise in (0.0, 0.35):
            with self.subTest(noise=noise):
                _, _, views = calibration._calib_prepare(make_dataset(noise=noise)[0])
                expected = calibration._calib_unpack(calibration._calib_initialize(views))
                with mock.patch.object(cv2, "calibrateHandEye", None):
                    actual = calibration._calib_unpack(calibration._calib_initialize(views))
                for actual_pose, expected_pose in zip(actual, expected):
                    self.assert_pose_close(actual_pose, expected_pose, 1e-8, 1e-6)

    def test_sparse_corner_outliers(self):
        data, X, Y = make_dataset(noise=0.2, count=25)
        for i in (0, 5, 10, 15):
            data["views"][i]["image_points_px"][0][0] += 50
        result = calibration.fit_calibration(data)
        self.assert_pose_close(result["T_base_camera"], X, 0.004, 0.5)
        self.assert_pose_close(result["T_ee_cube"], Y, 0.003, 0.5)

    def test_translation_and_single_axis_motion_rejected(self):
        for mode in ("translation", "single_axis"):
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, "rotation diversity"):
                calibration.fit_calibration(make_dataset(mode=mode)[0])

    def test_insufficient_or_duplicate_captures_rejected(self):
        with self.assertRaisesRegex(ValueError, "12 distinct"):
            calibration.fit_calibration(make_dataset(count=11)[0])
        data = make_dataset(count=12)[0]
        data["views"] = [copy.deepcopy(data["views"][0]) for _ in range(15)]
        with self.assertRaisesRegex(ValueError, "12 distinct"):
            calibration.fit_calibration(data)

    def test_held_out_observations_do_not_affect_training_fit(self):
        data, _, _ = make_dataset()
        original = calibration.fit_calibration(data)
        for i, view in enumerate(data["views"]):
            if i % 5 == 4:
                view["image_points_px"] = (np.array(view["image_points_px"]) + [30, -20]).tolist()
                # Held-out PnP estimates must not enter initialization either.
                view["T_camera_cube"][0][3] += 1
        with mock.patch.object(calibration, "_calib_solve", wraps=calibration._calib_solve) as observed:
            result = calibration.fit_calibration(data)
            training_indices = [v["index"] for v in observed.call_args_list[0].args[0]]
            self.assertTrue(all(i % 5 != 4 for i in training_indices))
            self.assertEqual(len(observed.call_args_list[1].args[0]), len(data["views"]))
        self.assertAlmostEqual(result["metrics"]["training"]["rms_px"],
                               original["metrics"]["training"]["rms_px"])
        self.assertAlmostEqual(result["metrics"]["held_out"]["rms_px"],
                               np.hypot(30, 20), places=5)

    def test_malformed_measurements_rejected(self):
        original = make_dataset()[0]
        for frame in ("T_base_ee", "T_camera_cube"):
            for row, column, value in ((0, 0, 2), (3, 0, 1), (0, 3, float("nan"))):
                data = copy.deepcopy(original)
                data["views"][0][frame][row][column] = value
                with self.subTest(frame=frame, row=row, column=column):
                    with self.assertRaisesRegex(ValueError, "rigid 4x4 transform"):
                        calibration.fit_calibration(data)
        data = copy.deepcopy(original)
        data["views"][0]["image_points_px"].pop()
        with self.assertRaisesRegex(ValueError, "matching 3D/2D points"):
            calibration.fit_calibration(data)
        data = copy.deepcopy(original)
        data["camera"]["camera_matrix"][0][0] = -1
        with self.assertRaisesRegex(ValueError, "camera matrix"):
            calibration.fit_calibration(data)


if __name__ == "__main__":
    unittest.main()
