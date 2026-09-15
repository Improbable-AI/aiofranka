"""Capture timing and motion gates, without opening camera or robot hardware."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

try:
    import cv2
except ImportError as exc:
    raise unittest.SkipTest("Camera calibration tests require optional OpenCV dependency") from exc


SCRIPT = Path(__file__).parents[1] / "examples" / "12_camera_calibration.py"
SPEC = importlib.util.spec_from_file_location("camera_calibration_capture", SCRIPT)
calibration = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(calibration)


class Clock:
    def __init__(self):
        self.now = 100.0

    def time(self):
        return self.now


class Camera:
    def __init__(self, clock, frame_delay=0.05):
        self.clock = clock
        self.frame_delay = frame_delay
        self.frame_number = 0

    def read(self):
        self.clock.now += self.frame_delay
        self.frame_number += 1
        return np.full((4, 4, 3), self.frame_number, dtype=np.uint8), {
            "frame_number": self.frame_number,
            "camera_timestamp_s": self.clock.now - 0.02,
            "received_timestamp_s": self.clock.now,
        }


class Robot:
    def __init__(self, clock, joint_speed=0.0, state_age=0.0):
        self.clock = clock
        self.joint_speed = joint_speed
        self.state_age = state_age
        self.timestamps = []

    @property
    def state(self):
        timestamp = self.clock.now - self.state_age
        self.timestamps.append(timestamp)
        joints = np.zeros(7)
        joints[0] = (self.clock.now - 100.0) * self.joint_speed
        speeds = np.zeros(7)
        speeds[0] = self.joint_speed
        return {"timestamp": timestamp, "qpos": joints, "qvel": speeds, "ee": np.eye(4)}


class StationaryCaptureTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.clock_patch = patch.object(
            calibration, "time",
            SimpleNamespace(time=self.clock.time, monotonic=self.clock.time),
        )
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)

    def test_stationary_sample_has_measured_history_on_both_sides_of_source_timestamp(self):
        camera, robot = Camera(self.clock), Robot(self.clock)
        image, view = calibration.capture_settled(camera, robot)

        timestamp = view["camera_timestamp_s"]
        self.assertGreaterEqual(timestamp - robot.timestamps[0], 0.4)
        self.assertGreaterEqual(robot.timestamps[-1] - timestamp, 0.4)
        self.assertLess(view["frame_number"], camera.frame_number)
        self.assertTrue(np.all(image == view["frame_number"]))
        np.testing.assert_array_equal(view["T_base_ee"], np.eye(4))
        self.assertEqual(view["max_joint_motion_rad"], 0.0)

    def test_fresh_image_after_camera_stall_does_not_establish_stationarity(self):
        # Fresh frame after a 0.9 s stall used to pass with only two states.
        camera, robot = Camera(self.clock, frame_delay=0.9), Robot(self.clock)
        with self.assertRaisesRegex(ValueError, "Gap in robot measurements"):
            calibration.capture_settled(camera, robot)

    def test_slow_joint_drift_rejected_even_below_speed_limit_and_with_fixed_ee(self):
        camera, robot = Camera(self.clock), Robot(self.clock, joint_speed=0.003)
        with self.assertRaisesRegex(ValueError, "Arm moved during capture"):
            calibration.capture_settled(camera, robot)

    def test_excess_joint_speed_rejected(self):
        camera, robot = Camera(self.clock), Robot(self.clock, joint_speed=0.006)
        with self.assertRaisesRegex(ValueError, "Arm is moving"):
            calibration.capture_settled(camera, robot)

    def test_stale_robot_state_aborts_before_pairing_image(self):
        camera, robot = Camera(self.clock), Robot(self.clock, state_age=0.25)
        with self.assertRaisesRegex(RuntimeError, "Robot state is stale"):
            calibration.capture_settled(camera, robot)
        self.assertEqual(camera.frame_number, 0)



if __name__ == "__main__":
    unittest.main()
