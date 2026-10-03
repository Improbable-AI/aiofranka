import math
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

try:
    import cv2

    from aiofranka import camera
except ImportError:  # the camera extra is not installed
    camera = None

K = np.array([[612.0, 0.0, 320.0], [0.0, 612.0, 240.0], [0.0, 0.0, 1.0]])
D = np.zeros(5)


def transform(rotation, translation):
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = rotation, translation
    return pose


def look_at(position, target):
    """A camera pose (x right, y down, z forward) at position, looking at target."""
    forward = np.subtract(target, position, dtype=float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, (0.0, 0.0, 1.0))
    right /= np.linalg.norm(right)
    return transform(np.column_stack([right, np.cross(forward, right), forward]), position)


def synthetic_session(count, rotate=True, noise_px=0.2, seed=0):
    """Views of the calibration cube, held under the flange, from a known fixed camera."""
    rng = np.random.default_rng(seed)
    cube = camera.Cube()
    X = look_at((0.0, -0.6, 0.5), (0.5, -0.1, 0.1))  # T_base_camera
    Y = transform(Rotation.from_rotvec([math.pi, 0.0, 0.0]).as_matrix(), (0.004, -0.002, 0.105))  # T_ee_cube
    flange_down = Rotation.from_rotvec([math.pi, 0.0, 0.0])
    views = []
    while len(views) < count:
        turn = Rotation.from_rotvec(rng.uniform(-0.5, 0.5, 3)) if rotate else Rotation.identity()
        ee = transform((turn * flange_down).as_matrix(), rng.uniform((0.35, -0.3, 0.25), (0.65, 0.2, 0.45)))
        camera_cube = np.linalg.inv(X) @ ee @ Y
        tags = [tag for tag, corners in cube.corners.items()
                if (camera_cube[:3, :3] @ cube.normals[tag]) @ (camera_cube[:3, :3] @ corners.mean(axis=0)
                                                                + camera_cube[:3, 3]) < -0.01]
        if len(tags) < 2:
            continue
        points = np.concatenate([cube.corners[tag] for tag in tags])
        rvec = cv2.Rodrigues(camera_cube[:3, :3])[0]
        pixels = cv2.projectPoints(points, rvec, camera_cube[:3, 3], K, D)[0].reshape(-1, 2)
        pixels += rng.normal(0.0, noise_px, pixels.shape)
        if pixels.min() < 0 or pixels[:, 0].max() >= 640 or pixels[:, 1].max() >= 480:
            continue
        _, rvec, tvec = cv2.solvePnP(points, pixels, K, D, flags=cv2.SOLVEPNP_SQPNP)
        views.append({"T_base_ee": ee.tolist(),
                      "T_camera_cube": transform(cv2.Rodrigues(rvec)[0], tvec.ravel()).tolist(),
                      "object_points_m": points.tolist(), "image_points_px": pixels.tolist()})
    return {"camera": {"camera_matrix": K.tolist(), "dist_coeffs": D.tolist()}, "views": views}, X, Y


@unittest.skipIf(camera is None, "needs the camera extra: pip install 'aiofranka[camera]'")
class FitTest(unittest.TestCase):
    def assert_pose_close(self, actual, expected, meters, degrees):
        actual = np.asarray(actual)
        self.assertLess(np.linalg.norm(actual[:3, 3] - expected[:3, 3]), meters)
        angle = Rotation.from_matrix(actual[:3, :3].T @ expected[:3, :3]).magnitude()
        self.assertLess(math.degrees(angle), degrees)

    def test_recovers_the_camera_and_the_cube_on_the_flange(self):
        dataset, X, Y = synthetic_session(20)

        result = camera.fit(dataset)

        self.assert_pose_close(result["T_base_camera"], X, 0.002, 0.2)
        self.assert_pose_close(result["T_ee_cube"], Y, 0.002, 0.2)
        np.testing.assert_allclose(np.linalg.inv(result["T_base_camera"]), result["T_camera_base"], atol=1e-12)
        metrics = result["metrics"]
        self.assertEqual((metrics["training"]["view_count"], metrics["held_out"]["view_count"]), (16, 4))
        self.assertEqual([view["view_index"] for view in metrics["held_out"]["per_view"]], [4, 9, 14, 19])
        self.assertLess(metrics["held_out"]["rms_px"], 0.5)

    def test_refuses_views_without_wrist_rotation(self):
        dataset, _, _ = synthetic_session(20, rotate=False)

        with self.assertRaisesRegex(ValueError, "rotation spread"):
            camera.fit(dataset)

    def test_refuses_too_few_views(self):
        dataset, _, _ = synthetic_session(8)

        with self.assertRaisesRegex(ValueError, "8 distinct poses, need 12"):
            camera.fit(dataset)

    def test_a_pose_is_new_far_enough_from_every_view(self):
        here = transform(np.eye(3), (0.5, 0.0, 0.3))
        views = [{"T_base_ee": here.tolist()}]

        self.assertFalse(camera._is_new(here, views))
        self.assertFalse(camera._is_new(transform(np.eye(3), (0.53, 0.0, 0.3)), views))
        self.assertTrue(camera._is_new(transform(np.eye(3), (0.56, 0.0, 0.3)), views))
        turned = Rotation.from_rotvec([0.0, 0.0, math.radians(12)]).as_matrix()
        self.assertTrue(camera._is_new(transform(turned, (0.5, 0.0, 0.3)), views))


if __name__ == "__main__":
    unittest.main()
