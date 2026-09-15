"""Calibration rendering geometry, without OpenGL, devices, or user recordings."""

import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import numpy as np

try:
    import mujoco
except ImportError:
    mujoco = None

if mujoco is not None:
    source = Path(__file__).parents[1] / "examples/camera_calibration_render.py"
    spec = importlib.util.spec_from_file_location("calibration_render_geometry", source)
    rendering = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rendering)


@unittest.skipIf(mujoco is None, "Renderer geometry tests require MuJoCo")
class CalibrationRendererTest(unittest.TestCase):
    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        directory = Path(temporary.name)
        self.robot_xml, self.cube_xml = directory / "robot.xml", directory / "cube.xml"
        bodies = "".join(
            f'<body name="link{i}" pos="0 0 .05"><joint name="joint{i}" axis="{axis}"/>'
            '<geom type="sphere" size=".015" mass=".1" group="2"/>'
            for i, axis in enumerate(["1 0 0", "0 1 0", "0 0 1", "1 0 0", "0 1 0", "0 0 1", "1 0 0"])
        )
        self.robot_xml.write_text(
            '<mujoco><compiler angle="radian"/><worldbody>'
            '<body name="base" pos=".2 -.1 .07" euler=".1 -.2 .3">' + bodies
            + '<site name="attachment_site" pos=".03 -.02 .08" euler=".2 -.3 .4"/>'
            + '</body>' * 8 + '</worldbody></mujoco>'
        )
        self.cube_xml.write_text(
            '<mujoco><worldbody><body name="cube"><freejoint/>'
            '<geom name="cube_visual" type="box" size=".02 .03 .04" group="1"/>'
            '</body></worldbody></mujoco>'
        )
        self.mount = np.eye(4)
        self.mount[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        self.mount[:3, 3] = [.012, -.023, .14]
        self.camera_pose = np.eye(4)
        self.camera_pose[:3, :3] = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
        self.camera_pose[:3, 3] = [.1, -.2, -.8]
        self.K = np.array([[700., 0, 570.], [0, 1100., 402.], [0, 0, 1.]])
        self.calibration = {"T_base_camera": self.camera_pose, "T_ee_cube": self.mount,
                            "camera": {"width": 1280, "height": 720, "camera_matrix": self.K}}
        self.renderer = rendering.CalibrationRenderer(self.robot_xml, self.cube_xml, self.calibration)
        # Keep real MuJoCo camera structs and FK; substitute only pixel generation.
        scene = SimpleNamespace(camera=[mujoco.MjvGLCamera(), mujoco.MjvGLCamera()], stereo=0)
        self.renderer.renderer = SimpleNamespace(
            scene=scene, update_scene=lambda *args, **kwargs: None,
            render=lambda: np.zeros((1, 1, 3), np.uint8),
            enable_segmentation_rendering=lambda: None, disable_segmentation_rendering=lambda: None,
            close=lambda: None,
        )

    def test_cube_is_rigid_at_fk_site_times_mount_including_site_offset(self):
        reference = mujoco.MjModel.from_xml_path(str(self.robot_xml))
        data = mujoco.MjData(reference)
        for joints in (np.zeros(7), np.array([-.4, .2, .3, -.5, .1, .6, -.2])):
            with self.subTest(joints=joints):
                data.qpos[:] = joints
                mujoco.mj_forward(reference, data)
                base = rendering._pose(data.body("base").xmat, data.body("base").xpos)
                site = data.site("attachment_site")
                ee = np.linalg.inv(base) @ rendering._pose(site.xmat, site.xpos)
                _, _, actual_ee = self.renderer.render(joints)
                cube = self.renderer.data.body("calibration_cube")
                actual_cube = np.linalg.inv(base) @ rendering._pose(cube.xmat, cube.xpos)
                np.testing.assert_allclose(actual_ee, ee, atol=1e-12)
                np.testing.assert_allclose(actual_cube, ee @ self.mount, atol=1e-12)
        self.assertEqual(self.renderer.model.nq, 7)
        self.assertEqual(int(self.renderer.model.body("calibration_cube").jntnum[0]), 0)

    def test_offcenter_unequal_focal_lengths_match_optical_pixel_projection(self):
        self.renderer.render(np.zeros(7))
        base = self.renderer.data.body("base")
        world_camera = rendering._pose(base.xmat, base.xpos) @ self.camera_pose
        optical = np.array([[0, 0, 1], [.13, -.08, 1.2], [-.2, .11, 1.8]])
        world = optical @ world_camera[:3, :3].T + world_camera[:3, 3]
        expected = optical[:, :2] / optical[:, 2:] * [700., 1100.] + [570., 402.]
        for camera in self.renderer.renderer.scene.camera:
            delta = world - camera.pos
            right = np.cross(camera.forward, camera.up)
            scale = camera.frustum_near / (delta @ camera.forward)
            x, y = (delta @ right) * scale, (delta @ camera.up) * scale
            left = camera.frustum_center - camera.frustum_width
            u = 1280 * (x - left) / (2 * camera.frustum_width) - .5
            v = 720 * (camera.frustum_top - y) / (camera.frustum_top - camera.frustum_bottom) - .5
            np.testing.assert_allclose(np.column_stack([u, v]), expected, atol=1e-3)

    def test_changed_robot_model_digest_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "differs from the model"):
            rendering.CalibrationRenderer(self.robot_xml, self.cube_xml,
                                          {**self.calibration, "robot_model_sha256": "0" * 64})


if __name__ == "__main__":
    unittest.main()
