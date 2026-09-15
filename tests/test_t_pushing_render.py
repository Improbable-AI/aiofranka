"""Offline mesh/FK checks and one isolated EGL render; no devices are accessed."""

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import numpy as np

try:
    import mujoco
except ImportError:
    mujoco = None

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "examples/t_pushing_render.py"
if mujoco is not None:
    spec = importlib.util.spec_from_file_location("pushing_render", SOURCE)
    rendering = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rendering)


@unittest.skipIf(mujoco is None, "Pushing render tests require MuJoCo")
class PushingRenderTest(unittest.TestCase):
    def setUp(self):
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.directory = Path(directory.name)
        self.robot_xml = self.directory / "robot.xml"
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
        self.stick_xml = ROOT / "assets/pocky/stick.xml"
        self.target_xml = ROOT / "assets/t_shape_target_xl/mujoco/cube.xml"
        camera = np.eye(4)
        camera[2, 3] = -1
        old_mount = np.eye(4)
        old_mount[:3, 3] = [3, 4, 5]  # An obsolete calibration mount must have no effect.
        self.calibration = {
            "T_base_camera": camera.tolist(), "T_ee_cube": old_mount.tolist(),
            "camera": {"width": 640, "height": 480,
                       "camera_matrix": [[520., 0, 301.], [0, 610., 211.], [0, 0, 1.]]},
        }
        self.view = rendering.PushingRenderer(self.robot_xml, self.stick_xml, self.target_xml, self.calibration)
        self.addCleanup(self.view.close)
        # Geometry tests keep MuJoCo FK/camera structs and substitute pixel generation.
        self.view.renderer = SimpleNamespace(
            scene=SimpleNamespace(camera=[mujoco.MjvGLCamera(), mujoco.MjvGLCamera()], stereo=0),
            update_scene=lambda *args, **kwargs: None,
            render=lambda: np.zeros((1, 1, 3), np.uint8),
            enable_segmentation_rendering=lambda: None, disable_segmentation_rendering=lambda: None,
            close=lambda: None,
        )

    def pose(self, body_id):
        return rendering._camera._pose(self.view.data.xmat[body_id], self.view.data.xpos[body_id])

    def test_stick_mount_matches_site_and_object_pose_is_independent(self):
        object_pose = np.eye(4)
        object_pose[:3, 3] = [.25, .07, .5]
        fixed_object = None
        for q in (np.zeros(7), np.array([-.4, .2, .3, -.5, .1, .6, -.2])):
            _, _, ee = self.view.render(q, object_pose)
            base = self.pose(self.view.base_id)
            stick = np.linalg.inv(base) @ self.pose(self.view.stick_body_id)
            target = np.linalg.inv(base) @ self.pose(self.view.object_body_id)
            np.testing.assert_allclose(stick, ee, atol=1e-12)
            np.testing.assert_allclose(target, object_pose, atol=1e-12)
            if fixed_object is not None:
                np.testing.assert_allclose(self.pose(self.view.object_body_id), fixed_object, atol=1e-12)
            fixed_object = self.pose(self.view.object_body_id)
        fixed_stick = self.pose(self.view.stick_body_id)
        object_pose[:3, 3] += [.1, -.05, .02]
        self.view.render(q, object_pose)
        np.testing.assert_allclose(self.pose(self.view.stick_body_id), fixed_stick, atol=1e-12)
        self.assertEqual(self.view.model.nq, 7)
        self.assertEqual(self.view.model.nmocap, 1)

    def test_authored_stick_mesh_offset_apex_and_opacity_are_preserved(self):
        self.view.render(np.zeros(7), np.eye(4))
        geom_id = self.view.semantic_geom_ids["stick"][0]
        mesh_id = self.view.model.geom_dataid[geom_id]
        start = self.view.model.mesh_vertadr[mesh_id]
        count = self.view.model.mesh_vertnum[mesh_id]
        vertices = self.view.model.mesh_vert[start:start + count]
        world = vertices @ self.view.data.geom_xmat[geom_id].reshape(3, 3).T + self.view.data.geom_xpos[geom_id]
        stick = self.pose(self.view.stick_body_id)
        local = (world - stick[:3, 3]) @ stick[:3, :3]
        # The OBJ apex is 156 mm; stick_visual's authored +5 mm offset makes 161 mm.
        self.assertAlmostEqual(float(local[:, 2].max()), .161, places=6)
        source = mujoco.MjModel.from_xml_path(str(self.stick_xml))
        source_data = mujoco.MjData(source)
        mujoco.mj_forward(source, source_data)
        source_geom = source.geom("stick_visual").id
        np.testing.assert_allclose(
            np.linalg.inv(stick) @ rendering._camera._pose(
                self.view.data.geom_xmat[geom_id], self.view.data.geom_xpos[geom_id]),
            rendering._camera._pose(source_data.geom_xmat[source_geom], source_data.geom_xpos[source_geom]),
            atol=1e-10,
        )
        self.assertEqual(self.view.model.geom_rgba[geom_id, 3], 1)
        self.assertEqual(self.view.model.mat_rgba[self.view.model.geom_matid[geom_id], 3], 1)
        imported_names = [self.view.model.geom(i).name for i in range(self.view.model.ngeom)
                          if self.view.model.geom(i).name.startswith(("pocky_", "target_"))]
        self.assertEqual(imported_names, ["pocky_stick_visual", "target_cube_visual"])

    def test_bad_object_pose_and_robot_digest_are_rejected(self):
        invalid = np.eye(4)
        invalid[0, 0] = 2
        with self.assertRaisesRegex(ValueError, "T_base_object"):
            self.view.render(np.zeros(7), invalid)
        with self.assertRaisesRegex(ValueError, "differs from the model"):
            rendering.PushingRenderer(self.robot_xml, self.stick_xml, self.target_xml,
                                      {**self.calibration, "robot_model_sha256": "0" * 64})

    def test_isolated_headless_render_matches_projected_target_bounds(self):
        calibration_path = self.directory / "calibration.json"
        calibration_path.write_text(json.dumps(self.calibration))
        program = r'''
import importlib.util, json, sys
import numpy as np
spec = importlib.util.spec_from_file_location("render", sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
import mujoco
calibration = json.load(open(sys.argv[5]))
pose = np.eye(4)
pose[:3, :3] = [[1,0,0], [0,0,-1], [0,1,0]]
pose[:3, 3] = [.25, .07, .5]
with module.PushingRenderer(sys.argv[2], sys.argv[3], sys.argv[4], calibration) as view:
    image, mask, ee = view.render(np.zeros(7), pose)
    view.renderer.enable_segmentation_rendering()
    segments = view.renderer.render()
    view.renderer.disable_segmentation_rendering()
    gid = view.semantic_geom_ids["object"][0]
    target = (segments[:, :, 0] == gid) & (segments[:, :, 1] == mujoco.mjtObj.mjOBJ_GEOM)
    yy, xx = np.nonzero(target)
    mid = view.model.geom_dataid[gid]
    start, count = view.model.mesh_vertadr[mid], view.model.mesh_vertnum[mid]
    vertices = view.model.mesh_vert[start:start+count]
    world = vertices @ view.data.geom_xmat[gid].reshape(3,3).T + view.data.geom_xpos[gid]
    base = module._camera._pose(view.data.xmat[view.base_id], view.data.xpos[view.base_id])
    camera = base @ view.T_base_camera
    optical = (world-camera[:3,3]) @ camera[:3,:3]
    pixels = optical[:, :2] / optical[:, 2:] * [view.K[0,0], view.K[1,1]] + view.K[:2,2]
    print(json.dumps({"shape":list(image.shape), "mask_pixels":int(mask.sum()),
        "bounds":[int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())],
        "projected_bounds":np.r_[pixels.min(0), pixels.max(0)].tolist()}))
'''
        environment = dict(os.environ, MUJOCO_GL="egl")
        completed = subprocess.run(
            [sys.executable, "-c", program, str(SOURCE), str(self.robot_xml), str(self.stick_xml),
             str(self.target_xml), str(calibration_path)],
            capture_output=True, text=True, env=environment, timeout=30,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        result = json.loads(completed.stdout.splitlines()[-1])
        self.assertEqual(result["shape"], [480, 640, 3])
        self.assertGreater(result["mask_pixels"], 100)
        np.testing.assert_allclose(result["bounds"], result["projected_bounds"], atol=2)


if __name__ == "__main__":
    unittest.main()
