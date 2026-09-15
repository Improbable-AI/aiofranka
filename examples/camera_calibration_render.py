"""Headless MuJoCo rendering using a fixed camera and calibrated rigid cube mount.

Render output is BGR in the supplied pinhole intrinsics, without lens distortion.
Real images must be undistorted to the same intrinsics before overlaying them.
Only measured robot joints determine the rendered pose; detections are not used.
"""

from copy import deepcopy
import hashlib
import os
from pathlib import Path
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np


def _pose(rotation, position):
    transform = np.eye(4)
    transform[:3, :3] = np.asarray(rotation).reshape(3, 3)
    transform[:3, 3] = position
    return transform


def _transform(value, name):
    result = np.asarray(value, dtype=float)
    if (result.shape != (4, 4) or not np.isfinite(result).all()
            or not np.allclose(result[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(result[:3, :3].T @ result[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(result[:3, :3]), 1, atol=1e-6)):
        raise ValueError(f"{name} must be a finite rigid transform")
    return result.copy()


def _absolute_assets(tree, xml_path):
    """Keep mesh and texture files valid after composing XML documents in memory."""
    compiler = tree.find("compiler")
    settings = {} if compiler is None else compiler.attrib
    for element in tree.findall("./asset/*"):
        filename = element.get("file")
        if filename:
            directory = settings.get("meshdir" if element.tag == "mesh" else "texturedir", "")
            element.set("file", str((xml_path.parent / directory / filename).resolve()))
    if compiler is not None:
        compiler.attrib.pop("meshdir", None)
        compiler.attrib.pop("texturedir", None)


class CalibrationRenderer:
    """Render FK robot/cube poses as ``(BGR, foreground_mask, T_base_ee)``."""

    def __init__(self, robot_xml, cube_xml, calibration):
        robot_xml, cube_xml = Path(robot_xml).resolve(), Path(cube_xml).resolve()
        self.robot_model_sha256 = hashlib.sha256(robot_xml.read_bytes()).hexdigest()
        expected = calibration.get("robot_model_sha256")
        if expected and expected != self.robot_model_sha256:
            raise ValueError("Robot XML differs from the model used during calibration")
        self.T_base_camera = _transform(calibration["T_base_camera"], "T_base_camera")
        self.T_ee_cube = _transform(calibration["T_ee_cube"], "T_ee_cube")
        camera = calibration["camera"]
        self.K = np.asarray(calibration.get("camera_matrix", camera["camera_matrix"]), dtype=float)
        self.width, self.height = int(camera["width"]), int(camera["height"])
        if (self.width <= 0 or self.height <= 0 or self.K.shape != (3, 3)
                or not np.isfinite(self.K).all() or min(self.K[0, 0], self.K[1, 1]) <= 0
                or not np.allclose(self.K[2], [0, 0, 1])
                or abs(self.K[0, 1]) > 1e-10 or abs(self.K[1, 0]) > 1e-10):
            raise ValueError("Expected positive image dimensions and pinhole intrinsics without skew")

        # Recover the site's local offset from compiled FK, including its rotation.
        robot_model = mujoco.MjModel.from_xml_path(str(robot_xml))
        site = robot_model.site("attachment_site")
        site_rotation = np.empty(9)
        mujoco.mju_quat2Mat(site_rotation, site.quat)
        T_body_cube = _pose(site_rotation, site.pos) @ self.T_ee_cube
        parent_name = robot_model.body(int(site.bodyid[0])).name
        self.joint_names = [robot_model.joint(i).name for i in range(robot_model.njnt)]
        if robot_model.nq != 7 or robot_model.njnt != 7:
            raise ValueError("Expected the seven-joint Franka model without additional free joints")

        robot_tree, cube_tree = ET.parse(robot_xml).getroot(), ET.parse(cube_xml).getroot()
        _absolute_assets(robot_tree, robot_xml)
        _absolute_assets(cube_tree, cube_xml)
        robot_assets = robot_tree.find("asset")
        if robot_assets is None:
            robot_assets = ET.SubElement(robot_tree, "asset")
        # Namespace imported cube assets and references without altering the originals.
        prefix = "calibration_"
        for element in cube_tree.iter():
            for attribute in ("name", "mesh", "material", "texture"):
                if attribute in element.attrib:
                    element.set(attribute, prefix + element.get(attribute))
        for asset in cube_tree.findall("./asset/*"):
            robot_assets.append(deepcopy(asset))
        cube_body = deepcopy(cube_tree.find("./worldbody/body"))
        if cube_body is None:
            raise ValueError("Cube XML must contain a worldbody/body")
        for parent in cube_body.iter():
            for child in list(parent):
                hidden_geom = child.tag == "geom" and (
                    "collision" in child.get("name", "")
                    or np.fromstring(child.get("rgba", "1 1 1 1"), sep=" ")[-1] == 0)
                if child.tag in ("joint", "freejoint", "site") or hidden_geom:
                    parent.remove(child)
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(quaternion, T_body_cube[:3, :3].ravel())
        cube_body.set("pos", " ".join(map(str, T_body_cube[:3, 3])))
        cube_body.set("quat", " ".join(map(str, quaternion)))
        parent = next((body for body in robot_tree.iter("body") if body.get("name") == parent_name), None)
        if parent is None:
            raise ValueError("Could not locate attachment_site's body in the robot XML")
        parent.append(cube_body)
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(robot_tree, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        self.site_id = self.model.site("attachment_site").id
        self.base_id = self.model.body("base").id
        self.qpos_addresses = [int(self.model.joint(name).qposadr[0]) for name in self.joint_names]
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, self.width)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, self.height)
        self.model.vis.headlight.active = 1
        self.model.vis.headlight.ambient[:] = 0.35
        self.model.vis.headlight.diffuse[:] = 0.7
        self.options = mujoco.MjvOption()
        self.options.sitegroup[:] = 0
        self.options.geomgroup[3:] = 0  # Robot visual group 2; cube visual group 1.
        self.foreground_ids = np.flatnonzero((self.model.geom_group < 3) & (self.model.geom_rgba[:, 3] > 0))
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_USER
        self.renderer = None

    def __enter__(self):
        self.renderer = mujoco.Renderer(self.model, width=self.width, height=self.height)
        return self

    def render(self, qpos):
        if self.renderer is None:
            raise RuntimeError("Use CalibrationRenderer as a context manager")
        qpos = np.asarray(qpos, dtype=float)
        if qpos.shape != (7,) or not np.isfinite(qpos).all():
            raise ValueError("Measured qpos must contain seven finite joint angles")
        self.data.qpos[self.qpos_addresses] = qpos
        mujoco.mj_forward(self.model, self.data)
        T_world_base = _pose(self.data.xmat[self.base_id], self.data.xpos[self.base_id])
        T_world_ee = _pose(self.data.site_xmat[self.site_id], self.data.site_xpos[self.site_id])
        T_world_camera = T_world_base @ self.T_base_camera
        near = float(self.model.vis.map.znear * self.model.stat.extent)
        far = float(self.model.vis.map.zfar * self.model.stat.extent)
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        for camera in self.renderer.scene.camera:
            camera.pos[:] = T_world_camera[:3, 3]
            camera.forward[:] = T_world_camera[:3, 2]
            camera.up[:] = -T_world_camera[:3, 1]
            camera.orthographic = 0
            camera.frustum_near, camera.frustum_far = near, far
            # OpenCV integer pixel centers vs OpenGL half-integer sample centers.
            camera.frustum_center = (self.width / 2 - cx - 0.5) * near / fx
            camera.frustum_bottom = -(self.height - cy - 0.5) * near / fy
            camera.frustum_top = (cy + 0.5) * near / fy
            camera.frustum_width = self.width / 2 * near / fx  # MuJoCo uses HALF-width.
        self.renderer.scene.stereo = mujoco.mjtStereo.mjSTEREO_NONE
        self.renderer.update_scene(self.data, camera=self.camera, scene_option=self.options)
        rgb = self.renderer.render()
        self.renderer.enable_segmentation_rendering()
        try:
            segmentation = self.renderer.render()
        finally:
            self.renderer.disable_segmentation_rendering()
        mask = ((segmentation[:, :, 1] == mujoco.mjtObj.mjOBJ_GEOM)
                & np.isin(segmentation[:, :, 0], self.foreground_ids))
        return rgb[:, :, ::-1].copy(), mask, np.linalg.inv(T_world_base) @ T_world_ee

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    def __exit__(self, *_):
        self.close()
