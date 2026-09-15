"""Render a measured robot, directly mounted Pocky mesh, and an independent T.

Only MuJoCo FK and rendering are used; this module never connects to hardware.
T_base_object maps the detected T-block frame into robot-base coordinates.
Output uses the saved pinhole intrinsics, so real images must be undistorted
before comparison. The calibration cube's former mounting transform is ignored.
"""

from copy import deepcopy
import hashlib
import importlib.util
import os
from pathlib import Path
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np


_spec = importlib.util.spec_from_file_location(
    "_t_pushing_camera_render", Path(__file__).with_name("camera_calibration_render.py"))
_camera = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_camera)


def _body_pose(body, transform):
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, transform[:3, :3].ravel())
    for name in ("euler", "axisangle", "xyaxes", "zaxis"):
        body.attrib.pop(name, None)
    body.set("pos", " ".join(map(str, transform[:3, 3])))
    body.set("quat", " ".join(map(str, quaternion)))


def _visual_body(robot_tree, source, prefix, visual_name, group):
    """Import the authored visual mesh and its defaults/assets under a namespace."""
    tree = ET.parse(source).getroot()
    _camera._absolute_assets(tree, source)
    source_body = tree.find("./worldbody/body")
    if source_body is None:
        raise ValueError(f"{source} must contain a worldbody/body")
    selected = source_body.findall(f".//geom[@name='{visual_name}']")
    if len(selected) != 1 or selected[0].get("mesh") is None:
        raise ValueError(f"{source} must contain exactly one mesh geom named {visual_name}")
    for parent in source_body.iter():
        for child in list(parent):
            if child.tag in ("joint", "freejoint", "site") or (
                    child.tag == "geom" and child.get("name") != visual_name):
                parent.remove(child)
    visual = selected[0]
    visual.set("group", str(group))
    visual.set("contype", "0")
    visual.set("conaffinity", "0")
    rgba = np.fromstring(visual.get("rgba", "1 1 1 1"), sep=" ")
    rgba[3] = 1
    visual.set("rgba", " ".join(map(str, rgba)))
    for material in tree.findall("./asset/material"):
        rgba = np.fromstring(material.get("rgba", "1 1 1 1"), sep=" ")
        rgba[3] = 1
        material.set("rgba", " ".join(map(str, rgba)))
    for element in tree.iter():
        for attribute in ("name", "mesh", "material", "texture", "class", "childclass"):
            if attribute in element.attrib:
                element.set(attribute, prefix + element.get(attribute))
    robot_defaults = robot_tree.find("default")
    if robot_defaults is None:
        robot_defaults = ET.SubElement(robot_tree, "default")
    imported_defaults = ET.SubElement(robot_defaults, "default", {"class": prefix + "main"})
    source_defaults = tree.find("default")
    if source_defaults is not None:
        for child in source_defaults:
            imported_defaults.append(deepcopy(child))
    source_body.set("childclass", source_body.get("childclass", prefix + "main"))
    assets = robot_tree.find("asset")
    if assets is None:
        assets = ET.SubElement(robot_tree, "asset")
    for asset in tree.findall("./asset/*"):
        assets.append(deepcopy(asset))
    return deepcopy(source_body)


class PushingRenderer(_camera.CalibrationRenderer):
    """Context-managed BGR rendering with an independently positioned T-block."""

    def __init__(self, robot_xml, stick_xml, target_xml, calibration):
        robot_xml, stick_xml, target_xml = (
            Path(path).expanduser().resolve() for path in (robot_xml, stick_xml, target_xml))
        self.robot_model_sha256 = hashlib.sha256(robot_xml.read_bytes()).hexdigest()
        expected = calibration.get("robot_model_sha256")
        if expected is not None and expected != self.robot_model_sha256:
            raise ValueError("Robot XML differs from the model used during calibration")
        self.T_base_camera = _camera._transform(calibration["T_base_camera"], "T_base_camera")
        camera = calibration["camera"]
        self.K = np.asarray(calibration.get("camera_matrix", camera.get("camera_matrix")), dtype=float)
        self.width, self.height = int(camera["width"]), int(camera["height"])
        if (self.width <= 0 or self.height <= 0 or self.K.shape != (3, 3)
                or not np.isfinite(self.K).all() or min(self.K[0, 0], self.K[1, 1]) <= 0
                or not np.allclose(self.K[2], [0, 0, 1])
                or abs(self.K[0, 1]) > 1e-10 or abs(self.K[1, 0]) > 1e-10):
            raise ValueError("Expected positive image dimensions and pinhole intrinsics without skew")

        original = mujoco.MjModel.from_xml_path(str(robot_xml))
        if original.nq != 7 or original.njnt != 7:
            raise ValueError("Expected a seven-joint Franka model")
        site = original.site("attachment_site")
        site_rotation = np.empty(9)
        mujoco.mju_quat2Mat(site_rotation, site.quat)
        mount = _camera._pose(site_rotation, site.pos)
        parent_name = original.body(int(site.bodyid[0])).name
        self.joint_names = [original.joint(i).name for i in range(original.njnt)]

        tree = ET.parse(robot_xml).getroot()
        _camera._absolute_assets(tree, robot_xml)
        stick = _visual_body(tree, stick_xml, "pocky_", "stick_visual", 2)
        target = _visual_body(tree, target_xml, "target_", "cube_visual", 1)
        _body_pose(stick, mount)
        parent = next((body for body in tree.iter("body") if body.get("name") == parent_name), None)
        if parent is None:
            raise ValueError("Cannot locate attachment_site's parent in the robot XML")
        parent.append(stick)
        _body_pose(target, np.eye(4))
        target.set("mocap", "true")
        tree.find("worldbody").append(target)

        self.model = mujoco.MjModel.from_xml_string(ET.tostring(tree, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        self.site_id = self.model.site("attachment_site").id
        self.base_id = self.model.body("base").id
        self.stick_body_id = self.model.body(stick.get("name")).id
        self.object_body_id = self.model.body(target.get("name")).id
        self.object_mocap_id = int(self.model.body_mocapid[self.object_body_id])
        self.qpos_addresses = [int(self.model.joint(name).qposadr[0]) for name in self.joint_names]
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, self.width)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, self.height)
        self.model.vis.headlight.active = 1
        self.model.vis.headlight.ambient[:] = .35
        self.model.vis.headlight.diffuse[:] = .7
        self.options = mujoco.MjvOption()
        self.options.sitegroup[:] = 0
        self.options.geomgroup[3:] = 0
        self.foreground_ids = np.flatnonzero((self.model.geom_group < 3) & (self.model.geom_rgba[:, 3] > 0))
        stick_ids = np.array([self.model.geom("pocky_stick_visual").id])
        object_ids = np.array([self.model.geom("target_cube_visual").id])
        self.semantic_geom_ids = {
            "stick": stick_ids, "object": object_ids,
            "robot": np.setdiff1d(self.foreground_ids, np.r_[stick_ids, object_ids]),
        }
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_USER
        self.renderer = None

    def render(self, qpos, T_base_object):
        """Return BGR, union foreground mask, and FK T_base_ee; poses use meters."""
        if self.renderer is None:
            raise RuntimeError("Use PushingRenderer as a context manager")
        qpos = np.asarray(qpos, dtype=float)
        if qpos.shape != (7,) or not np.isfinite(qpos).all():
            raise ValueError("Measured qpos must contain seven finite joint angles")
        object_pose = _camera._transform(T_base_object, "T_base_object")
        self.data.qpos[self.qpos_addresses] = qpos
        mujoco.mj_forward(self.model, self.data)
        world_base = _camera._pose(self.data.xmat[self.base_id], self.data.xpos[self.base_id])
        world_object = world_base @ object_pose
        self.data.mocap_pos[self.object_mocap_id] = world_object[:3, 3]
        mujoco.mju_mat2Quat(self.data.mocap_quat[self.object_mocap_id], world_object[:3, :3].ravel())
        return super().render(qpos)
