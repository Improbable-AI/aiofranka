"""Franka FK/IK and joint-space moves for the directly mounted 161 mm stick.

Poses use attachment_site in the robot base frame, in meters. IK selects joint
endpoints; robot.move() interpolates in joint space. Paths are not Cartesian
straight lines and are not collision checked.
"""

import time
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


HOME_Q = np.array([0., 0., 0., -1.57079, 0., 1.57079, -0.7853])
DOWNWARD = np.diag([1., -1., -1.])
OSC_GAINS = {
    "ee_kp": [64., 64., 144., 144., 144., 144.],
    "ee_kd": [16., 16., 24., 24., 24., 24.],
    "null_kp": [1.] * 7, "null_kd": [2.] * 7,
}


def _vector(value, length, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (length,) or not np.isfinite(value).all():
        raise ValueError(f"{name} must contain {length} finite values")
    return value.copy()


def _pose(value):
    value = np.asarray(value, dtype=float)
    if (value.shape != (4, 4) or not np.isfinite(value).all()
            or not np.allclose(value[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(value[:3, :3]), 1, atol=1e-6)):
        raise ValueError("Expected a finite rigid 4x4 end-effector pose")
    return value.copy()


def _angle(first, second):
    return float(Rotation.from_matrix(first @ second.T).magnitude())


class Kinematics:
    """Seven-joint FK/IK with a tip offset from attachment_site."""

    def __init__(self, robot_xml, tip_offset=(0., 0., .161)):
        self.model = mujoco.MjModel.from_xml_path(str(Path(robot_xml).resolve()))
        if self.model.nq != 7 or self.model.nv != 7 or self.model.njnt != 7:
            raise ValueError("Expected a seven-joint Franka model")
        self.data = mujoco.MjData(self.model)
        self.site = self.model.site("attachment_site").id
        self.base = self.model.body("base").id
        self.limits = self.model.jnt_range.copy()
        self.tip_offset = _vector(tip_offset, 3, "tip_offset")

    def check_joints(self, q):
        q = _vector(q, 7, "Joint positions")
        if np.any(q < self.limits[:, 0]) or np.any(q > self.limits[:, 1]):
            raise ValueError("Joint target is outside the model's joint limits")
        return q

    def fk(self, q):
        self.data.qpos[:] = self.check_joints(q)
        mujoco.mj_forward(self.model, self.data)
        rotation = self.data.xmat[self.base].reshape(3, 3)
        pose = np.eye(4)
        pose[:3, :3] = rotation.T @ self.data.site_xmat[self.site].reshape(3, 3)
        pose[:3, 3] = rotation.T @ (self.data.site_xpos[self.site] - self.data.xpos[self.base])
        return pose

    FK = fk

    def tip(self, pose):
        pose = _pose(pose)
        return pose[:3, 3] + pose[:3, :3] @ self.tip_offset

    def ee_for_tip(self, xyz, rotation=DOWNWARD):
        pose = np.eye(4)
        pose[:3, :3] = np.asarray(rotation, dtype=float)
        pose[:3, 3] = _vector(xyz, 3, "Tip position") - pose[:3, :3] @ self.tip_offset
        return _pose(pose)

    def ik(self, target, seed):
        target = _pose(target)
        lower, upper = self.limits[:, 0] + 1e-6, self.limits[:, 1] - 1e-6
        seed = np.clip(_vector(seed, 7, "IK seed"), lower, upper)

        def residual(q):
            current = self.fk(q)
            rotation = Rotation.from_matrix(target[:3, :3] @ current[:3, :3].T).as_rotvec()
            # A small posture term selects the nearby redundant-joint solution.
            return np.r_[5 * (current[:3, 3] - target[:3, 3]), rotation, .003 * (q - seed)]

        best = None
        for initial in (seed, np.clip(HOME_Q, lower, upper)):
            result = least_squares(residual, initial, bounds=(lower, upper), x_scale="jac",
                                   max_nfev=250, ftol=1e-9, xtol=1e-9, gtol=1e-9)
            pose = self.fk(result.x)
            position_error = np.linalg.norm(pose[:3, 3] - target[:3, 3])
            rotation_error = _angle(pose[:3, :3], target[:3, :3])
            best = (position_error, rotation_error)
            if position_error <= .0005 and rotation_error <= .005:
                return result.x.copy()
        raise ValueError(f"IK cannot reach target within tolerance: {best[0]*1000:.1f} mm, "
                         f"{np.rad2deg(best[1]):.2f} degrees")


class Motion:
    """Joint-space positioning, followed by OSC only for SpaceMouse pushing."""

    def __init__(self, robot, kinematics, *, home_q=HOME_Q):
        self.robot, self.kin = robot, kinematics
        self.home_q = self.kin.check_joints(home_q)
        self.osc_gains = OSC_GAINS

    def read_state(self):
        return self.robot.state

    def hold(self):
        state = self.read_state()
        self.robot.ee_desired = state["ee"].copy()
        return state

    def configure_osc(self):
        for name, value in self.osc_gains.items():
            setattr(self.robot, name, np.asarray(value, dtype=float))
        self.hold()
        self.robot.switch("osc")

    def _configure_joints(self):
        self.robot.kp = np.array([80., 80., 80., 80., 48., 48., 48.])
        self.robot.kd = np.array([8., 8., 8., 8., 6., 6., 6.])
        self.robot.set_freq(50)

    def _move_joints(self, q, label):
        target_tip = self.kin.tip(self.kin.fk(q))
        print(f"{label} (joint space): target tip {target_tip.round(3).tolist()} m in base")
        self.robot.move(q)  # aiofranka supplies the joint trajectory and impedance mode.
        time.sleep(1.0)
        state = self.read_state()
        print(f"  Measured tip after move: {self.kin.tip(state['ee']).round(3).tolist()} m in base")
        return state

    def go_home(self):
        self._configure_joints()
        return self._move_joints(self.home_q, "Home")

    def plan_start(self, start_tip_xyz):
        """Solve the final joint endpoint directly from the home posture."""
        start_pose = self.kin.ee_for_tip(start_tip_xyz)
        q_start = self.kin.ik(start_pose, self.home_q)
        return {"home_pose": self.kin.fk(self.home_q), "start_pose": start_pose,
                "start_q": q_start}

    def approach(self, start_tip_xyz):
        plan = self.plan_start(start_tip_xyz)
        self._configure_joints()
        return self._move_joints(plan["start_q"], "Start")
