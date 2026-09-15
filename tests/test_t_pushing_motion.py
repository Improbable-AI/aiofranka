"""Offline FK/IK and joint-command checks; no devices are opened."""

import importlib.util
from pathlib import Path
import time
import unittest
from unittest import mock

import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("t_pushing_motion", ROOT / "examples/t_pushing_motion.py")
motion_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(motion_module)
HOME_Q, DOWNWARD = motion_module.HOME_Q, motion_module.DOWNWARD


class FakeRobot:
    def __init__(self, kinematics, qpos=HOME_Q):
        self.kin = kinematics
        self.qpos = np.asarray(qpos).copy()
        self.commands = []
        self.mode = "osc"

    @property
    def state(self):
        return {"qpos": self.qpos.copy(), "qvel": np.zeros(7),
                "ee": self.kin.fk(self.qpos), "timestamp": time.time()}

    def switch(self, mode):
        self.mode = mode
        self.commands.append(("switch", mode))

    def set_freq(self, frequency):
        self.frequency = frequency

    @property
    def q_desired(self):
        return self.qpos.copy()

    @q_desired.setter
    def q_desired(self, target):
        self.commands.append(("q_desired", np.asarray(target).copy()))
        self.qpos = np.asarray(target).copy()

    @property
    def ee_desired(self):
        return self._ee_desired

    @ee_desired.setter
    def ee_desired(self, target):
        self.commands.append(("ee_desired", np.asarray(target).copy()))
        self._ee_desired = np.asarray(target).copy()

    def move(self, target):
        self.mode = "impedance"
        self.commands.append(("move", np.asarray(target).copy()))
        self.qpos = np.asarray(target).copy()


class PushingMotionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = motion_module.Kinematics(ROOT / "aiofranka/model/fr3.xml")

    def assert_pose_close(self, actual, target):
        self.assertLess(np.linalg.norm(actual[:3, 3] - target[:3, 3]), .0005)
        self.assertLess(motion_module._angle(actual[:3, :3], target[:3, :3]), .005)

    def test_home_fk_and_tip_frame_conversion(self):
        home = self.kin.fk(HOME_Q)
        np.testing.assert_allclose(home[:3, 3], [.554499478, 0, .624502429], atol=1e-8)
        np.testing.assert_allclose(self.kin.tip(home), home[:3, 3] + [0, 0, -.161], atol=1e-10)
        target = self.kin.ee_for_tip([.5, .1, .07])
        np.testing.assert_array_equal(target[:3, :3], DOWNWARD)
        np.testing.assert_allclose(target[:3, 3], [.5, .1, .231], atol=1e-12)
        np.testing.assert_allclose(self.kin.tip(target), [.5, .1, .07], atol=1e-12)

    def test_ik_reachable_pose_and_invalid_targets(self):
        target = self.kin.ee_for_tip([.5, .04, .18])
        solution = self.kin.ik(target, HOME_Q)
        self.assert_pose_close(self.kin.fk(solution), target)
        with self.assertRaisesRegex(ValueError, "IK cannot reach"):
            self.kin.ik(self.kin.ee_for_tip([3, 0, .2]), HOME_Q)
        with self.assertRaisesRegex(ValueError, "joint limits"):
            self.kin.fk(np.ones(7) * 10)
        with self.assertRaisesRegex(ValueError, "finite"):
            self.kin.ee_for_tip([.5, np.nan, .02])

    def test_offline_start_plan_solves_final_endpoint_without_robot_access(self):
        robot = mock.Mock()
        helper = motion_module.Motion(robot, self.kin)
        result = helper.plan_start([.5, 0, .07])
        self.assert_pose_close(self.kin.fk(result["start_q"]), self.kin.ee_for_tip([.5, 0, .07]))
        np.testing.assert_array_equal(result["start_pose"][:3, :3], DOWNWARD)
        self.assertEqual(set(result), {"home_pose", "start_pose", "start_q"})
        self.assertEqual(robot.mock_calls, [])

    @mock.patch.object(motion_module.time, "sleep")
    def test_home_uses_one_direct_joint_move(self, sleep):
        rotation = Rotation.from_euler("xyz", [165, 0, 25], degrees=True).as_matrix()
        initial = self.kin.ee_for_tip([.45, .03, .15], rotation)
        robot = FakeRobot(self.kin, self.kin.ik(initial, HOME_Q))
        helper = motion_module.Motion(robot, self.kin)
        final = helper.go_home()
        moves = [q for name, q in robot.commands if name == "move"]
        self.assertEqual(len(moves), 1)
        np.testing.assert_array_equal(moves[0], HOME_Q)
        np.testing.assert_array_equal(final["qpos"], HOME_Q)
        self.assertEqual(robot.mode, "impedance")
        self.assertFalse(any(name in ("switch", "ee_desired") for name, _ in robot.commands))
        sleep.assert_called_once_with(1.0)

    @mock.patch.object(motion_module.time, "sleep")
    def test_approach_moves_directly_to_target_height_and_osc_is_explicit(self, sleep):
        robot = FakeRobot(self.kin)
        helper = motion_module.Motion(robot, self.kin)
        final = helper.approach([.5, .03, .07])
        moves = [q for name, q in robot.commands if name == "move"]
        self.assertEqual(len(moves), 1)
        self.assert_pose_close(self.kin.fk(moves[0]), self.kin.ee_for_tip([.5, .03, .07]))
        self.assert_pose_close(final["ee"], self.kin.ee_for_tip([.5, .03, .07]))
        self.assertEqual(robot.mode, "impedance")
        self.assertFalse(any(name == "ee_desired" for name, _ in robot.commands))
        sleep.assert_called_once_with(1.0)
        helper.configure_osc()
        self.assertEqual(robot.mode, "osc")
        np.testing.assert_array_equal(robot.ee_desired, final["ee"])
        for name, expected in motion_module.OSC_GAINS.items():
            np.testing.assert_array_equal(getattr(robot, name), expected)

    def test_unreachable_approach_fails_before_commands(self):
        robot = FakeRobot(self.kin)
        with self.assertRaisesRegex(ValueError, "IK cannot reach"):
            motion_module.Motion(robot, self.kin).approach([3, 0, .02])
        self.assertEqual(robot.commands, [])


if __name__ == "__main__":
    unittest.main()
