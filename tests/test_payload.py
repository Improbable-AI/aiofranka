import asyncio
import time
import unittest
from types import SimpleNamespace

import mujoco
import numpy as np

from aiofranka.controller import FrankaController
from aiofranka.payload import (
    APPROACH,
    MODEL_PATH,
    _collision_model,
    _is_clear,
    _is_path_clear,
    _plan,
    _read_residual,
    fit_payload,
    payload_regressor,
)
from aiofranka.robot import RobotInterface


HOME = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
MASS = 1.2
COM = np.array([0.01, -0.02, 0.07])


def model_with_tool(gravcomp):
    """The arm with a tool body on the flange, whose gravity MuJoCo compensates if gravcomp."""
    spec = mujoco.MjSpec.from_file(str(MODEL_PATH))
    tool = spec.body("fr3_link7").add_body(name="tool", pos=[0, 0, 0.107], gravcomp=gravcomp)
    tool.mass = MASS
    tool.ipos = COM
    tool.inertia = [2e-3, 2e-3, 1e-3]
    tool.explicitinertial = True
    return spec.compile()


def gravity(model, qpos):
    data = mujoco.MjData(model)
    data.qpos[:7] = qpos
    mujoco.mj_forward(model, data)
    return data.qfrc_bias[:7].copy()


class _Viewer:
    def sync(self):
        pass


def simulated_robot(model):
    """RobotInterface in simulation with the given model and without a viewer."""
    robot = RobotInterface.__new__(RobotInterface)
    robot.real = False
    robot.torque_controller = None
    robot.robot_state = None
    robot.viewer = _Viewer()
    robot.model = model
    robot.data = mujoco.MjData(model)
    robot.site_name = "attachment_site"
    robot.site_id = model.site(robot.site_name).id
    robot.ip = None
    robot.load = {"mass": 0.0, "com": np.zeros(3), "inertia": np.zeros((3, 3))}
    robot.payload = dict(robot.load)
    last_link = model.body("fr3_link7").id
    robot._last_link_inertial = (
        float(model.body_mass[last_link]),
        model.body_ipos[last_link].copy(),
        model.body_iquat[last_link].copy(),
        model.body_inertia[last_link].copy(),
    )
    robot.data.qpos[:7] = HOME
    mujoco.mj_forward(model, robot.data)
    return robot


class PayloadRegressorTest(unittest.TestCase):
    def test_matches_gravity_torques_of_a_tool_body(self):
        arm = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        with_tool = model_with_tool(gravcomp=0)
        theta = np.concatenate([[MASS], MASS * COM])
        rng = np.random.default_rng(0)
        for _ in range(5):
            qpos = HOME + rng.uniform(-0.5, 0.5, 7)
            np.testing.assert_allclose(
                payload_regressor(qpos) @ theta,
                gravity(with_tool, qpos) - gravity(arm, qpos),
                atol=1e-9,
            )


class FitPayloadTest(unittest.TestCase):
    def setUp(self):
        self.poses = _plan(HOME).poses
        self.rng = np.random.default_rng(1)
        self.bias = self.rng.normal(0, 0.3, 7)

    def residuals(self, theta):
        return np.array([
            payload_regressor(q) @ theta + self.bias + self.rng.normal(0, 0.05, 7)
            for q in self.poses
        ])

    def test_recovers_load_and_offsets_from_noisy_torques(self):
        theta = np.concatenate([[MASS], MASS * COM])
        estimate = fit_payload(self.poses, self.residuals(theta))

        self.assertAlmostEqual(estimate.mass, MASS, delta=0.03)
        np.testing.assert_allclose(estimate.com, COM, atol=0.003)
        np.testing.assert_allclose(estimate.bias, self.bias, atol=0.1)
        self.assertLess(abs(estimate.mass - MASS), 4 * estimate.mass_std)
        self.assertLess(estimate.condition, 30)
        # Gravity does not load joint 1, whose axis is vertical.
        self.assertTrue((estimate.rms_after[1:] < estimate.rms_before[1:]).all())

    def test_adds_the_correction_to_the_previous_load(self):
        previous = {"mass": 0.5, "com": np.array([0.0, 0.0, 0.03])}
        theta = np.concatenate([[MASS], MASS * COM])
        theta -= np.concatenate([[previous["mass"]], previous["mass"] * previous["com"]])
        estimate = fit_payload(self.poses, self.residuals(theta), previous=previous)

        self.assertAlmostEqual(estimate.mass, MASS, delta=0.03)
        np.testing.assert_allclose(estimate.com, COM, atol=0.003)
        self.assertEqual(estimate.previous["mass"], 0.5)

    def test_rejects_too_few_poses(self):
        with self.assertRaises(ValueError):
            fit_payload(self.poses[:1], np.zeros((1, 7)))


class PlanTest(unittest.TestCase):
    def test_measures_each_pose_from_both_sides_along_clear_paths(self):
        plan = _plan(HOME, n_poses=12)
        # The default tool size, floor and clearance.
        model, data = _collision_model(0.2, 0.1, 0.0, 0.05)
        limits = model.jnt_range[:7]

        self.assertEqual(len(plan.poses), 12)
        for i, pose in enumerate(plan.poses):
            # Measured twice, once after coming from each side.
            self.assertEqual(np.sum(plan.pose == i), 2)
            for k in np.flatnonzero(plan.pose == i):
                np.testing.assert_allclose(plan.waypoints[k], pose)
            np.testing.assert_allclose(plan.waypoints[np.flatnonzero(plan.pose == i)[0] - 1], pose + APPROACH)
            np.testing.assert_allclose(plan.waypoints[np.flatnonzero(plan.pose == i)[1] - 1], pose - APPROACH)
        path = np.vstack([plan.center, plan.waypoints, plan.center])
        self.assertTrue((path >= limits[:, 0]).all() and (path <= limits[:, 1]).all())
        self.assertTrue(all(_is_path_clear(model, data, a, b) for a, b in zip(path[:-1], path[1:])))
        np.testing.assert_array_equal(path[:, 0], HOME[0])

    def test_says_what_is_too_close_at_the_current_pose(self):
        with self.assertRaisesRegex(ValueError, "the floor and link 1 collide"):
            _plan(HOME, floor=0.5)
        low = np.array([0, 0.5, 0, -1.6, 0, 2.1, 0.785])
        with self.assertRaisesRegex(ValueError, r"the floor is \d cm from the tool, closer than the 5 cm"):
            _plan(low, tool_length=0.25, tool_radius=0.05)


class ReadResidualTest(unittest.TestCase):
    def test_uses_tau_ext_hat_filtered_on_the_real_robot(self):
        # On the robot, tau_ext_hat_filtered matched tau_J minus libfranka's gravity
        # within 2 mNm also while the controller commanded 0.6 Nm: it does not
        # subtract the commanded torque tau_J_d.
        external = np.array([0.0, 0.32, 0.08, -0.03, -0.18, -0.18, -0.39])
        state = SimpleNamespace(q=HOME.tolist(), tau_ext_hat_filtered=external.tolist(),
                                tau_J_d=[0.09, 0.35, 0.13, -0.11, -0.58, -0.31, -0.61])
        robot = SimpleNamespace(real=True, robot_state=state)

        q, residual = _read_residual(robot)

        np.testing.assert_allclose(q, HOME)
        np.testing.assert_allclose(residual, external)


class IdentifyPayloadSimulationTest(unittest.TestCase):
    def identify(self, start_first, n_poses):
        """
        Identify a tool that the simulation does not compensate.

        Returns the robot, the estimate, whether the controller runs afterwards, and
        the longest gap between control ticks while it ran: the robot aborts the
        motion if the 1 kHz control loop stalls.
        """
        robot = simulated_robot(model_with_tool(gravcomp=0))
        controller = FrankaController(robot)
        sessions = []
        step, start = controller.step, controller.start

        def timed_step():
            sessions[-1].append(time.perf_counter())
            step()

        async def marked_start():
            sessions.append([])
            return await start()

        controller.step, controller.start = timed_step, marked_start

        async def run():
            if start_first:
                await controller.start()
            try:
                estimate = await controller.identify_payload(
                    n_poses=n_poses, speed=3.0, settle=0.2, duration=0.2, verbose=False
                )
                return estimate, controller.running
            finally:
                if controller.running:
                    controller.running = False
                    controller.task.cancel()

        estimate, running = asyncio.run(run())
        stall = max(np.max(np.diff(ticks)) for ticks in sessions if len(ticks) > 1)
        return robot, controller, estimate, running, stall

    def test_starts_and_stops_a_stopped_controller(self):
        robot, _, estimate, running, stall = self.identify(start_first=False, n_poses=6)

        self.assertAlmostEqual(estimate.mass, MASS, delta=0.01)
        np.testing.assert_allclose(estimate.com, COM, atol=0.001)
        self.assertLess(stall, 0.02)
        self.assertFalse(running)
        # It only measures.
        self.assertEqual(robot.load["mass"], 0.0)
        np.testing.assert_allclose(robot.data.qpos[:7], HOME, atol=0.2)

    def test_plans_with_a_running_controller_stopped(self):
        _, controller, estimate, running, stall = self.identify(start_first=True, n_poses=3)

        self.assertAlmostEqual(estimate.mass, MASS, delta=0.01)
        self.assertLess(stall, 0.02)
        self.assertTrue(running)
        self.assertEqual(controller.type, "impedance")


if __name__ == "__main__":
    unittest.main()
