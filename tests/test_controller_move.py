"""Joint move regression tests with real trajectory math and no robot backend."""

import ast
import asyncio
from contextlib import redirect_stdout
import io
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
from ruckig import InputParameter, Ruckig, Trajectory


def load_move():
    # Execute the actual method without importing aiofranka's hardware backends.
    path = Path(__file__).resolve().parents[1] / "aiofranka/controller.py"
    tree = ast.parse(path.read_text(), filename=str(path))
    controller = next(node for node in tree.body
                      if isinstance(node, ast.ClassDef) and node.name == "FrankaController")
    move = next(node for node in controller.body
                if isinstance(node, ast.AsyncFunctionDef) and node.name == "move")
    namespace = {"np": np, "InputParameter": InputParameter,
                 "Ruckig": Ruckig, "Trajectory": Trajectory}
    exec(compile(ast.Module(body=[move], type_ignores=[]), str(path), "exec"), namespace)
    return namespace["move"]


MOVE = load_move()
START_Q = np.array([0., 0., 0., -1.57079, 0., 1.57079, -.7853])


class FakeClock:
    def __init__(self, late_wake=0.):
        self.now, self.late_wake = 100., late_wake
        self.sleeps = []
        self.after_sleep = lambda: None

    async def sleep(self, duration):
        self.sleeps.append(duration)
        self.now += duration + self.late_wake
        self.late_wake = 0.
        self.after_sleep()


class NoHardwareRobot:
    @property
    def state(self):
        raise AssertionError("move() must not consume live readOnce() state")


class FakeController:
    def __init__(self, velocity=None, idle_gap=0., frequency=50., late_wake=0.):
        self.robot = NoHardwareRobot()
        self.clock = FakeClock(late_wake)
        self.clock.after_sleep = self.update_cached_state
        self.state = {"qpos": START_Q.copy(),
                      "qvel": np.zeros(7) if velocity is None else velocity.copy()}
        self.state_lock = threading.Lock()
        self._q_desired = START_Q + .7  # Stale target from a previous control mode.
        self._type = "osc"
        self._update_freq = frequency
        self._last_update_time = {"q_desired": self.clock.now - idle_gap, "ee_desired": 7.}
        self.mode_targets, self.targets, self.write_times = [], [], []

    @property
    def q_desired(self):
        return self._q_desired

    @q_desired.setter
    def q_desired(self, target):
        self._q_desired = np.asarray(target).copy()
        self.targets.append(self._q_desired.copy())
        self.write_times.append(self.clock.now)

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, mode):
        self.mode_targets.append((mode, self.q_desired.copy()))
        self._type = mode

    async def set(self, *_):
        raise AssertionError("move() must not use set()'s shared rate-limit clock")

    def update_cached_state(self):
        # Shared cached buffers can change after yielding to the control loop.
        self.state["qpos"][:] += .01
        self.state["qvel"][:] = -.1


def trajectory_for(target, velocity=None):
    parameters = InputParameter(7)
    parameters.current_position = START_Q
    parameters.current_velocity = np.zeros(7) if velocity is None else velocity
    parameters.current_acceleration = np.zeros(7)
    parameters.target_position = target
    parameters.target_velocity = parameters.target_acceleration = np.zeros(7)
    parameters.max_velocity = np.ones(7) * 10
    parameters.max_acceleration = np.ones(7) * 5
    parameters.max_jerk = np.ones(7)
    trajectory = Trajectory(7)
    Ruckig(7).calculate(parameters, trajectory)
    return trajectory


class ControllerMoveTest(unittest.TestCase):
    def run_move(self, controller, target):
        replacements = {"time": SimpleNamespace(perf_counter=lambda: controller.clock.now),
                        "asyncio": SimpleNamespace(sleep=controller.clock.sleep)}
        with redirect_stdout(io.StringIO()), mock.patch.dict(MOVE.__globals__, replacements):
            asyncio.run(MOVE(controller, target))

    def test_cached_state_avoids_hardware_reads_and_preserves_trajectory_sampling(self):
        velocity = np.array([.01, 0., 0., 0., 0., 0., 0.])
        controller = FakeController(velocity)
        target = START_Q.copy()
        target[0] += .02
        trajectory = trajectory_for(target, velocity)
        steps = int(np.ceil(trajectory.duration * 50))
        expected = [START_Q] + [trajectory.at_time(min(i / 50., trajectory.duration))[0]
                                for i in range(steps + 1)]
        self.run_move(controller, target)
        np.testing.assert_allclose(controller.targets, expected, atol=1e-12)
        np.testing.assert_array_equal(controller.targets[-1], target)
        self.assertEqual(len(controller.mode_targets), 1)
        self.assertEqual(controller.mode_targets[0][0], "impedance")
        np.testing.assert_array_equal(controller.mode_targets[0][1], START_Q)

    def test_pause_before_move_cannot_compress_trajectory_and_frequency_is_local(self):
        target = START_Q + np.array([.2, -.1, .1, -.2, 0., .3, .1])
        duration = np.ceil(trajectory_for(target).duration * 50) / 50
        for gap in (0., 1., 30.):
            for frequency in (10., 50., 100.):
                with self.subTest(idle_gap=gap, frequency=frequency):
                    controller = FakeController(idle_gap=gap, frequency=frequency)
                    self.run_move(controller, target)
                    self.assertAlmostEqual(controller.clock.now - 100., duration)
                    # Ignore the initial measured hold, which equals sample zero.
                    np.testing.assert_allclose(np.diff(controller.write_times[1:]), .02, atol=1e-12)
                    self.assertEqual(sum(t == 100. for t in controller.write_times), 2)
                    self.assertEqual(controller._update_freq, frequency)
                    self.assertEqual(controller._last_update_time, {"ee_desired": 7.})
                    np.testing.assert_array_equal(controller.q_desired, target)

    def test_late_wake_slows_move_without_bursting_subsequent_samples(self):
        target = START_Q.copy()
        target[0] += .1
        controller = FakeController(late_wake=1.)
        self.run_move(controller, target)
        gaps = np.diff(controller.write_times[1:])
        self.assertAlmostEqual(gaps[0], 1.02)
        np.testing.assert_allclose(gaps[1:], .02, atol=1e-12)
        nominal = np.ceil(trajectory_for(target).duration * 50) / 50
        self.assertAlmostEqual(controller.clock.now - 100., nominal + 1.)
        np.testing.assert_array_equal(controller.q_desired, target)

    def test_zero_distance_move_installs_exact_hold_without_waiting(self):
        controller = FakeController(idle_gap=30.)
        self.run_move(controller, START_Q)
        self.assertEqual(controller.type, "impedance")
        self.assertEqual(controller.clock.sleeps, [])
        np.testing.assert_array_equal(controller.q_desired, START_Q)
        np.testing.assert_array_equal(controller.mode_targets[0][1], START_Q)
        self.assertFalse(np.shares_memory(controller.q_desired, controller.state["qpos"]))
        self.assertNotIn("q_desired", controller._last_update_time)

    def test_missing_cached_state_fails_before_changing_mode_or_target(self):
        controller = FakeController()
        controller.state = None
        previous_target = controller.q_desired.copy()
        with self.assertRaisesRegex(RuntimeError, "No cached robot state"):
            self.run_move(controller, START_Q)
        self.assertEqual(controller.type, "osc")
        np.testing.assert_array_equal(controller.q_desired, previous_target)
        self.assertEqual(controller.mode_targets, [])
        self.assertEqual(controller.targets, [])

    def test_planner_exception_leaves_previous_mode_and_target_intact(self):
        controller = FakeController()
        previous_target = controller.q_desired.copy()
        planner = mock.Mock()
        planner.calculate.side_effect = RuntimeError("Offline planner failure")
        with mock.patch.dict(MOVE.__globals__, Ruckig=mock.Mock(return_value=planner)):
            with self.assertRaisesRegex(RuntimeError, "Offline planner failure"):
                self.run_move(controller, START_Q)
        self.assertEqual(controller.type, "osc")
        np.testing.assert_array_equal(controller.q_desired, previous_target)
        self.assertEqual(controller.mode_targets, [])
        self.assertEqual(controller.targets, [])


if __name__ == "__main__":
    unittest.main()
