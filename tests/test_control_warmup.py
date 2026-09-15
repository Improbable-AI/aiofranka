"""Real MuJoCo/control math with transport and IPC calls forbidden."""

import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest import mock

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("control_warmup_test", ROOT / "aiofranka/control_warmup.py")
warmup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(warmup)


def load_control_laws():
    source = ROOT / "aiofranka/controller.py"
    tree = ast.parse(source.read_text(), filename=str(source))
    controller = next(node for node in tree.body
                      if isinstance(node, ast.ClassDef) and node.name == "FrankaController")
    methods = [node for node in controller.body
               if isinstance(node, ast.FunctionDef) and node.name in {"_impedance_step", "_osc_step"}]
    namespace = {"np": np, "R": Rotation}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), "exec"), namespace)
    return type("DryController", (), {node.name: namespace[node.name] for node in methods})


DryController = load_control_laws()


class ForbiddenAccess:
    def __getattr__(self, name):
        raise AssertionError(f"Warmup touched live backend attribute {name}")


class ForbiddenLock:
    def __enter__(self):
        raise AssertionError("Warmup acquired the real controller's lock")

    def __exit__(self, *_):
        pass


class ControlWarmupTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(ROOT / "aiofranka/model/fr3.xml"))

    def controller(self):
        data = mujoco.MjData(self.model)
        data.qpos[:] = [0., .12, -.08, -1.57079, .05, 1.57079, -.7853]
        data.qvel[:] = np.linspace(-.01, .01, 7)
        data.ctrl[:] = np.linspace(-.03, .03, 7)
        mujoco.mj_forward(self.model, data)
        robot = ForbiddenAccess()
        robot.model, robot.data = self.model, data
        robot.site_id = self.model.site("attachment_site").id
        # Explicit traps give useful messages if later warmup changes call them.
        for method in ("start", "stop", "step"):
            setattr(robot, method, mock.Mock(side_effect=AssertionError(f"Real robot.{method} called")))
        controller = DryController()
        controller.robot = robot
        controller._shm = ForbiddenAccess()
        controller.state_lock = ForbiddenLock()
        controller.error_callback = mock.Mock(side_effect=AssertionError("Real error callback called"))
        controller.type, controller.running, controller.clip = "osc", False, True
        controller.torque_diff_limit = 990.
        controller.kp = np.full(7, 80.)
        controller.kd = np.full(7, 4.)
        controller.ee_kp = np.array([64., 64., 144., 144., 144., 144.])
        controller.ee_kd = np.array([16., 16., 24., 24., 24., 24.])
        controller.null_kp, controller.null_kd = np.ones(7), np.full(7, 2.)
        controller.torque_limit = np.array([87., 87., 87., 87., 12., 12., 12.])
        controller.initial_qpos = data.qpos.copy() + .1
        controller.q_desired = data.qpos.copy() + .2
        controller.initial_ee = np.eye(4)
        controller.ee_desired = np.eye(4)
        controller.ee_desired[:3, 3] = [.3, -.1, .7]
        controller.torque = np.arange(7.)
        controller.state = {"original": np.array([13., 17.])}
        return controller

    def snapshot(self, controller):
        return {
            "references": vars(controller).copy(),
            "arrays": {name: value.copy() for name, value in vars(controller).items()
                       if isinstance(value, np.ndarray)},
            "data": {name: getattr(controller.robot.data, name).copy() for name in (
                "qpos", "qvel", "ctrl", "act", "mocap_pos", "mocap_quat", "qacc",
                "qacc_warmstart", "site_xpos", "site_xmat")},
            "cached_state": controller.state["original"].copy(),
        }

    def assert_unchanged(self, controller, before):
        self.assertEqual(set(vars(controller)), set(before["references"]))
        for name, value in before["references"].items():
            self.assertIs(getattr(controller, name), value, name)
        for name, value in before["arrays"].items():
            np.testing.assert_array_equal(getattr(controller, name), value, err_msg=name)
        for name, value in before["data"].items():
            np.testing.assert_array_equal(getattr(controller.robot.data, name), value, err_msg=name)
        np.testing.assert_array_equal(controller.state["original"], before["cached_state"])
        for method in ("start", "stop", "step"):
            getattr(controller.robot, method).assert_not_called()
        controller.error_callback.assert_not_called()

    def test_real_laws_run_100_times_each_without_touching_live_state_or_transport(self):
        controller = self.controller()
        before = self.snapshot(controller)
        counts = {"impedance": 0, "osc": 0}
        originals = {name: getattr(DryController, f"_{name}_step") for name in counts}

        def counted(name):
            def run(shadow, state):
                counts[name] += 1
                self.assertIsNot(shadow, controller)
                self.assertIsNot(shadow.robot, controller.robot)
                self.assertIsInstance(shadow.robot, SimpleNamespace)
                for field in ("qpos", "qvel", "last_torque"):
                    source = controller.robot.data.ctrl if field == "last_torque" else getattr(controller.robot.data, field)
                    self.assertFalse(np.shares_memory(state[field], source))
                for field in before["arrays"]:
                    if field != "initial_ee":  # Unused by either law.
                        self.assertFalse(np.shares_memory(getattr(shadow, field), getattr(controller, field)), field)
                originals[name](shadow, state)
            return run

        with mock.patch.object(DryController, "_impedance_step", counted("impedance")), \
                mock.patch.object(DryController, "_osc_step", counted("osc")):
            report = warmup.warmup_control_math(controller)
        self.assertEqual(counts, {"impedance": 100, "osc": 100})
        self.assertEqual(report["iterations"], 100)
        self.assertEqual(report["dummy_torque_writes"], 200)
        self.assertGreater(report["elapsed_s"], 0)
        self.assertIn("no robot communication or IPC", report["timing_scope"])
        self.assertEqual(set(report["timing_ms"]), {"mujoco", "impedance", "osc"})
        for timing in report["timing_ms"].values():
            self.assertTrue(np.isfinite(list(timing.values())).all())
            self.assertGreaterEqual(timing["max"], timing["first"])
            self.assertGreaterEqual(timing["max"], timing["median"])
            self.assertGreaterEqual(timing["median"], 0.)
        self.assert_unchanged(controller, before)

    def test_invalid_dummy_torques_abort_without_mutating_original_controller(self):
        for torque in (np.full(7, np.nan), np.zeros(6)):
            controller = self.controller()
            before = self.snapshot(controller)

            def bad_osc(shadow, state):
                shadow.kp[0] = -5.
                shadow.q_desired[0] = 42.
                shadow.ee_desired[0, 3] = 9.
                state["qpos"][0] = 37.
                shadow.robot.step(torque)

            with self.subTest(shape=torque.shape), mock.patch.object(DryController, "_osc_step", bad_osc):
                with self.assertRaisesRegex(ValueError, "invalid torques"):
                    warmup.warmup_control_math(controller, iterations=2)
            self.assert_unchanged(controller, before)

    def test_real_control_law_nonfinite_output_aborts_before_transport(self):
        controller = self.controller()
        controller.kp[0] = np.nan
        before = self.snapshot(controller)
        with self.assertRaisesRegex(ValueError, "invalid torques"):
            warmup.warmup_control_math(controller, iterations=1)
        self.assert_unchanged(controller, before)

    def test_both_inverse_branches_are_warmed_for_six_and_seven_dimensions(self):
        controller = self.controller()
        with mock.patch.object(np.linalg, "inv", wraps=np.linalg.inv) as inverse, \
                mock.patch.object(np.linalg, "pinv", wraps=np.linalg.pinv) as pseudo:
            warmup.warmup_control_math(controller, iterations=1)
        for operation in (inverse, pseudo):
            shapes = {call.args[0].shape for call in operation.call_args_list}
            self.assertTrue({(6, 6), (7, 7)}.issubset(shapes))

    def test_invalid_local_state_and_iteration_count_fail_without_backend_access(self):
        for iterations in (0, -1, 1.5):
            controller = self.controller()
            before = self.snapshot(controller)
            with self.subTest(iterations=iterations), self.assertRaisesRegex(ValueError, "positive integer"):
                warmup.warmup_control_math(controller, iterations=iterations)
            self.assert_unchanged(controller, before)
        controller = self.controller()
        controller.robot.data.qvel[0] = np.nan
        before = self.snapshot(controller)
        with self.assertRaisesRegex(ValueError, "finite local robot state"):
            warmup.warmup_control_math(controller, iterations=1)
        self.assert_unchanged(controller, before)


if __name__ == "__main__":
    unittest.main()
