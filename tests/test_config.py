import tempfile
import unittest
from pathlib import Path

import mujoco
import numpy as np
import yaml

from aiofranka.config import HOME, load_config, same_controller, save_sim, sim_entry
from aiofranka.controller import FrankaController
from aiofranka.payload import MODEL_PATH
from aiofranka.tools import NO_END_EFFECTOR, Tool
from test_payload import simulated_robot

OSC = """\
# my policy's controller
mode: osc
tool: gripper
ee_kp: [300, 300, 300, 30, 30, 30]
ee_kd: 20            # one value for all axes
null_kp: 9
frequency: 50
"""


def tool(name, id="abc"):
    return Tool(name, 0.5, np.zeros(3), np.eye(3) * 1e-3, np.zeros(3), np.zeros(3), id=id)


class LoadConfigTest(unittest.TestCase):
    def test_reads_osc_with_defaults(self):
        config = load_config(yaml.safe_load(OSC))

        self.assertEqual(config["mode"], "osc")
        self.assertEqual(config["tool"], "gripper")
        self.assertEqual(config["frequency"], 50.0)
        np.testing.assert_allclose(config["ee_kp"], [300, 300, 300, 30, 30, 30])
        np.testing.assert_allclose(config["ee_kd"], np.full(6, 20.0))
        np.testing.assert_allclose(config["null_kp"], np.full(7, 9.0))
        np.testing.assert_allclose(config["null_kd"], np.ones(7))
        np.testing.assert_allclose(config["null_target"], HOME)
        np.testing.assert_allclose(config["tcp"], np.eye(4))
        self.assertEqual(config["sim"], [])

    def test_reads_impedance_and_tcp_forms(self):
        config = load_config({"mode": "impedance", "kp": 64, "kd": [16] * 7, "frequency": 25})
        np.testing.assert_allclose(config["kp"], np.full(7, 64.0))
        self.assertNotIn("tool", config)

        osc = yaml.safe_load(OSC)
        for tcp in ([0, 0, 0.1], np.eye(4).flatten().tolist(), np.eye(4).tolist()):
            with self.subTest(tcp=tcp):
                self.assertEqual(load_config(dict(osc, tcp=tcp))["tcp"].shape, (4, 4))

    def test_rejects_wrong_configurations(self):
        osc = yaml.safe_load(OSC)
        wrong = [
            dict(osc, mode="pid"),
            dict(osc, ee_kpp=1),               # typo
            dict(osc, kp=1),                   # impedance key in osc
            {k: v for k, v in osc.items() if k != "ee_kp"},
            {k: v for k, v in osc.items() if k != "frequency"},
            dict(osc, frequency=0),
            dict(osc, ee_kp=[1, 2]),
            dict(osc, null_kd=-1),
            dict(osc, tcp=[0, 0]),
            dict(osc, null_target=[0] * 6),
        ]
        for config in wrong:
            with self.subTest(config=config), self.assertRaises(ValueError):
                load_config(config)

    def test_rejects_values_that_are_not_finite_or_missing(self):
        osc = yaml.safe_load(OSC)
        for changes in ({"ee_kd": None}, {"ee_kp": float("nan")}, {"null_kp": float("inf")},
                        {"frequency": float("inf")}, {"frequency": None}, {"tcp": [0, 0, float("nan")]},
                        {"null_target": [0, 0, None, 0, 0, 0, 0]}, {"tool": None}, {"tool": False},
                        {"tool": ""}, {"sim": {"physics_dt": 0.005}}, {"sim": [{"kp": 1}]}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                load_config(dict(osc, **changes))
        with self.assertRaises(ValueError):
            load_config(["mode", "osc"])
        path = Path(tempfile.mkdtemp()) / "bad.yaml"
        path.write_text("mode: osc\nee_kp: [1, 2\n")
        with self.assertRaises(ValueError):
            load_config(path)

    def test_normalizes_none_and_copies_the_tcp(self):
        tcp = np.eye(4)
        config = load_config(dict(yaml.safe_load(OSC), tool="None", tcp=tcp))
        self.assertEqual(config["tool"], "none")
        tcp[2, 3] = 0.5
        self.assertEqual(config["tcp"][2, 3], 0.0)

    def test_same_controller(self):
        osc = yaml.safe_load(OSC)
        self.assertTrue(same_controller(osc, dict(osc, name="other", sim=[{"physics_dt": 0.002}])))
        self.assertFalse(same_controller(osc, dict(osc, ee_kd=21)))
        self.assertFalse(same_controller(osc, dict(osc, tool="none")))
        self.assertFalse(same_controller(osc, dict(osc, frequency=25)))


class SaveSimTest(unittest.TestCase):
    def setUp(self):
        self.path = Path(tempfile.mkdtemp()) / "osc.yaml"
        self.path.write_text(OSC)

    def test_keeps_the_file_and_replaces_the_same_physics_dt(self):
        save_sim(self.path, {"physics_dt": 0.005, "ee_kp": np.arange(6) * 1.23456789, "armature": [0.1] * 7})
        save_sim(self.path, {"physics_dt": 0.002, "ee_kp": [1.0] * 6})
        save_sim(self.path, {"physics_dt": 0.005, "ee_kp": [2.0] * 6, "fit": {"holdout": "right"}})

        text = self.path.read_text()
        self.assertTrue(text.startswith(OSC))  # comments and layout kept
        config = load_config(self.path)
        self.assertTrue(same_controller(config, yaml.safe_load(OSC)))
        self.assertEqual([s["physics_dt"] for s in config["sim"]], [0.002, 0.005])
        entry = sim_entry(config, 0.005)
        np.testing.assert_allclose(entry["ee_kp"], np.full(6, 2.0))
        self.assertEqual(entry["fit"], {"holdout": "right"})
        self.assertNotIn("armature", entry)  # replaced, not merged
        with self.assertRaises(KeyError):
            sim_entry(config, 0.001)

    def test_keeps_comments_in_and_after_the_sim_section(self):
        self.path.write_text(OSC + "sim:\n# first fit\n- physics_dt: 0.005\n  ee_kp: [1, 1, 1, 1, 1, 1]\n"
                             "# after the fits\n")
        save_sim(self.path, {"physics_dt": 0.002, "ee_kp": [3.0] * 6})

        config = load_config(self.path)
        self.assertEqual([s["physics_dt"] for s in config["sim"]], [0.002, 0.005])
        self.assertIn("# after the fits", self.path.read_text())
        self.assertTrue(self.path.read_text().startswith(OSC))

    def test_refuses_a_changed_controller_and_leaves_the_file(self):
        recorded = yaml.safe_load(OSC)
        self.path.write_text(OSC.replace("ee_kd: 20", "ee_kd: 8"))
        before = self.path.read_text()
        with self.assertRaises(ValueError):
            save_sim(self.path, {"physics_dt": 0.002, "ee_kp": [3.0] * 6}, controller=recorded)
        self.assertEqual(self.path.read_text(), before)

    def test_finds_a_float32_physics_dt(self):
        save_sim(self.path, {"physics_dt": 0.002, "ee_kp": [3.0] * 6})
        self.assertEqual(sim_entry(self.path, np.float32(0.002))["physics_dt"], 0.002)

    def test_moves_a_sim_section_in_the_middle_to_the_end(self):
        self.path.write_text("sim:\n- physics_dt: 0.01\n  ee_kp: [1, 1, 1, 1, 1, 1]\n" + OSC)
        save_sim(self.path, {"physics_dt": 0.002, "ee_kp": [3.0] * 6})

        config = load_config(self.path)
        self.assertEqual([s["physics_dt"] for s in config["sim"]], [0.002, 0.01])
        self.assertTrue(same_controller(config, yaml.safe_load(OSC)))
        self.assertTrue(self.path.read_text().startswith("# my policy's controller"))


class ActivateTest(unittest.TestCase):
    def setUp(self):
        self.robot = simulated_robot(mujoco.MjModel.from_xml_path(str(MODEL_PATH)))
        self.controller = FrankaController(self.robot)

    def test_applies_osc(self):
        config = load_config(dict(yaml.safe_load(OSC), tcp=[0, 0, 0.1], null_target=[0.1] * 7))
        self.controller.activate(config)

        c = self.controller
        self.assertEqual(c.type, "osc")
        np.testing.assert_allclose(c.ee_kp, config["ee_kp"])
        np.testing.assert_allclose(c.ee_kd, config["ee_kd"])
        np.testing.assert_allclose(c.null_kp, config["null_kp"])
        np.testing.assert_allclose(c.null_kd, config["null_kd"])
        np.testing.assert_allclose(c.initial_qpos, np.full(7, 0.1))
        np.testing.assert_allclose(c.control_transform[:3, 3], [0, 0, 0.1])
        # Holds the TCP where it is.
        np.testing.assert_allclose(c.ee_desired, self.robot._ee() @ c.control_transform)
        self.assertEqual(c._update_freq, 50.0)

    def test_applies_impedance_from_a_file(self):
        path = Path(tempfile.mkdtemp()) / "joint.yaml"
        path.write_text("mode: impedance\nkp: 64\nkd: 16\nfrequency: 25\n")
        self.controller.activate(path)

        c = self.controller
        self.assertEqual(c.type, "impedance")
        np.testing.assert_allclose(c.kp, np.full(7, 64.0))
        np.testing.assert_allclose(c.kd, np.full(7, 16.0))
        np.testing.assert_allclose(c.q_desired, self.robot.data.qpos)
        self.assertEqual(c._update_freq, 25.0)

    def test_checks_the_tool_on_the_robot(self):
        osc = yaml.safe_load(OSC)
        self.robot.real = True

        self.robot.tool = tool("gripper")
        self.controller.activate(osc)  # the active profile

        self.robot.tool = tool("No End Effector", NO_END_EFFECTOR)
        self.controller.activate(dict(osc, tool="none"))
        with self.assertRaisesRegex(RuntimeError, "gripper"):
            self.controller.activate(osc)
        self.controller.activate(osc, check_tool=False)
        self.controller.activate({k: v for k, v in osc.items() if k != "tool"})  # no tool, no check

        self.robot.tool, self.robot.tool_error = None, "Desk did not answer"
        with self.assertRaisesRegex(RuntimeError, "Desk did not answer"):
            self.controller.activate(osc)

    def test_move_keeps_its_pace_after_a_fast_configuration(self):
        import asyncio
        import time
        controller = self.controller

        async def run():
            await controller.start()
            try:
                controller.activate({"mode": "impedance", "kp": 80, "kd": 8, "frequency": 1000})
                target = np.array(controller.robot.data.qpos) + [0.15, 0, 0, 0, 0, 0, 0]
                t0 = time.perf_counter()
                await controller.move(target)
                return time.perf_counter() - t0
            finally:
                await controller.stop()

        duration = asyncio.run(run())
        # Ruckig plans about 1.5 s for this move; at 1000 Hz it would play in about 0.03 s.
        self.assertGreater(duration, 1.0)
        self.assertEqual(controller._update_freq, 1000.0)

    def test_does_not_check_the_tool_in_mujoco(self):
        self.robot.tool = None
        self.controller.activate(yaml.safe_load(OSC))
        self.assertEqual(self.controller.type, "osc")


if __name__ == "__main__":
    unittest.main()
