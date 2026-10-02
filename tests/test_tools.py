import copy
import json
import unittest
from types import SimpleNamespace
from unittest import mock

import mujoco
import numpy as np

from aiofranka import server, tools
from aiofranka.payload import MODEL_PATH
from aiofranka.robot import RobotInterface


HOME = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])


def part(mass, com, inertia):
    return {"mass": mass, "com": np.array(com, dtype=float), "inertia": np.diag(inertia)}


# A Franka Hand configured in Desk, and a camera mount set as the load.
HAND = part(0.73, [-0.01, 0.0, 0.03], [1e-3, 2.5e-3, 1.7e-3])
CAMERA = part(0.25, [0.04, 0.02, 0.02], [2e-4, 2e-4, 1e-4])


def combine(*parts):
    """Mass, center of mass and inertia about it of rigidly attached parts."""
    mass = sum(p["mass"] for p in parts)
    if mass == 0:
        return part(0.0, [0, 0, 0], [0, 0, 0])
    com = sum(p["mass"] * p["com"] for p in parts) / mass
    inertia = sum(
        p["inertia"] + p["mass"] * ((p["com"] - com) @ (p["com"] - com) * np.eye(3)
                                    - np.outer(p["com"] - com, p["com"] - com))
        for p in parts
    )
    return {"mass": mass, "com": com, "inertia": inertia}


class FakeFranka:
    """pylibfranka.Robot with an end effector configured in Desk and a load."""

    def __init__(self, ee, lag=0):
        self.ee = ee
        self.load = part(0.0, [0, 0, 0], [0, 0, 0])
        # Number of reads after set_load() that still show the previous load, as
        # the robot state takes a few cycles to show it.
        self.lag = lag
        self.pending = []

    def set_load(self, mass, com, inertia):
        load = {"mass": mass, "com": np.array(com),
                "inertia": np.array(inertia).reshape(3, 3, order="F")}
        self.pending = [self.load] * self.lag + [load]
        self.load = self.pending.pop(0)

    def read_once(self):
        if self.pending:
            self.load = self.pending.pop(0)
        def fields(prefix, mass_name, p):
            return {
                mass_name: p["mass"],
                f"F_x_C{prefix}": p["com"].tolist(),
                f"I_{prefix}": p["inertia"].flatten(order="F").tolist(),
            }

        return SimpleNamespace(
            q=HOME.tolist(), dq=[0.0] * 7, tau_J_d=[0.0] * 7,
            **fields("ee", "m_ee", self.ee),
            **fields("load", "m_load", self.load),
            **fields("total", "m_total", combine(self.ee, self.load)),
        )


def real_robot(ee, lag=0):
    """RobotInterface as on a real robot, with FakeFranka, as after connecting."""
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    robot = RobotInterface.__new__(RobotInterface)
    robot.real = True
    robot.ip = "172.16.0.2"
    robot.torque_controller = None
    robot.robot = FakeFranka(ee, lag)
    robot.model = model
    robot.data = mujoco.MjData(model)
    robot.site_name = "attachment_site"
    robot.site_id = model.site(robot.site_name).id
    robot.load = part(0.0, [0, 0, 0], [0, 0, 0])
    robot.payload = dict(robot.load)
    last_link = model.body("fr3_link7").id
    robot._last_link_inertial = (
        float(model.body_mass[last_link]),
        model.body_ipos[last_link].copy(),
        model.body_iquat[last_link].copy(),
        model.body_inertia[last_link].copy(),
    )
    robot.sync_mj()
    robot.sync_payload()
    return robot


def mass_matrix_with(*parts):
    """Mass matrix at HOME of the arm with the parts as bodies on the flange."""
    spec = mujoco.MjSpec.from_file(str(MODEL_PATH))
    for i, p in enumerate(parts):
        body = spec.body("fr3_link7").add_body(name=f"part{i}", pos=[0, 0, 0.107])
        body.mass = p["mass"]
        body.ipos = p["com"]
        body.fullinertia = [p["inertia"][0, 0], p["inertia"][1, 1], p["inertia"][2, 2],
                            p["inertia"][0, 1], p["inertia"][0, 2], p["inertia"][1, 2]]
        body.explicitinertial = True
    model = spec.compile()
    data = mujoco.MjData(model)
    data.qpos[:7] = HOME
    mujoco.mj_forward(model, data)
    mm = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, mm)
    return mm


class RobotPayloadTest(unittest.TestCase):
    def test_mujoco_includes_the_desk_end_effector(self):
        robot = real_robot(HAND)

        self.assertAlmostEqual(robot.payload["mass"], HAND["mass"])
        np.testing.assert_allclose(robot._mass_matrix(), mass_matrix_with(HAND), atol=1e-9)

    def test_mujoco_includes_the_desk_end_effector_and_the_load(self):
        robot = real_robot(HAND)
        robot.set_load(CAMERA["mass"], CAMERA["com"], CAMERA["inertia"])

        self.assertAlmostEqual(robot.load["mass"], CAMERA["mass"])
        self.assertAlmostEqual(robot.payload["mass"], HAND["mass"] + CAMERA["mass"])
        np.testing.assert_allclose(robot._mass_matrix(), mass_matrix_with(HAND, CAMERA), atol=1e-9)

    def test_waits_for_the_robot_state_to_show_the_load(self):
        robot = real_robot(HAND, lag=3)
        robot.set_load(CAMERA["mass"], CAMERA["com"], CAMERA["inertia"])

        np.testing.assert_allclose(robot._mass_matrix(), mass_matrix_with(HAND, CAMERA), atol=1e-9)

    def test_removing_the_load_restores_the_end_effector(self):
        robot = real_robot(HAND)
        robot.set_load(CAMERA["mass"], CAMERA["com"], CAMERA["inertia"])
        robot.set_load(0.0)

        np.testing.assert_allclose(robot._mass_matrix(), mass_matrix_with(HAND), atol=1e-9)


def profile(id, name, mass, com, inertia, translation, yaw, active=False):
    """An end-effector profile as Desk's admin API returns it."""
    x11, x12, x13, x22, x23, x33 = inertia
    return {
        "id": id, "name": name, "deviceId": tools.GENERIC_DEVICE, "active": active, "modelUri": None,
        "inertial": {
            "mass": mass,
            "centerOfMass": dict(zip("xyz", com)),
            "inertia": {"x11": x11, "x12": x12, "x13": x13, "x22": x22, "x23": x23, "x33": x33},
        },
        "tcp": {
            "translation": dict(zip("xyz", translation)),
            "rotation": {"roll": 0, "pitch": 0, "yaw": yaw},
        },
    }


# Profiles on the robot, from GET /admin/api/end-effector/profiles.
PROFILES = [
    profile("17f0b710", "AgileX Gripper", 0.101, [0, 0, 0], [0.00128, 0, 0, 0.00101, 0, 0.00047],
            [0, 0, 0.12], -2.356194490192345, active=True),
    profile("a4cfbbc5", "Wrist", 1.512, [-0.00066, 0.00359, 0.11561],
            [0.007525, 0.000127, 6.2e-05, 0.007472, 0.000187, 0.000982], [0, 0, 0.3], -2.356194490192345),
    profile(tools.NO_END_EFFECTOR, "No End Effector", 0.0, [0, 0, 0], [0] * 6, [0, 0, 0], 0),
]


class FakeResponse:
    def __init__(self, status_code, body=None):
        self.status_code = status_code
        self.body = body
        self.text = json.dumps(body) if not isinstance(body, str) else body

    def json(self):
        return self.body


class FakeAdmin:
    """_DeskClient for Desk's end-effector profiles, which wants the control token to activate one."""

    profiles = []
    requests = []

    def __init__(self, *args, **kwargs):
        pass

    def login(self):
        pass

    def _req(self, method, path, json=None, headers=None, **kwargs):
        FakeAdmin.requests.append((method, path, json, headers))
        base = "/admin/api/end-effector/profiles"
        by_id = {p["id"]: p for p in self.profiles}
        if (method, path) == ("GET", base):
            return FakeResponse(200, copy.deepcopy(self.profiles))
        if (method, path) == ("GET", f"{base}/active"):
            return FakeResponse(200, next(p for p in self.profiles if p["active"]))
        if (method, path) == ("POST", base):
            created = dict(copy.deepcopy(json), id=f"new{len(self.profiles)}", active=False)
            self.profiles.append(created)
            return FakeResponse(200, {"id": created["id"]})
        if (method, path) == ("PUT", f"{base}/active"):
            if (headers or {}).get("X-Control-Token") != "token":
                return FakeResponse(423, {"code": "Locked", "message": "Requires the control token"})
            for p in self.profiles:
                p["active"] = p["id"] == json
            return FakeResponse(204, "")
        profile_id = path[len(base) + 1:]
        if method == "PUT" and profile_id in by_id:
            by_id[profile_id].update(copy.deepcopy(json))
            return FakeResponse(204, "")
        if method == "DELETE" and profile_id in by_id:
            self.profiles.remove(by_id[profile_id])
            return FakeResponse(204, "")
        return FakeResponse(404, "Not found")


class FakeSpoc:
    """_DeskClientV2 that gets the control token."""

    released = 0

    def __init__(self, *args, **kwargs):
        self._token = None
        self._token_id = None

    def take_token(self, timeout=None):
        self._token = "token"

    def validate_token(self):
        return True

    def release_token(self, best_effort=False):
        FakeSpoc.released += 1

    def _with_retry(self, fn, **kwargs):
        return fn()


class DeskProfilesTest(unittest.TestCase):
    def setUp(self):
        FakeAdmin.profiles = copy.deepcopy(PROFILES)
        FakeAdmin.requests = []
        FakeSpoc.released = 0
        for name, value in (("_DeskClient", FakeAdmin), ("_DeskClientV2", FakeSpoc),
                            ("_load_token_state", lambda ip: (None, None)),
                            ("_resolve_from_config", lambda ip, u, p: ("ip", "u", "p"))):
            patcher = mock.patch.object(server, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)

    def profile(self, name):
        return next(p for p in FakeAdmin.profiles if p["name"] == name)

    def test_lists_the_profiles(self):
        listed = tools.list_tools()

        self.assertEqual(list(listed), ["AgileX Gripper", "Wrist", "No End Effector"])
        wrist = listed["Wrist"]
        self.assertFalse(wrist.active)
        self.assertTrue(listed["AgileX Gripper"].active)
        np.testing.assert_allclose(wrist.com, [-0.00066, 0.00359, 0.11561])
        np.testing.assert_allclose(wrist.inertia, wrist.inertia.T)
        self.assertEqual(wrist.inertia[0, 1], 0.000127)
        np.testing.assert_allclose(wrist.translation, [0, 0, 0.3])
        self.assertAlmostEqual(wrist.rotation[2], -2.356194490192345)

    def test_saves_a_new_tool_as_a_generic_device(self):
        tool = tools.save_tool("Camera", 0.25, [0.04, 0.02, 0.02])

        created = self.profile("Camera")
        self.assertEqual(created["id"], tool.id)
        self.assertEqual(created["deviceId"], tools.GENERIC_DEVICE)
        self.assertFalse(created["active"])
        self.assertEqual(created["inertial"]["mass"], 0.25)
        # The robot rejects a mass without inertia, so a new tool gets a sphere's.
        sphere = 0.4 * 0.25 * tools.DEFAULT_RADIUS ** 2
        self.assertEqual(created["inertial"]["inertia"]["x11"], sphere)
        self.assertEqual(created["tcp"]["translation"], {"x": 0.0, "y": 0.0, "z": 0.0})

    def test_updates_an_existing_tool_keeping_its_tcp_and_inertia(self):
        tools.save_tool("AgileX Gripper", 0.62, [0.001, -0.002, 0.045])

        updated = self.profile("AgileX Gripper")
        self.assertEqual(len(FakeAdmin.profiles), 3)
        self.assertEqual(updated["inertial"]["mass"], 0.62)
        self.assertEqual(updated["inertial"]["centerOfMass"], {"x": 0.001, "y": -0.002, "z": 0.045})
        self.assertEqual(updated["inertial"]["inertia"]["x11"], 0.00128)
        self.assertEqual(updated["tcp"]["translation"]["z"], 0.12)
        self.assertAlmostEqual(updated["tcp"]["rotation"]["yaw"], -2.356194490192345)
        self.assertTrue(updated["active"])

    def test_rejects_tools_desk_or_the_robot_would_reject(self):
        for kwargs in (dict(mass=3.5, com=[0, 0, 0]), dict(mass=1.0, com=[0, 0, 0.5]),
                       dict(mass=1.0, com=[0, 0, 0], inertia=[0, 0, 0])):
            with self.subTest(**kwargs), self.assertRaises(ValueError):
                tools.save_tool("Camera", **kwargs)
        with self.assertRaises(ValueError):
            tools.save_tool("No End Effector", 0.1, [0, 0, 0])
        self.assertEqual(len(FakeAdmin.profiles), 3)

    def test_loads_a_tool_with_the_control_token(self):
        tool = tools.load_tool("Wrist")

        self.assertTrue(tool.active)
        self.assertTrue(self.profile("Wrist")["active"])
        self.assertFalse(self.profile("AgileX Gripper")["active"])
        activations = [r for r in FakeAdmin.requests if r[:2] == ("PUT", "/admin/api/end-effector/profiles/active")]
        self.assertEqual(activations[-1][2], "a4cfbbc5")
        self.assertEqual(activations[-1][3], {"X-Control-Token": "token"})
        self.assertEqual(FakeSpoc.released, 1)

    def test_unloads_by_activating_the_built_in_profile(self):
        tool = tools.unload_tool()

        self.assertEqual(tool.id, tools.NO_END_EFFECTOR)
        self.assertTrue(self.profile("No End Effector")["active"])
        self.assertFalse(self.profile("AgileX Gripper")["active"])

    def test_removes_only_inactive_custom_tools(self):
        with self.assertRaises(ValueError):
            tools.remove_tool("AgileX Gripper")
        with self.assertRaises(ValueError):
            tools.remove_tool("No End Effector")
        tools.remove_tool("Wrist")

        self.assertEqual([p["name"] for p in FakeAdmin.profiles], ["AgileX Gripper", "No End Effector"])

    def test_unknown_tool(self):
        with self.assertRaises(KeyError):
            tools.load_tool("Pocky")
        with self.assertRaises(KeyError):
            tools.remove_tool("Pocky")


class FakeDesk:
    """_DeskClientV2 with an end effector configuration."""

    end_effector = None
    patches = []

    def __init__(self, *args, **kwargs):
        self._token = None

    def take_token(self, timeout=None):
        self._token = "token"

    def _with_retry(self, fn, **kwargs):
        return fn()

    def release_token(self, best_effort=False):
        pass

    def get_configuration(self):
        if self.end_effector is None:
            return {}
        return {"endEffectorConfiguration": {"endEffector": self.end_effector}}

    def set_configuration(self, patch):
        FakeDesk.patches.append(patch)


class SetConfigurationTest(unittest.TestCase):
    def configured_name(self, current, **kwargs):
        FakeDesk.end_effector = current
        FakeDesk.patches = []
        with mock.patch.object(server, "_DeskClientV2", FakeDesk), \
                mock.patch.object(server, "_load_token_state", return_value=(None, None)), \
                mock.patch.object(server, "_resolve_from_config", return_value=("ip", "u", "p")):
            server.set_configuration(**kwargs)
        return FakeDesk.patches[0]["endEffectorConfiguration"]["endEffector"]["name"]

    def test_keeps_the_configured_end_effector(self):
        self.assertEqual(self.configured_name({"name": "FrankaHand", "params": {}}, mass=0.8), "FrankaHand")

    def test_needs_an_end_effector_for_a_mass(self):
        self.assertEqual(self.configured_name({"name": "None", "params": {}}, mass=0.8), "Other")
        self.assertEqual(self.configured_name({"name": "None", "params": {}}, mass=0.0), "None")
        self.assertEqual(self.configured_name(None, mass=0.8), "Other")


class ConfirmToolTest(unittest.TestCase):
    """The CLI's question before saving an identified tool."""

    def confirm(self, *answers):
        from aiofranka.__main__ import _confirm_tool

        with mock.patch("builtins.input", side_effect=answers), mock.patch("builtins.print"):
            return _confirm_tool("hand", 0.712, np.array([0.008, 0.001, 0.032]), activate=True)

    def test_saves_the_estimate(self):
        mass, com = self.confirm("")

        self.assertEqual(mass, 0.712)
        np.testing.assert_allclose(com, [0.008, 0.001, 0.032])

    def test_edits_mass_in_grams_and_com_in_millimeters(self):
        mass, com = self.confirm("e", "730", "-10", "", "", "y")

        self.assertAlmostEqual(mass, 0.73)
        np.testing.assert_allclose(com, [-0.010, 0.001, 0.032])

    def test_asks_again_after_a_typo_and_cancels(self):
        self.assertIsNone(self.confirm("e", "heavy", "730", "", "", "", "x", "n"))


if __name__ == "__main__":
    unittest.main()
