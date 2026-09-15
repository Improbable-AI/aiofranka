"""Error-message regressions without importing robot or communication backends."""

import ast
from pathlib import Path
import re
import unittest


def load_error_class():
    source = Path(__file__).resolve().parents[1] / "aiofranka/remote.py"
    tree = ast.parse(source.read_text(), filename=str(source))
    names = {"_BOLD", "_DIM", "_RED", "_YELLOW", "_RST"}
    nodes = [node for node in tree.body
             if (isinstance(node, ast.ClassDef) and node.name == "ServerDiedError")
             or (isinstance(node, ast.Assign)
                 and all(isinstance(target, ast.Name) and target.id in names
                         for target in node.targets))]
    namespace = {}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), "exec"), namespace)
    return namespace["ServerDiedError"]


ServerDiedError = load_error_class()


def plain_message(error):
    return re.sub(r"\x1b\[[0-9;]*m", "", str(error))


class RemoteErrorMessageTest(unittest.TestCase):
    def test_communication_violation_explains_timing_and_requires_checks_before_restart(self):
        for raw in (
            "libfranka: Move command aborted: motion aborted by reflex! "
            "[communication_constraints_violation]",
            "communication_constraints_violation",
            "COMMUNICATION_CONSTRAINTS_VIOLATION",
        ):
            with self.subTest(raw=raw):
                error = ServerDiedError(raw)
                self.assertIsInstance(error, RuntimeError)
                self.assertEqual(error.server_error, raw)
                message = plain_message(error)
                self.assertIn(raw, message)
                self.assertIn("1 kHz", message)
                self.assertIn("timing requirements", message)
                self.assertIn("host scheduling/load", message)
                self.assertIn("network latency/packet loss", message)
                self.assertIn("Check real-time scheduling, host load, and the robot network connection", message)
                self.assertIn("before restarting the controller", message)
                self.assertNotIn("joint/velocity/torque", message)
                self.assertNotIn("gravcomp", message)
                self.assertNotIn("freely move", message)
                self.assertNotIn("auto-recover", message)

    def test_other_reflex_error_retains_existing_message(self):
        raw = "motion aborted by reflex! [joint_velocity_violation]"
        self.assertEqual(plain_message(ServerDiedError(raw)),
                         f"\n  Server died: {raw}\n\n"
                         "  The robot entered Reflex mode (safety stop).\n"
                         "  This usually means the robot hit a joint/velocity/torque limit.\n\n"
                         "  To recover:\n"
                         "    1. Run aiofranka gravcomp to freely move the robot\n"
                         "       to a safe configuration, then Ctrl+C and restart your script.\n"
                         "    2. Or just restart your script (it will auto-recover if possible).\n")

    def test_other_server_error_retains_existing_message(self):
        raw = "Unknown server error"
        self.assertEqual(plain_message(ServerDiedError(raw)),
                         f"\n  Server died: {raw}\n\n"
                         "  To recover:\n"
                         "    1. Run aiofranka gravcomp to freely move the robot\n"
                         "       to a safe configuration, then Ctrl+C and restart your script.\n"
                         "    2. Or just restart your script (it will auto-recover if possible).\n")


if __name__ == "__main__":
    unittest.main()
