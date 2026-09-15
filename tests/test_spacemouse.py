"""Public HID queue draining and legacy mappings using only fake devices."""

from collections import deque
import builtins
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np


path = Path(__file__).resolve().parents[1] / "aiofranka/utils/spacemouse.py"
spec = importlib.util.spec_from_file_location("spacemouse_test_utility", path)
utility = importlib.util.module_from_spec(spec)
spec.loader.exec_module(utility)


class FakeDevice:
    """One public read parses one report into the same mutable state object."""

    def __init__(self, reports):
        self.queue = deque(reports)
        self.state = SimpleNamespace(t=-1., x=0., y=0., z=0., roll=0., pitch=0.,
                                     yaw=0., buttons=[0, 0])
        self.calls = 0
        self.processed = []

    def read(self):
        self.calls += 1
        if self.queue:
            report = self.queue.popleft()
            self.state.t += 1
            for key, value in report.items():
                if key == "buttons":
                    self.state.buttons[:] = value
                else:
                    setattr(self.state, key, value)
            self.processed.append((self.state.t, list(self.state.buttons)))
        return self.state


class SpaceMouseDrainTest(unittest.TestCase):
    def test_final_zero_report_is_processed_without_private_reads(self):
        device = FakeDevice([{"x": .8, "y": -.4}, {"x": 0., "y": 0.}])
        state = utility.read_latest_state(device)
        np.testing.assert_array_equal([state.x, state.y, state.z], np.zeros(3))
        self.assertEqual(device.calls, 3)
        self.assertFalse(device.queue)

    def test_mutable_timestamp_split_axes_and_intermediate_buttons_are_processed(self):
        device = FakeDevice([{"x": .7, "y": -.2, "z": .1}, {"buttons": [1, 0]},
                             {"roll": .2, "pitch": -.3, "yaw": .4},
                             {"buttons": [0, 1]}, {"x": -.5}])
        state = utility.read_latest_state(device)
        self.assertEqual(state.t, 4.)
        self.assertEqual(device.calls, 6)
        np.testing.assert_allclose([state.x, state.y, state.z, state.roll, state.pitch, state.yaw],
                                   [-.5, -.2, .1, .2, -.3, .4])
        self.assertEqual([buttons for _, buttons in device.processed],
                         [[0, 0], [1, 0], [1, 0], [0, 1], [0, 1]])
        self.assertEqual(state.buttons, [0, 1])
        # A subsequent read mutates both the library state and its button list.
        device.queue.append({"x": 0., "buttons": [0, 0]})
        utility.read_latest_state(device)
        self.assertEqual(state.x, -.5)
        self.assertEqual(state.t, 4.)
        self.assertEqual(state.buttons, [0, 1])

    def test_empty_queue_and_exact_guard_boundary(self):
        state = utility.read_latest_state(FakeDevice([]))
        self.assertEqual(state.t, -1.)
        device = FakeDevice([{"x": .2}] * 7)
        self.assertEqual(utility.read_latest_state(device, max_reads=8).x, .2)
        self.assertEqual(device.calls, 8)
        device = FakeDevice([{"x": .2}] * 8)
        with self.assertRaisesRegex(RuntimeError, "within 8 reads"):
            utility.read_latest_state(device, max_reads=8)
        self.assertEqual(device.calls, 8)

    def test_nonfinite_input_is_rejected(self):
        for report in ({"x": float("nan")}, {"yaw": float("inf")}, {"t": float("nan")}):
            with self.subTest(report=report):
                with self.assertRaisesRegex(ValueError, "nonfinite"):
                    utility.read_latest_state(FakeDevice([report]))

    def test_existing_raw_scale_fields_rotation_mapping_and_yaw_only_are_preserved(self):
        reports = [{"x": .7, "y": -.2, "z": .1, "roll": .2, "pitch": -.3,
                    "yaw": .4, "buttons": [1, 0]}]
        for yaw_only, rotation in ((False, [.3, .2, -.4]), (True, [0., 0., -.4])):
            context = mock.MagicMock()
            context.__enter__.return_value = FakeDevice(reports)
            with self.subTest(yaw_only=yaw_only), \
                    mock.patch.object(utility, "open_spacemouse", return_value=context):
                mouse = utility.SpaceMouse(yaw_only=yaw_only)
                translation, actual_rotation, buttons = mouse.read()
            np.testing.assert_allclose(translation, [.7, -.2, .1])
            np.testing.assert_allclose(actual_rotation, rotation)
            self.assertEqual(buttons, [1, 0])
            self.assertEqual((mouse.translation_scale, mouse.translation_clip,
                              mouse.rotation_scale, mouse.rotation_clip), (.006, .006, .8, .8))

    def test_open_preserves_legacy_convention_with_older_version_fallback(self):
        for has_convention in (False, True):
            package = SimpleNamespace(open=mock.Mock())
            if has_convention:
                package.AxisConvention = SimpleNamespace(LEGACY=object())
            with self.subTest(has_convention=has_convention), \
                    mock.patch.dict(sys.modules, {"pyspacemouse": package}):
                self.assertIs(utility.open_spacemouse(), package.open.return_value)
            expected = {"nonblocking": True}
            if has_convention:
                expected["axis_convention"] = package.AxisConvention.LEGACY
            package.open.assert_called_once_with(**expected)

    def test_importing_utility_and_tracking_needs_no_robot_or_hid_backend(self):
        original_import = builtins.__import__

        def guarded_import(name, *args, **kwargs):
            if name.split(".")[0] in {"aiofranka", "pyspacemouse", "easyhid", "pylibfranka"}:
                raise AssertionError(f"Unexpected hardware backend import: {name}")
            return original_import(name, *args, **kwargs)

        for source in (path, path.parents[2] / "examples/t_pushing_tracking.py"):
            with self.subTest(source=source), mock.patch("builtins.__import__", guarded_import):
                isolated_spec = importlib.util.spec_from_file_location("_mouse_import_test", source)
                isolated_spec.loader.exec_module(importlib.util.module_from_spec(isolated_spec))


if __name__ == "__main__":
    unittest.main()
