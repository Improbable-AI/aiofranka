"""Offline checks for capturing and alternating A/B pushing goals."""

from contextlib import ExitStack, redirect_stderr, redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

try:
    import cv2
    import mujoco
except ImportError:
    available = False
else:
    available = True
    SOURCE = Path(__file__).resolve().parents[1] / "examples/14_collect_t_pushing.py"
    spec = importlib.util.spec_from_file_location("t_pushing_goals_test", SOURCE)
    pushing = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pushing)


@unittest.skipUnless(available, "T-pushing requires OpenCV and MuJoCo")
class TPushingGoalsTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.args = pushing.parser().parse_args([
            "collect", "--condition", "ground", "--no-video",
            "--output", str(self.directory / "run")])
        self.shape = pushing.geometry.TShape(self.args.target_config)
        self.a = np.eye(4)
        self.a[:3, :3] = [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]]
        self.a[:3, 3] = [.5, -.2, .032]
        self.b = self.a.copy()
        self.b[:3, 3] += [0., .4, .005]
        self.goals = {"A": self.a.tolist(), "B": self.b.tolist()}
        self.result = {"T_base_camera": np.eye(4).tolist(),
                       "camera": {"serial": "test", "width": 8, "height": 8},
                       "camera_matrix": [[500., 0., 4.], [0., 500., 4.], [0., 0., 1.]],
                       "dist_coeffs": [0.] * 5}
        self.calibration = self.directory / "calibration.json"
        self.calibration.write_text(json.dumps(self.result))

    def test_source_selection_uses_actual_footprint_orientation_before_xy_distance(self):
        same_center_b = self.a.copy()
        same_center_b[:3, :3] = np.diag([-1., -1., 1.]) @ self.a[:3, :3]
        goals = {"A": self.a.tolist(), "B": same_center_b.tolist()}
        self.assertLess(self.shape.overlap(self.a, same_center_b), .85)
        self.assertEqual(pushing.select_source_goal(self.shape, goals, same_center_b), "B")
        self.assertEqual(pushing.select_source_goal(self.shape, goals, self.a), "A")

    def test_source_selection_breaks_zero_overlap_ties_by_nearest_xy(self):
        for expected, offset in (("A", -.35), ("B", .35)):
            current = self.a.copy() if expected == "A" else self.b.copy()
            current[1, 3] += offset
            for goal in self.goals.values():
                self.assertEqual(self.shape.overlap(goal, current), 0.)
            self.assertEqual(pushing.select_source_goal(self.shape, self.goals, current), expected)

    def test_setting_target_preserves_run_height_and_records_direction_without_mutating_setup(self):
        setup = {**pushing.make_setup(self.args, self.result, self.shape, self.a),
                 "T_base_goals": self.goals, "T_base_goal": self.b.tolist()}
        before = json.dumps(pushing.recording.json_value(setup), sort_keys=True)
        for source, target in (("A", "B"), ("B", "A")):
            episode = pushing.setup_for_goal(setup, target)
            self.assertEqual(episode["source_goal"], source)
            self.assertEqual(episode["target_goal"], target)
            self.assertEqual(episode["direction"], f"{source}_to_{target}")
            np.testing.assert_array_equal(episode["T_base_goal"], self.goals[target])
            self.assertEqual(episode["table_height_m"], 0.)
            self.assertEqual(episode["osc_gains"], setup["osc_gains"])
            self.assertEqual(episode["T_base_goals"], self.goals)
        self.assertEqual(json.dumps(pushing.recording.json_value(setup), sort_keys=True), before)

    def run_collection(self, outcomes, *, fail_finish=False):
        self.args.episodes = len(outcomes)
        events, setups = [], []
        current_poses = []
        for index in range(len(outcomes)):
            current = (self.a if index % 2 == 0 else self.b).copy()
            current[:3, 3] += [.013 * (index + 1), .007, .01]
            current_poses.append(current)
        capture_poses = [self.a, self.b, *current_poses]

        def capture(_):
            index = len([event for event in events if isinstance(event, tuple) and event[0] == "capture"])
            events.append(("capture", index))
            pose = capture_poses[index]
            return pose, {"image": np.full((8, 8, 3), index, dtype=np.uint8),
                          "record": {"T_base_object": pose.tolist(), "sequence": index,
                                     "camera_timestamp_s": 1000. + index}}

        robot = SimpleNamespace(start=lambda: events.append("start"), stop=lambda: events.append("stop"))
        tracker = SimpleNamespace(metadata=self.result["camera"], attach=mock.Mock())
        camera, mouse = mock.MagicMock(), mock.MagicMock()
        camera.__enter__.return_value = tracker
        motion = SimpleNamespace(
            go_home=lambda: events.append("home"), plan_start=lambda xyz: {"start_q": [0.] * 7},
            approach=lambda xyz: events.append(("approach", list(xyz))), configure_osc=lambda: None)
        real_writer = pushing.recording.EpisodeWriter

        def writer_factory(*args, **kwargs):
            writer = real_writer(*args, **kwargs)
            finish = writer.finish

            def finished(status, reason):
                events.append(("finish", status))
                finish(status, reason)
                if fail_finish:
                    raise OSError("synthetic episode finish failure")
            writer.finish = finished
            return writer

        def episode(_robot, _motion, _kin, _mouse, _tracker, _shape, setup, writer, _args):
            index = len(setups)
            setups.append(setup)
            events.append(("episode", setup["direction"]))
            np.testing.assert_array_equal(setup["initial_T_base_object"], current_poses[index])
            saved = json.loads((writer.directory / "setup.json").read_text())
            self.assertEqual(saved["direction"], setup["direction"])
            np.testing.assert_array_equal(saved["T_base_goal"], self.goals[setup["target_goal"]])
            return outcomes[index]

        with ExitStack() as stack:
            stack.enter_context(redirect_stdout(io.StringIO()))
            stack.enter_context(redirect_stderr(io.StringIO()))
            stack.enter_context(mock.patch.dict(sys.modules, {"aiofranka": SimpleNamespace(
                FrankaRemoteController=lambda *_a, **_k: robot)}))
            stack.enter_context(mock.patch.object(pushing.tracking, "CameraTracker", return_value=camera))
            stack.enter_context(mock.patch.object(pushing.tracking, "MouseReader", return_value=mouse))
            stack.enter_context(mock.patch.object(pushing.tracking, "capture_target", side_effect=capture))
            stack.enter_context(mock.patch.object(pushing.motion_code, "Motion", return_value=motion))
            stack.enter_context(mock.patch.object(pushing.recording, "EpisodeWriter", side_effect=writer_factory))
            ready = stack.enter_context(mock.patch.object(pushing, "ask_ready", return_value=True))
            stack.enter_context(mock.patch.object(pushing, "collect_episode", side_effect=episode))
            stack.enter_context(mock.patch.object(np.random, "default_rng",
                                                 side_effect=AssertionError("Random initialization is removed")))
            if fail_finish:
                with self.assertRaisesRegex(OSError, "synthetic episode finish failure"):
                    pushing.collect(self.args, self.calibration, self.result, self.shape, object())
            else:
                self.assertEqual(pushing.collect(self.args, self.calibration, self.result, self.shape, object()), 0)
        self.assertEqual(ready.call_count, 3)  # A, B, then return to A once; no episode prompts.
        return setups, events

    def test_successes_alternate_active_goals_without_repeated_enter_prompts(self):
        setups, events = self.run_collection(["success", "success", "success"])
        self.assertEqual([setup["direction"] for setup in setups], ["A_to_B", "B_to_A", "A_to_B"])
        self.assertEqual(events.count("home"), 4)
        for index, event in enumerate(events):
            if event == ("finish", "success"):
                self.assertEqual(events[index + 1], "home")
        run_setup = json.loads((self.args.output / "setup.json").read_text())
        self.assertEqual(run_setup["T_base_goals"], self.goals)
        self.assertEqual(run_setup["T_base_goal"], self.b.tolist())
        self.assertEqual(run_setup["table_height_m"], 0.)
        np.testing.assert_array_equal(setups[0]["start_tip_base_m"], setups[2]["start_tip_base_m"])
        self.assertFalse(np.array_equal(setups[0]["initial_T_base_object"], setups[2]["initial_T_base_object"]))
        for setup, source in zip(setups, (self.a, self.b, self.a)):
            self.assertAlmostEqual(self.shape.distance_xy(source, setup["start_tip_base_m"][:2]), .075, places=6)

    def test_timeout_retries_the_same_destination_before_success_switches_it(self):
        setups, events = self.run_collection(["timeout", "success", "success"])
        self.assertEqual([setup["target_goal"] for setup in setups], ["B", "B", "A"])
        self.assertEqual([setup["direction"] for setup in setups], ["A_to_B", "A_to_B", "B_to_A"])
        self.assertEqual(events.count("home"), 4)
        np.testing.assert_array_equal(setups[0]["start_tip_base_m"], setups[1]["start_tip_base_m"])

    def test_writer_finish_failure_stops_before_another_capture_or_reset(self):
        setups, events = self.run_collection(["success", "success"], fail_finish=True)
        self.assertEqual(len(setups), 1)
        self.assertEqual(setups[0]["direction"], "A_to_B")
        self.assertEqual(events.count("home"), 1)
        self.assertIn("stop", events)
        self.assertEqual(len([event for event in events if isinstance(event, tuple) and event[0] == "capture"]), 3)


if __name__ == "__main__":
    unittest.main()
