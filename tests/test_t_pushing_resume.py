"""Offline regression checks for appending to an existing T-pushing run."""

from contextlib import ExitStack, redirect_stderr, redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import shutil
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
    script = Path(__file__).resolve().parents[1] / "examples" / "14_collect_t_pushing.py"
    spec = importlib.util.spec_from_file_location("t_pushing_resume_test", script)
    pushing = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pushing)


@unittest.skipUnless(available, "T-pushing requires OpenCV and MuJoCo")
class TPushingResumeTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.run = self.directory / "previous_run"
        self.run.mkdir()
        contexts = ExitStack()
        self.addCleanup(contexts.close)
        contexts.enter_context(redirect_stdout(io.StringIO()))
        contexts.enter_context(redirect_stderr(io.StringIO()))
        self.saved_args = pushing.parser().parse_args([
            "collect", "--condition", "ground", "--scale", ".07",
            "--frequency", "40", "--start-clearance", ".09", "--episodes", "6",
            "--output", str(self.run),
        ])
        self.shape = pushing.geometry.TShape(self.saved_args.target_config)
        self.goal = np.eye(4)
        self.goal[:3, :3] = [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]]
        self.goal[:3, 3] = [.5, .1, .12 + self.shape.half_thickness_m]
        self.goals = {"A": self.goal.copy(), "B": self.goal.copy()}
        self.goals["B"][0, 3] += .4
        self.goals["B"][2, 3] += .01  # Goal noise must not change the fixed height from A.
        self.result = {
            "schema_version": 1, "setup": "fixed_camera_robot_held_cube",
            "T_base_camera": np.eye(4).tolist(),
            "camera": {"serial": "test", "width": 8, "height": 8},
            "camera_matrix": [[500., 0., 4.], [0., 500., 4.], [0., 0., 1.]],
            "dist_coeffs": [0.] * 5,
        }
        self.save_json("calibration.json", self.result)
        shutil.copyfile(self.saved_args.target_config, self.run / "target_config.json")
        self.setup = {
            "T_base_goals": {name: pose.tolist() for name, pose in self.goals.items()},
            "T_base_goal": self.goals["B"].tolist(), "table_height_m": .12,
            "tip_offset_ee_m": self.saved_args.tip_offset, "tip_height_m": self.saved_args.tip_height,
            "R_base_ee": pushing.motion_code.DOWNWARD.tolist(), "condition": "ground",
            "osc_gains": pushing.motion_code.OSC_GAINS, "home_q": self.saved_args.home_q,
            "spacemouse_control": pushing.spacemouse_control(self.saved_args),
            "success_metric": "intersection_area / goal_T_footprint_area",
            "success_threshold": self.saved_args.success_threshold,
            "start_initialization": {"method": "fixed_source_outline", "clearance_m": .09,
                                     "side": "away_from_destination", "same_xy_fallback": "base_negative_y"},
        }
        self.save_json("setup.json", self.setup)
        self.manifest = {
            "schema_version": 1, "created_at": "2026-09-15T03:25:25+00:00",
            "status": "error", "reason": "original robot reflex",
            "condition": "ground", "arguments": vars(self.saved_args),
            "calibration_source": "/unavailable/original/calibration.json",
            "calibration_sha256": pushing.digest(self.run / "calibration.json"),
            "target_config_sha256": pushing.digest(self.run / "target_config.json"),
            "robot_model_sha256": pushing.digest(self.saved_args.robot_xml),
            "stick_xml_sha256": pushing.digest(self.saved_args.stick_xml),
            "spacemouse_control": pushing.spacemouse_control(self.saved_args),
            "camera": self.result["camera"],
        }
        self.save_json("run.json", self.manifest)
        (self.run / "goal.png").write_bytes(b"unchanged goal reference image")
        self.save_json("goal_detection.json", {"T_base_object": self.goal.tolist()})
        for name, pose in self.goals.items():
            (self.run / f"goal_{name}.png").write_bytes(f"unchanged reference {name}".encode())
            self.save_json(f"goal_{name}_detection.json", {"T_base_object": pose.tolist()})

    def save_json(self, name, value):
        path = self.run / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(pushing.recording.json_value(value), indent=2) + "\n")

    def parse(self, *extra):
        return pushing.parse_arguments(pushing.parser(), ["collect", "--resume", str(self.run), *extra])

    def test_resume_inherits_saved_settings_and_pins_copied_inputs(self):
        args = self.parse()
        self.assertEqual(args.condition, "ground")
        self.assertEqual(args.scale, .07)
        self.assertEqual(args.frequency, 40.)
        self.assertEqual(args.start_clearance, .09)
        for removed in ("start_min_distance", "start_distance", "seed"):
            self.assertFalse(hasattr(args, removed))
        self.assertEqual(args.episodes, 0)  # Each invocation has its own episode budget.
        self.assertIsNone(args.output)
        self.assertEqual(args.command, "collect")
        self.assertEqual(args.resume, self.run)
        self.assertEqual(args.calibration, self.run / "calibration.json")
        self.assertEqual(args.target_config, self.run / "target_config.json")
        self.assertIsInstance(args.robot_xml, Path)
        self.assertIsInstance(args.stick_xml, Path)

    def test_explicit_cli_overrides_are_preserved_even_when_equal_to_parser_defaults(self):
        args = self.parse("--scale", ".05", "--frequency=50", "--episodes", "1",
                          "--start-clearance", ".075")
        self.assertEqual(args.scale, .05)
        self.assertEqual(args.frequency, 50.)
        self.assertEqual(args.start_clearance, .075)
        self.assertEqual(args.episodes, 1)

    def test_resume_and_output_are_mutually_exclusive(self):
        with self.assertRaises(SystemExit):
            self.parse("--output", str(self.directory / "other"))

    def test_resume_rejects_changed_condition_and_changed_calibration_or_target(self):
        with self.assertRaises((SystemExit, ValueError)):
            self.parse("--condition", "elevated")
        for option, filename in (("--calibration", "calibration.json"),
                                 ("--target-config", "target_config.json")):
            with self.subTest(option=option):
                replacement = self.directory / filename
                replacement.write_bytes((self.run / filename).read_bytes() + b" \n")
                with self.assertRaises((SystemExit, ValueError)):
                    self.parse(option, str(replacement))

    def test_resume_requires_saved_goal_setup(self):
        (self.run / "setup.json").unlink()
        with self.assertRaises((SystemExit, ValueError, OSError)):
            self.parse()

    def test_legacy_single_goal_resume_explains_that_a_new_ab_run_is_required(self):
        legacy = dict(self.setup)
        del legacy["T_base_goals"]
        self.save_json("setup.json", legacy)
        with self.assertRaisesRegex(ValueError, r"(?i)(new.*A/B|A/B.*new)"):
            self.parse()

    def test_resume_adds_episode_after_incomplete_directory_and_preserves_existing_bytes(self):
        self.save_json("episode_0001/episode.json", {"status": "success"})
        (self.run / "episode_0001/states.jsonl").write_text('{"old":true}\n')
        # A killed process may create an episode directory before its manifest.
        (self.run / "episode_0006").mkdir()
        (self.run / "episode_0006/states.jsonl").write_text('{"partial":true}\n')
        before = {path.relative_to(self.run): path.read_bytes()
                  for path in self.run.rglob("*") if path.is_file()}
        args = self.parse("--episodes", "1", "--scale", ".05", "--success-threshold", ".86")
        initial = self.goal.copy()
        initial[:3, 3] += [.04, -.03, .25]
        frame = {"image": np.zeros((8, 8, 3), dtype=np.uint8),
                 "record": {"T_base_object": initial.tolist(), "camera_timestamp_s": 1000.,
                            "sequence": 7, "valid": True, "predicted": False}}
        robot, motion = mock.Mock(), mock.Mock()
        motion.plan_start.return_value = {"start_q": [0.] * 7}
        tracker = SimpleNamespace(metadata=self.result["camera"], attach=mock.Mock())
        camera_context, mouse_context = mock.MagicMock(), mock.MagicMock()
        camera_context.__enter__.return_value = tracker

        def save_episode(_robot, _motion, _kin, _mouse, _tracker, _shape, setup, writer, effective):
            np.testing.assert_array_equal(setup["T_base_goal"], self.goals["B"])
            self.assertEqual((setup["source_goal"], setup["target_goal"], setup["direction"]),
                             ("A", "B", "A_to_B"))
            np.testing.assert_array_equal(setup["initial_T_base_object"], initial)
            self.assertEqual(setup["table_height_m"], self.setup["table_height_m"])
            self.assertAlmostEqual(setup["start_tip_base_m"][2], .14)
            self.assertEqual(setup["spacemouse_control"]["xy_scale_m_per_unit"], [.05, .05])
            self.assertEqual(setup["success_threshold"], .86)
            self.assertEqual(effective.success_threshold, .86)
            self.assertEqual(setup["start_initialization"], {
                "method": "fixed_source_outline", "clearance_m": .09,
                "side": "away_from_destination", "same_xy_fallback": "base_negative_y"})
            self.assertNotIn("start_sampling", setup)
            self.assertAlmostEqual(setup["start_outline_distance_m"], .09)
            self.assertAlmostEqual(self.shape.distance_xy(self.goals["A"], setup["start_tip_base_m"][:2]), .09)
            self.assertEqual(writer.directory.name, "episode_0007")
            self.assertEqual(writer.metadata["episode"], 7)
            writer.record_state({"timestamp_s": 1000., "goal_coverage": .9})
            writer.record_frame(frame["image"], frame["record"])
            return "success"

        def encode_after_shutdown(directory, *, overlay=False):
            self.assertTrue(robot.stop.called)
            self.assertTrue(camera_context.__exit__.called)
            saved = json.loads((directory / "episode.json").read_text())
            self.assertEqual(saved["status"], "success")
            self.assertEqual(saved["unwritten_frame_count"], 0)
            return directory / ("aprilcube_overlay.mp4" if overlay else "video.mp4")

        encoder = mock.Mock(side_effect=encode_after_shutdown)
        with mock.patch.dict(sys.modules, {"aiofranka": SimpleNamespace(
                FrankaRemoteController=mock.Mock(return_value=robot))}), \
                mock.patch.object(pushing.tracking, "CameraTracker", return_value=camera_context), \
                mock.patch.object(pushing.tracking, "MouseReader", return_value=mouse_context), \
                mock.patch.object(pushing.tracking, "capture_target", return_value=(initial, frame)) as capture, \
                mock.patch.object(pushing.motion_code, "Motion", return_value=motion), \
                mock.patch.object(pushing.camera_code, "PycaasCamera", side_effect=AssertionError("camera opened")), \
                mock.patch.object(pushing, "ask_ready", return_value=True) as ready, \
                mock.patch.object(pushing, "sibling", return_value=SimpleNamespace(encode_episode=encoder)), \
                mock.patch.object(pushing, "collect_episode", side_effect=save_episode) as collect_episode:
            self.assertEqual(pushing.collect(args, args.calibration, self.result, self.shape, object()), 0)
        capture.assert_called_once()  # Current object selects direction; both saved goals are reused.
        ready.assert_not_called()
        collect_episode.assert_called_once()
        self.assertEqual(encoder.call_args_list, [
            mock.call(self.run / "episode_0007", overlay=False),
            mock.call(self.run / "episode_0007", overlay=True)])
        self.assertEqual(motion.go_home.call_count, 2)
        for relative, contents in before.items():
            self.assertEqual((self.run / relative).read_bytes(), contents, str(relative))
        self.assertFalse((self.run / "episode_0008").exists())
        episode = json.loads((self.run / "episode_0007/episode.json").read_text())
        self.assertEqual(episode["status"], "success")
        self.assertEqual(episode["state_count"], 1)
        self.assertEqual(episode["frame_count"], 1)
        attempts = list(self.run.glob("resume_*.json"))
        self.assertEqual(len(attempts), 1)
        self.assertEqual(json.loads(attempts[0].read_text())["status"], "complete")

    def test_old_sampled_ab_run_uses_fixed_default_without_rewriting_saved_settings(self):
        old_arguments = {**self.manifest["arguments"], "start_min_distance": .06,
                         "start_distance": .13, "seed": 19}
        old_arguments.pop("start_clearance")
        self.save_json("run.json", {**self.manifest, "arguments": old_arguments, "random_seed": 19})
        old_setup = {**self.setup, "start_sampling": {"method": "uniform_area_outside_T_outline",
                                                      "min_distance_m": .06, "max_distance_m": .13, "seed": 19}}
        old_setup.pop("start_initialization")
        self.save_json("setup.json", old_setup)
        original = {path: path.read_bytes() for path in self.run.rglob("*") if path.is_file()}
        args = self.parse("--episodes", "1", "--no-video")
        self.assertEqual(args.start_clearance, .075)
        for removed in ("start_min_distance", "start_distance", "seed"):
            self.assertFalse(hasattr(args, removed))
        initial = self.goals["A"].copy()
        initial[:2, 3] += [.03, -.02]
        frame = {"image": np.zeros((8, 8, 3), dtype=np.uint8),
                 "record": {"T_base_object": initial.tolist(), "camera_timestamp_s": 1000.,
                            "sequence": 1, "valid": True, "predicted": False}}
        robot, motion = mock.Mock(), mock.Mock()
        motion.plan_start.return_value = {"start_q": [0.] * 7}
        tracker = SimpleNamespace(metadata=self.result["camera"], attach=mock.Mock())
        camera_context, mouse_context = mock.MagicMock(), mock.MagicMock()
        camera_context.__enter__.return_value = tracker

        def collect_fixed(_robot, _motion, _kin, _mouse, _tracker, _shape, setup, writer, effective):
            self.assertEqual(effective.start_clearance, .075)
            self.assertNotIn("start_sampling", setup)
            self.assertEqual(setup["start_initialization"]["method"], "fixed_source_outline")
            self.assertEqual(setup["start_initialization"]["clearance_m"], .075)
            self.assertAlmostEqual(setup["start_outline_distance_m"], .075)
            self.assertAlmostEqual(self.shape.distance_xy(self.goals["A"], setup["start_tip_base_m"][:2]), .075)
            return "success"

        with mock.patch.dict(sys.modules, {"aiofranka": SimpleNamespace(
                FrankaRemoteController=mock.Mock(return_value=robot))}), \
                mock.patch.object(pushing.tracking, "CameraTracker", return_value=camera_context), \
                mock.patch.object(pushing.tracking, "MouseReader", return_value=mouse_context), \
                mock.patch.object(pushing.tracking, "capture_target", return_value=(initial, frame)), \
                mock.patch.object(pushing.motion_code, "Motion", return_value=motion), \
                mock.patch.object(pushing.camera_code, "PycaasCamera", side_effect=AssertionError("camera opened")), \
                mock.patch.object(pushing, "ask_ready", side_effect=AssertionError("resume asked for Enter")), \
                mock.patch.object(pushing, "collect_episode", side_effect=collect_fixed):
            self.assertEqual(pushing.collect(args, args.calibration, self.result, self.shape, object()), 0)
        for path, payload in original.items():
            self.assertEqual(path.read_bytes(), payload, str(path))
        attempt = json.loads((self.run / "resume_0001.json").read_text())
        self.assertNotIn("random_seed", attempt)
        self.assertNotIn("start_sampling", attempt["setup"])
        episode = json.loads((self.run / "episode_0001/setup.json").read_text())
        self.assertNotIn("start_sampling", episode)
        self.assertEqual(episode["start_initialization"]["clearance_m"], .075)

    def test_noisy_current_poses_cannot_shift_fixed_start_after_source_selection(self):
        args = self.parse()
        starts = []
        for offset in ([.02, -.01, .02], [-.03, .04, -.01]):
            initial = self.goals["A"].copy()
            initial[:3, 3] += offset
            self.assertEqual(pushing.select_source_goal(self.shape, self.goals, initial), "A")
            selected = pushing.setup_for_goal(self.setup, "B")
            episode = pushing.make_episode_setup(args, selected, self.shape, initial)
            starts.append(episode["start_tip_base_m"])
            np.testing.assert_array_equal(episode["initial_T_base_object"], initial)
            self.assertAlmostEqual(episode["start_outline_distance_m"], .09)
        np.testing.assert_array_equal(starts[0], starts[1])

    def test_resume_chooses_direction_from_live_pose_instead_of_saved_outcome_or_episode_number(self):
        baseline = self.run
        cases = [
            ("A", "success", "A_to_B", 6, [.02, -.01, .25]),
            ("B", "success", "B_to_A", 5, [-.02, .01, .25]),
            ("A", "error", "B_to_A", 8, [.02, 0., 0.]),
            ("B", "timeout", "A_to_B", 9, [-.02, 0., 0.]),
            # Both footprints have zero overlap: select the nearer goal in XY.
            ("A", "recording", "B_to_A", 12, [.02, .8, 0.]),
            ("B", "success", "B_to_A", 11, [-.02, .8, 0.]),
        ]
        for case_index, (source, status, old_direction, previous_index, offset) in enumerate(cases):
            with self.subTest(source=source, previous_status=status, old_direction=old_direction):
                self.run = self.directory / f"case_{case_index}"
                shutil.copytree(baseline, self.run)
                self.save_json(f"episode_{previous_index:04d}/episode.json", {
                    "status": status, "metadata": {"direction": old_direction,
                                                    "source_goal": old_direction[0],
                                                    "target_goal": old_direction[-1]},
                })
                args = self.parse("--episodes", "1", "--no-video")
                initial = self.goals[source].copy()
                initial[:3, 3] += offset
                frame = {"image": np.zeros((8, 8, 3), dtype=np.uint8),
                         "record": {"T_base_object": initial.tolist(), "camera_timestamp_s": 1000.,
                                    "sequence": 1, "valid": True, "predicted": False}}
                robot, motion = mock.Mock(), mock.Mock()
                motion.plan_start.return_value = {"start_q": [0.] * 7}
                tracker = SimpleNamespace(metadata=self.result["camera"], attach=mock.Mock())
                camera_context, mouse_context = mock.MagicMock(), mock.MagicMock()
                camera_context.__enter__.return_value = tracker
                target = "B" if source == "A" else "A"

                def capture_current(_tracker):
                    motion.go_home.assert_called_once()
                    return initial, frame

                def collect_current(_robot, _motion, _kin, _mouse, _tracker, _shape, setup, writer, _args):
                    self.assertEqual(setup["source_goal"], source)
                    self.assertEqual(setup["target_goal"], target)
                    self.assertEqual(setup["direction"], f"{source}_to_{target}")
                    np.testing.assert_array_equal(setup["T_base_goal"], self.goals[target])
                    np.testing.assert_array_equal(setup["initial_T_base_object"], initial)
                    self.assertAlmostEqual(setup["table_height_m"], .12)
                    self.assertAlmostEqual(setup["start_tip_base_m"][2], .14)
                    self.assertEqual(writer.metadata["episode"], previous_index + 1)
                    return "success"

                with mock.patch.dict(sys.modules, {"aiofranka": SimpleNamespace(
                        FrankaRemoteController=mock.Mock(return_value=robot))}), \
                        mock.patch.object(pushing.tracking, "CameraTracker", return_value=camera_context), \
                        mock.patch.object(pushing.tracking, "MouseReader", return_value=mouse_context), \
                        mock.patch.object(pushing.tracking, "capture_target", side_effect=capture_current) as capture, \
                        mock.patch.object(pushing.motion_code, "Motion", return_value=motion), \
                        mock.patch.object(pushing.camera_code, "PycaasCamera", side_effect=AssertionError("camera opened")), \
                        mock.patch.object(pushing, "ask_ready", side_effect=AssertionError("resume asked for Enter")), \
                        mock.patch.object(pushing, "collect_episode", side_effect=collect_current) as collect_episode:
                    self.assertEqual(pushing.collect(args, args.calibration, self.result, self.shape, object()), 0)
                capture.assert_called_once()
                collect_episode.assert_called_once()
        self.run = baseline


if __name__ == "__main__":
    unittest.main()
