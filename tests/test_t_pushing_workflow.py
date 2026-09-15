"""Hardware-free workflow checks for calibration selection and T-pushing control."""

import builtins
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
    script = Path(__file__).resolve().parents[1] / "examples" / "14_collect_t_pushing.py"
    spec = importlib.util.spec_from_file_location("t_pushing_workflow_test", script)
    pushing = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pushing)


class FakeClock:
    def __init__(self):
        self.elapsed = 0.

    def monotonic(self):
        return 10. + self.elapsed

    def time(self):
        return 1000. + self.elapsed

    def sleep(self, duration):
        self.elapsed += duration


class FakeKinematics:
    def __init__(self, *_args, **_kwargs):
        self.tip_offset = np.array([0., 0., .161])

    def ee_for_tip(self, tip):
        pose = np.eye(4)
        pose[:3, :3] = np.diag([1., -1., -1.])
        pose[:3, 3] = np.asarray(tip) - pose[:3, :3] @ self.tip_offset
        return pose

    def tip(self, pose):
        return pose[:3, 3] + pose[:3, :3] @ self.tip_offset

    def fk(self, _q):
        return self.ee_for_tip([.5, 0., .4])


class FakeRobot:
    def __init__(self, pose):
        self.pose = pose.copy()
        self.commands = []

    @property
    def ee_desired(self):
        return self.pose.copy()

    @ee_desired.setter
    def ee_desired(self, pose):
        self.pose = np.asarray(pose).copy()
        self.commands.append(self.pose)


class FakeMotion:
    def __init__(self, robot, clock):
        self.robot, self.clock = robot, clock
        self.holds = 0

    def read_state(self):
        return {"ee": self.robot.pose.copy(), "qpos": np.zeros(7), "qvel": np.zeros(7),
                "timestamp": self.clock.time(), "last_torque": np.zeros(7)}

    def hold(self):
        self.holds += 1
        self.robot.ee_desired = self.robot.pose
        return self.read_state()


class FakeTracker:
    def __init__(self, clock, goal, lost_after=None):
        self.clock, self.goal, self.lost_after = clock, goal, lost_after
        self.sequence = 0

    def snapshot(self):
        self.sequence += 1
        return {"record": {"sequence": self.sequence, "camera_timestamp_s": self.clock.time(),
                           "valid": self.lost_after is None or self.clock.elapsed < self.lost_after,
                           "T_base_object": self.goal.tolist()}}


@unittest.skipUnless(available, "T-pushing requires OpenCV and MuJoCo")
class TPushingWorkflowTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.contexts = ExitStack()
        self.addCleanup(self.contexts.close)
        self.contexts.enter_context(redirect_stdout(io.StringIO()))
        self.contexts.enter_context(redirect_stderr(io.StringIO()))
        self.args = pushing.parser().parse_args(["collect", "--condition", "ground"])
        self.shape = pushing.geometry.TShape(self.args.target_config)
        self.goal = np.eye(4)
        # Target thickness is local Y. Lay its +Y face upwards in robot base Z.
        self.goal[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
        self.goal[:3, 3] = [.5, 0., .1 + self.shape.half_thickness_m]
        self.source = self.goal.copy()
        self.source[1, 3] -= .35
        self.goals = {"A": self.source.tolist(), "B": self.goal.tolist()}
        self.result = {"T_base_camera": np.eye(4).tolist(),
                       "camera": {"serial": "test", "width": 640, "height": 480},
                       "camera_matrix": [[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]],
                       "dist_coeffs": [0.] * 5,
                       "robot_model_sha256": pushing.digest(self.args.robot_xml)}

    def test_default_inspect_and_plan_open_no_robot_camera_or_hid(self):
        goal_file = self.directory / "setup.json"
        goal_file.write_text(json.dumps(self.ab_setup()))
        original_import = builtins.__import__

        def guarded(name, *args, **kwargs):
            if name.split(".")[0] in {"aiofranka", "pycaas", "pyspacemouse"}:
                raise AssertionError(f"Offline command imported a device library: {name}")
            return original_import(name, *args, **kwargs)

        with mock.patch.object(pushing, "load_calibration", return_value=(self.directory / "calibration.json", self.result)), \
                mock.patch.object(pushing.motion_code, "Kinematics", FakeKinematics), \
                mock.patch.object(pushing.motion_code, "Motion") as motion, \
                mock.patch.object(pushing.tracking, "CameraTracker", side_effect=AssertionError("camera opened")), \
                mock.patch.object(pushing.tracking, "MouseReader", side_effect=AssertionError("HID opened")), \
                mock.patch("builtins.__import__", guarded):
            motion.return_value.plan_start.return_value = {"start_q": [0.] * 7}
            self.assertEqual(pushing.main([]), 0)
            motion.assert_not_called()
            self.assertEqual(pushing.main(["plan", "--goal-pose", str(goal_file)]), 0)
            self.assertIsNone(motion.call_args.args[0])
            motion.return_value.plan_start.assert_called_once()

    def test_offline_plan_matches_fixed_collection_start_and_saved_table_height(self):
        setup = self.ab_setup()
        setup["table_height_m"] = .123
        goal_file, object_file = self.directory / "setup.json", self.directory / "object.json"
        for saved_target, live_source, expected_target in ((None, None, "B"), ("A", None, "A"),
                                                           ("A", "A", "B"), ("B", "B", "A")):
            with self.subTest(saved_target=saved_target, live_source=live_source):
                saved = dict(setup)
                saved.pop("target_goal")
                if saved_target:
                    saved["target_goal"] = saved_target
                goal_file.write_text(json.dumps(saved))
                arguments = ["plan", "--goal-pose", str(goal_file)]
                source = "B" if expected_target == "A" else "A"
                current = np.asarray(self.goals[live_source or source]).copy()
                current[:3, 3] += [.006, -.008, .009]
                if live_source:
                    object_file.write_text(json.dumps({"T_base_object": current.tolist()}))
                    arguments += ["--object-pose", str(object_file)]
                output = io.StringIO()
                with redirect_stdout(output), \
                        mock.patch.object(pushing, "load_calibration", return_value=(self.directory / "calibration.json", self.result)), \
                        mock.patch.object(pushing.motion_code, "Kinematics", FakeKinematics), \
                        mock.patch.object(pushing.motion_code, "Motion") as motion, \
                        mock.patch.object(np.random, "default_rng", side_effect=AssertionError("Random initialization is removed")):
                    motion.return_value.plan_start.side_effect = lambda xyz: {"tip": list(xyz), "start_q": [0.] * 7}
                    self.assertEqual(pushing.main(arguments), 0)
                printed = output.getvalue()
                planned, _ = json.JSONDecoder().raw_decode(printed[printed.index("{"):])
                collected = pushing.make_episode_setup(
                    self.args, pushing.setup_for_goal(setup, expected_target), self.shape, current)
                self.assertEqual(planned["target_goal"], expected_target)
                self.assertEqual(planned["source_goal"], source)
                self.assertAlmostEqual(planned["table_height_m"], .123)
                self.assertAlmostEqual(planned["start_tip_base_m"][2], .143)
                np.testing.assert_array_equal(planned["start_tip_base_m"], collected["start_tip_base_m"])
                np.testing.assert_array_equal(motion.return_value.plan_start.call_args.args[0],
                                              planned["start_tip_base_m"])

    def test_plan_inherits_saved_initialization_and_geometry_but_explicit_flags_override(self):
        saved = self.ab_setup()
        saved["start_initialization"]["clearance_m"] = .09
        saved["home_q"] = [.1, -.2, .3, -1.4, .5, 1.6, .7]
        saved["tip_offset_ee_m"] = [.001, .002, .18]
        goal_file = self.directory / "setup.json"
        for wrapped in (False, True):
            with self.subTest(metadata_wrapper=wrapped):
                goal_file.write_text(json.dumps({"metadata": saved} if wrapped else saved))
                arguments = ["plan", "--goal-pose", str(goal_file)]
                inherited = pushing.parse_arguments(pushing.parser(), arguments)
                self.assertEqual(inherited.start_clearance, .09)
                self.assertEqual(inherited.home_q, saved["home_q"])
                self.assertEqual(inherited.tip_offset, saved["tip_offset_ee_m"])
                explicit = pushing.parse_arguments(pushing.parser(), arguments + [
                    "--start-clearance", ".075", "--home-q", "0", "0", "0", "-1", "0", "1", "0",
                    "--tip-offset", "0", "0", ".161"])
                self.assertEqual(explicit.start_clearance, .075)
                self.assertEqual(explicit.home_q, [0., 0., 0., -1., 0., 1., 0.])
                self.assertEqual(explicit.tip_offset, [0., 0., .161])

    def test_plan_rejects_legacy_scalar_or_matrix_goal_files_before_opening_devices(self):
        goal_file = self.directory / "old_goal.json"
        for saved in (0.5, self.goal.tolist()):
            with self.subTest(saved=saved):
                goal_file.write_text(json.dumps(saved))
                with self.assertRaisesRegex(ValueError, "saved setup with goals A/B"):
                    pushing.parse_arguments(pushing.parser(), ["plan", "--goal-pose", str(goal_file)])

    def test_latest_completed_calibration_is_pinned_and_bad_latest_is_not_skipped(self):
        def session(name):
            directory = self.directory / "camera_calibration" / name
            directory.mkdir(parents=True)
            result = {**self.result, "setup": "fixed_camera_robot_held_cube", "schema_version": 1,
                      "robot_model_sha256": "older-model", "dataset_sha256": "older-dataset",
                      "dataset": "/unavailable/original/views.json"}
            path = directory / "calibration.json"
            path.write_text(json.dumps(result))
            return path

        session("20260914_170000")
        newest = session("20260914_190000")
        (self.directory / "camera_calibration/20260914_200000").mkdir()
        selected, result = pushing.load_calibration(root=self.directory)
        self.assertEqual(selected, newest)
        self.assertEqual(result["T_base_camera"], self.result["T_base_camera"])
        # Hashes remain provenance; archived source data and the old model are not required.
        self.assertEqual(result["robot_model_sha256"], "older-model")
        newest.write_text("{}")
        with self.assertRaisesRegex(ValueError, "fixed-camera"):
            pushing.load_calibration(root=self.directory)

    def test_camera_profile_and_identity_must_match_selected_calibration(self):
        metadata = {**self.result["camera"], "camera_matrix": self.result["camera_matrix"],
                    "dist_coeffs": self.result["dist_coeffs"]}
        pushing.validate_camera(metadata, self.result)
        for key, value in (("serial", "another-camera"), ("width", 1280),
                           ("camera_matrix", np.diag([500., 500., 1.])),
                           ("dist_coeffs", [.01, 0., 0., 0., 0.])):
            with self.subTest(key=key):
                with self.assertRaisesRegex(ValueError, key):
                    pushing.validate_camera({**metadata, key: value}, self.result)

    def test_run_setup_fixes_goal_height_tool_and_initialization_policy(self):
        setup = pushing.make_setup(self.args, self.result, self.shape, self.goal)
        self.assertAlmostEqual(setup["table_height_m"], .1)
        self.assertNotIn("start_tip_base_m", setup)
        self.assertNotIn("approach_plan", setup)
        self.assertEqual(setup["start_initialization"], {
            "method": "fixed_source_outline", "clearance_m": .075,
            "side": "away_from_destination", "same_xy_fallback": "base_negative_y"})
        self.assertNotIn("start_sampling", setup)
        self.assertEqual(setup["success_threshold"], .85)
        np.testing.assert_allclose(setup["tip_offset_ee_m"], [0., 0., .161])
        np.testing.assert_allclose(setup["R_base_ee"], np.diag([1., -1., -1.]))
        self.assertEqual(setup["condition"], "ground")
        elevated = self.goal.copy()
        elevated[2, 3] += .15
        elevated_setup = pushing.make_setup(self.args, self.result, self.shape, elevated)
        self.assertAlmostEqual(elevated_setup["table_height_m"], .25)

    def ab_setup(self, target="B"):
        setup = pushing.make_setup(self.args, self.result, self.shape, self.source)
        setup["T_base_goals"] = self.goals
        return pushing.setup_for_goal(setup, target)

    def test_episode_start_uses_saved_source_outline_despite_changed_live_pose(self):
        setup = self.ab_setup()
        before = pushing.recording.json_value(setup)
        initial = self.source.copy()
        initial[:3, 3] += [.15, .18, .25]
        with mock.patch.object(np.random, "default_rng", side_effect=AssertionError("Random initialization is removed")):
            episode = pushing.make_episode_setup(self.args, setup, self.shape, initial)
        np.testing.assert_array_equal(episode["initial_T_base_object"], initial)
        np.testing.assert_array_equal(episode["T_base_goal"], self.goal)
        self.assertEqual(episode["table_height_m"], setup["table_height_m"])
        self.assertAlmostEqual(episode["start_tip_base_m"][2], .12)
        distance = self.shape.distance_xy(self.source, episode["start_tip_base_m"][:2])
        self.assertAlmostEqual(distance, .075, places=6)
        self.assertGreater(self.shape.distance_xy(initial, episode["start_tip_base_m"][:2]), .10)
        self.assertLess(episode["start_tip_base_m"][1], self.source[1, 3])
        self.assertAlmostEqual(episode["start_outline_distance_m"], distance)
        self.assertEqual(pushing.recording.json_value(setup), before)
        repeated = pushing.make_episode_setup(self.args, setup, self.shape, initial)
        np.testing.assert_array_equal(repeated["start_tip_base_m"], episode["start_tip_base_m"])
        translated = initial.copy()
        translated[:2, 3] += [.4, -.3]
        translated[:3, :3] = np.diag([-1., -1., 1.]) @ translated[:3, :3]
        moved = pushing.make_episode_setup(self.args, setup, self.shape, translated)
        np.testing.assert_array_equal(moved["start_tip_base_m"], episode["start_tip_base_m"])
        np.testing.assert_array_equal(moved["initial_T_base_object"], translated)

    def test_reverse_leg_initializes_beside_saved_b_away_from_a(self):
        forward = pushing.make_episode_setup(self.args, self.ab_setup("B"), self.shape, self.source)
        reverse = pushing.make_episode_setup(self.args, self.ab_setup("A"), self.shape, self.source)
        np.testing.assert_array_equal(reverse["T_base_goal"], self.source)
        self.assertEqual(reverse["source_goal"], "B")
        self.assertAlmostEqual(self.shape.distance_xy(self.goal, reverse["start_tip_base_m"][:2]), .075, places=6)
        self.assertGreater(reverse["start_tip_base_m"][1], self.goal[1, 3])
        self.assertNotEqual(reverse["start_tip_base_m"], forward["start_tip_base_m"])

    def test_success_stops_at_first_native_pose_above_threshold_even_with_latency(self):
        robot, motion, kin, mouse, tracker, setup, writer, rows = self.episode_fakes()
        original = tracker.snapshot

        def delayed_result():
            result = original()
            result["record"]["camera_timestamp_s"] -= .35
            return result

        tracker.snapshot = delayed_result
        # Exactly 85% continues; 85.1% succeeds on that same update. A 350 ms
        # processing delay must not silently invalidate AprilCube's result.
        with mock.patch.object(self.shape, "overlap", side_effect=[.85, .851]):
            outcome = pushing.collect_episode(robot, motion, kin, mouse, tracker, self.shape,
                                              setup, writer, self.args)
        self.assertEqual(outcome, "success")
        self.assertEqual([row["goal_coverage"] for row in rows], [.85, .851])
        self.assertTrue(all(row["tracking_valid"] for row in rows))
        self.assertEqual(motion.holds, 1)

    def test_success_uses_concave_t_footprint_not_its_bounding_box(self):
        reversed_pose = self.goal.copy()
        reversed_pose[:3, :3] = np.diag([-1., -1., 1.]) @ self.goal[:3, :3]
        goal_xy = self.shape.footprint(self.goal).reshape(-1, 2)
        actual_xy = self.shape.footprint(reversed_pose).reshape(-1, 2)
        np.testing.assert_allclose(goal_xy.min(axis=0), actual_xy.min(axis=0))
        np.testing.assert_allclose(goal_xy.max(axis=0), actual_xy.max(axis=0))
        ratio = self.shape.overlap(self.goal, reversed_pose)
        self.assertLess(ratio, .9)

    def test_xy_commands_use_measured_position_and_clip_only_normalized_input(self):
        actual = np.array([.5, -.2])
        target, command = pushing.osc_xy_target(actual, [.8, -.4, 99., -99., 42., 1.], .05)
        reference, reference_command = pushing.osc_xy_target(actual, [.8, -.4], .05)
        np.testing.assert_array_equal(target, reference)
        np.testing.assert_array_equal(command, reference_command)
        np.testing.assert_allclose(target, [.54, -.22])
        # Holding input recomputes the offset from actual motion, without target accumulation.
        moved = actual + [.003, .001]
        moved_target, _ = pushing.osc_xy_target(moved, [.8, -.4], .05)
        np.testing.assert_allclose(moved_target, [.543, -.219])
        released, _ = pushing.osc_xy_target(moved, [0., 0.], .05)
        np.testing.assert_array_equal(released, moved)
        diagonal, clipped = pushing.osc_xy_target(actual, [2., -3.], .05)
        np.testing.assert_allclose(diagonal - actual, [.05, -.05])
        np.testing.assert_array_equal(clipped, [1., -1.])
        unbounded, _ = pushing.osc_xy_target([.99, -.99], [1., -1.], .05)
        np.testing.assert_allclose(unbounded, [1.04, -1.04])
        released, _ = pushing.osc_xy_target(unbounded, [0., 0.], .05)
        np.testing.assert_array_equal(released, unbounded)

    def test_fixed_initialization_arguments_replace_random_sampling_and_have_no_workspace(self):
        self.assertEqual(self.args.start_clearance, .075)
        for removed in ("workspace_half_width", "start_offset", "start_direction",
                        "start_min_distance", "start_distance", "seed"):
            self.assertFalse(hasattr(self.args, removed))
        custom = pushing.parser().parse_args(["--start-clearance", ".08"])
        self.assertEqual(custom.start_clearance, .08)
        self.assertEqual(pushing.make_setup(custom, self.result, self.shape, self.goal)
                         ["start_initialization"]["clearance_m"], .08)
        setup = pushing.make_setup(self.args, self.result, self.shape, self.goal)
        self.assertNotIn("workspace_xy_bounds_m", setup)
        self.assertNotIn("workspace", setup["spacemouse_control"]["rule"])
        for value in ("0", "-1", "nan", "inf"):
            with self.subTest(distance=value), self.assertRaises(SystemExit):
                pushing.main(["--start-clearance", value])
        for removed in ("--start-min-distance", "--start-distance", "--seed"):
            with self.subTest(argument=removed), self.assertRaises(SystemExit):
                pushing.parser().parse_args([removed, "5"])

    def test_scale_default_override_metadata_and_validation(self):
        self.assertEqual(self.args.scale, .05)
        self.assertEqual(self.args.deadzone, 0.)
        setup = pushing.make_setup(self.args, self.result, self.shape, self.goal)
        self.assertEqual(setup["spacemouse_control"]["xy_scale_m_per_unit"], [.05, .05])
        self.assertEqual(setup["spacemouse_control"]["mode"], "measured_tip_offset")
        # OSC gain changes must not silently rescale the operator's selected offset.
        with mock.patch.dict(pushing.motion_code.OSC_GAINS, {"ee_kp": [500.] * 6, "ee_kd": [30.] * 6}):
            self.assertEqual(pushing.spacemouse_control(self.args)["xy_scale_m_per_unit"], [.05, .05])
        custom = pushing.parser().parse_args(["--scale", ".025"])
        self.assertEqual(pushing.spacemouse_control(custom)["xy_scale_m_per_unit"], [.025, .025])
        for value in ("0", "-1", "nan", "inf"):
            with self.subTest(scale=value), self.assertRaises(SystemExit) as stopped:
                pushing.main(["--scale", value])
            self.assertEqual(stopped.exception.code, 2)

    def test_optional_deadzone_suppresses_small_input_without_rescaling_remaining_axes(self):
        actual = [.5, -.2]
        target, command = pushing.osc_xy_target(actual, [.04, -.2], .05, .08)
        np.testing.assert_array_equal(command, [0., -.2])
        np.testing.assert_allclose(target, [.5, -.21])

    def episode_fakes(self, lost_after=None):
        clock, kin = FakeClock(), FakeKinematics()
        setup = pushing.make_episode_setup(self.args, self.ab_setup(), self.shape, self.goal)
        robot = FakeRobot(kin.ee_for_tip(setup["start_tip_base_m"]))
        motion = FakeMotion(robot, clock)
        tracker = FakeTracker(clock, self.goal, lost_after=lost_after)
        mouse = SimpleNamespace(read=lambda: (np.array([.7, -.4, 1., 1., -1., 1.]), [False], .001))
        rows = []
        writer = SimpleNamespace(check=lambda: None, record_state=lambda row: rows.append(pushing.recording.json_value(row)))
        self.contexts.enter_context(mock.patch.object(pushing, "time", clock))
        self.contexts.enter_context(mock.patch.object(pushing.tracking, "time", clock))
        return robot, motion, kin, mouse, tracker, setup, writer, rows

    def test_episode_automatically_succeeds_and_records_actual_state_and_fixed_plane_commands(self):
        robot, motion, kin, mouse, tracker, setup, writer, rows = self.episode_fakes()
        outcome = pushing.collect_episode(robot, motion, kin, mouse, tracker, self.shape, setup, writer, self.args)
        self.assertEqual(outcome, "success")
        self.assertEqual(len(rows), 1)
        self.assertEqual(motion.holds, 1)
        for pose in robot.commands:
            np.testing.assert_allclose(pose[:3, :3], np.diag([1., -1., -1.]))
            self.assertAlmostEqual(kin.tip(pose)[2], .12)
        for row in rows:
            self.assertEqual(row["phase"], "teleop")
            self.assertTrue(row["tracking_valid"])
            self.assertGreater(row["goal_coverage"], .9)
            self.assertEqual(len(row["qpos"]), 7)
            self.assertEqual(row["camera_timestamp_s"], row["robot_timestamp_s"])
            self.assertEqual(len(row["spacemouse_axes"]), 6)
            self.assertAlmostEqual(row["target_tip_base_m"][2], .12)

    def test_tracking_loss_continues_teleop_and_records_missing_pose_without_success(self):
        self.args.episode_seconds = 1.2  # Exceeds the removed one-second tracking timeout.
        robot, motion, kin, mouse, tracker, setup, writer, rows = self.episode_fakes(lost_after=.04)
        tracker.goal = self.goal.copy()
        tracker.goal[0, 3] += .4  # Object is away from the goal before tracking is lost.
        outcome = pushing.collect_episode(robot, motion, kin, mouse, tracker, self.shape, setup, writer, self.args)
        self.assertEqual(outcome, "timeout")
        self.assertEqual(motion.holds, 1)  # Only the normal episode-end hold.
        missing = [row for row in rows if not row["tracking_valid"]]
        self.assertGreater(len(missing), 50)
        for row in missing:
            self.assertEqual(row["phase"], "teleop")
            self.assertIsNone(row["goal_coverage"])
            self.assertIsNone(row["T_base_object"])
            np.testing.assert_allclose(row["command_xy_normalized"], [.7, -.4])
            np.testing.assert_allclose(np.array(row["target_tip_base_m"])[:2],
                                       np.array(row["tip_base_m"])[:2] + [.035, -.02])
        self.assertGreater(missing[-1]["target_tip_base_m"][0], missing[0]["target_tip_base_m"][0])

    def test_measured_pose_outside_former_workspace_keeps_requested_xy_offset(self):
        robot, motion, kin, mouse, tracker, setup, writer, rows = self.episode_fakes()
        measured = np.eye(4)
        measured[:3, 3] = [.85, 0., .2]
        motion.read_state = lambda: {"ee": measured.copy(), "qpos": np.zeros(7),
                                     "qvel": np.zeros(7), "timestamp": tracker.clock.time()}
        self.assertEqual(pushing.collect_episode(robot, motion, kin, mouse, tracker, self.shape,
                                                 setup, writer, self.args), "success")
        for row in rows:
            np.testing.assert_array_equal(row["T_base_ee"], measured)
            self.assertAlmostEqual(row["target_tip_base_m"][2], .12)
            np.testing.assert_allclose(row["target_tip_base_m"][:2], [.885, -.02])
            np.testing.assert_array_equal(np.asarray(row["T_base_ee_command"])[:3, :3],
                                          np.diag([1., -1., -1.]))

    def test_episode_released_input_removes_offset_even_if_robot_has_not_followed_target(self):
        robot, motion, kin, mouse, tracker, setup, writer, rows = self.episode_fakes()
        actual_pose = robot.pose.copy()
        actual_tip = kin.tip(actual_pose)
        motion.read_state = lambda: {"ee": actual_pose.copy(), "qpos": np.zeros(7),
                                     "qvel": np.zeros(7), "timestamp": tracker.clock.time()}
        mouse.read = lambda: (np.array([.2, -.4, 1.]) if tracker.clock.elapsed < .04 else np.zeros(3), [], .001)
        with mock.patch.object(self.shape, "overlap", side_effect=[.5, .5, 1.]):
            self.assertEqual(pushing.collect_episode(robot, motion, kin, mouse, tracker, self.shape,
                                                     setup, writer, self.args), "success")
        pushed = [row for row in rows if row["elapsed_s"] < .04 - 1e-9]
        released = [row for row in rows if row["elapsed_s"] >= .04 - 1e-9]
        self.assertTrue(pushed)
        self.assertTrue(released)
        for row in pushed:
            np.testing.assert_allclose(row["target_tip_base_m"][:2], actual_tip[:2] + [.01, -.02])
        for row in released:
            np.testing.assert_allclose(row["target_tip_base_m"][:2], actual_tip[:2])
        for row in rows:
            self.assertAlmostEqual(row["target_tip_base_m"][2], .12)
            np.testing.assert_array_equal(np.asarray(row["T_base_ee_command"])[:3, :3], np.diag([1., -1., -1.]))

    def test_episode_exception_stops_control_before_draining_recording_and_closing_devices(self):
        self.args.output = self.directory / "run"
        self.args.episodes = 1
        calibration_file = self.directory / "calibration.json"
        calibration_file.write_text(json.dumps(self.result))
        events = []
        robot = SimpleNamespace(start=lambda: events.append("start"), stop=lambda: events.append("stop"))
        tracker = SimpleNamespace(metadata=self.result["camera"],
                                  attach=lambda writer: events.append("attach" if writer else "detach"))
        camera_context, mouse_context = mock.MagicMock(), mock.MagicMock()
        camera_context.__enter__.return_value = tracker
        camera_context.__exit__.side_effect = lambda *_: events.append("camera closed")
        mouse_context.__enter__.return_value = SimpleNamespace(
            read=mock.Mock(side_effect=AssertionError("No mouse-neutral gate before teleoperation")))
        mouse_context.__exit__.side_effect = lambda *_: events.append("mouse closed")
        motion = SimpleNamespace(plan_start=lambda *_: {"start_q": [0.] * 7},
                                 go_home=lambda *_: events.append("home"),
                                 approach=lambda *_: events.append("approach"),
                                 configure_osc=lambda: events.append("osc"))
        writer = SimpleNamespace(finish=lambda *_: events.append("drain recording"))

        def writer_factory(path, *_args, **_kwargs):
            Path(path).mkdir(parents=True)
            return writer

        initial = self.goal.copy()
        goal_b = self.goal.copy()
        goal_b[1, 3] += .35
        frame = {"image": np.zeros((8, 8, 3), dtype=np.uint8), "record": {"T_base_object": self.goal.tolist()}}
        poses = [(self.goal, frame), (goal_b, frame), (initial, frame)]

        def failed_episode(*_):
            events.append("episode failed")
            raise RuntimeError("synthetic loop failure")

        with mock.patch.dict(sys.modules, {"aiofranka": SimpleNamespace(FrankaRemoteController=lambda *_a, **_k: robot)}), \
                mock.patch.object(pushing.tracking, "CameraTracker", return_value=camera_context), \
                mock.patch.object(pushing.tracking, "MouseReader", return_value=mouse_context), \
                mock.patch.object(pushing.tracking, "capture_target", side_effect=poses) as capture, \
                mock.patch.object(pushing.motion_code, "Motion", return_value=motion) as motion_factory, \
                mock.patch.object(pushing.recording, "EpisodeWriter", side_effect=writer_factory), \
                mock.patch.object(pushing, "sibling", return_value=SimpleNamespace(
                    encode_episode=lambda _, **kwargs: events.append("video encoded"))), \
                mock.patch.object(pushing, "ask_ready", return_value=True), \
                mock.patch.object(pushing, "collect_episode", side_effect=failed_episode):
            with self.assertRaisesRegex(RuntimeError, "synthetic loop failure"):
                pushing.collect(self.args, calibration_file, self.result, self.shape, FakeKinematics())
        self.assertLess(events.index("home"), events.index("approach"))
        self.assertLess(events.index("approach"), events.index("attach"))
        self.assertLess(events.index("attach"), events.index("osc"))
        self.assertLess(events.index("osc"), events.index("episode failed"))
        self.assertEqual(capture.call_count, 3)  # Goals A/B, then the actual initial pose.
        self.assertEqual(motion_factory.call_args.kwargs, {"home_q": self.args.home_q})
        self.assertLess(events.index("stop"), events.index("drain recording"))
        self.assertLess(events.index("stop"), events.index("mouse closed"))
        self.assertLess(events.index("stop"), events.index("camera closed"))
        self.assertLess(events.index("drain recording"), events.index("video encoded"))
        self.assertLess(events.index("camera closed"), events.index("video encoded"))
        self.assertEqual(events.count("video encoded"), 2)
        self.assertEqual(events.count("home"), 1)  # No automatic reset after a fault.
        manifest = json.loads((self.args.output / "run.json").read_text())
        self.assertEqual(manifest["status"], "error")
        self.assertIn("synthetic loop failure", manifest["reason"])

    def test_fixed_starts_are_shared_by_motion_and_recording_and_survive_approach_failure(self):
        for fail_approach in (False, True):
            with self.subTest(fail_approach=fail_approach):
                self.args.output = self.directory / str(fail_approach)
                self.args.episodes = 2
                calibration_file = self.directory / "calibration.json"
                calibration_file.write_text(json.dumps(self.result))
                goal_b = self.goal.copy()
                goal_b[:3, 3] += [0., .35, .004]  # A fixes table height despite B's measurement noise.
                first, second = self.goal.copy(), goal_b.copy()
                first[:2, 3] += [-.04, .03]
                second[:2, 3] += [.05, -.04]
                second[:3, :3] = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]) @ second[:3, :3]
                frames = [{"image": np.zeros((8, 8, 3), dtype=np.uint8),
                           "record": {"T_base_object": pose.tolist(), "sequence": i,
                                      "camera_timestamp_s": 1000. + i}}
                          for i, pose in enumerate((self.goal, goal_b, first, second))]
                captures = [(pose, frame) for pose, frame in zip((self.goal, goal_b, first, second), frames)]
                robot = mock.Mock()
                tracker = SimpleNamespace(metadata=self.result["camera"], attach=mock.Mock())
                camera_context, mouse_context = mock.MagicMock(), mock.MagicMock()
                camera_context.__enter__.return_value = tracker
                motion = mock.Mock()
                motion.plan_start.side_effect = lambda xyz: {"tip": list(xyz), "start_q": [0.] * 7}
                if fail_approach:
                    motion.approach.side_effect = RuntimeError("synthetic approach failure")
                episodes = []

                def collect_episode(_robot, _motion, _kin, _mouse, _tracker, _shape, setup, writer, _args):
                    episodes.append(setup)
                    saved = json.loads((writer.directory / "setup.json").read_text())
                    np.testing.assert_array_equal(saved["start_tip_base_m"], setup["start_tip_base_m"])
                    np.testing.assert_array_equal(writer.metadata["start_tip_base_m"], setup["start_tip_base_m"])
                    np.testing.assert_array_equal(motion.approach.call_args.args[0], setup["start_tip_base_m"])
                    return "success"

                with mock.patch.dict(sys.modules, {"aiofranka": SimpleNamespace(FrankaRemoteController=lambda *_a, **_k: robot)}), \
                        mock.patch.object(pushing.tracking, "CameraTracker", return_value=camera_context), \
                        mock.patch.object(pushing.tracking, "MouseReader", return_value=mouse_context), \
                        mock.patch.object(pushing.tracking, "capture_target", side_effect=captures), \
                        mock.patch.object(pushing.motion_code, "Motion", return_value=motion), \
                        mock.patch.object(pushing, "ask_ready", return_value=True) as ready, \
                        mock.patch.object(pushing, "collect_episode", side_effect=collect_episode), \
                        mock.patch.object(np.random, "default_rng", side_effect=AssertionError("Random initialization is removed")), \
                        mock.patch.object(self.shape, "fixed_start_xy", wraps=self.shape.fixed_start_xy) as fixed_start:
                    if fail_approach:
                        with self.assertRaisesRegex(RuntimeError, "synthetic approach failure"):
                            pushing.collect(self.args, calibration_file, self.result, self.shape, FakeKinematics())
                    else:
                        self.assertEqual(pushing.collect(self.args, calibration_file, self.result, self.shape, FakeKinematics()), 0)
                    self.assertEqual(fixed_start.call_count, 1 if fail_approach else 2)
                    self.assertEqual(ready.call_count, 3)  # Capture A, capture B, return object to A once.
                run_setup = json.loads((self.args.output / "setup.json").read_text())
                self.assertNotIn("start_tip_base_m", run_setup)
                np.testing.assert_array_equal(run_setup["T_base_goals"]["A"], self.goal)
                np.testing.assert_array_equal(run_setup["T_base_goals"]["B"], goal_b)
                np.testing.assert_array_equal(run_setup["T_base_goal"], goal_b)
                self.assertAlmostEqual(run_setup["table_height_m"], .1)
                for label, goal in (("A", self.goal), ("B", goal_b)):
                    self.assertTrue((self.args.output / f"goal_{label}.png").is_file())
                    detection = json.loads((self.args.output / f"goal_{label}_detection.json").read_text())
                    np.testing.assert_array_equal(detection["T_base_object"], goal)
                self.assertEqual(motion.go_home.call_count, 1 if fail_approach else 3)
                self.assertEqual(len(episodes), 0 if fail_approach else 2)
                for index, initial in enumerate((first,) if fail_approach else (first, second), 1):
                    folder = self.args.output / f"episode_{index:04d}"
                    saved = json.loads((folder / "setup.json").read_text())
                    np.testing.assert_array_equal(saved["initial_T_base_object"], initial)
                    np.testing.assert_array_equal(saved["T_base_goal"], goal_b if index == 1 else self.goal)
                    self.assertEqual(saved["source_goal"], "A" if index == 1 else "B")
                    self.assertEqual(saved["target_goal"], "B" if index == 1 else "A")
                    self.assertEqual(saved["direction"], "A_to_B" if index == 1 else "B_to_A")
                    self.assertEqual(saved["initial_detection"]["sequence"], index + 1)
                    self.assertAlmostEqual(saved["table_height_m"], .1)
                    self.assertAlmostEqual(saved["start_tip_base_m"][2], .12)
                    source = self.goal if index == 1 else goal_b
                    np.testing.assert_array_equal(motion.approach.call_args_list[index - 1].args[0],
                                                  saved["start_tip_base_m"])
                    np.testing.assert_array_equal(saved["approach_plan"]["tip"], saved["start_tip_base_m"])
                    self.assertAlmostEqual(self.shape.distance_xy(source, saved["start_tip_base_m"][:2]),
                                           .075, places=6)
                    self.assertAlmostEqual(saved["start_outline_distance_m"], .075, places=6)
                    self.assertNotAlmostEqual(self.shape.distance_xy(initial, saved["start_tip_base_m"][:2]),
                                              .075, places=4)
                    outcome = json.loads((folder / "episode.json").read_text())
                    self.assertEqual(outcome["status"], "error" if fail_approach else "success")

    def test_dependency_failure_closes_camera_and_never_starts_robot(self):
        self.args.output = self.directory / "run"
        calibration_file = self.directory / "calibration.json"
        calibration_file.write_text(json.dumps(self.result))
        camera_context = mock.MagicMock()
        controller = mock.Mock(side_effect=AssertionError("Robot must not be constructed"))
        with mock.patch.dict(sys.modules, {"aiofranka": SimpleNamespace(FrankaRemoteController=controller)}), \
                mock.patch.object(pushing.tracking, "CameraTracker", return_value=camera_context), \
                mock.patch.object(pushing.tracking, "MouseReader", side_effect=RuntimeError("HID unavailable")):
            with self.assertRaisesRegex(RuntimeError, "HID unavailable"):
                pushing.collect(self.args, calibration_file, self.result, self.shape, FakeKinematics())
        controller.assert_not_called()
        camera_context.__exit__.assert_called_once()
        manifest = json.loads((self.args.output / "run.json").read_text())
        self.assertEqual(manifest["status"], "error")
        self.assertIn("HID unavailable", manifest["reason"])


if __name__ == "__main__":
    unittest.main()
