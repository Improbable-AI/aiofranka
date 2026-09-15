"""Exact T footprint and support/tool geometry using generated fixtures only."""

import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

try:
    import cv2
except ImportError as exc:
    raise unittest.SkipTest("T pushing geometry requires optional OpenCV") from exc

SOURCE = Path(__file__).parents[1] / "examples/t_pushing_geometry.py"
SPEC = importlib.util.spec_from_file_location("t_pushing_geometry", SOURCE)
geometry = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(geometry)


def flat_pose(yaw=0, opposite_face=False):
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("z", yaw, degrees=True).as_matrix() @ Rotation.from_euler(
        "x", -90 if opposite_face else 90, degrees=True).as_matrix()
    pose[:3, 3] = [0, 0, .432]
    return pose


class TGeometryTest(unittest.TestCase):
    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        config = {"target": {"type": "voxel_cuboids", "voxel_size_mm": 64,
                             "cuboids": [{"origin": [1, 0, 0], "size": [1, 1, 4]},
                                         {"origin": [0, 0, 3], "size": [3, 1, 1]}]},
                  "box_dims": [192, 64, 256]}
        config_path = self.directory / "config.json"
        config_path.write_text(json.dumps(config))
        self.shape = geometry.TShape(config_path)

    def test_overlapping_cuboids_become_six_disjoint_voxels_and_correct_bounds(self):
        self.assertEqual(len(self.shape.cells), 6)
        self.assertAlmostEqual(self.shape.area_m2, 6 * .064**2)
        np.testing.assert_allclose(self.shape.corners_m.min(axis=0), [-.096, -.032, -.128])
        np.testing.assert_allclose(self.shape.corners_m.max(axis=0), [.096, .032, .128])

    def test_both_flat_y_faces_give_same_table_height(self):
        for opposite in (False, True):
            pose = flat_pose(yaw=37, opposite_face=opposite)
            self.assertAlmostEqual(self.shape.support_height(pose), .4)

    def test_flat_placement_height_does_not_reject_orientation_noise(self):
        for angle in (-4, 0, 4):
            pose = flat_pose()
            pose[:3, :3] = Rotation.from_euler("x", angle, degrees=True).as_matrix() @ pose[:3, :3]
            self.assertAlmostEqual(self.shape.support_height(pose), .4)

    def test_overlap_retains_concavity_for_translation_and_half_turn(self):
        goal = flat_pose()
        actual = goal.copy()
        self.assertAlmostEqual(self.shape.overlap(goal, actual), 1, places=6)
        actual[0, 3] += .064
        self.assertAlmostEqual(self.shape.overlap(goal, actual), 2 / 6, places=6)
        self.assertAlmostEqual(self.shape.overlap(goal, flat_pose(yaw=180)), 4 / 6, places=6)
        actual[0, 3] += .3
        self.assertEqual(self.shape.overlap(goal, actual), 0)

    def test_projected_overlap_accepts_lift_and_tilt_without_runtime_plane_checks(self):
        goal = flat_pose()
        lifted = goal.copy()
        lifted[2, 3] += .2
        self.assertAlmostEqual(self.shape.overlap(goal, lifted), 1., places=6)
        tilted = goal.copy()
        tilted[:3, :3] = Rotation.from_euler("x", 10, degrees=True).as_matrix() @ goal[:3, :3]
        points = np.zeros((*self.shape.rectangles_xz.shape[:2], 3))
        points[:, :, 0] = self.shape.rectangles_xz[:, :, 0]
        points[:, :, 2] = self.shape.rectangles_xz[:, :, 1]
        expected = (points @ tilted[:3, :3].T + tilted[:3, 3])[:, :, :2]
        np.testing.assert_allclose(self.shape.footprint(tilted), expected)
        self.assertTrue(0. < self.shape.overlap(goal, tilted) <= 1.)

    def test_distance_distinguishes_concave_gap_from_crossbar(self):
        pose = flat_pose()
        self.assertAlmostEqual(self.shape.distance_xy(pose, [.064, .032]), .032, places=6)
        self.assertEqual(self.shape.distance_xy(pose, [.064, -.096]), 0)
        self.assertAlmostEqual(self.shape.distance_xy(pose, [.16, -.096]), .064, places=6)
        self.assertEqual(self.shape.distance_xy(pose, [0, 0]), 0)

    def test_fixed_start_repeats_exactly_and_stays_away_from_destination(self):
        source, destination = flat_pose(), flat_pose(yaw=43)
        source[:2, 3] = [.55, -.22]
        destination[:2, 3] = [.9, .08]
        before_source, before_destination = source.copy(), destination.copy()
        point = self.shape.fixed_start_xy(source, destination)
        for _ in range(5):
            np.testing.assert_array_equal(self.shape.fixed_start_xy(source, destination), point)
        outward = source[:2, 3] - destination[:2, 3]
        outward /= np.linalg.norm(outward)
        projected_outline = self.shape.footprint(source).reshape(-1, 2) @ outward
        self.assertAlmostEqual(float(point @ outward - projected_outline.max()), .075)
        self.assertAlmostEqual(self.shape.distance_xy(source, point), .075, places=7)
        np.testing.assert_array_equal(source, before_source)
        np.testing.assert_array_equal(destination, before_destination)

    def test_fixed_start_rotates_and_translates_with_saved_goal_pair(self):
        source, destination = flat_pose(), flat_pose(yaw=-12)
        destination[:2, 3] = [.31, -.22]  # Oblique direction gives one support vertex.
        original = self.shape.fixed_start_xy(source, destination, clearance=.061)
        global_pose = np.eye(4)
        global_pose[:3, :3] = Rotation.from_euler("z", 37, degrees=True).as_matrix()
        global_pose[:3, 3] = [.65, -.3, .12]
        moved_source, moved_destination = global_pose @ source, global_pose @ destination
        actual = self.shape.fixed_start_xy(moved_source, moved_destination, clearance=.061)
        expected = global_pose[:2, :2] @ original + global_pose[:2, 3]
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        self.assertAlmostEqual(self.shape.distance_xy(moved_source, actual), .061, places=7)

    def test_fixed_start_has_exact_clearance_from_the_concave_outline_in_all_directions(self):
        for yaw in (0, 37, 143):
            source = flat_pose(yaw=yaw)
            source[:2, 3] = [.65, -.3]
            rotation_xy = Rotation.from_euler("z", yaw, degrees=True).as_matrix()[:2, :2]
            for angle in np.linspace(0., 2. * np.pi, 13)[:-1]:
                destination = source.copy()
                destination[:2, 3] += .4 * np.array([np.cos(angle), np.sin(angle)])
                point = self.shape.fixed_start_xy(source, destination)
                reference_xy = (point - source[:2, 3]) @ rotation_xy
                # Independent distance to the union of the axis-aligned stem and bar.
                stem = np.linalg.norm(np.maximum(np.abs(reference_xy) - [.032, .128], 0))
                bar = np.linalg.norm(np.maximum(np.abs(reference_xy - [0., -.096]) - [.096, .032], 0))
                self.assertAlmostEqual(min(stem, bar), .075, places=12)

    def test_fixed_start_same_xy_uses_negative_base_y_even_if_goals_differ_in_orientation_or_z(self):
        source, destination = flat_pose(), flat_pose(yaw=93)
        destination[2, 3] += .25
        point = self.shape.fixed_start_xy(source, destination)
        self.assertAlmostEqual(point[1], -.128 - .075)
        self.assertAlmostEqual(self.shape.distance_xy(source, point), .075, places=7)
        destination[0, 3] += 5e-10
        np.testing.assert_array_equal(self.shape.fixed_start_xy(source, destination), point)

    def test_fixed_start_rejects_invalid_clearance_and_either_invalid_transform(self):
        for clearance in (0., -.01, np.nan, np.inf):
            with self.assertRaisesRegex(ValueError, "positive and finite"):
                self.shape.fixed_start_xy(flat_pose(), flat_pose(), clearance)
        invalid = flat_pose()
        invalid[0, 0] = 2.
        for source, destination in ((invalid, flat_pose()), (flat_pose(), invalid)):
            with self.assertRaisesRegex(ValueError, "rigid"):
                self.shape.fixed_start_xy(source, destination)

    def test_sampled_starts_are_five_to_ten_cm_from_the_actual_outline(self):
        rng = np.random.default_rng(42)
        xy = np.array([self.shape.sample_start_xy(flat_pose(), rng) for _ in range(500)])
        # Independent distances to the two rectangles of the axis-aligned T.
        stem_delta = np.maximum(np.abs(xy) - [.032, .128], 0)
        bar_delta = np.maximum(np.abs(xy - [0, -.096]) - [.096, .032], 0)
        distance = np.minimum(np.linalg.norm(stem_delta, axis=1),
                              np.linalg.norm(bar_delta, axis=1))
        self.assertTrue(np.all(distance > .05))
        self.assertTrue(np.all(distance <= .10 + 1e-7))
        self.assertTrue(np.any(distance < .055))
        self.assertTrue(np.any(distance > .095))

    def test_sampled_starts_follow_relocated_rotated_object_and_seed(self):
        pose = flat_pose(yaw=37)
        pose[:2, 3] = [.65, -.3]
        rng = np.random.default_rng(5)
        samples = np.array([self.shape.sample_start_xy(pose, rng, .07, .02) for _ in range(100)])
        local_xz = (samples - pose[:2, 3]) @ np.linalg.inv(pose[:2, [0, 2]].T)
        # Map to base XY of the unrotated reference, whose local +Z is base -Y.
        reference_xy = local_xz * [1, -1]
        distances = [self.shape.distance_xy(flat_pose(), xy) for xy in reference_xy]
        self.assertTrue(np.all(np.asarray(distances) > .02))
        self.assertLessEqual(max(distances), .07 + 1e-7)
        np.testing.assert_array_equal(samples[0], self.shape.sample_start_xy(pose, np.random.default_rng(5), .07, .02))
        for minimum, maximum in ((0, 0), (-.1, .2), (.2, .2), (.2, .1), (np.nan, .2), (.1, np.inf)):
            with self.assertRaisesRegex(ValueError, "0 <= min < max"):
                self.shape.sample_start_xy(pose, rng, maximum, minimum)

    def test_invalid_rigid_transform_rejected(self):
        pose = flat_pose()
        pose[:3, 0] *= -1
        with self.assertRaisesRegex(ValueError, "rigid"):
            self.shape.support_height(pose)

    def test_tool_site_and_geometric_apex_are_distinct(self):
        (self.directory / "stick.obj").write_text("v 0 0 .156\nv .015 0 .141\nv 0 .015 .141\n")
        xml = self.directory / "stick.xml"
        xml.write_text('<mujoco><compiler meshdir="."/><asset><mesh name="mesh" file="stick.obj"/></asset>'
                       '<worldbody><body name="stick"><geom name="stick_visual" mesh="mesh" pos="0 0 .005"/>'
                       '<site name="stick_tip" pos="0 0 .16"/><geom name="stick_collision_sphere" '
                       'size=".015" pos="0 0 .146"/></body></worldbody></mujoco>')
        metadata = geometry.load_stick_tip(xml)
        np.testing.assert_allclose(metadata["geometry_tip_m"], [0, 0, .161])
        np.testing.assert_allclose(metadata["site_tip_m"], [0, 0, .16])
        self.assertAlmostEqual(metadata["discrepancy_m"], .001)
        self.assertAlmostEqual(metadata["contact_radius_m"], .015)


if __name__ == "__main__":
    unittest.main()
