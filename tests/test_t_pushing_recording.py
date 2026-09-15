"""Hardware-free checks for lossless, incremental experiment recording."""

import importlib.util
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest import mock

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

if cv2 is not None:
    script = Path(__file__).resolve().parents[1] / "examples" / "t_pushing_recording.py"
    spec = importlib.util.spec_from_file_location("t_pushing_recording_test", script)
    recording = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(recording)


@unittest.skipIf(cv2 is None, "Recording requires optional OpenCV")
class EpisodeRecordingTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name) / "episode_0000"
        self.image = np.arange(6 * 8 * 3, dtype=np.uint8).reshape(6, 8, 3)

    def read_lines(self, filename):
        return [json.loads(line) for line in (self.directory / filename).read_text().splitlines()]

    def test_roundtrip_preserves_native_image_and_missing_detection(self):
        writer = recording.EpisodeWriter(self.directory, {"goal": np.eye(4)})
        writer.record_state({"q": np.arange(7), "time_s": np.float64(20.0), "overlap": None})
        writer.record_frame(self.image, {"timestamp": 20.1, "tracking": None, "detected": False})
        writer.record_frame(self.image, {"camera_timestamp_s": 20.2, "tracking": {
            "T_base_object": np.eye(4), "corners": np.arange(8).reshape(4, 2),
        }})
        summary = writer.finish("success", "Goal overlap held above threshold")
        self.assertEqual((summary["state_count"], summary["frame_count"]), (1, 2))
        self.assertEqual(summary["unwritten_frame_count"], 0)
        self.assertEqual(summary["metadata"]["goal"], np.eye(4).tolist())
        self.assertEqual(self.read_lines("states.jsonl")[0]["q"], list(range(7)))
        frames = self.read_lines("camera.jsonl")
        self.assertIsNone(frames[0]["tracking"])
        self.assertFalse(frames[0]["detected"])
        self.assertEqual(frames[1]["tracking"]["corners"], [[0, 1], [2, 3], [4, 5], [6, 7]])
        for frame in frames:
            np.testing.assert_array_equal(cv2.imread(str(self.directory / frame["image"])), self.image)
            self.assertEqual(frame["image_shape"], [6, 8, 3])
        self.assertEqual(json.loads((self.directory / "episode.json").read_text())["status"], "success")
        self.assertEqual(writer.finish("success"), summary)
        with self.assertRaisesRegex(recording.RecordingError, "finished"):
            writer.record_state({"time_s": 21.0})

    def test_frame_is_copied_before_control_loop_reuses_buffer(self):
        entered, release = threading.Event(), threading.Event()
        original = recording.EpisodeWriter._save_image

        def delayed(instance, path, image):
            entered.set()
            self.assertTrue(release.wait(3))
            original(instance, path, image)

        expected = self.image.copy()
        with mock.patch.object(recording.EpisodeWriter, "_save_image", delayed):
            writer = recording.EpisodeWriter(self.directory, {})
            try:
                writer.record_frame(self.image, {"timestamp": 1.0})
                self.assertTrue(entered.wait(3))
                self.image[:] = 0
            finally:
                release.set()
                writer.finish("success")
        np.testing.assert_array_equal(cv2.imread(str(self.directory / "rgb/000000.png")), expected)

    def test_jpeg_records_native_shape_quality_and_matching_image_extension(self):
        horizontal = np.linspace(0, 240, 96, dtype=np.uint8)
        vertical = np.linspace(0, 240, 64, dtype=np.uint8)
        image = np.empty((64, 96, 3), dtype=np.uint8)
        image[:, :, 0] = horizontal
        image[:, :, 1] = vertical[:, None]
        image[:, :, 2] = (image[:, :, 0].astype(np.uint16) + image[:, :, 1]) // 2
        with recording.EpisodeWriter(self.directory, {}, image_format="jpeg") as writer:
            writer.record_frame(image, {"timestamp": 1.0, "tracking": None})
        manifest = json.loads((self.directory / "episode.json").read_text())
        self.assertEqual(manifest["image_codec"], "jpeg")
        self.assertEqual(manifest["jpeg_quality"], 95)
        self.assertFalse(manifest["image_lossless"])
        self.assertIsNone(manifest["png_compression_level"])
        self.assertEqual(manifest["frame_count"], 1)
        frame = self.read_lines("camera.jsonl")[0]
        self.assertEqual(frame["image"], "rgb/000000.jpg")
        decoded = cv2.imread(str(self.directory / frame["image"]))
        self.assertEqual(decoded.shape, image.shape)
        difference = np.abs(decoded.astype(float) - image)
        self.assertLess(float(difference.mean()), 3.)
        self.assertLess(float(np.quantile(difference, .99)), 6.)
        self.assertFalse(list((self.directory / "rgb").glob("*.png")))

    def test_invalid_image_format_creates_no_episode(self):
        with self.assertRaisesRegex(ValueError, "image_format"):
            recording.EpisodeWriter(self.directory, {}, image_format="gif")
        self.assertFalse(self.directory.exists())

    def test_duplicate_is_not_saved_and_backward_timestamps_are_rejected(self):
        with recording.EpisodeWriter(self.directory, {}) as writer:
            self.assertTrue(writer.record_frame(self.image, {"timestamp": 2.0}))
            self.assertFalse(writer.record_frame(self.image, {"timestamp": 2.0}))
            with self.assertRaisesRegex(ValueError, "backwards"):
                writer.record_frame(self.image, {"timestamp": 1.0})
            self.assertTrue(writer.record_frame(self.image, {"timestamp": 3.0}))
        manifest = json.loads((self.directory / "episode.json").read_text())
        self.assertEqual(manifest["frame_count"], 2)
        self.assertEqual(manifest["duplicate_frame_count"], 1)
        self.assertEqual([row["timestamp"] for row in self.read_lines("camera.jsonl")], [2.0, 3.0])

    def test_queue_full_is_fatal_but_previously_accepted_records_are_drained(self):
        entered, release = threading.Event(), threading.Event()
        original = recording.EpisodeWriter._save_image

        def delayed(instance, path, image):
            entered.set()
            self.assertTrue(release.wait(3))
            original(instance, path, image)

        with mock.patch.object(recording.EpisodeWriter, "_save_image", delayed):
            writer = recording.EpisodeWriter(self.directory, {}, queue_size=1)
            try:
                writer.record_frame(self.image, {"timestamp": 1.0})
                self.assertTrue(entered.wait(3))
                writer.record_state({"time_s": 1.1})
                with self.assertRaisesRegex(recording.RecordingError, "queue is full"):
                    writer.record_state({"time_s": 1.2})
                with self.assertRaisesRegex(recording.RecordingError, "queue is full"):
                    writer.check()
            finally:
                release.set()
                with self.assertRaises(recording.RecordingError):
                    writer.finish("success")
        manifest = json.loads((self.directory / "episode.json").read_text())
        self.assertEqual(manifest["status"], "error")
        self.assertEqual(manifest["requested_outcome"], "success")
        self.assertEqual((manifest["state_count"], manifest["frame_count"]), (1, 1))
        self.assertEqual(manifest["unwritten_state_count"], 0)
        self.assertEqual(self.read_lines("states.jsonl"), [{"time_s": 1.1}])

    def test_disk_failure_preserves_prior_records_and_reports_unwritten_frames(self):
        entered, release = threading.Event(), threading.Event()

        def broken(*_):
            entered.set()
            self.assertTrue(release.wait(3))
            raise OSError("Synthetic disk full")

        with mock.patch.object(recording.EpisodeWriter, "_save_image", broken):
            writer = recording.EpisodeWriter(self.directory, {})
            try:
                writer.record_state({"time_s": 1.0})
                writer.record_frame(self.image, {"timestamp": 1.1})
                self.assertTrue(entered.wait(3))
                writer.record_frame(self.image, {"timestamp": 1.2})
            finally:
                release.set()
                with self.assertRaisesRegex(recording.RecordingError, "Synthetic disk full"):
                    writer.finish("aborted")
            with self.assertRaisesRegex(recording.RecordingError, "Synthetic disk full"):
                writer.check()
        manifest = json.loads((self.directory / "episode.json").read_text())
        self.assertEqual(manifest["status"], "error")
        self.assertEqual(manifest["state_count"], 1)
        self.assertEqual(manifest["frame_count"], 0)
        self.assertEqual(manifest["unwritten_frame_count"], 2)
        self.assertEqual(self.read_lines("states.jsonl"), [{"time_s": 1.0}])
        self.assertEqual(self.read_lines("camera.jsonl"), [])

    def test_interruption_finalizes_partial_episode_without_masking_exception(self):
        with self.assertRaises(KeyboardInterrupt):
            with recording.EpisodeWriter(self.directory, {}) as writer:
                writer.record_state({"time_s": 1.0})
                raise KeyboardInterrupt("operator stop")
        manifest = json.loads((self.directory / "episode.json").read_text())
        self.assertEqual(manifest["status"], "interrupted")
        self.assertIn("operator stop", manifest["reason"])
        self.assertEqual(manifest["state_count"], 1)
        self.assertFalse(writer._worker.is_alive())

    def test_invalid_data_never_emits_nonstandard_json(self):
        with recording.EpisodeWriter(self.directory, {}) as writer:
            for invalid in (float("nan"), float("inf"), np.array([float("-inf")])):
                with self.subTest(invalid=invalid):
                    with self.assertRaisesRegex(ValueError, "non-finite"):
                        writer.record_state({"measurement": invalid})
            with self.assertRaisesRegex(ValueError, "finite"):
                writer.record_frame(self.image, {"timestamp": float("nan")})
            with self.assertRaisesRegex(ValueError, "BGR"):
                writer.record_frame(self.image.astype(float), {"timestamp": 1.0})
            writer.record_state({"measurement": None})
        self.assertEqual(self.read_lines("states.jsonl"), [{"measurement": None}])
        self.assertEqual(self.read_lines("camera.jsonl"), [])

    def test_existing_episode_is_never_overwritten(self):
        with recording.EpisodeWriter(self.directory, {}) as writer:
            writer.record_state({"preserve": True})
        before = (self.directory / "episode.json").read_bytes()
        with self.assertRaises(FileExistsError):
            recording.EpisodeWriter(self.directory, {"replacement": True})
        self.assertEqual((self.directory / "episode.json").read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
