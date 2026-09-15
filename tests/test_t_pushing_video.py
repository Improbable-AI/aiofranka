"""Offline video export checks using tiny saved images and camera timestamps."""

from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

try:
    import cv2
except ImportError:
    available = False
else:
    available = True
    path = Path(__file__).resolve().parents[1] / "examples/t_pushing_video.py"
    spec = importlib.util.spec_from_file_location("t_pushing_video_test", path)
    video = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(video)


@unittest.skipUnless(available, "Video export requires OpenCV")
class TPushingVideoTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.episode = self.directory / "episode_0001"
        (self.episode / "rgb").mkdir(parents=True)

    def save_frames(self, timestamps=(1000., 1000.1, 1000.4)):
        records = []
        for index, timestamp in enumerate(timestamps):
            image = np.zeros((32, 48, 3), dtype=np.uint8)
            image[:, :, index % 3] = 240
            relative = f"rgb/{index:06d}.png"
            self.assertTrue(cv2.imwrite(str(self.episode / relative), image))
            records.append({"camera_timestamp_s": timestamp, "image": relative,
                            "frame_index": index + 4})
        self.save_records(records)
        return records

    def save_records(self, records):
        (self.episode / "camera.jsonl").write_text("".join(json.dumps(row) + "\n" for row in records))

    def assert_clean_failure(self):
        self.assertFalse((self.episode / "video.mp4").exists())
        self.assertFalse((self.episode / "video.json").exists())
        self.assertEqual(list(self.episode.glob(".video-*")), [])

    def test_unequal_camera_gaps_preserve_playback_timing_and_frame_mapping(self):
        self.save_frames()
        original = {path: path.read_bytes() for path in self.episode.rglob("*") if path.is_file()}
        destination = video.encode_episode(self.episode, fps=10.)
        metadata = json.loads((self.episode / "video.json").read_text())
        self.assertEqual(destination, self.episode / "video.mp4")
        self.assertEqual(metadata["source_frame_count"], 3)
        self.assertEqual(metadata["output_frame_count"], 6)
        self.assertEqual(metadata["output_frame_to_camera_frame_index"], [4, 5, 5, 5, 6, 6])
        self.assertAlmostEqual(metadata["duration_s"], .6)
        self.assertAlmostEqual(metadata["final_source_frame_duration_s"], .2)
        self.assertAlmostEqual(metadata["start_timestamp_s"], 1000.)
        self.assertAlmostEqual(metadata["end_timestamp_s"], 1000.6)
        self.assertAlmostEqual(metadata["source_last_timestamp_s"], 1000.4)
        self.assertEqual((metadata["width"], metadata["height"]), (48, 32))
        capture = cv2.VideoCapture(str(destination))
        try:
            decoded = []
            while True:
                ok, image = capture.read()
                if not ok:
                    break
                decoded.append(int(image.mean(axis=(0, 1)).argmax()))
            self.assertAlmostEqual(capture.get(cv2.CAP_PROP_FPS), 10.)
        finally:
            capture.release()
        self.assertEqual(decoded, [0, 1, 1, 1, 2, 2])
        for path, payload in original.items():
            self.assertEqual(path.read_bytes(), payload)

    def test_single_camera_frame_gets_one_output_period_and_existing_video_is_kept(self):
        self.save_frames((1000.,))
        destination = video.encode_episode(self.episode)
        original_video = destination.read_bytes()
        original_metadata = (self.episode / "video.json").read_bytes()
        metadata = json.loads(original_metadata)
        self.assertEqual(metadata["output_frame_count"], 1)
        self.assertAlmostEqual(metadata["duration_s"], 1 / 30)
        with mock.patch.object(video.cv2, "VideoWriter", side_effect=AssertionError("existing video replaced")):
            self.assertEqual(video.encode_episode(self.episode, fps=60.), destination)
        self.assertEqual(destination.read_bytes(), original_video)
        self.assertEqual((self.episode / "video.json").read_bytes(), original_metadata)

    def test_overlay_uses_same_timeline_and_preserves_existing_plain_video(self):
        self.save_frames()
        video.encode_episode(self.episode, fps=10.)
        plain = (self.episode / "video.mp4").read_bytes()
        timing = json.loads((self.episode / "video.json").read_text())
        drawn_sources = []

        def annotate(image, record):
            drawn_sources.append(record["frame_index"])
            return 255 - image

        renderer = mock.Mock(side_effect=annotate)
        renderer.metadata = {"detector_rerun": False}
        with mock.patch.object(video, "_overlay_renderer", return_value=renderer):
            path = video.encode_episode(self.episode, fps=10., overlay=True)
        self.assertEqual(path.name, "aprilcube_overlay.mp4")
        self.assertEqual(drawn_sources, [4, 5, 6])  # Draw once per camera frame, then hold.
        overlay = json.loads((self.episode / "aprilcube_overlay.json").read_text())
        for key in ("start_timestamp_s", "duration_s", "output_frame_count",
                    "output_frame_to_camera_frame_index"):
            self.assertEqual(overlay[key], timing[key])
        self.assertFalse(overlay["overlay"]["detector_rerun"])
        self.assertEqual((self.episode / "video.mp4").read_bytes(), plain)
        original_overlay = path.read_bytes()
        with mock.patch.object(video, "_overlay_renderer", side_effect=AssertionError("existing overlay redrawn")):
            self.assertEqual(video.encode_episode(self.episode, overlay=True), path)
        self.assertEqual(path.read_bytes(), original_overlay)

    def test_missing_and_empty_camera_logs_skip_without_outputs(self):
        self.assertIsNone(video.encode_episode(self.episode))
        (self.episode / "camera.jsonl").write_text("\n")
        self.assertIsNone(video.encode_episode(self.episode))
        self.assert_clean_failure()

    def test_invalid_timestamps_and_missing_images_do_not_leave_partial_video(self):
        records = self.save_frames()
        for timestamp in (1000., float("nan"), "invalid"):
            with self.subTest(timestamp=timestamp):
                broken = [dict(row) for row in records]
                broken[1]["camera_timestamp_s"] = timestamp
                self.save_records(broken)
                with self.assertRaisesRegex(ValueError, "timestamp|camera record"):
                    video.encode_episode(self.episode)
                self.assert_clean_failure()
        self.save_records(records)
        (self.episode / records[1]["image"]).unlink()
        with self.assertRaisesRegex(OSError, "Cannot read recorded camera image"):
            video.encode_episode(self.episode)
        self.assert_clean_failure()

    def test_encoder_failure_releases_writer_and_cleans_temporary_files(self):
        self.save_frames()
        encoder = mock.Mock()
        encoder.isOpened.return_value = True
        encoder.write.side_effect = RuntimeError("synthetic encoder failure")
        with mock.patch.object(video.cv2, "VideoWriter", return_value=encoder):
            with self.assertRaisesRegex(RuntimeError, "synthetic encoder failure"):
                video.encode_episode(self.episode)
        encoder.release.assert_called_once()
        self.assert_clean_failure()
        encoder.reset_mock()
        encoder.isOpened.return_value = False
        with mock.patch.object(video.cv2, "VideoWriter", return_value=encoder):
            with self.assertRaisesRegex(RuntimeError, "could not open"):
                video.encode_episode(self.episode)
        encoder.release.assert_called_once()
        self.assert_clean_failure()

    def test_export_rejects_dimension_changes_or_odd_sizes_instead_of_cropping(self):
        records = self.save_frames()
        cv2.imwrite(str(self.episode / records[1]["image"]), np.zeros((34, 48, 3), dtype=np.uint8))
        with self.assertRaisesRegex(ValueError, "dimensions changed"):
            video.encode_episode(self.episode)
        self.assert_clean_failure()
        records = self.save_frames((1000.,))
        cv2.imwrite(str(self.episode / records[0]["image"]), np.zeros((31, 47, 3), dtype=np.uint8))
        with self.assertRaisesRegex(ValueError, "preserve native size"):
            video.encode_episode(self.episode)
        self.assert_clean_failure()

    def test_cli_exports_run_and_skips_empty_episodes(self):
        self.save_frames((1000.,))
        (self.directory / "episode_0002").mkdir()
        output = io.StringIO()
        with redirect_stdout(output):
            self.assertEqual(video.main([str(self.directory), "--fps", "10"]), 0)
            self.assertEqual(video.main([str(self.episode)]), 0)
        self.assertIn("1 encoded, 0 existing, 1 empty, 0 failed", output.getvalue())
        self.assertIn("0 encoded, 1 existing, 0 empty, 0 failed", output.getvalue())


if __name__ == "__main__":
    unittest.main()
