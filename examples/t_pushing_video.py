"""Make playback MP4s from saved T-pushing frames, without opening any devices.

    python examples/t_pushing_video.py data/t_pushing/RUN
    python examples/t_pushing_video.py data/t_pushing/RUN/episode_0001 --fps 30
    python examples/t_pushing_video.py data/t_pushing/RUN --overlay

Camera timestamps determine playback speed. Output frames hold the preceding
camera image; repeated frames do not represent additional camera observations.
The final camera frame lasts one median camera interval (one output period for
a single image), rounded up to a whole output frame. Existing videos are kept.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import statistics
import tempfile

import cv2


def _records(path):
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
                timestamp = record["camera_timestamp_s"]
                image = record["image"]
                if (isinstance(timestamp, bool) or not isinstance(timestamp, (int, float))
                        or not math.isfinite(timestamp) or not isinstance(image, str) or not image):
                    raise ValueError("expected a finite camera timestamp and image path")
            except (ValueError, KeyError, TypeError) as exc:
                raise ValueError(f"{path}:{line_number}: invalid camera record: {exc}") from exc
            yield record


def _overlay_renderer(directory):
    spec = importlib.util.spec_from_file_location(
        "_t_pushing_overlay", Path(__file__).with_name("t_pushing_overlay.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.AprilCubeOverlay(directory.parent)


def encode_episode(directory: Path, fps=30., *, overlay=False) -> Path | None:
    """Encode existing BGR images, keeping at most two decoded frames in memory.

    Missing/empty camera logs return None. Invalid logs, missing referenced
    images, or codec failures raise errors without changing source recordings.
    Existing videos are returned untouched. overlay=True draws the recorded
    AprilCube result and writes aprilcube_overlay.mp4 instead of video.mp4.
    """
    directory = Path(directory).expanduser().resolve()
    stem = "aprilcube_overlay" if overlay else "video"
    destination = directory / f"{stem}.mp4"
    if destination.exists():
        return destination
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("Video FPS must be positive and finite")
    camera_path = directory / "camera.jsonl"
    if not camera_path.exists():
        return None

    first_timestamp = last_timestamp = None
    intervals, source_count = [], 0
    for record in _records(camera_path):
        timestamp = record["camera_timestamp_s"]
        if last_timestamp is not None:
            if timestamp <= last_timestamp:
                raise ValueError(f"{camera_path}: camera timestamps must increase strictly")
            intervals.append(timestamp - last_timestamp)
        else:
            first_timestamp = timestamp
        last_timestamp = timestamp
        source_count += 1
    if not source_count:
        return None
    draw = _overlay_renderer(directory) if overlay else None
    tail_duration = statistics.median(intervals) if intervals else 1. / fps
    duration = last_timestamp - first_timestamp + tail_duration
    output_count = max(1, math.ceil(duration * fps - 1e-9))

    writer = None
    temporary_video = temporary_metadata = None
    published_video = False
    try:
        with tempfile.NamedTemporaryFile(dir=directory, prefix=".video-", suffix=".mp4", delete=False) as stream:
            temporary_video = Path(stream.name)
        current_image = current_record = None
        width = height = output_index = 0
        frame_mapping = []

        def write_until(end_timestamp):
            nonlocal output_index
            while (output_index < output_count
                   and first_timestamp + output_index / fps < end_timestamp):
                writer.write(current_image)
                frame_mapping.append(current_record["frame_index"])
                output_index += 1

        for source_index, record in enumerate(_records(camera_path)):
            image_path = directory / record["image"]
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                raise OSError(f"Cannot read recorded camera image: {image_path}")
            if current_image is None:
                height, width = image.shape[:2]
                if height % 2 or width % 2:
                    raise ValueError(f"mp4v needs even image dimensions to preserve native size: {width}x{height}")
                writer = cv2.VideoWriter(str(temporary_video), cv2.VideoWriter_fourcc(*"mp4v"),
                                         fps, (width, height))
                if not writer.isOpened():
                    raise RuntimeError("OpenCV could not open the mp4v video encoder")
            elif image.shape[:2] != (height, width):
                raise ValueError(f"Recorded camera dimensions changed at {image_path}")
            else:
                write_until(record["camera_timestamp_s"])
            current_image = draw(image, record) if draw is not None else image
            current_record = {**record, "frame_index": record.get("frame_index", source_index)}
        # Integer output_count sets the rounded playback duration explicitly.
        while output_index < output_count:
            writer.write(current_image)
            frame_mapping.append(current_record["frame_index"])
            output_index += 1
        writer.release()
        writer = None
        if temporary_video.stat().st_size == 0:
            raise OSError("OpenCV produced an empty video")
        metadata = {
            "schema_version": 1, "video": destination.name, "codec": "mp4v", "fps": float(fps),
            "width": width, "height": height,
            "start_timestamp_s": first_timestamp,
            "end_timestamp_s": first_timestamp + output_count / fps,
            "source_last_timestamp_s": last_timestamp,
            "final_source_frame_duration_s": tail_duration,
            "duration_s": output_count / fps,
            "source_frame_count": source_count, "output_frame_count": output_count,
            "output_frame_to_camera_frame_index": frame_mapping,
            "resampling": "At each output tick, hold the latest camera image at or before that tick",
            "duration_rule": "Last camera timestamp plus median camera interval; one output period for a single image; round up to whole output frames",
            "camera_clock": "PyCAAS driver-read completion, not hardware exposure",
            "source": "camera.jsonl and native recorded images; no additional camera observations",
        }
        if draw is not None:
            metadata["overlay"] = draw.metadata
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=directory,
                                         prefix=".video-", suffix=".json", delete=False) as stream:
            temporary_metadata = Path(stream.name)
            json.dump(metadata, stream, indent=2, allow_nan=False)
            stream.write("\n")
        # Exclusive publication also preserves a video created by another export.
        try:
            os.link(temporary_video, destination)
        except FileExistsError:
            return destination
        published_video = True
        os.replace(temporary_metadata, directory / f"{stem}.json")
        return destination
    except BaseException as exc:
        if published_video:
            destination.unlink(missing_ok=True)
        if isinstance(exc, cv2.error):
            raise RuntimeError(f"OpenCV video export failed for {directory}: {exc}") from exc
        raise
    finally:
        if writer is not None:
            writer.release()
        for path in (temporary_video, temporary_metadata):
            if path is not None:
                path.unlink(missing_ok=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("directory", type=Path, help="A T-pushing run or episode directory")
    parser.add_argument("--fps", type=float, default=30., help="Playback frame rate (default: 30)")
    parser.add_argument("--overlay", action="store_true", help="Render saved AprilCube results as aprilcube_overlay.mp4")
    args = parser.parse_args(argv)
    if not math.isfinite(args.fps) or args.fps <= 0:
        parser.error("--fps must be positive and finite")
    root = args.directory.expanduser().resolve()
    if not root.is_dir():
        parser.error(f"Directory does not exist: {root}")
    episodes = sorted(path for path in root.glob("episode_*") if path.is_dir())
    if not episodes:
        episodes = [root]
    encoded = kept = skipped = failed = 0
    for episode in episodes:
        existed = (episode / ("aprilcube_overlay.mp4" if args.overlay else "video.mp4")).exists()
        try:
            result = encode_episode(episode, args.fps, overlay=args.overlay)
            if result is None:
                skipped += 1
            elif existed:
                kept += 1
                print(f"Kept: {result}")
            else:
                encoded += 1
                print(f"Saved: {result}")
        except (OSError, RuntimeError, ValueError, cv2.error) as exc:
            failed += 1
            print(f"Video export failed for {episode}: {exc}")
    print(f"Videos: {encoded} encoded, {kept} existing, {skipped} empty, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
