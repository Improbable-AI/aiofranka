#!/usr/bin/env python3
"""Collect T-pushing episodes with PyCAAS, SpaceMouse, and Franka OSC.

    python examples/14_collect_t_pushing.py collect --condition ground
    python examples/14_collect_t_pushing.py collect --condition elevated
    python examples/14_collect_t_pushing.py collect --resume data/t_pushing/RUN

Place the T-block at the goal and press Enter to fix the goal and table height.
Each episode moves home, waits for you to relocate the block, then samples a
start uniformly 5–10 cm outside its outline. All
automatic moves use joint space. --seed makes the random draws reproducible.
SpaceMouse X/Y adds --scale (default 0.05 m) times the latest input to measured
base X/Y. The tip stays commanded 2 cm above the table with the stick pointing
down; Z and rotation input are ignored. The direct-mount tip offset is 0.161 m.

An episode succeeds as soon as goal footprint coverage exceeds 85%.
Missing AprilCube poses are recorded; teleoperation continues without goal-success checks.
Ctrl+C preserves partial recordings. Each run saves its calibration, goal, robot
states, camera poses, and native RGB frames. PyCAAS timestamps mark driver-read
completion. Start PyCAAS and enable robot control separately before collecting.
RGB and AprilCube overlay MP4s are encoded after the robot stops (--no-video to skip).

`inspect` (the default) and `plan --goal-pose setup.json` open no devices.
`--resume RUN` reuses its saved settings, calibration, goal, and table height,
and appends new episode directories after every existing episode.
"""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import time

# This process also forks the 1 kHz server. Set native-library limits before
# importing NumPy/SciPy so its small control matrices do not spawn worker pools.
for _thread_variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_thread_variable] = "1"

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def sibling(filename, name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(filename))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


camera_code = sibling("camera_calibration_capture.py", "_t_pushing_camera")
geometry = sibling("t_pushing_geometry.py", "_t_pushing_geometry")
motion_code = sibling("t_pushing_motion.py", "_t_pushing_motion")
tracking = sibling("t_pushing_tracking.py", "_t_pushing_tracking")
recording = sibling("t_pushing_recording.py", "_t_pushing_recording")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, data):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(recording.json_value(data), indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def load_calibration(path=None, root=ROOT):
    """Load the selected calibration, or the newest completed session."""
    if path is None:
        candidates = sorted((root / "camera_calibration").glob("*/calibration.json"), key=lambda p: p.parent.name)
        if not candidates:
            raise ValueError("No completed camera calibration found; pass --calibration")
        path = candidates[-1]
    path = Path(path).expanduser().resolve()
    result = json.loads(path.read_text())
    if result.get("setup") != "fixed_camera_robot_held_cube" or result.get("schema_version") != 1:
        raise ValueError("Expected the fixed-camera calibration from example 12")
    geometry.rigid_transform(result["T_base_camera"])
    return path, result


def validate_camera(metadata, result):
    expected = result["camera"]
    for key in ("serial", "width", "height"):
        if metadata.get(key) != expected.get(key):
            raise ValueError(f"PyCAAS {key} differs from the selected calibration")
    for key, saved in (("camera_matrix", result["camera_matrix"]), ("dist_coeffs", result["dist_coeffs"])):
        actual = np.asarray(metadata[key])
        if actual.shape != np.asarray(saved).shape or not np.allclose(actual, saved, rtol=1e-5, atol=1e-7):
            raise ValueError(f"PyCAAS {key} differs from the selected calibration; restore the calibrated profile")


def make_setup(args, result, shape, goal):
    height = shape.support_height(goal)
    return {"T_base_goal": np.asarray(goal).tolist(), "table_height_m": height,
            "tip_offset_ee_m": args.tip_offset, "tip_height_m": args.tip_height,
            "R_base_ee": motion_code.DOWNWARD.tolist(), "condition": args.condition,
            "osc_gains": motion_code.OSC_GAINS, "home_q": args.home_q,
            "spacemouse_control": spacemouse_control(args),
            "success_metric": "intersection_area / goal_T_footprint_area",
            "success_threshold": args.success_threshold,
            "start_sampling": {"method": "uniform_area_outside_T_outline",
                               "min_distance_m": args.start_min_distance,
                               "max_distance_m": args.start_distance, "seed": args.seed}}


def make_episode_setup(args, setup, shape, initial_object, rng):
    """Sample once around the relocated T; preserve the run's goal and height."""
    xy = shape.sample_start_xy(initial_object, rng, max_distance=args.start_distance,
                               min_distance=args.start_min_distance)
    start = np.r_[xy, setup["table_height_m"] + args.tip_height]
    return {**setup, "initial_T_base_object": np.asarray(initial_object).tolist(),
            "start_tip_base_m": start.tolist(),
            "start_outline_distance_m": shape.distance_xy(initial_object, xy)}


def spacemouse_control(args):
    return {"mode": "measured_tip_offset", "axis_convention": "pyspacemouse LEGACY",
            "mapping": "event.x/y -> base X/Y; ignore z and rotation",
            "xy_scale_m_per_unit": [args.scale, args.scale], "scale_source": "fixed --scale",
            "deadzone": args.deadzone,
            "rule": "measured_tip_xy + clip(input_xy, -1, 1) * xy_scale",
            "input_policy": "drain all pending HID reports before publishing latest state",
            "reference": "examples/02_spacemouse_teleop.py measured-pose update, with a fixed scale"}


def osc_xy_target(actual, axes, scale, deadzone=0.0):
    """Existing OSC teleop update restricted to XY.

    There is no previous-target term, dt multiplier, or diagonal normalization.
    Neutral input removes the input offset in this same update. An optional
    deadzone only suppresses small axes; the default matches example 02 (none).
    """
    command = np.clip(np.asarray(axes, dtype=float)[:2], -1., 1.)
    command[np.abs(command) <= deadzone] = 0.
    return np.asarray(actual) + command * np.asarray(scale), command


def collect_episode(robot, motion, kin, mouse, tracker, shape, setup, writer, args):
    """Only this phase records demonstration commands; moves/reset are excluded."""
    target_xy = np.asarray(setup["start_tip_base_m"])[:2].copy()
    goal = np.asarray(setup["T_base_goal"])
    target_z = setup["table_height_m"] + args.tip_height
    xy_scale = np.asarray(setup["spacemouse_control"]["xy_scale_m_per_unit"])
    began = time.monotonic()
    last_print = 0.0
    while True:
        now = time.monotonic()
        writer.check()
        state = motion.read_state()
        tip = kin.tip(state["ee"])
        snapshot = tracker.snapshot()
        observation = tracking.native_record(snapshot)
        axes, buttons, mouse_age = mouse.read()
        overlap = shape.overlap(goal, observation["T_base_object"]) if observation else None
        target_xy, command = osc_xy_target(tip[:2], axes, xy_scale, args.deadzone)
        command_pose = kin.ee_for_tip(np.r_[target_xy, target_z])
        command_time = time.time()
        robot.ee_desired = command_pose
        stamp = time.time()
        writer.record_state({
            "timestamp_s": stamp, "elapsed_s": now - began, "phase": "teleop",
            "robot_timestamp_s": float(state["timestamp"]),
            "qpos": state["qpos"], "qvel": state["qvel"], "T_base_ee": state["ee"],
            "tip_base_m": tip, "target_tip_base_m": kin.tip(command_pose),
            "command_timestamp_s": command_time, "T_base_ee_command": command_pose,
            "last_torque": state.get("last_torque"), "mass_matrix": state.get("mm"), "jacobian": state.get("jac"),
            "spacemouse_axes": axes, "spacemouse_buttons": buttons, "spacemouse_report_age_s": mouse_age,
            "command_xy_normalized": command, "goal_coverage": overlap,
            "camera_sequence": snapshot["record"]["sequence"] if snapshot else None,
            "camera_timestamp_s": snapshot["record"]["camera_timestamp_s"] if snapshot else None,
            "tracking_valid": observation is not None,
            "tracking_predicted": observation["predicted"] if observation and "predicted" in observation else None,
            "T_base_object": observation["T_base_object"] if observation else None,
        })
        success = overlap is not None and overlap > args.success_threshold
        if success or now - last_print >= 1:
            print(f"\r{now - began:6.1f}s | goal coverage: {100 * overlap:5.1f}%" if overlap is not None
                  else "\rTracking unavailable: teleop continues           ", end="", flush=True)
            last_print = now
        if success or now - began >= args.episode_seconds:
            motion.hold()
            print()
            return "success" if success else "timeout"
        time.sleep(max(0., 1 / args.frequency - (time.monotonic() - now)))


def ask_ready(message):
    if sys.stdin.isatty():
        import termios
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    return input(message + " [Enter to continue, q to stop] ").strip().lower() != "q"


def next_index(run, prefix):
    numbers = [int(path.stem[len(prefix):]) for path in run.glob(prefix + "*")
               if path.stem[len(prefix):].isdigit()]
    return max(numbers, default=0) + 1


def collect(args, calibration_path, result, shape, kin):
    # Imports are lazy; inspect/plan/tests cannot start a controller or HID reader.
    # Running `python examples/...` otherwise resolves an older installed copy.
    sys.path.insert(0, str(ROOT))
    import aiofranka
    from aiofranka import FrankaRemoteController

    run = (args.resume or args.output or ROOT / "data/t_pushing" / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + args.condition)).expanduser().resolve()
    setup = None
    if args.resume:
        saved = json.loads((run / "setup.json").read_text())
        setup = {**saved, **make_setup(args, result, shape, saved["T_base_goal"]),
                 "table_height_m": saved["table_height_m"], "osc_gains": saved["osc_gains"]}
        manifest_path = run / f"resume_{next_index(run, 'resume_'):04d}.json"
    else:
        run.mkdir(parents=True, exist_ok=False)
        shutil.copyfile(calibration_path, run / "calibration.json")
        shutil.copyfile(args.target_config, run / "target_config.json")
        manifest_path = run / "run.json"
    first_episode = next_index(run, "episode_")
    # Resuming a seeded run starts a distinct, reproducible random stream.
    random_seed = [args.seed, first_episode] if args.resume and args.seed is not None else args.seed
    manifest = {"schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
                "condition": args.condition, "calibration_source": str(calibration_path),
                "calibration_sha256": digest(calibration_path), "target_config_sha256": digest(args.target_config),
                "robot_model_sha256": digest(args.robot_xml), "stick_xml_sha256": digest(args.stick_xml),
                "stick_geometry": geometry.load_stick_tip(args.stick_xml),
                "tool_mount": "Direct to attachment_site; tip offset confirmed by operator",
                "arguments": vars(args), "status": "setting_up", "first_episode": first_episode,
                "random_seed": random_seed,
                "aiofranka_source": getattr(aiofranka, "__file__", None),
                "numerical_threads": {name: os.environ.get(name) for name in
                                      ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")},
                "timing": "Camera driver-read completion and robot-state wall clocks; not synchronized exposures",
                "spacemouse_control": spacemouse_control(args)}
    # Reserve a new attempt manifest; never replace an earlier run/resume file.
    with manifest_path.open("x") as stream:
        json.dump(recording.json_value(manifest), stream, indent=2, allow_nan=False)
    print(f"Run: {run}\nUsing calibration: {calibration_path}")
    if args.resume:
        print(f"Resuming at episode {first_episode:04d}; reusing the saved goal and table height.")
    print(f"Control package: {manifest['aiofranka_source']}")
    print("SpaceMouse: measured XY + scaled input; scale "
          f"{manifest['spacemouse_control']['xy_scale_m_per_unit']} m/unit at {args.frequency:g} Hz")
    rng = np.random.default_rng(random_seed)
    robot = None
    try:
        def detector(metadata):
            validate_camera(metadata, result)
            return tracking.create_detector(args.target_config, metadata, result["T_base_camera"])

        with ExitStack() as stack:
            tracker = stack.enter_context(tracking.CameraTracker(
                lambda: camera_code.PycaasCamera(args), detector))
            mouse = stack.enter_context(tracking.MouseReader())  # Fail before robot connection if unavailable.
            if setup is None:
                if not ask_ready("Place the T-block flat at the GOAL."):
                    manifest["status"] = "cancelled"
                    return 0
                goal, goal_frame = tracking.capture_target(tracker)
                setup = make_setup(args, result, shape, goal)
                write_json(run / "setup.json", setup)
                if not cv2.imwrite(str(run / "goal.png"), goal_frame["image"]):
                    raise OSError("Could not save goal reference image")
                write_json(run / "goal_detection.json", goal_frame["record"])
            manifest["setup"] = setup
            print(f"Table Z: {setup['table_height_m']:.4f} m; episode starts "
                  f"{args.start_min_distance:g}–{args.start_distance:g} m outside the relocated T outline")
            print("Goal fixed. Moving to the home joint pose.")
            robot = FrankaRemoteController(args.robot_ip, home=False)
            stack.callback(robot.stop)  # Stop control before camera/HID cleanup on every exit.
            robot.start()
            motion = motion_code.Motion(robot, kin, home_q=args.home_q)
            motion.osc_gains = setup["osc_gains"]
            motion.go_home()
            manifest.update(status="collecting", camera=tracker.metadata)
            write_json(manifest_path, manifest)
            episode = first_episode - 1
            while args.episodes == 0 or episode < first_episode - 1 + args.episodes:
                if not ask_ready("Robot is parked. Randomize the T-block on the table."):
                    break
                initial_object, initial_frame = tracking.capture_target(tracker)
                episode_setup = make_episode_setup(args, setup, shape, initial_object, rng)
                episode_setup["run_attempt"] = manifest_path.name
                episode_setup["initial_detection"] = initial_frame["record"]
                episode_setup["approach_plan"] = motion.plan_start(
                    episode_setup["start_tip_base_m"])
                episode += 1
                episode_path = run / f"episode_{episode:04d}"
                writer = recording.EpisodeWriter(episode_path, {
                    **episode_setup, "episode": episode,
                    "calibration_sha256": manifest["calibration_sha256"], "control_frequency_hz": args.frequency},
                    image_format=args.image_format)
                outcome, reason = "interrupted", "Collection interrupted"
                try:
                    write_json(episode_path / "setup.json", episode_setup)
                    print(f"Episode {episode} start tip: {episode_setup['start_tip_base_m']} m "
                          f"({episode_setup['start_outline_distance_m']:.3f} m outside T)")
                    motion.approach(episode_setup["start_tip_base_m"])
                    tracker.attach(writer)
                    motion.configure_osc()
                    outcome = collect_episode(robot, motion, kin, mouse, tracker, shape, episode_setup, writer, args)
                    reason = "Goal coverage exceeded threshold" if outcome == "success" else "Episode time limit"
                except BaseException as exc:
                    outcome = "interrupted" if isinstance(exc, KeyboardInterrupt) else "error"
                    reason = str(exc) or "Collection interrupted"
                    # Stop control before waiting for a potentially slow recording disk.
                    robot.stop()
                    robot = None
                    raise
                finally:
                    tracker.attach(None)
                    writer.finish(outcome, reason)
                print(f"Episode {episode}: {outcome}. Resetting through the default joint pose.")
                motion.go_home()
            manifest["status"] = "complete"
    except BaseException as exc:
        manifest.update(status="interrupted" if isinstance(exc, KeyboardInterrupt) else "error", reason=str(exc))
        raise
    finally:
        if robot is not None:
            robot.stop()
        write_json(manifest_path, manifest)
        print(f"Saved run: {run}")
        if args.video:
            video_code = sibling("t_pushing_video.py", "_t_pushing_video")
            for directory in sorted(run.glob("episode_*")):
                suffix = directory.name.removeprefix("episode_")
                if directory.is_dir() and suffix.isdigit() and int(suffix) >= first_episode:
                    for overlay in (False, True):
                        try:
                            path = video_code.encode_episode(directory, overlay=overlay)
                            if path is not None:
                                print(f"Video saved: {path}")
                        except Exception as exc:
                            print(f"Video export failed for {directory.name} (overlay={overlay}): {exc}", file=sys.stderr)
    return 0


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=("inspect", "plan", "collect"), nargs="?", default="inspect")
    p.add_argument("--robot-ip", default="173.16.0.2")
    p.add_argument("--condition", choices=("ground", "elevated"))
    p.add_argument("--calibration", type=Path, help="Default: newest completed camera_calibration session")
    p.add_argument("--target-config", type=Path, default=ROOT / "assets/t_shape_target_xl/config.json")
    p.add_argument("--stick-xml", type=Path, default=ROOT / "assets/pocky/stick.xml")
    p.add_argument("--robot-xml", type=Path, default=ROOT / "aiofranka/model/fr3.xml")
    p.add_argument("--tip-offset", type=float, nargs=3, default=[0., 0., .161], metavar=("X", "Y", "Z"))
    p.set_defaults(tip_height=.020)  # This experiment fixes physical tip clearance at 2 cm.
    p.add_argument("--home-q", type=float, nargs=7, default=motion_code.HOME_Q.tolist())
    p.add_argument("--start-min-distance", type=float, default=.05,
                   help="Minimum distance outside the relocated T outline for sampled starts, meters")
    p.add_argument("--start-distance", type=float, default=.10,
                   help="Maximum distance outside the relocated T outline for sampled starts, meters")
    p.add_argument("--seed", type=int, help="Random seed for repeatable episode start samples")
    p.add_argument("--scale", type=float, default=.05,
                   help="XY offset from measured position, meters per input unit (default: 0.05)")
    p.add_argument("--deadzone", type=float, default=0.,
                   help="Suppress input axes below this magnitude; 0 matches example 02")
    p.add_argument("--frequency", type=float, default=50.)
    p.add_argument("--success-threshold", type=float, default=.85)
    p.add_argument("--episode-seconds", type=float, default=300.)
    p.add_argument("--episodes", type=int, default=0, help="Number of new episodes this invocation; 0: until q")
    destination = p.add_mutually_exclusive_group()
    destination.add_argument("--output", type=Path, help="New run directory (must not exist)")
    destination.add_argument("--resume", type=Path, help="Append episodes using an existing run's saved setup/settings")
    p.add_argument("--image-format", choices=("jpeg", "png"), default="jpeg",
                   help="JPEG quality 95 (default), or lossless PNG with higher disk use")
    p.add_argument("--video", action=argparse.BooleanOptionalAction, default=True,
                   help="Encode RGB and AprilCube overlay MP4s after robot shutdown (default: enabled)")
    p.add_argument("--goal-pose", type=Path, help="For offline plan: setup.json or a JSON 4x4 T_base_goal")
    p.add_argument("--object-pose", type=Path,
                   help="For plan: JSON T_base_object/initial_T_base_object or 4x4 pose; default uses goal as example")
    p.add_argument("--stream", help="PyCAAS stream; default selects the calibrated camera serial")
    p.add_argument("--pycaas-endpoint", default="ipc:///tmp/pycaas.sock")
    return p


def parse_arguments(p, argv=None):
    args = p.parse_args(argv)
    if args.resume is None:
        return args
    run = args.resume.expanduser().resolve()
    original = json.loads((run / "run.json").read_text())
    setup = json.loads((run / "setup.json").read_text())
    geometry.rigid_transform(setup["T_base_goal"])
    if not np.isfinite(setup["table_height_m"]):
        raise ValueError("Saved table height is not finite")
    excluded = {"command", "output", "resume", "episodes", "calibration", "target_config", "goal_pose", "object_pose"}
    inherited = {name: value for name, value in original["arguments"].items()
                 if name in vars(args) and name not in excluded}
    p.set_defaults(**inherited, calibration=run / "calibration.json",
                   target_config=run / "target_config.json", goal_pose=run / "setup.json")
    args = p.parse_args(argv)  # Explicit flags override inherited collection settings.
    args.resume = run
    if args.condition != setup["condition"]:
        raise ValueError("--resume must keep the saved table condition")
    for filename, path, key in (("calibration.json", args.calibration, "calibration_sha256"),
                                ("target_config.json", args.target_config, "target_config_sha256")):
        if digest(path) != original[key] or digest(run / filename) != original[key]:
            raise ValueError(f"--resume must use the unchanged saved {filename}")
    return args


def main(argv=None):
    p = parser()
    try:
        args = parse_arguments(p, argv)
    except (OSError, ValueError, KeyError) as exc:
        p.error(str(exc))
    positive = (args.tip_height, args.start_distance, args.frequency,
                args.scale,
                args.episode_seconds)
    if (not np.isfinite(positive).all() or min(positive) <= 0 or not 0 < args.success_threshold < 1
            or not 0 <= args.start_min_distance < args.start_distance
            or not 0 <= args.deadzone < 1 or args.episodes < 0 or not 10 <= args.frequency <= 100
            or not np.isfinite(args.tip_offset + args.home_q).all()
            or (args.seed is not None and args.seed < 0)):
        p.error("Invalid dimensions, rates, thresholds, or joint/tool values")
    if args.command == "collect" and args.condition is None:
        p.error("collect requires --condition ground or elevated")
    if args.command == "plan" and args.goal_pose is None:
        p.error("plan requires --goal-pose (a saved setup.json or a 4x4 transform)")
    cv2.setNumThreads(1)
    try:
        control = spacemouse_control(args)
        selected, result = load_calibration(args.calibration)
        args.stream = args.stream or "rs_" + result["camera"]["serial"] + "_color"
        shape = geometry.TShape(args.target_config)
        kin = motion_code.Kinematics(args.robot_xml, tip_offset=args.tip_offset)
        if args.command == "collect":
            return collect(args, selected, result, shape, kin)
        print(f"Calibration: {selected}")
        print(f"T-block footprint: {shape.area_m2:.6f} m²; physical tip offset: {args.tip_offset} m")
        print(f"Default joint-pose tip: {kin.tip(kin.fk(args.home_q)).tolist()} m")
        print(f"SpaceMouse XY scale: {control['xy_scale_m_per_unit']} m/unit from measured position")
        if args.resume:
            print(f"Resume: {args.resume}; next episode {next_index(args.resume, 'episode_'):04d}")
        if args.command == "plan":
            saved = json.loads(args.goal_pose.read_text())
            goal = saved["T_base_goal"] if isinstance(saved, dict) else saved
            setup = make_setup(args, result, shape, goal)
            initial_object = goal
            if args.object_pose is None:
                print("Sampling around the goal pose as an example object; collection uses each relocated T pose.")
            else:
                initial_object = json.loads(args.object_pose.read_text())
                if isinstance(initial_object, dict):
                    initial_object = initial_object.get("metadata", initial_object)
                    initial_object = initial_object.get("T_base_object", initial_object.get("initial_T_base_object"))
                if initial_object is None:
                    raise ValueError("--object-pose needs T_base_object, initial_T_base_object, or a 4x4 pose")
                print(f"Sampling around object pose: {args.object_pose}")
            episode_setup = make_episode_setup(args, setup, shape, initial_object, np.random.default_rng(args.seed))
            plan = motion_code.Motion(None, kin, home_q=args.home_q).plan_start(
                episode_setup["start_tip_base_m"])
            episode_setup["approach_plan"] = recording.json_value(plan)
            print(json.dumps(episode_setup, indent=2))
        print("Offline checks only; no robot, camera, or SpaceMouse opened.")
        return 0
    except KeyboardInterrupt:
        print("\nStopped; partial recordings are preserved.")
        return 130
    except (ImportError, OSError, RuntimeError, ValueError, KeyError) as exc:
        print(f"T-pushing error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
