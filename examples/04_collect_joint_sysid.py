#!/usr/bin/env python3
"""
Collect joint impedance data to identify the FR3 with 05_fit_joint_sysid.py.

With a joint impedance configuration (see aiofranka.config: kp, kd, the policy rate as
frequency, and the tool), it plays 13 s blocks of joint targets around three base poses,
two at each. The targets change at the policy
rate and are held in between, as a policy's actions are:

    hold 0.5 s | 2 steps, 3 s | multisine 0.17-3 Hz, 6 s | slow ramps, 3 s | hold 0.5 s

Every 1 kHz control tick goes to one npz, examples/sysid_data/joint_sysid_<date>.npz.

    python examples/04_collect_joint_sysid.py 173.16.0.2 --activate configs/joint_impedance.yaml
    mjpython examples/04_collect_joint_sysid.py --activate configs/joint_impedance.yaml   # MuJoCo

Add --plan to check the plan and exit. It refuses to move if the configuration's tool is
not the end effector active in Desk, and applies the configuration with
controller.activate() for every block; 05_fit_joint_sysid.py adds its fit to the same file. The robot compensates gravity, including the end
effector active in Desk, and the torques stay inside aiofranka's 990 Nm/s rate limit and
torque limits. Before moving, every target and every move between poses is checked for
5 cm of clearance between the arm, a cylinder around the tool and the floor, and every
target for 0.1 rad from the joint limits. While playing, only the robot's own limits
apply: past its joint position, velocity or torque limits its reflexes stop it, and the
data so far is saved. Ctrl+C stops playing and holds the arm where it is. Joints 5-7 are
light: joint 7 oscillated at kd 20 (stable at 12). Keep a hand on the enabling device.
"""

from __future__ import annotations

import argparse
import asyncio
import datetime
import hashlib
import json
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from aiofranka import FrankaController, RobotInterface
from aiofranka.config import add_recording, load_config, to_yaml
from aiofranka.payload import MODEL_PATH, _closest, _collision_model, _is_clear

CONTROL_HZ = 1000
TORQUE_LIMIT = np.array([87.0] * 4 + [12.0] * 3)  # FrankaController.torque_limit [Nm]

POSES = {
    "home": [0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, -0.7854],
    "left": [0.6, -0.3, 0.2, -2.2, 0.0, 1.9, -0.2],
    "right": [-0.6, 0.2, -0.2, -1.6, 0.0, 1.8, -1.4],
}

# Gains for moving between poses and for holding after a stop.
MOVE_KP = np.array([80.0] * 4 + [48.0] * 3)
MOVE_KD = np.array([8.0] * 4 + [6.0] * 3)
MOVE_SPEED = 0.5  # peak joint speed [rad/s]
SETTLE = 1.0  # s after a move

# The excitation, as offsets from the base pose.
SEGMENTS = ("hold", "steps", "multisine", "ramps")
HOLD = 0.5  # s at the start and end of a block
STEP_COUNT, STEP_HOLD = 2, 0.75  # steps; s at each step's offset, then as long back
STEP_MAX = np.array([0.12] * 4 + [0.2] * 3)  # rad
STEP_TORQUE = 0.5  # largest kp * step, as a fraction of the torque limit
MULTISINE_S, MULTISINE_FMAX = 6.0, 3.0  # s (one period of the lowest harmonic), Hz
MULTISINE_PEAK = np.array([0.12] * 4 + [0.2] * 3)  # rad
MULTISINE_SPEED = np.array([0.4] * 4 + [0.8] * 3)  # rad/s
FADE = 0.5  # s of cosine fade at both ends of the multisine
RAMP, RAMP_SPEED = 0.06, 0.08  # rad, rad/s

PLAN_LIMIT_MARGIN = 0.1  # rad between every target and the joint limits


@dataclass
class Block:
    """One run of the excitation at one pose."""

    pose: str
    kp: np.ndarray  # (7,)
    kd: np.ndarray  # (7,)
    rate: int
    targets: np.ndarray  # joint targets at every tick, held between updates [rad] (ticks, 7)
    segment: np.ndarray  # index into SEGMENTS at every tick (ticks,)


def ticks(seconds):
    return round(seconds * CONTROL_HZ)


def steps(kp, torque_limit, rng):
    """Steps of all joints at once, each to a random offset and back."""
    size = np.minimum(STEP_MAX, STEP_TORQUE * torque_limit / kp)
    parts = []
    for _ in range(STEP_COUNT):
        offset = rng.choice([-1.0, 1.0], 7) * rng.uniform(0.5, 1.0, 7) * size
        parts += [np.tile(offset, (ticks(STEP_HOLD), 1)), np.zeros((ticks(STEP_HOLD), 7))]
    return np.concatenate(parts)


def multisine():
    """
    Sums of sines from 1 / MULTISINE_S to MULTISINE_FMAX, with different harmonics for
    each joint, amplitudes falling as 1 / frequency (about flat in velocity) and Schroeder
    phases, scaled to MULTISINE_PEAK and MULTISINE_SPEED.
    """
    t = np.arange(ticks(MULTISINE_S)) / CONTROL_HZ
    fade = np.clip(np.minimum(t, MULTISINE_S - t) / FADE, 0.0, 1.0)
    fade = 0.5 - 0.5 * np.cos(np.pi * fade)
    offsets = np.zeros((len(t), 7))
    for joint in range(7):
        k = np.arange(joint + 1, round(MULTISINE_FMAX * MULTISINE_S) + 1, 7)
        n = np.arange(len(k))
        phase = -np.pi * n * (n - 1) / len(k)
        shape = (np.sin(2 * np.pi * np.outer(t, k) / MULTISINE_S + phase) / k).sum(1) * fade
        speed = np.abs(np.gradient(shape, 1.0 / CONTROL_HZ)).max()
        offsets[:, joint] = shape * min(MULTISINE_PEAK[joint] / np.abs(shape).max(),
                                        MULTISINE_SPEED[joint] / speed)
    return offsets


def ramps():
    """A triangle at constant speed, up and down to +/-RAMP, alternating sign by joint."""
    quarter = RAMP / RAMP_SPEED
    u = np.arange(ticks(4 * quarter)) / CONTROL_HZ / quarter
    triangle = RAMP * (1.0 - np.abs((u + 1.0) % 4.0 - 2.0))
    return np.outer(triangle, [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0])


def plan(poses, kp, kd, rate, repeats, torque_limit, seed):
    """The blocks in the order they run, repeats at each pose with different steps."""
    hold = np.zeros((ticks(HOLD), 7))
    shared = multisine(), ramps()
    period = CONTROL_HZ // rate
    blocks = []
    for pose, q0 in poses.items():
        for _ in range(repeats):
            rng = np.random.default_rng([seed, len(blocks)])
            parts = [hold, steps(kp, torque_limit, rng), *shared, hold]
            offsets = np.concatenate(parts)
            segment = np.concatenate([np.full(len(p), i, np.int8) for i, p in zip((0, 1, 2, 3, 0), parts)])
            # Zero-order hold: each target lasts CONTROL_HZ / rate ticks.
            held = offsets[np.arange(len(offsets)) // period * period]
            blocks.append(Block(pose, kp, kd, rate, q0 + held, segment))
    return blocks


def check(blocks, poses, model, data, clearance):
    """Problems with the targets and the moves between poses, if any."""
    problems = []
    lower = model.jnt_range[:7, 0] + PLAN_LIMIT_MARGIN
    upper = model.jnt_range[:7, 1] - PLAN_LIMIT_MARGIN
    for pose in poses:
        targets = np.concatenate([b.targets for b in blocks if b.pose == pose])
        low, high = targets.min(0), targets.max(0)
        bad = np.flatnonzero((low < lower) | (high > upper))
        if len(bad):
            problems.append(f"{pose}: joints {(bad + 1).tolist()} come within "
                            f"{PLAN_LIMIT_MARGIN} rad of their limits")
            continue
        changes = np.ones(len(targets), bool)
        changes[1:] = np.any(targets[1:] != targets[:-1], axis=1)
        for q in targets[changes]:
            if not _is_clear(model, data, q):
                problems.append(f"{pose}: {_closest(model, data, q, clearance)} "
                                f"at {np.round(q, 3).tolist()}")
                break
    order = list(poses) + [next(iter(poses))]
    for a, b in zip(order, order[1:]):
        if a != b and not path_clear(model, data, poses[a], poses[b]):
            problems.append(f"the move from {a} to {b} is not clear")
    return problems


def path_clear(model, data, start, end, step=0.02):
    count = max(int(np.ceil(np.abs(end - start).max() / step)), 1)
    return all(_is_clear(model, data, start + s * (end - start))
               for s in np.linspace(0.0, 1.0, count + 1))


class Collector(FrankaController):
    """Joint impedance that plays a block's targets tick by tick and records each tick."""

    def setup(self, capacity):
        self.playing = False
        self.abort_reason = None
        self.count = 0
        vector = lambda: np.zeros((capacity, 7))  # noqa: E731
        self.log = {
            "time": np.zeros(capacity),
            "block": np.zeros(capacity, np.int16),
            "segment": np.zeros(capacity, np.int8),
            "q": vector(), "dq": vector(), "q_des": vector(),
            "tau_cmd": vector(), "tau_J_d": vector(),
            "tau_J": np.full((capacity, 7), np.nan, np.float32),
            "success_rate": np.ones(capacity, np.float32),
        }

    def play(self, index, block):
        self.block_index, self.targets, self.segment, self.tick = index, block.targets, block.segment, 0
        self.playing = True

    def step(self):
        playing = self.playing
        if playing:
            with self.state_lock:
                self.q_desired = self.targets[self.tick]
            sim_time = self.robot.data.time
        super().step()
        if playing:
            self._record(sim_time)
            self.tick += 1
            if self.tick == len(self.targets):
                self.playing = False

    def _record(self, sim_time):
        i = self.count
        if i == len(self.log["time"]):
            self._abort("the log is full")
            return
        log, state, robot_state = self.log, self.state, self.robot.robot_state
        q, dq, q_des = state["qpos"], state["qvel"], self.targets[self.tick]
        log["block"][i] = self.block_index
        log["segment"][i] = self.segment[self.tick]
        log["q"][i], log["dq"][i], log["q_des"][i] = q, dq, q_des
        log["tau_cmd"][i] = self.last_command  # sent, after the rate limit and clip
        log["tau_J_d"][i] = state["last_torque"]  # the last command the robot got
        if robot_state is None:
            log["time"][i] = sim_time
        else:
            log["time"][i] = robot_state.time.to_sec()
            log["tau_J"][i] = robot_state.tau_J
            log["success_rate"][i] = robot_state.control_command_success_rate
        self.count += 1

    def _abort(self, reason):
        """Stop playing and hold the current joint positions."""
        self.abort_reason = reason
        self.playing = False
        with self.state_lock:
            self.kp, self.kd = MOVE_KP.copy(), MOVE_KD.copy()
            self.q_desired = np.array(self.state["qpos"])


async def move_to(controller, target):
    """Move in a straight line in joint space with a quintic time scaling."""
    with controller.state_lock:
        start = np.array(controller.q_desired, dtype=float)
    duration = max(1.875 * np.abs(target - start).max() / MOVE_SPEED, 0.5)
    t0 = time.perf_counter()
    while controller.abort_reason is None:
        s = min((time.perf_counter() - t0) / duration, 1.0)
        with controller.state_lock:
            controller.q_desired = start + s ** 3 * (10 - 15 * s + 6 * s ** 2) * (target - start)
        if s == 1.0:
            return
        await asyncio.sleep(0.01)


def set_gains(controller, kp, kd):
    with controller.state_lock:
        controller.kp = np.broadcast_to(np.asarray(kp, float), (7,)).copy()
        controller.kd = np.broadcast_to(np.asarray(kd, float), (7,)).copy()


async def collect(controller, blocks, poses, config):
    started = time.perf_counter()
    pose = None
    for index, block in enumerate(blocks):
        if block.pose != pose:
            print(f"  Moving to {block.pose}")
            set_gains(controller, MOVE_KP, MOVE_KD)
            await move_to(controller, poses[block.pose])
            await asyncio.sleep(SETTLE)
            pose = block.pose
        if controller.abort_reason is not None:
            return
        left = sum(len(b.targets) for b in blocks[index:]) / CONTROL_HZ
        print(f"  [{index + 1}/{len(blocks)}] {block.pose}   ({(time.perf_counter() - started) / 60:.1f} min, "
              f"{left / 60:.1f} min of blocks left)")
        controller.activate(config, check_tool=False)  # checked before moving
        controller.play(index, block)
        while controller.playing:
            await asyncio.sleep(0.05)
        if controller.abort_reason is not None:
            return
    first = next(iter(poses))
    print(f"  Moving back to {first}")
    set_gains(controller, MOVE_KP, MOVE_KD)
    await move_to(controller, poses[first])
    await asyncio.sleep(SETTLE)


def provenance():
    root = Path(__file__).resolve().parents[1]
    try:
        commit = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"],
                                capture_output=True, text=True).stdout.strip()
        dirty = bool(subprocess.run(["git", "-C", str(root), "status", "--porcelain", "aiofranka", "examples"],
                                    capture_output=True, text=True).stdout.strip())
    except OSError:
        commit, dirty = "", True
    return {"aiofranka_commit": commit, "aiofranka_dirty": dirty,
            "fr3_xml_sha256": hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()}


def robot_load(robot):
    """The payload the robot compensates and that is merged into the MuJoCo model."""
    payload = {key: np.asarray(value).tolist() for key, value in robot.payload.items()}
    if robot.robot_state is None:
        return {"payload": payload}
    state = robot.robot_state
    desk = {key: np.asarray(getattr(state, key)).tolist() for key in
            ("m_ee", "F_x_Cee", "I_ee", "m_load", "F_x_Cload", "I_load", "m_total", "F_x_Ctotal",
             "I_total", "F_T_EE", "F_T_NE", "NE_T_EE")}
    return {"payload": payload, "robot_state": desk}


def save(path, controller, blocks, poses, meta):
    n = controller.count
    arrays = {key: value[:n] for key, value in controller.log.items()}
    pose_names = list(poses)
    arrays.update(
        kp=np.array([b.kp for b in blocks]),
        kd=np.array([b.kd for b in blocks]),
        rate_hz=np.array([b.rate for b in blocks]),
        pose=np.array([pose_names.index(b.pose) for b in blocks]),
        poses=np.array(list(poses.values())),
    )
    meta = dict(meta, ticks=n, abort_reason=controller.abort_reason,
                completed=controller.abort_reason is None and n == sum(len(b.targets) for b in blocks))
    arrays["meta"] = np.array(json.dumps(meta, indent=1))
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(f".{path.stem}.partial.npz")
    np.savez(partial, **arrays)
    partial.replace(path)


def summarize(path):
    """Tracking per pose, how often the rate limit and clip acted, and a replay check."""
    data = np.load(path)
    meta = json.loads(str(data["meta"]))
    n = meta["ticks"]
    if n == 0:
        return
    block = data["block"]
    kp, kd = data["kp"][block], data["kd"][block]
    q, dq, q_des, tau_cmd, tau_J_d = (data[k] for k in ("q", "dq", "q_des", "tau_cmd", "tau_J_d"))
    limit, rate_limit = np.array(meta["torque_limit"]), meta["torque_rate_limit"]

    # The torque aiofranka sends, recomputed from the logged state.
    tau = (q_des - q) * kp - dq * kd
    tau = tau_J_d + np.clip((tau - tau_J_d) / 1e-3, -rate_limit, rate_limit) * 1e-3
    tau = np.clip(tau, -limit, limit)
    replay = np.abs(tau - tau_cmd).max()

    rate_limited = (np.abs(tau_cmd - tau_J_d) >= 0.999 * rate_limit * 1e-3).any(1)
    clipped = (np.abs(tau_cmd) >= limit - 1e-9).any(1)
    print(f"\n  {n} ticks ({n / CONTROL_HZ / 60:.1f} min) in {len(np.unique(block))} blocks")
    print(f"  Replay check: max |recomputed - sent torque| = {replay:.1e} Nm")
    if not meta["sim"]:
        dt = np.diff(data["time"])
        new_block = np.diff(block) != 0
        lost = np.round(dt[~new_block] * CONTROL_HZ).astype(int) - 1
        print(f"  Lost packets: {lost[lost > 0].sum()}, min success rate "
              f"{data['success_rate'].min():.3f}")
    print("\n  pose     RMS error [mrad]   max |tau| j1-4 / j5-7 [Nm]   rate-limited   clipped")
    pose = data["pose"][block]
    for index, name in enumerate(meta["pose_names"]):
        rows = pose == index
        if not rows.any():
            continue
        rms = 1000 * np.sqrt(np.mean((q_des[rows] - q[rows]) ** 2))
        peak = np.abs(tau_cmd[rows]).max(0)
        print(f"  {name:6s}   {rms:16.1f}   {peak[:4].max():14.1f} / {peak[4:].max():4.1f}"
              f"   {100 * rate_limited[rows].mean():11.1f}%   {100 * clipped[rows].mean():6.1f}%")


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[1].replace("\n", " "),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("ip", nargs="?", help="Robot IP; omit it to run MuJoCo")
    parser.add_argument("--activate", type=Path, required=True,
                        help="Joint impedance configuration (YAML, see aiofranka.config)")
    parser.add_argument("--plan", action="store_true", help="Check the plan and exit")
    parser.add_argument("--no-check-tool", action="store_true",
                        help="Collect even if Desk's active end effector is not the configuration's tool "
                             "or cannot be read")
    parser.add_argument("--poses", nargs="+", default=list(POSES), choices=list(POSES), help="Base poses")
    parser.add_argument("--repeats", type=int, default=2, help="Blocks at each pose")
    parser.add_argument("--seed", type=int, default=0, help="Seed of the step offsets")
    parser.add_argument("--tool-length", type=float, default=0.2,
                        help="Tool length from the flange, for the clearance checks [m]")
    parser.add_argument("--tool-radius", type=float, default=0.1,
                        help="Tool radius, for the clearance checks [m]")
    parser.add_argument("--floor", type=float, default=0.0,
                        help="Floor or table height in the base frame [m]")
    parser.add_argument("--clearance", type=float, default=0.05,
                        help="Clearance between the arm, the tool and the floor [m]")
    parser.add_argument("--out", type=Path, default=Path(__file__).resolve().parent / "sysid_data",
                        help="Output folder")
    parser.add_argument("--headless", action="store_true", help="In MuJoCo, run without the viewer")
    parser.add_argument("-y", "--yes", action="store_true", help="Do not ask before moving")
    args = parser.parse_args()
    try:
        args.config = load_config(args.activate)
    except (OSError, ValueError) as error:
        parser.error(f"--activate {args.activate}: {error}")
    if args.config["mode"] != "impedance":
        parser.error(f"--activate needs mode impedance, not {args.config['mode']}; use 06_collect_osc_sysid.py")
    args.kp, args.kd, args.hz = args.config["kp"], args.config["kd"], round(args.config["frequency"])
    if args.hz != args.config["frequency"] or CONTROL_HZ % args.hz:
        parser.error(f"The frequency must divide {CONTROL_HZ} Hz, not {args.config['frequency']}")
    if np.any(args.kp <= 0):
        parser.error("kp must be positive")
    return args


async def main() -> int:
    args = parse_args()
    poses = {name: np.array(POSES[name]) for name in args.poses}
    blocks = plan(poses, args.kp, args.kd, args.hz, args.repeats, TORQUE_LIMIT, args.seed)

    model, data = _collision_model(args.tool_length, args.tool_radius, args.floor, args.clearance)
    problems = check(blocks, poses, model, data, args.clearance)
    total = sum(len(b.targets) for b in blocks)
    moves = sum(max(1.875 * np.abs(poses[a] - poses[b]).max() / MOVE_SPEED, 0.5) + SETTLE
                for a, b in zip(list(poses), list(poses)[1:] + list(poses)[:1]) if a != b)
    print(f"\n  {args.activate}: kp {args.kp.tolist()}, kd {args.kd.tolist()}, targets at {args.hz} Hz, "
          f"tool {args.config.get('tool', 'not set')}")
    print(f"  {len(blocks)} blocks, {args.repeats} at each of {', '.join(poses)}: about "
          f"{(total / CONTROL_HZ + moves) / 60:.1f} min, {total * 380 / 1e6:.0f} MB")
    size = np.minimum(STEP_MAX, STEP_TORQUE * TORQUE_LIMIT / args.kp)
    print(f"  Steps up to {np.round(size, 3).tolist()} rad")
    if problems:
        print("\n  Not safe to run:")
        for problem in problems:
            print(f"    {problem}")
        print()
        return 1
    print(f"  Every target and move keeps {100 * args.clearance:.0f} cm of clearance "
          f"(tool {args.tool_length} m x {args.tool_radius} m, floor at {args.floor} m)")
    if args.plan:
        print()
        return 0

    if args.ip is None and args.headless:
        class NoViewer:
            def sync(self):
                pass
        mujoco.viewer.launch_passive = lambda *a, **k: NoViewer()
    robot = RobotInterface(args.ip)
    start = robot.data.qpos[:7].copy()
    first = next(iter(poses))
    if not path_clear(model, data, start, poses[first]):
        print(f"\n  The move from the current pose to {first} is not clear; move the arm closer to it.\n")
        return 1
    load = robot_load(robot)
    print(f"  Payload the robot compensates: {load['payload']['mass']:.3f} kg at "
          f"{np.round(load['payload']['com'], 4).tolist()} m")

    controller = Collector(robot)
    if not args.no_check_tool:
        try:
            controller.check_tool(args.config)
        except RuntimeError as error:
            print(f"\n  {error} Here: --no-check-tool.\n")
            return 1
    controller.setup(total)
    stamp = datetime.datetime.now()
    path = args.out / f"joint_sysid_{stamp:%Y%m%d_%H%M%S}{'' if robot.real else '_sim'}.npz"
    for suffix in range(2, 100):  # never overwrite another recording
        if not path.exists():
            break
        path = path.with_name(f"{path.stem.rsplit('-', 1)[0]}-{suffix}.npz")
    meta = {
        "format": "aiofranka-sysid-2",
        "controller": "impedance",
        "date": stamp.isoformat(timespec="seconds"),
        "sim": not robot.real,
        "ip": args.ip,
        "control_hz": CONTROL_HZ,
        "torque_limit": controller.torque_limit.tolist(),
        "torque_rate_limit": controller.torque_diff_limit,
        "segments": SEGMENTS,
        "config": to_yaml(args.config),
        "config_path": str(args.activate.resolve()),
        "tool": robot.tool.name if robot.tool is not None else None,
        "tool_checked": not args.no_check_tool and robot.real,
        "hz": args.hz,
        "pose_names": list(poses),
        "load": load,
        "settings": {k: (str(v) if isinstance(v, Path) else v.tolist() if isinstance(v, np.ndarray) else v)
                     for k, v in vars(args).items() if k != "config"},
        **provenance(),
    }
    controller.error_callback = lambda error: save(path, controller, blocks, poses, dict(meta, loop_error=error))

    if not args.yes:
        input(f"\n  The arm will move to {first} and play {len(blocks)} blocks. "
              "Press Enter to start, Ctrl+C to cancel... ")
    print()
    await controller.start()
    status = 0
    try:
        await collect(controller, blocks, poses, args.config)
    except asyncio.CancelledError:
        controller._abort("interrupted")
        await asyncio.sleep(0.5)
        status = 130
    finally:
        await controller.stop()
        save(path, controller, blocks, poses, meta)
    if controller.abort_reason is not None:
        print(f"\n  Stopped: {controller.abort_reason}")
        status = status or 1
    print(f"\n  Saved {path}")
    summarize(path)
    if robot.real and status == 0:
        # A complete recording from the robot: list it in the configuration, where the fit finds it.
        try:
            add_recording(args.activate, {"path": path, "date": meta["date"], "tool": meta["tool"]},
                          controller=args.config)
            print(f"\n  Added it to the recordings of {args.activate}; fit it with:\n"
                  f"    python examples/05_fit_joint_sysid.py --activate {args.activate} --physics_dt <s>")
        except (OSError, ValueError, RuntimeError) as error:
            print(f"\n  Could not add it to {args.activate}: {error}")
    print()
    return status


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
