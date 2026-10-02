#!/usr/bin/env python3
"""
Collect operational space control data to identify the FR3 with 07_fit_osc_sysid.py.

With the OSC gains, null-space gains and target, TCP and policy rate you will use, it
plays 13 s blocks of TCP pose targets around three base poses, two at each. The targets
change at the policy rate and are held in between, as a policy's actions are:

    hold 0.5 s | 2 steps, 3 s | 6-DOF multisine 0.17-3 Hz, 6 s | slow ramps, 3 s | hold 0.5 s

Position offsets are in the base frame, rotation offsets are rotation vectors in the base
frame about the TCP. Every 1 kHz control tick goes to one npz,
examples/sysid_data/osc_sysid_<date>.npz.

    python examples/06_collect_osc_sysid.py 173.16.0.2 --ee_kp 300 300 300 30 30 30 \\
        --ee_kd 30 30 30 3 3 3 --null_kp 10 --hz 50 --tcp 0 0 0.1034
    mjpython examples/06_collect_osc_sysid.py --ee_kp ... --ee_kd ... --null_kp 10 --hz 50   # dry run in MuJoCo

Add --plan to check the plan and exit. Each block starts at its base joint pose with joint
impedance, switches to the OSC and waits 1 s for the null space to settle toward the null
target, then plays. Before moving, every target is solved with inverse kinematics (biased
toward the null target, as the OSC's null space is) and checked for 5 cm of clearance
between the arm, a cylinder around the tool and the floor and for 0.1 rad from the joint
limits, and so is every move between poses. While playing, only the robot's own limits
apply: past its joint position, velocity or torque limits its reflexes stop it, and the
data so far is saved. Ctrl+C stops playing and holds the arm where it is with joint
impedance. Keep a hand on the enabling device.
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
from aiofranka.payload import MODEL_PATH, _closest, _collision_model, _is_clear

CONTROL_HZ = 1000
POSES = {
    "home": [0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, -0.7854],
    "left": [0.6, -0.3, 0.2, -2.2, 0.0, 1.9, -0.2],
    "right": [-0.6, 0.2, -0.2, -1.6, 0.0, 1.8, -1.4],
}

# Gains for moving between poses and for holding after a stop.
MOVE_KP = np.array([80.0] * 4 + [48.0] * 3)
MOVE_KD = np.array([8.0] * 4 + [6.0] * 3)
MOVE_SPEED = 0.5  # peak joint speed [rad/s]
SETTLE = 1.0  # s after a move, and after switching to the OSC

# The excitation, as offsets of the TCP pose: x, y, z [m] and a rotation vector [rad].
SEGMENTS = ("hold", "steps", "multisine", "ramps")
HOLD = 0.5  # s at the start and end of a block
STEP_COUNT, STEP_HOLD = 2, 0.75  # steps; s at each step's offset, then as long back
MULTISINE_S, MULTISINE_FMAX, FADE = 6.0, 3.0, 0.5  # s (one period of the lowest harmonic), Hz, s
RAMP_QUARTER = 0.75  # s from 0 to the ramp's peak
SIZE = np.array([0.03] * 3 + [0.1] * 3)  # largest offset of each segment [m, rad]
WRENCH = np.array([20.0] * 3 + [5.0] * 3)  # largest ee_kp * offset [N, Nm]; smaller offsets for stiffer gains
SPEED = np.array([0.1] * 3 + [0.4] * 3)  # largest multisine speed [m/s, rad/s]
RAMP_SIZE = 0.5  # ramp peak, as a fraction of the size

PLAN_LIMIT_MARGIN = 0.1  # rad between every IK solution and the joint limits


@dataclass
class Block:
    """One run of the excitation at one pose."""

    pose: str
    offsets: np.ndarray  # TCP offsets at every tick, held between updates (ticks, 6)
    rotations: np.ndarray  # the rotation offsets as matrices (ticks, 3, 3)
    segment: np.ndarray  # index into SEGMENTS at every tick (ticks,)


def ticks(seconds):
    return round(seconds * CONTROL_HZ)


def rotation(rotvec):
    """Rotation matrices from rotation vectors (..., 3) -> (..., 3, 3) (Rodrigues)."""
    angle = np.linalg.norm(rotvec, axis=-1)[..., None, None]
    k = np.zeros(rotvec.shape[:-1] + (3, 3))
    k[..., 0, 1], k[..., 0, 2], k[..., 1, 2] = -rotvec[..., 2], rotvec[..., 1], -rotvec[..., 0]
    k = k - np.swapaxes(k, -1, -2)
    small = angle < 1e-9
    a = np.where(small, 1.0, np.sin(angle) / np.where(small, 1.0, angle))
    b = np.where(small, 0.5, (1 - np.cos(angle)) / np.where(small, 1.0, angle) ** 2)
    return np.eye(3) + a * k + b * k @ k


def rotvec(matrix):
    """Rotation vectors of rotation matrices (..., 3, 3) -> (..., 3), for angles below pi."""
    vee = 0.5 * np.stack([matrix[..., 2, 1] - matrix[..., 1, 2], matrix[..., 0, 2] - matrix[..., 2, 0],
                          matrix[..., 1, 0] - matrix[..., 0, 1]], -1)
    sin = np.linalg.norm(vee, axis=-1, keepdims=True)
    cos = 0.5 * (np.trace(matrix, axis1=-2, axis2=-1)[..., None] - 1.0)
    angle = np.arctan2(sin, cos)
    return vee * np.where(sin > 1e-12, angle / np.where(sin > 1e-12, sin, 1.0), 1.0)


def apply(base, offsets):
    """TCP poses (n, 4, 4) from a base pose and offsets (n, 6)."""
    poses = np.tile(np.eye(4), (len(offsets), 1, 1))
    poses[:, :3, 3] = base[:3, 3] + offsets[:, :3]
    poses[:, :3, :3] = rotation(offsets[:, 3:]) @ base[:3, :3]
    return poses


def steps(size, rng):
    """Steps of all axes at once, each to a random offset and back."""
    parts = []
    for _ in range(STEP_COUNT):
        offset = rng.choice([-1.0, 1.0], 6) * rng.uniform(0.5, 1.0, 6) * size
        parts += [np.tile(offset, (ticks(STEP_HOLD), 1)), np.zeros((ticks(STEP_HOLD), 6))]
    return np.concatenate(parts)


def multisine(size):
    """Sums of sines with different harmonics per axis, about flat in velocity, Schroeder phases."""
    t = np.arange(ticks(MULTISINE_S)) / CONTROL_HZ
    fade = np.clip(np.minimum(t, MULTISINE_S - t) / FADE, 0.0, 1.0)
    fade = 0.5 - 0.5 * np.cos(np.pi * fade)
    offsets = np.zeros((len(t), 6))
    for axis in range(6):
        k = np.arange(axis + 1, round(MULTISINE_FMAX * MULTISINE_S) + 1, 6)
        n = np.arange(len(k))
        phase = -np.pi * n * (n - 1) / len(k)
        shape = (np.sin(2 * np.pi * np.outer(t, k) / MULTISINE_S + phase) / k).sum(1) * fade
        speed = np.abs(np.gradient(shape, 1.0 / CONTROL_HZ)).max()
        offsets[:, axis] = shape * min(size[axis] / np.abs(shape).max(), SPEED[axis] / speed)
    return offsets


def ramps(size):
    """A triangle up and down to +/- RAMP_SIZE of the size, alternating sign by axis."""
    u = np.arange(ticks(4 * RAMP_QUARTER)) / CONTROL_HZ / RAMP_QUARTER
    triangle = 1.0 - np.abs((u + 1.0) % 4.0 - 2.0)
    return np.outer(triangle, RAMP_SIZE * size * np.array([1.0, -1.0, 1.0, -1.0, 1.0, -1.0]))


def plan(poses, ee_kp, rate, repeats, seed):
    """The blocks in the order they run, repeats at each pose with different steps."""
    size = np.minimum(SIZE, WRENCH / ee_kp)
    hold = np.zeros((ticks(HOLD), 6))
    shared = multisine(size), ramps(size)
    period = CONTROL_HZ // rate
    blocks = []
    for pose in poses:
        for _ in range(repeats):
            rng = np.random.default_rng([seed, len(blocks)])
            parts = [hold, steps(size, rng), *shared, hold]
            offsets = np.concatenate(parts)
            segment = np.concatenate([np.full(len(p), i, np.int8) for i, p in zip((0, 1, 2, 3, 0), parts)])
            # Zero-order hold: each target lasts CONTROL_HZ / rate ticks.
            held = offsets[np.arange(len(offsets)) // period * period]
            blocks.append(Block(pose, held, rotation(held[:, 3:]), segment))
    return blocks, size


def site_pose(model, data, site, q):
    data.qpos[:7] = q
    mujoco.mj_kinematics(model, data)
    pose = np.eye(4)
    pose[:3, :3] = data.site_xmat[site].reshape(3, 3)
    pose[:3, 3] = data.site_xpos[site]
    return pose


def ik(model, data, site, target, q, q_null, iterations=10, null_gain=0.05):
    """
    Damped least squares inverse kinematics of the flange site, with a null-space pull
    toward q_null. Returns the joint positions and the remaining position and rotation errors.
    """
    jac = np.zeros((6, model.nv))
    for _ in range(iterations):
        current = site_pose(model, data, site, q)
        error = np.concatenate([target[:3, 3] - current[:3, 3], rotvec(target[:3, :3] @ current[:3, :3].T)])
        mujoco.mj_comPos(model, data)
        mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site)
        j = jac[:, :7]
        pinv = j.T @ np.linalg.inv(j @ j.T + 1e-4 * np.eye(6))
        q = q + pinv @ error + null_gain * (np.eye(7) - pinv @ j) @ (q_null - q)
    current = site_pose(model, data, site, q)
    error = np.concatenate([target[:3, 3] - current[:3, 3], rotvec(target[:3, :3] @ current[:3, :3].T)])
    return q, np.linalg.norm(error[:3]), np.linalg.norm(error[3:])


def check(blocks, poses, tcp, q_null, model, data, clearance):
    """Problems with the targets and the moves between poses, if any."""
    problems = []
    site = model.site("attachment_site").id
    lower = model.jnt_range[:7, 0] + PLAN_LIMIT_MARGIN
    upper = model.jnt_range[:7, 1] - PLAN_LIMIT_MARGIN
    flange_from_tcp = np.linalg.inv(tcp)
    for pose, q0 in poses.items():
        base = site_pose(model, data, site, q0) @ tcp
        # Where the null space settles while the OSC holds the base pose.
        q_base, _, _ = ik(model, data, site, base @ flange_from_tcp, q0, q_null, iterations=300)
        targets = np.concatenate([b.offsets for b in blocks if b.pose == pose])
        changes = np.ones(len(targets), bool)
        changes[1:] = np.any(targets[1:] != targets[:-1], axis=1)
        q = q_base
        for offset in targets[changes]:
            target = apply(base, offset[None])[0] @ flange_from_tcp
            q, position_error, rotation_error = ik(model, data, site, target, q, q_null)
            if position_error > 1e-3 or rotation_error > 5e-3:
                problems.append(f"{pose}: a target is out of reach (offset {np.round(offset, 3).tolist()})")
                break
            if np.any(q < lower) or np.any(q > upper):
                problems.append(f"{pose}: a target needs a joint within {PLAN_LIMIT_MARGIN} rad of its limit: "
                                f"{np.round(q, 3).tolist()}")
                break
            if not _is_clear(model, data, q):
                problems.append(f"{pose}: {_closest(model, data, q, clearance)} at {np.round(q, 3).tolist()}")
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
    """OSC that plays a block's TCP targets tick by tick and records each tick."""

    def setup(self, capacity):
        self.playing = False
        self.abort_reason = None
        self.count = 0
        vector = lambda: np.zeros((capacity, 7))  # noqa: E731
        self.log = {
            "time": np.zeros(capacity),
            "block": np.zeros(capacity, np.int16),
            "segment": np.zeros(capacity, np.int8),
            "q": vector(), "dq": vector(),
            "ee_des": np.zeros((capacity, 4, 4)), "ee": np.zeros((capacity, 4, 4)),
            "tau_cmd": vector(), "tau_J_d": vector(),
            "tau_J": np.full((capacity, 7), np.nan, np.float32),
            "success_rate": np.ones(capacity, np.float32),
        }

    def play(self, index, base, block):
        self.block_index, self.base, self.block, self.tick = index, base, block, 0
        self.playing = True

    def step(self):
        playing = self.playing
        if playing:
            # The target: the block's offset at this tick applied to the base pose.
            target = np.eye(4)
            target[:3, :3] = self.block.rotations[self.tick] @ self.base[:3, :3]
            target[:3, 3] = self.base[:3, 3] + self.block.offsets[self.tick, :3]
            with self.state_lock:
                self.ee_desired = target
            sim_time = self.robot.data.time
        super().step()
        if playing:
            self._record(sim_time)
            self.tick += 1
            if self.tick == len(self.block.offsets):
                self.playing = False

    def _record(self, sim_time):
        i = self.count
        if i == len(self.log["time"]):
            self._abort("the log is full")
            return
        log, state, robot_state = self.log, self.state, self.robot.robot_state
        log["block"][i] = self.block_index
        log["segment"][i] = self.block.segment[self.tick]
        log["q"][i], log["dq"][i] = state["qpos"], state["qvel"]
        log["ee_des"][i] = self.ee_desired
        log["ee"][i] = state["ee"] @ self.control_transform
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
        """Stop playing and hold the current joint positions with joint impedance."""
        self.abort_reason = reason
        self.playing = False
        hold(self)


def hold(controller):
    """Hold the current joint positions with joint impedance."""
    with controller.state_lock:
        controller.kp, controller.kd = MOVE_KP.copy(), MOVE_KD.copy()
    controller.switch("impedance")


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


async def collect(controller, blocks, poses, gains):
    started = time.perf_counter()
    for index, block in enumerate(blocks):
        print(f"  [{index + 1}/{len(blocks)}] {block.pose}   ({(time.perf_counter() - started) / 60:.1f} min, "
              f"{sum(len(b.offsets) for b in blocks[index:]) / CONTROL_HZ / 60:.1f} min of blocks left)")
        hold(controller)
        await move_to(controller, poses[block.pose])
        await asyncio.sleep(SETTLE)
        if controller.abort_reason is not None:
            return
        controller.switch("osc")  # holds the current TCP pose
        with controller.state_lock:
            controller.ee_kp, controller.ee_kd = gains["ee_kp"].copy(), gains["ee_kd"].copy()
            controller.null_kp, controller.null_kd = gains["null_kp"].copy(), gains["null_kd"].copy()
            controller.initial_qpos = gains["null_target"].copy()  # the OSC's null-space target
        await asyncio.sleep(SETTLE)
        if controller.abort_reason is not None:
            return
        with controller.state_lock:
            base = np.array(controller.ee_desired)
        controller.play(index, base, block)
        while controller.playing:
            await asyncio.sleep(0.05)
        if controller.abort_reason is not None:
            return
    first = next(iter(poses))
    print(f"  Moving back to {first}")
    hold(controller)
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


def save(path, controller, blocks, poses, gains, rate, meta):
    n = controller.count
    arrays = {key: value[:n] for key, value in controller.log.items()}
    pose_names = list(poses)
    count = len(blocks)
    arrays.update({name: np.tile(value, (count, 1)) for name, value in gains.items()})
    arrays.update(
        rate_hz=np.full(count, rate),
        pose=np.array([pose_names.index(b.pose) for b in blocks]),
        poses=np.array(list(poses.values())),
        tcp=controller.control_transform,
    )
    meta = dict(meta, ticks=n, abort_reason=controller.abort_reason,
                completed=controller.abort_reason is None and n == sum(len(b.offsets) for b in blocks))
    arrays["meta"] = np.array(json.dumps(meta, indent=1))
    path.parent.mkdir(parents=True, exist_ok=True)
    partial = path.with_name(f".{path.stem}.partial.npz")
    np.savez(partial, **arrays)
    partial.replace(path)


def summarize(path):
    """Tracking per pose and how often the rate limit and clip acted."""
    data = np.load(path)
    meta = json.loads(str(data["meta"]))
    n = meta["ticks"]
    if n == 0:
        return
    block = data["block"]
    tau_cmd, tau_J_d, ee, ee_des = data["tau_cmd"], data["tau_J_d"], data["ee"], data["ee_des"]
    limit, rate_limit = np.array(meta["torque_limit"]), meta["torque_rate_limit"]
    rate_limited = (np.abs(tau_cmd - tau_J_d) >= 0.999 * rate_limit * 1e-3).any(1)
    clipped = (np.abs(tau_cmd) >= limit - 1e-9).any(1)
    position = np.linalg.norm(ee_des[:, :3, 3] - ee[:, :3, 3], axis=1)
    angle = np.linalg.norm(rotvec(ee_des[:, :3, :3] @ np.swapaxes(ee[:, :3, :3], 1, 2)), axis=1)
    print(f"\n  {n} ticks ({n / CONTROL_HZ / 60:.1f} min) in {len(np.unique(block))} blocks")
    if not meta["sim"]:
        dt = np.diff(data["time"])
        lost = np.round(dt[np.diff(block) == 0] * CONTROL_HZ).astype(int) - 1
        print(f"  Lost packets: {lost[lost > 0].sum()}, min success rate {data['success_rate'].min():.3f}")
    print("\n  pose     RMS error [mm, mrad]   max |tau| j1-4 / j5-7 [Nm]   rate-limited   clipped")
    pose = data["pose"][block]
    for index, name in enumerate(meta["pose_names"]):
        rows = pose == index
        if not rows.any():
            continue
        peak = np.abs(tau_cmd[rows]).max(0)
        print(f"  {name:6s}   {1000 * np.sqrt(np.mean(position[rows] ** 2)):8.1f} {1000 * np.sqrt(np.mean(angle[rows] ** 2)):8.1f}"
              f"     {peak[:4].max():14.1f} / {peak[4:].max():4.1f}"
              f"   {100 * rate_limited[rows].mean():11.1f}%   {100 * clipped[rows].mean():6.1f}%")


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[1].replace("\n", " "),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("ip", nargs="?", help="Robot IP; omit it to run MuJoCo")
    parser.add_argument("--ee_kp", "--ee-kp", type=float, nargs=6, required=True,
                        help="TCP stiffness: x, y, z [N/m] and rotation [Nm/rad]")
    parser.add_argument("--ee_kd", "--ee-kd", type=float, nargs=6, required=True,
                        help="TCP damping: x, y, z [N s/m] and rotation [Nm s/rad]")
    parser.add_argument("--null_kp", "--null-kp", type=float, nargs="+", required=True,
                        help="Null-space stiffness, 7 values or one for all joints")
    parser.add_argument("--null_kd", "--null-kd", type=float, nargs="+", default=[1.0],
                        help="Null-space damping, 7 values or one for all joints")
    parser.add_argument("--null_target", "--null-target", type=float, nargs=7, default=POSES["home"],
                        help="Null-space target joint positions [rad]")
    parser.add_argument("--hz", type=int, required=True, help="Policy rate: how often the targets change [Hz]")
    parser.add_argument("--tcp", type=float, nargs="+", default=[0.0, 0.0, 0.0],
                        help="TCP in the flange frame: a translation (3 values) [m] or a 4x4 transform "
                             "(16 values, row by row)")
    parser.add_argument("--plan", action="store_true", help="Check the plan and exit")
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
    if args.hz <= 0 or CONTROL_HZ % args.hz:
        parser.error(f"--hz must divide {CONTROL_HZ} Hz, not {args.hz}")
    for name in ("null_kp", "null_kd"):
        values = getattr(args, name)
        if len(values) not in (1, 7):
            parser.error(f"--{name} takes 1 or 7 values")
        setattr(args, name, np.broadcast_to(np.array(values), (7,)).copy())
    tcp = np.array(args.tcp)
    if tcp.shape == (3,):
        tcp = np.block([[np.eye(3), tcp[:, None]], [np.zeros(3), 1.0]])
    elif tcp.shape == (16,):
        tcp = tcp.reshape(4, 4)
    else:
        parser.error("--tcp takes 3 or 16 values")
    if (not np.allclose(tcp[3], [0, 0, 0, 1]) or not np.allclose(tcp[:3, :3] @ tcp[:3, :3].T, np.eye(3), atol=1e-6)
            or np.linalg.det(tcp[:3, :3]) < 0):
        parser.error("--tcp must be a translation or a 4x4 pose with a rotation")
    args.tcp = tcp
    args.ee_kp, args.ee_kd, args.null_target = np.array(args.ee_kp), np.array(args.ee_kd), np.array(args.null_target)
    return args


async def main() -> int:
    args = parse_args()
    poses = {name: np.array(POSES[name]) for name in args.poses}
    gains = {"ee_kp": args.ee_kp, "ee_kd": args.ee_kd, "null_kp": args.null_kp, "null_kd": args.null_kd,
             "null_target": args.null_target}
    blocks, size = plan(poses, args.ee_kp, args.hz, args.repeats, args.seed)

    model, data = _collision_model(args.tool_length, args.tool_radius, args.floor, args.clearance)
    problems = check(blocks, poses, args.tcp, args.null_target, model, data, args.clearance)
    total = sum(len(b.offsets) for b in blocks)
    moves = len(blocks) * (2 * SETTLE + 1.0) + sum(
        max(1.875 * np.abs(poses[a] - poses[b]).max() / MOVE_SPEED, 0.5)
        for a, b in zip(list(poses), list(poses)[1:] + list(poses)[:1]) if a != b)
    print(f"\n  ee_kp {args.ee_kp.tolist()}, ee_kd {args.ee_kd.tolist()}")
    print(f"  null_kp {args.null_kp.tolist()}, null_kd {args.null_kd.tolist()}, "
          f"null target {args.null_target.tolist()}")
    print(f"  TCP translation {args.tcp[:3, 3].tolist()} m, targets at {args.hz} Hz")
    print(f"  {len(blocks)} blocks, {args.repeats} at each of {', '.join(poses)}: about "
          f"{(total / CONTROL_HZ + moves) / 60:.1f} min, {total * 580 / 1e6:.0f} MB")
    print(f"  Offsets up to {1000 * size[:3].max():.0f} mm and {1000 * size[3:].max():.0f} mrad")
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
    first = next(iter(poses))
    if not path_clear(model, data, robot.data.qpos[:7].copy(), poses[first]):
        print(f"\n  The move from the current pose to {first} is not clear; move the arm closer to it.\n")
        return 1
    load = robot_load(robot)
    print(f"  Payload the robot compensates: {load['payload']['mass']:.3f} kg at "
          f"{np.round(load['payload']['com'], 4).tolist()} m")

    controller = Collector(robot)
    controller.setup(total)
    controller.set_tcp(args.tcp)
    stamp = datetime.datetime.now()
    path = args.out / f"osc_sysid_{stamp:%Y%m%d_%H%M%S}{'' if robot.real else '_sim'}.npz"
    meta = {
        "format": "aiofranka-sysid-2",
        "controller": "osc",
        "date": stamp.isoformat(timespec="seconds"),
        "sim": not robot.real,
        "ip": args.ip,
        "control_hz": CONTROL_HZ,
        "torque_limit": controller.torque_limit.tolist(),
        "torque_rate_limit": controller.torque_diff_limit,
        "segments": SEGMENTS,
        "hz": args.hz,
        "pose_names": list(poses),
        "load": load,
        "settings": {k: (str(v) if isinstance(v, Path) else v.tolist() if isinstance(v, np.ndarray) else v)
                     for k, v in vars(args).items()},
        **provenance(),
    }
    controller.error_callback = lambda error: save(path, controller, blocks, poses, gains, args.hz,
                                                   dict(meta, loop_error=error))

    if not args.yes:
        input(f"\n  The arm will move to {first} and play {len(blocks)} blocks. "
              "Press Enter to start, Ctrl+C to cancel... ")
    print()
    await controller.start()
    status = 0
    try:
        await collect(controller, blocks, poses, gains)
    except asyncio.CancelledError:
        controller._abort("interrupted")
        await asyncio.sleep(0.5)
        status = 130
    finally:
        await controller.stop()
        save(path, controller, blocks, poses, gains, args.hz, meta)
    if controller.abort_reason is not None:
        print(f"\n  Stopped: {controller.abort_reason}")
        status = status or 1
    print(f"\n  Saved {path}")
    summarize(path)
    print()
    return status


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
