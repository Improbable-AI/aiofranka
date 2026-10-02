#!/usr/bin/env python3
"""
Fit a MuJoCo FR3 under operational space control to a 06_collect_osc_sysid.py recording
with CMA-ES, at your simulation's physics step.

The simulation runs the way a policy's environment does: at each policy step it takes
the recorded TCP target, and for each of its decimation = (1 / hz) / physics_dt physics
steps it computes aiofranka's OSC in Python and applies it through fr3.xml's <motor>
actuators:

    tau = J^T Lambda (ee_kp * e - ee_kd * J dq) + (I - J^T Jbar^T) (null_kp * (q_null - q) - null_kd * dq)

with the TCP and null-space target of the recording, the 990 Nm/s rate limit and the
torque clip. Lambda and Jbar come from the mass matrix with fr3.xml's armature, as on the
robot (--mass_matrix plant takes the fitted armature instead, for an environment that
uses its own model's mass matrix). Starting from the recorded gains and fr3.xml's joint
parameters, it fits ee_kp, ee_kd, null_kp, null_kd and each joint's armature, damping and
friction loss to the measured motion, replaying 2 s windows from the measured state at
their start. The error combines the TCP response and the joints, as a length:

    sqrt(tcp_position^2 + (rotation_weight * tcp_rotation)^2 + (joint_weight * joints)^2)

each an RMS over the window samples (the joints per joint). The windows at one pose are
held out to check the fit.

    pip install mjbatch   # batched MuJoCo; it pins its own mujoco version
    python examples/07_fit_osc_sysid.py --traj examples/sysid_data/osc_sysid_<date>.npz --physics_dt 0.002

Writes osc_sysid_<date>_fit_<physics_dt>.json next to the recording.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np
from mjbatch import Batch

CONTROL_HZ = 1000
# aiofranka's model; found without importing aiofranka, which needs pylibfranka.
MODEL_PATH = Path(importlib.util.find_spec("aiofranka").origin).parent / "model" / "fr3.xml"
GAINS = ("ee_kp", "ee_kd", "null_kp", "null_kd")
PHYSICAL = ("armature", "damping", "frictionloss")
SIZES = {"ee_kp": 6, "ee_kd": 6, "null_kp": 7, "null_kd": 7, "armature": 7, "damping": 7, "frictionloss": 7}
BOUNDS = {"ee_kp": (0.1, 1e4), "ee_kd": (0.01, 1e3), "null_kp": (0.01, 1e3), "null_kd": (0.01, 1e2),
          "armature": (0.001, 2.0), "damping": (0.001, 20.0), "frictionloss": (0.001, 10.0)}


class Run:
    """A 06_collect_osc_sysid.py recording: one set of gains, null-space target, TCP and policy rate."""

    def __init__(self, path):
        data = np.load(path)
        self.meta = json.loads(str(data["meta"]))
        if self.meta.get("controller") != "osc":
            raise ValueError("Not an OSC recording; fit joint impedance recordings with 05_fit_joint_sysid.py")
        for key in ("time", "q", "dq", "ee_des", "tau_cmd", "tau_J_d", "pose", "tcp"):
            setattr(self, key, data[key])
        self.block = data["block"].astype(int)
        self.gains = {}
        for name in GAINS + ("null_target", "rate_hz"):
            values = data[name]
            if len(np.unique(values, axis=0)) > 1:
                raise ValueError(f"The recording has several values of {name}; record one setting per file")
            self.gains[name] = values[0].astype(float)
        self.hz = int(self.gains.pop("rate_hz"))
        self.null_target = self.gains.pop("null_target")
        self.torque_limit = np.array(self.meta["torque_limit"])
        self.rate_limit = self.meta["torque_rate_limit"]
        self.payload = self.meta["load"]["payload"]
        self.pose_names = self.meta["pose_names"]

    def windows(self, length, poses):
        """First ticks of back-to-back windows from the start of each block at the poses, without lost packets."""
        lost = np.zeros(len(self.time), bool)
        lost[1:] = np.abs(np.diff(self.time) - 1.0 / CONTROL_HZ) > 0.25e-3
        lost_so_far = np.cumsum(lost)
        starts = []
        for block in np.unique(self.block):
            if self.pose_names[self.pose[block]] not in poses:
                continue
            ticks = np.flatnonzero(self.block == block)
            for start in range(ticks[0], ticks[-1] + 1 - length, length):
                if lost_so_far[start + length] == lost_so_far[start]:
                    starts.append(start)
        return np.array(starts, int)


def build_model(run, physics_dt):
    """fr3.xml with the payload the robot compensated, in free space."""
    spec = mujoco.MjSpec.from_file(str(MODEL_PATH))
    for key in list(spec.keys):
        spec.delete(key)
    mass = run.payload["mass"]
    if mass > 0:
        site = spec.site("attachment_site")
        body = site.parent.add_body(name="payload", pos=list(site.pos), quat=list(site.quat))
        body.gravcomp = 1.0  # the robot compensates its gravity, as fr3.xml does the arm's
        body.explicitinertial = True
        body.mass = mass
        body.ipos = list(run.payload["com"])
        inertia = np.array(run.payload["inertia"])
        body.fullinertia = [inertia[0, 0], inertia[1, 1], inertia[2, 2], inertia[0, 1], inertia[0, 2], inertia[1, 2]]
    spec.option.timestep = physics_dt
    spec.option.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    return spec.compile()


def rotvec(matrix):
    """Rotation vectors of rotation matrices (..., 3, 3) -> (..., 3), for angles below pi."""
    vee = 0.5 * np.stack([matrix[..., 2, 1] - matrix[..., 1, 2], matrix[..., 0, 2] - matrix[..., 2, 0],
                          matrix[..., 1, 0] - matrix[..., 0, 1]], -1)
    sin = np.linalg.norm(vee, axis=-1, keepdims=True)
    cos = 0.5 * (np.trace(matrix, axis1=-2, axis2=-1)[..., None] - 1.0)
    angle = np.arctan2(sin, cos)
    return vee * np.where(sin > 1e-12, angle / np.where(sin > 1e-12, sin, 1.0), 1.0)


class Simulator:
    """Batched closed-loop replays of windows of a run."""

    def __init__(self, run, physics_dt, mass_matrix="nominal", threads=0):
        self.run, self.physics_dt, self.mass_matrix, self.threads = run, physics_dt, mass_matrix, threads
        self.period = CONTROL_HZ // run.hz  # ticks per policy step
        self.ticks = round(physics_dt * CONTROL_HZ)  # ticks per physics step
        if not math.isclose(self.ticks / CONTROL_HZ, physics_dt) or self.period % self.ticks:
            raise ValueError(f"physics_dt must divide the policy period, 1 / {run.hz} s, in whole milliseconds")
        self.decimation = self.period // self.ticks
        self.every = math.lcm(self.ticks, 10)  # ticks between position samples
        self.model = build_model(run, physics_dt)
        self.site = self.model.site("attachment_site").id
        self.nominal_armature = self.model.dof_armature[:7].copy()
        # The mass matrix is stored as its lower triangle, row by row.
        self.rows = np.repeat(np.arange(7), self.model.M_rownnz[:7])
        self.cols = self.model.M_colind[: len(self.rows)]
        self.batches = {}

    def window_length(self, seconds):
        """Ticks closest to seconds that hold whole policy steps and samples."""
        unit = math.lcm(self.period, self.every)
        return max(1, round(seconds * CONTROL_HZ / unit)) * unit

    def start_values(self):
        """Recorded gains and fr3.xml's joint parameters."""
        values = {name: self.run.gains[name].copy() for name in GAINS}
        values.update({name: getattr(self.model, f"dof_{name}")[:7].copy() for name in PHYSICAL})
        return values

    def _batch(self, count):
        if count not in self.batches:
            # forward=True keeps the kinematics and mass matrix current with the state, as on the robot.
            batch = Batch(self.model, count, self.threads, forward=True)
            fields = {name: batch.bind(name) for name in
                      ("qpos", "qvel", "ctrl", "M", "xanchor", "xaxis", "site_xpos", "site_xmat")}
            expanded = {name: batch.expand(f"dof_{name}") for name in PHYSICAL}
            self.batches = {count: (batch, fields, expanded)}  # keep one, they are large
        return self.batches[count]

    def torque(self, fields, goal, gains, q_null, armature, previous):
        """aiofranka's OSC (FrankaController._osc_step), for every simulation at once."""
        run = self.run
        q, dq = fields["qpos"][:, :7], fields["qvel"][:, :7]
        flange_pos = fields["site_xpos"][:, self.site]
        flange_rot = fields["site_xmat"][:, self.site].reshape(-1, 3, 3)
        # Jacobian of the flange: hinge axes and anchors.
        axis, anchor = fields["xaxis"][:, :7], fields["xanchor"][:, :7]
        jac = np.concatenate([np.cross(axis, flange_pos[:, None] - anchor), axis], -1).transpose(0, 2, 1)
        # The TCP and its Jacobian.
        ee_rot = flange_rot @ run.tcp[:3, :3]
        offset = flange_rot @ run.tcp[:3, 3]
        ee_pos = flange_pos + offset
        jac[:, :3] += np.cross(jac[:, 3:].transpose(0, 2, 1), offset[:, None]).transpose(0, 2, 1)
        mass = np.zeros((len(q), 7, 7))
        mass[:, self.rows, self.cols] = fields["M"][:, : len(self.rows)]
        mass[:, self.cols, self.rows] = fields["M"][:, : len(self.rows)]
        if self.mass_matrix == "nominal":
            mass[:, np.arange(7), np.arange(7)] += self.nominal_armature - armature

        twist = np.concatenate([goal[:, :3, 3] - ee_pos, rotvec(goal[:, :3, :3] @ ee_rot.transpose(0, 2, 1))], -1)
        ee_vel = np.einsum("nij,nj->ni", jac, dq)
        minv = np.linalg.inv(mass)
        jac_t = jac.transpose(0, 2, 1)
        mx = np.linalg.inv(jac @ minv @ jac_t)
        feedback = np.einsum("nij,nj->ni", jac_t @ mx, gains["ee_kp"] * twist - gains["ee_kd"] * ee_vel)
        ddq = gains["null_kp"] * (q_null - q) - gains["null_kd"] * dq
        jbar = minv @ jac_t @ mx
        null = ddq - np.einsum("nij,nj->ni", jac_t, np.einsum("nji,nj->ni", jbar, ddq))
        tau = feedback + null
        step_limit = run.rate_limit * self.physics_dt
        tau = previous + np.clip(tau - previous, -step_limit, step_limit)
        return np.clip(tau, -run.torque_limit, run.torque_limit)

    def _load(self, params, starts):
        run = self.run
        count_params, count_windows = len(params["ee_kp"]), len(starts)
        batch, fields, expanded = self._batch(count_params * count_windows)
        per_sim = {name: np.repeat(params[name], count_windows, axis=0) for name in SIZES}
        for name in PHYSICAL:
            expanded[name][:, :7] = per_sim[name]
        batch.set_const()
        ticks = np.tile(starts, count_params)
        fields["qpos"][:] = run.q[ticks]
        fields["qvel"][:] = run.dq[ticks]
        batch.forward()
        return batch, fields, per_sim, ticks

    def rollout(self, params, starts, length):
        """
        Simulated motion every self.every ticks of each window, for each parameter set.

        Args:
            params (dict): ee_kp, ee_kd (P, 6) and null_kp, null_kd, armature, damping, frictionloss (P, 7)
            starts (np.ndarray): First ticks of the windows (W,)
            length (int): Ticks per window, from window_length()

        Returns:
            dict: Joint positions q (P, W, S, 7), TCP positions pos (P, W, S, 3) and rotations
                rot (P, W, S, 3, 3), with S = length // self.every
        """
        run = self.run
        batch, fields, per_sim, ticks = self._load(params, starts)
        previous = run.tau_J_d[ticks].copy()  # the robot's last command at the window start
        samples = length // self.every
        out = {"q": np.empty((len(ticks), samples, 7)), "pos": np.empty((len(ticks), samples, 3)),
               "rot": np.empty((len(ticks), samples, 3, 3))}
        elapsed = 0  # ticks since the window start
        for step in range(length // self.period):
            goal = run.ee_des[ticks + step * self.period]  # the policy's action, held for the step
            for _ in range(self.decimation):
                tau = self.torque(fields, goal, per_sim, run.null_target, per_sim["armature"], previous)
                fields["ctrl"][:] = tau
                previous = tau
                batch.step()
                elapsed += self.ticks
                if elapsed % self.every == 0:
                    sample = elapsed // self.every - 1
                    flange_rot = fields["site_xmat"][:, self.site].reshape(-1, 3, 3)
                    out["q"][:, sample] = fields["qpos"][:, :7]
                    out["pos"][:, sample] = fields["site_xpos"][:, self.site] + flange_rot @ run.tcp[:3, 3]
                    out["rot"][:, sample] = flange_rot @ run.tcp[:3, :3]
        shape = (len(params["ee_kp"]), len(starts), samples)
        return {key: value.reshape(shape + value.shape[2:]) for key, value in out.items()}

    def replay_error(self, count=2000, seed=0):
        """Largest difference between the torques sent and those recomputed from the log [Nm]."""
        run = self.run
        rng = np.random.default_rng(seed)
        ticks = np.sort(rng.choice(len(run.time), size=min(count, len(run.time)), replace=False))
        start = self.start_values()
        batch, fields, per_sim, _ = self._load({k: v[None] for k, v in start.items()}, ticks)
        saved, self.physics_dt = self.physics_dt, 1.0 / CONTROL_HZ  # the robot's rate limit per tick
        try:
            tau = self.torque(fields, run.ee_des[ticks], per_sim, run.null_target, per_sim["armature"],
                              run.tau_J_d[ticks])
        finally:
            self.physics_dt = saved
        return np.abs(tau - run.tau_cmd[ticks]).max()

    def measured(self, starts, length):
        """Measured motion at the samples of rollout(), without the parameter axis."""
        q = self.run.q[starts[:, None] + np.arange(self.every, length + 1, self.every)[None]]
        data = mujoco.MjData(self.model)
        pos, rot = np.empty(q.shape[:-1] + (3,)), np.empty(q.shape[:-1] + (3, 3))
        for index in np.ndindex(q.shape[:-1]):
            data.qpos[:7] = q[index]
            mujoco.mj_kinematics(self.model, data)
            flange_rot = data.site_xmat[self.site].reshape(3, 3)
            pos[index] = data.site_xpos[self.site] + flange_rot @ self.run.tcp[:3, 3]
            rot[index] = flange_rot @ self.run.tcp[:3, :3]
        return {"q": q, "pos": pos, "rot": rot}


def errors(simulated, measured):
    """
    RMS errors of each parameter set (P,): joints [rad, per joint], TCP position [m] and
    TCP rotation [rad].
    """
    q = np.nan_to_num(simulated["q"] - measured["q"][None], nan=10.0)
    pos = np.nan_to_num(simulated["pos"] - measured["pos"][None], nan=10.0)
    rot = np.nan_to_num(rotvec(simulated["rot"] @ np.swapaxes(measured["rot"], -1, -2)[None]), nan=10.0)
    return {"joints": np.sqrt(np.mean(q ** 2, axis=(1, 2, 3))),
            "tcp_position": np.sqrt(np.mean(np.sum(pos ** 2, -1), axis=(1, 2))),
            "tcp_rotation": np.sqrt(np.mean(np.sum(rot ** 2, -1), axis=(1, 2)))}


def cost(error, weights):
    """The fit's error, as a length [m] (P,)."""
    return np.sqrt(error["tcp_position"] ** 2 + (weights["rotation"] * error["tcp_rotation"]) ** 2
                   + (weights["joint"] * error["joints"]) ** 2)


class CMA:
    """Plain CMA-ES (Hansen's tutorial defaults) minimizing over R^n from a mean of 0."""

    def __init__(self, n, popsize, sigma, seed):
        self.n, self.lam, self.mu = n, popsize, popsize // 2
        w = np.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.weights = w / w.sum()
        self.mueff = 1.0 / np.sum(self.weights ** 2)
        self.cc = (4 + self.mueff / n) / (n + 4 + 2 * self.mueff / n)
        self.cs = (self.mueff + 2) / (n + self.mueff + 5)
        self.c1 = 2 / ((n + 1.3) ** 2 + self.mueff)
        self.cmu = min(1 - self.c1, 2 * (self.mueff - 2 + 1 / self.mueff) / ((n + 2) ** 2 + self.mueff))
        self.damps = 1 + 2 * max(0.0, math.sqrt((self.mueff - 1) / (n + 1)) - 1) + self.cs
        self.chi = math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))
        self.mean, self.sigma = np.zeros(n), sigma
        self.pc, self.ps, self.C = np.zeros(n), np.zeros(n), np.eye(n)
        self.B, self.D = np.eye(n), np.ones(n)
        self.generation, self.rng = 0, np.random.default_rng(seed)

    def ask(self):
        z = self.rng.standard_normal((self.lam, self.n))
        return self.mean + self.sigma * (z * self.D) @ self.B.T

    def tell(self, x, cost):
        best = x[np.argsort(cost)[: self.mu]]
        old, self.mean = self.mean, self.weights @ best
        y = (self.mean - old) / self.sigma
        inv_sqrt = self.B @ np.diag(1 / self.D) @ self.B.T
        self.ps = (1 - self.cs) * self.ps + math.sqrt(self.cs * (2 - self.cs) * self.mueff) * inv_sqrt @ y
        self.generation += 1
        norm = np.linalg.norm(self.ps) / math.sqrt(1 - (1 - self.cs) ** (2 * self.generation))
        hsig = norm / self.chi < 1.4 + 2 / (self.n + 1)
        self.pc = (1 - self.cc) * self.pc + hsig * math.sqrt(self.cc * (2 - self.cc) * self.mueff) * y
        steps = (best - old) / self.sigma
        self.C = ((1 - self.c1 - self.cmu) * self.C
                  + self.c1 * (np.outer(self.pc, self.pc) + (1 - hsig) * self.cc * (2 - self.cc) * self.C)
                  + self.cmu * (steps.T * self.weights) @ steps)
        self.sigma *= math.exp(self.cs / self.damps * (np.linalg.norm(self.ps) / self.chi - 1))
        self.C = (self.C + self.C.T) / 2
        d2, self.B = np.linalg.eigh(self.C)
        self.D = np.sqrt(np.maximum(d2, 1e-20))


def decode(x, base):
    """Parameter sets from log offsets to the base values, clipped to BOUNDS: (P, 47) -> dict of (P, size)."""
    x = np.atleast_2d(x)
    out, i = {}, 0
    for name, size in SIZES.items():
        low, high = BOUNDS[name]
        out[name] = np.clip(np.maximum(base[name], low) * np.exp(x[:, i:i + size]), low, high)
        i += size
    return out


def fit(sim, starts, length, base, weights, generations, popsize, sigma, seed):
    """CMA-ES over log offsets to base; returns the best parameters and their error [m]."""
    measured = sim.measured(starts, length)
    cma = CMA(sum(SIZES.values()), popsize, sigma, seed)
    best_x = np.zeros(cma.n)
    best_cost = cost(errors(sim.rollout(decode(best_x, base), starts, length), measured), weights)[0]
    print(f"  generation   0: error {1000 * best_cost:.3f} mm (start)")
    t0 = time.perf_counter()
    for generation in range(1, generations + 1):
        x = cma.ask()
        costs = cost(errors(sim.rollout(decode(x, base), starts, length), measured), weights)
        cma.tell(x, costs)
        if costs.min() < best_cost:
            best_cost, best_x = costs.min(), x[costs.argmin()].copy()
        if generation % 20 == 0 or generation == generations:
            print(f"  generation {generation:3d}: error {1000 * best_cost:.3f} mm, sigma {cma.sigma:.3f} "
                  f"({time.perf_counter() - t0:.0f} s)")
    return {name: value[0] for name, value in decode(best_x, base).items()}, best_cost


def evaluate(sim, params, starts, length, weights):
    """The errors of one parameter set over the windows, and the fit's error [m, rad]."""
    if len(starts) == 0:
        return {key: float("nan") for key in ("error", "tcp_position", "tcp_rotation", "joints")}
    error = errors(sim.rollout({k: v[None] for k, v in params.items()}, starts, length),
                   sim.measured(starts, length))
    return {"error": float(cost(error, weights)[0]), **{key: float(value[0]) for key, value in error.items()}}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[1].replace("\n", " "),
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--traj", type=Path, required=True, help="Recording of 06_collect_osc_sysid.py")
    parser.add_argument("--physics_dt", "--physics-dt", type=float, required=True,
                        help="Physics step of your simulation [s], a whole number of ms dividing the policy period")
    parser.add_argument("--mass_matrix", "--mass-matrix", choices=("nominal", "plant"), default="nominal",
                        help="Armature in the OSC's mass matrix: fr3.xml's (as aiofranka) or the fitted one")
    parser.add_argument("--holdout", default="last",
                        help="Pose whose windows are left out of the fit: a pose name, last, or none")
    parser.add_argument("--rotation_weight", "--rotation-weight", type=float, default=0.1,
                        help="Weight of the TCP rotation error in the fit [m/rad]")
    parser.add_argument("--joint_weight", "--joint-weight", type=float, default=0.3,
                        help="Weight of the joint position error in the fit [m/rad]")
    parser.add_argument("--window", type=float, default=2.0, help="Window length [s]")
    parser.add_argument("--generations", type=int, default=200)
    parser.add_argument("--popsize", type=int, default=32)
    parser.add_argument("--sigma", type=float, default=0.3, help="Initial step size, in log of the parameters")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--threads", type=int, default=0, help="Simulation threads (0: all CPUs)")
    parser.add_argument("--out", type=Path, help="Output JSON (default: next to the recording)")
    args = parser.parse_args()

    run = Run(args.traj)
    sim = Simulator(run, args.physics_dt, args.mass_matrix, threads=args.threads)
    recorded = sorted({run.pose_names[run.pose[b]] for b in np.unique(run.block)}, key=run.pose_names.index)
    holdout = recorded[-1] if args.holdout == "last" and len(recorded) > 1 else args.holdout
    if holdout not in recorded + ["none", "last"]:
        parser.error(f"--holdout must be one of {recorded}, last or none")
    length = sim.window_length(args.window)
    train = run.windows(length, [p for p in recorded if p != holdout])
    test = run.windows(length, [holdout])

    print(f"\n  {args.traj.name}: {run.hz} Hz, TCP translation {run.tcp[:3, 3].tolist()} m, "
          f"payload {run.payload['mass']:.3f} kg")
    print("  " + ", ".join(f"{name} {run.gains[name].tolist()}" for name in GAINS))
    print(f"  null target {run.null_target.tolist()}")
    print(f"  Replay check: max |recomputed - sent torque| = {sim.replay_error():.1e} Nm")
    print(f"  physics_dt {args.physics_dt * 1000:g} ms, decimation {sim.decimation}, {args.mass_matrix} mass matrix; "
          f"fitting {len(train)} windows of {length / CONTROL_HZ:g} s, holding out {len(test)} at {holdout}\n")

    weights = {"rotation": args.rotation_weight, "joint": args.joint_weight}
    base = sim.start_values()
    params, _ = fit(sim, train, length, base, weights, args.generations, args.popsize, args.sigma, args.seed)

    results = {label: {"train": evaluate(sim, p, train, length, weights),
                       "held_out": evaluate(sim, p, test, length, weights)}
               for label, p in (("start", base), ("fit", params))}
    print(f"\n  RMS                          error [mm]   TCP position [mm]   TCP rotation [mrad]   joints [mrad]")
    for label, result in results.items():
        for split, e in result.items():
            name = f"{label}, {split.replace('_', ' ')}" + (f" ({holdout})" if split == "held_out" else "")
            print(f"  {name:28s} {1000 * e['error']:6.2f}   {1000 * e['tcp_position']:17.2f}"
                  f"   {1000 * e['tcp_rotation']:19.2f}   {1000 * e['joints']:13.2f}")
    print("\n  axis   ee_kp              ee_kd")
    for i, axis in enumerate(("x", "y", "z", "rx", "ry", "rz")):
        print(f"  {axis:4s}   " + "   ".join(f"{base[n][i]:7.4g} -> {params[n][i]:<7.4g}" for n in ("ee_kp", "ee_kd")))
    print("\n  joint  null_kp          null_kd          armature         damping          frictionloss")
    for j in range(7):
        print(f"  {j + 1}      " + "  ".join(f"{base[n][j]:6.3g} -> {params[n][j]:<6.3g}"
                                          for n in ("null_kp", "null_kd") + PHYSICAL))

    out = args.out or args.traj.with_name(f"{args.traj.stem}_fit_{args.physics_dt * 1000:g}ms.json")
    joint_xml = [f'<joint name="fr3_joint{j + 1}" armature="{params["armature"][j]:.5g}" '
                 f'damping="{params["damping"][j]:.5g}" frictionloss="{params["frictionloss"][j]:.5g}"/>'
                 for j in range(7)]
    report = {
        "data": str(args.traj),
        "physics_dt": args.physics_dt,
        "policy_hz": run.hz,
        "decimation": sim.decimation,
        "controller": {
            "law": "aiofranka OSC every physics step, rate limited and clipped, on <motor> actuators",
            **{name: params[name].tolist() for name in GAINS},
            "null_target": run.null_target.tolist(),
            "tcp": run.tcp.tolist(),
            "mass_matrix": args.mass_matrix,
            "mass_matrix_armature": (sim.nominal_armature if args.mass_matrix == "nominal"
                                     else params["armature"]).tolist(),
            "torque_rate_limit": run.rate_limit,
            "torque_limit": run.torque_limit.tolist(),
        },
        "joints": {n: params[n].tolist() for n in PHYSICAL},
        "mjcf": [f'<option timestep="{args.physics_dt:g}" integrator="implicitfast"/>'] + joint_xml,
        "start": {n: base[n].tolist() for n in SIZES},
        "rms": results,
        "rms_units": {"error": "m", "tcp_position": "m", "tcp_rotation": "rad", "joints": "rad"},
        "weights": weights,
        "holdout": holdout,
        "settings": {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()},
    }
    out.write_text(json.dumps(report, indent=1))
    print(f"\n  Saved {out}\n")


if __name__ == "__main__":
    main()
