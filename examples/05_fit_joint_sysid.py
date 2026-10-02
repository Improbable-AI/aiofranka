#!/usr/bin/env python3
"""
Fit a MuJoCo FR3 to a 04_collect_joint_sysid.py recording with CMA-ES, at your simulation's
physics step.

The simulation runs the way a policy's environment does: at each policy step it takes
the recorded joint targets, and for each of its decimation = (1 / hz) / physics_dt
physics steps it computes aiofranka's joint impedance in Python, kp * (q_des - q) -
kd * dq with the 990 Nm/s rate limit and the torque clip, and applies it through
fr3.xml's <motor> actuators. Starting from the recorded kp and kd and fr3.xml's joint
parameters, it fits kp, kd, armature, damping and friction loss of each joint to the
measured joint positions, replaying 2 s windows from the measured state at their start.
The windows at one pose are held out to check the fit.

    pip install mjbatch   # batched MuJoCo; it pins its own mujoco version
    python examples/05_fit_joint_sysid.py --traj examples/sysid_data/joint_sysid_<date>.npz --physics_dt 0.002

Adds the fit to the sim section of the configuration the recording was collected with
(or --activate), as the entry for this physics_dt (see aiofranka.config). kd and joint
damping both damp the joint velocity, so mostly their sum is determined; the fit reports both.
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np
from mjbatch import Batch

from aiofranka.config import load_config, same_controller, save_sim
from aiofranka.payload import MODEL_PATH
from aiofranka.robot import link_inertial, merge_payload

CONTROL_HZ = 1000
PARAMETERS = ("kp", "kd", "armature", "damping", "frictionloss")
BOUNDS = {"kp": (1.0, 2000.0), "kd": (0.01, 500.0), "armature": (0.001, 2.0),
          "damping": (0.001, 20.0), "frictionloss": (0.001, 10.0)}
PHYSICAL = PARAMETERS[2:]


class Run:
    """A 04_collect_joint_sysid.py recording: one gain set and one policy rate."""

    def __init__(self, path):
        data = np.load(path)
        self.meta = json.loads(str(data["meta"]))
        if self.meta.get("controller", "impedance") != "impedance":
            raise ValueError("Not a joint impedance recording; fit OSC recordings with 07_fit_osc_sysid.py")
        for key in ("time", "q", "dq", "q_des", "tau_J_d", "pose"):
            setattr(self, key, data[key])
        self.block = data["block"].astype(int)
        kp, kd, hz = data["kp"], data["kd"], data["rate_hz"]
        if len(np.unique(kp, axis=0)) > 1 or len(np.unique(kd, axis=0)) > 1 or len(np.unique(hz)) > 1:
            raise ValueError("The recording has several gain sets or rates; collect one configuration per recording")
        self.kp, self.kd, self.hz = kp[0].astype(float), kd[0].astype(float), int(hz[0])
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
    """
    fr3.xml in free space, with the payload the robot compensated merged into its last
    link as RobotInterface.sync_payload() merges it into the model of aiofranka's controllers.
    """
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    model.opt.timestep = physics_dt
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    payload = run.payload
    merge_payload(model, link_inertial(model), payload["mass"], payload["com"], payload["inertia"])
    mujoco.mj_setConst(model, mujoco.MjData(model))
    return model


class Simulator:
    """Batched closed-loop replays of windows of a run."""

    def __init__(self, run, physics_dt, threads=0):
        self.run, self.physics_dt, self.threads = run, physics_dt, threads
        self.period = CONTROL_HZ // run.hz  # ticks per policy step
        self.ticks = round(physics_dt * CONTROL_HZ)  # ticks per physics step
        if not math.isclose(self.ticks / CONTROL_HZ, physics_dt) or self.period % self.ticks:
            raise ValueError(f"physics_dt must divide the policy period, 1 / {run.hz} s, in whole milliseconds")
        self.decimation = self.period // self.ticks
        self.every = math.lcm(self.ticks, 10)  # ticks between position samples
        self.model = build_model(run, physics_dt)
        self.batches = {}

    def window_length(self, seconds):
        """Ticks closest to seconds that hold whole policy steps and samples."""
        unit = math.lcm(self.period, self.every)
        return max(1, round(seconds * CONTROL_HZ / unit)) * unit

    def start_values(self):
        """Recorded kp and kd and fr3.xml's joint parameters."""
        values = {"kp": self.run.kp.copy(), "kd": self.run.kd.copy()}
        values.update({name: getattr(self.model, f"dof_{name}")[:7].copy() for name in PHYSICAL})
        return values

    def _batch(self, count):
        if count not in self.batches:
            batch = Batch(self.model, count, self.threads)
            fields = {name: batch.bind(name) for name in ("qpos", "qvel", "ctrl")}
            expanded = {name: batch.expand(f"dof_{name}") for name in PHYSICAL}
            self.batches = {count: (batch, fields, expanded)}  # keep one, they are large
        return self.batches[count]

    def rollout(self, params, starts, length):
        """
        Simulated joint positions every self.every ticks of each window, for each parameter set.

        Args:
            params (dict): kp, kd, armature, damping, frictionloss per parameter set (P, 7)
            starts (np.ndarray): First ticks of the windows (W,)
            length (int): Ticks per window, from window_length()

        Returns:
            np.ndarray: (P, W, length // self.every, 7)
        """
        run = self.run
        count_params, count_windows = len(params["kp"]), len(starts)
        batch, fields, expanded = self._batch(count_params * count_windows)
        for name in PHYSICAL:
            expanded[name][:, :7] = np.repeat(params[name], count_windows, axis=0)
        batch.set_const()
        ticks = np.tile(starts, count_params)
        fields["qpos"][:] = run.q[ticks]
        fields["qvel"][:] = run.dq[ticks]
        kp = np.repeat(params["kp"], count_windows, axis=0)
        kd = np.repeat(params["kd"], count_windows, axis=0)
        previous = run.tau_J_d[ticks].copy()  # the robot's last command at the window start
        step_limit = run.rate_limit * self.physics_dt
        out = np.empty((len(ticks), length // self.every, 7))
        elapsed = 0  # ticks since the window start
        for step in range(length // self.period):
            target = run.q_des[ticks + step * self.period]  # the policy's action, held for the step
            for _ in range(self.decimation):
                tau = kp * (target - fields["qpos"]) - kd * fields["qvel"]
                tau = previous + np.clip(tau - previous, -step_limit, step_limit)
                tau = np.clip(tau, -run.torque_limit, run.torque_limit)
                fields["ctrl"][:] = tau
                previous = tau
                batch.step()
                elapsed += self.ticks
                if elapsed % self.every == 0:
                    out[:, elapsed // self.every - 1] = fields["qpos"]
        return out.reshape(count_params, count_windows, -1, 7)

    def measured(self, starts, length):
        return self.run.q[starts[:, None] + np.arange(self.every, length + 1, self.every)[None]]


def rms(simulated, measured):
    """RMS joint position error of each parameter set [rad] (P,)."""
    error = np.nan_to_num(simulated - measured[None], nan=10.0)
    return np.sqrt(np.mean(error ** 2, axis=(1, 2, 3)))


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
    """Parameter sets from log offsets to the base values, clipped to BOUNDS: (P, 35) -> dict of (P, 7)."""
    x = np.atleast_2d(x).reshape(-1, len(PARAMETERS), 7)
    return {name: np.clip(np.maximum(base[name], BOUNDS[name][0]) * np.exp(x[:, i]), *BOUNDS[name])
            for i, name in enumerate(PARAMETERS)}


def fit(sim, starts, length, base, generations, popsize, sigma, seed):
    """CMA-ES over log offsets to base; returns the best parameters and their RMS error."""
    measured = sim.measured(starts, length)
    cma = CMA(len(PARAMETERS) * 7, popsize, sigma, seed)
    best_x = np.zeros(cma.n)
    best_cost = rms(sim.rollout(decode(best_x, base), starts, length), measured)[0]
    print(f"  generation   0: RMS {1000 * best_cost:.3f} mrad (start)")
    t0 = time.perf_counter()
    for generation in range(1, generations + 1):
        x = cma.ask()
        cost = rms(sim.rollout(decode(x, base), starts, length), measured)
        cma.tell(x, cost)
        if cost.min() < best_cost:
            best_cost, best_x = cost.min(), x[cost.argmin()].copy()
        if generation % 20 == 0 or generation == generations:
            print(f"  generation {generation:3d}: RMS {1000 * best_cost:.3f} mrad, sigma {cma.sigma:.3f} "
                  f"({time.perf_counter() - t0:.0f} s)")
    return {name: value[0] for name, value in decode(best_x, base).items()}, best_cost


def error(sim, params, starts, length):
    """RMS error of one parameter set over the windows [rad]."""
    if len(starts) == 0:
        return float("nan")
    return float(rms(sim.rollout({k: v[None] for k, v in params.items()}, starts, length),
                     sim.measured(starts, length))[0])


def configuration(run, path):
    """
    The configuration file to add the fit to, path or the recording's, if it describes the
    recorded controller; and the recorded controller.
    """
    if path is None:
        if run.meta.get("sim"):
            raise ValueError("The recording comes from MuJoCo: pass --activate to say which configuration "
                             "its fit goes to")
        if not run.meta.get("config_path"):
            raise ValueError("The recording does not name its configuration; pass --activate")
        path = run.meta["config_path"]
        if not Path(path).exists():
            raise ValueError(f"The recording's configuration {path} is not there; pass --activate")
    path = Path(path)
    config = load_config(path)
    recorded = run.meta.get("config") or {"mode": "impedance", "kp": run.kp.tolist(), "kd": run.kd.tolist(),
                                          "frequency": run.hz, "tool": config.get("tool")}
    if not same_controller(config, recorded):
        raise ValueError(f"{path} no longer describes the controller of the recording")
    return path, recorded


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[1].replace("\n", " "),
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--traj", type=Path, required=True, help="Recording of 04_collect_joint_sysid.py")
    parser.add_argument("--activate", type=Path,
                        help="Configuration to add the fit to (default: the one the recording was collected with)")
    parser.add_argument("--physics_dt", "--physics-dt", type=float, required=True,
                        help="Physics step of your simulation [s], a whole number of ms dividing the policy period")
    parser.add_argument("--holdout", default="last",
                        help="Pose whose windows are left out of the fit: a pose name, last, or none")
    parser.add_argument("--window", type=float, default=2.0, help="Window length [s]")
    parser.add_argument("--generations", type=int, default=200)
    parser.add_argument("--popsize", type=int, default=24)
    parser.add_argument("--sigma", type=float, default=0.3, help="Initial step size, in log of the parameters")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--threads", type=int, default=0, help="Simulation threads (0: all CPUs)")
    args = parser.parse_args()

    try:
        run = Run(args.traj)
        config_path, recorded_controller = configuration(run, args.activate)
    except (OSError, ValueError) as problem:
        parser.error(str(problem))
    sim = Simulator(run, args.physics_dt, threads=args.threads)
    recorded = sorted({run.pose_names[run.pose[b]] for b in np.unique(run.block)}, key=run.pose_names.index)
    holdout = recorded[-1] if args.holdout == "last" and len(recorded) > 1 else args.holdout
    if holdout not in recorded + ["none", "last"]:
        parser.error(f"--holdout must be one of {recorded}, last or none")
    length = sim.window_length(args.window)
    train = run.windows(length, [p for p in recorded if p != holdout])
    test = run.windows(length, [holdout])

    print(f"\n  {args.traj.name}: kp {run.kp.tolist()}, kd {run.kd.tolist()}, {run.hz} Hz")
    print(f"  Tool {run.meta.get('tool') or '(not read from Desk)'}: the model carries the payload the robot compensated, "
          f"{run.payload['mass']:.3f} kg at {np.round(run.payload['com'], 4).tolist()} m")
    print(f"  physics_dt {args.physics_dt * 1000:g} ms, decimation {sim.decimation}; fitting {len(train)} windows "
          f"of {length / CONTROL_HZ:g} s, holding out {len(test)} at {holdout}\n")

    base = sim.start_values()
    params, cost = fit(sim, train, length, base, args.generations, args.popsize, args.sigma, args.seed)

    errors = {label: {"train": error(sim, p, train, length), "held_out": error(sim, p, test, length)}
              for label, p in (("start", base), ("fit", params))}
    print(f"\n  RMS error [mrad]   train    held out ({holdout})")
    for label, e in errors.items():
        print(f"  {label:15s}  {1000 * e['train']:6.2f}   {1000 * e['held_out']:6.2f}")
    print("\n  joint   kp                kd              armature        damping         frictionloss    kd + damping")
    for j in range(7):
        cells = "  ".join(f"{base[n][j]:6.3g} -> {params[n][j]:<6.3g}" for n in PARAMETERS)
        print(f"  {j + 1}     {cells}  {params['kd'][j] + params['damping'][j]:.3g}")

    save_sim(config_path, {
        "physics_dt": args.physics_dt,
        **{name: params[name] for name in PARAMETERS},
        # The payload the fit assumed, merged into fr3_link7 as aiofranka.robot.merge_payload() does.
        "payload": {"mass": run.payload["mass"], "com": run.payload["com"], "inertia": run.payload["inertia"]},
        "fit": {
            "data": args.traj.name,
            "date": datetime.datetime.now().isoformat(timespec="seconds"),
            "holdout": holdout,
            "plant": "mujoco" if run.meta.get("sim") else "robot",
            "tool": run.meta.get("tool"),
            "rms_mrad": {label: {split: 1000 * value for split, value in e.items()} for label, e in errors.items()},
        },
    }, controller=recorded_controller)
    print(f"\n  Added the fit for physics_dt {args.physics_dt:g} to {config_path}\n")


if __name__ == "__main__":
    main()
