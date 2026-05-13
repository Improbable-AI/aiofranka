"""
Collect sysid trajectories with decoupled position-command and recording
frequencies.  Position targets update at --freq Hz (e.g. 10 Hz) while
joint state is recorded at --record_freq Hz (e.g. 50 Hz).

Usage:
    python examples/01_sample_hf_vel.py --freq 10 --record_freq 50
"""

import asyncio
import numpy as np
import argparse
import os
from aiofranka.robot import RobotInterface
from aiofranka import FrankaController


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--freq", required=True, type=float, help="Position target update frequency (Hz)")
    parser.add_argument("--record_freq", required=True, type=float, help="State recording frequency (Hz)")
    args = parser.parse_args()

    if args.record_freq < args.freq:
        parser.error("--record_freq must be >= --freq")

    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)
    await controller.start()

    base = np.array([1, 1, 1, 1, 0.6, 0.6, 0.6])

    kps = [ 16, 32, 64, 128, 160, 256, 512 ]
    kds = [ 1, 2, 4, 8, 12, 16, 24 ]
    kp_kd_pairs = [(kp, kd) for kp in kps for kd in kds]

    for kp, kd in kp_kd_pairs:
        with controller.state_lock:
            controller.kp = base * 80
            controller.kd = base * 4
            print("Moving to initial position...")
        await controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])

        print(f"Testing with kp={kp}, kd={kd}, cmd freq={args.freq}, record freq={args.record_freq}")
        await asyncio.sleep(1.0)

        controller.switch("impedance")
        controller.set_freq(int(args.freq))

        with controller.state_lock:
            controller.kp = base * kp
            controller.kd = base * kd

        logs = {
            'qpos': [],
            'qvel': [],
            'qdes': [],
            'ctrl': [],
        }

        duration = 4.0
        total_records = int(args.record_freq * duration)

        record_done = asyncio.Event()
        cmd_count = 0

        async def record_loop():
            for i in range(total_records):
                logs['qpos'].append(controller.robot.data.qpos.copy())
                logs['qvel'].append(controller.robot.data.qvel.copy())
                logs['ctrl'].append(controller.robot.data.ctrl.copy())
                logs['qdes'].append(controller.q_desired.copy())
                await asyncio.sleep(1.0 / args.record_freq)
            record_done.set()

        async def cmd_loop():
            nonlocal cmd_count
            total_cmds = int(args.freq * duration)
            for cnt in range(total_cmds):
                delta = np.sin(cnt / args.freq * np.pi) * 0.1
                init = controller.initial_qpos
                await controller.set("q_desired", delta + init)
                cmd_count = cnt + 1
                if record_done.is_set():
                    break
                await asyncio.sleep(1.0 / args.freq)

        await asyncio.gather(record_loop(), cmd_loop())

        for key in logs:
            logs[key] = np.stack(logs[key])

        dir = f"sysid_traj_freq_{int(args.freq)}_rec_{int(args.record_freq)}"
        os.makedirs(f"./examples/{dir}/", exist_ok=True)
        path = f"./examples/{dir}/sysid_K{int(kp)}_D{int(kd)}.npz"
        np.savez(
            path,
            **logs,
            cmd_freq=args.freq,
            record_freq=args.record_freq,
        )
        print(f"Saved to {path}")
        print(f"  Command steps: {cmd_count}, Record samples: {logs['qpos'].shape[0]}")


if __name__ == "__main__":
    asyncio.run(main())
