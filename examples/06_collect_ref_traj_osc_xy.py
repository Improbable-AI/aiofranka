"""Collect 50 Hz planar OSC sine responses at several Franka poses."""

import asyncio
import os

import numpy as np

from aiofranka import FrankaController
from aiofranka.robot import RobotInterface


POSTURES = {
    "low": {
        "qpos": [
            1.550789783,
            -1.362605144,
            -1.732187188,
            -2.522379601,
            -1.579155054,
            1.833371802,
            1.602923482,
        ],
        "xy_kp": 64.0,
        "xy_kd": 16.0,
    },
    "mid": {
        "qpos": [
            -1.381162231,
            0.670045700,
            1.202792184,
            -2.552114918,
            -1.163445926,
            2.458645076,
            -0.092722208,
        ],
        "xy_kp": 64.0,
        "xy_kd": 16.0,
    },
    "high": {
        "qpos": [
            -0.667854741,
            0.237641239,
            0.585359676,
            -2.639087670,
            -0.413853269,
            2.812239851,
            -0.050493410,
        ],
        "xy_kp": 64.0,
        "xy_kd": 16.0,
    },
}

CONTROL_FREQ = 50
N_STEPS_PER_AXIS = 200
AMPLITUDE = 0.07
FREQUENCY_HZ = 0.5


async def main():
    robot = RobotInterface("173.16.0.2")
    controller = FrankaController(robot)
    await controller.start()
    controller.set_freq(CONTROL_FREQ)

    os.makedirs("./examples/sysid_osc_xy", exist_ok=True)

    try:
        for posture_name, posture in POSTURES.items():
            qpos = posture["qpos"]
            kp = posture["xy_kp"]
            kd = posture["xy_kd"]

            # move() runs the joint-space impedance motion to the chosen pose.
            controller.switch("impedance")
            with controller.state_lock:
                controller.kp = np.array([80, 80, 80, 80, 48, 48, 48])
                controller.kd = np.array([8, 8, 8, 8, 6, 6, 6])
            await controller.move(qpos)
            await asyncio.sleep(2.0)

            controller.switch("osc")
            with controller.state_lock:
                # The OSC's mx(q) supplies the posture-dependent physical gains.
                # Keeping 64/16 fixed gives the same ideal free-space response.
                controller.ee_kp = np.array([kp, kp, 100, 100, 100, 100])
                controller.ee_kd = np.array([kd, kd, 20, 20, 20, 20])
                controller.null_kp = np.ones(7) * 9
                controller.null_kd = np.ones(7) * 6
                initial_ee = controller.ee_desired.copy()

            logs = {
                "qpos": [],
                "qvel": [],
                "ee": [],
                "ee_desired": [],
                "ctrl": [],
                "axis": [],
                "command_xy": [],
            }

            # First excite X, then excite Y. The command itself is only [dx, dy].
            for axis in (0, 1):
                for step in range(N_STEPS_PER_AXIS):
                    with controller.state_lock:
                        state = controller.state
                        logs["qpos"].append(state["qpos"].copy())
                        logs["qvel"].append(state["qvel"].copy())
                        logs["ee"].append(state["ee"].copy())
                        logs["ctrl"].append(state["last_torque"].copy())

                    time_s = step / CONTROL_FREQ
                    delta = AMPLITUDE * np.sin(
                        2.0 * np.pi * FREQUENCY_HZ * time_s
                    )
                    command_xy = np.zeros(2)
                    command_xy[axis] = delta
                    target_ee = initial_ee.copy()
                    target_ee[:2, 3] += command_xy
                    logs["ee_desired"].append(target_ee.copy())
                    logs["axis"].append(axis)
                    logs["command_xy"].append(command_xy)
                    await controller.set("ee_desired", target_ee)

                await controller.set("ee_desired", initial_ee.copy())
                await asyncio.sleep(1.0)

            for key in logs:
                logs[key] = np.stack(logs[key])

            output = (
                "./examples/sysid_osc_xy/"
                f"{posture_name}_K{kp:g}_D{kd:g}.npz"
            )
            np.savez(output, **logs)
            print(f"Saved {output}")
    finally:
        await controller.stop()


if __name__ == "__main__":
    asyncio.run(main())
