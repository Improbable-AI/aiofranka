"""
This script collects reference trajectory data under different impedance control gains.
It moves the Franka robot arm in a sinusoidal pattern while logging joint positions, velocities,
desired positions, and control torques. The collected data is saved to a .npz file for further analysis.
"""


import asyncio 
import numpy as np 
from aiofranka.robot import RobotInterface
from aiofranka import FrankaController
import time 


async def main():
    robot = RobotInterface("172.16.0.2") 
    # robot = RobotInterface()
    controller = FrankaController(robot)


    await controller.start()

    kp_kds = [
        (120.0, 10.0),
        (80.0, 4.0), 
        (60.0, 3.0), 
        (40.0, 2.0), 
    ]
    

    logs = { 
        'qpos': [], 
        'qvel': [],
        'qdes': [], 
        'ctrl': [], 
    }

    for kp, kd in kp_kds:
        print(f"Testing with kp={kp}, kd={kd}")
        await controller.move([0, 0, 0.3, -1.57079, 0, 1.57079, -0.7853])

        await asyncio.sleep(1.0)

        # run the controller test 
        print("switched to impedance control")
        controller.switch("impedance")

        with controller.state_lock:
            controller.kp = np.ones(7) * kp
            controller.kd = np.ones(7) * kd
        print(controller.kp, controller.kd)


        for cnt in range(200): 

            logs['qpos'].append(controller.robot.data.qpos.copy())
            logs['qvel'].append(controller.robot.data.qvel.copy())
            logs['ctrl'].append(controller.robot.data.ctrl.copy())
            logs['qdes'].append(controller.q_desired.copy())

            delta = np.sin(cnt / 50.0 * np.pi) * 0.15
            init = controller.initial_qpos
            print(controller.torque)
            await controller.set("q_desired", delta + init)

        # await controller.stop()
        await asyncio.sleep(1.0)

    for key in logs.keys():
        logs[key] = np.stack(logs[key])
    np.savez("./examples/sysid.npz", **logs)


if __name__ == "__main__":
    asyncio.run(main())