import pyspacemouse
import time
import asyncio
import numpy as np
from aiofranka.robot import RobotInterface
from aiofranka import FrankaController
from scipy.spatial.transform import Rotation as R


async def main(): 

    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)

    await controller.start()

    await controller.move()

    await asyncio.sleep(1.0)

    controller.switch("osc")
    controller.ee_kp = np.array([300.0, 300.0, 300.0, 300.0, 300.0, 300.0])
    controller.ee_kd = np.ones(6) * 10.0
    controller.set_freq(50)  # Set 100Hz update rate

    success = pyspacemouse.open()
    if success: 
        print("Spacemouse connected. Use it to move the robot end-effector.")
    else: 
        print("Failed to connect Spacemouse.")
        return
    
    while True:
        event = pyspacemouse.read()

        # Scale the inputs to get reasonable movements
        translation_delta = np.clip(np.array([event.x, event.y, event.z]) * 0.03, -0.02, 0.02) 
        rotation_delta = np.array([-event.pitch, event.roll, -event.yaw]) * 5.0
        rotation_delta = np.clip(rotation_delta, -3.0, 3.0)
        rotation_delta = R.from_euler('xyz', rotation_delta, degrees=True).as_matrix() 

        # Get current desired end-effector pose
        with controller.state_lock:
            current_ee = controller.state['ee'].copy()

        # Update position
        current_ee[:3, 3] += translation_delta
        current_ee[:3, :3] = rotation_delta @ current_ee[:3, :3]

        # # Update orientation using small angle approximation
        # rot_matrix = euler_to_rot_matrix(rotation_delta)
        # current_ee[:3, :3] = rot_matrix @ current_ee[:3, :3]

        await controller.set("ee_desired", current_ee)


if __name__ == "__main__":
    asyncio.run(main())