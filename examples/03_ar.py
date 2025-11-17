import pyspacemouse
import time
import asyncio
import numpy as np
from aiofranka.robot import RobotInterface
from aiofranka import FrankaController
from scipy.spatial.transform import Rotation as R
import amfiprot
import amfiprot_amfitrack as amfitrack
from loop_rate_limiters import RateLimiter
from copy import deepcopy
from mujoco_ar import MujocoARConnector




async def main(): 

    robot = RobotInterface()
    controller = FrankaController(robot)
    # Initialize the connector with your desired parameters
    connector = MujocoARConnector()

    # Start the connector
    connector.start()

    # Retrieve the latest AR data (after connecting the iOS device, see the guide below)
    data = connector.get_latest_data()  # Returns {"position": (3, 1), "rotation": (3, 3), "button": bool, "toggle": bool}
    

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

        data = connector.get_latest_data()
        print(data)
        if data['position'] is not None: 
            init_pos = deepcopy(np.array(data['position']))
            print("Initial position acquired:", init_pos)
            break 

    
    while True:

        
        data = connector.get_latest_data()
        position = data['position']
        # print(position)
        
        # Scale the inputs to get reasonable movements
        # translation_delta = np.clip(np.array([event.x, event.y, event.z]) * 0.03, -0.02, 0.02) 
        # # rotation_delta = np.array([-event.pitch, event.roll, -event.yaw]) * 5.0
        # # rotation_delta = np.clip(rotation_delta, -3.0, 3.0)
        # rotation_delta = R.from_euler('xyz', rotation_delta, degrees=True).as_matrix() 

        # Get current desired end-effector pose
        with controller.state_lock:
            target_ee = deepcopy(controller.initial_ee)

            # Update position
            target_ee[:3, 3] += (position - init_pos) * 0.01

        # print(position, init_pos)

        # print(position - init_pos)
        # # current_ee[:3, :3] = rotation_delta @ current_ee[:3, :3]

        # # Update orientation using small angle approximation
        # rot_matrix = euler_to_rot_matrix(rotation_delta)
        # current_ee[:3, :3] = rot_matrix @ current_ee[:3, :3]

        await controller.set("ee_desired", target_ee)


if __name__ == "__main__":
    asyncio.run(main())