Examples
========

This page provides complete, working examples for common use cases.

Example 1: Simple Motion
-------------------------

Move the robot through a series of positions:

.. code-block:: python

   import asyncio
   import numpy as np
   from aiofranka import RobotInterface, FrankaController

   async def simple_motion():
       robot = RobotInterface("172.16.0.2")
       controller = FrankaController(robot)
       
       await controller.start()
       
       try:
           # Define waypoints
           home = [0, 0, 0, -1.57079, 0, 1.57079, -0.7853]
           pose1 = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
           pose2 = [0.5, -0.785, 0, -2.356, 0, 1.571, 0.785]
           
           # Move through waypoints
           for pose in [home, pose1, pose2, home]:
               print(f"Moving to: {pose}")
               await controller.move(pose)
               await asyncio.sleep(1.0)  # Pause at each waypoint
               
       finally:
           await controller.stop()

   if __name__ == "__main__":
       asyncio.run(simple_motion())

Example 2: Impedance Control
-----------------------------

Compliant joint-space control with sinusoidal motion:

.. code-block:: python

   import asyncio
   import numpy as np
   from aiofranka import RobotInterface, FrankaController

   async def impedance_demo():
       robot = RobotInterface("172.16.0.2")
       controller = FrankaController(robot)
       
       await controller.start()
       
       try:
           # Move to start position
           await controller.move([0, 0, 0.3, -1.57, 0, 1.57, -0.785])
           
           # Configure impedance control
           controller.switch("impedance")
           controller.kp = np.ones(7) * 80.0
           controller.kd = np.ones(7) * 4.0
           controller.set_freq(50)
           
           # Execute smooth motion
           print("Executing sinusoidal motion...")
           for i in range(200):  # 4 seconds at 50 Hz
               # Sinusoidal variation on joint 3
               delta = np.sin(i / 50.0 * np.pi) * 0.15
               target = controller.initial_qpos.copy()
               target[2] += delta
               
               await controller.set("q_desired", target)
               
           print("Motion complete!")
           
       finally:
           await controller.stop()

   if __name__ == "__main__":
       asyncio.run(impedance_demo())

Example 3: Operational Space Control
-------------------------------------

Control end-effector position in Cartesian space:

.. code-block:: python

   import asyncio
   import numpy as np
   from aiofranka import RobotInterface, FrankaController
   from scipy.spatial.transform import Rotation as R

   async def osc_demo():
       robot = RobotInterface("172.16.0.2")
       controller = FrankaController(robot)
       
       await controller.start()
       
       try:
           # Move to start position
           await controller.move()
           await asyncio.sleep(1.0)
           
           # Configure OSC
           controller.switch("osc")
           controller.ee_kp = np.array([300, 300, 300, 1000, 1000, 1000])
           controller.ee_kd = np.ones(6) * 10.0
           controller.set_freq(50)
           
           print("Moving in Cartesian space...")
           
           # Circular motion in XY plane
           for i in range(200):
               angle = i / 50.0 * np.pi
               radius = 0.05
               
               desired_ee = controller.initial_ee.copy()
               desired_ee[0, 3] += radius * np.cos(angle)
               desired_ee[1, 3] += radius * np.sin(angle)
               
               await controller.set("ee_desired", desired_ee)
               
           print("Circle complete!")
           
       finally:
           await controller.stop()

   if __name__ == "__main__":
       asyncio.run(osc_demo())

Example 4: Data Collection
---------------------------

Collect synchronized robot data during operation:

.. code-block:: python

   import asyncio
   import numpy as np
   from aiofranka import RobotInterface, FrankaController
   import time

   async def collect_data():
       robot = RobotInterface("172.16.0.2")
       controller = FrankaController(robot)
       
       await controller.start()
       
       try:
           # Move to start
           await controller.move([0, 0, 0.3, -1.57, 0, 1.57, -0.785])
           
           # Setup impedance control
           controller.switch("impedance")
           controller.kp = np.ones(7) * 80.0
           controller.kd = np.ones(7) * 4.0
           controller.set_freq(100)  # 100 Hz
           
           # Data storage
           logs = {
               'qpos': [],
               'qvel': [],
               'qdes': [],
               'ctrl': [],
               'ee': [],
               'timestamp': []
           }
           
           start_time = time.time()
           
           print("Collecting data...")
           for i in range(500):  # 5 seconds at 100 Hz
               # Log data
               state = controller.state
               logs['qpos'].append(state['qpos'].copy())
               logs['qvel'].append(state['qvel'].copy())
               logs['ctrl'].append(state['last_torque'].copy())
               logs['ee'].append(state['ee'].copy())
               logs['qdes'].append(controller.q_desired.copy())
               logs['timestamp'].append(time.time() - start_time)
               
               # Execute motion
               delta = np.sin(i / 100.0 * 2 * np.pi) * 0.1
               target = controller.initial_qpos.copy()
               target[2] += delta
               
               await controller.set("q_desired", target)
           
           # Convert to numpy arrays
           for key in logs:
               logs[key] = np.array(logs[key])
           
           # Save data
           np.savez("robot_data.npz", **logs)
           print(f"Saved {len(logs['qpos'])} samples to robot_data.npz")
           
       finally:
           await controller.stop()

   if __name__ == "__main__":
       asyncio.run(collect_data())

Example 5: Teleoperation
------------------------

Control robot with SpaceMouse (requires ``pyspacemouse``):

.. code-block:: python

   import asyncio
   import numpy as np
   from aiofranka import RobotInterface, FrankaController
   from scipy.spatial.transform import Rotation as R
   import pyspacemouse

   async def teleoperation():
       robot = RobotInterface("172.16.0.2")
       controller = FrankaController(robot)
       
       await controller.start()
       
       # Connect SpaceMouse
       success = pyspacemouse.open()
       if not success:
           print("Failed to connect SpaceMouse")
           await controller.stop()
           return
       
       try:
           # Move to start
           await controller.move()
           await asyncio.sleep(1.0)
           
           # Configure OSC
           controller.switch("osc")
           controller.ee_kp = np.array([300, 300, 300, 1000, 1000, 1000])
           controller.ee_kd = np.ones(6) * 10.0
           controller.set_freq(50)
           
           print("Teleoperation active. Use SpaceMouse to control robot.")
           print("Press Ctrl+C to stop.")
           
           while True:
               # Read SpaceMouse
               event = pyspacemouse.read()
               
               # Scale inputs
               translation = np.clip(
                   np.array([event.x, event.y, event.z]) * 0.003,
                   -0.003, 0.003
               )
               rotation = np.array([0, 0, -event.yaw]) * 0.5
               rotation = np.clip(rotation, -0.5, 0.5)
               rotation_delta = R.from_euler('xyz', rotation, degrees=True).as_matrix()
               
               # Update desired pose
               current_ee = controller.ee_desired.copy()
               current_ee[:3, 3] += translation
               current_ee[:3, :3] = rotation_delta @ current_ee[:3, :3]
               
               await controller.set("ee_desired", current_ee)
               
       except KeyboardInterrupt:
           print("\nStopping teleoperation...")
           
       finally:
           pyspacemouse.close()
           await controller.stop()

   if __name__ == "__main__":
       asyncio.run(teleoperation())

Example 6: Custom Controller
-----------------------------

Implement your own control law using torque mode:

.. code-block:: python

   import asyncio
   import numpy as np
   from aiofranka import RobotInterface, FrankaController

   async def custom_controller():
       robot = RobotInterface("172.16.0.2")
       controller = FrankaController(robot)
       
       await controller.start()
       
       try:
           # Move to start
           await controller.move()
           
           # Switch to torque mode
           controller.switch("torque")
           
           # Control parameters
           kp = np.ones(7) * 60.0
           kd = np.ones(7) * 3.0
           target = controller.initial_qpos.copy()
           
           print("Running custom controller...")
           
           for i in range(500):
               # Get state
               state = controller.state
               q = state['qpos']
               dq = state['qvel']
               
               # Compute control torque (simple PD)
               tau = kp * (target - q) - kd * dq
               
               # Set torque (thread-safe)
               with controller.state_lock:
                   controller.torque = tau
               
               await asyncio.sleep(1.0 / 500.0)  # 500 Hz
               
           print("Custom control complete!")
           
       finally:
           await controller.stop()

   if __name__ == "__main__":
       asyncio.run(custom_controller())

.. warning::
   Direct torque control requires careful implementation. Test in simulation first!

Example 7: Gain Tuning
----------------------

Systematically test different controller gains:

.. code-block:: python

   import asyncio
   import numpy as np
   from aiofranka import RobotInterface, FrankaController

   async def gain_tuning():
       robot = RobotInterface("172.16.0.2")
       controller = FrankaController(robot)
       
       await controller.start()
       
       try:
           # Test different gain combinations
           kp_values = [20, 40, 80, 160]
           kd_values = [2, 4, 8]
           
           for kp in kp_values:
               for kd in kd_values:
                   print(f"\nTesting kp={kp}, kd={kd}")
                   
                   # Move to start
                   await controller.move([0, 0, 0.3, -1.57, 0, 1.57, -0.785])
                   
                   # Configure gains
                   controller.switch("impedance")
                   controller.kp = np.ones(7) * kp
                   controller.kd = np.ones(7) * kd
                   controller.set_freq(50)
                   
                   # Test motion
                   for i in range(100):
                       delta = np.sin(i / 50.0 * np.pi) * 0.1
                       target = controller.initial_qpos.copy()
                       target[2] += delta
                       await controller.set("q_desired", target)
                   
                   await asyncio.sleep(1.0)
                   
           print("\nGain tuning complete!")
           
       finally:
           await controller.stop()

   if __name__ == "__main__":
       asyncio.run(gain_tuning())

Example 8: Simulation Testing
------------------------------

Test your controller in simulation before deploying to real robot:

.. code-block:: python

   import asyncio
   import numpy as np
   from aiofranka import RobotInterface, FrankaController

   async def test_algorithm(robot_ip=None):
       """
       Test control algorithm.
       
       Args:
           robot_ip: Robot IP address or None for simulation
       """
       robot = RobotInterface(robot_ip)
       controller = FrankaController(robot)
       
       await controller.start()
       
       try:
           print(f"Testing in {'SIMULATION' if robot_ip is None else 'REAL'} mode")
           
           # Your control algorithm
           await controller.move()
           
           controller.switch("impedance")
           controller.kp = np.ones(7) * 80.0
           controller.kd = np.ones(7) * 4.0
           controller.set_freq(50)
           
           for i in range(100):
               delta = np.sin(i / 50.0 * np.pi) * 0.1
               target = controller.initial_qpos + delta
               await controller.set("q_desired", target)
               
           print("Test successful!")
           
       finally:
           await controller.stop()

   if __name__ == "__main__":
       # First test in simulation
       print("=" * 50)
       print("Testing in simulation...")
       print("=" * 50)
       asyncio.run(test_algorithm(None))
       
       # Then deploy to real robot
       print("\n" + "=" * 50)
       print("Testing on real robot...")
       print("=" * 50)
       asyncio.run(test_algorithm("172.16.0.2"))

More Examples
-------------

For more examples, check the ``examples/`` directory in the repository:

- ``01_collect_ref_traj.py``: System identification data collection
- ``02_spacemouse_teleop.py``: SpaceMouse teleoperation
- ``03_collect.py``: Vision-based data collection with markers

Next Steps
----------

- Review :doc:`controllers` for detailed controller documentation
- Check :doc:`safety` before running on real hardware
- Explore the :doc:`api/controller` for advanced features
