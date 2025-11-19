Quick Start
===========

This guide will get you controlling a Franka robot in minutes.

Basic Example
-------------

Here's a minimal example that demonstrates the core functionality:

.. code-block:: python

   import asyncio 
   import numpy as np 
   from aiofranka import RobotInterface, FrankaController

   async def main():
       # Connect to robot (use IP for real robot, None for simulation)
       robot = RobotInterface("172.16.0.2") 
       controller = FrankaController(robot)
       
       # Start the 1kHz control loop
       await controller.start()

       try:
           # Test connection quality
           await controller.test_connection()

           # Move to home position with smooth trajectory
           await controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])

           # Switch to impedance control
           controller.switch("impedance")
           controller.kp = np.ones(7) * 80.0
           controller.kd = np.ones(7) * 4.0
           controller.set_freq(50)  # 50Hz update rate
           
           # Execute sinusoidal motion
           for cnt in range(100): 
               delta = np.sin(cnt / 50.0 * np.pi) * 0.1
               init = controller.initial_qpos
               await controller.set("q_desired", delta + init)

       finally:
           # Always stop gracefully
           await controller.stop()

   if __name__ == "__main__":
       asyncio.run(main())

Understanding the Code
----------------------

Let's break down what each part does:

1. **Create Robot Interface**

   .. code-block:: python

      robot = RobotInterface("172.16.0.2")  # Real robot
      # robot = RobotInterface(None)        # Simulation

   The IP address connects to your real robot. Use ``None`` for simulation mode.

2. **Create Controller**

   .. code-block:: python

      controller = FrankaController(robot)

   The controller manages the 1kHz control loop and provides high-level commands.

3. **Start Control Loop**

   .. code-block:: python

      await controller.start()

   This starts the background control loop at 1kHz. Must be awaited!

4. **Test Connection**

   .. code-block:: python

      await controller.test_connection()

   Runs for 5 seconds and prints timing statistics. Useful for diagnosing issues.

5. **Move with Trajectory**

   .. code-block:: python

      await controller.move([0, 0, 0, -1.57, 0, 1.57, -0.785])

   Generates and executes a smooth, jerk-limited trajectory to the target position.

6. **Switch Controller Mode**

   .. code-block:: python

      controller.switch("impedance")

   Switches between impedance, OSC, and torque control modes at runtime.

7. **Set Gains**

   .. code-block:: python

      controller.kp = np.ones(7) * 80.0  # Stiffness
      controller.kd = np.ones(7) * 4.0   # Damping

   Configure controller gains for desired behavior.

8. **Set Update Frequency**

   .. code-block:: python

      controller.set_freq(50)  # 50 Hz

   Enforces timing for subsequent ``set()`` calls.

9. **Send Commands**

   .. code-block:: python

      await controller.set("q_desired", target)

   Rate-limited setter that automatically maintains the specified frequency.

10. **Stop Controller**

    .. code-block:: python

       await controller.stop()

    Gracefully stops the control loop and robot. Always use in a ``finally`` block!

Simulation Mode
---------------

Test your code without hardware:

.. code-block:: python

   import asyncio
   from aiofranka import RobotInterface, FrankaController

   async def test_in_simulation():
       # None = simulation mode
       robot = RobotInterface(None)
       controller = FrankaController(robot)
       
       await controller.start()
       
       # Same code works in simulation!
       await controller.move()
       
       await controller.stop()

   asyncio.run(test_in_simulation())

The MuJoCo viewer will open automatically, showing the robot motion.

Reading Robot State
-------------------

Access current robot state at any time:

.. code-block:: python

   state = controller.state

   print(f"Joint positions: {state['qpos']}")      # [rad]
   print(f"Joint velocities: {state['qvel']}")     # [rad/s]
   print(f"End-effector pose:\n{state['ee']}")     # 4x4 transform
   print(f"Jacobian:\n{state['jac']}")             # (6, 7)
   print(f"Mass matrix:\n{state['mm']}")           # (7, 7)
   print(f"Last torques: {state['last_torque']}")  # [Nm]

Controller Modes
----------------

aiofranka supports three control modes:

Impedance Control (Joint Space)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   controller.switch("impedance")
   controller.kp = np.ones(7) * 80.0
   controller.kd = np.ones(7) * 4.0
   controller.set_freq(50)

   for i in range(100):
       target = compute_target(i)
       await controller.set("q_desired", target)

**Best for**: Joint-space trajectories, compliant behavior

Operational Space Control (Task Space)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   controller.switch("osc")
   controller.ee_kp = np.array([300, 300, 300, 1000, 1000, 1000])
   controller.ee_kd = np.ones(6) * 10.0
   controller.set_freq(50)

   desired_ee = np.eye(4)
   desired_ee[:3, 3] = [0.5, 0.0, 0.4]  # Position [x, y, z]
   await controller.set("ee_desired", desired_ee)

**Best for**: Cartesian motions, end-effector tracking

