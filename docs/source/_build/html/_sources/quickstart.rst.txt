Quick Start Guide
=================

This guide will help you get started with aiofranka and perform basic robot control operations.

Basic Setup
-----------

First, import the necessary modules:

.. code-block:: python

    import asyncio
    import numpy as np
    from aiofranka import RobotInterface, FrankaController

Initialize the Robot
--------------------

Create a robot interface with your robot's IP address. If no IP is provided, 
the robot will run in simulation mode using MuJoCo:

.. code-block:: python

    # For real robot
    robot = RobotInterface("172.16.0.2")
    
    # For simulation
    robot = RobotInterface()

Create a Controller
-------------------

Initialize the controller with the robot interface:

.. code-block:: python

    controller = FrankaController(robot)

Starting the Control Loop
--------------------------

Start the background control loop:

.. code-block:: python

    async def main():
        await controller.start()
        
        # Your control code here...
        
        await controller.stop()

Testing the Connection
----------------------

Test the 1kHz connection with the robot:

.. code-block:: python

    await controller.test_connection()

This will print average frequency and jitter statistics.

Moving to a Position
--------------------

Move to a specific joint configuration using trajectory generation:

.. code-block:: python

    target_position = [0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853]
    await controller.move(target_position)

This uses Ruckig for smooth offline trajectory generation.

Impedance Control
-----------------

Switch to impedance control mode and set gains:

.. code-block:: python

    controller.switch("impedance")
    controller.kp = np.ones(7) * 80.0  # Position gains
    controller.kd = np.ones(7) * 4.0   # Velocity gains
    controller.set_freq(50)  # 50Hz update rate

Update the desired position at the specified frequency:

.. code-block:: python

    for cnt in range(100):
        delta = np.sin(cnt / 50.0 * np.pi) * 0.1
        init = controller.initial_qpos
        await controller.set("q_desired", delta + init)

Operational Space Control (OSC)
--------------------------------

Switch to OSC mode for end-effector control:

.. code-block:: python

    controller.switch("osc")
    controller.set_freq(50)

Control the end-effector pose:

.. code-block:: python

    for cnt in range(100):
        delta = np.sin(cnt / 50.0 * np.pi) * 0.1
        init = controller.initial_ee
        
        desired_ee = np.eye(4)
        desired_ee[:3, :3] = init[:3, :3]
        desired_ee[:3, 3] = init[:3, 3] + np.array([0, delta, 0])
        
        await controller.set("ee_desired", desired_ee)

Complete Example
----------------

Here's a complete example that demonstrates various control modes:

.. code-block:: python

    import asyncio
    import numpy as np
    from aiofranka import RobotInterface, FrankaController

    async def main():
        robot = RobotInterface("172.16.0.2")
        controller = FrankaController(robot)
        
        await controller.start()
        
        # Test connection
        await controller.test_connection()
        
        # Move to home position
        await controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])
        
        # Impedance control example
        controller.switch("impedance")
        controller.kp = np.ones(7) * 80.0
        controller.kd = np.ones(7) * 4.0
        controller.set_freq(50)
        
        for cnt in range(100):
            delta = np.sin(cnt / 50.0 * np.pi) * 0.1
            await controller.set("q_desired", controller.initial_qpos + delta)
        
        # OSC control example
        controller.switch("osc")
        controller.set_freq(50)
        
        for cnt in range(100):
            delta = np.sin(cnt / 50.0 * np.pi) * 0.1
            init = controller.initial_ee
            
            desired_ee = np.eye(4)
            desired_ee[:3, :3] = init[:3, :3]
            desired_ee[:3, 3] = init[:3, 3] + np.array([0, delta, 0])
            
            await controller.set("ee_desired", desired_ee)
        
        await controller.stop()

    if __name__ == "__main__":
        asyncio.run(main())

Next Steps
----------

* Check the :doc:`api` for detailed API documentation
* Explore the :doc:`examples` for advanced use cases
* Learn about different control modes and their parameters
