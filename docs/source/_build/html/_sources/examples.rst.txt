Examples
========

This page provides detailed examples of using aiofranka for various robot control tasks.

Basic Control Loop
------------------

A minimal example showing the basic control loop:

.. code-block:: python

    import asyncio
    import numpy as np
    from aiofranka import RobotInterface, FrankaController

    async def main():
        robot = RobotInterface("172.16.0.2")
        controller = FrankaController(robot)
        
        await controller.start()
        await controller.test_connection()
        await controller.stop()

    if __name__ == "__main__":
        asyncio.run(main())

Position Control
----------------

Moving the robot to specific joint positions:

.. code-block:: python

    import asyncio
    from aiofranka import RobotInterface, FrankaController

    async def main():
        robot = RobotInterface("172.16.0.2")
        controller = FrankaController(robot)
        
        await controller.start()
        
        # Home position
        home = [0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853]
        await controller.move(home)
        
        # Another position
        target = [0.5, -0.3, 0.2, -2.0, 0.1, 1.5, -0.5]
        await controller.move(target)
        
        await controller.stop()

    if __name__ == "__main__":
        asyncio.run(main())

Impedance Control with Sinusoidal Motion
-----------------------------------------

Using impedance control to create smooth sinusoidal motion:

.. code-block:: python

    import asyncio
    import numpy as np
    from aiofranka import RobotInterface, FrankaController

    async def main():
        robot = RobotInterface("172.16.0.2")
        controller = FrankaController(robot)
        
        await controller.start()
        
        # Move to home position first
        await controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])
        
        # Switch to impedance control
        controller.switch("impedance")
        controller.kp = np.ones(7) * 80.0
        controller.kd = np.ones(7) * 4.0
        controller.set_freq(50)  # 50Hz update rate
        
        # Sinusoidal motion in joint 0
        duration = 5.0  # seconds
        frequency = 0.5  # Hz
        amplitude = 0.2  # radians
        
        for cnt in range(int(duration * 50)):
            t = cnt / 50.0
            delta = np.zeros(7)
            delta[0] = amplitude * np.sin(2 * np.pi * frequency * t)
            
            await controller.set("q_desired", controller.initial_qpos + delta)
        
        await controller.stop()

    if __name__ == "__main__":
        asyncio.run(main())

Operational Space Control
--------------------------

Controlling the end-effector position in Cartesian space:

.. code-block:: python

    import asyncio
    import numpy as np
    from aiofranka import RobotInterface, FrankaController

    async def main():
        robot = RobotInterface("172.16.0.2")
        controller = FrankaController(robot)
        
        await controller.start()
        
        # Move to home position
        await controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])
        
        # Switch to OSC
        controller.switch("osc")
        controller.ee_kp = np.ones(6) * 100.0
        controller.ee_kd = np.ones(6) * 4.0
        controller.set_freq(50)
        
        # Circular motion in the y-z plane
        radius = 0.05
        duration = 5.0
        
        for cnt in range(int(duration * 50)):
            t = cnt / 50.0
            angle = 2 * np.pi * t / duration
            
            init = controller.initial_ee
            desired_ee = np.eye(4)
            desired_ee[:3, :3] = init[:3, :3]
            desired_ee[:3, 3] = init[:3, 3] + np.array([
                0,
                radius * np.sin(angle),
                radius * (np.cos(angle) - 1)
            ])
            
            await controller.set("ee_desired", desired_ee)
        
        await controller.stop()

    if __name__ == "__main__":
        asyncio.run(main())

Custom Gain Scheduling
----------------------

Dynamically adjusting control gains:

.. code-block:: python

    import asyncio
    import numpy as np
    from aiofranka import RobotInterface, FrankaController

    async def main():
        robot = RobotInterface("172.16.0.2")
        controller = FrankaController(robot)
        
        await controller.start()
        
        controller.switch("impedance")
        controller.set_freq(50)
        
        # Start with stiff control
        controller.kp = np.ones(7) * 200.0
        controller.kd = np.ones(7) * 10.0
        
        for cnt in range(250):  # 5 seconds at 50Hz
            # Gradually reduce stiffness
            progress = cnt / 250.0
            controller.kp = np.ones(7) * (200.0 - 150.0 * progress)
            controller.kd = np.ones(7) * (10.0 - 7.0 * progress)
            
            # Small motion
            delta = 0.1 * np.sin(cnt / 50.0 * np.pi)
            await controller.set("q_desired", controller.initial_qpos + delta)
        
        await controller.stop()

    if __name__ == "__main__":
        asyncio.run(main())

Switching Between Control Modes
--------------------------------

Demonstrating seamless transitions between control modes:

.. code-block:: python

    import asyncio
    import numpy as np
    from aiofranka import RobotInterface, FrankaController

    async def main():
        robot = RobotInterface("172.16.0.2")
        controller = FrankaController(robot)
        
        await controller.start()
        
        # Start with impedance control
        controller.switch("impedance")
        controller.kp = np.ones(7) * 80.0
        controller.kd = np.ones(7) * 4.0
        controller.set_freq(50)
        
        # Joint space motion
        for cnt in range(100):
            delta = 0.1 * np.sin(cnt / 50.0 * np.pi)
            await controller.set("q_desired", controller.initial_qpos + delta)
        
        # Switch to OSC
        controller.switch("osc")
        controller.ee_kp = np.ones(6) * 100.0
        controller.ee_kd = np.ones(6) * 4.0
        controller.set_freq(50)
        
        # Cartesian space motion
        for cnt in range(100):
            delta = 0.05 * np.sin(cnt / 50.0 * np.pi)
            init = controller.initial_ee
            
            desired_ee = np.eye(4)
            desired_ee[:3, :3] = init[:3, :3]
            desired_ee[:3, 3] = init[:3, 3] + np.array([0, delta, 0])
            
            await controller.set("ee_desired", desired_ee)
        
        await controller.stop()

    if __name__ == "__main__":
        asyncio.run(main())

Simulation Mode
---------------

Running in simulation without a physical robot:

.. code-block:: python

    import asyncio
    import numpy as np
    from aiofranka import RobotInterface, FrankaController

    async def main():
        # Initialize without IP for simulation
        robot = RobotInterface()
        controller = FrankaController(robot)
        
        await controller.start()
        
        # All control commands work the same in simulation
        controller.switch("impedance")
        controller.kp = np.ones(7) * 80.0
        controller.kd = np.ones(7) * 4.0
        controller.set_freq(50)
        
        for cnt in range(100):
            delta = 0.1 * np.sin(cnt / 50.0 * np.pi)
            await controller.set("q_desired", controller.initial_qpos + delta)
        
        await controller.stop()

    if __name__ == "__main__":
        asyncio.run(main())
