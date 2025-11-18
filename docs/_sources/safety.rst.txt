Safety Guidelines
=================

**Read this entire page before controlling a real robot.**

Safety is paramount when working with robotic systems. This page outlines essential safety practices for using aiofranka.

General Safety
--------------

Before You Start
~~~~~~~~~~~~~~~~

1. **Read the manual**: Familiarize yourself with the Franka Emika robot documentation
2. **Training**: Ensure you have proper training in robot safety
3. **Workspace**: Clear the workspace of obstacles and people
4. **Emergency stop**: Know where the E-stop button is and keep it accessible
5. **Test in simulation**: Always test new code in simulation first

Physical Safety
~~~~~~~~~~~~~~~

- ⚠️ Never enter the robot workspace while it's powered
- ⚠️ Keep one hand near the E-stop at all times during testing
- ⚠️ Start with slow, small motions
- ⚠️ Never leave a running robot unattended
- ⚠️ Use collaborative mode features when appropriate

Software Safety
---------------

Always Use Try-Finally
~~~~~~~~~~~~~~~~~~~~~~~

Ensure the robot stops even if your code crashes:

.. code-block:: python

   async def safe_control():
       robot = RobotInterface("172.16.0.2")
       controller = FrankaController(robot)
       
       await controller.start()
       
       try:
           # Your control code
           await controller.move()
           
       except KeyboardInterrupt:
           print("Interrupted by user")
           
       except Exception as e:
           print(f"Error: {e}")
           
       finally:
           # ALWAYS stop
           await controller.stop()

   asyncio.run(safe_control())

Start Conservative
~~~~~~~~~~~~~~~~~~

Begin with low gains and small motions:

.. code-block:: python

   # ✅ Good: Conservative starting point
   controller.kp = np.ones(7) * 20.0
   controller.kd = np.ones(7) * 2.0
   
   # ❌ Bad: Too aggressive
   controller.kp = np.ones(7) * 200.0
   controller.kd = np.ones(7) * 1.0

Test in Simulation
~~~~~~~~~~~~~~~~~~

Always test new code in simulation first:

.. code-block:: python

   # Test your algorithm
   async def test_algorithm(use_real_robot=False):
       ip = "172.16.0.2" if use_real_robot else None
       robot = RobotInterface(ip)
       # ... rest of code

   # First run in simulation
   asyncio.run(test_algorithm(use_real_robot=False))

   # Then, if successful, try on real robot
   # asyncio.run(test_algorithm(use_real_robot=True))

Torque Limits
-------------

Understanding Limits
~~~~~~~~~~~~~~~~~~~~

The Franka FR3 has built-in torque limits:

- **Joints 1-4**: ±87 Nm
- **Joints 5-7**: ±12 Nm

Exceeding these limits triggers a safety stop.

Torque Rate Limiting
~~~~~~~~~~~~~~~~~~~~

aiofranka automatically limits torque rate of change:

.. code-block:: python

   # Default: 990 Nm/s (safe)
   controller.torque_diff_limit = 990.0
   controller.clip = True  # Enable rate limiting

**Do not disable** unless you know what you're doing:

.. code-block:: python

   # ⚠️ Dangerous! Only for advanced users
   controller.clip = False

Monitor Torques
~~~~~~~~~~~~~~~

Add safety checks in your control loop:

.. code-block:: python

   MAX_TORQUE = 80.0  # Nm (conservative)

   for i in range(1000):
       state = controller.state
       torque = state['last_torque']
       
       if np.any(np.abs(torque) > MAX_TORQUE):
           print("WARNING: High torque detected!")
           await controller.stop()
           break
       
       await controller.set("q_desired", target)

Collision Behavior
------------------

Configure Collision Thresholds
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The robot has configurable collision detection:

.. code-block:: python

   robot = RobotInterface("172.16.0.2")
   
   # Default: High thresholds (less sensitive)
   robot.robot.set_collision_behavior(
       [100.0] * 7,  # Joint contact thresholds
       [100.0] * 7,  # Joint collision thresholds
       [100.0] * 6,  # Cartesian contact thresholds
       [100.0] * 6,  # Cartesian collision thresholds
   )

For safer operation, use lower thresholds:

.. code-block:: python

   # More sensitive collision detection
   robot.robot.set_collision_behavior(
       [50.0] * 7,   # Joint contact
       [50.0] * 7,   # Joint collision
       [50.0] * 6,   # Cartesian contact
       [50.0] * 6,   # Cartesian collision
   )

.. warning::
   Lower thresholds mean more frequent safety stops but safer operation.

Gain Tuning Safety
-------------------

Safe Gain Ranges
~~~~~~~~~~~~~~~~

Recommended safe ranges for impedance control:

.. list-table::
   :header-rows: 1
   :widths: 30 35 35

   * - Parameter
     - Safe Range
     - Danger Zone
   * - ``kp``
     - 10-100 Nm/rad
     - > 200 Nm/rad
   * - ``kd``
     - 1-10 Nm⋅s/rad
     - < 0.5 or > 20
   * - ``ee_kp`` (trans)
     - 100-400 N/m
     - > 600 N/m
   * - ``ee_kp`` (rot)
     - 500-1500 Nm/rad
     - > 2500 Nm/rad

Gradual Tuning
~~~~~~~~~~~~~~

Increase gains gradually:

.. code-block:: python

   # Start low
   kp = 20.0
   
   for mult in [1.0, 1.5, 2.0, 3.0]:
       print(f"Testing kp = {kp * mult}")
       
       controller.kp = np.ones(7) * kp * mult
       controller.kd = np.ones(7) * kp * mult / 10  # Maintain damping ratio
       
       # Test motion
       await test_motion()
       
       # Check if stable
       response = input("Stable? (y/n): ")
       if response != 'y':
           break

Motion Planning
---------------

Avoid Discontinuities
~~~~~~~~~~~~~~~~~~~~~

Sudden large changes in target position can trigger safety stops:

.. code-block:: python

   # ❌ Bad: Discontinuous jump
   await controller.set("q_desired", far_away_target)

   # ✅ Good: Use smooth trajectory
   await controller.move(far_away_target)

Or interpolate manually:

.. code-block:: python

   # ✅ Good: Linear interpolation
   start = controller.state['qpos']
   target = np.array([...])
   steps = 100

   for i in range(steps):
       alpha = i / steps
       interpolated = start + alpha * (target - start)
       await controller.set("q_desired", interpolated)

Check Joint Limits
~~~~~~~~~~~~~~~~~~

Verify targets are within joint limits:

.. code-block:: python

   # FR3 joint limits (approximate)
   JOINT_LIMITS = np.array([
       [-2.8973, 2.8973],
       [-1.7628, 1.7628],
       [-2.8973, 2.8973],
       [-3.0718, -0.0698],
       [-2.8973, 2.8973],
       [-0.0175, 3.7525],
       [-2.8973, 2.8973]
   ])

   def check_limits(q):
       for i, (q_i, (q_min, q_max)) in enumerate(zip(q, JOINT_LIMITS)):
           if q_i < q_min or q_i > q_max:
               print(f"Joint {i} out of limits: {q_i}")
               return False
       return True

   # Before sending command
   if check_limits(target):
       await controller.set("q_desired", target)
   else:
       print("Target violates joint limits!")

Velocity Limits
~~~~~~~~~~~~~~~

Monitor joint velocities:

.. code-block:: python

   MAX_VEL = 2.0  # rad/s

   state = controller.state
   if np.any(np.abs(state['qvel']) > MAX_VEL):
       print("High velocity detected!")
       await controller.stop()

Emergency Procedures
--------------------

If Something Goes Wrong
~~~~~~~~~~~~~~~~~~~~~~~

1. **Press E-stop immediately** if robot behaves unexpectedly
2. **Ctrl+C** to interrupt Python script
3. **Check terminal** for error messages
4. **Review code** before restarting

Common Triggers
~~~~~~~~~~~~~~~

Safety stops can be triggered by:

- Excessive torques (> joint limits)
- High torque rate of change
- Collision detection
- Joint limit violations
- Communication loss

After a Safety Stop
~~~~~~~~~~~~~~~~~~~

1. **Assess the situation**: Was it a false alarm or real issue?
2. **Review code**: Check for discontinuities, high gains, or errors
3. **Lower gains**: Start more conservative
4. **Unlock robot**: May need to unlock brakes via Desk interface
5. **Test again**: Carefully restart with safer parameters

Network Safety
--------------

Connection Quality
~~~~~~~~~~~~~~~~~~

Poor network connection can cause issues:

.. code-block:: python

   # Test connection before experiments
   await controller.test_connection()

Expected output:

.. code-block:: text

   Control loop stats (last 1000 iterations):
     Frequency: 1000.2 Hz (target: 1000 Hz)
     Jitter (max-min): 0.040 ms

If frequency < 990 Hz or jitter > 1 ms, investigate network issues.

Wired Connection
~~~~~~~~~~~~~~~~

- ✅ Use wired Ethernet (not WiFi)
- ✅ Direct connection or managed switch
- ✅ Low-latency network (< 1ms ping)
- ❌ Avoid shared networks
- ❌ Avoid VPN or tunnels

Best Practices Checklist
-------------------------

Before Running
~~~~~~~~~~~~~~

- [ ] Workspace clear of obstacles and people
- [ ] E-stop accessible
- [ ] Code tested in simulation
- [ ] Conservative gains set
- [ ] Try-finally block in place
- [ ] Safety monitors implemented
- [ ] Network connection tested

During Execution
~~~~~~~~~~~~~~~~

- [ ] One hand near E-stop
- [ ] Monitor robot motion
- [ ] Watch terminal for errors
- [ ] Ready to interrupt (Ctrl+C)

After Execution
~~~~~~~~~~~~~~~

- [ ] Review logs for anomalies
- [ ] Check joint positions/torques
- [ ] Gradually increase complexity
- [ ] Document any issues

Code Review Checklist
~~~~~~~~~~~~~~~~~~~~~

- [ ] ``await controller.start()`` in try block
- [ ] ``await controller.stop()`` in finally block
- [ ] Gains within safe ranges
- [ ] No discontinuous commands
- [ ] Safety monitors present
- [ ] Error handling implemented
- [ ] Tested in simulation

Advanced Safety
---------------

Redundant Safety Checks
~~~~~~~~~~~~~~~~~~~~~~~~

Implement multiple layers of safety:

.. code-block:: python

   class SafeController:
       def __init__(self, controller):
           self.controller = controller
           self.max_torque = 70.0
           self.max_vel = 2.0
           self.emergency_stop = False
       
       async def safe_set(self, attr, value):
           # Check state before sending
           state = self.controller.state
           
           if np.any(np.abs(state['last_torque']) > self.max_torque):
               self.emergency_stop = True
               await self.controller.stop()
               raise RuntimeError("Torque limit exceeded!")
           
           if np.any(np.abs(state['qvel']) > self.max_vel):
               self.emergency_stop = True
               await self.controller.stop()
               raise RuntimeError("Velocity limit exceeded!")
           
           # If safe, send command
           await self.controller.set(attr, value)

Logging
~~~~~~~

Log all commands and states for debugging:

.. code-block:: python

   import logging

   logging.basicConfig(
       filename='robot_control.log',
       level=logging.INFO,
       format='%(asctime)s - %(levelname)s - %(message)s'
   )

   logger = logging.getLogger(__name__)

   # Log important events
   logger.info("Starting control loop")
   logger.info(f"Gains: kp={controller.kp}, kd={controller.kd}")
   
   # Log commands
   logger.debug(f"Setting q_desired: {target}")

Remember
--------

.. important::
   **Safety is your responsibility**
   
   - The robot can cause injury or damage
   - Always prioritize safety over performance
   - When in doubt, press E-stop
   - Start slow and conservative
   - Test thoroughly in simulation

Next Steps
----------

- Review :doc:`troubleshooting` for common issues
- Study :doc:`examples` with safety in mind
- Read the `Franka Emika safety manual <https://frankaemika.github.io/docs/>`_
