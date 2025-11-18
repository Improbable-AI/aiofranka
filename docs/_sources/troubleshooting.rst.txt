Troubleshooting
===============

This page covers common issues and their solutions.

Connection Issues
-----------------

Error: "Error requesting control token"
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Symptoms:**

.. code-block:: text

   Error requesting control token.
   AssertionError

**Causes:**

- Another user or program has control of the robot
- Previous session didn't release token properly
- Robot is locked via Desk interface

**Solutions:**

**Option 1: Wait for release**

.. code-block:: python

   from aiofranka.client import FrankaLockUnlock

   client = FrankaLockUnlock("172.16.0.2", "admin", "admin")
   client.run(unlock=True, wait=True, fci=True, persistent=True)

This will wait until the token becomes available.

**Option 2: Check Desk interface**

1. Open web browser to ``https://172.16.0.2``
2. Login with admin credentials
3. Check "System" → "Control Token"
4. Release active token if yours

**Option 3: Force take control** (use with caution!)

.. code-block:: python

   client.run(unlock=True, force=True, fci=True, persistent=True)

.. warning::
   Only use force if you're sure no other program is controlling the robot!

Cannot Connect to Robot
~~~~~~~~~~~~~~~~~~~~~~~

**Symptoms:**

.. code-block:: python

   ConnectionError: Unable to connect to robot

**Diagnosis:**

.. code-block:: bash

   # Test network connection
   ping 172.16.0.2
   
   # Should show < 1ms latency

**Solutions:**

1. **Check physical connection**
   - Ethernet cable connected?
   - Cable not damaged?
   - LED on robot's network port lit?

2. **Check IP configuration**
   - Correct IP address?
   - Subnet mask correct (typically 255.255.255.0)?
   - No IP conflicts?

3. **Check firewall**
   
   .. code-block:: bash
   
      # Ubuntu: Allow traffic
      sudo ufw allow from 172.16.0.0/24

4. **Check robot state**
   - Robot powered on?
   - No error lights?
   - Robot in FCI mode?

Robot Behavior Issues
---------------------

Robot Triggers Safety Stop
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Symptoms:**

Robot suddenly stops moving, yellow lights flash.

**Common Causes:**

1. **Torque Rate Too High**

   **Solution**: Lower gains or ensure smooth commands
   
   .. code-block:: python
   
      # Decrease stiffness
      controller.kp /= 2
      
      # Increase damping
      controller.kd *= 1.5
      
      # Ensure rate limiting
      controller.set_freq(50)

2. **Discontinuous Commands**

   **Solution**: Use smooth trajectories
   
   .. code-block:: python
   
      # ❌ Bad
      await controller.set("q_desired", far_target)
      
      # ✅ Good
      await controller.move(far_target)

3. **Collision Detected**

   **Solution**: Check workspace for obstacles
   
   - Clear workspace
   - Adjust collision thresholds
   - Test motion path in simulation

4. **Joint Limits**

   **Solution**: Verify target within limits
   
   .. code-block:: python
   
      # Check before sending
      if np.all(target > -2.8) and np.all(target < 2.8):
           await controller.set("q_desired", target)

**Recovery:**

1. Release brakes if locked
2. Check error in Desk interface
3. Review and fix code
4. Test with lower gains
5. Restart carefully

Jerky or Oscillating Motion
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Symptoms:**

Robot moves in jerky manner or oscillates around target.

**Causes & Solutions:**

**1. No Rate Limiting**

.. code-block:: python

   # ❌ Problem: Commands sent too fast
   for i in range(100):
       await controller.set("q_desired", target)
   
   # ✅ Solution: Add rate limiting
   controller.set_freq(50)
   for i in range(100):
       await controller.set("q_desired", target)

**2. Low Damping**

.. code-block:: python

   # ✅ Increase damping
   controller.kd = np.ones(7) * 6.0

**3. High Gains**

.. code-block:: python

   # ✅ Reduce stiffness
   controller.kp = np.ones(7) * 40.0

**4. Discontinuous Targets**

.. code-block:: python

   # ✅ Smooth target changes
   delta = np.sin(cnt / 50.0 * np.pi) * 0.1  # Smooth sinusoid

Robot Too Compliant
~~~~~~~~~~~~~~~~~~~

**Symptoms:**

Robot doesn't track targets well, feels "soft."

**Solution:**

.. code-block:: python

   # Gradually increase stiffness
   controller.kp *= 1.5
   
   # Test and repeat if still too compliant
   # But don't exceed kp ~ 200

Robot Doesn't Move
~~~~~~~~~~~~~~~~~~

**Symptoms:**

Robot stays still even when sending commands.

**Diagnosis:**

.. code-block:: python

   # Check control loop
   print(f"Running: {controller.running}")
   print(f"Type: {controller.type}")
   print(f"Current position: {controller.state['qpos']}")
   print(f"Desired position: {controller.q_desired}")

**Common Issues:**

1. **Forgot to start**

   .. code-block:: python
   
      # ❌ Missing
      # await controller.start()
      
      # ✅ Fixed
      await controller.start()

2. **Wrong controller type**

   .. code-block:: python
   
      # Sending q_desired but in OSC mode
      controller.switch("impedance")  # Fix

3. **Target equals current**

   .. code-block:: python
   
      # Check if target differs from current
      diff = np.linalg.norm(target - controller.state['qpos'])
      print(f"Distance to target: {diff}")

4. **Gains too low**

   .. code-block:: python
   
      # Increase stiffness
      controller.kp = np.ones(7) * 80.0

Control Loop Issues
-------------------

Low Control Frequency
~~~~~~~~~~~~~~~~~~~~~

**Symptoms:**

``test_connection()`` shows frequency < 990 Hz or high jitter.

.. code-block:: text

   Frequency: 850 Hz (target: 1000 Hz)  # ❌ Too low
   Jitter (max-min): 5.2 ms              # ❌ Too high

**Causes:**

1. **High CPU load**
   - Close unnecessary programs
   - Use dedicated control computer
   - Check ``top`` or ``htop``

2. **Network latency**
   
   .. code-block:: bash
   
      # Check ping time
      ping 172.16.0.2
      # Should be < 1ms

3. **Heavy computation in main loop**

   .. code-block:: python
   
      # ❌ Bad: Heavy computation in loop
      controller.set_freq(50)
      for i in range(100):
          target = expensive_computation()  # Slow!
          await controller.set("q_desired", target)
      
      # ✅ Good: Precompute
      targets = [expensive_computation(i) for i in range(100)]
      controller.set_freq(50)
      for target in targets:
          await controller.set("q_desired", target)

4. **Not using asyncio properly**

   .. code-block:: python
   
      # ❌ Bad: Blocking operation
      time.sleep(1.0)
      
      # ✅ Good: Async sleep
      await asyncio.sleep(1.0)

**Solutions:**

- Use wired connection
- Reduce system load
- Optimize code
- Consider real-time Linux kernel

Control Loop Crashes
~~~~~~~~~~~~~~~~~~~~~

**Symptoms:**

Robot stops unexpectedly, script exits.

**Check terminal for:**

.. code-block:: text

   Error in control loop: ...
   High torque rate of change detected on axes: [2]

**Common Errors:**

1. **High torque rate**
   - Lower gains
   - Ensure smooth commands
   - Check ``torque_diff_limit``

2. **NaN or Inf values**
   
   .. code-block:: python
   
      # Check before sending
      if np.any(np.isnan(target)) or np.any(np.isinf(target)):
          print("Invalid target!")
          await controller.stop()

3. **Wrong array shapes**
   
   .. code-block:: python
   
      # Ensure correct shape
      target = np.array([...])  # Must be (7,) for joints
      assert target.shape == (7,), f"Wrong shape: {target.shape}"

OSC Issues
----------

OSC Unstable or Oscillates
~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Symptoms:**

Robot oscillates violently in OSC mode.

**Causes:**

1. **Near singularity**

   .. code-block:: python
   
      # Check Jacobian condition number
      jac = controller.state['jac']
      cond = np.linalg.cond(jac)
      
      if cond > 100:
          print(f"Warning: Poor conditioning: {cond}")
          # Switch to impedance
          controller.switch("impedance")

2. **Gains too high**

   .. code-block:: python
   
      # Reduce OSC gains
      controller.ee_kp = np.array([200, 200, 200, 600, 600, 600])
      controller.ee_kd = np.ones(6) * 8.0

3. **Null-space conflict**

   .. code-block:: python
   
      # Lower null-space gains
      controller.null_kp = np.ones(7) * 5.0
      controller.null_kd = np.ones(7) * 1.0

OSC Doesn't Reach Target
~~~~~~~~~~~~~~~~~~~~~~~~~

**Symptoms:**

Robot stops before reaching desired pose.

**Diagnosis:**

.. code-block:: python

   # Check error
   state = controller.state
   ee_current = state['ee']
   ee_desired = controller.ee_desired
   
   pos_error = np.linalg.norm(ee_desired[:3, 3] - ee_current[:3, 3])
   print(f"Position error: {pos_error * 1000:.1f} mm")

**Causes:**

1. **Target unreachable** (outside workspace)
   - Verify target is reachable
   - Check joint limits allow configuration

2. **Null-space pulling away**
   - Adjust ``null_kp`` or ``null_kd``
   - Change null-space reference (``initial_qpos``)

3. **Singularity prevents motion**
   - Move via intermediate poses
   - Use impedance control instead

Programming Issues
------------------

ImportError: No module named 'pylibfranka'
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Solution:**

.. code-block:: bash

   # Install pylibfranka
   pip install pylibfranka

See :doc:`installation` for detailed instructions.

ModuleNotFoundError: No module named 'mujoco'
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Solution:**

.. code-block:: bash

   pip install mujoco

TypeError: 'coroutine' object is not iterable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Cause:**

Forgot to ``await`` an async function.

.. code-block:: python

   # ❌ Wrong
   controller.start()  # Missing await
   
   # ✅ Correct
   await controller.start()

RuntimeError: This event loop is already running
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Cause:**

Using ``asyncio.run()`` inside async function.

.. code-block:: python

   # ❌ Wrong
   async def my_function():
       asyncio.run(controller.start())  # Don't do this
   
   # ✅ Correct
   async def my_function():
       await controller.start()

Simulation vs Real Discrepancies
---------------------------------

Different Behavior
~~~~~~~~~~~~~~~~~~

**Symptoms:**

Code works in simulation but not on real robot, or vice versa.

**Causes:**

1. **Friction and compliance**
   - Real robot has friction MuJoCo doesn't model
   - Real robot has compliance in joints/links

2. **Latency**
   - Real robot has network latency (~1ms)
   - Simulation is instantaneous

3. **Model inaccuracy**
   - MuJoCo model may not match real robot exactly
   - Payload/tool differences

**Solutions:**

.. code-block:: python

   # Tune gains separately for sim and real
   if robot.real:
       controller.kp = np.ones(7) * 100.0  # Real robot
   else:
       controller.kp = np.ones(7) * 80.0   # Simulation
   
   # Test more conservatively on real robot
   # If works in sim, reduce gains 20-30% for real

Getting Help
------------

Before Asking for Help
~~~~~~~~~~~~~~~~~~~~~~

1. **Check this guide** - Is your issue covered here?
2. **Test in simulation** - Does it work in simulation?
3. **Review examples** - Are you following the patterns?
4. **Check logs** - Any error messages in terminal?
5. **Simplify** - Does a minimal example reproduce the issue?

When Asking for Help
~~~~~~~~~~~~~~~~~~~~

Include:

1. **aiofranka version**: ``pip show aiofranka``
2. **Operating system**: Linux/macOS/Windows
3. **Python version**: ``python --version``
4. **Minimal code** that reproduces the issue
5. **Full error message** from terminal
6. **What you've tried** already

Where to Ask
~~~~~~~~~~~~

- **GitHub Issues**: https://github.com/Improbable-AI/aiofranka/issues
- **Discussions**: For questions (not bugs)

Debug Workflow
--------------

Systematic Debugging
~~~~~~~~~~~~~~~~~~~~

1. **Isolate the problem**
   
   .. code-block:: python
   
      # Test each component separately
      
      # 1. Can you connect?
      robot = RobotInterface("172.16.0.2")
      print(robot.state)
      
      # 2. Can you start control?
      controller = FrankaController(robot)
      await controller.start()
      await asyncio.sleep(1.0)
      await controller.stop()
      
      # 3. Can you move?
      await controller.start()
      await controller.move()
      await controller.stop()
      
      # 4. Can you send commands?
      await controller.start()
      controller.set_freq(10)
      for i in range(10):
          await controller.set("q_desired", controller.initial_qpos)
      await controller.stop()

2. **Add logging**

   .. code-block:: python
   
      import logging
      logging.basicConfig(level=logging.DEBUG)
      
      # Now you'll see detailed info

3. **Check state**

   .. code-block:: python
   
      # Print everything
      state = controller.state
      for key, value in state.items():
          print(f"{key}: {value}")

4. **Verify assumptions**

   .. code-block:: python
   
      # Check what you think is true
      assert controller.running, "Control loop not running!"
      assert controller.type == "impedance", "Wrong controller type!"
      print(f"Gains: kp={controller.kp}, kd={controller.kd}")

Common Error Messages
---------------------

.. code-block:: python

   # "Error in control loop: ..."
   # → Check terminal for full traceback
   # → Usually indicates problem in control computation

   # "High torque rate of change detected"
   # → Gains too high or discontinuous command
   # → Lower gains or smooth commands

   # "Error logging in"
   # → Wrong username/password
   # → Default is admin/admin

   # "Error requesting control token"
   # → Another user has control
   # → Wait or force take control

   # "Cannot connect to robot"
   # → Network issue or robot not powered
   # → Check ping and physical connection

Still Stuck?
------------

If none of the above helps:

1. Try the ``test.py`` script - does it work?
2. Compare your code to working examples
3. Test each component separately
4. Ask for help (see "Getting Help" above)

Remember: Most issues are configuration or usage errors, not bugs in aiofranka!
