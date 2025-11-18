Controllers
===========

aiofranka supports three control modes, each suited for different applications.

Overview
--------

.. list-table:: Controller Comparison
   :header-rows: 1
   :widths: 20 20 20 20 20

   * - Mode
     - Control Space
     - Input Type
     - Complexity
     - Use Case
   * - Impedance
     - Joint
     - Positions
     - Low
     - Joint trajectories
   * - OSC
     - Task
     - EE Pose
     - Medium
     - Cartesian motion
   * - Torque
     - Joint
     - Torques
     - High
     - Custom control

Impedance Control
-----------------

Joint-space impedance control implements a spring-damper system in joint space:

.. math::

   \\tau = K_p (q_{des} - q) - K_d \\dot{q}

where:

- :math:`\\tau`: Joint torques [Nm]
- :math:`K_p`: Position stiffness gains [Nm/rad]
- :math:`K_d`: Damping gains [Nm⋅s/rad]
- :math:`q_{des}`: Desired joint positions [rad]
- :math:`q`: Current joint positions [rad]
- :math:`\\dot{q}`: Joint velocities [rad/s]

Usage
~~~~~

.. code-block:: python

   controller.switch("impedance")
   controller.kp = np.ones(7) * 80.0  # Stiffness
   controller.kd = np.ones(7) * 4.0   # Damping
   controller.set_freq(50)

   for i in range(200):
       target = compute_target(i)
       await controller.set("q_desired", target)

Gain Tuning
~~~~~~~~~~~

**Starting Point:**

- ``kp = 20-40`` (low stiffness, safe)
- ``kd = 2-4`` (light damping)

**Tuning Guidelines:**

1. Start with low gains
2. Increase ``kp`` for stiffer response (max ~200)
3. If oscillations occur, increase ``kd``
4. Rule of thumb: :math:`K_d \\approx \\sqrt{K_p}/2`

**Per-Joint Tuning:**

.. code-block:: python

   # Joints 1-4 can handle higher gains
   kp = np.array([100, 100, 100, 100, 60, 60, 60])
   kd = np.array([5, 5, 5, 5, 3, 3, 3])

Advantages
~~~~~~~~~~

- ✅ Simple to understand and tune
- ✅ Robust and stable
- ✅ Natural compliance
- ✅ Fast execution

Limitations
~~~~~~~~~~~

- ❌ No Cartesian straight-line motion
- ❌ Configuration-dependent behavior
- ❌ No direct force control

Best For
~~~~~~~~

- Joint-space trajectory following
- Learning from demonstration (joint space)
- Compliant interaction tasks
- General manipulation

Operational Space Control (OSC)
--------------------------------

OSC controls the end-effector in Cartesian space while managing null-space behavior:

.. math::

   \\tau = J^T \\Lambda (K_p e - K_d \\dot{x}) + N^T (K_{null} (q_0 - q) - K_{d,null} \\dot{q})

where:

- :math:`J`: End-effector Jacobian
- :math:`\\Lambda`: Operational space inertia matrix
- :math:`e`: End-effector pose error
- :math:`\\dot{x}`: End-effector velocity
- :math:`N`: Null-space projection
- :math:`q_0`: Null-space reference configuration

Usage
~~~~~

.. code-block:: python

   controller.switch("osc")
   
   # Task-space gains [x, y, z, roll, pitch, yaw]
   controller.ee_kp = np.array([300, 300, 300, 1000, 1000, 1000])
   controller.ee_kd = np.ones(6) * 10.0
   
   # Null-space gains (keeps robot away from limits)
   controller.null_kp = np.ones(7) * 10.0
   controller.null_kd = np.ones(7) * 1.0
   
   controller.set_freq(50)
   
   # Create desired pose (4x4 homogeneous transform)
   desired_ee = np.eye(4)
   desired_ee[:3, :3] = rotation_matrix  # 3x3 rotation
   desired_ee[:3, 3] = [x, y, z]         # position
   
   await controller.set("ee_desired", desired_ee)

End-Effector Pose Format
~~~~~~~~~~~~~~~~~~~~~~~~~

The end-effector pose is a 4×4 homogeneous transformation matrix:

.. code-block:: python

   ee = [[R | p],
         [0 | 1]]
   
   # Where:
   # R: 3x3 rotation matrix (SO(3))
   # p: 3x1 position vector [x, y, z] in meters

Example with scipy:

.. code-block:: python

   from scipy.spatial.transform import Rotation as R

   ee = np.eye(4)
   # Set rotation (e.g., 180° around X)
   ee[:3, :3] = R.from_euler('xyz', [180, 0, 0], degrees=True).as_matrix()
   # Set position
   ee[:3, 3] = [0.5, 0.0, 0.4]  # meters

Gain Tuning
~~~~~~~~~~~

**Task-Space Gains:**

- Translation: ``ee_kp[:3] = 100-500`` N/m
- Rotation: ``ee_kp[3:] = 500-2000`` Nm/rad
- Generally: rotation stiffness > translation stiffness

**Null-Space Gains:**

- ``null_kp = 5-20`` Nm/rad (keeps configuration stable)
- ``null_kd = 1-5`` Nm⋅s/rad

.. code-block:: python

   # Conservative starting point
   controller.ee_kp = np.array([150, 150, 150, 600, 600, 600])
   controller.ee_kd = np.ones(6) * 8.0
   controller.null_kp = np.ones(7) * 10.0
   controller.null_kd = np.ones(7) * 2.0

Advantages
~~~~~~~~~~

- ✅ Cartesian straight-line motion
- ✅ Intuitive end-effector control
- ✅ Null-space optimization (avoid joint limits)
- ✅ Task-space impedance

Limitations
~~~~~~~~~~~

- ❌ More complex than impedance
- ❌ Can fail near singularities
- ❌ Higher computational cost

Best For
~~~~~~~~

- Cartesian straight-line trajectories
- End-effector tracking (visual servoing, teleoperation)
- Task-space impedance control
- Contact tasks requiring Cartesian compliance

Singularities
~~~~~~~~~~~~~

OSC can become unstable near singularities (where Jacobian is ill-conditioned):

.. code-block:: python

   # Check Jacobian conditioning
   state = controller.state
   jac = state['jac']
   
   # Compute condition number
   cond = np.linalg.cond(jac)
   
   if cond > 100:
       print("Warning: Near singularity!")
       # Consider switching to impedance mode

Direct Torque Control
---------------------

Torque mode gives you full control authority - you compute torques directly:

.. math::

   \\tau = f(q, \\dot{q}, t)

where :math:`f` is your custom control law.

Usage
~~~~~

.. code-block:: python

   controller.switch("torque")
   
   for i in range(500):
       # Get state
       state = controller.state
       q = state['qpos']
       dq = state['qvel']
       mm = state['mm']  # Mass matrix
       
       # Compute your control law
       tau = compute_control(q, dq, mm)
       
       # Set torques (thread-safe)
       with controller.state_lock:
           controller.torque = tau
       
       await asyncio.sleep(1.0 / 500.0)

Safety Features
~~~~~~~~~~~~~~~

Even in torque mode, some safety features remain active:

**Torque Rate Limiting:**

.. code-block:: python

   controller.torque_diff_limit = 990.0  # Nm/s (default)
   controller.clip = True  # Enable rate limiting

To disable (not recommended):

.. code-block:: python

   controller.clip = False

**Joint Torque Limits:**

Hardware limits (enforced by robot):

- Joints 1-4: ±87 Nm
- Joints 5-7: ±12 Nm

Advantages
~~~~~~~~~~

- ✅ Full control authority
- ✅ Can implement any control law
- ✅ Optimal for learning-based control
- ✅ Direct force control

Limitations
~~~~~~~~~~~

- ❌ Requires careful implementation
- ❌ No built-in stability
- ❌ Easy to trigger safety stops
- ❌ Requires deep understanding

Best For
~~~~~~~~

- Custom control algorithms
- Learning-based control (RL, IL)
- Advanced research applications
- Implementing novel controllers

.. danger::
   **Torque mode is dangerous!**
   
   - Always test in simulation first
   - Start with zero torques and gradually increase
   - Monitor torques continuously
   - Keep E-stop within reach
   - Use conservative limits

Switching Controllers
---------------------

You can switch between controllers at runtime:

.. code-block:: python

   # Start with impedance
   controller.switch("impedance")
   controller.kp = np.ones(7) * 80.0
   await controller.set("q_desired", target1)

   # Switch to OSC
   controller.switch("osc")
   controller.ee_kp = np.array([300, 300, 300, 1000, 1000, 1000])
   await controller.set("ee_desired", target2)

   # Switch to torque
   controller.switch("torque")
   controller.torque = np.zeros(7)

**Note:** Switching resets initial states (``initial_qpos``, ``initial_ee``) and clears timing state.

Performance Comparison
----------------------

.. list-table:: Typical Performance
   :header-rows: 1
   :widths: 20 20 20 20 20

   * - Mode
     - Update Rate
     - CPU Usage
     - Latency
     - Precision
   * - Impedance
     - 10-100 Hz
     - Low
     - ~1 ms
     - High
   * - OSC
     - 10-200 Hz
     - Medium
     - ~2 ms
     - High
   * - Torque
     - Up to 1 kHz
     - High
     - ~0.5 ms
     - Very High

The control loop always runs at 1 kHz internally; update rate refers to how often you send commands via ``set()``.

Troubleshooting
---------------

Robot Oscillates
~~~~~~~~~~~~~~~~

**Cause:** Gains too high or damping too low

**Solution:**

.. code-block:: python

   # Decrease stiffness
   controller.kp /= 2
   
   # Increase damping
   controller.kd *= 1.5

Robot Too Compliant
~~~~~~~~~~~~~~~~~~~

**Cause:** Gains too low

**Solution:**

.. code-block:: python

   # Increase stiffness gradually
   controller.kp *= 1.5

Motion is Jerky
~~~~~~~~~~~~~~~

**Cause:** Commands not rate-limited or gains need tuning

**Solution:**

.. code-block:: python

   # Ensure rate limiting
   controller.set_freq(50)
   
   # Increase damping
   controller.kd = np.ones(7) * 6.0

Safety Stop Triggered
~~~~~~~~~~~~~~~~~~~~~

**Cause:** Torque rate too high or torques exceed limits

**Solution:**

.. code-block:: python

   # Lower gains
   controller.kp = np.ones(7) * 40.0
   controller.kd = np.ones(7) * 3.0
   
   # Ensure smooth commands
   controller.set_freq(50)
   
   # Check for discontinuities in target

OSC Unstable
~~~~~~~~~~~~

**Cause:** Near singularity or gains too high

**Solution:**

.. code-block:: python

   # Check Jacobian conditioning
   cond = np.linalg.cond(controller.state['jac'])
   if cond > 100:
       # Switch to impedance mode
       controller.switch("impedance")
   
   # Or decrease OSC gains
   controller.ee_kp /= 2

Next Steps
----------

- Try the :doc:`examples` for each controller
- Review :doc:`safety` guidelines
- Check :doc:`api/controller` for detailed API
