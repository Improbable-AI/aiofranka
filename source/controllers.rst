Controllers
===========

aiofranka supports three control modes, each suited for different applications.

.. contents:: Table of Contents
   :local:
   :depth: 2
   




Impedance Control
-----------------

Joint-space impedance control implements a spring-damper system in joint space:

.. math::

   \tau = K_{\text{p}} (\mathbf{q}_{\text{desired}} - \mathbf{q}) - K_{\text{d}} \dot{\mathbf{q}}

where:

- :math:`\tau`: Joint torques [Nm]  (:py:attr:`~aiofranka.controller.FrankaController.torque`)
- :math:`K_{\text{p}}`: Position stiffness gains [Nm/rad]  (:py:attr:`~aiofranka.controller.FrankaController.kp`)
- :math:`K_{\text{d}}`: Damping gains [Nm⋅s/rad]  (:py:attr:`~aiofranka.controller.FrankaController.kd`)
- :math:`\mathbf{q}_{\text{desired}}`: Desired joint positions [rad]  (:py:attr:`~aiofranka.controller.FrankaController.q_desired`)
- :math:`\mathbf{q}`: Current joint positions [rad]  (:py:attr:`~aiofranka.controller.FrankaController.q`)
- :math:`\dot{\mathbf{q}}`: Joint velocities [rad/s]  (:py:attr:`~aiofranka.controller.FrankaController.qd`)

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



Operational Space Control (OSC)
--------------------------------

OSC controls the end-effector in Cartesian space while managing null-space behavior:

.. math::

   \tau = \mathbf{J}^T \mathbf{M}_{\mathbf{x}} (K_{\text{p}}^{\text{ee}} \mathbf{e} - K_{\text{d}}^{\text{ee}} \dot{\mathbf{x}}) + (\mathbf{I} - \mathbf{J}^T \bar{\mathbf{J}}^T) (K_{\text{p}}^{\text{null}} (\mathbf{q}_0 - \mathbf{q}) - K_{\text{d}}^{\text{null}} \dot{\mathbf{q}})

where:

- :math:`\mathbf{J}`: End-effector Jacobian (:py:attr:`~aiofranka.controller.FrankaController.state`\[`jac`])
- :math:`\mathbf{M}_{\mathbf{x}}x = (\mathbf{J} \mathbf{M}^{-1} \mathbf{J}^T)^{-1}`: Operational space inertia matrix
- :math:`\mathbf{M}`: Joint-space mass matrix (:py:attr:`~aiofranka.controller.FrankaController.state`\['mm'])
- :math:`\mathbf{T}_{\text{goal}} = \begin{bmatrix} \mathbf{R}_{\text{goal}} & \mathbf{p}_{\text{goal}} \\ 0 & 1 \end{bmatrix}`: Desired end-effector pose (:py:attr:`~aiofranka.controller.FrankaController.ee_desired`)
- :math:`\mathbf{e} = \begin{bmatrix} \mathbf{p}_{\text{goal}} - \mathbf{p} \\ \text{Log}(\mathbf{R}_{\text{goal}} \mathbf{R}^{-1}) \end{bmatrix}`: End-effector pose error (position + rotation as axis-angle)
- :math:`\dot{\mathbf{x}} = \mathbf{J} \dot{\mathbf{q}}`: End-effector velocity
- :math:`\bar{\mathbf{J}} = \mathbf{M}^{-1} \mathbf{J}^T \mathbf{M}_{\mathbf{x}}`: Dynamically consistent pseudoinverse
- :math:`(\mathbf{I} - \mathbf{J}^T \bar{\mathbf{J}}^T)`: Null-space projection matrix
- :math:`\mathbf{q}_0`: Null-space reference configuration (initial joint positions)
- :math:`\mathbf{q}`: Current joint positions (:py:attr:`~aiofranka.controller.FrankaController.state`\['qpos'])
- :math:`\dot{\mathbf{q}}`: Joint velocities (:py:attr:`~aiofranka.controller.FrankaController.state`\['qvel'])
- :math:`K_{\text{p}}^{\text{ee}}`: Task-space stiffness gains [N/m, Nm/rad]  (:py:attr:`~aiofranka.controller.FrankaController.ee_kp`)
- :math:`K_{\text{d}}^{\text{ee}}`: Task-space damping gains [N⋅s/m, Nm⋅s/rad]  (:py:attr:`~aiofranka.controller.FrankaController.ee_kd`)
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

The end-effector pose is a 4x4 homogeneous transformation matrix:

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


