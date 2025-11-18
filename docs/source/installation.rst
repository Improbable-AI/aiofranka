Installation
============

Prerequisites
-------------

Python Version
~~~~~~~~~~~~~~

aiofranka requires Python 3.8 or later.

System Requirements
~~~~~~~~~~~~~~~~~~~

For controlling a real robot:

- Linux operating system (Ubuntu 20.04+ recommended)
- Real-time kernel (optional but recommended for best performance)
- Network connection to Franka robot (< 1ms latency)

For simulation only:

- Any OS supported by MuJoCo (Linux, macOS, Windows)

Dependencies
------------

Required Dependencies
~~~~~~~~~~~~~~~~~~~~~

The following packages are automatically installed with aiofranka:

- ``numpy``: Numerical computations
- ``scipy``: Spatial transformations
- ``requests``: HTTP client for robot API
- ``asyncio``: Asynchronous programming (Python stdlib)

Optional Dependencies
~~~~~~~~~~~~~~~~~~~~~

For real robot control:

**libfranka and pylibfranka** (Required)

Follow the official instructions to install libfranka and pylibfranka:

1. Install libfranka from source or packages:

   .. code-block:: bash

      # Ubuntu/Debian
      sudo apt install libfranka-dev

      # Or build from source
      git clone --recursive https://github.com/frankaemika/libfranka.git
      cd libfranka
      mkdir build && cd build
      cmake -DCMAKE_BUILD_TYPE=Release ..
      make -j4
      sudo make install

2. Install pylibfranka:

   .. code-block:: bash

      pip install pylibfranka

**MuJoCo** (Required)

Install MuJoCo for kinematics and dynamics:

.. code-block:: bash

   pip install mujoco

**Ruckig** (Required for trajectory generation)

Install Ruckig for smooth trajectory planning:

.. code-block:: bash

   pip install ruckig

For teleoperation examples:

.. code-block:: bash

   pip install pyspacemouse

Installing aiofranka
--------------------

From PyPI (Recommended)
~~~~~~~~~~~~~~~~~~~~~~~

Install the latest stable release:

.. code-block:: bash

   pip install aiofranka

From Source
~~~~~~~~~~~

For development or to get the latest features:

.. code-block:: bash

   git clone https://github.com/Improbable-AI/aiofranka.git
   cd aiofranka
   pip install -e .

Development Installation
~~~~~~~~~~~~~~~~~~~~~~~~

To contribute to aiofranka:

.. code-block:: bash

   git clone https://github.com/Improbable-AI/aiofranka.git
   cd aiofranka
   pip install -e ".[dev]"

Verifying Installation
----------------------

Test your installation:

.. code-block:: python

   import aiofranka
   print(aiofranka.__version__)

   # Test simulation mode (no robot required)
   from aiofranka import RobotInterface
   robot = RobotInterface(None)  # None = simulation
   state = robot.state
   print(f"Joint positions: {state['qpos']}")

If this runs without errors, you're ready to go!

First-Time Robot Setup
----------------------

Before using aiofranka with a real robot, you need to unlock the brakes and activate FCI:

.. code-block:: python

   from aiofranka.client import FrankaLockUnlock

   client = FrankaLockUnlock(
       hostname="172.16.0.2",  # Your robot's IP
       username="admin",
       password="admin"
   )

   # Unlock brakes and activate FCI
   client.run(unlock=True, fci=True, persistent=True)

.. note::
   You only need to do this once per power cycle unless another user takes control.

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**ImportError: No module named 'pylibfranka'**

Solution: pylibfranka must be installed manually. See instructions above.

**ImportError: No module named 'mujoco'**

Solution: Install MuJoCo:

.. code-block:: bash

   pip install mujoco

**ConnectionError when connecting to robot**

Solution: 

1. Check network connection: ``ping 172.16.0.2``
2. Ensure robot is powered on and in FCI mode
3. Check firewall settings
4. Verify no other program has control token

**ModuleNotFoundError: No module named 'ruckig'**

Solution: Install Ruckig:

.. code-block:: bash

   pip install ruckig

Getting Help
~~~~~~~~~~~~

If you encounter issues not covered here:

1. Check the `GitHub Issues <https://github.com/Improbable-AI/aiofranka/issues>`_
2. Consult the :doc:`troubleshooting` guide
3. Open a new issue with details about your problem

Next Steps
----------

- Read the :doc:`quickstart` guide
- Explore :doc:`examples`
- Review :doc:`safety` guidelines before controlling a real robot
