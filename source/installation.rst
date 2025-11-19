Installation
============

System Requirements
-------------

- Linux operating system (Ubuntu 20.04+ recommended)
- Real-time kernel 
- Network connection to Franka robot 



Installing aiofranka
--------------------

From PyPI (Recommended)
~~~~~~~~~~~

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

