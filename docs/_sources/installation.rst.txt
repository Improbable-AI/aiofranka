Installation
============

Requirements
------------

* Python >= 3.8
* ``pylibfranka`` (official Franka Emika control library)
* ``mujoco``
* ``numpy``
* ``scipy``
* ``ruckig``

Installing pylibfranka
-----------------------

Before installing aiofranka, you need to install the official ``pylibfranka`` library.
Please follow the instructions from the official Franka Emika repository:

https://frankaemika.github.io/docs/

Installing aiofranka
--------------------

Once ``pylibfranka`` is installed, you can install aiofranka via pip::

    pip install aiofranka

Development Installation
------------------------

To install aiofranka for development::

    git clone https://github.com/Improbable-AI/aiofranka.git
    cd aiofranka
    pip install -e .

Optional Dependencies
---------------------

For the vision examples, you may need additional dependencies::

    pip install opencv-python pyzmq

For spacemouse teleoperation::

    pip install pyspacemouse
