.. aiofranka documentation master file

Welcome to aiofranka's documentation!
======================================

**aiofranka** is an asyncio-based client for controlling Franka robots using the official ``pylibfranka`` package as a control interface with Franka, and ``mujoco`` as kinematics/dynamics model.

.. image:: https://img.shields.io/pypi/v/aiofranka
   :alt: PyPI version
   :target: https://pypi.org/project/aiofranka/

.. image:: https://img.shields.io/badge/License-MIT-yellow.svg
   :alt: License: MIT
   :target: https://opensource.org/licenses/MIT

Installation
------------

1. Follow instructions `here <https://frankaemika.github.io/docs/>`_ and install ``pylibfranka``.
2. Install the package:

.. code-block:: bash

   pip install aiofranka

Quick Start
-----------

.. code-block:: python

   import asyncio 
   import numpy as np 
   from aiofranka import RobotInterface, FrankaController

   async def main():
       robot = RobotInterface("172.16.0.2") 
       controller = FrankaController(robot)
       
       await controller.start()

       # Test the 1kHz connection with the robot 
       await controller.test_connection()

       # Move to a specific position
       await controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])

       # Switch to impedance controller
       controller.switch("impedance")
       controller.kp = np.ones(7) * 80.0
       controller.kd = np.ones(7) * 4.0
       controller.set_freq(50) 
       
       for cnt in range(100): 
           delta = np.sin(cnt / 50.0 * np.pi) * 0.1
           init = controller.initial_qpos
           await controller.set("q_desired", delta + init)

   if __name__ == "__main__":
       asyncio.run(main())

Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: API Reference:

   api/robot
   api/controller
   api/client

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
