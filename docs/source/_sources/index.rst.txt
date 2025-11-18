Welcome to aiofranka's documentation!
=====================================

.. image:: https://img.shields.io/pypi/v/aiofranka
   :target: https://pypi.org/project/aiofranka/
   :alt: PyPI version

.. image:: https://img.shields.io/badge/License-MIT-yellow.svg
   :target: https://opensource.org/licenses/MIT
   :alt: License: MIT

**aiofranka** is an asyncio-based client for controlling Franka robots using the official 
``pylibfranka`` package as a control interface with Franka, and ``mujoco`` as kinematics/dynamics model.

Features
--------

* **Asyncio-based control**: Non-blocking control loops running at 1kHz
* **Multiple control modes**: Impedance control, Operational Space Control (OSC), and direct torque control
* **Real-time trajectory generation**: Using Ruckig for smooth motion planning
* **MuJoCo integration**: Accurate kinematics and dynamics simulation
* **Rate-limited setters**: Precise timing control for controller updates

Quick Links
-----------

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   installation
   quickstart
   api
   examples

Installation
------------

1. Follow instructions from the official repository and install ``pylibfranka``.
2. Install the package::

    pip install aiofranka

Quick Example
-------------

.. code-block:: python

    import asyncio
    import numpy as np
    from aiofranka import RobotInterface, FrankaController

    async def main():
        robot = RobotInterface("172.16.0.2")
        controller = FrankaController(robot)
        
        await controller.start()
        
        # Move to a position
        await controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])
        
        # Switch to impedance control
        controller.switch("impedance")
        controller.kp = np.ones(7) * 80.0
        controller.kd = np.ones(7) * 4.0
        controller.set_freq(50)
        
        for cnt in range(100):
            delta = np.sin(cnt / 50.0 * np.pi) * 0.1
            await controller.set("q_desired", controller.initial_qpos + delta)

    if __name__ == "__main__":
        asyncio.run(main())

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
