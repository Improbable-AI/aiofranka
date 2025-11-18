.. aiofranka documentation master file

Welcome to aiofranka's documentation!
======================================

**aiofranka** is an asyncio-based Python library for controlling Franka Emika robots. It provides a high-level, asynchronous interface that combines:

- **pylibfranka**: Official low-level control interface (1kHz torque control)
- **MuJoCo**: Fast kinematics/dynamics computation
- **Ruckig**: Smooth trajectory generation

.. image:: https://img.shields.io/pypi/v/aiofranka
   :alt: PyPI version
   :target: https://pypi.org/project/aiofranka/

.. image:: https://img.shields.io/badge/License-MIT-yellow.svg
   :alt: License: MIT
   :target: https://opensource.org/licenses/MIT

Features
--------

- 🚀 **High-frequency control**: 1kHz torque control loop with asyncio integration
- 🎮 **Multiple controllers**: Joint impedance, Operational Space Control (OSC), and direct torque control
- 🔄 **Hot-swappable**: Switch controllers and gains at runtime without stopping
- 📊 **Real-time monitoring**: Built-in connection testing and timing diagnostics
- 🛡️ **Safety features**: Torque rate limiting and collision behavior configuration
- 🎯 **Smooth trajectories**: Automatic trajectory generation with Ruckig
- 💻 **Simulation mode**: Test without hardware using MuJoCo viewer

Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: Getting Started:

   installation
   quickstart
   examples

.. toctree::
   :maxdepth: 2
   :caption: User Guide:

   controllers
   safety
   troubleshooting

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
