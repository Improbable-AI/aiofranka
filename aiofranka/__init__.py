"""
aiofranka: Asyncio-based Franka Robot Control

A high-level Python library for controlling Franka Emika robots using asyncio.
Combines pylibfranka for real-time control with MuJoCo for kinematics/dynamics.

Main Components:
    RobotInterface: Low-level robot interface (real or simulation)
    FrankaController: High-level asyncio controller with multiple modes
    FrankaLockUnlock: Client for robot authentication and brake control

Quick Example:
    >>> import asyncio
    >>> from aiofranka import RobotInterface, FrankaController
    >>> 
    >>> async def main():
    ...     robot = RobotInterface("172.16.0.2")
    ...     controller = FrankaController(robot)
    ...     await controller.start()
    ...     await controller.move()  # Move to home
    ...     await controller.stop()
    >>> 
    >>> asyncio.run(main())

For detailed documentation, see README.md and USAGE_GUIDE.md
"""

from aiofranka.controller import FrankaController
from aiofranka.robot import RobotInterface
from aiofranka.async_utils import asyncify, async_input, CudaInferenceThread, mpify
from aiofranka.remote import FrankaRemoteController, ServerDiedError
from aiofranka.remote_v2 import FrankaRemoteControllerV2
from aiofranka.server import start, stop, lock, unlock, set_configuration

from aiofranka.gripper import (
    GripperController,
    RobotiqGripperController,
    FrankaGripperController,
    RobotiqGripperInterface,
    create_gripper,
)
from aiofranka.gripper_remote import (
    GripperRemoteController,
    RobotiqGripperRemoteController,
    FrankaGripperRemoteController,
)

__version__ = "0.1.0"
__all__ = ["RobotInterface", "FrankaController", "FrankaRemoteController", "FrankaRemoteControllerV2", "ServerDiedError", "asyncify", "async_input", "CudaInferenceThread", "mpify", "start", "stop", "lock", "unlock", "set_configuration"]

__all__.extend(
    [
        "GripperController",
        "GripperRemoteController",
        "RobotiqGripperController",
        "FrankaGripperController",
        "RobotiqGripperRemoteController",
        "FrankaGripperRemoteController",
        "RobotiqGripperInterface",
        "create_gripper",
    ]
)
