"""
Gripper frontend (drop-in API).

This module preserves the historical `GripperController` name while selecting
between backend implementations:
- `gripper_robotiq.py`  (serial Robotiq)
- `gripper_franka.py`   (native Franka hand)
"""

from __future__ import annotations

from aiofranka.gripper_robotiq import (
    GripperController as RobotiqGripperController,
    RobotiqGripperInterface,
)
from aiofranka.gripper_franka import FrankaGripperController


def _is_serial_endpoint(endpoint: str) -> bool:
    endpoint = str(endpoint)
    return endpoint.startswith("/dev/") or endpoint.upper().startswith("COM")


def _resolve_backend(backend: str, endpoint: str) -> str:
    backend = backend.lower()
    if backend not in {"auto", "robotiq", "franka"}:
        raise ValueError("backend must be one of: auto, robotiq, franka")
    if backend != "auto":
        return backend
    return "robotiq" if _is_serial_endpoint(endpoint) else "franka"


class GripperController:
    """
    Drop-in gripper controller selector.

    Args:
        port: Serial port (Robotiq) or robot IP (Franka hand).
        backend: "auto", "robotiq", or "franka".
        **kwargs: forwarded to backend controller.
    """

    def __new__(cls, port: str = "/dev/ttyUSB1", *args, backend: str = "auto", **kwargs):
        selected = _resolve_backend(backend, port)
        if selected == "robotiq":
            kwargs.pop("home", None)
            return RobotiqGripperController(port, *args, **kwargs)
        return FrankaGripperController(port, *args, **kwargs)


def create_gripper(
    port: str = "/dev/ttyUSB1",
    *,
    backend: str = "auto",
    **kwargs,
):
    """Factory that returns a backend-specific gripper controller."""
    return GripperController(port, backend=backend, **kwargs)


__all__ = [
    "GripperController",
    "RobotiqGripperController",
    "FrankaGripperController",
    "RobotiqGripperInterface",
    "create_gripper",
]
