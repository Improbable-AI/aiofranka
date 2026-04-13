"""
Synchronous gripper frontend (drop-in API).

Preserves `GripperRemoteController` while selecting backend implementations:
- `gripper_remote_robotiq.py`
- `gripper_remote_franka.py`
"""

from __future__ import annotations

from aiofranka.gripper_remote_robotiq import (
    GripperRemoteController as RobotiqGripperRemoteController,
)
from aiofranka.gripper_remote_franka import FrankaGripperRemoteController


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


class GripperRemoteController:
    """
    Drop-in sync gripper controller selector.

    Args:
        port: Serial port (Robotiq) or robot IP (Franka hand).
        backend: "auto", "robotiq", or "franka".
        **kwargs: forwarded to backend controller.
    """

    def __new__(cls, port: str = "/dev/ttyUSB1", *args, backend: str = "auto", **kwargs):
        selected = _resolve_backend(backend, port)
        if selected == "robotiq":
            kwargs.pop("home", None)
            return RobotiqGripperRemoteController(port, *args, **kwargs)
        return FrankaGripperRemoteController(port, *args, **kwargs)


__all__ = [
    "GripperRemoteController",
    "RobotiqGripperRemoteController",
    "FrankaGripperRemoteController",
]
