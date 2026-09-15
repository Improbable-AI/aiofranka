"""SpaceMouse input helpers; importing this module does not open HID devices."""

from types import SimpleNamespace

import numpy as np


def open_spacemouse():
    """Open nonblocking input with the axis convention used by existing teleop."""
    try:
        import pyspacemouse
    except ImportError as exc:
        raise RuntimeError("Install SpaceMouse support: python -m pip install 'pyspacemouse>=2,<3'") from exc
    legacy = getattr(getattr(pyspacemouse, "AxisConvention", None), "LEGACY", None)
    options = {"axis_convention": legacy} if legacy is not None else {}
    return pyspacemouse.open(nonblocking=True, **options)


def read_latest_state(device, max_reads=256):
    """Drain nonblocking PySpaceMouse 2.x public reads and copy the final state.

    Each public read parses one queued report, combining split axis and button
    reports into the device's state. Its timestamp stops changing when no report
    remains. The library mutates that same state object, so save its timestamp
    before reading again. Never read the private HID handle: that drops reports.

    ``max_reads`` includes the final empty-queue probe. If the queue cannot drain
    within this bound, raise instead of returning a potentially stale command.
    The caller must open the device in nonblocking mode (the 2.x default).
    """
    if not isinstance(max_reads, int) or max_reads < 2:
        raise ValueError("max_reads must be an integer of at least two")
    state = device.read()
    for _ in range(max_reads - 1):
        previous_t = float(state.t)
        if not np.isfinite(previous_t):
            raise ValueError("SpaceMouse sent a nonfinite timestamp")
        state = device.read()
        if state.t == previous_t:
            names = ("x", "y", "z", "roll", "pitch", "yaw")
            axes = np.array([getattr(state, name) for name in names], dtype=float)
            if not np.isfinite(axes).all():
                raise ValueError("SpaceMouse sent nonfinite axes")
            return SimpleNamespace(t=previous_t, **dict(zip(names, axes)),
                                   buttons=list(state.buttons))
    raise RuntimeError(f"SpaceMouse input queue did not drain within {max_reads} reads")


class SpaceMouse:
    """Wrapper around pyspacemouse that drains the HID buffer on each read.

    Without draining, stale HID reports queue up in the kernel buffer and
    cause the robot to keep moving after the spacemouse is released.

    Scale and clip arguments remain available as attributes for callers.
    read() returns raw normalized axes and does not apply those attributes;
    the teleoperation loop applies its own translation scale.

    Args:
        translation_scale: Multiplier for translation deltas (m per axis unit).
        translation_clip: Max absolute translation delta per read (m).
        rotation_scale: Multiplier for rotation deltas (degrees per axis unit).
        rotation_clip: Max absolute rotation delta per read (degrees).
    """

    def __init__(
        self,
        translation_scale: float = 0.006,
        translation_clip: float = 0.006,
        rotation_scale: float = 0.8,
        rotation_clip: float = 0.8,
        yaw_only: bool = False,
    ):
        self.translation_scale = translation_scale
        self.translation_clip = translation_clip
        self.rotation_scale = rotation_scale
        self.rotation_clip = rotation_clip
        self.yaw_only = yaw_only
        self._device = open_spacemouse().__enter__()

    def read(self):
        """Read the latest spacemouse state, draining any buffered HID reports.

        Returns:
            (translation_raw, rotation_raw, buttons):
                translation_raw: np.ndarray (3,) raw normalized axes [-1, 1].
                rotation_raw: np.ndarray (3,) normalized rotation axes,
                    respecting the original signs and yaw_only setting.
                buttons: list[int] button states from the latest event.
        """
        event = read_latest_state(self._device)

        translation_raw = np.array([event.x, event.y, event.z])

        if self.yaw_only:
            rotation_raw = np.array([0, 0, -event.yaw])
        else:
            rotation_raw = np.array([-event.pitch, event.roll, -event.yaw])

        return translation_raw, rotation_raw, event.buttons
