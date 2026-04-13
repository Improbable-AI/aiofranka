import importlib.util

import easyhid.easyhid as _easyhid_backend
import pyspacemouse
import numpy as np


_HID_BACKEND_PATCHED = False


def _patch_hid_backend() -> bool:
    """Patch easyhid to use the hidraw extension when libhidapi-hidraw is missing.

    Some conda environments provide `hidraw` as a Python extension module with
    exported hidapi symbols, but do not provide a standalone `libhidapi-hidraw`
    shared library discoverable by `ctypes.find_library`. In that case, easyhid
    falls back to `dlopen(None)` and later fails with `hid_enumerate` missing.
    """
    global _HID_BACKEND_PATCHED
    if _HID_BACKEND_PATCHED:
        return True

    spec = importlib.util.find_spec("hidraw")
    if spec is None or spec.origin is None:
        return False

    try:
        hidapi_lib = _easyhid_backend.ffi.dlopen(spec.origin)
        # Validate a key symbol before replacing backend.
        _ = hidapi_lib.hid_enumerate
        _easyhid_backend.hidapi = hidapi_lib
        _HID_BACKEND_PATCHED = True
        return True
    except Exception:
        return False


class SpaceMouse:
    """Wrapper around pyspacemouse that drains the HID buffer on each read.

    Without draining, stale HID reports queue up in the kernel buffer and
    cause the robot to keep moving after the spacemouse is released.

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
        try:
            self._device = pyspacemouse.open().__enter__()
        except RuntimeError as e:
            # Recover from missing hidapi shared-library discovery in some envs.
            if "HID API is probably not installed" in str(e) and _patch_hid_backend():
                self._device = pyspacemouse.open().__enter__()
            else:
                raise

    def read(self):
        """Read the latest spacemouse state, draining any buffered HID reports.

        Returns:
            (translation_raw, rotation_raw, buttons):
                translation_raw: np.ndarray (3,) raw normalized axes [-1, 1].
                rotation_raw: np.ndarray (3,) raw euler angles (degrees),
                    respecting yaw_only setting.
                buttons: list[int] button states from the latest event.
        """
        event = self._device.read()
        while self._device._device.read(self._device._info.bytes_to_read):
            event = self._device.read()

        translation_raw = np.array([event.x, event.y, event.z])

        if self.yaw_only:
            rotation_raw = np.array([0, 0, -event.yaw])
        else:
            rotation_raw = np.array([-event.pitch, event.roll, -event.yaw])

        return translation_raw, rotation_raw, event.buttons
