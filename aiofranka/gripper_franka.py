"""
Franka Hand gripper controller (async API).

Provides a Robotiq-compatible command surface:
- q_desired: 0=open, 255=closed
- qpos: current position in the same 0..255 scale
- speed: 1..255
- force: 0..255
"""

from __future__ import annotations

import asyncio
import threading
import time

import numpy as np


class FrankaGripperController:
    """Async controller for the built-in Franka Hand gripper."""

    _FRANKA_MIN_SPEED = 0.01  # m/s
    _FRANKA_MAX_SPEED = 0.20  # m/s
    _FRANKA_MIN_FORCE = 1.0   # N
    _FRANKA_MAX_FORCE = 70.0  # N

    def __init__(
        self,
        ip: str = "172.16.0.2",
        speed: int = 255,
        force: int = 255,
        loop_rate: float = 50.0,
        read_every_n: int = 2,
        home: bool = False,
    ):
        try:
            import pylibfranka
        except ImportError as exc:
            raise ImportError(
                "Franka gripper backend requires pylibfranka."
            ) from exc

        self._gripper_cls = pylibfranka.Gripper
        self._ip = ip
        self._home = bool(home)

        self._gripper = None
        self._loop_rate = float(loop_rate)
        self._read_every_n = max(1, int(read_every_n))

        self._speed = int(np.clip(speed, 1, 255))
        self._force = int(np.clip(force, 0, 255))
        self._position = 0

        self._current_position = 0
        self._is_grasped = False
        self._max_width_m = 0.08
        self._state_lock = threading.Lock()

        self.running = False
        self._thread = None
        self._task = None
        self._pending_motion = True

        self._update_freq = 50.0
        self._last_update_time = {}

    @property
    def q_desired(self) -> int:
        with self._state_lock:
            return self._position

    @q_desired.setter
    def q_desired(self, value: int):
        with self._state_lock:
            self._position = int(np.clip(value, 0, 255))
            self._pending_motion = True

    @property
    def position(self) -> int:
        return self.q_desired

    @position.setter
    def position(self, value: int):
        self.q_desired = value

    @property
    def speed(self) -> int:
        return self._speed

    @speed.setter
    def speed(self, value: int):
        self._speed = int(np.clip(value, 1, 255))
        with self._state_lock:
            self._pending_motion = True

    @property
    def force(self) -> int:
        return self._force

    @force.setter
    def force(self, value: int):
        self._force = int(np.clip(value, 0, 255))
        with self._state_lock:
            self._pending_motion = True

    @property
    def qpos(self) -> int:
        with self._state_lock:
            return self._current_position

    @property
    def current_position(self) -> int:
        return self.qpos

    @property
    def state(self) -> dict:
        with self._state_lock:
            return {
                "qpos": self._current_position,
                "q_desired": self._position,
                "speed": self._speed,
                "force": self._force,
                "error": abs(self._position - self._current_position),
                "backend": "franka",
                "is_grasped": self._is_grasped,
            }

    def set_freq(self, freq: float):
        self._update_freq = float(freq)

    async def set(self, attr: str, value):
        current_time = time.perf_counter()
        dt = 1.0 / self._update_freq

        if attr not in self._last_update_time:
            self._last_update_time[attr] = current_time
            await asyncio.sleep(dt)
            setattr(self, attr, value)
            self._last_update_time[attr] = current_time + dt
            return

        target_time = self._last_update_time[attr] + dt
        sleep_time = target_time - current_time
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)

        setattr(self, attr, value)
        self._last_update_time[attr] = target_time

    def _speed_to_mps(self, speed: int) -> float:
        alpha = (int(np.clip(speed, 1, 255)) - 1) / 254.0
        return self._FRANKA_MIN_SPEED + alpha * (self._FRANKA_MAX_SPEED - self._FRANKA_MIN_SPEED)

    def _force_to_newton(self, force: int) -> float:
        alpha = int(np.clip(force, 0, 255)) / 255.0
        return self._FRANKA_MIN_FORCE + alpha * (self._FRANKA_MAX_FORCE - self._FRANKA_MIN_FORCE)

    def _q_to_width(self, q: int) -> float:
        q = int(np.clip(q, 0, 255))
        return float(np.clip((1.0 - q / 255.0) * self._max_width_m, 0.0, self._max_width_m))

    def _width_to_q(self, width: float) -> int:
        if self._max_width_m <= 1e-6:
            return 0
        alpha = 1.0 - float(np.clip(width / self._max_width_m, 0.0, 1.0))
        return int(np.clip(round(alpha * 255.0), 0, 255))

    def _read_state(self):
        state = self._gripper.read_once()
        max_width = float(state.max_width)
        width = float(state.width)
        if max_width > 1e-5:
            self._max_width_m = max_width
        qpos = self._width_to_q(width)
        with self._state_lock:
            self._current_position = qpos
            self._is_grasped = bool(state.is_grasped)

    def _command_motion(self, target: int, speed: int, force: int, current: int):
        target_width = self._q_to_width(target)
        speed_mps = self._speed_to_mps(speed)
        force_n = self._force_to_newton(force)
        closing = target > current

        if closing:
            ok = self._gripper.grasp(target_width, speed_mps, force_n)
            if not ok:
                self._gripper.move(target_width, speed_mps)
        else:
            self._gripper.move(target_width, speed_mps)

    def _control_loop_step(self, read: bool):
        with self._state_lock:
            pending = self._pending_motion
            target = self._position
            speed = self._speed
            force = self._force
            current = self._current_position
            if pending:
                self._pending_motion = False

        if pending:
            try:
                self._command_motion(target, speed, force, current)
            except Exception as e:  # noqa: BLE001
                print(f"Franka gripper command error: {e}")
            read = True

        if read:
            try:
                self._read_state()
            except Exception:
                pass

    def _thread_loop(self):
        dt = 1.0 / self._loop_rate
        cycle = 0

        while self.running:
            t0 = time.perf_counter()
            read = cycle % self._read_every_n == 0
            self._control_loop_step(read)
            cycle += 1

            elapsed = time.perf_counter() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    async def _run(self):
        self.running = True
        self._thread = threading.Thread(target=self._thread_loop, daemon=True)
        self._thread.start()

        try:
            while self.running:
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            self.running = False
            if self._thread:
                self._thread.join(timeout=1.0)

    async def start(self):
        print(f"Starting Franka gripper on robot {self._ip}...")
        self._gripper = self._gripper_cls(self._ip)
        if self._home:
            self._gripper.homing()
        self._read_state()
        with self._state_lock:
            self._position = self._current_position
            self._pending_motion = False

        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run())

        await asyncio.sleep(0.2)
        return self._task

    async def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
        if self._gripper is not None:
            try:
                self._gripper.stop()
            except Exception:
                pass

    def open(self):
        self.q_desired = 0

    def close(self):
        self.q_desired = 255

    async def wait_until_reached(self, tolerance: int = 5, timeout: float = 5.0) -> bool:
        start = time.perf_counter()
        while time.perf_counter() - start < timeout:
            with self._state_lock:
                err = abs(self._current_position - self._position)
                grasped = self._is_grasped
                target = self._position
            if err <= tolerance:
                return True
            if target >= 250 and grasped:
                return True
            await asyncio.sleep(0.02)
        return False
