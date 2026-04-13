"""
Robotiq Gripper Interface for aiofranka

This module provides an async wrapper for Robotiq grippers, following the same
design pattern as FrankaController - a background control loop continuously
sends commands while you update the target position.

Requires optional dependencies: pip install "aiofranka[robotiq]"

Example:
    >>> import asyncio
    >>> from aiofranka.gripper import GripperController
    >>> 
    >>> async def main():
    ...     gripper = GripperController("/dev/ttyUSB1")
    ...     await gripper.start()
    ...     gripper.speed = 128  # Set speed (like kp/kd)
    ...     gripper.q_desired = 200  # Set target position
    ...     await asyncio.sleep(1)
    ...     await gripper.stop()
    >>> 
    >>> asyncio.run(main())
"""

import asyncio
import threading
import time
from typing import Optional
import numpy as np


class GripperController:
    """
    Async gripper controller with background control loop.
    
    Similar to FrankaController, this runs a background loop that continuously
    sends goTo commands. You only need to set the target position - the loop
    handles the communication.
    
    Attributes:
        q_desired (int): Target gripper position (0=open, 255=closed)
        qpos (int): Current gripper position (read-only)
        speed (int): Gripper speed setting (1-255), like kp gain
        force (int): Gripper force setting (0-255), like kd gain
        state (dict): Current gripper state (qpos, q_desired, speed, force)
        running (bool): Whether control loop is active
        
    Args:
        port: Serial port for the gripper (e.g., "/dev/ttyUSB1")
        speed: Initial speed setting (default: 255)
        force: Initial force setting (default: 255)
        loop_rate: Control loop frequency in Hz (default: 50)
        
    Example:
        >>> gripper = GripperController("/dev/ttyUSB1")
        >>> await gripper.start()
        >>> 
        >>> # Set gains (speed/force) like you set kp/kd
        >>> gripper.speed = 128
        >>> gripper.force = 200
        >>> 
        >>> # Set target position (like q_desired for Franka)
        >>> gripper.q_desired = 255  # Close
        >>> await asyncio.sleep(0.5)
        >>> gripper.q_desired = 0    # Open
        >>> await asyncio.sleep(0.5)
        >>> 
        >>> # Read current position (like qpos for Franka)
        >>> print(gripper.qpos)
        >>> 
        >>> await gripper.stop()
    """
    
    def __init__(
        self, 
        port: str = "/dev/ttyUSB1",
        speed: int = 255,
        force: int = 255,
        loop_rate: float = 50.0,
        read_every_n: int = 2,
    ):
        try:
            from pyrobotiqgripper import RobotiqGripper
            self._gripper_cls = RobotiqGripper
        except ImportError:
            raise ImportError(
                "pyrobotiqgripper is required for gripper control. "
                "Install it with: pip install 'aiofranka[robotiq]'"
            )
        
        self._port = port
        self._gripper = None
        self._loop_rate = loop_rate
        self._read_every_n = read_every_n  # Read position every Nth cycle
        
        # Control parameters (like kp/kd for Franka)
        self._speed = speed
        self._force = force
        
        # Target position (like q_desired for Franka)
        self._position = 0
        
        # State
        self._current_position = 0
        self._state_lock = threading.Lock()  
        
        # Control loop
        self.running = False
        self._thread = None
        self._task = None
        
        # Rate limiting for set() method
        self._update_freq = 50.0
        self._last_update_time = {}
    
    @property
    def q_desired(self) -> int:
        """Target gripper position (0=open, 255=closed)."""
        with self._state_lock:
            return self._position
    
    @q_desired.setter
    def q_desired(self, value: int):
        """Set target position (0-255)."""
        value = int(np.clip(value, 0, 255))
        with self._state_lock:
            self._position = value
    
    # Backwards compatibility alias
    @property
    def position(self) -> int:
        """Alias for q_desired (deprecated)."""
        return self.q_desired
    
    @position.setter
    def position(self, value: int):
        self.q_desired = value
    
    @property
    def speed(self) -> int:
        """Gripper speed (1-255). Higher = faster. Like kp gain."""
        return self._speed
    
    @speed.setter
    def speed(self, value: int):
        """Set speed (1-255)."""
        self._speed = int(np.clip(value, 1, 255))
    
    @property
    def force(self) -> int:
        """Gripper force (0-255). Higher = stronger grip. Like kd gain."""
        return self._force
    
    @force.setter
    def force(self, value: int):
        """Set force (0-255)."""
        self._force = int(np.clip(value, 0, 255))
    
    @property
    def qpos(self) -> int:
        """Current gripper position (read-only)."""
        with self._state_lock:
            return self._current_position
    
    # Backwards compatibility alias
    @property
    def current_position(self) -> int:
        """Alias for qpos (deprecated)."""
        return self.qpos
    
    @property
    def state(self) -> dict:
        """Current gripper state dictionary."""
        with self._state_lock:
            return {
                'qpos': self._current_position,
                'q_desired': self._position,
                'speed': self._speed,
                'force': self._force,
                'error': abs(self._position - self._current_position)
            }
    
    def set_freq(self, freq: float):
        """
        Set the update frequency for rate-limited set() calls.
        
        Args:
            freq: Desired update frequency in Hz (typically 10-100 Hz)
        """
        self._update_freq = freq
    
    async def set(self, attr: str, value):
        """
        Rate-limited setter (same pattern as FrankaController).
        
        Args:
            attr: Attribute name ("q_desired", "speed", "force")
            value: Value to set
        """
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
    
    def _write_goto(self, position: int, speed: int, force: int):
        """Direct register write for goTo - skips all the overhead in pyrobotiqgripper's goTo()."""
        self._gripper.write_registers(1000, [
            0b0000100100000000,
            position,
            speed * 0b100000000 + force
        ])
    
    def _read_position(self) -> int:
        """Direct register read for position - skips readAll() overhead."""
        registers = self._gripper.read_registers(2000, 3)
        return (registers[2] >> 8) & 0xFF
    
    def _control_loop_step(self, read: bool):
        """One iteration of the control loop (runs in dedicated thread).
        
        Args:
            read: Whether to read position this cycle.
        """
        # Write target position first (lowest latency to gripper)
        with self._state_lock:
            target = self._position
            speed = self._speed
            force = self._force
        
        try:
            self._write_goto(target, speed, force)
        except Exception as e:
            print(f"Gripper command error: {e}")
        
        # Read current position after (only every Nth cycle)
        if read:
            try:
                pos = self._read_position()
                with self._state_lock:
                    self._current_position = pos
            except Exception:
                pass
    
    def _thread_loop(self):
        """Dedicated thread running the control loop.
        
        Avoids run_in_executor overhead by running continuously in its own thread.
        """
        dt = 1.0 / self._loop_rate
        cycle = 0
        
        while self.running:
            t0 = time.perf_counter()
            
            read = (cycle % self._read_every_n == 0)
            self._control_loop_step(read)
            cycle += 1
            
            elapsed = time.perf_counter() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    async def _run(self):
        """Start the control loop in a dedicated thread.
        
        Uses a real thread instead of run_in_executor to avoid
        thread pool scheduling overhead on every iteration.
        """
        self.running = True
        self._thread = threading.Thread(target=self._thread_loop, daemon=True)
        self._thread.start()
        
        # Keep the task alive until stopped
        try:
            while self.running:
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            self.running = False
            if self._thread:
                self._thread.join(timeout=1.0)
    
    async def start(self):
        """
        Initialize gripper and start background control loop.
        
        Returns:
            asyncio.Task: The background control loop task
        """
        print(f"Starting gripper on {self._port}...")
        
        # Initialize gripper
        self._gripper = self._gripper_cls(self._port)
        
        # Reduce serial timeout from 200ms to 50ms for faster round-trips
        self._gripper.serial.timeout = 0.05
        
        self._gripper.resetActivate()
        
        # Read initial position using direct register read
        self._current_position = self._read_position()
        self._position = self._current_position
        
        print(f"Gripper activated. Initial position: {self._current_position}")
        
        # Start control loop
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run())
        
        await asyncio.sleep(0.5)  # Let loop start
        return self._task
    
    async def stop(self):
        """Stop the control loop and wait for thread to finish."""
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
        print("Gripper control loop stopped.")
    
    # Convenience methods
    def open(self):
        """Set target to fully open (0)."""
        self.q_desired = 0
    
    def close(self):
        """Set target to fully closed (255)."""
        self.q_desired = 255
    
    async def wait_until_reached(self, tolerance: int = 5, timeout: float = 5.0) -> bool:
        """
        Wait until gripper reaches target position.
        
        Args:
            tolerance: Position tolerance in gripper units
            timeout: Maximum wait time in seconds
            
        Returns:
            True if position reached, False if timeout
        """
        start = time.perf_counter()
        while time.perf_counter() - start < timeout:
            if abs(self.qpos - self.q_desired) <= tolerance:
                return True
            await asyncio.sleep(0.02)
        return False


# Backwards compatibility alias
RobotiqGripperInterface = GripperController


def create_gripper(port: str = "/dev/ttyUSB1") -> GripperController:
    """
    Factory function to create a GripperController.
    
    Note: You still need to call await gripper.start() to activate.
    
    Args:
        port: Serial port for the gripper
        
    Returns:
        GripperController instance (not yet started)
    """
    return GripperController(port)
