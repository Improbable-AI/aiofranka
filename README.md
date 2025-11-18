# aiofranka

<div align="center">
  <img width="340" src="assets/image.png">
</div>
<p align="center">
  <a href="https://pypi.org/project/aiofranka/">
    <img src="https://img.shields.io/pypi/v/aiofranka" alt="CI">
  </a>
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="CI">
  </a>
</p>

**aiofranka** is an asyncio-based Python library for controlling Franka Emika robots. It provides a high-level, asynchronous interface that combines:
- **pylibfranka**: Official low-level control interface (1kHz torque control)
- **MuJoCo**: Fast kinematics/dynamics computation
- **Ruckig**: Smooth trajectory generation

The library is designed for research applications requiring precise, real-time control with minimal latency and maximum flexibility.

## Features

- 🚀 **High-frequency control**: 1kHz torque control loop with asyncio integration
- 🎮 **Multiple controllers**: Joint impedance, Operational Space Control (OSC), and direct torque control
- 🔄 **Hot-swappable**: Switch controllers and gains at runtime without stopping
- 📊 **Real-time monitoring**: Built-in connection testing and timing diagnostics
- 🛡️ **Safety features**: Torque rate limiting and collision behavior configuration
- 🎯 **Smooth trajectories**: Automatic trajectory generation with Ruckig
- 💻 **Simulation mode**: Test without hardware using MuJoCo viewer

## Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [Core Concepts](#core-concepts)
- [Controllers](#controllers)
- [Examples](#examples)
- [API Reference](#api-reference)
- [Safety & Caveats](#safety--caveats)
- [Troubleshooting](#troubleshooting)

## Installation

### Prerequisites

1. **libfranka and pylibfranka**: Required for real robot control
   ```bash
   # Follow official Franka instructions to install libfranka
   # Then install pylibfranka
   pip install pylibfranka
   ```

2. **MuJoCo**: Required for kinematics/dynamics
   ```bash
   pip install mujoco
   ```

### Installing aiofranka

```bash
pip install aiofranka
```

Or for development:
```bash
git clone https://github.com/Improbable-AI/aiofranka.git
cd aiofranka
pip install -e .
```

## Quick Start

```bash 
python test.py 
```

Basic usage pattern:

```python
import asyncio 
import numpy as np 
from aiofranka import RobotInterface, FrankaController

async def main():
    # Connect to robot (use IP address for real robot, None for simulation)
    robot = RobotInterface("172.16.0.2") 
    controller = FrankaController(robot)
    
    # Start the 1kHz control loop
    await controller.start()

    # Test connection quality
    await controller.test_connection()

    # Move to home position using smooth trajectory
    await controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])

    # Switch to impedance control
    controller.switch("impedance")
    controller.kp = np.ones(7) * 80.0
    controller.kd = np.ones(7) * 4.0
    controller.set_freq(50)  # 50Hz update rate for set() commands
    
    for cnt in range(100): 
        delta = np.sin(cnt / 50.0 * np.pi) * 0.1
        init = controller.initial_qpos
        await controller.set("q_desired", delta + init)

    # Switch to operational space control (OSC)
    controller.switch("osc")
    controller.set_freq(50)  

    for cnt in range(100): 
        delta = np.sin(cnt / 50.0 * np.pi) * 0.1
        init = controller.initial_ee 

        desired_ee = np.eye(4) 
        desired_ee[:3, :3] = init[:3, :3]
        desired_ee[:3, 3] = init[:3, 3] + np.array([0, delta, 0])

        await controller.set("ee_desired", desired_ee)

    # Stop control loop
    await controller.stop()

if __name__ == "__main__":
    asyncio.run(main()) 
```

## Core Concepts

### Asyncio-based Design

The library uses Python's `asyncio` for non-blocking control. The control loop runs at 1kHz in the background while your code sends commands asynchronously:

```python
# Control loop runs in background at 1kHz
await controller.start()

# Your code can await other operations without blocking the control loop
await asyncio.sleep(1.0)
await controller.set("q_desired", target)
```

### Rate Limiting

Use `set_freq()` to enforce strict timing for command updates:

```python
controller.set_freq(50)  # Set 50Hz update rate

# This will automatically sleep to maintain 50Hz timing
for i in range(100):
    await controller.set("q_desired", compute_target())
```

**Caveat**: Without `set_freq()`, commands are sent as fast as possible, which may not be desirable for smooth control.

### State Access

Robot state is continuously updated at 1kHz and accessible via `controller.state`:

```python
state = controller.state  # Thread-safe access
# Contains: qpos, qvel, ee, jac, mm, last_torque
print(f"Joint positions: {state['qpos']}")
print(f"End-effector pose: {state['ee']}")  # 4x4 homogeneous transform
```

## Controllers

### 1. Impedance Control (Joint Space)

Controls joint positions with spring-damper behavior:

```python
controller.switch("impedance")
controller.kp = np.ones(7) * 80.0   # Position gains
controller.kd = np.ones(7) * 4.0    # Damping gains

await controller.set("q_desired", target_joint_angles)
```

**Use case**: Precise joint-space motions, compliant behavior

**Caveats**:
- High `kp` values make the robot stiff and less safe
- Recommended range: `kp` ∈ [20, 200], `kd` ∈ [1, 10]
- Use lower gains near singularities

### 2. Operational Space Control (Task Space)

Controls end-effector pose in Cartesian space:

```python
controller.switch("osc")
controller.ee_kp = np.array([300, 300, 300, 1000, 1000, 1000])  # [xyz, rpy]
controller.ee_kd = np.ones(6) * 10.0

desired_ee = np.eye(4)  # 4x4 homogeneous transform
desired_ee[:3, 3] = [0.4, 0.0, 0.5]  # Position
await controller.set("ee_desired", desired_ee)
```

**Use case**: Cartesian trajectories, end-effector tracking

**Caveats**:
- Higher translation stiffness (kp[:3]) than rotation stiffness (kp[3:])
- Null-space control keeps robot away from joint limits
- May fail near singularities - check Jacobian conditioning

### 3. Direct Torque Control

Send raw joint torques:

```python
controller.switch("torque")
controller.torque = np.array([0, 0, 0, 0, 0, 0, 0])  # Direct torque commands
```

**Use case**: Custom control algorithms, learning-based control

**Caveats**:
- ⚠️ **Most dangerous mode** - no built-in safety
- Torque rate limiting is still active by default
- Test in simulation first!

## Examples

### Example 1: Smooth Trajectory Execution

```python
async def smooth_motion():
    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)
    await controller.start()
    
    # Move uses Ruckig for jerk-limited trajectories
    await controller.move([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
    
    await controller.stop()
```

### Example 2: Spacemouse Teleoperation

See `examples/02_spacemouse_teleop.py` for full code:

```python
import pyspacemouse

controller.switch("osc")
controller.set_freq(50)

while True:
    event = pyspacemouse.read()
    translation = np.array([event.x, event.y, event.z]) * 0.003
    
    current_ee = controller.ee_desired.copy()
    current_ee[:3, 3] += translation
    
    await controller.set("ee_desired", current_ee)
```

### Example 3: Data Collection

Collect synchronized robot state and control commands:

```python
controller.switch("impedance")
controller.set_freq(50)

logs = {'qpos': [], 'qvel': [], 'ctrl': [], 'qdes': []}

for cnt in range(200):
    logs['qpos'].append(controller.state['qpos'].copy())
    logs['qvel'].append(controller.state['qvel'].copy())
    logs['ctrl'].append(controller.state['last_torque'].copy())
    logs['qdes'].append(controller.q_desired.copy())
    
    # Generate sinusoidal reference
    delta = np.sin(cnt / 50.0 * np.pi) * 0.15
    await controller.set("q_desired", delta + controller.initial_qpos)

# Save data
for key in logs:
    logs[key] = np.stack(logs[key])
np.savez("trajectory.npz", **logs)
```

### Example 4: Simulation Mode

Test without hardware:

```python
# Pass None for IP to use MuJoCo viewer
robot = RobotInterface(None)
controller = FrankaController(robot)

await controller.start()
# Same control code works in simulation!
await controller.move([0, 0, 0, -1.57, 0, 1.57, -0.785])
```

## API Reference

### RobotInterface

```python
robot = RobotInterface(ip: str | None)
```

**Parameters**:
- `ip`: Robot IP address (e.g., "172.16.0.2") or `None` for simulation

**Properties**:
- `robot.state`: Current robot state (dict with qpos, qvel, ee, jac, mm, last_torque)
- `robot.model`: MuJoCo model
- `robot.data`: MuJoCo data

**Methods**:
- `robot.start()`: Start torque control mode
- `robot.stop()`: Stop robot
- `robot.step(torque)`: Send torque command (called by controller)

### FrankaController

```python
controller = FrankaController(robot: RobotInterface)
```

**Properties**:
- `controller.state`: Robot state (updated at 1kHz)
- `controller.type`: Current controller type ("impedance", "osc", "torque")
- `controller.kp`, `controller.kd`: Joint impedance gains (shape: (7,))
- `controller.ee_kp`, `controller.ee_kd`: OSC gains (shape: (6,), [xyz, rpy])
- `controller.null_kp`, `controller.null_kd`: Null-space gains (shape: (7,))
- `controller.initial_qpos`: Initial joint positions
- `controller.initial_ee`: Initial end-effector pose
- `controller.q_desired`: Desired joint positions
- `controller.ee_desired`: Desired end-effector pose (4x4)

**Methods**:
- `await controller.start()`: Start control loop
- `await controller.stop()`: Stop control loop
- `controller.switch(type: str)`: Switch controller ("impedance", "osc", "torque")
- `controller.set_freq(freq: float)`: Set update frequency for `set()` (Hz)
- `await controller.set(attr: str, value)`: Rate-limited setter
- `await controller.move(qpos: list)`: Move to joint position with smooth trajectory
- `await controller.test_connection()`: Test control loop timing (5 seconds)

### FrankaClient (Advanced)

Low-level client for robot authentication and control token management:

```python
from aiofranka.client import FrankaLockUnlock

client = FrankaLockUnlock(
    hostname="172.16.0.2",
    username="admin", 
    password="admin",
    protocol="https"
)

# Unlock brakes and activate FCI
client.run(unlock=True, fci=True, persistent=True)
```

**Caveats**:
- Must call before using `RobotInterface` for first time
- Requires admin credentials
- `persistent=True` keeps token active

## Safety & Caveats

### ⚠️ Critical Safety Notes

1. **Always test in simulation first**
   ```python
   robot = RobotInterface(None)  # Simulation mode
   ```

2. **Start with low gains**
   ```python
   controller.kp = np.ones(7) * 20.0  # Start conservative
   controller.kd = np.ones(7) * 2.0
   ```

3. **Enable emergency stop**
   - Keep one hand near the E-stop button
   - Configure external activation device (EAD)

4. **Torque rate limiting**
   ```python
   controller.torque_diff_limit = 990.0  # N⋅m/s (default)
   controller.clip = True  # Enable rate limiting (default)
   ```

5. **Collision behavior**
   ```python
   robot.robot.set_collision_behavior(
       [100.0] * 7,  # Lower thresholds
       [100.0] * 7,
       [100.0] * 6,
       [100.0] * 6
   )
   ```

### Common Pitfalls

1. **Forgetting to await**
   ```python
   # ❌ Wrong - controller never starts!
   controller.start()
   
   # ✅ Correct
   await controller.start()
   ```

2. **Not using set_freq()**
   ```python
   # ❌ Sends commands too fast
   for i in range(100):
       await controller.set("q_desired", target)
   
   # ✅ Enforces 50Hz timing
   controller.set_freq(50)
   for i in range(100):
       await controller.set("q_desired", target)
   ```

3. **Modifying state without lock**
   ```python
   # ❌ Race condition
   controller.kp = new_gains
   
   # ✅ Thread-safe
   with controller.state_lock:
       controller.kp = new_gains
   ```

4. **Large discontinuous commands**
   ```python
   # ❌ Sudden jump causes safety trigger
   await controller.set("q_desired", far_away_target)
   
   # ✅ Use smooth trajectory
   await controller.move(far_away_target)
   ```

### Debugging

**Test connection quality:**
```python
await controller.test_connection()
# Prints: frequency, jitter, timing statistics
```

**Check state synchronization:**
```python
print(f"Control loop running: {controller.running}")
print(f"Current type: {controller.type}")
print(f"Joint positions: {controller.state['qpos']}")
```

**Monitor torques:**
```python
torque = controller.state['last_torque']
if np.any(np.abs(torque) > 80):
    print("High torque detected!")
```

## Troubleshooting

### "Error requesting control token"
- Another user/program has control
- Use `FrankaLockUnlock` with `wait=True` to request control
- Check Desk interface for active sessions

### High torque rate warnings
- Decrease gains: `controller.kp /= 2`
- Increase damping: `controller.kd *= 2`
- Check for discontinuous commands

### Control loop frequency drops
- Reduce computation in main loop
- Check network latency (ping robot IP)
- Ensure no blocking operations in control loop

### Robot motion is jerky
- Use `set_freq()` for consistent timing
- Increase damping gains
- Check if commands are smooth (plot q_desired)

### Simulation vs Real discrepancies
- MuJoCo uses idealized dynamics
- Real robot has friction, compliance, latency
- Tune gains separately for sim and real

## Contributing

Contributions welcome! Please:
1. Test changes in simulation and on hardware
2. Follow existing code style
3. Add docstrings and type hints
4. Update documentation

## License

MIT License - see LICENSE file

## Citation

If you use this library in your research, please cite:

```bibtex
@software{aiofranka,
  author = {Improbable AI Lab},
  title = {aiofranka: Asyncio-based Franka Robot Control},
  year = {2024},
  url = {https://github.com/Improbable-AI/aiofranka}
}
```

## Acknowledgments

- Built on [libfranka](https://frankaemika.github.io/docs/) by Franka Emika
- Uses [MuJoCo](https://mujoco.org/) physics engine
- Trajectory generation with [Ruckig](https://github.com/pantor/ruckig)