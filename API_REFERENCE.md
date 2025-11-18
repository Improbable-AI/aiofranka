# API Reference

Quick reference for aiofranka classes, methods, and attributes.

## RobotInterface

```python
robot = RobotInterface(ip: str | None)
```

Interface for Franka FR3 robot (real or simulation).

### Parameters
- **ip** (`str | None`): Robot IP address (e.g., "172.16.0.2") or `None` for simulation

### Properties
- **state** (`dict`): Current robot state
  - `qpos` (`np.ndarray`): Joint positions [rad] (7,)
  - `qvel` (`np.ndarray`): Joint velocities [rad/s] (7,)
  - `ee` (`np.ndarray`): End-effector pose 4x4 transform
  - `jac` (`np.ndarray`): Jacobian (6, 7)
  - `mm` (`np.ndarray`): Mass matrix (7, 7)
  - `last_torque` (`np.ndarray`): Last torques [Nm] (7,)
- **model** (`mujoco.MjModel`): MuJoCo model
- **data** (`mujoco.MjData`): MuJoCo data
- **real** (`bool`): True if connected to real robot

### Methods

#### start()
```python
robot.start()
```
Start torque control mode (real robot only).

#### stop()
```python
robot.stop()
```
Stop torque control mode (real robot only).

#### step(torque)
```python
robot.step(torque: np.ndarray)
```
Send torque command or step simulation.
- **torque** (`np.ndarray`): Joint torques [Nm] (7,)

---

## FrankaController

```python
controller = FrankaController(robot: RobotInterface)
```

High-level asyncio controller with multiple control modes.

### Attributes

#### Control Mode
- **type** (`str`): Current controller ("impedance" | "osc" | "torque")
- **running** (`bool`): Whether control loop is active

#### Impedance Control Gains
- **kp** (`np.ndarray`): Joint stiffness [Nm/rad] (7,)
- **kd** (`np.ndarray`): Joint damping [Nm⋅s/rad] (7,)

#### OSC Gains
- **ee_kp** (`np.ndarray`): EE stiffness (6,) [N/m for xyz, Nm/rad for rpy]
- **ee_kd** (`np.ndarray`): EE damping (6,)
- **null_kp** (`np.ndarray`): Null-space stiffness [Nm/rad] (7,)
- **null_kd** (`np.ndarray`): Null-space damping (7,)

#### Target States
- **q_desired** (`np.ndarray`): Desired joint positions [rad] (7,)
- **ee_desired** (`np.ndarray`): Desired EE pose 4x4 transform
- **torque** (`np.ndarray`): Direct torque command [Nm] (7,)

#### Initial States
- **initial_qpos** (`np.ndarray`): Initial joint positions [rad] (7,)
- **initial_ee** (`np.ndarray`): Initial EE pose 4x4 transform

#### Safety
- **clip** (`bool`): Enable torque rate limiting (default: True)
- **torque_diff_limit** (`float`): Max torque rate [Nm/s] (default: 990)

#### Other
- **state** (`dict`): Current robot state (same as robot.state)
- **state_lock** (`threading.Lock`): Lock for thread-safe updates

### Methods

#### async start()
```python
await controller.start()
```
Start 1kHz background control loop. Returns asyncio Task.

#### async stop()
```python
await controller.stop()
```
Stop control loop and robot.

#### switch(controller_type)
```python
controller.switch(controller_type: str)
```
Switch control mode at runtime.
- **controller_type** (`str`): "impedance" | "osc" | "torque"

#### set_freq(freq)
```python
controller.set_freq(freq: float)
```
Set update frequency for rate-limited `set()` calls.
- **freq** (`float`): Update frequency [Hz] (typically 10-200)

#### async set(attr, value)
```python
await controller.set(attr: str, value)
```
Rate-limited setter with automatic timing.
- **attr** (`str`): Attribute name ("q_desired" | "ee_desired" | "torque")
- **value**: Value to set (type depends on attr)

#### async move(qpos)
```python
await controller.move(qpos: list | np.ndarray = [0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
```
Move to target joint position with smooth trajectory.
- **qpos**: Target joint positions [rad] (7,)

#### async test_connection()
```python
await controller.test_connection()
```
Test control loop timing for 5 seconds and print statistics.

---

## FrankaLockUnlock

```python
from aiofranka.client import FrankaLockUnlock

client = FrankaLockUnlock(
    hostname: str,
    username: str,
    password: str,
    protocol: str = 'https',
    relock: bool = False
)
```

Client for robot authentication and brake control.

### Parameters
- **hostname** (`str`): Robot IP (e.g., "172.16.0.2")
- **username** (`str`): Admin username (typically "admin")
- **password** (`str`): Admin password
- **protocol** (`str`): "http" or "https" (default: "https")
- **relock** (`bool`): Auto-lock brakes on exit (default: False)

### Methods

#### run()
```python
client.run(
    unlock: bool = False,
    force: bool = False,
    wait: bool = False,
    request: bool = False,
    persistent: bool = False,
    fci: bool = False,
    home: bool = False
)
```

Execute robot control actions.

**Parameters:**
- **unlock** (`bool`): Open/close brakes
- **force** (`bool`): Force action even if unsafe
- **wait** (`bool`): Wait for control token if unavailable
- **request** (`bool`): Request physical access (button press)
- **persistent** (`bool`): Keep control token active
- **fci** (`bool`): Activate Franka Control Interface
- **home** (`bool`): Home the gripper

**Common Usage:**
```python
# Unlock and activate FCI (first time setup)
client.run(unlock=True, fci=True, persistent=True)

# Lock brakes
client.run(unlock=False)

# Request physical access
client.run(unlock=True, request=True, wait=True)
```

---

## Type Reference

### Joint Positions/Velocities/Torques
- **Shape**: `(7,)`
- **Type**: `np.ndarray`
- **Units**: [rad], [rad/s], [Nm]
- **Order**: [joint1, joint2, ..., joint7]

### End-Effector Pose
- **Shape**: `(4, 4)`
- **Type**: `np.ndarray`
- **Format**: Homogeneous transformation matrix
```python
ee = np.eye(4)
ee[:3, :3] = rotation_matrix  # 3x3 rotation (SO(3))
ee[:3, 3] = position_vector   # 3x1 position [x, y, z] in meters
```

### Jacobian
- **Shape**: `(6, 7)`
- **Type**: `np.ndarray`
- **Format**: [linear velocity; angular velocity] per joint velocity

### Mass Matrix
- **Shape**: `(7, 7)`
- **Type**: `np.ndarray`
- **Units**: [kg⋅m²]
- **Property**: Symmetric, positive-definite

---

## Controller Modes Comparison

| Feature | Impedance | OSC | Torque |
|---------|-----------|-----|--------|
| **Control Space** | Joint | Task | Joint |
| **Input Type** | Joint positions | EE pose | Torques |
| **Complexity** | Low | Medium | High |
| **Safety** | High | High | User-dependent |
| **Use Case** | Joint trajectories | Cartesian motion | Custom control |
| **Recommended Gains** | kp: 20-200<br>kd: 2-10 | ee_kp: 100-500<br>ee_kd: 5-20 | N/A |

---

## Common Patterns

### Basic Control Loop
```python
async def control_loop():
    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)
    
    await controller.start()
    
    try:
        controller.switch("impedance")
        controller.set_freq(50)
        
        for i in range(200):
            target = compute_target(i)
            await controller.set("q_desired", target)
    finally:
        await controller.stop()

asyncio.run(control_loop())
```

### Thread-Safe Parameter Update
```python
# Reading (always safe)
state = controller.state
gains = controller.kp

# Writing (use lock)
with controller.state_lock:
    controller.kp = new_gains
    controller.kd = new_damping
```

### Data Collection
```python
logs = {'qpos': [], 'qvel': [], 'ctrl': []}
controller.set_freq(100)

for i in range(1000):
    state = controller.state
    logs['qpos'].append(state['qpos'].copy())
    logs['qvel'].append(state['qvel'].copy())
    logs['ctrl'].append(state['last_torque'].copy())
    
    await controller.set("q_desired", target)

# Convert to numpy
for key in logs:
    logs[key] = np.array(logs[key])
np.savez("data.npz", **logs)
```

### Error Handling
```python
try:
    await controller.start()
    # ... control code ...
except KeyboardInterrupt:
    print("Interrupted")
except Exception as e:
    print(f"Error: {e}")
finally:
    await controller.stop()
```

---

## Constants & Limits

### Joint Limits (FR3)
- **Position**: See robot datasheet
- **Velocity**: ±2.62 rad/s (joints 1-3), ±2.62 rad/s (joints 4-7)
- **Torque**: 87 Nm (joints 1-4), 12 Nm (joints 5-7)
- **Jerk**: Limited by Ruckig trajectory generation

### Control Loop
- **Frequency**: 1000 Hz (real robot), variable (simulation)
- **Command Rate**: Typically 10-200 Hz via `set_freq()`
- **Max Update Rate**: Don't exceed 500 Hz for `set()` calls

### Safety Defaults
- **Torque Rate Limit**: 990 Nm/s
- **Collision Thresholds**: [100.0] * 7 (joints), [100.0] * 6 (Cartesian)

---

## See Also

- **README.md**: Overview and quick start
- **USAGE_GUIDE.md**: Comprehensive usage guide with examples
- **examples/**: Example scripts for common tasks
- **docs/**: Auto-generated API documentation

---

## Version

Current version: 0.1.0

Last updated: November 2024
