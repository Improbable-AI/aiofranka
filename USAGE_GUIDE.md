# aiofranka Usage Guide

Comprehensive guide for using aiofranka to control Franka Emika robots.

## Table of Contents

1. [Getting Started](#getting-started)
2. [Controller Modes](#controller-modes)
3. [Advanced Topics](#advanced-topics)
4. [Best Practices](#best-practices)
5. [Troubleshooting](#troubleshooting)
6. [Real-World Examples](#real-world-examples)

## Getting Started

### First Time Setup

Before using aiofranka for the first time, you need to unlock the robot brakes and activate FCI:

```python
from aiofranka.client import FrankaLockUnlock

# Connect and unlock robot
client = FrankaLockUnlock(
    hostname="172.16.0.2",  # Your robot's IP
    username="admin",
    password="admin"
)

# Unlock brakes and activate FCI
client.run(
    unlock=True,      # Open brakes
    fci=True,         # Activate Franka Control Interface
    persistent=True,  # Keep control token active
)
```

**Important**: You only need to do this once per power cycle unless another user takes control.

### Basic Control Script

```python
import asyncio
import numpy as np
from aiofranka import RobotInterface, FrankaController

async def main():
    # Create robot interface
    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)
    
    # Start control loop
    await controller.start()
    
    try:
        # Your control code here
        await controller.move()  # Move to home position
        
    finally:
        # Always stop gracefully
        await controller.stop()

if __name__ == "__main__":
    asyncio.run(main())
```

## Controller Modes

### 1. Impedance Control (Joint Space)

Best for: Joint-space motions, compliant behavior, trajectory following

```python
# Switch to impedance mode
controller.switch("impedance")

# Set gains (per-joint)
controller.kp = np.ones(7) * 80.0  # Position stiffness [Nm/rad]
controller.kd = np.ones(7) * 4.0   # Damping [Nm⋅s/rad]

# Set update frequency
controller.set_freq(50)  # 50 Hz

# Send joint position commands
for i in range(200):
    # Compute target (e.g., sinusoidal)
    delta = np.sin(i / 50.0 * np.pi) * 0.1
    target = controller.initial_qpos + delta
    
    await controller.set("q_desired", target)
```

**Gain Tuning Guide**:
- Start conservative: `kp = 20-40`, `kd = 2-4`
- Increase `kp` for stiffer response (up to 200)
- Increase `kd` if oscillations occur (typically `kd = sqrt(kp)/2`)
- Lower gains near singularities or obstacles
- Per-joint tuning: joints 1-4 can handle higher gains than 5-7

**When to Use**:
- ✅ Following joint-space trajectories
- ✅ Learning from demonstration (joint space)
- ✅ Compliant interaction tasks
- ❌ Cartesian straight-line motions (use OSC)
- ❌ Force control (use OSC or torque mode)

### 2. Operational Space Control (Task Space)

Best for: Cartesian motions, end-effector tracking, teleoperation

```python
# Switch to OSC mode
controller.switch("osc")

# Set Cartesian gains
# Format: [x, y, z, roll, pitch, yaw]
controller.ee_kp = np.array([300, 300, 300, 1000, 1000, 1000])
controller.ee_kd = np.ones(6) * 10.0

# Set null-space gains (keeps robot away from limits)
controller.null_kp = np.ones(7) * 10.0
controller.null_kd = np.ones(7) * 1.0

# Set update frequency
controller.set_freq(50)

# Send end-effector pose commands
for i in range(200):
    # Get current desired pose
    desired_ee = controller.ee_desired.copy()
    
    # Modify position (keep orientation)
    delta_y = np.sin(i / 50.0 * np.pi) * 0.1
    desired_ee[:3, 3] = controller.initial_ee[:3, 3] + [0, delta_y, 0]
    
    await controller.set("ee_desired", desired_ee)
```

**End-Effector Pose Format**:
```python
# 4x4 homogeneous transformation matrix
ee_pose = np.eye(4)
ee_pose[:3, :3] = rotation_matrix  # 3x3 rotation (SO(3))
ee_pose[:3, 3] = position_vector   # 3x1 position [x, y, z]

# Example: position only
from scipy.spatial.transform import Rotation as R

ee_pose = np.eye(4)
ee_pose[:3, :3] = R.from_euler('xyz', [0, 0, 45], degrees=True).as_matrix()
ee_pose[:3, 3] = [0.5, 0.0, 0.4]  # x, y, z in meters
```

**Gain Tuning Guide**:
- Translation stiffness (`ee_kp[:3]`): 100-500 N/m
- Rotation stiffness (`ee_kp[3:]`): 500-2000 Nm/rad
- Generally: rotation stiffness > translation stiffness
- Null-space gains: 5-20 Nm/rad (keeps robot configuration stable)

**When to Use**:
- ✅ Cartesian straight-line motions
- ✅ End-effector tracking (camera servo, teleoperation)
- ✅ Task-space impedance (contact tasks)
- ✅ Force control in Cartesian space
- ❌ Fast joint-space motions (use impedance)
- ❌ When Jacobian is poorly conditioned (near singularities)

### 3. Direct Torque Control

Best for: Custom controllers, learning-based control, advanced users

```python
# Switch to torque mode
controller.switch("torque")

# Compute your own torques
controller.set_freq(50)

for i in range(200):
    # Get current state
    state = controller.state
    q = state['qpos']
    dq = state['qvel']
    mm = state['mm']  # Mass matrix
    
    # Compute control torques
    # Example: Simple PD control
    kp = 80.0
    kd = 4.0
    q_desired = controller.initial_qpos
    
    torque = kp * (q_desired - q) - kd * dq
    
    # Set torques directly (thread-safe)
    with controller.state_lock:
        controller.torque = torque
    
    await asyncio.sleep(1.0 / 50.0)
```

**Safety Notes**:
- ⚠️ **Most dangerous mode** - no built-in limits!
- Torque rate limiting still active (`torque_diff_limit = 990 Nm/s`)
- Disable rate limiting: `controller.clip = False` (NOT recommended)
- Test in simulation first: `robot = RobotInterface(None)`
- Monitor torques: `max |tau| < 87 Nm` for joints 1-4, `< 12 Nm` for joints 5-7

**When to Use**:
- ✅ Implementing custom control algorithms
- ✅ Learning-based control (RL, IL)
- ✅ Research requiring full control authority
- ❌ General applications (use impedance/OSC)
- ❌ Without thorough testing and safety measures

## Advanced Topics

### Thread-Safe State Access

The control loop runs at 1kHz in a background thread. State is shared between threads:

```python
# Reading state (always safe)
state = controller.state
qpos = state['qpos']

# Modifying controller parameters (use lock!)
with controller.state_lock:
    controller.kp = new_gains
    controller.kd = new_damping
```

**What needs locking**:
- ✅ `controller.kp`, `controller.kd`, `controller.ee_kp`, etc. (gains)
- ✅ `controller.q_desired`, `controller.ee_desired` (if setting directly)
- ❌ `controller.state` (read-only property)
- ❌ `controller.type`, `controller.running` (atomic)

### Rate-Limited Updates

Use `set()` for automatic rate limiting:

```python
# Set update frequency
controller.set_freq(50)  # 50 Hz

# These calls automatically sleep to maintain 50 Hz
for i in range(100):
    target = compute_target(i)
    await controller.set("q_desired", target)
    # No manual sleep needed!
```

**How it works**:
1. First `set()` call: sleeps for `1/freq` seconds, then sets value
2. Subsequent calls: sleep until next scheduled update time
3. Compensates for computation time automatically
4. Per-attribute timing (can set different attrs at different rates)

**Common pattern**:
```python
controller.set_freq(50)

# Run for 4 seconds at 50 Hz (200 iterations)
for i in range(200):
    # Your computation here
    target = compute_target(i)
    
    # Automatically maintains 50 Hz
    await controller.set("q_desired", target)
```

### Smooth Trajectory Execution

Use `move()` for automatic trajectory generation:

```python
# Move to target with smooth, time-optimal trajectory
target = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
await controller.move(target)

# Customize trajectory limits (edit controller.py)
inp.max_velocity = np.ones(7) * 5       # Slower motion
inp.max_acceleration = np.ones(7) * 2.5  # Gentler acceleration
inp.max_jerk = np.ones(7) * 0.5         # Smoother motion
```

### Data Collection

Collect synchronized data at high frequency:

```python
# Prepare data structures
logs = {
    'qpos': [],
    'qvel': [],
    'ctrl': [],
    'qdes': [],
    'ee': [],
    'timestamp': []
}

controller.set_freq(100)  # 100 Hz logging

import time
start_time = time.time()

for i in range(1000):  # 10 seconds at 100 Hz
    # Log data
    state = controller.state
    logs['qpos'].append(state['qpos'].copy())
    logs['qvel'].append(state['qvel'].copy())
    logs['ctrl'].append(state['last_torque'].copy())
    logs['ee'].append(state['ee'].copy())
    logs['qdes'].append(controller.q_desired.copy())
    logs['timestamp'].append(time.time() - start_time)
    
    # Send command
    await controller.set("q_desired", compute_target(i))

# Convert to numpy arrays
for key in logs:
    logs[key] = np.array(logs[key])

# Save
np.savez("experiment_data.npz", **logs)
```

### Simulation Mode

Test your code without hardware:

```python
# None = simulation mode
robot = RobotInterface(None)
controller = FrankaController(robot)

await controller.start()

# Same code works in simulation!
await controller.move()
controller.switch("impedance")
# ... etc ...

await controller.stop()
```

**Differences from real robot**:
- No latency or network delays
- Perfect model (no friction, compliance, backlash)
- No safety limits or collision detection
- Visual feedback via MuJoCo viewer
- Useful for algorithm development and debugging

## Best Practices

### 1. Always Use Try-Finally

Ensure robot stops even if error occurs:

```python
async def main():
    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)
    
    await controller.start()
    
    try:
        # Your control code
        await controller.move()
        controller.switch("impedance")
        # ... etc ...
        
    except KeyboardInterrupt:
        print("Interrupted by user")
        
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        # Always stop
        await controller.stop()
```

### 2. Start Conservative

Begin with low gains and gradually increase:

```python
# Start conservative
controller.kp = np.ones(7) * 20.0
controller.kd = np.ones(7) * 2.0

# Test motion
await controller.move()

# If stable, increase gains
controller.kp *= 2
controller.kd *= 1.5
```

### 3. Monitor Safety

Add safety checks in your control loop:

```python
async def safe_control():
    MAX_TORQUE = 80.0  # Nm
    MAX_VEL = 2.0      # rad/s
    
    for i in range(1000):
        state = controller.state
        
        # Check torques
        if np.any(np.abs(state['last_torque']) > MAX_TORQUE):
            print("High torque detected!")
            await controller.stop()
            break
        
        # Check velocities
        if np.any(np.abs(state['qvel']) > MAX_VEL):
            print("High velocity detected!")
            await controller.stop()
            break
        
        # Send command
        await controller.set("q_desired", target)
```

### 4. Test Connection Quality

Always test before experiments:

```python
await controller.start()
await controller.test_connection()  # Runs for 5 seconds

# Check output:
# Frequency: 1000.2 Hz (good!)
# Jitter: 0.040 ms (good!)
# If frequency < 990 Hz or jitter > 1 ms, investigate!
```

### 5. Use Appropriate Update Rates

| Task | Recommended Frequency |
|------|----------------------|
| Impedance control | 20-100 Hz |
| OSC control | 50-200 Hz |
| Teleoperation | 50-100 Hz |
| Data collection | 100-500 Hz |
| Trajectory tracking | 50-100 Hz |

Higher frequency = more responsive but more CPU usage.

## Troubleshooting

### Problem: "Error requesting control token"

**Cause**: Another user/program has control.

**Solution**:
```python
# Option 1: Wait for release
client.run(unlock=True, wait=True, fci=True, persistent=True)

# Option 2: Force take control (use with caution!)
client.run(unlock=True, force=True, fci=True, persistent=True)

# Option 3: Check Desk interface and release manually
```

### Problem: Robot triggers safety stop

**Causes**:
1. Torque rate too high
2. Torques exceed limits
3. Joint position/velocity limits
4. Collision detected

**Solutions**:
```python
# 1. Increase damping
controller.kd *= 2

# 2. Decrease stiffness
controller.kp /= 2

# 3. Smooth commands
controller.set_freq(50)  # Don't update too fast

# 4. Check for discontinuities
# Bad:
await controller.set("q_desired", far_away_target)

# Good:
await controller.move(far_away_target)  # Uses smooth trajectory
```

### Problem: Jerky motion

**Causes**:
1. No rate limiting (commands sent too fast)
2. Low damping
3. Discontinuous commands

**Solutions**:
```python
# 1. Use rate limiting
controller.set_freq(50)

# 2. Increase damping
controller.kd = np.ones(7) * 6.0  # Higher damping

# 3. Smooth commands
delta = np.sin(cnt / 50.0 * np.pi) * 0.1  # Smooth sinusoid
```

### Problem: Control loop frequency drops

**Causes**:
1. Heavy computation in main loop
2. Network latency
3. System load

**Solutions**:
```python
# 1. Move computation outside control loop
precomputed_targets = [compute_target(i) for i in range(100)]
for target in precomputed_targets:
    await controller.set("q_desired", target)

# 2. Check network
# Run: ping 172.16.0.2
# Should be < 1ms latency

# 3. Reduce system load
# Close unnecessary programs
# Use dedicated control computer
```

### Problem: Simulation vs real discrepancies

**Causes**:
- Friction in real robot
- Compliance/flexibility
- Model inaccuracies
- Network latency

**Solution**: Tune gains separately for sim and real:

```python
if robot.real:
    controller.kp = np.ones(7) * 100.0  # Real robot
else:
    controller.kp = np.ones(7) * 80.0   # Simulation
```

## Real-World Examples

### Example 1: Pick and Place

```python
async def pick_and_place():
    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)
    
    await controller.start()
    
    try:
        # Switch to OSC for Cartesian control
        controller.switch("osc")
        controller.ee_kp = np.array([300, 300, 300, 1000, 1000, 1000])
        controller.set_freq(50)
        
        # Approach position
        approach = np.eye(4)
        approach[:3, :3] = R.from_euler('xyz', [180, 0, 0], degrees=True).as_matrix()
        approach[:3, 3] = [0.5, 0.0, 0.3]  # Above object
        await move_to_pose(controller, approach)
        
        # Descend
        pick = approach.copy()
        pick[2, 3] = 0.05  # Lower z
        await move_to_pose(controller, pick)
        
        # Grasp (TODO: add gripper control)
        await asyncio.sleep(0.5)
        
        # Lift
        await move_to_pose(controller, approach)
        
        # Place position
        place = approach.copy()
        place[0, 3] = 0.3  # Different x
        await move_to_pose(controller, place)
        
        # Lower
        place_low = place.copy()
        place_low[2, 3] = 0.05
        await move_to_pose(controller, place_low)
        
        # Release (TODO: add gripper control)
        await asyncio.sleep(0.5)
        
        # Return home
        await controller.move()
        
    finally:
        await controller.stop()

async def move_to_pose(controller, target_pose, duration=2.0):
    """Move to target pose over specified duration."""
    steps = int(duration * 50)  # 50 Hz
    start_pose = controller.ee_desired.copy()
    
    for i in range(steps):
        alpha = i / steps
        # Linear interpolation (simple, not optimal)
        interp_pose = np.eye(4)
        interp_pose[:3, 3] = (1 - alpha) * start_pose[:3, 3] + alpha * target_pose[:3, 3]
        interp_pose[:3, :3] = target_pose[:3, :3]  # Keep target orientation
        
        await controller.set("ee_desired", interp_pose)
```

### Example 2: Compliance Control

```python
async def compliant_interaction():
    """Low-stiffness control for safe physical interaction."""
    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)
    
    await controller.start()
    
    try:
        controller.switch("impedance")
        
        # Very low stiffness for compliance
        controller.kp = np.ones(7) * 10.0   # Low stiffness
        controller.kd = np.ones(7) * 3.0    # Moderate damping
        
        controller.set_freq(50)
        
        # Hold position but allow easy deflection
        target = controller.initial_qpos
        
        for i in range(500):  # 10 seconds
            # Monitor force (via torque)
            state = controller.state
            torque = state['last_torque']
            
            if np.any(np.abs(torque) > 20):
                print("External force detected!")
                # Robot will deflect due to low stiffness
            
            await controller.set("q_desired", target)
            
    finally:
        await controller.stop()
```

### Example 3: Learning Data Collection

```python
async def collect_demonstrations():
    """Collect teleoperation demonstrations for learning."""
    import pyspacemouse
    
    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)
    
    await controller.start()
    
    # Setup spacemouse
    success = pyspacemouse.open()
    if not success:
        print("Failed to connect spacemouse")
        return
    
    try:
        controller.switch("osc")
        controller.ee_kp = np.array([300, 300, 300, 1000, 1000, 1000])
        controller.set_freq(50)
        
        # Data collection
        episode = 0
        while episode < 10:
            input(f"Press Enter to start episode {episode}...")
            
            trajectory = {
                'qpos': [],
                'qvel': [],
                'ee_pose': [],
                'actions': [],  # Spacemouse commands
                'timestamp': []
            }
            
            start_time = time.time()
            
            for i in range(500):  # 10 seconds at 50 Hz
                # Read spacemouse
                event = pyspacemouse.read()
                translation = np.clip(np.array([event.x, event.y, event.z]) * 0.002, -0.002, 0.002)
                rotation = np.array([0, 0, -event.yaw]) * 0.5
                
                # Apply to current pose
                current_ee = controller.ee_desired.copy()
                current_ee[:3, 3] += translation
                rot_delta = R.from_euler('xyz', rotation, degrees=True).as_matrix()
                current_ee[:3, :3] = rot_delta @ current_ee[:3, :3]
                
                # Log data
                state = controller.state
                trajectory['qpos'].append(state['qpos'].copy())
                trajectory['qvel'].append(state['qvel'].copy())
                trajectory['ee_pose'].append(current_ee.copy())
                trajectory['actions'].append(np.concatenate([translation, rotation]))
                trajectory['timestamp'].append(time.time() - start_time)
                
                # Send command
                await controller.set("ee_desired", current_ee)
            
            # Save episode
            for key in trajectory:
                trajectory[key] = np.array(trajectory[key])
            np.savez(f"demo_ep{episode}.npz", **trajectory)
            
            episode += 1
            
    finally:
        await controller.stop()
        pyspacemouse.close()
```

### Example 4: System Identification

```python
async def system_identification():
    """Collect data for dynamics identification."""
    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)
    
    await controller.start()
    
    # Test different gains
    kp_values = [16, 32, 64, 128]
    kd_values = [2, 4, 8, 16]
    
    for kp in kp_values:
        for kd in kd_values:
            print(f"Testing kp={kp}, kd={kd}")
            
            # Move to start position
            await controller.move([0, 0, 0.3, -1.57, 0, 1.57, -0.785])
            
            # Set gains
            controller.switch("impedance")
            controller.kp = np.ones(7) * kp
            controller.kd = np.ones(7) * kd
            controller.set_freq(50)
            
            # Collect data
            logs = {
                'qpos': [],
                'qvel': [],
                'qdes': [],
                'ctrl': [],
                'timestamp': []
            }
            
            start_time = time.time()
            
            # Sinusoidal reference
            for i in range(200):  # 4 seconds
                state = controller.state
                logs['qpos'].append(state['qpos'].copy())
                logs['qvel'].append(state['qvel'].copy())
                logs['ctrl'].append(state['last_torque'].copy())
                logs['timestamp'].append(time.time() - start_time)
                
                # Sinusoidal motion on joint 3
                delta = np.sin(i / 50.0 * np.pi) * 0.2
                target = controller.initial_qpos.copy()
                target[2] += delta
                logs['qdes'].append(target.copy())
                
                await controller.set("q_desired", target)
            
            # Save data
            for key in logs:
                logs[key] = np.array(logs[key])
            np.savez(f"sysid_K{kp}_D{kd}.npz", **logs)
            
            await asyncio.sleep(1.0)
    
    await controller.stop()
```

## Further Resources

- **libfranka documentation**: https://frankaemika.github.io/docs/
- **MuJoCo documentation**: https://mujoco.readthedocs.io/
- **Ruckig**: https://github.com/pantor/ruckig
- **asyncio tutorial**: https://docs.python.org/3/library/asyncio.html

## Contributing

Found a bug or have a suggestion? Please open an issue on GitHub!

## Support

For questions or issues:
1. Check this guide
2. Check the API documentation
3. Open an issue on GitHub
4. Contact the maintainers

Happy robot controlling! 🤖
