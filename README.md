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

**aiofranka** is an asyncio-based Python library for controlling Franka Emika robots. It provides a high-level, asynchronous interface that combines **`pylibfranka`** for official low-level control interface (1kHz torque control), **`MuJoCo`** for kinematics/dynamics computation, **`Ruckig`** for  smooth trajectory generation.

The library is designed for research applications requiring precise, real-time control with minimal latency and maximum flexibility.

## Installation

aiofranka requires Python 3.10 or newer.

Make sure you can access Franka Desk GUI from your machine's browser by typing in the robot's IP (e.g. 172.16.0.2). Then, install:


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

There are two ways to use aiofranka:

### Option A: Server mode

Run the 1kHz control loop in a subprocess. Your scripts use a simple sync API — no `async`/`await` needed.

- **No `async`/`await`** — plain Python scripts, easy to integrate with existing codebases
- **Process-isolated** — heavy computation (policy inference, camera processing) can't starve the 1kHz loop
- **Automatic lifecycle** — server subprocess starts with your script and stops when it exits

```python
import numpy as np
import aiofranka
from aiofranka import FrankaRemoteController

# 1. Unlock the robot (opens brakes + activates FCI)
aiofranka.unlock()

# 2. Create controller and start server subprocess
controller = FrankaRemoteController()
controller.start()

# 3. Use the robot
controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])

controller.switch("impedance")
controller.kp = np.ones(7) * 80.0
controller.kd = np.ones(7) * 4.0
controller.set_freq(50)

for cnt in range(100):
    state = controller.state
    delta = np.sin(cnt / 50.0 * np.pi) * 0.1
    controller.set("q_desired", delta + controller.initial_qpos)

# 4. Stop server and lock robot
controller.stop()
aiofranka.lock()
```

The server subprocess terminates automatically when your script exits (Ctrl+C, crash, etc.), so it won't leave orphaned processes. `controller.start()` checks that the robot is unlocked and FCI is active before launching — if not, it prints a status summary and exits cleanly.


### Option B: Async mode

Run the 1kHz control loop in-process using asyncio — everything in a single script.

- **Single script** — no separate server process, simpler deployment
- **Direct access** — no IPC overhead, full control over the event loop
- **Requires async discipline** — any blocking call >1ms after `controller.start()` will cause `communication_constraints_violation` (see [Async Mode Guide](docs/ASYNC_MODE.md))

```python
import asyncio
import numpy as np
from aiofranka import RobotInterface, FrankaController

async def main():
    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)

    await controller.start()
    await controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])

    controller.switch("impedance")
    controller.kp = np.ones(7) * 80.0
    controller.kd = np.ones(7) * 4.0
    controller.set_freq(50)

    for cnt in range(100):
        delta = np.sin(cnt / 50.0 * np.pi) * 0.1
        init = controller.initial_qpos
        await controller.set("q_desired", delta + init)

    await controller.stop()

if __name__ == "__main__":
    asyncio.run(main())
```

## CLI Reference

The CLI handles robot setup, server lifecycle, and diagnostics.

```
aiofranka start-server [--ip IP] [--no-home]  Start the control server
aiofranka unlock   [--ip IP]              Unlock joints + activate FCI
aiofranka lock     [--ip IP]              Lock joints + deactivate FCI
aiofranka gravcomp [--ip IP] [--damping]  Gravity compensation (freedrive)
aiofranka home     [--ip IP]              Move the robot to its home pose
aiofranka status   [--ip IP]              Show robot & server status
aiofranka stop     [--ip IP]              Stop a running server
aiofranka mode     [--ip IP] [--set MODE] View/change operating mode
aiofranka config   [--ip IP] [--mass M]   View/set end-effector config
aiofranka selftest [--ip IP] [--force]    Run safety self-tests
aiofranka log      [-n LINES] [-f]        View server logs
aiofranka gripper  --open|--close          Control the Robotiq gripper
aiofranka rt-benchmark [--duration SEC]    Benchmark the 1 kHz control loop
```

### `unlock` / `lock`

Unlock opens the brakes and activates FCI so the robot is ready for torque control. Lock does the reverse. Credentials are prompted on first use and saved to `~/.aiofranka/config.json`.

```bash
# Unlock before running your script
aiofranka unlock

# Lock when you're done
aiofranka lock
```

You can also do this from Python:

```python
import aiofranka
aiofranka.unlock()   # opens brakes + activates FCI
# ... run your control script ...
aiofranka.lock()     # closes brakes + deactivates FCI
```

### `gravcomp`

Runs gravity compensation mode in the foreground. The robot is freely movable by hand. Press Ctrl+C to stop control; the joints remain unlocked with FCI active. Run `aiofranka lock` when finished.

```bash
aiofranka gravcomp                  # default: zero damping
aiofranka gravcomp --damping 2.0    # add velocity damping
```

### `status`

Shows robot state (joints locked/unlocked, FCI active/inactive, control token, self-test status, end-effector configuration) and server status if running.

```bash
aiofranka status
```

### `stop`

Sends a shutdown signal to a running server process. The server deactivates FCI, locks joints, and releases the control token.

```bash
aiofranka stop
```

### `mode`

View or change the operating mode. `Execution` is needed for FCI control. `Programming` enables freedrive via the pilot interface button near the end-effector.

```bash
aiofranka mode                  # view current mode
aiofranka mode --set Execution  # switch to FCI mode
```

### `config`

View or set the end-effector configuration (mass, center of mass, inertia, flange-to-EE transform). Changes are applied via the Franka Desk API.

```bash
aiofranka config                                # view current config
aiofranka config --mass 0.5 --com 0,0,0.03      # set mass + CoM
aiofranka config --translation 0,0,0.1           # set flange-to-EE offset
```

### `selftest`

Run the robot's safety self-tests. The robot will lock joints during the test.

```bash
aiofranka selftest          # run if due
aiofranka selftest --force  # run even if not due
```

### `log`

View recent server log entries from `~/.aiofranka/server.log`.

```bash
aiofranka log              # last 20 lines
aiofranka log -n 100       # last 100 lines
aiofranka log -f           # follow (like tail -f)
```

### Common flags

Most commands accept these flags:

| Flag | Description |
|------|-------------|
| `--ip IP` | Robot IP address (default: last used, or `172.16.0.2`) |
| `--username USER` | Franka Desk web UI username (default: saved or prompted) |
| `--password PASS` | Franka Desk web UI password (default: saved or prompted) |
| `--protocol http\|https` | Web UI protocol (default: `https`) |

## Core Concepts

### Server Mode vs Async Mode

|                          | Server mode                        | Async mode                          |
|--------------------------|------------------------------------|-------------------------------------|
| **Class**                | `FrankaRemoteController`           | `FrankaController`                  |
| **API style**            | Synchronous (plain Python)         | `async`/`await`                     |
| **1kHz loop runs in**    | Subprocess (auto-managed)          | Your process (asyncio task)         |
| **Blocking calls OK?**   | Yes — can't starve the loop        | No — must stay under ~1ms           |
| **State reads**          | Shared memory (zero-copy)          | Direct attribute access             |
| **Commands**             | ZMQ IPC (msgpack)                  | Direct method calls                 |
| **Setup**                | `unlock()` + `ctrl.start()`        | Single script                       |
| **Best for**             | Heavy workloads (GPU inference, vision pipelines) | Lightweight scripts, rapid prototyping |

### Rate Limiting

Use `set_freq()` to enforce strict timing for command updates:

```python
controller.set_freq(50)  # Set 50Hz update rate

# This will automatically sleep to maintain 50Hz timing
for i in range(100):
    controller.set("q_desired", compute_target())
```


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

controller.set("q_desired", target_joint_angles)
```

**Use case**: Precise joint-space motions, compliant behavior


### 2. Operational Space Control (Task Space)

Controls end-effector pose in Cartesian space:

```python
controller.switch("osc")
controller.ee_kp = np.array([300, 300, 300, 1000, 1000, 1000])  # [xyz, rpy]
controller.ee_kd = np.ones(6) * 10.0

desired_ee = np.eye(4)  # 4x4 homogeneous transform
desired_ee[:3, 3] = [0.4, 0.0, 0.5]  # Position
controller.set("ee_desired", desired_ee)
```

**Use case**: Cartesian trajectories, end-effector tracking

## License

aiofranka's original code is available under the MIT License; see [LICENSE](LICENSE). The bundled FR3 model and meshes retain their upstream Apache-2.0 and BSD-3-Clause terms in [aiofranka/model/LICENSE](aiofranka/model/LICENSE).

## Citation

If you use this library in your research, please cite:

```bibtex
@software{aiofranka,
  author = {Park, Younghyo},
  title = {aiofranka: Asyncio-based Franka Robot Control},
  year = {2025},
  url = {https://github.com/Improbable-AI/aiofranka}
}
```

## Acknowledgments

- Built on [libfranka](https://frankarobotics.github.io/docs/) by Franka Emika
- Uses [MuJoCo](https://mujoco.org/) physics engine
- Trajectory generation with [Ruckig](https://github.com/pantor/ruckig)
