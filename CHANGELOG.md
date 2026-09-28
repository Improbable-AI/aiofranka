# Changelog

## 0.5.1 - 2026-09-28

### Highlights

- On Apple Silicon Macs with macOS 15 or newer, depend on [pylibfranka-macos](https://pypi.org/project/pylibfranka-macos/), an unofficial macOS build of pylibfranka, so `pip install aiofranka` works without building pylibfranka from source.

## 0.5.0 - 2026-09-28

### Breaking changes

- Replace the examples with minimal async mode scripts. The trajectory collection, plotting, system-identification, SpaceMouse teleoperation, and Robotiq examples moved to the `research-scripts` branch.

### Highlights

- Support macOS on Apple Silicon. PyPI has no macOS build of pylibfranka, so on macOS aiofranka does not depend on it; install pylibfranka from [younghyopark/libfranka](https://github.com/younghyopark/libfranka/tree/macos-support) first, as described in the README. Without it, connecting to a robot, including starting the server, fails with the install command.
- On macOS, run every control loop (async mode, server, v2 RT thread, and `rt-benchmark`) with the QoS class `USER_INTERACTIVE`, which libfranka's busy-waiting needs to stay on a performance core, and skip the Linux-only CPU pinning, `SCHED_FIFO`, and `mlockall` instead of failing.
- Report the robot-side view in `rt-benchmark`: the response time from `readOnce` to `writeOnce` against a 300 us budget, skipped robot states, and dropped commands with their likely cause.
- Warm up the first `mj_jacSite`, `mj_fullM`, and `data.site()` calls when creating a `RobotInterface`, so the first control cycles no longer miss their deadline.
- Release the torque controller in `RobotInterface.stop()`, so the stopped motion is cleaned up right away instead of at interpreter exit.
- Remove the per-step torque-clipping diagnostic prints. Torque rate and torque limit clipping still apply.
- Add minimal examples that move a joint, hold with joint impedance, hold with OSC, and stream zero torque. Each runs in MuJoCo without a robot.

## 0.4.0 - 2026-08-22

### Breaking changes

- Require Python 3.10+ and MuJoCo 3.10+; Python 3.8 and 3.9 are no longer supported.
- `aiofranka gravcomp` now leaves the robot unlocked with FCI active after stopping. Run `aiofranka lock` when finished.
- Remove unused bundled cube and extrinsic sample assets.

### Highlights

- Restore clean-install compatibility with modern MuJoCo by migrating every mass-matrix calculation to the current `mj_fullM` API.
- Improve 1 kHz real-time control with preallocated buffers, CPU affinity and `SCHED_FIFO` support, jitter telemetry, second-generation remote/server controllers, and new benchmark and tuning tools.
- Make server shutdown, restart, and control-token recovery more reliable, with actionable server-failure reporting.
- Restore `aiofranka start-server` and add `home`, Robotiq `gripper`, and `rt-benchmark` commands, plus an optional gravity-compensation `/qpos` endpoint.
- Add synchronous background Robotiq control through `GripperRemoteController`.
- Add new trajectory collection, plotting, and system-identification examples.
