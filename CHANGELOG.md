# Changelog

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
- Add torque-clipping diagnostics and new trajectory collection, plotting, and system-identification examples.
