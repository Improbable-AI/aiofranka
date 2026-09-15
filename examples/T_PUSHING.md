# T-pushing collection

`14_collect_t_pushing.py` uses the newest completed camera calibration at startup and pins it for the run. Generate a calibration with `12_camera_calibration.py`, or supply an existing result with `--calibration /path/to/calibration.json`. Recorded calibration sessions and datasets are excluded from Git; a fresh checkout does not contain a fitted camera transform. The removed calibration attachment's mounting transform is **not** applied to the pushing stick.

## Installation

From the repository root, in the environment used for collection:

```bash
python -m pip install -e '.[t-pushing]'
python -m pip install 'numpy>=1.24' 'pyzmq>=25' 'click>=8' 'pyrealsense2>=2.50'
python -m pip install --no-deps -e /path/to/pycaas
```

The optional extra installs OpenCV contrib, AprilCube 0.3, and PySpaceMouse 2.x. Use your PyCAAS checkout path in the last command. These commands target PyCAAS 0.1.0: its dependencies are installed explicitly, substituting OpenCV contrib for `opencv-python`. Both wheels supply `cv2`, so use only the contrib wheel in this environment. The RealSense package is needed on the machine running the camera daemon.

MuJoCo image exports use EGL by default and require a working EGL driver. `MUJOCO_GL=osmesa` selects software rendering when OSMesa is installed. The isolated rendering integration test explicitly requires EGL. Run the offline checks with `python -m unittest discover -s tests -p 'test_*.py'`; no robot, camera, or SpaceMouse is opened.

Collection records two goal poses, A and B, and alternates demonstrations A→B, B→A, A→B. The robot goes home and starts the next approach automatically after each episode.

The confirmed stick tip is `[0, 0, 0.161]` meters from `attachment_site`. Each direction reuses a fixed tip start **7.5 cm outside the recorded source T outline**: saved A for A→B, saved B for B→A. The start uses the outline vertex farthest away from the destination along the line between the saved pose centers, then offsets outward by 7.5 cm. Tied vertices use a stable first choice. This gives the requested 5–10 cm clearance without random sampling. If A and B have the same XY center, the outward direction is robot-base −Y. Live tracking noise and the current block position do not shift the point for a selected direction.

`--start-clearance` changes this fixed outline clearance (meters, default 0.075). It measures the tip's XY point relative to the saved source outline. Tip height stays at the fixed table height + 0.020 meters, with the stick pointing down. Raw SpaceMouse X/Y control robot-base X/Y; Z and all rotation inputs are ignored.

## T-block pose detection

T tracking uses `aprilcube.detector()` with the library defaults, then `process_frame()` for each PyCAAS image. AprilCube owns marker recovery, PnP, outlier handling, optical flow, Kalman filtering, and prediction. The collector imposes no tag-count or reprojection-error threshold. A small compatibility adapter reshapes OpenCV 5 marker IDs to the column arrays AprilCube 0.3 expects; it does not change detections or pose decisions.

The calibrated `T_base_camera` is passed as AprilCube's `extrinsic`. `world_pose(result)` returns the object-to-base transform in meters. Camera records retain the native `predicted`, `n_tags`, and `n_inliers` fields, plus `aprilcube_T_camera_object_mm` for the library's original millimeter pose. Reprojection error is null when the native result does not supply a finite value. Native successful predictions are recorded and used as estimates, with their prediction flag preserved.

## SpaceMouse commands

At each 50 Hz update, the collector uses the existing OSC teleop rule with a fixed scale:

```python
target_xy = measured_tip_xy + clip(latest_mouse_xy, -1, 1) * 0.05
```

There is no XY workspace boundary. `--scale` defaults to **0.05 meters per input unit** on each axis, independently of OSC gains. Input `0.2` requests a 1 cm offset from the measured position; full input requests 5 cm per axis. Holding input keeps the target ahead as the robot moves. Releasing the cap removes that input offset on the next update. Z and downward orientation stay fixed. The scale is a position offset; actual velocity depends on OSC dynamics.

The shared `aiofranka/utils/spacemouse.py` helper drains queued reports through public PySpaceMouse reads before publishing the latest state. Every report is processed, including split axes and final button/axis releases. An input thread repeats this at nominal 1 kHz. A queue that cannot drain raises an error; stale input is zeroed after 0.2 seconds, and a reader stall raises after 0.1 seconds. The existing legacy axis convention is preserved. `--deadzone` defaults to zero, matching existing teleop commands.

The startup output, `run.json`, `setup.json`, and episode metadata record the actual XY scale and update rule.

## Control timing

The 50 Hz SpaceMouse target loop feeds a separate server that must exchange robot commands at 1 kHz. The collector uses this checkout's control package and prints its path at startup. It limits OpenBLAS, OpenMP, and MKL to one thread before loading the numerical libraries; the server inherits those settings.

Before opening torque control, the server sets CPU affinity/FIFO priority and runs 100 warmup iterations of MuJoCo, joint impedance, and OSC computations. Warmup uses a separate local state and a dummy torque sink: it sends no robot commands and preserves the live controller's mode, targets, and state. Both matrix inverse and pseudoinverse paths are warmed. Startup logs show warmup completion and computation timings; a warmup failure prevents torque startup.

On Intel hybrid CPUs the standard server prefers an allowed performance core. `AIOFRANKA_RT_CPU` can select another allowed core. Warmup and this core selection apply to the standard server used by this collector; `server_v2` has a separate startup path. This is process-level scheduling; it does not install a realtime kernel or change power/network settings.

On failure, the server reports the controller mode, completed step count, step timing, and the last available robot command-success rate. Step duration includes the wait for incoming robot state. A `communication_constraints_violation` indicates missed control communication deadlines; investigate host scheduling and network latency before restarting control. Short ping tests and offline math timings do not validate realtime performance.

## Offline inspection

These commands open no robot, camera, or HID device:

```bash
python examples/14_collect_t_pushing.py inspect
python examples/14_collect_t_pushing.py plan --resume /path/to/run
python examples/14_collect_t_pushing.py plan --goal-pose /path/to/run/setup.json --object-pose /path/to/episode/setup.json
```

`plan` takes a saved run or episode setup containing both goals and the fixed table height. A run setup previews A→B; an episode setup previews its saved direction. It inherits the saved start clearance, home joints, and tool offset, with explicit CLI overrides allowed. `--object-pose` supplies an example current pose (a 4×4 matrix or saved episode setup) to simulate resume direction selection. That pose selects A→B or B→A but does not shift the corresponding fixed start. The same geometry helper and IK compute collection and preview endpoints. This checks neither collisions nor intermediate path clearance.

## A physical run, when ready

Complete the installation above. Start the camera daemon with `pycaas start`, enable Franka FCI, and configure the actual stick payload separately.

```bash
python examples/14_collect_t_pushing.py collect --robot-ip 173.16.0.2 --condition ground
python examples/14_collect_t_pushing.py collect --robot-ip 173.16.0.2 --condition elevated
```

To append to an existing run after a stop, pass its directory:

```bash
python examples/14_collect_t_pushing.py collect --resume /path/to/data/t_pushing/RUN
```

Resume inherits the saved robot IP, condition, scale, fixed start clearance, tool geometry, home pose, gains, timing, and success threshold. It uses the run's copied calibration and target configuration, plus its saved goals A/B and table height, and skips goal capture. After moving home, it captures the current native AprilCube pose and selects the opposite destination: if the block matches A, push to B; if it matches B, push to A. Matching compares footprint overlap first, then XY distance when coverage is equal (including when neither goal overlaps). An exact tie selects A as the source. Previous episode outcomes and numbering do not determine this first resumed direction. The selected direction uses its fixed start from the recorded source geometry. There is no Enter prompt on resume.

Existing episode directories, including failed or unfinished ones, are preserved; numbering continues after the highest existing index. `--episodes N` collects N additional episodes. Explicit collection flags can override inherited settings; calibration, target config, and condition stay pinned to the run. Use a new run if the camera or table placement has changed. Old single-goal runs lack the required A/B pair: this collector reports that a new A/B run is needed when asked to resume one.

Each resumed invocation writes `resume_NNNN.json` with its effective settings, first episode, and outcome. The original `run.json`, goal references, setup, and earlier episodes remain unchanged. New episode setups identify their `run_attempt`. Earlier A/B runs that used sampled starts can resume: new episodes use the fixed 7.5 cm default (or explicit `--start-clearance`), and their metadata records the new initialization policy. Old sampling distances and seeds no longer apply. `inspect --resume RUN` checks the saved inputs and prints the next episode number without opening any devices; its direction is selected only during collection from the live pose.

If you relocate the dataset or checkout, override the saved absolute model paths with `--robot-xml /path/to/aiofranka/model/fr3.xml --stick-xml /path/to/assets/pocky/stick.xml`.

1. Place the T-block flat at A and press Enter, then repeat at B. Each goal uses the next valid native AprilCube pose. There is no extra sample window or motion, tilt, or tag-count gate. Table height is inferred once from A: the object's center Z minus 32 mm (half its 64 mm local-Y thickness). Both goals use that same fixed table height.
2. The robot moves directly to the default Franka joint pose. For a new run, place the block back at A and press Enter once to begin A→B. Capturing A followed by B leaves the block at B, so this initial placement is needed.
3. The camera measures the current T pose for recording. One joint-space move goes directly to the direction's fixed start at table height + 2 cm. It switches to OSC for SpaceMouse XY pushing.
4. An episode succeeds as soon as more than 85% of its destination's actual T-shaped area is covered. There is no dwell time or minimum frame count. This is intersection/goal area, not bounding-box overlap or IoU. After the success is saved, the destination switches: B→A follows A→B, then A→B again. The robot goes directly home and automatically approaches the next direction's fixed start. Leave the block where the previous demonstration ended; there is no manual reset or Enter prompt between episodes.

One run keeps its A/B pair and table height. Start another run for the other table condition. `q` cancels at a startup prompt; Ctrl+C interrupts at any time. `--episodes N` ends after N episode attempts. A 300-second episode timeout is saved as `timeout`, then the robot goes home and retries the same destination from the block's current pose. Only success switches destinations within a process. On resume, live pose matching selects the destination again.

All automatic positioning uses `robot.move(q)` in joint impedance mode. IK selects the start endpoint; interpolation happens in joint space, so the tip does not follow an exact vertical or straight Cartesian path. The sequence is episode end → home → fixed start, with no separate raise/lower waypoints. Each move starts its own 50 Hz command schedule and sends the exact final joint target; pauses between moves do not cause trajectory samples to be sent in a burst. A one-second pause follows each move. The terminal labels each episode's direction and prints its target and measured tip coordinates. `--home-q` selects the home posture.

Custom collision/path clearance checks, home-height rejection, approach-zone checks, and measured tip-height/orientation/workspace aborts have been removed. Choose the layout and home posture yourself. Normal controller limits remain in aiofranka/libfranka. The camera continues recording detected poses when the object tilts or lifts; this does not change the run's fixed table height. Goal coverage uses the T footprint projected into base XY.

When AprilCube returns no pose, the object pose and goal coverage are recorded as null. SpaceMouse control and camera/robot recording continue. Native valid results are used without an additional pose-age cutoff, and the same coverage value displayed and recorded determines success. There is no tracking-loss hold or timeout. Camera backend, recording, and controller errors still end the run and preserve partial data. Joint `move()` uses aiofranka's blocking API.

Useful options: `--calibration FILE`, `--output NEW_DIRECTORY`, `--episodes N`, `--scale METERS_PER_UNIT`, `--start-clearance METERS` (default 0.075), and `--stream rs_SERIAL_color`. The old `--start-min-distance`, `--start-distance`, and `--seed` flags have been removed.

## Saved data

Each run contains copied calibration and target configuration, `run.json`, `setup.json`, `goal_A.png`, `goal_B.png`, and `goal_A_detection.json` / `goal_B_detection.json`. Run `setup.json` stores `T_base_goals` with keys `A` and `B`, and `T_base_goal` as the initial destination B. Each `episode_XXXX` contains:

- `setup.json`: active `T_base_goal`, nominal `source_goal` / `target_goal` labels, `direction` (`A_to_B` or `B_to_A`), fixed `start_tip_base_m`, `start_initialization` policy, distance from the saved source outline, actual tracked starting pose, and joint approach plan. This is saved before the approach. A timeout retry may find the object partway between the nominal endpoints, while reusing the same fixed tip start.
- `episode.json`: outcome, condition, goal, fixed height, fixed start, geometry, and record counts.
- `states.jsonl`: measured joints, velocities, EE/tip poses, torques, Jacobian/mass matrix, commanded targets, SpaceMouse input, coverage, and camera-frame references at nominal 50 Hz.
- `camera.jsonl` and `rgb/*`: native-resolution frames, source timestamps, AprilCube pose results, native prediction flags, reprojection errors, and base-frame object poses, including frames with no pose.
- `video.mp4` and `video.json`: plain RGB video plus source timing and frame mapping, generated after collection stops.
- `aprilcube_overlay.mp4` and `aprilcube_overlay.json`: recorded AprilCube results drawn over the same images and playback timeline.

Images use JPEG quality 95 by default. `--image-format png` preserves lossless pixels; measured storage was about 4.6 GiB/minute at 1280×720 and 30 Hz. Pose detection always uses original camera pixels before encoding.

Images and robot states have separate timestamps. PyCAAS timestamps driver-read completion, not hardware exposure; the data does not claim hardware synchronization. Queue overflow or disk errors abort explicitly rather than silently dropping records. Reset and approach motions are excluded from demonstration episodes.

## Policy inputs and actions

The existing records contain the requested observation history, goal, and absolute XY target. All positions are in the robot base frame, in meters:

| Policy value | Saved source |
| --- | --- |
| Measured tip XY | `states.jsonl`: `tip_base_m[:2]` |
| Object XY | `states.jsonl`: `T_base_object[:2, 3]` |
| Object planar angle | `atan2(T_base_object[1, 0], T_base_object[0, 0])` |
| Goal XY and angle | Episode `setup.json`: `T_base_goal`, using the same translation and angle extraction |
| Action: commanded target tip XY | `states.jsonl`: `target_tip_base_m[:2]` |

Use the **episode's** `T_base_goal` for policy conditioning; it follows the active A/B destination. The run-level `T_base_goal` represents only the first intended destination, B.

The angle is the direction of the object's local +X axis projected into base XY. Use that same convention at training and inference; representing it as `(cos(theta), sin(theta))` avoids the wrap at ±π. Histories can use consecutive control rows, which pair each command with the robot state and latest native object estimate used for that command. Missing poses are null, and `tracking_valid` / `tracking_predicted` identify availability and native predictions. Handle missing observations when building histories; they are not zero-valued object positions.

Control rows are nominally 50 Hz, while the recorded camera rate is limited by AprilCube processing (about 6 Hz in the measured run). Repeated object poses across rows therefore represent the actual observations available to the collector. `robot_timestamp_s`, `camera_timestamp_s`, and `command_timestamp_s` preserve their different times; use those for alignment, rather than substituting a later camera observation into an earlier policy input.

## Video export

Like example 02, video encoding is deferred. The T collector already saves images through its recording worker during each episode; it adds no camera requests, overlay drawing, or MP4 encoding to the live control loop. After the robot server and camera reader stop, the default `--video` option encodes plain and overlay videos for newly recorded episodes, including partial episodes, one at a time. `--no-video` skips both exports.

MP4s use the existing example's OpenCV `mp4v` codec. Playback follows saved camera timestamps, holding images between arrivals on a 30 FPS output timeline; this does not create 30 distinct camera images per second. `video.json` records the timing and source-frame mapping. Full camera-rate capture would require decoupling acquisition from the detector; these videos use the images already recorded.

You can also export saved images later, without opening any devices:

```bash
python examples/t_pushing_video.py /path/to/data/t_pushing/RUN
python examples/t_pushing_video.py /path/to/data/t_pushing/RUN --overlay
```

Export skips episodes with no saved camera frames and leaves existing videos intact.

The overlay calls AprilCube's native `draw_result()` on the saved result, without running detection or pose fitting again. It shows RGB axes, the library's green bounding cuboid, tag count, reprojection error, and camera-frame translation, plus a tracked/predicted/no-pose label. The cuboid is AprilCube's bounding wireframe, not a T-shaped mesh. New camera records retain native `aprilcube_detections` corner arrays and `aprilcube_visible_faces` so the yellow detected-marker outlines can also be reproduced. Older recordings lack those corners: their overlays still show the saved pose, and label that corner observations were not recorded.
