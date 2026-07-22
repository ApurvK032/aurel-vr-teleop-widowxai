# WidowXAI + Meta Quest 3 Teleoperation: Clean Project Handoff

Updated: 2026-07-20

> **Current implementation note:** This file records the pre-build handoff and
> remains useful for its calibration and hardware findings, but its proposed
> architecture and baseline timing are historical. The user subsequently chose
> to follow the article/reference control path directly. The implemented source
> of truth is [README.md](README.md): the article/reference 200 Hz single-step
> decoupled IK, reference pose gains and caps, immediate commands, and direct
> MuJoCo `qpos` visualization. Earlier 50/100 Hz and 30 ms-horizon experiments
> remain historical observations, not the implemented control path.
> A gated 50%-gain physical demo is prepared, but the official Trossen driver
> supports Ubuntu/macOS rather than native Windows, so live output fails closed
> on this PC. See the README physical-demo section.

## Instructions For The Receiving Codex

This document is the source of truth for starting a **new, clean project** on a
Windows laptop. Do not assume that the previous Linux workspace or chat history
is available. Do not blindly reproduce the old repository structure; use the
verified facts, calibration data, safety limits, and lessons below to build a
smaller, testable system.

At the start of the new session:

1. Read this entire file before creating or changing code.
2. Summarize the current state, major risks, and proposed first milestone in no
   more than 10 bullets.
3. Inspect the Windows/WSL environment and available hardware/software.
4. Create a new Git repository for the clean implementation.
5. Begin with model tests and MuJoCo simulation. Do not command the physical arm
   until the user explicitly requests a hardware test and all safety gates pass.
6. Preserve a known-good baseline and put experimental solvers/transports behind
   configuration flags so they can be compared using the same input recording.

## Goal

Teleoperate a Trossen/WidowXAI follower arm with a Meta Quest 3 controller. The
motion should feel direct, smooth, intuitive, and safe. The previous wireless
system felt imperfect and had an estimated controller-to-physical-motion latency
of roughly 70-80 ms. The clean project should improve control quality first and
measure real latency correctly before claiming a reduction.

The new design is inspired by:

- Article: <https://aurelarnold.xyz/blog/vr-teleoperation-stack/>
- Reference implementation: <https://github.com/Dream-Machines-Robotics/vr-teleop-kit>

Decision: **port the useful control ideas, not the entire reference stack.** The
reference is especially relevant because its six arm-joint axes and dimensions
are extremely close to WidowXAI. However, its reported network numbers are
headset-to-workstation RTT, not full Quest tracking-to-physical-arm latency.

## Important Data Availability Warning

The old workspace contained important uncommitted files. A fresh clone of the
existing GitHub repositories will not reconstruct the final June 2026 working
state.

Verified remotes and committed revisions as of 2026-07-17:

| Component | Repository | Verified commit |
|---|---|---|
| Old experiment layer | <https://github.com/ApurvK032/xr-widowxai-teleop-experiments> | `15c7d82100b55a2bdb3844a6252bd6c200a09172` |
| Modified XR Python base | <https://github.com/sig-research/XRoboToolkit-Teleop-Sample-Python> | `74409fcacad0b8ede9d17f6e52c03752fe63dd2b` |
| Quest Unity client | <https://github.com/sig-research/XRoboToolkit-Unity-Client-Quest> | `85c6b0f4d0ad666578a3e6c126f80dd4116e2c60` |
| WidowXAI URDF | <https://github.com/TrossenRobotics/trossen_arm_description> | `21d8b360c211c2ad8a065d8f462cbec0207626e7` |
| WidowXAI MuJoCo model | <https://github.com/TrossenRobotics/trossen_arm_mujoco> | `77aba5d32654f17945f139e63310e7a3ac2cd4ab` |
| XR service Python bindings | <https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind> | `c64ccf6acd577a333e03b66fafe8efeeceb511b1` |

At the time of audit, local-only work included 83 new files in the experiment
repo, four important Python controller/URDF modifications, and intended Unity
changes. This handoff contains the critical design facts and calibration values,
but it is not a byte-for-byte replacement for those source files. Ask the user
for the old workspace archive only if exact implementation recovery is needed.

## Recommended Windows Architecture

Use:

- Windows for Unity Hub, Quest application development, and normal Windows ADB.
- WSL2 Ubuntu for the Python control stack, MuJoCo, Placo, Bash launchers, and the
  existing Linux XRoboToolkit service.
- Unity editor version `6000.0.44f1` for the existing Quest project.

Put the Python repository inside the WSL filesystem, not under `/mnt/c`, for
better file semantics and performance. Keep the Unity project on the Windows
filesystem and communicate over a documented TCP/UDP interface.

The old XRoboToolkit PC service is a Linux binary normally installed at:

```text
/opt/apps/roboticsservice/RoboticsServiceProcess
```

It listens for the Quest client on TCP port `63901`. Do not assume this binary
will run natively on Windows. Either run it in WSL2, obtain an official Windows
service, or replace this transport in the clean project.

For physical-arm experiments, native Ubuntu or dual boot is likely simpler and
more deterministic than WSL2 because the old workflow depends on Linux Ethernet
configuration, Bash, and direct hardware networking. WSL2 is appropriate for
development and simulation, but validate its USB and Ethernet path separately
before hardware control.

## Hardware And Network Facts

- Headset: Meta Quest 3.
- Input: left Quest controller; left grip is the teleoperation clutch.
- Robot: Trossen/WidowXAI follower, six revolute arm joints plus gripper.
- Robot IP: `192.168.1.2`.
- Previous laptop arm-side address: `192.168.1.3/24`, no gateway or DNS.
- Previous good physical topology: laptop USB-C Ethernet -> Netgear GS305
  switch -> robot.
- Quest wireless connection must use the laptop's Wi-Fi-side address, not the
  robot-Ethernet address.
- USB Quest test used `adb reverse tcp:63901 tcp:63901` and `127.0.0.1` in the
  Quest application.
- Adapter names such as `enx00e04c5c5270` and `eth0` belong to the old laptop.
  Discover and configure the new interface; never copy the old name blindly.

Use only one ADB owner at a time. If Windows owns the Quest USB connection, run
ADB on Windows. If WSL owns it through USB passthrough, run ADB in WSL.

## Verified WidowXAI Kinematic Geometry

Use the official URDF/MuJoCo files as the canonical model. The arm chain is:

| Joint | Parent -> child | Origin xyz (m) | Axis | Limits (rad) |
|---|---|---:|---:|---:|
| `joint_0` | `base_link -> link_1` | `(0, 0, 0.05725)` | `(0, 0, 1)` | `[-3.05433, 3.05433]` |
| `joint_1` | `link_1 -> link_2` | `(0.02, 0, 0.04625)` | `(0, 1, 0)` | `[0, 3.14159]` |
| `joint_2` | `link_2 -> link_3` | `(-0.264, 0, 0)` | `(0, -1, 0)` | `[0, 2.35619]` |
| `joint_3` | `link_3 -> link_4` | `(0.245, 0, 0.06)` | `(0, -1, 0)` | `[-1.5708, 1.5708]` |
| `joint_4` | `link_4 -> link_5` | `(0.06775, 0, 0.0455)` | `(0, 0, -1)` | `[-1.5708, 1.5708]` |
| `joint_5` | `link_5 -> link_6` | `(0.02895, 0, -0.0455)` | `(1, 0, 0)` | `[-3.14159, 3.14159]` |

The gripper-tip end-effector is `0.156062 m` along link 6's positive X axis.
The official MuJoCo model already has an `ee_site` approximately at
`(0.156, 0, 0)` in `link_6`.

This geometry makes the reference article's 3+3 split appropriate:

- joints 0-2 control the wrist anchor position;
- joints 3-5 control end-effector orientation;
- the `joint_3`/`link_4` origin is the wrist anchor and is unaffected by joints
  3-5.

Add a named MuJoCo site at the joint-3 anchor if needed. Verify numerically that
its world position does not change when only joints 3-5 move.

## Previous Working Pipeline

The best old path was:

```text
Quest controller pose
  -> Unity Quest application
  -> XRoboToolkit TCP service on port 63901
  -> Python XrClient latest pose
  -> grip-relative calibrated mapper
  -> Placo IK q_des at 50 Hz
  -> q_des low-pass filter
  -> joint step/velocity/acceleration/jerk shaper at 100 Hz
  -> nonblocking Trossen position command with a 30 ms goal horizon
  -> physical WidowXAI arm
```

The solver was not computationally expensive. Typical Placo solve time was
about `0.10 ms` median and `0.13 ms` p95. Replacing the solver alone will not
remove 80 ms of latency. A better solver may nevertheless allow less filtering
and improve orientation smoothness.

## Interaction Semantics To Preserve

Implement clutch-relative teleoperation:

1. On the rising edge of left grip, capture the current controller pose and
   current robot target pose as anchors.
2. While held, apply controller translation and rotation deltas relative to the
   anchors. Do not map absolute Quest room coordinates directly to the robot.
3. On release, hold the last safe robot command and stop integrating controller
   motion.
4. On the next press, re-anchor without a jump.
5. Pausing, stale-frame recovery, reconnecting, or changing mapping mode must
   also re-anchor before motion resumes.
6. Preserve the old startup behavior: initialize safely, move to the normal home
   pose, then require grip engagement before teleoperation.

The user normally stands behind the arm for the confirmed full-pose mapping.
A front/mirrored position mapping was also confirmed, but its orientation was
not confirmed on hardware.

## Confirmed Calibration Data

### Behind, full pose

This mapping was confirmed in MuJoCo and felt correct on the real arm.

```json
{
  "name": "left_behind_full_pose_good_20260602",
  "position_matrix": [
    [-0.4108349377824204, 0.13024321842754505, -0.9023587745187333],
    [-0.9022873931624731, 0.08384295950588601, 0.4229040296348135],
    [0.13073681209737126, 0.9879306970607448, 0.0830711969799855]
  ],
  "rotation_matrix": [
    [0.3386376558425662, -0.038989247194031455, 0.9401087046978629],
    [0.938475008942356, 0.08593148123551272, -0.33448533319611246],
    [-0.06774360217832504, 0.9955378541953656, 0.06569006947774796]
  ],
  "orientation_delta_mode": "matrix_conjugate",
  "task_rotation_signs": [-1.0, -1.0, 1.0]
}
```

### Front/mirrored, position only

Real-arm run 081 confirmed forward/depth, left/right, and up/down translation.
Keep orientation disabled until isolated pitch/yaw/roll probes validate it.

```json
{
  "name": "left_mirrored_position_good_20260602",
  "position_matrix": [
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0]
  ],
  "rotation_matrix_unconfirmed": [
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0]
  ],
  "orientation_delta_mode": "matrix_conjugate",
  "task_rotation_signs_unconfirmed": [1.0, 1.0, 1.0]
}
```

When implementing these matrices, write convention tests using explicit basis
vectors. Document whether vectors are columns, which side the matrix multiplies,
and the quaternion composition order. Do not transpose a matrix merely because
the motion looks wrong; prove conventions with tests and MuJoCo axes.

## Known-Good Conservative Control Baseline

Start the clean implementation with these values for A/B comparison:

```yaml
ik_rate_hz: 50
command_rate_hz: 100
command_goal_time_s: 0.030
joint_target_lowpass_hz: 8.0
xr_pose_filter: none
prediction_lead_time_s: 0.0
feedforward_source: none
position_task: hard
orientation_task_weight: 0.005
max_ee_velocity_m_s: 0.25
max_ee_acceleration_m_s2: 1.0
max_joint_velocity_rad_s: [1.0, 1.0, 1.0, 1.5, 1.5, 1.5]
max_joint_acceleration_rad_s2: [4.0, 4.0, 4.0, 8.0, 8.0, 8.0]
max_joint_jerk_rad_s3: [240, 240, 240, 360, 360, 360]
max_joint_step_rad: [0.005, 0.005, 0.005, 0.010, 0.010, 0.010]
```

The old system used Placo for `q_des` only. It deliberately did **not** send raw
Placo qdot/qddot feedforward to the robot.

Do not start physical tests above 50 Hz IK. A previous 75 Hz hardware experiment
produced visibly pulsed motion and table shake. The 100 Hz command shaper may
remain independent of the 50 Hz target/IK loop.

## What Earlier Experiments Established

- Position-only control was smoother and more consistent than coupled full-pose
  Placo IK.
- A good position-only run had Placo qdot p95 about `3.79 rad/s` and Placo qddot
  p95 about `157.9 rad/s^2`.
- Adding weak coupled orientation caused Placo qdot p95 around `15-16 rad/s` and
  qddot p95 around `712-789 rad/s^2`, with a much larger q_des-to-feedback gap.
- Raw Placo FFV/FFA made simulation and hardware more aggressive/flickery.
- Prediction caused continued movement or drift after the controller stopped.
- Heavy filtering of the raw controller pose added separation and visible lag.
- Lower goal times and higher hardware control rates could reintroduce pulsing.
- Blunt displacement caps felt like a soft wall and could cause reversal pulses.
- MuJoCo was useful for geometry and control validation but did not perfectly
  reproduce the real motor/network response.

Do not repeat these failed approaches as defaults.

## Latency Facts And Measurement Requirements

The previous 70-80 ms total was an engineering budget, not a synchronized
end-to-end measurement:

```text
Quest tracking/application/stream: approximately 30-40 ms
PC loop/sample waiting: approximately 10-20 ms
mapping + IK: less than 1 ms
PC-to-arm network/send: less than 1 ms
arm response and command horizon: approximately 22-30 ms
estimated total: approximately 70-80 ms
```

A representative real run measured:

```text
command period median: 10.068 ms
IK period median: 20.068 ms
IK loop compute median: 0.656 ms
Placo solve median/p95: 0.0987 / 0.1301 ms
XR processing/mapping median: 0.3077 ms
XR-read-to-command median/p95: 11.255 / 19.088 ms
command send median/p95: 0.459 / 0.933 ms
feedback read median: 0.651 ms
```

The old `xr_to_command_age` clock started after Python received/cached a sample.
It excluded Quest tracking, Unity scheduling, Unity queue age, Wi-Fi, and service
waiting. Therefore it was not true motion-to-command latency.

The new transport/logging schema must carry:

- monotonically increasing pose sequence number;
- Quest monotonic capture timestamp;
- Quest enqueue/send timestamp;
- PC socket arrival timestamp;
- control-loop sample-consume timestamp;
- IK start/end timestamps;
- command-shaper and command-send timestamps;
- robot feedback receive timestamp;
- stale/repeated sample count and reconnect generation.

Synchronize clocks or estimate their offset before subtracting Quest and PC
timestamps. Validate the final physical result independently with a high-speed
camera (ideally 240 fps) observing controller movement and robot movement. Keep
video glass-to-glass latency separate from motion-command latency.

## Existing Quest Transport Defect To Avoid

The previous Unity `TcpHandler.cs` had a two-entry tracking FIFO. `Update()` only
enqueued when the queue contained fewer than two messages, and the sender sent
the oldest queued sample. At 120 Hz this alone could preserve roughly 16.7 ms of
stale tracking during congestion. It also did not robustly loop until an entire
TCP payload was sent.

The clean transport should:

1. Use a capacity-one overwrite-oldest/latest-state mailbox for pose data.
2. Coalesce/drain to the newest pose before serialization or send.
3. Include sequence and capture timestamps in every pose packet.
4. Complete partial socket sends or use a framed library that guarantees this.
5. Clear stale state on reconnect.
6. Avoid a busy-spin sender when no tracking sample exists.
7. Separate high-rate pose state from lower-rate commands/events.

Switching from raw TCP to WebSocket does not by itself solve stale FIFO or TCP
head-of-line blocking. First fix freshness semantics. Later, compare TCP
latest-state against UDP with sequence numbers and loss/out-of-order rejection.

## Clean Target Architecture

Create one new repository with clear interfaces:

```text
widowxai-quest-teleop/
  README.md
  pyproject.toml
  configs/
    baseline.yaml
    calibrations/
  src/widowxai_quest_teleop/
    model.py
    transport.py
    sample_buffer.py
    mapping.py
    clutch.py
    decoupled_ik.py
    placo_backend.py
    command_shaper.py
    safety.py
    hardware.py
    telemetry.py
  unity/
    README.md
  tests/
  scripts/
    run_xr_benchmark.py
    run_sim.py
    replay_recording.py
    run_hardware.py
```

Keep transport, mapping, IK, command shaping, and hardware I/O independent.
Recorded Quest pose streams must be replayable through both the legacy Placo
backend and the new decoupled backend with identical timestamps and mapping.

## New Decoupled IK Backend

Implement the reference article's main idea as a separate backend:

1. Compute the desired joint-3 wrist-anchor position from the desired tool pose
   and calibrated tool/wrist geometry.
2. Solve joints 0-2 for wrist-anchor position using damped least squares.
3. With joints 0-2 updated/fixed, solve joints 3-5 for tool orientation using a
   separate damped least-squares step.
4. Apply separate adaptive damping/manipulability logic to the arm and wrist.
5. Add a small null-space/rest-pose bias without overriding the primary task.
6. Clamp every step and final command to official joint limits.
7. Handle angle wrapping, orientation antipodes, near-pi errors, and wrist
   gimbal singularities explicitly.
8. Record residuals, manipulability, damping value, step norm, iteration count,
   joint-limit margin, and solver status every cycle.

Canonical DLS form:

```text
dq = J^T (J J^T + lambda^2 I)^-1 e
```

Use stable linear solves, not explicit matrix inversion. Put damping thresholds,
maximum task steps, and iteration limits in configuration. Establish values in
simulation and recorded-stream replay before any hardware test.

Preserve the Placo backend for A/B comparison. Feed either backend's `q_des`
through the same time-based safety and command-shaping layer.

## Other Reference Ideas Worth Porting

### Controller wrist-pivot calibration

The old mapper used the controller object's reported pose directly. Human wrist
rotation can then appear as unwanted translation because the controller origin
is offset from the anatomical wrist. Calibrate a controller-to-wrist pivot and
apply rotation around that pivot before computing translation deltas.

### Absorbing/slipping reach limits

Do not use a static hard displacement cap that stores unreachable error. Maintain
an incremental reference relative to current robot pose. When the desired motion
exceeds translational or rotational reach, discard the excess so reversing the
controller produces an immediate robot response instead of first unwinding a
hidden error reservoir.

### Stale XR watchdog

Detect freshness from unique Quest sequence/timestamp changes, not merely from
how often Python reads a cached pose. If the stream becomes stale:

- stop advancing targets;
- hold or enter the configured safe state;
- require a fresh sample window;
- re-anchor the clutch before resuming.

### Per-engage yaw correction

Headset-relative yaw correction can be added later. Freeze the operator frame at
grip engagement so normal head motion does not rotate the mapping. Be careful:
the old custom calibration-matrix path bypassed its headset-yaw composition, so
the new implementation must make composition order explicit and tested.

### Haptics

Haptics can signal clutch, reach boundary, stale stream, or singularity. It is a
usability feature, not a latency fix, so implement it after the core control loop.

## Staged Implementation And Acceptance Gates

### Stage 0: Reproducible bootstrap

- Initialize the clean Git repository.
- Record Windows version, WSL distribution, Python, Unity, Quest OS/app version,
  arm firmware/SDK, and all dependency versions.
- Old working environment reference: Python `3.10.20`, NumPy `2.2.6`, MuJoCo
  `3.8.1`; Placo imported successfully but did not expose a useful version.
- Do not trust the old `requirements-dry.txt`; it listed only three packages and
  even disagreed with the installed NumPy version.

Gate: a fresh environment can run unit tests and load the official WidowXAI
model from documented commands.

### Stage 1: Geometry and solver tests

- Validate FK at reference configurations against MuJoCo.
- Verify the joint-3 anchor is invariant under joints 3-5.
- Add finite-difference Jacobian tests.
- Test decoupled IK on reachable poses, joint-limit edges, near singularities,
  orientation antipodes, and unreachable targets.

Gate: bounded errors, no NaNs, no joint-limit violations, deterministic replay.

### Stage 2: Mapper and clutch tests

- Implement both confirmed calibration modes.
- Test basis-vector mapping and quaternion composition.
- Test engage, release, re-engage, stale recovery, pause, and reconnect for zero
  target discontinuity.
- Add controller wrist-pivot calibration.

Gate: all state transitions are jump-free in tests and visualization.

### Stage 3: Quest transport benchmark, no arm

- Implement latest-state semantics, sequence IDs, timestamps, and stale watchdog.
- Compare wireless and USB reverse under controlled motion.
- Report unique sample rate, inter-arrival jitter, repeated frames, packet age,
  queue age, drops, and reconnect behavior.

Gate: no growing queue, bounded age, and safe stale/reconnect behavior.

### Stage 4: Full MuJoCo integration

- Run recorded and live Quest streams through Placo and decoupled IK.
- Use the same command shaper and telemetry for both.
- Compare Cartesian error, joint derivatives, limit margin, damping, command
  delay, and visible smoothness.

Gate: decoupled IK improves or matches orientation behavior without degrading
position tracking or creating derivative spikes.

### Stage 5: Physical arm, tiny motions

- Verify physical emergency stop, network, home behavior, joint feedback, and
  command timeout.
- Start position-only with tiny, slow motions.
- Validate translation axes independently.
- Add orientation one axis at a time: pitch/yes, yaw/no, roll/screwdriver.
- Stop immediately for pulsing, table shake, unexpected axes, aggressive sound,
  stale motion, or any discontinuity.

Gate: user confirms mapping and smoothness before expanding workspace or speed.

### Stage 6: Latency tuning

Only after stable decoupled control, sweep one setting at a time:

- joint-target low-pass: `8 -> 10 -> 12 Hz`;
- goal horizon: `30 -> 27 -> 25 ms`;
- transport: wireless latest-state vs USB reverse;
- optional higher IK rate only in simulation first.

Do not simultaneously change filtering, limits, solver, rate, and goal horizon.
The expected gain from better IK and cautious retuning is modest, perhaps on the
order of 5-20 ms; do not promise an 80-to-20 ms reduction.

## Mandatory Telemetry

Log at minimum:

- raw Quest pose, sequence, and capture time;
- mapped target pose and clutch/reanchor generation;
- wrist-anchor target/current/error;
- orientation target/current/error;
- `q_des`, `q_cmd`, and joint feedback;
- estimated qdot/qddot for target, command, and feedback;
- DLS damping/manipulability/residual/status;
- command clipping and each safety limiter's activity;
- all latency timestamps listed earlier;
- network reconnect/stale/drop counters;
- configuration snapshot and Git commit.

Each run must have a unique label and write machine-readable CSV or Parquet plus
a concise summary. Never delete a useful run before it has been summarized.

## Physical Safety Rules

- No autonomous physical-arm execution merely because tests pass.
- Require the user to explicitly request each hardware run.
- Confirm arm IP/interface and successful feedback before enabling torque/motion.
- Begin from the normal home posture with a clear workspace and reachable stop.
- Require clutch engagement for teleoperation.
- On stale XR, lost arm feedback, exception, solver failure, or limit violation,
  stop advancing targets and enter the defined safe hold/disable behavior.
- Clamp position, velocity, acceleration, jerk, per-cycle step, joint limits, and
  Cartesian reach independently.
- Never enable raw Placo qdot/qddot feedforward by default.
- Do not raise hardware IK/control rate above 50 Hz without a simulation result,
  one-variable test plan, and explicit user approval.

## First Assignment For The New Codex

After reading this handoff, do the following:

1. Inspect the Windows laptop and decide whether the Python controller will run
   in WSL2 or native Ubuntu/dual boot. Explain any hardware-access limitation.
2. Create a clean implementation plan and repository skeleton, but do not run
   the physical arm.
3. Clone or reference the official WidowXAI URDF and MuJoCo repositories at the
   verified commits.
4. Add the confirmed calibration JSONs and conservative baseline configuration.
5. Implement/test the model abstraction and joint-3 wrist-anchor invariant.
6. Implement the decoupled IK backend with offline tests.
7. Implement replayable mapper/clutch and telemetry interfaces.
8. Demonstrate the pipeline in MuJoCo before proposing the first Quest or arm
   test.

If exact old code is needed, ask the user for the archived old workspace. If the
goal is the clean reference-inspired system described here, proceed without the
old clutter and use the verified repositories only as model/driver references.

## Suggested New-Chat Message

Give this file to Codex on the Windows laptop and say:

```text
We are starting a clean WidowXAI + Meta Quest 3 teleoperation project. Read the
attached WINDOWS_CLEAN_PROJECT_HANDOFF.md completely and treat it as the source
of truth. Do not assume access to my previous laptop or chat. First summarize the
goal, known-good facts, risks, and staged plan. Then inspect this laptop and
scaffold only the simulation-first clean project. Do not command the physical
robot unless I explicitly request a hardware test.
```
