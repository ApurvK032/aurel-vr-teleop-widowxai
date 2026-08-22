# Camera-tracked leader to physical WidowXAI

This path reuses the proven WidowXAI driver, firmware checks, guarded startup,
command shaping, feedback supervision, and return-to-rest behavior
from the Quest project. Its input is the five-joint UDP stream from the M3T CAD
tracker instead of a Quest controller pose.

It is currently a **guarded physical commissioning path**, not an accepted
full-workspace teleoperator. The original profile still requires one selected
axis. The dedicated J0/J1/J2 profile permits only the exact selection
`--commission-joints 0,1,2`; it does not enable J3, J4, joint 5, or the gripper.

## What is controlled

| M3T source | WidowXAI follower | First-run behavior |
|---|---|---|
| `link1_link` | `joint_0` | candidate sign `+`; 30% baseline / 50% J012 |
| `link2_link` | `joint_1` | candidate sign `+`; 30% baseline / 50% J012 |
| `link3_link` | `joint_2` | candidate sign `-`; 30% baseline / 50% J012 |
| `link4_link` | `joint_3` | candidate sign `-`; fixed in J012 |
| `link5_link` | `joint_4` | candidate sign `-`; fixed in J012 |
| not tracked | `joint_5` | zero in MuJoCo; measured session-start angle on hardware |
| not tracked | gripper | mode and command disabled |

The follower starts and remains at all-zero rest while position control is
enabled and stabilized. The current visual leader pose is anchored to that
same rest pose when the hold-to-run control is pressed. Therefore a new camera
calibration or a different visual zero does not create an absolute robot jump.

The joint-0 commissioning envelope is ±60° around rest by explicit operator
request. Pending joints 1–4 retain their ≤2° envelopes. Because joints 1 and 2
are already at their official lower limits at zero, their first physical tests
allow only positive travel. The run is capped at 15 seconds. Expanding any
remaining limit requires separate physical evidence; do not weaken unrelated
gates. The model proves self-collision clearance across joint 0's range but
cannot prove clearance from real tables, arms, people, or other equipment.

The original positive joint-1 sign is retained. The two earlier joint-1 stops
are treated as boundary evidence from an imperfect printed leader, not proof
of a sign error. In the J0/J1/J2 profile, an unreachable request saturates at
the official follower model boundary rather than terminating the run.

Joint 0 begins responding after 0.03 rad (about 1.7°) of tracked leader motion.
This is more than six times the maximum filtered stationary deviation measured
in the first rest-anchored physical run. The earlier 0.15 rad threshold delayed
visible response until the leader had already moved 8.6° and is no longer used.

## Safety behavior

Physical output requires all of the following:

- `root_locked=true` in every accepted M3T packet;
- exact five-joint names, finite values, recent timestamps, and monotonic
  sequences;
- three continuous fresh packets before recovery;
- no source change above 0.12 rad per frame;
- raw-packet jump/freshness checks followed by a time-based, speed-adaptive
  circular joint filter that updates once per unique M3T packet;
- a Linux evdev key, foot pedal, or gamepad button whose physical state is
  queried on every control iteration;
- an explicit input-device path on every live command, with the input released
  both before preflight and after rest-anchor stabilization;
- a CAD-specific live token that cannot enable the Quest launcher;
- an extra first-run override while the sign mapping is marked pending;
- an exact profile-authorized follower-joint selection while the mapping is
  pending (one axis in the baseline, or exactly J0/J1/J2 in the J012 profile);
- the arm measured within 0.08 rad of all-zero rest before position mode;
- official model/controller limits, self-collision checks, velocity,
  acceleration, and per-tick gates;
- a 20 ms driver interpolation horizon; joint 0 is capped at 0.25 rad/s and
  1.0 rad/s² while pending joints 1–4 retain 0.10 rad/s and 0.50 rad/s²;
- 50 Hz time-aligned encoder checks with the unchanged 0.08 rad hard stop;
- return to all-zero rest on normal exit, Ctrl+C, or a safety exception.
- AC power, the `performance` power profile, and a direct route to the robot
  sourced from the dedicated `192.168.1.10` Ethernet address.

Releasing the physical control normally holds the last safe command. Any input
fault—loss or rejection of camera packets, an unlocked root, tracker restart,
source jump, mapping rejection, or evdev failure—records a fault, sends at most
one held command where possible, exits the control loop, and performs the
guarded return to all-zero rest. Typing `S` (or `Q`) has the same return-and-exit
behavior. Live mode deliberately cannot restart with terminal `E`; after a
fault, start a new process so the complete stream, workstation, network,
driver, rest-pose, model, and physical hold-to-run preflight runs again. The
terminal `e`/`r` latch remains available only in MuJoCo and dry runs. The code
never chases delayed encoder feedback when re-anchoring.

## Latency target and measurement

The target is less than 35 ms from physical leader motion to follower motion.
That target must not be inferred from the M3T UDP timestamp alone: the legacy
`time_ns` value is created after tracking and therefore omits camera exposure
and pose-estimation time. New M3T packets retain `time_ns` for compatibility
and also publish a separate D455 `frame_time_ns` in the RealSense
`global_time` domain. Hardware and MuJoCo telemetry now preserve:

- D455 frame capture to M3T publication;
- M3T publication to UDP receipt;
- packet age when the control loop consumes it;
- raw-to-filtered, desired-to-limited-command, and command-to-encoder phase;
- driver call duration and fresh encoder-read timestamps.

Analyze a run without opening a camera or robot:

```bash
env -u PYTHONPATH .venv/bin/python scripts/analyze_cad_latency.py \
  runs/YYYY-MM-DD/<cad-run> \
  --joint 0 \
  --target-ms 35
```

The last pre-instrumentation physical run measured about 215 ms of
tracker-packet-arrival-to-encoder velocity-phase lag. Approximately 165 ms was
desired-to-command limiter lag, about 20 ms was command-to-encoder phase, UDP
was about 1--2 ms, and source filtering contributed effectively 0--10 ms. It
cannot report capture-to-publish time because that tracker process predated the
new frame timestamp.

The current D455 tracking profile is 848 x 480 at 30 FPS. An arbitrary leader
motion can wait up to one 33.3 ms frame period before being observed; its p95
sampling wait alone is about 31.7 ms. Therefore a true physical p95 below
35 ms is impossible at 30 FPS even with zero computation and actuator delay.
Do not call the target achieved until a measured profile uses at least 60 FPS,
the command limiter no longer adds a large phase delay, and both the internal
telemetry and a synchronized high-speed video check agree. The analyzer's
cross-correlation is a phase estimate; encoder feedback is not an optical
measurement of link motion.

After recording the default 30 FPS baseline, stage the tracker only with
`M3T_D455_FPS=60 ./experiments/m3t_leader_arm_assembly/run_joints_d455_mat.sh`
from the CAD repository. This override changes the M3T D455 stream, not the
mat-check capture. Validate the overlay, packet rate, capture-to-publish time,
and stationary jitter before connecting it to any physical run.

## Before going to the lab

The tested hardware environment is CPython 3.10 with `trossen-arm==1.8.6`; the
controller firmware is 1.8.3. The follower is normally `192.168.1.2` and the PC
robot-LAN address is `192.168.1.10/24`, with no gateway. Never probe controller
port 50001 using `nc`, telnet, or a raw socket.

Run the offline suite:

```bash
cd "/home/apurv/Codex-Complete/322/VR Teleoperation Stack for Robot Manipulation-WXAI/aurel-vr-teleop-widowxai"
env -u PYTHONPATH PYTHONDONTWRITEBYTECODE=1 \
  .venv/bin/python -m pytest -q -p no:cacheprovider
env -u PYTHONPATH PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .venv-arm18/bin/python -m pytest -q -p no:cacheprovider
```

## Lab commissioning sequence

Use AC power and the performance power profile. Keep arm Ethernet separate from
the camera hub if possible; the shared dock previously reset Ethernet and
camera devices together under load. The live launcher now checks these
workstation conditions and the direct robot route before constructing the
Trossen backend. It will intentionally reject the current Wi-Fi route until the
dedicated robot Ethernet interface is restored.

Select a stable `/dev/input/by-id/...-event-*` path for a USB foot pedal,
gamepad button, or keyboard key. A foot pedal or a second operator is preferred
so the person moving the printed leader never has to let go of it. The default
Linux key code is 57 (`KEY_SPACE`). List candidates and qualify a complete
release/press/release cycle without importing the robot driver:

```bash
env -u PYTHONPATH .venv/bin/python scripts/check_hold_to_run.py --list
export CAD_HOLD_DEVICE=/dev/input/by-id/<your-device>-event-kbd
env -u PYTHONPATH .venv/bin/python scripts/check_hold_to_run.py \
  --device "$CAD_HOLD_DEVICE" \
  --key-code 57
```

Do not continue unless the checker prints `RESULT PASS`. Do not run the robot
launcher with `sudo`; correct the evdev device permissions instead. The live
launcher rejects a missing, unsupported, inaccessible, or already-held input
before workstation checks or robot construction.

Check the workstation without importing the arm driver or contacting a robot:

```bash
env -u PYTHONPATH .venv/bin/python scripts/check_workstation.py \
  --config configs/cad_hardware_commissioning.yaml
```

1. Firmly mount the follower, clear both arm workspaces, keep the controller
   power cutoff in reach, and put the follower at all-zero rest.
2. Recalibrate the camera session and launch the M3T overlay from the CAD
   repository. For the present ZED/D455 placement:

   ```bash
   cd /home/apurv/Codex-Complete/322/CAD-pose_detection
   ./experiments/m3t_leader_arm_assembly/run_joints_d455_mat.sh
   ```

   Apply the detector with `d`, start tracking with `t`, wait for a correct
   overlay, then press `l`. The ZED currently validates the mat seed; live M3T
   evidence still comes from D455 RGB-D.
3. With the printed leader completely stationary, validate five seconds of the
   exact stream. This command never imports or contacts the robot driver:

   ```bash
   cd "/home/apurv/Codex-Complete/322/VR Teleoperation Stack for Robot Manipulation-WXAI/aurel-vr-teleop-widowxai"
   env -u PYTHONPATH .venv/bin/python scripts/check_cad_stream.py \
     --config configs/cad_hardware_commissioning.yaml \
     --duration 5
   ```

   Do not continue unless it prints `"passed": true`.
4. Stop the stream checker. Only one process can bind UDP port 5055. Preview the
   mapping in MuJoCo from the all-zero joint pose and validate each source joint
   direction separately. The legacy config filename is retained for command
   compatibility, but its simulation rest pose is `[0,0,0,0,0,0]`. Start with
   follower joint 0:

   ```bash
   env -u PYTHONPATH .venv/bin/python scripts/run_cad_sim.py \
     --config configs/cad_home_commissioning_mujoco.yaml \
     --commission-joint 0 \
     --label cad-j0-mujoco
   ```

   Click the MuJoCo window and press `E` to start from all-zero rest, `R` to
   hold/re-anchor at the current simulated command, `S` to return to all-zero
   rest while keeping the viewer open, and `Q` to return to rest and close.
   The terminal alternatives are `e`, `r`, `s`, or `q` followed by Enter. A
   stale/rejected packet, tracker restart or jump, mapping error, predicted
   collision, or command-gate fault also returns to rest and enters
   `REST HOLD`; it cannot move again until the source has recovered and the
   operator presses `E`. Repeat separate MuJoCo runs with
   `--commission-joint 1`, `2`, `3`, and `4`. Even if every source estimate
   moves, only the selected follower joint may leave zero rest. This simulation
   profile uses 100% one-for-one scale, no source deadband, and the official
   WidowXAI joint ranges. The all-zero pose puts follower joints 1 and 2 at
   their lower model limits, so a source request past either boundary saturates
   only that joint while the other joints continue following. The baseline
   physical profile remains independently limited to 30%, ±60° on joint 0,
   and ≤2° on pending joints 1–4. The separate J012 physical profile described
   below also saturates at official model limits.

   For an unattended simulation-only startup check, add
   `--simulation-auto-deadman`. It presses `E` only once after the initial
   three fresh packets. It never re-arms after an issue: the viewer returns to
   rest and requires a manual `E`, exactly as above. Physical output never uses
   automatic engagement.
5. Perform the official-driver read-only check. It configures a driver session
   but does not enable position mode or send a command:

   ```bash
   env -u PYTHONPATH .venv-arm18/bin/python scripts/preflight_hardware.py \
     --config configs/cad_hardware_commissioning.yaml
   ```
6. Exercise the complete hardware lifecycle against the in-memory backend:

   ```bash
   env -u PYTHONPATH .venv-arm18/bin/python scripts/run_cad_hardware.py \
     --config configs/cad_hardware_commissioning.yaml \
     --commission-joint 0 \
     --duration 10 \
     --label cad-j0-hardware-dry-run
   ```
7. Only after explicit same-session authorization, run the physical sign
   commissioning:

   ```bash
   env -u PYTHONPATH .venv-arm18/bin/python scripts/run_cad_hardware.py \
     --config configs/cad_hardware_commissioning.yaml \
     --live \
     --confirm-live LIVE-WIDOWXAI-CAD-192.168.1.2 \
     --accept-unvalidated-mapping \
     --commission-joint 0 \
     --deadman-device "$CAD_HOLD_DEVICE" \
     --deadman-key-code 57 \
     --duration 15 \
     --label cad-j0-sign-physical-01
   ```

   Keep the physical control released while the follower stabilizes at its
   all-zero rest anchor. After `rest anchor ready`, press and continuously hold it while moving only the
   printed-leader joint mapped to the selected follower joint, slowly in both
   directions. Release the control to hold; then type `s` (or `q`) to return
   the arm to all-zero rest and end that run. Start a new complete process with
   joint 1 and a matching
   label, then repeat for joints 2, 3, and 4. The pending profile rejects
   `--live` if `--commission-joint` is omitted, even when the mapping override
   and live token are present. Release the physical control immediately if any
   direction, sound, smoothness, or overlay behavior is wrong.

### Current J0/J1/J2 experiment

First run the exact three-axis selection in MuJoCo. This remains 100% scale and
cannot open the physical driver:

```bash
cd "/home/apurv/Codex-Complete/322/VR Teleoperation Stack for Robot Manipulation-WXAI/aurel-vr-teleop-widowxai"
env -u PYTHONPATH .venv/bin/python scripts/run_cad_sim.py \
  --config configs/cad_home_commissioning_mujoco.yaml \
  --commission-joints 0,1,2 \
  --label cad-j012-mujoco
```

Click the MuJoCo window and press `E`. Only joints 0, 1, and 2 may move. Press
`S` to return to rest without closing the viewer or `Q` to return and exit.

The matching physical profile is
`configs/cad_hardware_j012_experimental.yaml`. It uses 50% scale and
permits the full official J0/J1/J2 pose ranges with boundary saturation. It
does not impose the baseline ±60°/2° commissioning envelope. This removes an
artificial pose stop. For this profile only, the operator requested automatic
activation and a 20-second run: after all preflights and rest stabilization it
shows a three-second countdown, anchors the current filtered leader pose, and
starts following without Space or an evdev device. Rate limiting, stream
freshness/restart checks on the commanded axes, collision prediction, feedback
supervision, interactive `S`/`Q`, and return to rest remain. J3, J4, joint 5,
and the gripper remain fixed.

After visually accepting the MuJoCo directions, validate the full lifecycle
without opening the robot:

```bash
env -u PYTHONPATH .venv-arm18/bin/python scripts/run_cad_hardware.py \
  --config configs/cad_hardware_j012_experimental.yaml \
  --commission-joints 0,1,2 \
  --duration 20 \
  --label cad-right-j012-dry
```

Only with explicit same-session physical authorization, a clear workspace,
the right arm at all-zero rest, and the terminal focused for `S`/`Q`, use:

```bash
env -u PYTHONPATH .venv-arm18/bin/python scripts/run_cad_hardware.py \
  --config configs/cad_hardware_j012_experimental.yaml \
  --live \
  --confirm-live LIVE-WIDOWXAI-CAD-192.168.1.3 \
  --accept-unvalidated-mapping \
  --commission-joints 0,1,2 \
  --duration 20 \
  --label cad-right-j012-rest-50pct-auto-r1
```

The live command accepts only the exact set `0,1,2`. It does not accept or
require `--deadman-device`. Type `s` or `q` followed by Enter at any point to
exit and return to rest. Ctrl+C, a commanded-axis source fault, collision
prediction, or feedback fault does the same.

## What remains before full teleoperation

The bridge is fail-closed, but these evidence gaps prevent calling the complete
system “perfect” today:

1. The five candidate direction signs need operator acceptance from five
   separate single-joint hardware runs.
2. M3T supplies no per-joint confidence score. A smooth visual drift can pass a
   freshness/step watchdog, so full-range or unattended output is not allowed.
3. The current live tracker uses D455 RGB-D after a ZED/D455 mat check; ZED is
   not yet a second live M3T modality.
4. Wrist joint 5 and the gripper are not observed. In MuJoCo both remain
   exactly zero. On hardware, joint 5 is locked to its measured session-start
   angle through startup, tracking, holds, recovery, and shutdown; the gripper
   never enters position mode or receives a command. Feedback drift beyond
   0.005 rad or 0.001 m respectively stops the run. They must remain excluded
   until the leader model/stream is extended and separately commissioned.
5. Stationary noise, latency, occlusion recovery, and deliberate-motion traces
   must be measured at the final camera distance before increasing the current
   per-joint envelopes or the current J012 50% scale.

After a clean first physical run, inspect the exact telemetry under
`runs/YYYY-MM-DD/`. Promote the mapping status only after the operator confirms
all five signs and the trace shows no stale episode, discontinuity, limiter
fault, deadman fault, feedback fault, or abnormal send time. The trace records
the deadman source, device, key code, state-query age, and press/release
generations.
Run `scripts/analyze_cad_latency.py` on the same directory and retain its
capture, transport, limiter, and encoder results with the run evidence.
