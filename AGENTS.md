# AGENTS.md — WidowXAI + Meta Quest 3 Teleoperation

This file applies to the entire repository. It is the durable handoff for future
Codex sessions. Read it before changing code, selecting a configuration, opening
an arm connection, interpreting latency, or deleting experimental evidence.

## Mission and current truth

The project teleoperates one Trossen WidowXAI follower arm from a Meta Quest 3
Touch Plus controller. It began as a clean implementation of Aurel Arnold's
article and `Dream-Machines-Robotics/vr-teleop-kit`, with DK1 geometry replaced
by the official WidowXAI model. It now combines:

- the article/reference clutch-relative controller mapping, latest-state
  transport, filtered pose, and decoupled 3+3 IK during teleoperation;
- the proven WidowXAI rest-to-home and home-to-rest lifecycle recovered from
  the user's earlier XRoboToolkit/GELLO work;
- a legacy Trossen driver adapter matched to this physical arm;
- measured smoothness/latency improvements developed one step at a time.

As of 2026-07-24, the current accepted single-arm profile is:

```text
configs/quest_50pct_hardware.yaml
```

It selects a fixed task profile from the Quest page before connecting to the
arm. Right controller + Mirrored is accepted at 50% of the article task scale
with full gripper control. The latest accepted evidence is:

```text
runs/2026-07-23/20260723-171126_four-mode-50pct-continuous
```

That run contains 4,627 rows over 51.398 seconds at 90.003 Hz and zero IK
failures. The user described this profile as smooth, responsive, and working
well. The accepted calibration is
`configs/calibrations/right_mirror_20260723_accepted.json`.

The tagged left-controller Step-3 profile remains the rollback baseline:

```text
configs/step3_velocity_feedforward_25ms_hardware.yaml
```

It is protected by Git tag `milestone-3-step-3-best` at commit `521f55b`;
commit `95e04a1` adds operator-heading lock. Its best complete evidence is
`runs/2026-07-22/20260722-165441_milestone3-step3-60s-acceptance`: 5,400
rows and zero IK failures.

Left/Behind, Left/Mirror, and Right/Behind remain pending at the current 50%
scale. The older reduced-gain Right/Behind acceptance and rejected calibration
attempts remain historical evidence; they do not override the current
Right/Mirror acceptance.

## Non-negotiable physical-arm rules

1. Never move the physical arm without an explicit command from the user in the
   current conversation. Inspection, documentation, simulation, and offline
   telemetry analysis do not authorize motion.
2. Before any physical run, require a clear workspace, a firmly mounted arm,
   and a person ready to cut controller power.
3. Keep the grip released during preflight and the automatic rest-to-home move.
   Grip is the live deadman only after the launcher prints `home reached`.
4. Use MuJoCo first after calibration, mapping, IK, timing, or limit changes.
5. Never bypass the `--live` and exact
   `--confirm-live LIVE-WIDOWXAI-192.168.1.2` gates.
6. Never automatically clear controller errors, flash firmware, update firmware,
   or change parameters stored on the arm.
7. Never probe TCP port `50001` with `nc`, telnet, a raw socket, or a generic
   readiness check. The controller appears to support one client and a raw probe
   wedged its server. Inspect local sockets with `ss` and use ICMP `ping` only.
8. If the driver hangs after printing the TCP and UDP connection lines but
   before driver/firmware versions, stop the process. If a stale local session
   clears but the next driver handshake still hangs, have the user power-cycle
   the controller with the arm supported. Do not repeatedly reconnect.
9. Do not assume a safety stop is permission to weaken all limits. Diagnose the
   exact check and retain hard joint, collision, stale-stream, driver-version,
   feedback, and shutdown gates. The repository owner explicitly superseded
   the duration-gate requirement for the shared four-mode validation profile
   on 2026-07-23; this does not authorize weakening any other gate.
10. On any normal or safety-stop exit after motion starts, the launcher must
    return the arm to all-zero rest. If the return fails, hold measured position
    if possible and instruct the user to cut controller power.

## Confirmed laboratory facts

| Item | Confirmed value |
|---|---|
| Arm | Trossen WidowXAI follower |
| Arm IP | `192.168.1.2` |
| PC LAN address observed | `192.168.1.3` |
| Driver TCP/UDP | TCP `50001`, UDP `50000` |
| Trossen driver | `trossen-arm==1.8.6` |
| Driver-reported version | `1.8.6` |
| Controller firmware | `1.8.3` |
| End-effector profile | `StandardEndEffector.wxai_v0_follower` / project `legacy_1_8` |
| Hardware Python | CPython 3.10 in `.venv-arm18` |
| Simulation Python | project `.venv` |
| Quest | Meta Quest 3 over USB ADB reverse |
| Quest ADB | One authorized Quest 3 observed |
| Relay | `0.0.0.0:8443`, Quest opens `http://localhost:8443/` |
| Rest joints | `[0, 0, 0, 0, 0, 0]` degrees |
| Home joints | `[0, 60, 75, -60, 0, 0]` degrees |
| Home radians | `[0, pi/3, 5pi/12, -pi/3, 0, 0]` |
| Gripper | `0.040 m` open, `0.000 m` closed |

The Quest and LAN were connected through a USB/LAN hub on this Ubuntu PC. The
arm and PC were visible on the same LAN. No router/cloud hop is needed for the
arm driver.

## Environments and common commands

Always start in:

```bash
cd /path/to/aurel-vr-teleop-widowxai
```

Use `env -u PYTHONPATH` to prevent packages from the parent workspace or the
active Conda environment from shadowing this repository.

### Restart Quest localhost

```bash
bash scripts/restart_localhost.sh
```

This restores ADB reverse `tcp:8443 -> tcp:8443`, starts or restarts the relay,
and applies the Quest virtual proximity/mounted override when supported.

Open the controller-selection page:

```text
http://localhost:8443/
```

Before entering passthrough, choose left or right and `Behind / Parallel` or
`Mirrored`, then press `Apply input`. Behind/Parallel is the matched task
motion. Mirrored flips left/right translation, screw, and nod-no while leaving
up/down and nod-yes unchanged. On 2026-07-23 the owner additionally required
front/back translation to flip in Mirrored mode. The transport values remain
`real` and `mirror` for backward compatibility. The choice is remembered on
the headset. Old `?hand=left` and `?hand=right` links remain backward
compatible but are no longer required.

MuJoCo and the Quest-only guided calibration follow the page selection
automatically and record the chosen hand/mode. A changed selection forces a
simulation re-anchor; changing it during calibration is rejected. Existing
physical profiles require the page to match their configured `quest.hand` and
`quest.mapping_mode`. The current
`quest_50pct_hardware.yaml` profile instead resolves its fixed
four-way transform from the first fresh, grip-released page sample before any
arm connection, then locks the selection. A later selection change is a safety
stop, never a live transform swap. On 2026-07-23 the user requested gripper
testing in this shared profile. It now uses the existing validated Step-3
full-stroke gripper limits and separate blocking open/close lifecycle, while
retaining every non-duration safety gate. After one clean 15-second gripper run
(1,350 rows, zero IK failures), the repository owner explicitly requested that
this profile run without a duration deadline. Its
`hardware.max_demo_duration_s` is therefore explicit `null`; omitting
`--duration` runs until Ctrl+C or an error/safety exit. Ctrl+C and all error
paths still return the arm to rest. This does not broaden the earlier
right-Behind acceptance, which had no gripper commands. Enter passthrough
before a live run.

The owner revised the shared profile to 50% of the article values: gains
`0.75`, position reach `0.125 m`, and rotation reach `0.30 rad`. Right/Mirror
was subsequently accepted through three physical runs with 12,585 total rows,
zero IK failures, and full gripper control. The other three modes remain
pending at this scale.

### Current MuJoCo check

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_live_sim.py \
  --config configs/quest_50pct_hardware.yaml \
  --label quest-50pct-mujoco-check
```

The hardware configuration can be consumed by the simulator; `run_live_sim.py`
does not import the arm driver or send arm commands.

### Current physical command

Do not execute this unless the user explicitly authorizes a physical run:

```bash
env -u PYTHONPATH .venv-arm18/bin/python scripts/run_hardware.py \
  --config configs/quest_50pct_hardware.yaml \
  --live \
  --confirm-live LIVE-WIDOWXAI-192.168.1.2 \
  --duration 30 \
  --label right-mirror-50pct-arm-test
```

The explicit command-line gate replaced the earlier second interactive
`ARM READY` prompt. Startup and shutdown remain automatic and supervised.

### Deferred left calibration

This is Quest-only and never imports or contacts the arm driver:

```bash
env -u PYTHONPATH .venv/bin/python scripts/calibrate_task_frame.py \
  --config configs/step3_velocity_feedforward_25ms_hardware.yaml \
  --output configs/calibrations/left_guided_6dof.json \
  --repeats 2 \
  --overwrite
```

After calibration, inspect the reported translation and rotation axis errors,
then validate in MuJoCo. Preserve the old calibration through Git/milestone
recovery before accepting the new file.

### Quest-free diagnostic

Dry validation:

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_six_axis_diagnostic.py
```

Physical diagnostic, only with explicit authorization:

```bash
env -u PYTHONPATH .venv-arm18/bin/python scripts/run_six_axis_diagnostic.py \
  --live \
  --confirm-live LIVE-WIDOWXAI-192.168.1.2
```

## Tagged Step-3 rollback profile

`configs/step3_velocity_feedforward_25ms_hardware.yaml` currently means:

- left Touch Plus controller;
- clutch/deadman on left grip;
- full gripper control on left trigger;
- nominal Quest-synchronized 90 Hz updates;
- each unique Quest pose is consumed once;
- minimum command interval of 8 ms placed immediately before final send;
- pose EMA alpha `0.8`;
- speed-adaptive rotation filtering;
- 30% of the article task-motion scale, represented by translation and rotation
  gains `0.45`;
- position reach `0.075 m` and rotation reach `0.18 rad`;
- decoupled IK: joints 0–2 position the wrist anchor and joints 3–5 orient it;
- a 25 ms driver position interpolation horizon;
- filtered, half-gain velocity feedforward with conservative per-joint caps;
- full gripper stroke, not 30%-scaled;
- feedback compared with a delayed/interpolated command reference;
- two-second measured-pose-to-home startup;
- separate two-second blocking gripper open;
- two-second blocking return to all-zero rest on exit;
- separate gripper close after the arm return;
- hard physical limits, MuJoCo collision screening, stale tracking rejection,
  and a maximum live duration.

The user asked for article-like freedom but observed physical table shaking when
all shaping was removed. Step 3 is the measured compromise: short enough driver
horizon for responsiveness, no queued catch-up bursts, and limited
velocity-feedforward so adjacent legacy-driver segments do not stop and restart
as sharply.

## Architecture and article correspondence

The live path is:

```text
Quest controller
  -> WebXR frame callback
  -> persistent WebSocket over USB ADB reverse
  -> capacity-one/latest-state relay
  -> clutch-relative task mapping and pose filter
  -> decoupled WidowXAI IK
  -> final command-spacing guard
  -> Trossen 1.8.6 position command with a short horizon
  -> physical arm
```

The article's controller does not need to "catch up" to an absolute robot pose.
On a grip edge, software records the current controller pose and the current
robot/tool pose. Subsequent controller deltas are applied relative to those two
anchors. Releasing grip holds the last safe command and permits repositioning.
Re-gripping reanchors without a jump. Stale tracking or a relay reconnect also
forces a zero-delta reanchor before motion can resume.

The rest/home lifecycle is intentionally project-specific:

```text
all-zero rest
  -> validated blocking ramp to [0, 60, 75, -60, 0, 0] degrees
  -> article-derived relative teleoperation while grip is held
  -> hold on grip release/stale input
  -> validated blocking return to all-zero rest on exit
```

The old XRoboToolkit ZIPs were used only to recover the lifecycle strategy. They
are not runtime dependencies and must not replace the article-derived control
path.

The workstation remains necessary in the current design. The Quest browser only
produces WebXR poses and button state; the PC owns calibration, filtering,
WidowXAI kinematics, collision/safety checks, telemetry, and the official
Trossen Python driver. Removing the PC would require a supported native Quest
application that ports the entire control/safety stack and an arm protocol
client. Sending browser WebSocket values directly to the arm would discard the
validated gates and is not an approved shortcut.

The pinned model sources are:

- `TrossenRobotics/trossen_arm_mujoco` at
  `77aba5d32654f17945f139e63310e7a3ac2cd4ab`;
- `TrossenRobotics/trossen_arm_description` at
  `21d8b360c211c2ad8a065d8f462cbec0207626e7`.

## Latency: what is measured and what is estimated

The article's transport table reports headset-to-workstation round-trip time,
not controller-motion-to-physical-arm-motion latency:

| Connection | median | mean | p95 | max |
|---|---:|---:|---:|---:|
| USB cable | 1.6 ms | 1.8 ms | 3.5 ms | 3.8 ms |
| LAN | 7.6 ms | 32 ms | 130 ms | 144 ms |

The user's earlier local `xr-widowxai-experiments` project achieved approximately
80 ms best end-to-end behavior. This motivated the article-style latest-state
USB path.

Two avoidable phase waits were found in this project:

- approximately 8 ms in a browser send timer after WebXR capture;
- approximately 8 ms waiting after IK before the driver send.

The browser now sends directly from the WebXR frame callback. The PC consumes
the freshest unique sample and places pacing immediately before the final send.
In the best 60-second run, browser capture-to-send timing was approximately
0.38 ms median, PC arrival-to-consume approximately 0.05 ms median, and IK
approximately 0.66 ms median. Earlier project analysis summarized the
capture/PC/IK/send software path around 3.29 ms median, 4.05 ms p95, and
5.11 ms p99 for the accepted trace.

The commonly quoted approximately 29–34 ms "total" is an estimate:

- measured software timestamps cover WebXR capture/send, relay arrival, IK, and
  driver-command send;
- a 25 ms driver interpolation horizon is added as a physical-response proxy;
- Quest tracking exposure/prediction age, motor/servo response onset, and
  encoder-to-observer perception are not directly measured;
- Quest and PC monotonic clocks are not inherently the same clock;
- no synchronized high-speed camera was used.

Therefore do not call the estimate a camera-verified physical end-to-end result.
Use "estimated controller-capture-to-command/driver response" and report p50,
p95, p99, and max separately. p95/p99 are naturally higher than the median
because scheduler, USB, tracking, IK, and driver stalls affect tail samples.

Reducing the driver horizon below 25 ms or raising the command rate does not
guarantee lower physical latency. On this legacy controller it increased
segment replanning, wrist reversals, table vibration, and audible jerks. A
slightly longer horizon can be both smoother and perceptually more responsive
when it avoids stop/start pulses.

The driver horizon is the time given to the arm controller to interpolate from
its current trajectory state to each newly commanded joint target. Commands
arrive before the previous horizon finishes, so the controller continuously
replans. A long horizon generally smooths but adds following lag; a very short
horizon can reduce nominal lag but amplify tiny target reversals. Trossen also
requires at least 300 Hz when interpolation is disabled with `goal_time=0`;
this Python/WebXR path does not guarantee that rate. Firmware `1.8.3` and driver
`1.8.6` are a confirmed matching pair. A newer firmware was discussed but never
assumed to be a vibration fix and must not be flashed automatically.

## Headset, passthrough, and tabletop operation

The Quest browser normally suspends tracking when the proximity sensor reports
that the headset is not worn. `scripts/restart_localhost.sh` applies the ADB
mounted/proximity override and restores USB forwarding. The override is lost on
a headset reboot.

The override keeps the session awake; it cannot manufacture optical controller
tracking. Touch Plus controllers must remain visible to the headset cameras.
When the headset was placed flat or pointed away, controller coordinates became
wrong or large fresh pose discontinuities appeared. The article's author
removed the headset by pulling it down to the chest, keeping it upright and
facing the controllers, and used a proximity-sensor workaround. It was not
arbitrarily placed on a table.

Commit `95e04a1` locks operator heading for the WebXR session so setting the
headset down does not rotate robot-forward. It does not correct a fresh optical
pose jump. Recommended tabletop setup:

- enter passthrough while facing robot-forward;
- keep the headset upright;
- point front tracking cameras toward the controller workspace;
- do not reposition it while grip is held;
- release grip before any headset/controller tracking interruption;
- prefer wearing the headset for calibration and first physical acceptance.

One accepted 60-second worn-headset run later showed a fresh approximately
42.3 mm controller-pose discontinuity near 14.661 s and an approximately
3.89-degree joint-command jerk. Fresh discontinuities are different from stale
tracking and need an explicit discontinuity policy if tabletop use remains a
goal.

## Guided six-axis calibration

The guided calibration fits a proper Quest-to-robot rotation using two repeats
of six gestures:

1. move right;
2. move up;
3. move robot-forward;
4. screw/twist;
5. nod yes;
6. nod no.

Translations should be about 10–15 cm with minimal rotation. Rotations should
keep the wrist pivot still. The operator must wear the headset and face the
direction that should mean robot-forward. The script is Quest-only.

The saved left calibration from 2026-07-22 had good translation fitting
(approximately 4.79-degree median and 8.66-degree maximum axis error) but weaker
rotation fitting (approximately 12.66-degree median and 15.98-degree maximum).
It worked well enough for the earlier milestone but was not accepted after the
later session/frame changes.

Temporary right calibration was added at:

```text
configs/calibrations/temporary_right_guided_6dof.json
```

Its observed quality was approximately:

- translation median 6.28 degrees, maximum 8.07 degrees;
- rotation median 7.83 degrees, maximum 21.47 degrees;
- determinant approximately `+1`, so the fitted transform was a proper
  rotation rather than a reflection.

The weakest right sample was a screw/twist repeat. MuJoCo and a physical launch
worked, but the user rejected the right mapping. Do not infer that a low median
alone makes the mapping correct; validate each signed direction and rotation.

## Development chronology from the full conversation

### 1. Prior experiment and article selection

- The user presented Aurel Arnold's article and the earlier local
  `xr-widowxai-experiments` repository.
- The earlier Quest 3/WidowXAI attempt worked but felt imperfect; the best
  wireless end-to-end result was about 80 ms.
- The article looked much smoother because it used persistent WebSocket
  transport, latest-state semantics, relative clutching, decoupled IK, and a
  high-rate control loop.
- The conclusion was that the article architecture could improve smoothness and
  software latency, but its low transport RTT did not itself prove low physical
  end-to-end latency.

### 2. Windows handoff and repository

- A clean-project handoff was prepared for a second Windows laptop.
- The project was pushed to the user's personal GitHub repository:
  `ApurvK032/aurel-vr-teleop-widowxai`.
- The user initially kept GitHub CLI access limited to the personal repository.
  During the later release pass, the official GitHub CLI was authorized for the
  private `sys3-lab` organization repository.
- Windows MuJoCo execution worked.
- Trossen documentation did not support the WidowXAI Python driver on Windows,
  so physical execution moved back to this Ubuntu PC.

### 3. Article-faithful MuJoCo baseline

- The cloned project was compared with the article/reference repository.
- An early 50 Hz adaptation felt slow and did not follow correctly. The user
  explicitly requested that no unrelated behavior be invented; the article
  workflow should be used for WidowXAI.
- The article-style 200 Hz IK/latest-state path was restored for simulation.
- MuJoCo looked good, confirming mapping/IK logic but not physical motor
  smoothness.

### 4. Quest/LAN/arm discovery

- Quest USB and LAN were attached through a hub.
- The arm was visible at `192.168.1.2`; the PC was observed at `192.168.1.3`.
- Connectivity was inspected without arm motion.
- Necessary legacy-driver and lifecycle details were taken from the previous
  WidowXAI experiment rather than changing any arm-side setting.

### 5. Rest and home discovery

- Several prior project folders and `trossen_arm_description` were inspected.
- The decisive source was the earlier GELLO/WidowXAI
  `leader-arm-redesign-plan.md` and its parent project.
- The confirmed teleoperation home was `[0, 60, 75, -60, 0, 0]` degrees.
- All-zero joint position was confirmed as rest.
- Early manual home/rest commands were issued only after explicit user
  confirmation. The lifecycle was then built into the launcher.

### 6. First safe physical profile

- A 30%-scale physical demo was created and tested in MuJoCo first.
- Initial startup from rest hit a joint-limit check because encoder noise
  reported a tiny negative value such as `-0.004768` at a zero boundary. The
  check was made tolerant to measured/command numerical noise without removing
  physical joint limits.
- The arm successfully ramped to home.
- A first gripper-enabled attempt stopped because `-0.000000` fell just outside
  `[0.000000, 0.040000]`; later attempts hit the configured per-tick gripper
  cap. Boundary clamping and trigger-transition handling were corrected.
- Startup gripper tracking checks also falsely failed while the arm was moving.
  The proven strategy was restored: move J0–J5 to home, then issue one separate
  two-second blocking gripper-open move.
- The second `ARM READY` prompt was removed at the user's request. The explicit
  CLI live token remains the authorization gate.
- Shutdown was changed from leaving the arm at its last teleop pose to a
  validated two-second blocking return to all-zero rest, followed by separate
  gripper cleanup.

### 7. Scaling and full gripper

- A 50%-scale profile with full gripper was tested in MuJoCo and on hardware.
- The gripper was intentionally not scaled to 30% or 50%; trigger commands the
  full open/close stroke at its normal safe command rate.
- The user ultimately preferred 30% task-motion scale while latency and
  smoothness were optimized.

### 8. Article-rate physical behavior and vibration

- Removing most shaping and following the raw article loop on hardware made the
  arm highly responsive but caused table shaking, pulsed motion, audible wrist
  noise, and minute strong jerks.
- The effect was not visible in MuJoCo because MuJoCo directly displayed
  commanded joints rather than modeling the legacy driver's replanning and
  motor dynamics.
- The 200 Hz loop and very short driver horizon repeatedly replanned small
  position segments. High-frequency reversals were especially visible in later
  wrist joints.
- Frequency, driver horizon, command spacing, feedback alignment, and motion
  shaping were then changed one variable at a time.

### 9. Latency instrumentation and removal of two waits

- A camera/high-speed external measurement was considered but rejected as too
  much setup.
- Software telemetry was expanded with Quest capture/send, relay arrival,
  consume, IK, driver send, feedback, and sequence information.
- The approximately 8 ms browser timer and approximately 8 ms post-IK phase wait
  were removed.
- The direct send path made MuJoCo and hardware feel markedly more real-time.
- The user repeatedly requested p50/p95/p99 interpretation. Higher p95/p99 were
  explained as tail behavior, not an error in percentile ordering.

### 10. Smooth responsive profiles

- Quest-synchronized consumption replaced blindly solving repeated samples at a
  fixed high rate.
- Each unique Quest sample is filtered once.
- Adaptive rotation filtering preserves fast intentional wrist motion while
  damping small reversals.
- A no-catch-up scheduler prevents a late loop from emitting queued command
  bursts.
- Command spacing moved to the final send barrier.
- Feedback comparison was aligned to a delayed command reference.
- Conservative velocity feedforward reduced stop/start behavior between
  short legacy-driver segments.
- A transient `NameError: control is not defined` in the no-catch-up hardware
  launcher was fixed after it caused an immediate safe return to rest.

### 11. Gripper shutdown fixes

- Pressing trigger initially stopped the robot because closed mapped to a tiny
  negative floating-point command.
- After boundary clamping, a full trigger step still violated a per-tick cap.
- The gripper path was changed so full trigger travel remains available while
  normal command-rate limiting prevents a one-tick discontinuity.
- These stops were launcher safety stops, not arm-controller shutdowns.

### 12. Direction calibration

- The required motions were explicitly identified as left/right, up/down,
  forward/back, screw, nod yes, and nod no.
- A guided task-frame calibration script was built so mapping could be measured
  rather than guessed from axis signs.
- A calibration regression where `capture_gesture()` was called without the
  new `hand` argument caused a `TypeError`; it was fixed and covered by tests.
- Left mapping became good enough to continue tuning, though not perfect.

### 13. Quest-free physical diagnostic

- A six-axis deterministic diagnostic was added to distinguish teleoperation
  input jitter from driver/arm behavior.
- The first motions were too small to judge, so amplitudes were increased to
  representative 40 mm translations and 10-degree rotations.
- One diagnostic completed motion but the TCP connection closed during return
  to rest and a loop overrun of about 193 ms was reported. The launcher printed
  emergency shutdown warnings as designed.
- At a 30 ms horizon the sharp jerk sound disappeared, but visible shaking
  remained.
- Minimum-snap trajectories, no catch-up pacing, and smoother driver segments
  substantially improved the diagnostic. This showed that part of the
  vibration was in the command/driver/physical path, not Quest input alone.

### 14. Milestones

- `milestone-1` / commit `14f8cc4`: calibrated low-latency physical
  teleoperation with working gripper and lifecycle.
- `milestone-2-smooth-30ms` / commit `837407f`: smooth 30 ms no-catch-up
  baseline.
- `milestone-3-step-1-smooth` and `latency-step-1-8ms-guard` / commit
  `d49091f`: final-send 8 ms burst guard at smooth 30 ms behavior.
- Step 2 reduced the driver horizon to 25 ms and was validated physically.
- `milestone-3-step-3-best` / commit `521f55b`: 25 ms horizon plus conservative
  velocity feedforward; best balance of smoothness and latency.
- Commit `95e04a1`: session operator-heading lock for tabletop use.

### 15. Headset removal experiments

- ADB was used to keep the Quest session awake without continuously wearing the
  headset.
- Tracking only remained correct when the headset cameras faced the controller.
  Other placements changed the coordinate frame or caused large jerks.
- The article was rechecked: its headset was kept upright/facing the workspace,
  not placed in an arbitrary orientation.
- Heading lock fixed yaw-frame rotation but not loss of optical tracking or
  fresh pose discontinuities.

### 16. Temporary mirrored right controller

- Web client, transport, calibration, simulation, benchmark, and hardware
  launchers were generalized to select `left` or `right`.
- Right controller uses an independent wrist-offset storage key and right-hand
  haptics.
- `temporary_right_mirrored_step3_25ms_hardware.yaml` was created without
  replacing the left baseline.
- Guided right calibration completed; MuJoCo ran, and the physical launcher
  successfully completed a right-controller run.
- The user rejected the mapping. Returning to the left page also produced an
  unacceptable latest mapping, so calibration was deferred rather than
  hard-coding more guessed signs.
- On 2026-07-23 the new page-level `Mirror` mode was treated explicitly as a
  sagittal reflection. The tested task frame used
  `F = Q @ diag(-1, 1, 1)` and therefore had determinant `-1`; this canceled
  the page reflection and reproduced the captured Right/Real frame rather than
  producing mirrored task behavior.
- `right_mirror_axis_validation_mujoco.yaml` and
  `right_mirror_axis_validation_hardware.yaml` isolate this mapping at reduced
  `0.20` gains, `0.035 m`/`0.08 rad` reach, and no gripper commands.
- The operator checked all six motions across six live runs from
  `runs/2026-07-23/20260723-155250_right-mirror-axis-01-right` through
  `runs/2026-07-23/20260723-155505_right-mirror-axis-01-right` and reported the
  mapping perfect.
  The six summaries total 8,099 rows and zero IK failures.
- The accepted Behind/native-parallel transform is
  `configs/calibrations/right_behind_guided_6dof_20260723_accepted.json`.
  Its acceptance is limited to the exact reduced-gain validation profile.
  `right_mirror_guided_6dof_20260723_accepted.json` is preserved but marked as
  a superseded mislabel; true mirror semantics remain unvalidated.
- The four-way task profiles are indexed by
  `configs/calibrations/task_profiles.json`. With
  `configs/quest_50pct_hardware.yaml`, the first fresh,
  grip-released page sample maps Left/Behind to `left_behind`, Left/Mirrored to
  `left_mirror`, Right/Behind to `right_behind`, and Right/Mirrored to
  `right_mirror`; the internal transport values remain `real` and `mirror`.
  The transform is selected before any arm connection and
  locked for the run. `--task-profile` remains a strict manual override.
  Behind uses no task-axis sign changes. The operator-defined Mirrored mode uses
  position signs `[-1,-1,1]` and rotation signs `[-1,1,-1]`, so left/right,
  front/back, screw, and nod-no reverse. Each hand retains its own captured task frame. The
  shared profile enables full-stroke trigger control using the already
  validated Step-3 gripper lifecycle. After the successful bounded gripper run, the
  owner explicitly made only this shared profile duration-unbounded and then
  revised its task gain/reach from 70% to the current 50% of article values.
  Right/Mirror was accepted at 50% through the runs at `164957`, `165313`, and
  `171126` (12,585 total rows, zero IK failures). The other three catalog
  entries remain pending operator validation at this scale.

### 17. Controller TCP-server incident

- A generic raw TCP readiness probe connected to port `50001` and left an
  orphaned `FIN-WAIT-2` session.
- Subsequent official-driver runs hung after the TCP/UDP connection messages
  and before version output. No position mode or motion command had been sent.
- Stopping the process and waiting for the local timeout was insufficient
  because the controller server remained wedged.
- Power-cycling the controller cleared it. The next official-driver run worked.
- This is why raw arm-port probes are forbidden in this repository.

### 18. Repository hygiene

- All 132 existing runs, totaling approximately 1.1 GB, were preserved and
  moved losslessly into `runs/2026-07-21/` and `runs/2026-07-22/`.
- The local `runs/manifest.csv` indexes every run, category, status, row count when
  available, size, Git commit, artifacts, relative path, and selected milestone.
- New telemetry automatically writes to `runs/YYYY-MM-DD/`.
- `scripts/organize_runs.py` migrates legacy top-level runs and regenerates the
  manifest.
- The two prior XRoboToolkit ZIPs were retained under
  `archive/reference_implementations/` and remain ignored by Git.
- Obsolete Ubuntu and Windows handoff documents were removed after their
  current setup instructions were consolidated into `README.md`.
- Publishable milestone statistics moved to `results/major_runs.csv`; raw
  telemetry and the full manifest remain ignored local evidence.
- Generated Python bytecode, pytest cache, and editable-install metadata were
  removed from the project tree. Virtual environments were preserved.

## Known failure signatures and interpretation

| Message/symptom | Meaning and response |
|---|---|
| `waiting for fresh Quest tracking with grip released` | WebXR session/controller pose is not fresh, headset is asleep, wrong hand page is open, or grip is held. Do not bypass. |
| Hangs after TCP/UDP connection lines | Controller handshake/server problem. Stop; inspect local sockets without connecting; power-cycle if repeated. |
| Tiny negative joint command at zero boundary | Numerical/encoder noise. Clamp only within a small tolerance; retain physical limit. |
| `gripper command -0.000000 outside` | Floating-point boundary issue; clamp to configured physical range. |
| `gripper command exceeded per-tick cap` | Trigger discontinuity; rate-limit smoothly without reducing full stroke. |
| Startup gripper tracking error | Do not supervise arm and gripper as one simultaneous ramp on this legacy setup; use separate blocking gripper move. |
| Joint tracking error during live run | Could be reference timing rather than a stall. Compare to delayed/interpolated command, but never remove the hard stall threshold blindly. |
| Table vibration/audible wrist pulses | Usually short-horizon replanning, high-rate reversals, catch-up bursts, or noisy orientation commands. MuJoCo cannot validate this physical effect. |
| Random direction after headset movement | Coordinate-frame/calibration or optical tracking discontinuity, not necessarily IK failure. Release grip and recalibrate/validate. |
| `TCP connection closed unexpectedly` on shutdown | Controller/driver connection loss. Attempt measured hold only if possible; tell user to cut power. |
| `NameError: control is not defined` | Historical no-catch-up launcher regression; fixed. |
| Calibration `missing ... hand` | Historical right-hand generalization regression; fixed and tested. |

## Repository layout and ownership

```text
AGENTS.md                         authoritative full project handoff
README.md                         user-facing setup and architecture
archive/reference_implementations ignored XRoboToolkit ZIPs
configs/README.md                 stable/rollback/experimental catalog
configs/calibrations/             left baseline and temporary right fits
docs/                             dual-arm and CAD commissioning notes
results/major_runs.csv            compact publishable milestone metrics
runs/manifest.csv                 ignored local full run index
runs/YYYY-MM-DD/                  ignored telemetry payloads
scripts/organize_runs.py          lossless run migration/indexing
src/                              article-derived implementation
tests/                            offline acceptance suite
third_party/                      pinned official Trossen submodules
```

The CAD rest commissioning files are experimental and separate from the Quest
baseline:

```text
configs/cad_rest_commissioning.yaml
scripts/run_cad_sim.py
src/widowxai_quest_teleop/cad_input.py
tests/test_cad_input.py
```

Do not delete or merge them into the live Quest path without an explicit user
decision.

Right/Mirror at 50% is the current operator-accepted single-arm mapping. Preserve
the tagged left Step-3 rollback and the historical rejected/working calibration
records. Do not infer acceptance for the three remaining page profiles.

## Git and recovery

Publishing target:

```text
https://github.com/sys3-lab/vr-telop-widowxai
```

The historical personal remote remains:

```text
https://github.com/ApurvK032/aurel-vr-teleop-widowxai
```

At the start of the hygiene pass, local `main` was nine commits ahead of the
personal `origin/main`. GitHub initially rejected the organization push under
GH007 because ten commits used the user's private email address. The owner then
resolved the GitHub email-privacy gate and the private organization `main`
branch was confirmed at release commit `8647b3d`. Do not expose or copy OAuth
tokens into project files or documentation.

Recovery points:

```text
milestone-1
milestone-2-smooth-30ms
milestone-3-step-1-smooth
latency-step-1-8ms-guard
milestone-3-step-3-best
```

Run payloads, the local full manifest, virtual environments, certificates, and
historical ZIPs are not Git data. `results/major_runs.csv` is the compact
publishable record.

There may be user or experimental changes in a dirty worktree. Inspect
`git status` and `git diff` before editing. Never discard or reset unrelated
work. Make scoped commits; do not bundle CAD, right-controller experiments, and
stable baseline changes without explaining the scope.

## Verification requirements

For code/document hygiene changes:

```bash
env -u PYTHONPATH PYTHONDONTWRITEBYTECODE=1 \
  .venv/bin/python -m pytest -q -p no:cacheprovider
```

For the legacy hardware environment without opening the arm:

```bash
env -u PYTHONPATH PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .venv-arm18/bin/python -m pytest -q -p no:cacheprovider
```

Tests do not authorize a physical connection. Avoid commands that instantiate
`TrossenArmBackend` unless the user requested a preflight or live run.

After a new run:

```bash
env -u PYTHONPATH .venv/bin/python scripts/organize_runs.py
```

Then inspect `runs/manifest.csv`, the exact config snapshot, summary row count,
IK failures, freshness/reanchor events, command spacing, and tail latency before
changing another variable.

## Dual stabilization log

### 2026-08-11 — Milestone 1: bimanual latest-state relay

- Work is isolated on the local `dual-stabilization` branch; the accepted
  single-arm profiles and launchers were not changed.
- Root cause: the relay recognized only the schema-v1 `pose` type as control
  state. Schema-v2 `bimanual_pose` messages fell through to the 32-entry event
  FIFO, received no relay timestamps or reconnect generation, and were absent
  from pose health counts. Under backpressure, obsolete bimanual frames could
  therefore queue and still look fresh when the PC finally parsed them.
- `pose` and `bimanual_pose` now share the capacity-one latest-state path. Both
  receive relay monotonic/epoch timestamps and the source connection generation.
  `/health` retains the aggregate `pose_messages` count and adds per-type counts.
- `scripts/smoke_test_relay.py` now uses an ephemeral loopback port and exercises
  a real relay, a raw subscriber, the actual `BimanualQuestReceiver` mailbox,
  both schemas, and a browser/source reconnect. Unit coverage pins capacity-one
  overwrite behavior and rejects accidental routing of unknown event types.
- Verification: focused relay/transport suite `20 passed`; standalone relay
  smoke passed; full `.venv` suite `204 passed`; full no-arm `.venv-arm18` suite
  `204 passed`. No Quest or arm was connected and no physical command was sent.
- Next milestone: make dual feedback freshness/timestamps truthful and preserve
  the faulting joint sample before a safety-stop exception leaves the loop.

### 2026-08-11 — Milestone 2: truthful dual feedback telemetry

- `scripts/run_dual_hardware.py` now records an encoder-read timestamp only
  after that arm's backend actually returns feedback. Every control row starts
  with `feedback_sample_fresh=false` and a zero read timestamp, and only an arm
  read during that row is marked fresh. The previous code incorrectly wrote
  each arm's next scheduled deadline as though it were the completed read time
  and marked both arms fresh on every non-idle row.
- Quest-synchronized idle cycles now reach the common telemetry path. They can
  perform due encoder checks, but report zero command-send timestamps instead
  of reusing the preceding command's timestamps. This preserves the distinction
  between an idle observation and a command-bearing tick.
- A time-aligned tracking failure now retains the measured feedback, reference,
  all-joint error, actual read timestamp, and exception text in the final CSV
  row before the existing fail-closed exception is re-raised. If the first arm
  faults, the other arm remains explicitly not-read rather than receiving a
  false freshness claim.
- The conservative `hardware.max_feedback_error_rad: 0.08` safety threshold was
  not changed. This milestone makes the next joint-2 event diagnosable; it does
  not assume that the prior 0.101761 rad event was either genuine lag or a bad
  reference.
- Offline verification: focused dual/hardware suite `104 passed`; full `.venv`
  suite `209 passed`; full no-arm `.venv-arm18` suite `209 passed`. No Quest or
  arm was connected and no physical command was sent.
- Next milestone: use the newly preserved evidence to test the time-aligned
  feedback reference and hold behavior offline, especially the distinction
  between an interpolated reference and one clamped to the newest command.

### 2026-08-11 — Milestone 3: reference-state and hold evidence

- Each arm's fresh feedback row now records
  `feedback_reference_state`, `feedback_newest_command_age_ms`, and
  `feedback_history_span_ms`. The state is one of `clamped-to-oldest`,
  `interpolated`, or `clamped-to-newest`; rows without an encoder read leave
  these per-read fields empty. This makes an error trend diagnosable before it
  crosses the stop threshold and avoids relying only on exception prose.
- Offline tests pin the 25 ms delayed-reference boundaries and prove an
  interpolated example: a read at 10.000 s resolves to the 9.975 s command
  between commands sent at 9.950 and 9.980 s. They also prove that a fault at
  the newest boundary reports `clamped-to-newest`.
- Hold semantics are now regression-pinned: losing one controller's tracking
  preserves that arm's last committed command while the other arm remains
  independently usable; a combined collision or external arm fault rejects
  the entire tick and preserves both arms' last committed commands. No rejected
  proposal becomes authoritative.
- The 25 ms delay and 0.08 rad stop threshold were not changed. A future
  physical fault row can now show whether joint 2 was compared with a genuine
  interpolated command or with the final newest command after the stream went
  idle.
- Offline verification: focused dual/hardware suite `105 passed`; full `.venv`
  suite `210 passed`; full no-arm `.venv-arm18` suite `210 passed`. No Quest or
  arm was connected and no physical command was sent.
- Next milestone: replace placeholder base transforms with measured bench
  geometry before treating any cross-arm clearance result as physical evidence.

### 2026-08-11 — Milestone 4: placeholder base geometry fails closed

- Root cause: both dual YAML profiles and all operator documentation called the
  500 mm base transforms placeholders, but the live configuration gate checked
  only that transforms existed and differed. Supplying the explicit
  calibration-only override could therefore reach live setup while collision
  checks still used unmeasured bench geometry.
- Every configured arm base now has an explicit `measurement_status` of
  `placeholder` or `measured`. Both shipped dual profiles remain honestly
  marked `placeholder`; simulation and no-contact preflight continue to work,
  while live dual output requires `measured` for both arms. There is no geometry
  override, and the calibration override neither bypasses nor records acceptance
  for placeholder bases.
- The no-contact preflight prints each arm's base status and reports all current
  live blockers. With the shipped Behind profile it still clears the synthetic
  rest/home and one-arm-moving paths at the placeholder 0.500 m separation, but
  labels those results as placeholder evidence and contacts no controller.
- README, configuration catalog, and dual-arm extension instructions now show
  the required status field and the measured position/quaternion structure.
- Offline verification: focused dual/config/hardware suite `107 passed`;
  full `.venv` suite `212 passed`; full no-arm `.venv-arm18` suite `212 passed`;
  `preflight_dual_hardware.py` passed without `--contact-arms`. No Quest or arm
  was connected and no physical command was sent.
- Next physical prerequisite: measure each base position and orientation in one
  shared world frame. Replace the placeholder numbers and set `measured` only
  when that measurement has actually been recorded.

## Next planned work

1. Preserve the accepted Right/Mirror 50% profile as the single-arm baseline.
2. Measure transport RTT/clock offset and repeated controller/encoder reversal
   events before claiming physical p95/p99 latency.
3. Validate Left/Behind, Left/Mirror, and Right/Behind independently before
   marking them accepted at 50%.
4. The dual-arm architecture in `docs/DUAL_ARM_EXTENSION.md` is implemented:
   bimanual WebXR packet, combined MuJoCo scene, per-arm runtime, coordinated
   controller, dual telemetry, and three launchers, with offline tests. What
   remains is operational: measure both arm base transforms (the shipped
   500 mm values in `configs/dual_widowxai.yaml` are placeholders), then
   physically accept a per-hand calibration for each arm.
5. Two-arm driver output stays configuration-blocked until each arm has an
   explicitly accepted calibration. `require_live_dual_arm_config` enforces
   this, and the dual token `LIVE-WIDOWXAI-DUAL-<left-ip>-<right-ip>` is
   separate from the single-arm token by design. Do not weaken either gate to
   make a two-arm run launch.
6. At 300 mm base separation the two arms collide when yawed roughly 0.25 rad
   toward each other, and the 30 mm clearance margin rejects at about
   0.20 rad. Both figures come from the 300 mm fixtures in
   `tests/test_dual_arm_safety.py` and no longer describe the configured
   bench, which the operator reports at 500 mm. Re-derive them at the measured
   separation before expecting overlapping bimanual tasks.
7. `scripts/run_cad_sim.py` passes 13 telemetry keys that `TELEMETRY_COLUMNS`
   never declared, so they have always been silently discarded. This is
   pre-existing and left unchanged; `tests/test_dual_arm_safety.py` pins it as
   a known exception so it cannot spread.

The main optimization objective remains: preserve the approximately 30 ms
estimated controller-capture-to-encoder response and physical smoothness while
tightening tail behavior. Do not trade away validated smoothness merely to
lower one median software number.
