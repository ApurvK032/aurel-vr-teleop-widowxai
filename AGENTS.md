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

Right/Behind is physically accepted on arm `.3` at 40% scale with full gripper
control, recorded in
`configs/calibrations/right_behind_all_motions_20260812_accepted.json`.
Left/Behind is physically accepted on arm `.2` at 40% scale for all six signed
motions, recorded in
`configs/calibrations/left_behind_all_motions_20260812_accepted.json`; its
gripper was not commanded. Left/Mirror is physically accepted at 45% for the
front-facing dual profile, recorded in
`configs/calibrations/left_mirror_all_motions_20260812_accepted.json`; its
gripper was also not commanded. Both Behind mappings remain untested at 50%;
none of these facts overrides the independent Right/Mirror 50% acceptance.

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
| PC robot-LAN address | `192.168.1.10/24` on `benfei-lab-lan` (no gateway) |
| Left/right arm IPs | `192.168.1.2` / `192.168.1.3` |
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
docs/PROJECT_PROGRESS.md          concise status/performance/blocker ledger
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

### 2026-08-11 — Milestone 5: provisional 500 mm tabletop simulation

- `scripts/run_dual_sim.py --tabletop` now builds the requested simulation-only
  scene with the existing parallel bases at `y = +/-0.250 m` (500 mm
  center-to-center). Default provisional table dimensions are 1.000 m wide,
  0.700 m deep, and 0.040 m thick, with the base centerline 0.100 m forward of
  the rear edge. All four assumptions have command-line overrides and are
  written into the telemetry config snapshot.
- The tabletop is a collidable MuJoCo box, not only viewer decoration.
  `DualArmCollisionModel` now classifies non-base arm/table contact as an
  `environment` collision, identifies the affected side and body, and feeds it
  through the existing coordinated rejection path. The fixed base/table
  support relationship is intentionally allowed.
- Offline checks prove rest and home are clear with the provisional table and
  that a deliberately lowered link is rejected for tabletop penetration. The
  combined home pose retains approximately 60 mm arm-to-arm separation against
  the configured 30 mm cross-arm margin.
- README and dual-extension instructions include the exact Quest-driven viewer
  command and clearly separate the provisional table assumptions from measured
  physical geometry.
- Verification: focused tabletop/dual suite `52 passed`; full `.venv` suite
  `215 passed`; full no-arm `.venv-arm18` suite `215 passed`. The Quest was
  authorized over USB, ADB reverse and the relay were started, and the
  MuJoCo-only viewer reached a clear home pose. No robot backend was opened and
  no physical command was sent.
- To drive this scene, select `Bimanual` on the Quest page and enter its WebXR
  session. A USB connection alone creates the localhost tunnel but does not
  start controller pose transmission.

### 2026-08-11 — Milestone 6: isolated bimanual MuJoCo calibration path

- The operator reported that all controller directions in the provisional
  dual MuJoCo run were wrong and requested a simulation-specific remap that
  must not enter a real test. `scripts/calibrate_task_frame.py` can now project
  one explicitly selected controller from schema-v2 `bimanual_pose` packets by
  using `--bimanual-hand left|right --mapping-mode real|mirror`.
- Bimanual calibration requires `--simulation-only`. Its output records
  `validation_scope.status: simulation_only`,
  `validation_scope.hardware_accepted: false`, and
  `physical_validation.status: simulation_only`, so the existing live
  calibration gate cannot infer hardware acceptance.
- Explicit untracked packets are skipped; no stale controller pose can become
  a calibration sample. Initial tracking and operator-start waits are long
  enough for a supervised headset workflow. Calibration pulses carry the
  target hand, and the web client routes them to that controller in bimanual
  mode instead of relying on the single-arm page selection.
- Verification: focused calibration/transport suite `28 passed`; full `.venv`
  suite `217 passed`; full no-arm `.venv-arm18` suite `217 passed`. The first
  guided left-hand session reached live tracking but timed out before any grip
  press, so no calibration JSON or capture log was created. Do not invent a
  matrix: resume with the operator present, capture all six signed gestures
  twice per hand, and only then create the hardware-disabled MuJoCo profile.
- The existing `configs/dual_widowxai.yaml`, all accepted/candidate physical
  calibration files, and every physical launcher remain unchanged by this
  milestone.

### 2026-08-11 — Milestone 7: calibrated dual MuJoCo mapping validated

- The owner completed all twelve guided left-controller captures (right, up,
  forward, screw-clockwise, nod-down, and nod-left, each twice). The resulting
  proper task frame is
  `configs/calibrations/left_behind_mujoco_20260811.json`: translation median
  error 5.2 degrees / maximum 10.3 degrees and linked-rotation median 9.6
  degrees / maximum 15.2 degrees. It remains explicitly `simulation_only`.
- At the owner's request, no independent right-hand capture was performed.
  `right_behind_mujoco_20260811_derived.json` uses the same matrix because both
  `real` controllers are polar/axial deltas in one Quest world frame and both
  MuJoCo arms have the same local task-axis convention. The file declares an
  identity hand transfer, names its left source, has no right capture samples,
  and remains explicitly `simulation_only`.
- `configs/dual_widowxai_mujoco_calibrated.yaml` is the only profile referencing
  these files. Both `hardware.enabled` and `hardware.require_explicit_enable`
  are false, the physical dual profile is unchanged, and the conservative
  0.45 gains / 0.075 m / 0.18 rad envelope was retained for signed-axis
  validation.
- The owner described both simulated arms as good and controllable. Evidence is
  `runs/2026-08-11/20260811-181254_dual-500mm-mujoco-calibrated`: 10,211 rows
  over 114.878 s at 88.877 Hz, every consumed Quest sequence unique, and zero
  IK failures across 3,818 left-active and 3,849 right-active ticks. On
  collision-free active ticks, position residual p95 was 3.8 mm left / 4.6 mm
  right and orientation residual p95 was 0.00013 rad / 0.00028 rad.
- Neither simulated arm approached a joint limit: minimum active margins were
  0.315 rad left and 0.467 rad right. Motion shaping engaged briefly and as
  intended (joint-velocity flags on 79 left and 138 right active ticks; one
  right joint-jerk flag), without a stop. Command skew was 0.006 ms median,
  0.020 ms p95, and 2.876 ms maximum, below the 10 ms configured bound.
- Five deliberate arm-to-arm approaches generated 218
  `cross-arm-clearance` rejections. Every collision row held both sides, and
  both arms' commanded joints changed by exactly 0.0 rad from the pre-collision
  command and throughout every rejection episode. No tabletop/environment
  collision and no non-collision fault occurred.
- Verification: focused calibration/dual suite `77 passed`; full `.venv` suite
  `219 passed`; full no-arm `.venv-arm18` suite `219 passed`. This validates the
  mapping, latest-state transport, IK, shaping, tabletop scene, and coordinated
  rejection in MuJoCo. It does not validate encoder feedback timing, the joint-2
  physical tracking fault fix, real base geometry, or any live driver behavior.

### 2026-08-12 — Milestone 8: isolated dual-D405 operator views

- Two D405s and the Quest were initially confirmed on this workstation at USB
  SuperSpeed (5 Gb/s negotiated). Quest ADB sustained approximately 47 MB/s in
  a 128 MiB one-way transfer, comfortably above the first camera milestone's
  measured compressed rate.
- `camera_service.py` discovers the D405 V4L2 YUYV color nodes, assigns stable
  `scene` and `wrist` roles (or explicit ASIC serials), runs one FFmpeg capture
  process per camera at 640 × 480 × 30 FPS, and keeps only the latest complete
  JPEG for each role. It exposes health/snapshot endpoints and independent
  binary WebSockets on port 8444; it never imports or contacts an arm backend.
- `restart_localhost.sh --with-cameras` now owns both services and creates ADB
  reverse mappings for 8443 and 8444. The default command remains camera-free.
  Shutdown waits for the relay and camera process so their FFmpeg children do
  not retain V4L2 devices.
- The Quest page shows two live previews and renders stereo-correct panels at
  finite depth inside immersive passthrough. The first Quest acceptance run
  confirmed both feeds and all three B-button states. Follow-up operator input
  replaced the initial head-following transform with a session-fixed
  `local-floor` anchor, enlarged and lowered the overview panels to sit just
  above center, and added a 380 ms eased position/size transition between
  overview, scene focus, and wrist focus. A view change is rejected with haptic
  feedback while either tracked grip is held. The non-primary feed remains
  visible as a small preview.
- Before the direct-port camera disconnected physically from USB, simultaneous
  five-second transport validation delivered 151 scene frames at 30.06 FPS and
  152 wrist frames at 30.21 FPS. Mean JPEG sizes were 30.0 KiB and 23.1 KiB.
  The camera health endpoint remained fresh and the separate pose relay stayed
  responsive. Normal-browser verification showed both real feeds, all three
  view-cycle states, and no application JavaScript error; a favicon-only 404
  was removed afterward.
- The kernel recorded an actual `usb 4-2: USB disconnect` for D405 RealSense
  serial `218622273557` / ASIC serial `235123071818` during an earlier restart.
  After reconnection, both D405s negotiated 5 Gb/s again, both WebSockets opened
  in Quest Browser, and the owner reported the two-feed passthrough workflow as
  working well. The revised world anchor/layout animation still requires the
  immediate headset feel-check before it is treated as accepted polish.
- Verification after the browser fixes: focused camera/relay/transport suite
  `34 passed`; full `.venv` suite `226 passed`; full no-arm `.venv-arm18`
  suite `226 passed`; JavaScript, Python, shell syntax, and `git diff --check`
  passed. No robot backend was opened.
- This milestone is an operator-display prototype. RGB video freshness is not
  yet connected to a physical-motion hold, and no physical arm was contacted.

### 2026-08-12 — Milestone 9: configurable three-camera operator layout

- The fixed `scene`/`wrist` camera pair is generalized to three logical roles:
  `scene`, `left_wrist`, and `right_wrist`. The camera service discovers
  physical V4L2 640 × 480 × 30 FPS YUYV color devices (including RealSense
  D405/D455), exposes their stable serials through `/configuration`, and starts
  FFmpeg only for roles that are both assigned and enabled.
- The Quest page now has one device dropdown and **Show in passthrough**
  checkbox per role. The same physical serial cannot be assigned twice.
  Selecting no camera forces that role off; unchecking a selected role stops
  its capture while retaining the assignment. **Apply camera setup** is
  unavailable during WebXR, so a feed cannot be silently remapped during live
  clutch operation.
- Successful assignments are persisted outside the repository at
  `${XDG_CONFIG_HOME:-~/.config}/widowxai-quest-teleop/cameras.json`. The first
  run preserves the original two-camera behavior by assigning stable camera 1
  to Scene and camera 2 to Right wrist. Browser/API changes thereafter require
  no code edit and no camera-service restart.
- Passthrough adapts to zero, one, two, or three enabled roles. Zero produces
  no panel/B action; one produces one larger overview panel; two and three add
  every enabled role to the B-button cycle. Focus mode keeps one or two small
  previews, and the existing session-fixed world anchor plus 380 ms eased
  transitions remain in use. After the operator found the three-view layout
  obstructed the direct passthrough workspace, one shared vertical offset was
  set to `+0.18 m` for overview, focused panels, and previews; preserve the
  single offset rather than independently drifting layouts during later tuning.
- The camera layer remains isolated from `/ws` pose traffic and from all robot
  backends. RGB loss still displays status only; it is not a physical-motion
  hold gate.
- Focused verification covers normalized three-role configuration, duplicate
  rejection, persisted restore, enabled-only process startup, first-run
  migration, dynamic UI controls, and existing transport primitives. Both full
  offline suites pass at 229 tests; JavaScript, Python, shell, and diff checks
  pass. Live browser validation discovered the two D405s plus the built-in UVC
  webcam, prevented duplicate dropdown choices, enabled/disabled the checkbox
  with selection, persisted an unchanged Apply, and cycled Overview → Scene →
  Right wrist → Overview with no console warning/error. The final workstation
  uses one D455 plus two D405s for Scene/Left wrist/Right wrist; the owner
  accepted all three feeds, focus cycling, world anchoring, smooth transitions,
  and the later +0.18 m common height adjustment. Final health was 29.970,
  30.019, and 30.004 FPS respectively. The approximate current compressed
  payload is 2.3 MB/s before WebSocket overhead. Glass-to-glass latency and
  controlled three-stream impact on active WebXR pose rate remain unmeasured.

### 2026-08-12 — Dual-arm LAN visibility confirmed

- The old single-arm workstation profile placed the PC at `192.168.1.3/24`,
  which collided with the configured right-arm address. Pinging `.3` therefore
  looped back to the PC and did not test the right controller.
- The `benfei-lab-lan` NetworkManager profile now uses the unused static address
  `192.168.1.10/24`, has no gateway, and is marked never-default so Wi-Fi keeps
  the workstation's default route.
- After the change, both controllers responded independently with zero packet
  loss: left `192.168.1.2` / MAC `04:e9:e5:19:15:13`, right `192.168.1.3` / MAC
  `04:e9:e5:1b:23:ce`. The USB Ethernet link negotiated 1 Gb/s full duplex.
  Only ICMP and ARP visibility were checked; no driver session or motion command
  was opened.
- The shared USB-C dock briefly reset and removed its Ethernet adapter and both
  D405s together; kernel logs showed the entire hub disconnect/re-enumerate.
  NetworkManager automatically restored `.10`, and both arms were reachable
  again afterward. Treat another simultaneous LAN/camera disappearance as a
  dock/cable/power event rather than three independent device failures.

### 2026-08-12 — Milestone 10: physical dual-base geometry recorded

- The owner measured the two bases exactly 500 mm center-to-center, with no
  forward/back or vertical offset, equal height, parallel forward axes, and no
  relative yaw. Both base centers are exactly 2 in (50.8 mm) forward of the
  table's rear edge.
- The canonical shared frame remains +X forward, +Y toward the left arm, and +Z
  upward, with its origin midway between the bases. The resulting transforms
  are left `[0.0, +0.25, 0.0]` and right `[0.0, -0.25, 0.0]`, both with identity
  quaternion `[1.0, 0.0, 0.0, 0.0]`.
- All three dual profiles now mark those transforms `measured`. The MuJoCo-only
  profile remains hardware-disabled and its task-frame calibrations remain
  `simulation_only`; a measured base does not convert calibration evidence.
- The tabletop launcher's default rear inset changed from the provisional
  0.100 m to the measured 0.0508 m. Table width, depth, and thickness remain
  provisional assumptions and are labelled separately in the run snapshot.
- This removes the geometry live gate, but it does not authorize motion. The
  candidate per-hand physical calibrations still block an ordinary live dual
  launch. The official-driver read-only preflight required separate operator
  approval; its completed result is recorded below.
- The no-contact preflight passed every 501-sample simultaneous and one-arm-
  moving rest/home path. Modeled home/home clearance is 0.060 m against the
  configured 0.030 m margin. It reported both bases `measured`, left and right
  calibrations `candidate_pending_physical_validation`, and live output blocked
  only on calibration acceptance. No controller was contacted and no command
  was sent.
- Verification after the geometry update: focused dual suite `48 passed` in
  each environment; full `.venv` suite `229 passed`; full no-arm `.venv-arm18`
  suite `229 passed`; Python, JavaScript, shell syntax, and `git diff --check`
  passed.
- With explicit owner authorization, the official-driver read-only preflight
  then passed for both controllers. Both reported driver `1.8.6` and firmware
  `1.8.3`. Left `.2` reported arm joints
  `[-0.000572, -0.000572, 0.006294, 0.000191, -0.001335, -0.000954]` rad and
  gripper `-0.000022` m; right `.3` reported
  `[-0.001335, -0.004005, -0.005150, -0.001717, 0.000572, -0.000191]` rad and
  gripper `0.000022` m. Both are effectively at rest. Position mode was never
  enabled, no command was sent, and both sessions closed normally.

### 2026-08-12 — Milestone 11: isolated left-arm home cycle passed

- After explicit owner motion authorization and confirmation that the workspace
  was clear, both arms were firmly mounted, controller power cutoff was within
  reach, and both Quest grips were released, only the left arm at
  `192.168.1.2` was commissioned through rest → home → rest.
- `scripts/commission_one_arm.py` is a deliberately narrower physical launcher:
  it connects the other controller read-only, screens 501 samples of the
  selected arm's self-collision path and its cross-arm distance against the
  other arm's fresh measured pose, enables position mode only on the selected
  arm, leaves both grippers untouched, checks feedback during the existing
  vetted ramp, and returns to rest on success or fault.
- The stationary right arm's measured rest pose has joint 2 at `-0.005150 rad`,
  which the pinned model labels as a pre-existing `right_link_2` / `right_link_4`
  self-contact at path sample zero. The commissioning screen does not reinterpret
  that stationary model artifact as motion: it still applies the full right-arm
  geometry to cross-arm distance, while independently enforcing every moving-
  left-arm self-collision sample. No collision constraint on commanded geometry
  is bypassed.
- Fresh start readings matched the read-only preflight. Both outbound and return
  paths retained 0.060 m modeled cross-arm separation against the configured
  0.030 m margin. The left arm reached home with 0.009119 rad maximum joint
  error, then returned to rest with 0.008202 rad maximum error, both well below
  the unchanged 0.080 rad feedback stop. The right arm remained read-only;
  neither gripper entered position mode or received a command.
- The right arm has not yet been authorized for its own cycle. Do not infer
  right-arm motion approval or begin dual motion from this left-only result.
- Verification after adding the isolated commissioning path: focused dual
  safety suite `49 passed` in each environment; full `.venv` suite `230 passed`;
  full `.venv-arm18` suite `230 passed`; Python, JavaScript, shell syntax, and
  `git diff --check` passed.

### 2026-08-12 — Milestone 12: isolated right-arm home cycle passed

- The first attempt was interrupted before position mode or motion. Its offline
  screen identified the known near-zero right joint-2 model artifact: a
  0.359 mm `link_2` / `link_4` overlap at the measured rest pose which clears
  at the first 501-sample step (0.2%) toward home.
- The one-arm commissioning screen now permits a marginal measured start only
  up to 2 mm and only when the path never deepens the initial penetration,
  fully clears it, never re-enters contact, and retains the cross-arm margin.
  It still fails closed for any contact from a clear start, a deepening contact,
  an over-limit start, a path that never clears, or a later re-entry. Focused
  tests cover both the accepted clearing case and the never-clears rejection.
- With separate owner authorization, only the right arm at `192.168.1.3`
  entered position mode. The left arm was freshly read and remained read-only;
  neither gripper entered position mode or received a command.
- The right rest → home → rest paths retained 0.060 m modeled separation against
  the configured 0.030 m margin. Maximum home error was 0.008738 rad and final
  rest error was 0.012016 rad, both below the unchanged 0.080 rad stop limit.
  Both controllers closed normally.
- Verification after the marginal-start rule and right-arm cycle: focused dual
  safety suite `50 passed` in each environment; full `.venv` suite `231 passed`;
  full `.venv-arm18` suite `231 passed`; Python, JavaScript, shell syntax, and
  `git diff --check` passed.

### 2026-08-12 — Milestone 13: guarded Right/Behind mapping run on arm `.3`

- `right_dual_bench_axis_validation_hardware.yaml` targets physical right arm
  `.3` with the exact `right_behind_all_motions_20260723_candidate.json` used by
  the dual profile. Translation and rotation scales are both 0.20, reach limits
  are 0.035 m / 0.08 rad, the run cap is 15 s, and gripper control is disabled.
- The single-arm launcher now optionally loads the measured dual profile as a
  stationary-arm guard. It connects `.2` read-only, screens the actual startup
  and shutdown paths, refreshes the stationary state at feedback cadence, and
  checks every outgoing `.3` command against the configured 0.030 m cross-arm
  margin. The guard identity and IPs are saved in the telemetry config snapshot.
- After two stale D405 processes were found, the relay/camera stack was
  restarted. D455 and both D405s returned healthy at approximately 30 FPS, ADB
  reverse was active on ports 8443/8444, and the physical run then completed.
- Evidence is
  `runs/2026-08-12/20260812-172002_right-dual-bench-axis-validation-01`:
  1,350 rows over 14.985 s at 90.026 Hz, all Quest sequences unique and
  consecutive, 1,348 fresh rows, 1,059 active rows across two grip episodes,
  zero IK failures, and zero limiter flags.
- Position residual was 0.323 mm median / 0.909 mm p95 / 1.301 mm maximum;
  orientation residual was 0.0000025 rad median / 0.0000125 rad p95 / 0.0000968
  rad maximum. Minimum joint-limit margin was 0.423 rad. Feedback absolute
  error was 0.004163 rad p95 and 0.006429 rad maximum, far below the unchanged
  0.080 rad safety stop. PC socket-arrival-to-command was 1.918 ms median /
  2.459 ms p95. Offline reconstruction retained 0.060 m cross-arm separation.
- A post-run official-driver read-only preflight confirmed both arms at rest;
  position mode was not enabled and no command was sent during that check.
- This is a technical pass, not yet calibration acceptance. Telemetry confirms
  motion across translation/rotation components but cannot know which gesture
  the operator intended. Do not change the candidate file until the owner says
  whether right/left, up/down, forward/back, screw, nod-yes, and nod-no all felt
  correct.
- Verification before the run: focused hardware/dual suite `109 passed` in both
  environments; full `.venv` and `.venv-arm18` suites `232 passed`; Python,
  JavaScript, shell syntax, and `git diff --check` passed.

### 2026-08-12 — Milestone 14: Right/Behind accepted at 40% with full gripper

- The clean 0.20 no-gripper evidence profile remains unchanged. A separate
  `right_dual_bench_40pct_full_gripper_hardware.yaml` profile stages the owner's
  requested next run on physical right arm `.3`: 0.40 translation/rotation
  scale, 0.070 m / 0.16 rad reach, full trigger-controlled 0.000–0.040 m
  gripper travel, and a 45 s cap.
- The `.2` stationary-arm guard now screens not only the arm path but also the
  moving gripper aperture. Startup opening, every combined arm/gripper command,
  and the combined return-to-rest/gripper-close path are checked against the
  measured dual geometry and unchanged 0.030 m cross-arm margin.
- The matching MuJoCo check recorded 2,532 rows over 28.971 s, including 2,081
  active rows, two grip episodes, the complete 0–40 mm simulated gripper range,
  and zero IK failures. The viewer session ended before the configured 45 s;
  there is no evidence of an IK stop in its telemetry.
- Physical evidence is
  `runs/2026-08-12/20260812-173543_right-dual-bench-40pct-full-gripper-01`:
  4,043 rows over 44.987 s at 89.849 Hz, 3,705 active rows across three grip
  episodes, zero IK failures, no arm limiter flags, and no early safety stop.
  Two Quest sequence numbers were skipped without a reconnect or stale-stream
  event; every received sequence was unique.
- Position IK residual was 0.410 mm median / 1.367 mm p95 / 2.565 mm maximum;
  orientation residual was 0.0000043 rad median / 0.0000295 rad p95 /
  0.0002470 rad maximum. The target-to-measured-arm gap remained below both
  moving reach gates: position 10.271 mm p95 / 21.792 mm maximum against 70 mm,
  and orientation 0.03059 rad p95 / 0.07837 rad maximum against 0.16 rad.
  Motion was exercised around all three tool-rotation axes, but telemetry still
  cannot determine whether the operator intended each signed gesture.
- Joint feedback error was 0.005237 rad p95 by worst joint per sample and
  0.010144 rad maximum, well below the unchanged 0.080 rad stop. Minimum
  joint-limit margin was 0.423 rad. PC socket-arrival-to-command was 2.295 ms
  median / 3.099 ms p95.
- The trigger covered 0–1 and the physical gripper covered 0.442–40.036 mm.
  Command-to-feedback error was 0.030 mm p95 / 2.459 mm maximum, below the 3 mm
  stop. The 152 gripper-only limiter rows are expected velocity shaping, not a
  fault; the second complete close request reached 1 mm in about 0.434 s.
- Every outgoing state passed the live 30 mm stationary-arm guard. Offline
  reconstruction using the last recorded left-rest state retained 60 mm
  minimum modeled separation. The exact stationary sample is not stored per
  telemetry row, so this numeric reconstruction does not replace the live
  fail-closed checks.
- The owner then reported that everything worked as intended and in the correct
  direction. This accepts right/left, up/down, forward/back, screw, nod-yes,
  nod-no, gripper feel, and normal shutdown at the tested 40% scope.
- Acceptance is preserved in the new
  `right_behind_all_motions_20260812_accepted.json`; the 2026-07-23 candidate is
  unchanged as historical evidence. The task catalog and the dual profile's
  right side now reference the accepted file. The dual right settings were
  reduced from the untested provisional 0.45 / 0.075 m / 0.18 rad to the exact
  accepted 0.40 / 0.070 m / 0.16 rad scope. Left/Behind remains pending, so
  coordinated live output stays configuration-blocked.
- Verification after these changes: full `.venv` and `.venv-arm18` suites each
  report `234 passed`; Python compilation and `git diff --check` pass.

### 2026-08-12 — Milestone 15: direct dual-hardware launcher import fixed

- The first operator invocation of `python scripts/run_dual_hardware.py` with a
  clean `PYTHONPATH` stopped immediately with `ModuleNotFoundError: scripts`.
  It failed during Python imports, before Quest preflight, controller connection,
  position mode, or any arm command; neither arm moved.
- Cause: the launcher imported three sibling launchers through the `scripts.*`
  package path, but direct script execution places `scripts/` itself rather
  than the repository root on `sys.path`. The launcher now follows the existing
  project pattern: package-qualified imports when imported as a module and
  direct sibling imports when executed as a script.
- A regression test launches `scripts/run_dual_hardware.py --help` in a child
  process with `PYTHONPATH` removed. Direct launch was also checked manually in
  both `.venv` and `.venv-arm18`.
- Verification: focused hardware/dual suites report `117 passed` in each
  environment; full `.venv` and `.venv-arm18` suites report `235 passed`; and
  `git diff --check` passes.

### 2026-08-12 — Milestone 16: first simultaneous physical dual-arm run

- With explicit operator authorization, the left calibration override, both
  hands selected as Behind/real, and both arms at the exact 0.40 / 0.070 m /
  0.16 rad scope, both physical arms entered the coordinated launcher. Evidence
  is `runs/2026-08-12/20260812-180149_dual-40pct-first-coordinated-01`.
- The telemetry contains 4,045 rows over 44.996 s at 89.874 Hz. The owner
  clarified that they intentionally changed the live command to 45 s, so the
  observed duration is correct and there is no duration-control discrepancy.
- Left was active for 3,906 rows, right for 3,786, and both were active together
  for 3,786 rows. Both arms exercised substantial ranges on all six joints.
  There were zero IK failures, arm limiter flags, rejected proposals, fault
  rows, cross-arm collision rows, or early safety stops. Three initial held
  rows occurred before fresh bimanual input; the remaining 4,042 rows were not
  in coordinated hold. Modeled separation remained 0.060 m against 0.030 m.
- The historical joint-2 tracking failure did not recur. Left joint 2 error was
  0.002840 rad p95 / 0.007212 rad maximum; right was 0.002988 rad p95 /
  0.004884 rad maximum. Worst-joint error over either arm was 0.010517 rad,
  far below the unchanged 0.080 rad time-aligned stop.
- Dual send skew was 0.445 ms median / 0.754 ms p95 / 4.336 ms maximum, below
  the 10 ms gate. Control arrival-to-send was 5.662 ms p95 for left and
  6.144 ms p95 for right. Three isolated Quest sequence numbers were skipped;
  there was no reconnect or stale episode after initial recovery.
- The owner confirmed neither trigger was intentionally pressed. Both grippers
  therefore correctly stayed open at 0.040 m; this run does not validate
  simultaneous dual gripper motion. The owner subsequently confirmed that both
  arms worked perfectly in every direction. This accepts both Behind mappings
  at the tested 40% scope. The post-telemetry return-to-rest was not separately
  reported.

### 2026-08-12 — Milestone 17: dual Behind directions accepted; issue audit

- The accepted left calibration preserves the exact matrix used during the
  simultaneous run; no mapping, gain, reach, feedback, collision, or shutdown
  limit changed. The normal dual profile now selects accepted calibrations on
  both sides, so future Behind runs do not require
  `--accept-unvalidated-calibrations`. Explicit current authorization, the
  exact dual live token, and every independent safety gate still apply.
- GitHub issues #1–#4 all remain open upstream. Current local evidence rates
  them as follows: #1 did not reproduce in the 44.996 s run but needs a longer
  representative stress run; #2 remains unvalidated because it is specifically
  the front-facing Mirrored profile; #3 has its core three-camera operator UI
  implemented and accepted, but latency, active pose-rate impact, and a
  camera-loss hold policy remain; #4 remains because bimanual Apply input still
  silently disarms wrist calibration and no in-VR reach/clutch indicators exist.

### 2026-08-12 — Milestone 18: front-facing Mirrored bimanual accepted

- With the operator standing in front of and facing the pair, the Mirrored
  profile completed 4,039 rows over 44.990 s at 89.752 Hz. The right controller
  drove physical arm `.2` and the left controller drove `.3`, as the swapped
  front-facing assignment intended. Both arms were active together for 3,229
  rows. Evidence is
  `runs/2026-08-12/20260812-181814_dual-mirrored-45s-first-physical-01`.
- There were zero IK failures, rejected proposals, collision rows, coordinated
  holds, or fault rows. Dual send skew was 0.748 ms p95 / 5.015 ms maximum,
  below the unchanged 10 ms gate. Maximum feedback error was 0.011424 rad on
  `.2` and 0.016873 rad on `.3`, below the unchanged 0.080 rad stop. Modeled
  separation remained 0.060 m against the 0.030 m margin.
- The owner reported that Mirrored bimanual motion worked perfectly. This
  physically accepts the swapped hand-to-arm assignment and Left/Mirror at the
  tested 0.45 / 0.075 m / 0.18 rad scope. Both trigger streams remained zero,
  so the run does not accept dual gripper behavior or report shutdown quality.
- The accepted file preserves the exact candidate matrix; no mapping, gain,
  reach, feedback, collision, or shutdown limit changed. Future normal
  Mirrored runs do not require `--accept-unvalidated-calibrations`.
- GitHub issue #2's two hazardous unknowns—hand assignment and the left mirror
  transform—are resolved for this bench. Raising gains is optional because the
  operator accepted 45% performance. The position/rotation convention remains
  intentionally non-unified but is documented and now accepted on both hands.
  The formal Quest-driven Mirrored MuJoCo pass requested by the issue remains
  outstanding, and the upstream issue itself is still open.

## Next planned work

1. Preserve the accepted Right/Mirror 50% profile as the single-arm baseline.
2. Measure transport RTT/clock offset and repeated controller/encoder reversal
   events before claiming physical p95/p99 latency.
3. Preserve Left/Mirror at its accepted 45% scope and both Behind mappings at
   40%; validate each separately before any promotion to 50%.
4. The dual-arm architecture in `docs/DUAL_ARM_EXTENSION.md` is implemented:
   bimanual WebXR packet, combined MuJoCo scene, per-arm runtime, coordinated
   controller, dual telemetry, and three launchers, with offline tests. The
   measured 500 mm aligned base transforms are recorded, both individual home
   cycles passed, the two Behind mappings are physically accepted at 40%, and
   front-facing Mirrored is accepted at 45%. Dual gripper behavior and longer
   representative reliability remain pending.
5. Two-arm driver output still requires accepted per-arm calibrations,
   measured bases, all safety gates, explicit current authorization, and the
   dual token `LIVE-WIDOWXAI-DUAL-<left-ip>-<right-ip>`. The normal Behind
   Behind and Mirrored profiles now satisfy the stored calibration gate; do not
   weaken the other gates or reuse a single-arm token.
6. At 300 mm base separation the two arms collide when yawed roughly 0.25 rad
   toward each other, and the 30 mm clearance margin rejects at about
   0.20 rad. Both figures come only from synthetic 300 mm test fixtures and do
   not describe the measured 500 mm bench. Re-derive task-specific clearance
   limits at the measured separation before expecting overlapping bimanual
   tasks.
7. `scripts/run_cad_sim.py` passes 13 telemetry keys that `TELEMETRY_COLUMNS`
   never declared, so they have always been silently discarded. This is
   pre-existing and left unchanged; `tests/test_dual_arm_safety.py` pins it as
   a known exception so it cannot spread.
8. Measure camera glass-to-glass latency plus active bimanual WebXR pose-rate
   impact, and add an operator-critical camera-loss hold policy before treating
   the accepted three-view UI as ready for unattended remote teleoperation.

The main optimization objective remains: preserve the approximately 30 ms
estimated controller-capture-to-encoder response and physical smoothness while
tightening tail behavior. Do not trade away validated smoothness merely to
lower one median software number.
