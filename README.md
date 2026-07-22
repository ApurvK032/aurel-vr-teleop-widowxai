# WidowXAI + Meta Quest 3 Teleoperation

Native-Windows, simulation-first teleoperation for one Trossen WidowXAI follower arm. A WebXR page runs in the Quest Browser and streams the left controller through a capacity-one/latest-state WebSocket relay. The Windows process applies the article's filtered clutch mapping and decoupled 3+3 inverse kinematics, then mirrors the commanded joints directly in MuJoCo.

Native-Windows physical output remains deliberately disabled. The connected Quest can drive MuJoCo here; a separate fail-closed demo launcher can use Trossen's official driver only on its supported Ubuntu/macOS hosts and only after explicit live gates.

For the Ubuntu transfer, Quest setup, network setup, and first physical-arm demo procedure, use [UBUNTU_PROJECT_HANDOFF.md](UBUNTU_PROJECT_HANDOFF.md).

## What was ported

The design follows Aurel Arnold's [VR Teleoperation Stack for Robot Manipulation](https://aurelarnold.xyz/blog/vr-teleoperation-stack/) and its Apache-2.0 [reference kit](https://github.com/Dream-Machines-Robotics/vr-teleop-kit), audited at commit [`8d37f3d`](https://github.com/Dream-Machines-Robotics/vr-teleop-kit/commit/8d37f3d6bd44c8ce646f7bc110ac98e802e0ab57). The control path is the reference path, with its DK1 geometry replaced by WidowXAI geometry:

- clutch-relative controller motion;
- a joint-3 wrist anchor controlled only by joints 0-2;
- orientation controlled only by joints 3-5;
- separate manipulability-adaptive DLS damping for the arm and wrist;
- a weak configured home-pose bias;
- absorbing position and orientation reach limits;
- the reference pose EMA (`alpha=0.8`), 200 Hz IK loop, and one final per-joint delta cap;
- a capacity-one pose path with sequence and capture/send timestamps;
- stale/reconnect zero-delta re-anchoring and independent joint safety limits;
- direct `qpos` MuJoCo display, matching the reference kit's `viewer_client.py`.

There is no target cube, Ruckig stage, low-pass joint filter, actuator trajectory, prediction, or feedforward in this path. The only project-specific adapters are the confirmed Quest-to-WidowXAI calibration, the official WidowXAI model and joint limits, and the Windows USB/WebXR transport. MuJoCo is therefore an article-faithful kinematic viewer here, not a simulation of motor tracking dynamics.

The official model sources are Git submodules pinned to the audited revisions:

- `TrossenRobotics/trossen_arm_mujoco` at `77aba5d32654f17945f139e63310e7a3ac2cd4ab`
- `TrossenRobotics/trossen_arm_description` at `21d8b360c211c2ad8a065d8f462cbec0207626e7`

## Windows setup

From PowerShell in this directory:

```powershell
git submodule update --init --recursive
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
python -m pytest -q
python scripts\run_sim.py --duration 5 --realtime --viewer
```

The verified environment uses Python 3.12 and MuJoCo 3.8.1. The headless test suite validates the official limits and lit target-free viewer, direct joint display, FK Jacobians, wrist-anchor invariance, reachable/unreachable IK behavior, calibration conventions, pose filtering, latest-state buffering, stale/reconnect behavior, and per-tick joint limits.

## Connect the Quest over USB

The Quest must have Developer Mode enabled and must authorize this Windows PC for USB debugging. Use a data-capable USB cable, put on the headset, and accept **Always allow from this computer** when prompted.

Open four PowerShell terminals with the virtual environment activated. As in the reference repository, the 200 Hz IK loop and display-rate MuJoCo viewer are separate processes.

Terminal 1—serve the WebXR page and relay:

```powershell
widowxai-quest-relay
```

Terminal 2—verify ADB and install USB reverse forwarding:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\setup_quest_usb.ps1
```

In the Quest Browser, open:

```text
http://localhost:8443/
```

Select **Enter Passthrough** when available; the page falls back to a dark-blue VR tracking space if the browser does not expose WebXR passthrough. On first use, squeeze both grip buttons and rotate both hands for five seconds while keeping the anatomical wrists still. The browser estimates and stores the left controller's wrist-pivot offset. During normal use, left grip is the clutch and left trigger controls the simulated gripper.

On Ubuntu, start or restart the relay with `./scripts/restart_localhost.sh`. In
addition to restoring USB reverse forwarding, it forces Horizon OS's virtual
proximity state to mounted and verifies that the override was accepted. After
you enter passthrough once, the headset can remain on the table for repeated
MuJoCo or arm runs. The override is cleared by a headset reboot, so run the
script again after every reboot. Valid controller tracking is still required;
the teleoperation freshness watchdog and left-grip deadman are not bypassed.

Terminal 3—start the 200 Hz IK process, which publishes `ik_state` through the relay:

```powershell
python scripts\run_live_sim.py
```

Terminal 4—start the passive viewer:

```powershell
python scripts\viewer_client.py
```

The viewer loads Trossen's official `wxai_follower.xml`, adds only a light and floor, and displays the newest broadcast IK joint command directly. It cannot throttle the 200 Hz IK process. There is intentionally no target cube. For the first acceptance check, face the direction you want to count as robot-forward, press grip without moving, and confirm there is no jump. Move one axis slowly, then release grip and confirm the arm freezes. Re-gripping captures the current headset heading and re-anchors without a jump. Disconnecting or suspending the Quest stream freezes the command; if grip remains held, fresh tracking re-anchors at zero delta before motion resumes.

For a transport-only measurement with no simulation or robot output:

```powershell
python scripts\run_xr_benchmark.py --duration 15
```

If `adb devices -l` is empty, Windows does not currently see the Quest as an ADB device. Check Developer Mode, the in-headset USB prompt, the cable, and the USB port. `adb reverse` must be rerun after reconnecting the cable.

## Control and safety behavior

- Pressing grip captures the current controller, simulated tool, and wrist-anchor poses.
- The headset yaw is captured on each grip edge using the article's `R_engage = R_calib R_y(-yaw)` mapping, so operator-forward remains robot-forward while normal head motion during the grab has no effect.
- Releasing grip holds the last joint command exactly and permits hand repositioning.
- A stale stream or reconnect stops IK; fresh tracking re-anchors before resuming, preventing catch-up motion.
- Position and rotation overshoot are absorbed at reach boundaries, so reversal responds immediately.
- Each 200 Hz solve performs one position step, one wrist-orientation step, one joint-limit clamp, and the reference caps of `0.06 rad/tick` for joints 0-2 and `0.24 rad/tick` for joints 3-5.
- The deadline scheduler resets after an overrun instead of emitting catch-up command bursts.
- Prediction, feedforward, actuator simulation, and physical-arm output are off.
- Every simulation run writes a configuration snapshot, machine-readable CSV telemetry, and summary under `runs/`.

The conservative baseline is in `configs/baseline.yaml`. Do not change several timing or safety variables in the same experiment.

## Physical WidowXAI demo

The physical demo is prepared but **cannot run on native Windows**. Trossen's [official supported-platform table](https://docs.trossenrobotics.com/trossen_arm/main/getting_started/software_setup.html) lists Ubuntu 20.04/22.04/24.04 and macOS 14/15; it does not provide a Windows driver. `scripts/run_hardware.py --live` therefore exits before driver import or network access on this PC. The project does not reimplement or bypass Trossen's arm protocol.

This specific arm was already proven with controller firmware `1.8.3`, Python `3.10`, `trossen-arm==1.8.6`, and `StandardEndEffector.wxai_v0_follower`. Keep the normal `.venv` for MuJoCo and create a separate version-matched hardware environment:

```bash
bash scripts/setup_legacy_hardware_env.sh
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .venv-arm18/bin/python -m pytest -q
```

The known-good physical profile is [configs/safe_demo_30pct.yaml](configs/safe_demo_30pct.yaml). It keeps the article mapping and decoupled IK, but uses the arm-side settings proven on this robot: the legacy follower profile, a `100 Hz` command stream, a `30 ms` interpolation horizon, and conservative per-joint velocity/acceleration limits. The confirmed lab home is `[0, 60, 75, -60, 0, 0]` degrees (`[0, pi/3, 5pi/12, -pi/3, 0, 0]` radians); it is the MuJoCo start, IK rest bias, and feedback-supervised physical startup target. Controller-to-robot travel is 30%. This rollback profile commands arm joints J0-J5 only and leaves the physical gripper untouched.

After the 30% profile completed a smooth physical-arm run, [configs/live_demo_50pct_full_gripper.yaml](configs/live_demo_50pct_full_gripper.yaml) was added as the next measured step. It keeps the proven `100 Hz`/`30 ms` timing, increases pose gain and reach from 30% to 50% of the article values, moderately increases joint velocity/acceleration, and maps the left trigger over the full follower-gripper range from `0.040 m` open to `0.000 m` closed. Test this profile in MuJoCo before selecting it for hardware:

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_live_sim.py \
  --config configs/live_demo_50pct_full_gripper.yaml
```

For an article-style command path at 30% controller scale, use [configs/article_30pct_full_gripper.yaml](configs/article_30pct_full_gripper.yaml). It removes the project-specific joint and gripper velocity/acceleration shapers and runs the reference 200 Hz loop. Trossen requires at least 300 Hz when `goal_time=0`, and the complete Python path cannot guarantee that rate, so the WidowXAI adapter uses a minimal 10 ms (two-tick) nonblocking driver horizon instead of the previous 30 ms horizon. It retains the article's pose EMA, absorbing reach limits, per-tick IK caps, clutch/re-anchor behavior, and the project's hard joint, collision, feedback, and stale-stream rejection checks. The full physical gripper range is unscaled.

The raw article profile exposed high-frequency wrist reversals on the physical WidowXAI, with measured command acceleration peaks above `200 rad/s^2`. [configs/optimized_30pct_25ms.yaml](configs/optimized_30pct_25ms.yaml) is the low-latency physical candidate: 30% mapping, 120 Hz control, a 15 ms driver horizon, generous velocity ceilings, and lighter acceleration spike suppression (`15 rad/s^2` arm, `30 rad/s^2` wrist). The Quest sends each pose directly from the WebXR frame callback, and the PC sends the command immediately after IK before pacing the next control tick; this removes the two independent scheduler phase waits previously measured near 8 ms each. It retains full gripper travel and the proven rest/home lifecycle; its feedback-supervised startup and blocking shutdown are both configured for two-second moves. The profile name records the controller-motion-to-arm-motion onset target; it is not a guarantee of measured end-to-end latency.

[configs/smooth_30pct_full_gripper.yaml](configs/smooth_30pct_full_gripper.yaml) addresses the remaining minute pulses without restoring those scheduler waits. It consumes every unique approximately 90 Hz Quest frame exactly once, applies the pose EMA once per unique sequence, retains the 15 ms driver horizon, and uses responsive velocity/acceleration headroom with a light two-frame reversal guard. The trigger maps linearly over the complete `0.040 m` open to `0.000 m` closed gripper stroke at the existing `0.120 m/s` command rate; the arm remains at 30% scale. Replay of the latest physical input trace preserved the large reduction in 28–32 Hz acceleration concentration, produced no predicted self-collisions, and kept every command below the IK per-command caps. Test it in MuJoCo before hardware:

Physical encoder feedback is compared with a 25 ms delayed, interpolated command reference. This accounts for the controller's 15 ms linear goal horizon plus feedback transport/sampling lag without weakening the existing `0.08 rad` stall threshold.

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_live_sim.py \
  --config configs/smooth_30pct_full_gripper.yaml \
  --label smooth-30pct-mujoco
```

First enter WebXR with the grip released, then exercise the complete Quest pipeline without arm I/O:

```bash
env -u PYTHONPATH .venv-arm18/bin/python scripts/run_hardware.py \
  --config configs/safe_demo_30pct.yaml --duration 15
```

Only after the arm is firmly mounted, the workspace is empty, controller power can be cut immediately, the correct IP and firmware have been verified, and the dry run passes:

```bash
env -u PYTHONPATH .venv-arm18/bin/python scripts/preflight_hardware.py \
  --config configs/safe_demo_30pct.yaml

env -u PYTHONPATH .venv-arm18/bin/python scripts/run_hardware.py \
  --config configs/safe_demo_30pct.yaml \
  --live \
  --confirm-live LIVE-WIDOWXAI-192.168.1.2 \
  --duration 30
```

Driver 1.8.6 predates the discovery API, so its preflight opens the version-matched driver with `clear_error=False`, reads firmware, joint state, limits, and tolerances, then cleans up. It does not enable position mode or send a position command. The current Python 3.13 environment contains driver 1.11.0, but the launcher rejects it before constructing a driver or contacting the arm.

The explicit `--live --confirm-live LIVE-WIDOWXAI-<IP>` command-line gate authorizes startup; there is no second interactive `ARM READY` prompt. Before motion, the launcher rejects malformed safety settings, requires fresh Quest tracking with grip released, enforces the exact `1.8.6` driver and firmware `1.8.x`, validates finite feedback and physical limits, and screens the complete startup path with the pinned MuJoCo model. The arm reaches home with the gripper untouched, then a gripper-enabled profile reproduces the previous XRoboToolkit strategy with one separate blocking two-second gripper-open command. Keep the left grip released during startup; after home is reached, holding left grip engages relative teleoperation and releasing it holds the latest safe command. Stale tracking holds and re-anchors from measured feedback. On any exit after motion begins, the launcher collision-checks the path and uses a blocking two-second arm move to the all-zero rest pose; a gripper-enabled profile then closes the gripper separately. If that transition cannot be validated or completed, it falls back to a measured-position hold and prints an emergency warning.

### Quest-free six-axis arm diagnostic

Use the deterministic diagnostic to separate Quest/teleoperation command jitter from the arm/driver path. It precomputes representative `40 mm` translations and `10 degree` rotations to both sides of home for left/right, up/down, forward/backward, screw, nod-yes, and nod-no. Each one-second leg uses a minimum-snap joint trajectory with a maximum `90 Hz` command cadence and the previously smooth `30 ms` legacy-driver horizon. Timing is paced from the last successful send, so a late driver or feedback call may create one longer interval but can never emit queued catch-up commands. The complete plan is rejected before diagnostic motion if IK does not converge, MuJoCo predicts a collision, or any configured step, velocity, acceleration, jerk, or joint-margin limit is exceeded. It leaves the physical gripper untouched, moves rest-to-home on startup, and returns home-to-rest on every normal or safety-stop exit. Partial command, driver-call, and feedback timing is retained even if a run aborts.

Validate the plan without physical output:

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_six_axis_diagnostic.py
```

After clearing the workspace and preparing to cut controller power, run the physical sequence:

```bash
env -u PYTHONPATH .venv-arm18/bin/python scripts/run_six_axis_diagnostic.py \
  --live \
  --confirm-live LIVE-WIDOWXAI-192.168.1.2
```

No Quest, WebXR page, relay, or localhost server is required. A smooth run points toward the live Quest/filter/online-IK command stream as the vibration source. Vibration in this test narrows the source to the deterministic command/driver/arm side, but by itself does not prove a mechanical fault. Each run records command timing and sampled tracking error under `runs/`.

Trossen requires driver and controller firmware major/minor versions to match. Do not automatically clear controller errors or flash firmware for a demo; diagnose any reported error using the [official troubleshooting guide](https://docs.trossenrobotics.com/trossen_arm/main/troubleshooting.html).

## Layout

```text
configs/                         conservative baseline and confirmed calibrations
src/widowxai_quest_teleop/      model, IK, mapping, relay, safety, telemetry
src/.../web/                    Quest WebXR page
scripts/run_sim.py              synthetic MuJoCo verification
scripts/run_live_sim.py         Quest-to-MuJoCo pipeline
scripts/viewer_client.py        passive viewer for broadcast IK state
scripts/run_xr_benchmark.py     transport-only timing check
scripts/replay_recording.py     deterministic recorded-input replay
scripts/preflight_hardware.py   no-motion version/state check
scripts/run_hardware.py         gated official-driver demo (supported hosts only)
scripts/run_six_axis_diagnostic.py  Quest-free six-axis arm/driver check
scripts/setup_quest_usb.ps1     Windows ADB reverse setup
tests/                          offline acceptance tests
third_party/                    pinned official Trossen model submodules
UBUNTU_PROJECT_HANDOFF.md       Ubuntu continuation and physical-demo procedure
```
