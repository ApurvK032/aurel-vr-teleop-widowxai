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
- a weak official staged/rest-pose bias;
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

Open three PowerShell terminals with the virtual environment activated.

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

Terminal 3—start the live MuJoCo consumer:

```powershell
python scripts\run_live_sim.py
```

The viewer loads Trossen's official `wxai_follower.xml`, adds only a light and floor, and displays the IK joint command directly as the article's viewer does. There is intentionally no target cube. For the first acceptance check, face the direction you want to count as robot-forward, press grip without moving, and confirm there is no jump. Move one axis slowly, then release grip and confirm the arm freezes. Re-gripping captures the current headset heading and re-anchors without a jump. Disconnecting or suspending the Quest stream freezes the command; if grip remains held, fresh tracking re-anchors at zero delta before motion resumes.

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

On a supported host, install the official driver version whose major/minor matches the controller firmware. This profile was tested against the `1.11` API shape:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e ".[dev,hardware]"
python -m pytest -q
```

The demo profile is [configs/hardware_demo.yaml](configs/hardware_demo.yaml). Its `0.75` translation and rotation gains are exactly 50% of the article defaults (`1.5 × 0.5`). It keeps the article's 200 Hz loop and `alpha=0.8` pose EMA. The existing IK per-tick caps are configured to approximately 50% of the published WidowXAI joint velocities. There is no Ruckig stage, extra low-pass, prediction, command queue, or trajectory interpolation; the official driver receives `goal_time=0.0`, nonblocking position commands.

First enter WebXR with the grip released, then exercise the complete Quest pipeline without arm I/O:

```bash
python scripts/run_hardware.py --duration 15
```

Only after the arm is firmly mounted, the workspace is empty, controller power can be cut immediately, the correct IP and firmware have been verified, and the dry run passes:

```bash
python scripts/run_hardware.py \
  --live \
  --robot-ip 192.168.1.2 \
  --confirm-live LIVE-WIDOWXAI-192.168.1.2 \
  --duration 30
```

Live startup additionally requires an exact interactive confirmation. Before position mode, the launcher requires fresh Quest tracking with grip released, uses official discovery to confirm one error-free WidowXAI at the specified IP, reads the physical joint limits and measured pose, and validates every command. It then performs the article-style five-second ramp to Trossen's official WidowXAI staged posture. Grip remains the deadman switch; stale tracking holds position and re-anchors; excessive joint steps, limit approach, tracking error, nonfinite values, or the two-minute maximum terminate the demo. Gripper travel is limited to its outer 50% for the first demo.

Trossen requires driver and controller firmware major/minor versions to match. Do not automatically clear controller errors or flash firmware for a demo; diagnose any reported error using the [official troubleshooting guide](https://docs.trossenrobotics.com/trossen_arm/main/troubleshooting.html).

## Layout

```text
configs/                         conservative baseline and confirmed calibrations
src/widowxai_quest_teleop/      model, IK, mapping, relay, safety, telemetry
src/.../web/                    Quest WebXR page
scripts/run_sim.py              synthetic MuJoCo verification
scripts/run_live_sim.py         Quest-to-MuJoCo pipeline
scripts/run_xr_benchmark.py     transport-only timing check
scripts/replay_recording.py     deterministic recorded-input replay
scripts/run_hardware.py         gated official-driver demo (supported hosts only)
scripts/setup_quest_usb.ps1     Windows ADB reverse setup
tests/                          offline acceptance tests
third_party/                    pinned official Trossen model submodules
UBUNTU_PROJECT_HANDOFF.md       Ubuntu continuation and physical-demo procedure
```
