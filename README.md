# WidowXAI + Meta Quest 3 Teleoperation

Native-Windows, simulation-first teleoperation for one Trossen WidowXAI follower arm. A WebXR page runs in the Quest Browser and streams the left controller through a capacity-one/latest-state WebSocket relay. The Windows Python process applies the confirmed calibration, clutch semantics, decoupled 3+3 inverse kinematics, command shaping, safety checks, and MuJoCo simulation.

Physical-arm output is deliberately disabled. The connected Quest can drive MuJoCo once Windows ADB recognizes it; no script in the current baseline can command the real arm.

## What was ported

The design follows Aurel Arnold's [VR Teleoperation Stack for Robot Manipulation](https://aurelarnold.xyz/blog/vr-teleoperation-stack/) and its Apache-2.0 [reference kit](https://github.com/Dream-Machines-Robotics/vr-teleop-kit), while replacing the DK1-specific model and IK with WidowXAI geometry:

- clutch-relative controller motion;
- a joint-3 wrist anchor controlled only by joints 0-2;
- orientation controlled only by joints 3-5;
- separate manipulability-adaptive DLS damping for the arm and wrist;
- a weak official staged/rest-pose bias;
- absorbing position and orientation reach limits;
- controller-to-anatomical-wrist pivot calibration;
- a capacity-one pose path with sequence and capture/send timestamps;
- stale/reconnect re-anchoring and independent joint safety limits.

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

The verified environment uses Python 3.12 and MuJoCo 3.8.1. The headless test suite validates the official limits, FK Jacobians, wrist-anchor invariance, reachable/unreachable IK behavior, calibration conventions, wrist-pivot estimation, latest-state buffering, stale detection, and command limits.

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

Select **Enter VR**. On first use, squeeze both grip buttons and rotate both hands for five seconds while keeping the anatomical wrists still. The browser estimates and stores the left controller's wrist-pivot offset. During normal use, left grip is the clutch and left trigger controls the simulated gripper.

Terminal 3—start the live MuJoCo consumer:

```powershell
python scripts\run_live_sim.py
```

For a transport-only measurement with no simulation or robot output:

```powershell
python scripts\run_xr_benchmark.py --duration 15
```

If `adb devices -l` is empty, Windows does not currently see the Quest as an ADB device. Check Developer Mode, the in-headset USB prompt, the cable, and the USB port. `adb reverse` must be rerun after reconnecting the cable.

## Control and safety behavior

- Pressing grip captures the current controller, simulated tool, and wrist-anchor poses.
- Releasing grip holds the last safe command and permits hand repositioning.
- A stale stream or reconnect stops target advancement and requires grip release followed by a new press.
- Position and rotation overshoot are absorbed at reach boundaries, so reversal responds immediately.
- IK output always passes through joint-limit, low-pass, velocity, acceleration, jerk, and per-cycle step limits.
- Prediction, raw Placo velocity/acceleration feedforward, and physical-arm output are off.
- Every simulation run writes a configuration snapshot, machine-readable CSV telemetry, and summary under `runs/`.

The conservative baseline is in `configs/baseline.yaml`. Do not change several timing or safety variables in the same experiment.

## Physical WidowXAI status

The official Trossen driver repository currently distributes prebuilt libraries for Linux and macOS, but not Windows. Consequently, `scripts/run_hardware.py` exits without opening a robot connection. The [official WidowXAI specifications](https://docs.trossenrobotics.com/trossen_arm/v1.8/specifications/wxai.html) and [MuJoCo documentation](https://docs.trossenrobotics.com/trossen_arm/main/tutorials/trossen_arm_mujoco.html) remain the model authority.

Before a real-arm run, a reviewed Windows-compatible driver or isolated driver bridge must implement the `HardwareBackend` interface. It must then pass network/feedback, emergency-stop, home, timeout, tiny-motion, and axis-by-axis gates. Merely passing simulation tests never enables the arm.

## Layout

```text
configs/                         conservative baseline and confirmed calibrations
src/widowxai_quest_teleop/      model, IK, mapping, relay, safety, shaping, telemetry
src/.../web/                    Quest WebXR page
scripts/run_sim.py              synthetic MuJoCo verification
scripts/run_live_sim.py         Quest-to-MuJoCo pipeline
scripts/run_xr_benchmark.py     transport-only timing check
scripts/replay_recording.py     deterministic recorded-input replay
scripts/setup_quest_usb.ps1     Windows ADB reverse setup
tests/                          offline acceptance tests
third_party/                    pinned official Trossen model submodules
```
