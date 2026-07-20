# Ubuntu Handoff: Aurel VR Teleoperation on WidowXAI

This is the continuation guide for `ApurvK032/aurel-vr-teleop-widowxai`. It moves the tested Meta Quest 3-to-MuJoCo path from Windows to Ubuntu and then prepares the first gated run on one physical Trossen WidowXAI follower arm.

## Current state

- The Quest 3 WebXR controller stream, clutch mapping, gripper input, 200 Hz IK loop, and direct MuJoCo joint display work on Windows.
- The physical-arm launcher is implemented but has not yet been exercised on the real arm. Treat the first Ubuntu run as commissioning, not as a normal demo.
- Native Windows physical output is intentionally disabled. Ubuntu uses Trossen's official `trossen-arm` Python driver.
- `configs/hardware_demo.yaml` applies 50% of the article's translation and rotation gains with no added command delay, interpolation queue, trajectory generator, prediction, or feedforward.
- `runs/`, local certificates, virtual environments, and `Previous versions/` are intentionally excluded from Git.

## Canonical sources

The implementation follows:

- Aurel Arnold, [VR Teleoperation Stack for Robot Manipulation](https://aurelarnold.xyz/blog/vr-teleoperation-stack/)
- [Dream-Machines-Robotics/vr-teleop-kit](https://github.com/Dream-Machines-Robotics/vr-teleop-kit) at commit `8d37f3d6bd44c8ce646f7bc110ac98e802e0ab57`
- [Trossen Arm official software setup](https://docs.trossenrobotics.com/trossen_arm/main/getting_started/software_setup.html)
- [Trossen Arm official troubleshooting](https://docs.trossenrobotics.com/trossen_arm/main/troubleshooting.html)

The official robot assets are pinned Git submodules:

- `trossen_arm_mujoco`: `77aba5d32654f17945f139e63310e7a3ac2cd4ab`
- `trossen_arm_description`: `21d8b360c211c2ad8a065d8f462cbec0207626e7`

Do not replace this path with the old XRoboToolkit/Unity snapshots under `Previous versions/`. They were examined only as historical reference and are not required by the article implementation.

## Architecture and article fidelity

```text
Quest Browser WebXR
  -> capacity-one/latest-state WebSocket relay
  -> pose EMA (alpha 0.8)
  -> grip-relative clutch mapping
  -> decoupled 3+3 damped least-squares IK at 200 Hz
  -> direct MuJoCo qpos display OR official Trossen position command
```

The position solve uses joints 0-2 and a joint-3 wrist anchor. The orientation solve uses joints 3-5. The implementation keeps the reference pose filter, damping structure, weak staged-pose bias, reach-limit absorption, and final per-joint delta cap. There is no target cube, Ruckig, joint low-pass filter, actuator trajectory, command queue, or simulated motor-following stage.

## Recommended Ubuntu host

Use Ubuntu Desktop 24.04 on x86_64 if possible. Trossen lists Ubuntu 20.04, 22.04, and 24.04 for the driver, but this project requires Python 3.11 or newer; Ubuntu 24.04 provides Python 3.12 directly.

Required equipment:

- Meta Quest 3 with Developer Mode enabled
- data-capable USB cable
- firmly mounted WidowXAI follower and controller
- dedicated Ethernet connection from the Ubuntu PC to the controller
- immediate access to controller power/E-stop behavior
- a second person as safety observer for the first physical run

## 1. Clone and install

```bash
sudo apt update
sudo apt install -y git python3 python3-venv python3-pip android-tools-adb

git clone --recurse-submodules https://github.com/ApurvK032/aurel-vr-teleop-widowxai.git
cd aurel-vr-teleop-widowxai
git submodule update --init --recursive
git submodule status

python3 --version
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
python -m pytest -q
```

`python3 --version` must report 3.11 or newer. The pinned, verified simulation dependency is MuJoCo 3.8.1.

Run the synthetic smoke test before connecting the Quest or arm:

```bash
python scripts/run_sim.py --duration 5 --realtime --viewer
```

If the desktop cannot open a MuJoCo window, first confirm that the command is running inside an Ubuntu graphical session. For a computation-only check, omit `--viewer`.

## 2. Connect the Quest over USB

Put on the Quest and accept the USB debugging prompt. Prefer **Always allow from this computer** only for this trusted Ubuntu machine.

```bash
adb kill-server
adb start-server
adb devices -l
adb reverse tcp:8443 tcp:8443
adb reverse --list
```

`adb devices -l` must show one device in the `device` state, not `unauthorized`. USB reverse forwarding must be recreated after reconnecting or rebooting the headset.

Open three terminals in the repository and activate `.venv` in each.

Terminal 1 - serve WebXR and the latest-state relay:

```bash
source .venv/bin/activate
widowxai-quest-relay
```

In the Quest Browser open:

```text
http://localhost:8443/
```

Choose **Enter Passthrough**. If requested on the first use, squeeze both grips and rotate both hands for five seconds while keeping the anatomical wrists still so the page can estimate the wrist pivot.

Terminal 2 - keep ADB forwarding available and recheck it if the cable moves:

```bash
adb devices -l
adb reverse --list
```

Terminal 3 - run the live MuJoCo consumer:

```bash
source .venv/bin/activate
python scripts/run_live_sim.py
```

Acceptance check:

1. Stand behind the intended robot-forward direction; the current confirmed calibration is `configs/calibrations/left_behind_full_pose_good_20260602.json`.
2. Face robot-forward, press the left grip without moving, and confirm there is no jump.
3. Move one Cartesian direction slowly and verify the simulated arm follows the same intended direction.
4. Release grip and verify the joints freeze immediately.
5. Reposition the hand, re-grip, and verify motion resumes with no jump.
6. Interrupt tracking or the relay and verify the command freezes, then re-anchors before resuming.
7. Verify the left trigger opens/closes the gripper.

Do not proceed to hardware if any check fails.

## 3. Configure the arm network

Keep controller power off while configuring the dedicated wired interface. Trossen's factory example uses controller `192.168.1.2/24`; verify the actual controller address rather than assuming it.

For a factory-address controller, configure the Ubuntu wired interface to an unused address on the same subnet, for example:

```text
IPv4 method: Manual
Address:     192.168.1.10
Netmask:     255.255.255.0
Gateway:     leave blank on a dedicated link
DNS:         leave blank on a dedicated link
```

Use Ubuntu Settings -> Network -> Wired -> IPv4, or apply the equivalent setting to the correct NetworkManager connection. Do not alter the Wi-Fi connection or another shared interface.

After powering the mounted controller:

```bash
ip -4 address
ip route
ping -c 4 192.168.1.2
```

The Ubuntu interface and controller must be on the same subnet. If there are multiple network interfaces, confirm traffic to the controller is routed through the dedicated Ethernet interface. Check the firewall if TCP/UDP traffic is blocked.

## 4. Install and verify the official hardware driver

The project profile is pinned to `trossen-arm==1.11.0`. Trossen requires the driver and controller firmware major/minor versions to match. Verify the controller firmware before attempting control; do not automatically flash firmware or clear errors for the demo.

```bash
source .venv/bin/activate
python -m pip install -e ".[dev,hardware]"
python -m pytest -q
```

The launcher uses official discovery and refuses a missing, mismatched, duplicated, or error-state arm. If the controller is not on firmware 1.11.x, stop and deliberately select the matching official driver/firmware combination using Trossen's documentation before continuing.

## 5. Hardware dry run

Leave the arm disconnected or controller power off. Start the relay and Quest exactly as in the MuJoCo test, enter WebXR with left grip released, then run:

```bash
source .venv/bin/activate
python scripts/run_hardware.py --duration 15
```

This exercises tracking, mapping, IK, timing, validation, and telemetry without importing the driver or opening a robot connection. Verify that it exits cleanly and writes a run under `runs/`.

## 6. First physical demo

Before enabling live output, verify every item:

- arm base and controller are firmly mounted;
- cables cannot enter the work envelope;
- the work envelope is empty;
- the operator is outside the arm's reach;
- a safety observer can cut controller power immediately;
- only one program/driver instance can own the controller;
- correct robot IP, model, firmware, and error-free state are known;
- Quest tracking is fresh and left grip is released;
- MuJoCo direction, clutch, stale-stream, and gripper tests passed;
- the 15-second hardware dry run passed.

Use a 30-second first run. Replace the example IP only with the verified controller IP:

```bash
python scripts/run_hardware.py \
  --live \
  --robot-ip 192.168.1.2 \
  --confirm-live LIVE-WIDOWXAI-192.168.1.2 \
  --duration 30
```

The launcher asks for an additional exact interactive confirmation. It then verifies fresh grip-released tracking, official discovery, model/error state, measured joints, and physical limits. Before enabling teleoperation it performs a five-second ramp to Trossen's staged WidowXAI posture.

Keep movements slow and small. Do not increase the configured 50% motion scale or the first-run duration during commissioning.

## Safety behavior in the live launcher

- Left grip is the deadman/clutch. Released grip holds the latest safe joint command.
- Stale or reconnected tracking holds and re-anchors before motion resumes.
- Each command is checked for finite values, joint limits, per-tick deltas, and feedback error.
- The configured hardware joint deltas are approximately 50% of WidowXAI's published velocity limits at 200 Hz.
- Gripper travel is limited to its outer 50% for the first demo.
- Excessive step, joint-limit approach, feedback error, nonfinite state, or the maximum duration terminates the launcher.
- No error is automatically cleared, and no firmware is automatically modified.

## Troubleshooting

`ModuleNotFoundError: mujoco`

```bash
source .venv/bin/activate
python -m pip install -e ".[dev]"
python -c "import mujoco; print(mujoco.__version__)"
```

Quest `localhost` does not load:

```bash
adb devices -l
adb reverse tcp:8443 tcp:8443
adb reverse --list
```

Also verify that the relay is still running and the Quest shows `device`, not `unauthorized`.

Arm discovery/connection fails:

- verify the controller IP and the Ubuntu wired-interface subnet;
- inspect `ip route` when Wi-Fi and Ethernet are both active;
- confirm no other driver owns the controller;
- allow required TCP and UDP traffic through the host firewall;
- power-cycle only when the official troubleshooting procedure calls for it.

Driver/firmware mismatch:

- stop before motion;
- record both versions;
- install the matching official driver or follow Trossen's documented firmware procedure;
- never flash firmware immediately before a demo without a configuration backup and recovery plan.

## Important files

```text
README.md                              project overview and Windows workflow
configs/baseline.yaml                 article-faithful simulation profile
configs/hardware_demo.yaml            gated 50%-scale hardware profile
configs/calibrations/                 confirmed Quest/WidowXAI calibration
scripts/run_live_sim.py               Quest-to-MuJoCo acceptance run
scripts/run_hardware.py               dry/live official-driver launcher
src/widowxai_quest_teleop/relay.py    WebXR server and latest-state relay
src/widowxai_quest_teleop/clutch.py   clutch-relative pose mapping
src/widowxai_quest_teleop/decoupled_ik.py  article-style 3+3 IK
src/widowxai_quest_teleop/hardware.py official-driver adapter and live gates
tests/                                offline safety and control tests
third_party/                          pinned official Trossen submodules
```

## Ubuntu continuation checklist

1. Clone recursively and record submodule revisions.
2. Run the full tests and synthetic MuJoCo smoke test.
3. Re-establish Quest ADB reverse forwarding.
4. Pass the complete Quest-to-MuJoCo acceptance check.
5. Verify the controller's real IP and firmware.
6. Install the firmware-matched official driver.
7. Pass the 15-second hardware dry run.
8. Complete the mechanical and safety preflight.
9. Run one 30-second, 50%-scale live session with an observer.
10. Inspect the telemetry and any reported safety stop before changing a single parameter.

Change only one timing, scale, or safety parameter per experiment and preserve the corresponding telemetry directory outside Git.
