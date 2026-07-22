# Ubuntu Handoff: Aurel VR Teleoperation on WidowXAI

This is the continuation guide for `ApurvK032/aurel-vr-teleop-widowxai`. It moves the tested Meta Quest 3-to-MuJoCo path from Windows to Ubuntu and then prepares the first gated run on one physical Trossen WidowXAI follower arm.

## Current state

- The Quest 3 WebXR controller stream, clutch mapping, gripper input, article-profile 200 Hz IK loop, and direct MuJoCo joint display work on Windows.
- The 30% arm-only profile completed a smooth physical WidowXAI teleoperation run on 2026-07-21 with the Quest 3, `trossen-arm==1.8.6`, firmware `1.8.3`, 100 Hz commands, and a 30 ms driver horizon.
- Native Windows physical output is intentionally disabled. Ubuntu uses Trossen's official `trossen-arm` Python driver.
- `configs/safe_demo_30pct.yaml` is the physical-arm profile. It preserves the article's mapping and decoupled IK, while reproducing the previously successful arm-side combination: Python 3.10, `trossen-arm==1.8.6`, legacy follower, 100 Hz commands, and a 30 ms command horizon. Quest translation and rotation are limited to 30% for the first demo. This profile commands J0-J5 only and leaves the physical gripper untouched.
- `configs/live_demo_50pct_full_gripper.yaml` is the next-step profile: 50% article pose gain/reach, moderately higher joint motion limits, and the full `0.040 m` to `0.000 m` follower-gripper stroke. Keep the 30% file unchanged as the rollback profile.
- On 2026-07-22, the optimized 30% physical run measured about 29 ms median hand-onset-to-encoder-onset latency, but retained a visible 30/60 Hz command beat because the approximately 90 Hz Quest stream fed an independent 120 Hz arm loop.
- `configs/smooth_30pct_full_gripper.yaml` is the current accepted 30% profile. It commands once per unique Quest frame, filters each sequence once, preserves the 15 ms horizon, uses responsive velocity/acceleration headroom with a light two-frame reversal guard, and maps the trigger across the complete 0–40 mm gripper stroke at 0.120 m/s. The responsive retune reduced limiter activation on the MuJoCo acceptance trace from 55.2% to 37.8%. The first physical attempt returned safely to rest after the old feedback check compared an encoder sample against a just-sent interpolated goal. Live feedback is now checked against a 25 ms time-aligned command history; the `0.08 rad` stall threshold and all other safety gates remain enabled.
- `runs/`, local certificates, virtual environments, and `Previous versions/` are intentionally excluded from Git.

### This Ubuntu host on 2026-07-21

- The repository and both pinned submodules are present on `main`.
- The normal Python 3.13 `.venv` contains MuJoCo 3.8.1 and is retained for simulation.
- The isolated Python 3.10 `.venv-arm18` contains the exact previously working `trossen-arm==1.8.6` package.
- The isolated offline suite, synthetic MuJoCo smoke test, and first physical teleoperation run pass.
- The USB hub, Quest connection, dedicated Ethernet route, and controller at `192.168.1.2` were visible during inspection. The wired host address was `192.168.1.3`, with a gigabit link and sub-millisecond local ping.
- Controller identification reported WidowXAI firmware `1.8.3` with no error. No arm motion was commanded and no firmware or controller setting was changed.

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
  -> decoupled 3+3 damped least-squares IK (200 Hz article simulation; 100 Hz proven arm profile)
  -> direct MuJoCo qpos display OR immediate official Trossen command
```

The position solve uses joints 0-2 and a joint-3 wrist anchor. The orientation solve uses joints 3-5. The implementation keeps the reference pose filter, damping structure, weak configured home-pose bias, reach-limit absorption, and final per-joint delta cap. There is no target cube, Ruckig, extra joint low-pass, prediction, feedforward, command queue, or simulated motor-following stage. The physical profile passes the old setup's 30 ms goal time directly to the official driver; there is no additional application-side queue.

The article's ideas should improve intuitive control and remove software queuing, but its smooth MuJoCo video does not prove physical WidowXAI smoothness. Its USB/LAN figures cover headset-to-workstation transport, not complete controller-motion-to-arm-motion latency. Use USB for commissioning; `run_xr_benchmark.py` reports transport rate/jitter only, not physical end-to-end latency.

## Recommended Ubuntu host

Use Ubuntu Desktop on x86_64. The simulation project supports Python 3.10 or newer. Physical control of this particular firmware uses the separate Python 3.10 environment created later in this guide.

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
unset PYTHONPATH  # if this terminal inherited ROS Jazzy's Python 3.12 path
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
python -m pytest -q
```

`python3 --version` must report 3.10 or newer. The pinned, verified simulation dependency is MuJoCo 3.8.1. On this host, ROS Jazzy injects Python 3.12 packages into `PYTHONPATH`; leaving that set inside another virtual environment makes pytest load incompatible ROS plugins. Use a clean non-ROS terminal or run `unset PYTHONPATH` before activating `.venv`.

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

Open four terminals in the repository and activate `.venv` in each. The reference architecture keeps the 200 Hz IK process separate from the display-rate passive viewer.

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

Terminal 3 - run the 200 Hz IK process:

```bash
source .venv/bin/activate
python scripts/run_live_sim.py
```

Terminal 4 - render its broadcast `ik_state` without throttling IK:

```bash
source .venv/bin/activate
python scripts/viewer_client.py
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

The local arm was already operated successfully with controller firmware `1.8.3`, Python `3.10`, and `trossen-arm==1.8.6`. Do not alter the controller firmware for this project. Use the isolated legacy environment so the normal MuJoCo environment remains independent.

```bash
bash scripts/setup_legacy_hardware_env.sh
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .venv-arm18/bin/python -m pytest -q
```

The launcher requires the exact proven driver `1.8.6` before it constructs a driver or contacts the arm. After connection it verifies that the controller firmware is `1.8.x`. The proven driver has no discovery API and provides only the unversioned `StandardEndEffector.wxai_v0_follower`; the config names this exact selection `legacy_1_8`.

When explicitly ready to permit a read-only connection, run the no-motion state preflight once:

```bash
env -u PYTHONPATH .venv-arm18/bin/python scripts/preflight_hardware.py \
  --config configs/safe_demo_30pct.yaml
```

It configures the version-matched driver with `clear_error=False`, reads versions, joint positions, tolerances, and limits, then cleans up. It does not enable position mode or send a position command. The controller IP comes from the safe config unless explicitly overridden.

## 5. Hardware dry run

Leave the arm disconnected or controller power off. Start the relay and Quest exactly as in the MuJoCo test, enter WebXR with left grip released, then run:

```bash
env -u PYTHONPATH .venv-arm18/bin/python scripts/run_hardware.py \
  --config configs/safe_demo_30pct.yaml \
  --duration 15
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
- the `legacy_1_8` follower profile and exact `trossen-arm==1.8.6` environment are selected;
- the no-motion state preflight passed;
- Quest tracking is fresh and left grip is released;
- MuJoCo direction, clutch, stale-stream, and gripper tests passed;
- the 15-second hardware dry run passed.

Use a 30-second first run. Replace the example IP only with the verified controller IP:

```bash
env -u PYTHONPATH .venv-arm18/bin/python scripts/run_hardware.py \
  --config configs/safe_demo_30pct.yaml \
  --live \
  --confirm-live LIVE-WIDOWXAI-192.168.1.2 \
  --duration 30
```

The explicit `--live --confirm-live LIVE-WIDOWXAI-<IP>` arguments authorize startup; there is no additional interactive `ARM READY` prompt. Before any connection the launcher rejects nonfinite or invalid timing, step-cap, feedback-threshold, and limit settings, and rejects any driver other than `1.8.6`. It then verifies fresh grip-released tracking, firmware compatibility, finite measured joints, and physical limits. Before enabling teleoperation it checks the full path against the pinned MuJoCo self-collision geometry, then performs the configured arm-only ramp to the confirmed lab home `[0, 60, 75, -60, 0, 0]` degrees. A gripper-enabled profile opens only after the arm reaches home, using the previous stack's separate blocking two-second command. Keep the left grip released through startup. After the launcher reports that home was reached, holding left grip engages relative teleoperation; releasing it holds position. This screen cannot see the table, cables, people, or other external obstacles.

Keep movements slow and small. Do not increase scaling, rate, speed limits, gripper travel, or first-run duration during commissioning.

## Safety behavior in the live launcher

- Left grip is the deadman/clutch. Released grip holds the latest safe joint command.
- Stale or reconnected tracking holds and re-anchors before motion resumes.
- Each command is checked for finite values, joint limits, reference per-tick deltas, official-model self-collision, and arm/gripper feedback error.
- Delayed mailbox samples are stale on their first control tick; reconnects require fresh samples and re-anchoring.
- The IK retains the reference `0.06 rad/tick` cap for joints 0-2 and `0.24 rad/tick` cap for joints 3-5, then the single hardware-facing limiter enforces the proven velocity and acceleration bounds at 100 Hz.
- The safe first-demo profile does not command the physical gripper; left trigger input is logged but ignored by hardware output.
- Excessive step, joint-limit approach, self-collision, feedback error, nonfinite state, or the maximum duration terminates the launcher.
- After any position command has begun, a stop collision-checks the current-to-rest path and uses the previous stack's blocking two-second all-zero arm transition; gripper-enabled profiles then close the gripper separately. If return-to-rest fails validation or feedback checks, the launcher attempts a measured-position hold and prints an emergency warning to cut controller power.
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

Arm preflight/connection fails:

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
configs/hardware_demo.yaml            article-settings comparison profile; rejected for live use
configs/safe_demo_30pct.yaml           default version-matched physical demo profile
configs/live_demo_50pct_full_gripper.yaml  next-step 50% full-gripper profile
configs/smooth_30pct_full_gripper.yaml  Quest-synchronized 30% smooth/full-gripper candidate
configs/calibrations/                 confirmed Quest/WidowXAI calibration
scripts/run_live_sim.py               Quest-to-MuJoCo acceptance run
scripts/viewer_client.py              passive broadcast-state MuJoCo viewer
scripts/preflight_hardware.py          no-motion arm/version/state check
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
5. Verify the controller's real IP and firmware without changing controller settings.
6. Install the exact proven driver environment and pass the no-motion state preflight.
7. Pass the 15-second hardware dry run.
8. Complete the mechanical and safety preflight.
9. Run one 30-second, 30%-scale 100 Hz live session with an observer.
10. Inspect the telemetry and any reported safety stop before changing a single parameter.

Change only one timing, scale, or safety parameter per experiment and preserve the corresponding telemetry directory outside Git.
