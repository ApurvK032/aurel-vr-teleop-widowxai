# WidowXAI + Meta Quest 3 Teleoperation

Low-latency, clutch-relative teleoperation of a Trossen WidowXAI arm from a
Meta Quest 3 Touch Plus controller.

This project adapts Aurel Arnold's
[VR Teleoperation Stack](https://aurelarnold.xyz/blog/vr-teleoperation-stack/)
and the
[Dream Machines reference kit](https://github.com/Dream-Machines-Robotics/vr-teleop-kit)
to the WidowXAI. It adds the official robot model, Trossen driver integration,
rest/home lifecycle, task-frame calibration, physical safety checks, and
latency telemetry.

> Experimental research software: validate changes in MuJoCo first, firmly
> mount the arm, clear its workspace, and remain ready to cut controller power.

## Status

| Capability | Status |
|---|---|
| Quest 3 WebXR input over USB | Working |
| Left/right controller selection | Working |
| Right controller + Mirrored mapping | Physically accepted at 50% task scale |
| Full-stroke trigger gripper | Working |
| MuJoCo on Windows and Ubuntu | Working |
| One physical WidowXAI on Ubuntu | Working |
| Automatic rest → home → rest | Working |
| Bimanual WebXR capture, dual-arm MuJoCo, coordinated runtime | Working |
| Two-arm physical operation | Implemented, blocked pending per-arm calibration acceptance |

The current single-arm profile is
[`configs/quest_50pct_hardware.yaml`](configs/quest_50pct_hardware.yaml).
The validated left-controller Step-3 profile remains available as a rollback
baseline. See [`configs/README.md`](configs/README.md) before changing profiles.

The latest accepted run sustained 90.0 Hz for 51.4 seconds with zero IK
failures. Software timestamps and encoder-trajectory alignment estimate about
30 ms from captured controller pose to arm encoder response. This is not a
camera-verified hand-to-arm measurement; exact components and caveats are in
[`results/`](results/).

## How it works

```text
Quest controller
  → WebXR frame callback
  → WebSocket over USB ADB reverse
  → latest-state relay
  → clutch-relative calibrated mapping
  → filtered decoupled WidowXAI IK
  → safety checks
  → MuJoCo or Trossen driver
  → WidowXAI
```

Grip acts as a deadman and clutch. Pressing it anchors the current controller
and robot poses. Releasing it holds the last safe command and lets the operator
reposition their hand. Re-gripping creates a new zero-delta anchor without a
jump.

## Quick start: MuJoCo

Clone with the pinned Trossen model submodules:

```bash
git clone --recurse-submodules \
  https://github.com/sys3-lab/vr-telop-widowxai.git
cd vr-telop-widowxai

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
python -m pytest -q
```

Connect the Quest by USB, accept USB debugging, and start the local relay:

```bash
bash scripts/restart_localhost.sh
```

Open `http://localhost:8443/` in the Quest Browser. For the accepted profile,
select **Right + Mirrored**, press **Apply input**, and enter passthrough.

Run MuJoCo:

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_live_sim.py \
  --config configs/quest_50pct_hardware.yaml \
  --label quest-50pct-mujoco
```

Windows setup, Quest details, physical-arm prerequisites, calibration, and
troubleshooting are in the
**[setup and operation guide](docs/SETUP_AND_OPERATION.md)**.

## Physical arm

Physical output requires Ubuntu, CPython 3.10, the legacy Trossen 1.8.6 driver,
and wired Ethernet to the arm. Native Windows is supported only for MuJoCo.

Before enabling output:

1. verify all six translations/rotations and the gripper in MuJoCo;
2. firmly mount the arm and clear its workspace;
3. run the no-motion preflight and dry run;
4. keep grip released during the automatic rest-to-home move;
5. remain ready to cut controller power.

The full, copyable commands and lifecycle behavior are documented in
[`docs/SETUP_AND_OPERATION.md`](docs/SETUP_AND_OPERATION.md#run-one-physical-widowxai).

## Results

Large raw traces remain local under the ignored `runs/` directory. The
repository contains only compact publishable data:

- [`results/major_runs.csv`](results/major_runs.csv) — major physical runs;
- [`results/README.md`](results/README.md) — measured versus estimated latency;
- [`scripts/organize_runs.py`](scripts/organize_runs.py) — local run organizer.

## Two arms (bimanual)

Both Quest controllers are captured in one timestamped WebXR frame, each drives
its own arm through its own calibration, and both arms are solved and screened
for cross-arm collision in **one** MuJoCo scene before either command is sent.
The single-arm page, profile, and tests are unchanged and remain the rollback
path.

Select **Bimanual** on the Quest page, then run the simulation:

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_dual_sim.py \
  --config configs/dual_widowxai.yaml
```

No-motion preflight, including simultaneous and one-arm-moving path screening
in the combined scene:

```bash
env -u PYTHONPATH .venv/bin/python scripts/preflight_dual_hardware.py \
  --config configs/dual_widowxai.yaml
```

`scripts/run_dual_hardware.py` implements the full physical lifecycle
(lockstep rest→home ramp, per-arm grippers, coordinated fault hold, sequential
return to rest). **Live two-arm output is currently blocked by configuration**:
it requires an explicitly accepted calibration for each arm, and both per-hand
Behind/Parallel calibrations are still candidates. It also requires its own
`LIVE-WIDOWXAI-DUAL-<left-ip>-<right-ip>` token; the single-arm
`LIVE-WIDOWXAI-<ip>` token can never enable two arms.

Base transforms in `configs/dual_widowxai.yaml` encode a 300 mm separation and
are **placeholders**. Measure both arm bases against one shared world frame
before any physical work — every cross-arm collision result depends on them.

Architecture, staged validation, and the definition of done are in
[`docs/DUAL_ARM_EXTENSION.md`](docs/DUAL_ARM_EXTENSION.md).

## Repository map

```text
configs/                    accepted, rollback, and experimental profiles
docs/                       operating and extension guides
results/                    compact publishable milestone metrics
scripts/                    setup, simulation, calibration, and hardware tools
src/widowxai_quest_teleop/  transport, mapping, IK, safety, and telemetry
tests/                      offline unit and integration tests
third_party/                pinned official Trossen model submodules
runs/                       ignored local raw telemetry
```

`AGENTS.md` preserves the detailed experimental history, safety constraints,
known failure signatures, and recovery points for future development sessions.

## Documentation

- [Setup and operation](docs/SETUP_AND_OPERATION.md)
- [Configuration catalog](configs/README.md)
- [Latency and results](results/README.md)
- [Dual-arm extension](docs/DUAL_ARM_EXTENSION.md)
- [Third-party notices](THIRD_PARTY_NOTICES.md)

## Acknowledgements

- Aurel Arnold and
  [Dream Machines Robotics](https://github.com/Dream-Machines-Robotics/vr-teleop-kit)
- [Trossen Robotics](https://www.trossenrobotics.com/) for the WidowXAI model,
  driver, and documentation

Project-specific code is available under the [MIT License](LICENSE). Adapted
components and pinned submodules retain their original licenses; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
