# CAD Leader to WidowXAI: Rest-Only Commissioning

This path mirrors five M3T-tracked joints from the 3D-printed leader into the
WidowXAI model. It intentionally has no intermediate home pose. The six arm
joints start at all-zero rest and return to all-zero rest on every normal exit.
The physical gripper is not part of this commissioning path.

## Current safety boundary

`scripts/run_cad_sim.py` is MuJoCo-only. It rejects any configuration with
`hardware.enabled: true`, never imports `trossen_arm`, and has no `--live`
option. Do not replace that check or connect its targets directly to the arm.

The current path provides:

- exact validation of all five expected M3T joint names and finite values;
- rejection of stale or future-dated packets at receipt;
- a capacity-one/latest-state UDP receiver;
- monotonic-sequence, source-step, and 100 ms freshness checks;
- three consecutive fresh packets before recovery;
- mandatory deadman release/repress after timeout, tracker restart, jump, or
  command fault;
- relative anchoring, so engagement and recovery produce zero motion;
- wrapped angular differences across plus/minus pi;
- a 0.01 rad source deadband around every anchor;
- ten-percent leader-to-follower angular scale;
- a global two-degree command envelope around rest;
- inward-only initial travel for J1 and J2, whose official lower limits are
  exactly zero at rest;
- J5 fixed at rest because M3T currently publishes only five joints;
- 0.10 rad/s velocity and 0.50 rad/s^2 acceleration limits;
- per-command limits and official MuJoCo self-collision checks; and
- a limited return to exact rest when the process stops.

The console `e`/`r` control is only a simulation latch. It is not an acceptable
physical deadman. The first hardware version should use the already-tested
Quest left grip or a dedicated momentary switch.

## Run with the connected RealSense

Terminal 1, from the CAD project:

```bash
cd /home/apurv/Codex-Complete/322/CAD-pose_detection
./experiments/m3t_leader_arm_assembly/run_joints_stream.sh
```

Click the `rgb_cad_overlay` window and press `t` to start tracking.

Terminal 2, from this project:

```bash
cd "/home/apurv/Codex-Complete/322/VR Teleoperation Stack for Robot Manipulation-WXAI/aurel-vr-teleop-widowxai"
env -u PYTHONPATH .venv/bin/python scripts/run_cad_sim.py \
  --config configs/cad_rest_commissioning.yaml
```

In that simulation terminal:

- `e` + Enter captures the current leader and simulated robot positions and
  engages the simulation deadman;
- `r` + Enter releases and holds the latest command; and
- `q` + Enter releases, returns MuJoCo to rest, and exits.

If a fault occurs while engaged, release with `r`, wait for a fresh stable
stream, and use `e` to capture a new zero-delta anchor. Never enlarge a limit
just to make a fault disappear.

For a no-window stream check that cannot move physical hardware:

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_cad_sim.py \
  --headless --duration 15
```

## Evidence from the first live-camera checks

The connected M3T tracker produced roughly 31-37 valid packets per second with
no malformed packets. The safety behavior worked as intended:

- no tracker packets meant the command remained exactly at rest;
- tracker-start discontinuities prevented automatic engagement;
- a target below the J1/J2 lower limit caused an immediate hold;
- continued deadman pressure could not resume after a fault;
- release and repress captured a zero-delta anchor; and
- every run returned MuJoCo to exact all-zero rest.

The stationary-looking 20-second trace is not yet good enough for physical
output. Its source spans were approximately:

```text
[J0, J1, J2, J3, J4] = [0.104, 0.034, 0.013, 0.021, 0.254] rad
```

It also recorded four discontinuities. J4 drift alone was about 14.5 degrees
at the leader estimate. The two-degree follower envelope contained the result,
but an envelope is not a substitute for trustworthy tracking.

## Required acceptance before a hardware launcher is added

1. Hold the leader fixed for at least 30 seconds.
2. Receive at least 99 percent fresh samples and no gap longer than 100 ms.
3. Record no source-step, sequence-restart, mapping, limit, or collision faults.
4. Keep the total stationary span of every tracked joint below 0.02 rad.
5. Move only J0 and verify the intended sign in MuJoCo; repeat individually for
   J1 through J4. J1 and J2 must move inward from zero, never below zero.
6. Release the simulation deadman and verify exact hold.
7. Occlude the arm, recover tracking, and verify that motion remains disabled
   until release/repress and that repress produces no jump.
8. Unplug/reconnect the camera and repeat the same recovery check.
9. Repeat a complete run from rest back to rest with no safety event.

Only then add a CAD-specific physical launcher around the existing
`TrossenArmBackend`, `CommandGate`, collision checks, encoder following-error
watchdog, exact 1.8.6 driver gate, and explicit `LIVE-WIDOWXAI-<IP>` token.
Its first physical profile must retain the two-degree envelope, ten-percent
scale, J5 fixed, gripper disabled, and a real hold-to-run deadman.
