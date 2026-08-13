# Project progress and measured performance

Last updated: 2026-08-12

This is the short operational ledger. `AGENTS.md` retains the complete history,
safety rules, failure analysis, and exact milestone chronology. Raw run evidence
stays under `runs/`; compact publishable metrics stay under `results/`.

## Current system state

| Area | Status | Best evidence / performance | What it does not prove |
|---|---|---|---|
| Single-arm Quest teleoperation | Accepted baseline | Right controller + Mirrored at 50% task scale; latest accepted run 4,627 rows over 51.398 s at 90.003 Hz with zero IK failures | The other three hand/mapping combinations are not accepted at this scale |
| Single-arm response | Accepted as smooth and responsive | Approximately 30 ms estimated controller-capture-to-encoder response from software/driver telemetry | Not a synchronized high-speed-camera measurement |
| Bimanual Quest transport | Physically exercised and direction-accepted | Schema-v2 packets drove both arms together for 3,786 rows at 89.874 Hz overall; the operator confirmed correct response in every direction | Transport evidence does not validate Mirrored/front-facing mode or dual grippers |
| Dual feedback telemetry | Physically exercised | Truthful per-arm feedback showed left/right joint-2 maxima of 0.007212/0.004884 rad with no fault in a 44.996 s simultaneous run | One clean run is not a long-duration reliability study |
| Dual MuJoCo tabletop | Operator-accepted | 10,211 rows over 114.878 s at 88.877 Hz; zero IK failures; 218 deliberate cross-arm clearance rejections held both arms exactly | Does not validate drivers, encoders, measurement tolerances, or physical collision distance |
| Dual-arm bench geometry | Measured | Bases exactly 500 mm apart, no X/Z offset, equal height, parallel with no yaw difference; both centers 50.8 mm (2 in) from the rear edge | Overall table width/depth/thickness and physical collision clearance remain unmeasured |
| Dual-arm controllers | Coordinated physical pass; Behind directions accepted | Both use driver 1.8.6/firmware 1.8.3; they completed a simultaneous 44.996 s run with 4.336 ms maximum send skew and correct operator-observed response in every direction | Dual grippers, post-run rest confirmation, and longer reliability remain pending |
| Left-arm home cycle | Physical pass | `.2` alone completed rest → home → rest; maximum home error 0.009119 rad, final rest error 0.008202 rad, modeled separation 60 mm against 30 mm margin | Does not validate signed teleoperation, gripper motion, or coordinated dual motion |
| Right-arm home cycle | Physical pass | `.3` alone completed rest → home → rest; 0.359 mm resting model artifact cleared by 0.2% without deepening; maximum home error 0.008738 rad, final rest error 0.012016 rad | Does not validate signed teleoperation, gripper motion, or coordinated dual motion |
| Right/Behind mapping on arm `.3` | Operator-accepted at 40% | 4,043 rows/44.987 s at 89.849 Hz; 3,705 active rows, full 0.442–40.036 mm physical gripper range, zero IK failures/arm limiter flags; max joint tracking error 0.010144 rad; operator confirmed correct directions and intended behavior | Does not accept 45%/50% gains, the left-hand mapping, or simultaneous dual motion |
| First simultaneous physical dual run | Technical and signed-direction pass | 4,045 rows/44.996 s at 89.874 Hz; both arms active together for 3,786 rows; zero IK/limiter/fault/collision-rejection rows; max tracking error 0.010517 rad; send skew 0.754 ms p95 / 4.336 ms max; operator confirmed both arms worked perfectly in every direction | Triggers were intentionally unused; shutdown was not separately confirmed |
| Front-facing Mirrored dual run | Technical and operator pass at 45% | 4,039 rows/44.990 s at 89.752 Hz; both arms active together for 3,229 rows; swapped controller assignment worked; zero IK/fault/hold/collision-rejection rows; max tracking error 0.016873 rad; operator reported perfect behavior | Triggers were intentionally unused; formal Quest-driven Mirrored MuJoCo pass and shutdown report remain pending |
| Three-camera Quest operator view | Operator-accepted | One D455 plus two D405 feeds at 640×480×30 FPS; measured rates 29.97, 30.02, and 30.00 FPS | Glass-to-glass latency and three-stream impact on active WebXR pose rate remain unmeasured |
| Camera UI | Operator-accepted | Persistent serial-based Scene/Left wrist/Right wrist assignment; adaptive 1/2/3-panel world anchor; B-button focus cycle; 380 ms transitions; common +0.18 m vertical offset | Camera loss currently warns but does not force robot hold |
| Offline regression suite | Passing | 235 tests in `.venv` and 235 in `.venv-arm18`; the clean-PYTHONPATH direct dual-launch regression and `git diff --check` pass | Offline tests cannot authorize or validate physical motion |

## Camera milestone details

The accepted workstation exposes one RealSense D455 and two D405s. All three
negotiate USB SuperSpeed (5 Gb/s). At the final acceptance snapshot:

| Role | V4L2 serial | Measured FPS | Latest JPEG size |
|---|---|---:|---:|
| Scene | `208223060917` | 29.970 | 20,147 bytes |
| Left wrist | `323743070189` | 30.019 | 28,926 bytes |
| Right wrist | `323743071937` | 30.004 | 28,253 bytes |

The instantaneous encoded payload represented by those frame sizes is roughly
2.3 MB/s total before WebSocket overhead. Earlier Quest ADB testing sustained
approximately 47 MB/s, so raw USB forwarding capacity is not the current
bottleneck. This comparison is not a video-latency measurement.

Accepted behavior:

- each logical role selects any compatible free camera from the browser;
- one physical serial cannot be assigned to multiple roles;
- assignments persist on the PC and can change without editing code;
- an unselected or unchecked role does not appear in passthrough;
- overview adapts to one, two, or three enabled panels;
- B cycles overview and each enabled focus view while grips are released;
- non-primary feeds remain visible as previews;
- panels remain fixed in the Quest `local-floor` world frame;
- every layout uses a 380 ms eased transition and a common +0.18 m height
  adjustment, accepted as no longer obstructing the direct work area.

Known camera limitations:

- RGB freshness is display-only and is not yet a physical-motion safety input;
- glass-to-glass latency has not been measured;
- active WebXR pose rate with all three streams has not been recorded in a
  controlled before/after trace;
- the shared USB-C dock has reset once, simultaneously removing Ethernet and
  the hub-connected D405s. It recovered, but dock power/cabling should be
  stabilized before physical dual-arm operation.

## Dual-arm work completed

1. Bimanual WebXR packets and the latest-state relay were corrected and tested.
2. Dual per-arm runtime, synchronized command proposal, combined collision
   rejection, and dual telemetry were implemented.
3. Feedback timestamps/reference states were made truthful so a future joint-2
   fault remains diagnosable.
4. Placeholder base geometry was made a fail-closed live-output blocker, then
   replaced with the measured 2026-08-12 bench transforms.
5. A provisional 500 mm tabletop MuJoCo scene was created with table collision.
6. A simulation-only bimanual calibration was captured for the left controller
   and explicitly derived for the right controller.
7. The operator accepted the dual MuJoCo mapping and controllability.
8. Deliberate simulated arm approaches confirmed coordinated collision hold.
9. The PC/arm address conflict was removed and both controllers are now visible
   as distinct Ethernet devices.
10. The exact 500 mm aligned base layout and 50.8 mm rear inset were recorded
    in all dual profiles; the old 100 mm simulated rear-inset assumption was
    removed while overall table dimensions remain explicitly provisional.
11. No-contact preflight against that measured geometry passed simultaneous
    rest/home, gripper endpoint, and both one-arm-moving path screens. Modeled
    home/home clearance is 60 mm against the configured 30 mm margin.
12. The separately authorized official-driver read-only preflight passed for
    both controllers. Position mode remained disabled, no command was sent,
    and both sessions closed normally.
13. With explicit motion authorization and the physical checklist confirmed,
    the left arm alone completed rest → home → rest. The right controller
    remained read-only and neither gripper was placed in position mode.
14. With separate authorization, the right arm completed the same isolated
    cycle. Its shallow near-zero model artifact was bounded to 2 mm, measured
    at 0.359 mm, proven non-deepening, and cleared by 0.2% of the outbound path.
15. The dual profile's exact Right/Behind candidate was exercised on arm `.3`
    at 0.20 translation/rotation scale with arm `.2` read-only. The run was
    technically clean; operator confirmation of the six signed directions is
    still required before changing calibration acceptance evidence.
16. The separate 0.40 translation/rotation profile passed a guarded 44.987 s
    physical run on arm `.3` with full trigger-controlled gripper motion. It had
    zero IK failures, no arm limiter flags, maximum joint tracking error
    0.010144 rad, and physical gripper travel from 0.442 to 40.036 mm. The live
    stationary-arm guard accepted every command. The operator subsequently
    accepted all directions, orientation response, gripper feel, and shutdown.
17. That verdict is preserved in a separate accepted calibration document. The
    dual profile's right side now uses the exact accepted 0.40 / 0.070 m /
    0.16 rad scope; the untested provisional 0.45 settings were not promoted.
18. The first dual-hardware command exposed a direct-script import-path bug and
    stopped before connecting to either arm. The launcher now supports the
    documented clean-`PYTHONPATH` command, with a subprocess regression test;
    no physical motion occurred during the failed invocation.
19. With an explicit left-calibration override, both arms then completed the
    first simultaneous coordinated run at 40%. Joint-2 tracking remained
    healthy on both sides, send skew stayed below its gate, and no collision or
    control fault occurred. The owner clarified the 45 s duration was
    intentional and that triggers were deliberately unused.
20. The operator subsequently confirmed that both arms worked perfectly in
    every direction. Left/Behind is therefore accepted at the exact 40% scope
    used by the run, alongside the already accepted Right/Behind mapping. The
    accepted file preserves the tested matrix without changing any limit. Dual
    grippers and Mirrored/front-facing mode remain separate validation items.
21. The front-facing Mirrored profile then completed a 44.990 s physical pass
    at 45%. The right controller correctly drove `.2`, the left controller
    correctly drove `.3`, and the operator reported perfect behavior. The run
    had zero faults, holds, IK failures, collision rows, or rejected proposals.
    Left/Mirror is now accepted at that exact scope; both triggers remained
    unused, so dual grippers remain separate.

## Remaining validation work

1. Confirm normal return-to-rest after the next run; the completed run's
   shutdown was not separately reported.
2. Exercise both triggers in a short separated-workspace run if simultaneous
   dual gripper behavior is required; this run left both grippers open.
3. The historical joint-2 fault did not recur in this 45 s run. Repeat under a
   longer representative task before calling it closed; keep the 0.08 rad stop.
4. Repeat dual motion with both grippers in separated work volumes. Do not
   deliberately collide the physical arms.
5. Add a camera-loss policy before unattended remote teleoperation. The safest
   default is to force a clutch release/hold when every operator-critical view
   is stale.
6. Measure three-camera glass-to-glass latency and Quest pose-rate impact during
   an active bimanual WebXR session.
7. Run the formal Quest-driven Mirrored MuJoCo pass retained in issue #2. This
   is documentation/validation closure; physical assignment and mapping are
   already accepted at 45%.
8. Fix the bimanual wrist-calibration arming UX and add in-VR calibration,
   clutch, and reach-limit status.

## GitHub issue audit

All four issues in `sys3-lab/vr-telop-widowxai` are still open upstream. Local
implementation and evidence are ahead of their tracker state:

- **#1 joint-2 tracking fault:** not reproduced in either 45 s dual run; the
  unchanged 0.08 rad stop remained active and the worst observed maximum was
  0.016873 rad. Treat as provisionally healthy, not closed, until a longer
  representative run passes.
- **#2 Mirrored/front-facing mode:** functionally resolved at 45%. The swapped
  hand assignment, left mapping, and performance were operator-accepted in a
  clean physical run. Only the formal Quest-driven Mirrored MuJoCo record and
  optional future gain work remain; the non-unified position/rotation signs are
  documented as an intentional operator convention.
- **#3 camera operator view:** core feature implemented and operator-accepted
  with one D455 and two D405 roles, independent camera transport, dropdowns,
  toggles, overview/focus cycling, and world-anchored transitions. Remaining
  closure work is glass-to-glass latency, active pose-rate impact, and a
  camera-loss hold policy.
- **#4 calibration/reach UX:** still outstanding. Bimanual Apply input still
  disarms calibration until the 2D Calibrate wrist button is pressed, and the
  headset does not persistently show per-hand calibration, clutch, or near-limit
  state.

## Recommended next milestone

Behind at 40% and front-facing Mirrored at 45% are technically clean and
operator-accepted. The next smallest physical test is a short dual-trigger run
in separated workspaces with explicit shutdown confirmation, followed by a
longer representative run to build evidence against recurrence of issue #1.

## Change-control state

The current work is on local branch `dual-stabilization`. Camera, bimanual
stabilization, documentation, and test changes are intentionally uncommitted
and unpushed until the owner asks for publication. No physical robot command was
sent during the camera and LAN milestones.
