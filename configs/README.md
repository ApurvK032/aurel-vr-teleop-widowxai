# Configuration guide

Configuration files stay at stable paths because telemetry snapshots and
rollback tags refer to them. Use the profiles below instead of choosing a file
only from its name.

## Recommended

### `quest_50pct_hardware.yaml`

Current single-arm profile:

- selects Left/Right and Behind/Mirrored from the Quest page before connecting
  to the arm;
- locks that choice for the complete run;
- runs at the Quest cadence (nominally 90 Hz);
- uses a 25 ms Trossen interpolation horizon and conservative velocity
  feedforward;
- maps controller motion at 50% of the reference article scale;
- enables the full 0–40 mm gripper stroke;
- returns the arm to all-zero rest on exit.

Right + Mirrored is operator-accepted at this scale. The other three mappings
remain candidates and must be checked in MuJoCo before physical use.

### `step3_velocity_feedforward_25ms_hardware.yaml`

Tagged left-controller rollback baseline (`milestone-3-step-3-best`):

- 30% task scale;
- full gripper;
- nominal 90 Hz Quest-synchronized commands;
- 25 ms driver horizon;
- conservative velocity feedforward;
- maximum 60-second live duration.

## Task profiles

`calibrations/task_profiles.json` maps the four page selections to fixed
calibration documents:

| Quest page | Profile | Status at 50% |
|---|---|---|
| Left + Behind/Parallel | `left_behind` | Pending |
| Left + Mirrored | `left_mirror` | Pending |
| Right + Behind/Parallel | `right_behind` | Pending |
| Right + Mirrored | `right_mirror` | Accepted |

Mirrored semantics flip left/right and forward/back translation, screw, and
nod-no. Up/down and nod-yes remain unchanged.

## Rollback ladder

These files preserve the measured tuning sequence:

1. `safe_demo_30pct.yaml`
2. `calibrated_adaptive_30pct_hardware.yaml`
3. `no_catchup_30pct_hardware.yaml`
4. `send_barrier_30pct_hardware.yaml`
5. `step2_25ms_30pct_hardware.yaml`
6. `step3_velocity_feedforward_25ms_hardware.yaml`

Matching MuJoCo/replay profiles and the article-rate experiments are retained
for controlled comparisons. Do not use an experimental profile on hardware
without first testing it in MuJoCo.

## Specialized profiles

- `six_axis_arm_diagnostic.yaml` drives a deterministic Quest-free diagnostic.
- `cad_rest_commissioning.yaml` is an independent CAD/leader-arm experiment.
- `right_mirror_axis_validation_*`, `right_real_axis_validation_hardware.yaml`,
  and `temporary_*` preserve calibration experiments and are not normal run
  profiles.

Change one control variable per experiment. Never weaken physical joint,
collision, stale-stream, feedback, driver-version, or shutdown gates merely to
make a run launch.

## Dual-arm profile

`dual_widowxai.yaml` is the bimanual profile: both Quest controllers, two arms,
one coordinated runtime. It replaces the single `quest` and `hardware.robot_ip`
blocks with per-arm `arms.left` / `arms.right` entries plus a `safety` block,
and it is validated by `config.parse_dual_arm_config` rather than by the
single-arm loader.

Its base transforms are placeholders encoding a 500 mm separation. Measure both
arm bases against one shared world frame before any physical dual-arm work,
then replace each transform and change its explicit `measurement_status` from
`placeholder` to `measured`. Live mode rejects any other status, including when
the calibration-only override is supplied.

Live two-arm output additionally requires an explicitly accepted calibration
for each arm and the dual `LIVE-WIDOWXAI-DUAL-<left-ip>-<right-ip>` token. Do
not use the single-arm profiles or token for two arms.
