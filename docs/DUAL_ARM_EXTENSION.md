# Dual-arm extension

The current project controls one WidowXAI. This document defines a safe path to
two arms without duplicating two single-arm processes and hoping they remain
synchronized.

## Target architecture

```text
Quest left + right controllers
  -> one timestamped bimanual WebXR packet
  -> latest-state relay
  -> shared bimanual safety coordinator
       -> left calibration/filter/clutch/IK -> left arm driver
       -> right calibration/filter/clutch/IK -> right arm driver
  -> dual-arm MuJoCo collision model and telemetry
```

Each arm needs independent state:

- controller hand, grip, trigger, and wrist-pivot calibration;
- Quest-to-arm-base task transform;
- clutch anchor and pose filter;
- WidowXAI model instance and IK state;
- arm IP, driver connection, limits, feedback history, and gripper state;
- rest/home poses and lifecycle completion.

The two arms share:

- one WebXR frame timestamp and reconnect generation;
- a single latest-state mailbox;
- cross-arm collision checking;
- startup/shutdown coordination;
- a fault policy that can hold or return both arms safely;
- one telemetry record per control tick.

## Required changes

### 1. Bimanual transport

Change the WebXR page from a selected-hand pose to one packet containing both
controller poses and button states. Preserve sequence numbers, capture/send
timestamps, reconnect handling, and capacity-one/latest-state behavior.

Do not merge two independently timed WebSocket streams at the driver layer.
One frame-level packet makes controller alignment and stale detection explicit.

### 2. Configuration schema

Replace single `quest` and `hardware` blocks with explicit arms:

```yaml
arms:
  left:
    controller_hand: left
    robot_ip: 192.168.1.2
    calibration: configs/calibrations/left_*.json
  right:
    controller_hand: right
    robot_ip: 192.168.1.3
    calibration: configs/calibrations/right_*.json
```

Keep global transport, cadence, telemetry, and coordinated safety policy
separate from per-arm settings. Do not copy the example IPs without checking
the actual controllers.

### 3. Dual-arm MuJoCo model

Create one scene containing both arms at their measured base transforms.
Validate:

- each arm's self-collision;
- left-arm versus right-arm collision;
- tool/gripper clearance;
- rest-to-home and home-to-rest paths for both arms;
- one-arm motion while the other arm holds.

Two independent single-arm MuJoCo instances cannot detect cross-arm collisions.

### 4. Coordinated control loop

Consume each Quest sequence once, solve both arms, collision-check the combined
state, and only then send commands. A practical first policy is:

- independent grips engage each arm;
- loss of one controller holds only that arm;
- loss of the shared WebXR/relay connection holds both;
- driver or collision failure prevents new motion on both;
- shutdown returns arms sequentially if simultaneous paths are not proven safe.

The correct send ordering and acceptable skew should be measured. Do not claim
synchronization merely because two driver calls occur in the same Python loop.

### 5. Hardware abstraction

Refactor the current launcher into:

- a reusable per-arm runtime object;
- a bimanual coordinator owning two driver backends;
- one explicit live confirmation covering both controller IPs;
- cleanup that records which arms connected, entered position mode, moved,
  held, and reached rest.

Never raw-probe either controller's TCP port. Connect each official driver only
after both Quest profiles and both complete startup paths pass offline checks.

## Validation sequence

1. Unit-test bimanual packet parsing, stale handling, and per-arm clutching.
2. Replay synthetic left-only, right-only, simultaneous, and reconnect cases.
3. Validate all twelve signed controller motions in the dual-arm MuJoCo scene.
4. Validate cross-arm collision rejection and coordinated shutdown in MuJoCo.
5. Run one physical arm through the new coordinator while the second backend is
   dry-run.
6. Swap sides and repeat.
7. Connect both arms with grips released and perform read-only state checks.
8. Run short, low-scale, one-arm-at-a-time motion.
9. Only then test slow simultaneous motion.

Record command skew, per-arm encoder response, IK failures, collision margins,
stale/reanchor events, and shutdown completion in every stage.

## Reuse versus replacement

Reusable without major redesign:

- relative pose mapping;
- per-arm filters and decoupled IK;
- task-frame calibration format;
- Trossen backend and version checks;
- telemetry primitives;
- per-arm rest/home lifecycle.

Must be redesigned:

- the selected-hand WebXR payload;
- the single-arm configuration schema;
- the single-arm launcher and safety state machine;
- MuJoCo scene/model ownership;
- collision screening and coordinated shutdown.
