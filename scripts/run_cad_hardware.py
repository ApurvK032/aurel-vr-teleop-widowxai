from __future__ import annotations

import argparse
import platform
import select
import sys
import time
from dataclasses import replace

import numpy as np

from widowxai_quest_teleop.cad_input import (
    CadDeadmanController,
    CadFreshnessWatchdog,
    CadJointFilter,
    CadRestMapper,
    CadSafetyError,
    CadUdpReceiver,
    validate_cad_commissioning_selection,
    cad_live_confirmation_token,
    require_cad_joint5_locked,
    rest_command_limits,
    validate_cad_hardware_config,
)
from widowxai_quest_teleop.config import load_config
from widowxai_quest_teleop.hardware import (
    CommandGate,
    DryRunBackend,
    END_EFFECTOR_PROFILE_TO_VARIANT,
    HardwareSafetyError,
    HardwareUnavailableError,
    TrossenArmBackend,
)
from widowxai_quest_teleop.hold_to_run import (
    HoldToRunError,
    HoldToRunSample,
    LinuxEvdevHoldToRun,
    validate_evdev_key_code,
)
from widowxai_quest_teleop.model import WidowXAIModel
from widowxai_quest_teleop.one_arm_safety import screen_self_collision_path
from widowxai_quest_teleop.motion_limiter import (
    bounded_command_period,
    limiter_from_config,
)
from widowxai_quest_teleop.safety import TimeAlignedCommandHistory
from widowxai_quest_teleop.telemetry import CAD_TELEMETRY_COLUMNS, TelemetryLogger
from widowxai_quest_teleop.workstation import (
    WorkstationSafetyError,
    run_workstation_preflight,
)

try:  # Direct script execution puts scripts/, not the repository root, on sys.path.
    from scripts.run_hardware import (
        apply_locked_arm_joints,
        make_startup_command_gate,
        ramp_to_home,
        return_to_rest,
        validate_time_aligned_feedback,
    )
except ModuleNotFoundError:
    from run_hardware import (  # type: ignore[no-redef]
        apply_locked_arm_joints,
        make_startup_command_gate,
        ramp_to_home,
        return_to_rest,
        validate_time_aligned_feedback,
    )


class CadDeadmanConsole:
    """Dry-run latch and live return-to-rest terminal.

    Live activation is selected by the validated hardware profile. The console
    always retains S/Q return-to-rest handling.
    """

    def __init__(self, *, latch_enabled: bool) -> None:
        self.latch_enabled = bool(latch_enabled)
        self.pressed = False
        self.quit_requested = False
        self.interactive = False
        self.press_generation = 0
        self.release_generation = 0

    def start(self) -> None:
        self.interactive = sys.stdin.isatty()
        if not self.interactive:
            print("stdin is not interactive; CAD operator input remains released")
        elif not self.latch_enabled:
            print(
                "live terminal accepts only S/Q return-and-exit commands; "
                "motion activation is controlled by the validated profile"
            )

    def set_pressed(self, pressed: bool) -> None:
        value = bool(pressed)
        if value != self.pressed:
            if value:
                self.press_generation += 1
            else:
                self.release_generation += 1
        self.pressed = value

    def poll(self) -> None:
        if not self.interactive:
            return
        readable, _writable, _exceptional = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return
        self.handle_command(sys.stdin.readline().strip().lower())

    def handle_command(self, command: str) -> None:
        command = str(command).strip().lower()
        if not self.latch_enabled:
            if command in ("s", "stop", "q", "quit", ""):
                self.set_pressed(False)
                self.quit_requested = True
                print("CAD stop requested; returning the physical arm to rest")
            else:
                print(
                    "live CAD commands: s/q=return to rest and quit"
                )
            return
        if command in ("e", "engage"):
            self.set_pressed(True)
            print("CAD deadman ENGAGED; r + Enter releases")
        elif command in ("r", "release"):
            self.set_pressed(False)
            print("CAD deadman released; follower holds its last safe command")
        elif command in ("s", "stop", "q", "quit", ""):
            self.set_pressed(False)
            self.quit_requested = True
            print("CAD stop requested; returning to rest")
        else:
            print(
                "CAD commands: e=engage, r=release/hold, "
                "s/q=return to rest and quit"
            )


def resolve_cad_duration(configured_s: float, requested_s: float) -> float:
    configured = float(configured_s)
    requested = float(requested_s)
    if not np.isfinite(configured) or configured <= 0.0:
        raise CadSafetyError("configured CAD duration must be finite and positive")
    if not np.isfinite(requested) or requested < 0.0:
        raise CadSafetyError("requested CAD duration must be finite and nonnegative")
    return configured if requested == 0.0 else min(configured, requested)


def require_initial_rest(state, rest_q: np.ndarray, tolerance_rad: float) -> None:
    error = np.abs(np.asarray(state.q_arm, dtype=float).reshape(6) - rest_q)
    joint = int(np.argmax(error))
    if error[joint] > float(tolerance_rad):
        raise HardwareSafetyError(
            f"CAD startup requires all-zero rest; joint {joint} is "
            f"{error[joint]:.6f} rad from rest (limit {float(tolerance_rad):.6f})"
        )


def require_released_hold_to_run(
    control: LinuxEvdevHoldToRun,
    phase: str,
) -> HoldToRunSample:
    try:
        sample = control.sample()
    except HoldToRunError as exc:
        raise HardwareSafetyError(str(exc)) from None
    if sample.pressed:
        raise HardwareSafetyError(
            f"hold-to-run control must be released {phase}; release it and restart"
        )
    return sample


def wait_for_locked_cad(
    receiver: CadUdpReceiver,
    watchdog: CadFreshnessWatchdog,
    timeout_s: float,
):
    """Require a continuous locked-root source before any arm connection."""

    deadline = time.perf_counter() + float(timeout_s)
    last_sample = None
    while time.perf_counter() < deadline:
        sample, _generation = receiver.mailbox.take_latest()
        now_ns = time.perf_counter_ns()
        if sample is not None:
            last_sample = sample
            freshness = watchdog.observe(sample, now_ns)
            if freshness.fresh:
                return last_sample, freshness
        else:
            watchdog.poll(now_ns)
        time.sleep(0.005)
    detail = receiver.last_error or "no valid locked-root M3T packets arrived"
    raise CadSafetyError(f"CAD source preflight failed: {detail}")


def make_cad_command_gate(
    state,
    initial_q: np.ndarray,
    command_limits: np.ndarray,
    max_step: np.ndarray,
) -> CommandGate:
    """Intersect model commissioning bounds with controller-reported bounds."""

    arm_limits = np.asarray(command_limits, dtype=float).reshape(6, 2).copy()
    arm_limits[:, 0] = np.maximum(arm_limits[:, 0], state.joint_limits[:6, 0])
    arm_limits[:, 1] = np.minimum(arm_limits[:, 1], state.joint_limits[:6, 1])
    if np.any(arm_limits[:, 0] >= arm_limits[:, 1]):
        raise HardwareSafetyError(
            "controller and CAD commissioning limits leave an empty joint range"
        )
    tolerance = float(np.asarray(state.position_tolerances, dtype=float)[6])
    gripper_limits = (
        float(state.joint_limits[6, 0] - tolerance),
        float(state.joint_limits[6, 1] + tolerance),
    )
    combined = np.vstack([arm_limits, np.asarray(gripper_limits)])
    return CommandGate(
        np.asarray(initial_q, dtype=float),
        float(state.gripper_position_m),
        combined,
        np.asarray(max_step, dtype=float),
        joint_limit_margin_rad=0.0,
        gripper_limits_m=gripper_limits,
        # The gripper is never commanded, so this cap is only required to make
        # the unchanged measured value a valid CommandGate state.
        max_gripper_delta_m=max(1e-6, gripper_limits[1] - gripper_limits[0]),
    )


def validate_cad_fixed_feedback(
    state,
    *,
    locked_joint5_rad: float,
    locked_gripper_m: float,
    max_joint5_drift_rad: float,
    max_gripper_drift_m: float,
) -> None:
    """Fail if an actuator excluded from this demo physically changes."""

    joint5_drift = abs(float(state.q_arm[5]) - float(locked_joint5_rad))
    if joint5_drift > float(max_joint5_drift_rad):
        raise HardwareSafetyError(
            "fixed CAD joint 5 drifted "
            f"{joint5_drift:.6f} rad from its session-start position"
        )
    gripper_drift = abs(
        float(state.gripper_position_m) - float(locked_gripper_m)
    )
    if gripper_drift > float(max_gripper_drift_m):
        raise HardwareSafetyError(
            "uncommanded CAD gripper drifted "
            f"{gripper_drift:.6f} m from its session-start position"
        )


def feedback_reference_fields(
    history: TimeAlignedCommandHistory,
    feedback_time_s: float,
) -> tuple[str, float, float]:
    return (
        history.clamp_state(feedback_time_s),
        (feedback_time_s - history.newest_time_s) * 1000.0,
        (history.newest_time_s - history.oldest_time_s) * 1000.0,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Guarded five-joint M3T leader to one WidowXAI follower commissioning"
        )
    )
    parser.add_argument("--config", default="configs/cad_hardware_commissioning.yaml")
    parser.add_argument("--udp-host")
    parser.add_argument("--udp-port", type=int)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--label", default="cad-hardware-commissioning")
    parser.add_argument("--no-telemetry", action="store_true")
    parser.add_argument(
        "--live",
        action="store_true",
        help="open the official Trossen driver; omitted means in-memory dry run",
    )
    parser.add_argument("--robot-ip")
    parser.add_argument(
        "--confirm-live",
        default="",
        help="must equal LIVE-WIDOWXAI-CAD-<robot-ip>",
    )
    parser.add_argument(
        "--accept-unvalidated-mapping",
        action="store_true",
        help="required only for supervised sign commissioning while mapping is pending",
    )
    selection_group = parser.add_mutually_exclusive_group()
    selection_group.add_argument(
        "--commission-joint",
        type=int,
        choices=range(5),
        help=(
            "select one follower joint (0-4); the baseline pending profile "
            "requires this isolated form"
        ),
    )
    selection_group.add_argument(
        "--commission-joints",
        help=(
            "comma-separated follower joints; accepted only when the selected "
            "hardware profile explicitly permits that exact pending set"
        ),
    )
    parser.add_argument(
        "--deadman-device",
        default="",
        help=(
            "live only: explicit Linux evdev path for a held key, foot pedal, "
            "or gamepad button"
        ),
    )
    parser.add_argument(
        "--deadman-key-code",
        type=int,
        help="live only: Linux EV_KEY code; defaults to hardware.hold_to_run.key_code",
    )
    parser.add_argument(
        "--dry-auto-deadman",
        action="store_true",
        help="dry run only: auto-engage once after the locked stream is fresh",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    try:
        validate_cad_hardware_config(config)
    except (CadSafetyError, KeyError, TypeError, ValueError) as exc:
        raise SystemExit(f"CAD CONFIGURATION REJECTED: {exc}") from None

    hardware = config["hardware"]
    commissioning = config["commissioning"]
    activation = hardware.get("activation", {"mode": "physical_hold_to_run"})
    activation_mode = str(activation.get("mode", "physical_hold_to_run"))
    automatic_activation = activation_mode == "automatic_after_rest"
    activation_countdown_s = (
        float(activation["countdown_s"]) if automatic_activation else 0.0
    )
    robot_ip = str(args.robot_ip or hardware.get("robot_ip", "")).strip()
    if not robot_ip:
        raise SystemExit("CAD hardware robot IP is required")
    try:
        commission_joints = validate_cad_commissioning_selection(
            live=args.live,
            mapping_status=str(commissioning["mapping_status"]),
            accept_unvalidated_mapping=args.accept_unvalidated_mapping,
            commission_joint=args.commission_joint,
            commission_joints=args.commission_joints,
            allowed_pending_joint_sets=commissioning.get(
                "allowed_pending_joint_sets"
            ),
        )
    except CadSafetyError as exc:
        raise SystemExit(f"CAD COMMISSIONING SELECTION REJECTED: {exc}") from None
    physical_deadman: LinuxEvdevHoldToRun | None = None
    deadman_sample: HoldToRunSample | None = None
    deadman_device = ""
    deadman_key_code: int | str = ""
    if activation_mode == "physical_hold_to_run":
        try:
            deadman_key_code = validate_evdev_key_code(
                hardware["hold_to_run"]["key_code"]
                if args.deadman_key_code is None
                else args.deadman_key_code
            )
        except (HoldToRunError, KeyError, TypeError) as exc:
            raise SystemExit(
                f"CAD HOLD-TO-RUN CONFIGURATION REJECTED: {exc}"
            ) from None
    elif args.deadman_device or args.deadman_key_code is not None:
        raise SystemExit(
            "the automatic J0/J1/J2 profile does not use --deadman-device "
            "or --deadman-key-code"
        )
    if args.live:
        if platform.system() != "Linux":
            raise SystemExit("live CAD output requires Linux")
        expected_token = cad_live_confirmation_token(robot_ip)
        if args.confirm_live != expected_token:
            raise SystemExit(
                f"CAD live output remains disabled; pass --confirm-live {expected_token}"
            )
        if args.dry_auto_deadman:
            raise SystemExit("--dry-auto-deadman can never be combined with --live")
        if activation_mode == "physical_hold_to_run":
            deadman_device = str(args.deadman_device).strip()
            if not deadman_device:
                raise SystemExit(
                    "CAD live output requires --deadman-device "
                    "/dev/input/by-id/...; the terminal e/r latch is disabled "
                    "for live motion"
                )
            assert isinstance(deadman_key_code, int)
            physical_deadman = LinuxEvdevHoldToRun(
                deadman_device, deadman_key_code
            )
            try:
                deadman_sample = physical_deadman.open(
                    require_released=bool(
                        hardware["hold_to_run"]["require_initial_release"]
                    )
                )
            except HoldToRunError as exc:
                raise SystemExit(
                    f"CAD HOLD-TO-RUN PREFLIGHT REJECTED: {exc}"
                ) from None
        try:
            workstation = run_workstation_preflight(hardware, [robot_ip])
        except WorkstationSafetyError as exc:
            if physical_deadman is not None:
                physical_deadman.close()
            raise SystemExit(f"live workstation preflight failed: {exc}") from None
        print(
            "preflight: workstation "
            f"AC={workstation.power_supply} profile={workstation.power_profile} "
            f"robot_source={workstation.route_source_ip} "
            f"devices={list(workstation.route_devices)}"
        )
        if physical_deadman is not None:
            print(
                "preflight: physical hold-to-run released and readable "
                f"device={deadman_device} key_code={deadman_key_code}"
            )
        else:
            print(
                "preflight: J0/J1/J2 automatic activation enabled; motion "
                f"starts after rest stabilization and a {activation_countdown_s:g} s "
                "countdown"
            )

    duration_s = resolve_cad_duration(
        hardware["max_demo_duration_s"], args.duration
    )
    model = WidowXAIModel(config["model"]["xml_path"])
    rest_q = np.asarray(config["model"]["rest_q_rad"], dtype=float).reshape(6)
    anchor_q = np.asarray(
        config["model"]["command_anchor_q_rad"], dtype=float
    ).reshape(6)
    gripper_preview_m = 0.0
    if model.in_self_collision(rest_q, gripper_preview_m):
        raise SystemExit("official model reports a self-collision at all-zero rest")
    collision_alpha = model.first_self_collision_on_path(
        rest_q,
        anchor_q,
        start_gripper_q=gripper_preview_m,
        end_gripper_q=gripper_preview_m,
        samples=int(hardware["startup_collision_samples"]),
    )
    if collision_alpha is not None:
        raise SystemExit(
            "official model rejects the rest-to-command-anchor path near "
            f"{collision_alpha * 100.0:.1f}%"
        )
    cad = config["cad"]
    watchdog = CadFreshnessWatchdog(
        cad["stale_timeout_s"],
        cad["fresh_samples_to_recover"],
        np.asarray(cad["max_source_step_rad"], dtype=float),
        monitored_joints=commission_joints,
    )
    source_filter = CadJointFilter(
        minimum_cutoff_hz=cad["source_filter"]["minimum_cutoff_hz"],
        speed_coefficient=cad["source_filter"]["speed_coefficient"],
        derivative_cutoff_hz=cad["source_filter"]["derivative_cutoff_hz"],
        maximum_cutoff_hz=cad["source_filter"]["maximum_cutoff_hz"],
    )
    receiver = CadUdpReceiver(
        args.udp_host or cad["udp_host"],
        cad["udp_port"] if args.udp_port is None else args.udp_port,
        expected_names=cad["expected_names"],
        max_packet_age_s=cad["max_packet_age_s"],
        max_future_skew_s=cad["max_future_skew_s"],
        require_root_locked=True,
    )

    console = CadDeadmanConsole(latch_enabled=not args.live)
    console.start()
    if args.live and not console.interactive:
        if physical_deadman is not None:
            physical_deadman.close()
        raise SystemExit("live CAD output requires an interactive quit terminal")

    if args.live and physical_deadman is not None:
        try:
            deadman_sample = require_released_hold_to_run(
                physical_deadman,
                "immediately before robot backend construction",
            )
        except HardwareSafetyError as exc:
            physical_deadman.close()
            raise SystemExit(f"CAD HOLD-TO-RUN PREFLIGHT REJECTED: {exc}") from None
    if args.live:
        profile = str(hardware["end_effector_profile"])
        backend = TrossenArmBackend(
            robot_ip,
            command_goal_time_s=float(hardware["command_goal_time_s"]),
            end_effector_variant=END_EFFECTOR_PROFILE_TO_VARIANT[profile],
            required_driver_version=str(hardware["driver_version_tested"]),
        )
    else:
        backend = DryRunBackend(rest_q, model.joint_limits, gripper_position_m=0.0)

    telemetry = None
    if not args.no_telemetry:
        telemetry = TelemetryLogger(
            args.label,
            config,
            config["telemetry"]["output_dir"],
            columns=CAD_TELEMETRY_COLUMNS,
            ik_status_columns=(),
            strict_columns=True,
        )

    connected = False
    motion_started = False
    last_sample = None
    last_filtered_sample = None
    filter_generation = watchdog.discontinuity_generation
    controller = None
    q_command = anchor_q.copy()
    locked_joint5_q: float | None = None
    locked_gripper_m: float | None = None
    shutdown_warnings: list[str] = []
    try:
        receiver.start()
        print(
            f"preflight: waiting for three fresh locked-root M3T packets on "
            f"udp://{receiver.host}:{receiver.port}"
        )
        last_sample, freshness = wait_for_locked_cad(receiver, watchdog, 10.0)
        print(
            "preflight: locked CAD source is fresh; "
            f"sequence={last_sample.sequence} age_ms={freshness.age_s * 1000.0:.1f}"
        )
        filtered_q = source_filter.reset(
            last_sample.q, last_sample.arrival_monotonic_ns
        )
        last_filtered_sample = replace(last_sample, q=filtered_q)
        filter_generation = freshness.discontinuity_generation

        if physical_deadman is not None:
            deadman_sample = require_released_hold_to_run(
                physical_deadman,
                "immediately before robot connection",
            )

        state = backend.connect()
        connected = True
        # This five-joint demo never actuates WidowX joint 5 or the gripper.
        # Preserve their measured session-start positions rather than nudging
        # either one toward a nominal configuration value.
        locked_joint5_q = float(state.q_arm[5])
        locked_gripper_m = float(state.gripper_position_m)
        locked_arm_joints = {5: locked_joint5_q}
        session_anchor_q = apply_locked_arm_joints(anchor_q, locked_arm_joints)
        require_initial_rest(
            state,
            rest_q,
            float(hardware["startup_rest_tolerance_rad"]),
        )
        print(
            f"preflight: driver={state.driver_version} firmware={state.firmware_version} "
            f"q={np.round(state.q_arm, 4).tolist()}"
        )
        print(
            "preflight: fixed outputs "
            f"joint_5={locked_joint5_q:.6f} rad "
            f"gripper={locked_gripper_m:.6f} m; neither will receive motion"
        )
        screen_self_collision_path(
            model,
            moving_side="right" if robot_ip.endswith(".3") else "left",
            moving_start=state.q_arm,
            moving_end=session_anchor_q,
            moving_gripper_m=state.gripper_position_m,
            samples=int(hardware["startup_collision_samples"]),
            allow_marginal_start_m=float(
                hardware.get("startup_marginal_contact_m", 0.0)
            ),
        )

        startup_gate = make_startup_command_gate(
            state,
            np.asarray(hardware["startup_max_joint_delta_rad"], dtype=float),
            hardware,
        )
        backend.enable_position_control(include_gripper=False)
        motion_started = True
        backend.send_positions(
            state.q_arm,
            state.gripper_position_m,
            include_gripper=False,
        )
        anchor_is_rest = np.allclose(session_anchor_q[:5], rest_q[:5], atol=1e-9)
        print(
            "startup: stabilizing at all-zero rest; no intermediate home pose"
            if anchor_is_rest
            else "startup: ramping from all-zero rest to the CAD command anchor"
        )
        ramp_to_home(
            backend,
            startup_gate,
            state,
            session_anchor_q,
            config,
            locked_arm_joints=locked_arm_joints,
        )
        settled = backend.read_state()
        if np.max(np.abs(settled.q_arm - session_anchor_q)) > float(
            hardware["max_feedback_error_rad"]
        ):
            raise HardwareSafetyError("arm did not reach the CAD command anchor")
        validate_cad_fixed_feedback(
            settled,
            locked_joint5_rad=locked_joint5_q,
            locked_gripper_m=locked_gripper_m,
            max_joint5_drift_rad=float(hardware["fixed_joint_5_max_drift_rad"]),
            max_gripper_drift_m=float(hardware["fixed_gripper_max_drift_m"]),
        )
        if physical_deadman is not None:
            deadman_sample = require_released_hold_to_run(
                physical_deadman,
                "after the home ramp",
            )
            print(
                "rest anchor ready: hold-to-run is released; motion requires a new "
                "physical press"
            )

        command_limits = rest_command_limits(
            model.joint_limits,
            session_anchor_q,
            np.asarray(commissioning["minimum_delta_rad"], dtype=float),
            np.asarray(commissioning["maximum_delta_rad"], dtype=float),
        )
        mapper = CadRestMapper(
            rest_q=session_anchor_q,
            signs=np.asarray(cad["signs"], dtype=float),
            scales=np.asarray(cad["scales"], dtype=float),
            source_deadband_rad=np.asarray(cad["source_deadband_rad"], dtype=float),
            commission_joints=commission_joints,
            clip_to_command_limits=bool(
                commissioning.get("clip_to_command_limits", False)
            ),
            command_limits=command_limits,
        )
        controller = CadDeadmanController(mapper)
        limiter = limiter_from_config(
            config["control"]["joint_command_limits"], session_anchor_q
        )
        if limiter is None:
            raise HardwareSafetyError("CAD hardware output requires a motion limiter")
        gate = make_cad_command_gate(
            settled,
            session_anchor_q,
            command_limits,
            np.asarray(commissioning["max_command_step_rad"], dtype=float),
        )
        q_command = session_anchor_q.copy()
        q_des = session_anchor_q.copy()
        q_feedback = settled.q_arm.copy()
        q_feedback_reference = session_anchor_q.copy()
        q_feedback_error = q_feedback - q_feedback_reference
        gripper_feedback = float(settled.gripper_position_m)
        history = TimeAlignedCommandHistory(
            session_anchor_q,
            time.perf_counter(),
            float(hardware["feedback_tracking_delay_s"]),
        )

        selection = (
            "all mapped joints"
            if commission_joints is None
            else (
                "ONLY follower joints "
                + ", ".join(
                    f"{joint} ({cad['expected_names'][joint]})"
                    for joint in commission_joints
                )
            )
        )
        if physical_deadman is not None:
            deadman_instruction = (
                f"Hold key code {deadman_key_code} continuously to move; release "
                "to hold; s or q + Enter returns to rest and stops."
            )
        elif automatic_activation:
            deadman_instruction = (
                f"Motion starts automatically after a {activation_countdown_s:g} s "
                "countdown; s or q + Enter returns to rest and stops."
            )
        else:
            deadman_instruction = (
                "Type e + Enter to engage, r + Enter to hold, and s or q + Enter "
                "to return to rest and stop."
            )
        print(
            f"CAD follower ready for at most {duration_s:g} s; {selection}; gripper "
            f"and joint 5 are fixed. {deadman_instruction}"
        )
        loop_hz = float(config["control"]["loop_rate_hz"])
        period = 1.0 / loop_hz
        feedback_period = 1.0 / float(hardware["feedback_check_rate_hz"])
        loop_started = time.perf_counter()
        run_started = None if automatic_activation else loop_started
        auto_start_deadline = loop_started + activation_countdown_s
        previous_tick = loop_started
        next_tick = loop_started
        next_feedback = loop_started
        last_status = loop_started
        auto_engaged = False
        last_countdown_second: int | None = None
        last_event = "rest_anchor_ready" if anchor_is_rest else "command_anchor_ready"
        observed_invalid_packets = receiver.invalid_packets

        while True:
            loop_now = time.perf_counter()
            if run_started is not None and loop_now - run_started >= duration_s:
                break
            console.poll()
            if console.quit_requested:
                break
            sample, _generation = receiver.mailbox.take_latest()
            now_ns = time.perf_counter_ns()
            if sample is not None:
                last_sample = sample
                freshness = watchdog.observe(sample, now_ns)
            else:
                freshness = watchdog.poll(now_ns)
            if freshness.discontinuity_generation != filter_generation:
                source_filter.reset()
                last_filtered_sample = None
                filter_generation = freshness.discontinuity_generation
            if sample is not None:
                filtered_q = source_filter.update(
                    sample.q, sample.arrival_monotonic_ns
                )
                last_filtered_sample = replace(sample, q=filtered_q)

            source_fault: HardwareSafetyError | None = None
            if receiver.invalid_packets > observed_invalid_packets:
                source_fault = HardwareSafetyError(
                    "CAD receiver rejected a packet: "
                    + (receiver.last_error or "unknown packet validation error")
                )
            elif not freshness.fresh:
                source_fault = HardwareSafetyError(
                    "CAD source became unsafe: "
                    + (freshness.last_discontinuity or "stream is not fresh")
                )
            observed_invalid_packets = receiver.invalid_packets

            if (
                args.dry_auto_deadman
                and not automatic_activation
                and freshness.fresh
                and not auto_engaged
            ):
                console.set_pressed(True)
                auto_engaged = True

            cycle_fault: HardwareSafetyError | None = source_fault
            if automatic_activation:
                remaining_s = max(0.0, auto_start_deadline - time.perf_counter())
                countdown_second = int(np.ceil(remaining_s))
                if not auto_engaged and countdown_second != last_countdown_second:
                    print(
                        f"AUTOMATIC J012 START IN {countdown_second} s; "
                        "keep the workspace clear; S/Q + Enter stops"
                    )
                    last_countdown_second = countdown_second
                if not auto_engaged and remaining_s <= 0.0:
                    auto_engaged = True
                    console.set_pressed(True)
                    run_started = time.perf_counter()
                    print(
                        f"AUTOMATIC J012 FOLLOWING STARTED for {duration_s:g} s"
                    )
                deadman_pressed = auto_engaged
            elif physical_deadman is not None:
                try:
                    deadman_sample = physical_deadman.sample(now_ns)
                    deadman_pressed = deadman_sample.pressed
                except HoldToRunError as exc:
                    deadman_pressed = False
                    if cycle_fault is None:
                        cycle_fault = HardwareSafetyError(str(exc))
            else:
                deadman_pressed = console.pressed

            if cycle_fault is not None:
                controller.fault(False)
                target = None
                last_event = f"safety_stop:{cycle_fault}"
                print(f"CAD SAFETY STOP REQUESTED: {cycle_fault}")
            else:
                try:
                    target = controller.update(
                        deadman_pressed=deadman_pressed,
                        stream_fresh=freshness.fresh,
                        sample=last_filtered_sample,
                        # Anchor to the last accepted command, never to delayed
                        # encoder feedback (which would create a chasing loop).
                        robot_q=q_command,
                    )
                    last_event = ""
                except CadSafetyError as exc:
                    target = None
                    cycle_fault = HardwareSafetyError(
                        f"CAD mapping became unsafe: {exc}"
                    )
                    last_event = f"mapping_stop:{exc}"
                    print(f"CAD SAFETY STOP REQUESTED: {cycle_fault}")

            now = time.perf_counter()
            dt = bounded_command_period(max(1e-6, now - previous_tick), loop_hz)
            previous_tick = now
            limiter_flags: list[str] = []
            if target is None:
                q_des = q_command.copy()
                limiter.reset(q_command)
                candidate = q_command.copy()
            else:
                q_des = target
                limited = limiter.step(q_des, dt)
                candidate = limited.command
                limiter_flags = limited.flags("joint")

            require_cad_joint5_locked(candidate, locked_joint5_q)
            if model.in_self_collision(candidate, gripper_feedback):
                raise HardwareSafetyError("official model predicts a self-collision")
            gate.validate(candidate, gripper_feedback)
            send_started_ns = time.perf_counter_ns()
            backend.send_positions(
                candidate,
                locked_gripper_m,
                include_gripper=False,
            )
            send_finished_ns = time.perf_counter_ns()
            q_command = candidate
            history.append(send_finished_ns / 1e9, q_command)

            feedback_read_ns = 0
            feedback_sample_fresh = False
            feedback_state = ""
            feedback_newest_age_ms: float | str = ""
            feedback_history_span_ms: float | str = ""
            pending_fault: HardwareSafetyError | None = cycle_fault
            if send_finished_ns / 1e9 >= next_feedback:
                feedback = backend.read_state()
                feedback_read_ns = time.perf_counter_ns()
                feedback_sample_fresh = True
                q_feedback = feedback.q_arm.copy()
                gripper_feedback = float(feedback.gripper_position_m)
                try:
                    validate_cad_fixed_feedback(
                        feedback,
                        locked_joint5_rad=locked_joint5_q,
                        locked_gripper_m=locked_gripper_m,
                        max_joint5_drift_rad=float(
                            hardware["fixed_joint_5_max_drift_rad"]
                        ),
                        max_gripper_drift_m=float(
                            hardware["fixed_gripper_max_drift_m"]
                        ),
                    )
                    q_feedback_reference, q_feedback_error = (
                        validate_time_aligned_feedback(
                            q_feedback,
                            feedback_read_ns / 1e9,
                            history,
                            float(hardware["max_feedback_error_rad"]),
                        )
                    )
                except HardwareSafetyError as exc:
                    if pending_fault is None:
                        pending_fault = exc
                    q_feedback_reference = history.reference_at(feedback_read_ns / 1e9)
                    q_feedback_error = q_feedback - q_feedback_reference
                (
                    feedback_state,
                    feedback_newest_age_ms,
                    feedback_history_span_ms,
                ) = feedback_reference_fields(history, feedback_read_ns / 1e9)
                next_feedback = feedback_read_ns / 1e9 + feedback_period

            if telemetry is not None:
                telemetry.log(
                    pc_epoch_ns=time.time_ns(),
                    pc_monotonic_ns=time.perf_counter_ns(),
                    cad_sequence="" if last_sample is None else last_sample.sequence,
                    cad_source_time_ns=""
                    if last_sample is None
                    else last_sample.source_time_ns,
                    cad_frame_time_ns=(
                        ""
                        if last_sample is None or last_sample.frame_time_ns is None
                        else last_sample.frame_time_ns
                    ),
                    cad_frame_timestamp_domain=(
                        "" if last_sample is None else last_sample.frame_time_domain
                    ),
                    cad_frame_skew_ms=(
                        ""
                        if last_sample is None or last_sample.frame_skew_ms is None
                        else last_sample.frame_skew_ms
                    ),
                    cad_capture_to_publish_ms=(
                        ""
                        if last_sample is None or last_sample.frame_time_ns is None
                        else (
                            last_sample.source_time_ns - last_sample.frame_time_ns
                        )
                        / 1e6
                    ),
                    cad_publish_to_arrival_ms=(
                        ""
                        if last_sample is None
                        else (
                            last_sample.arrival_epoch_ns
                            - last_sample.source_time_ns
                        )
                        / 1e6
                    ),
                    cad_frame_to_arrival_ms=(
                        ""
                        if last_sample is None or last_sample.frame_time_ns is None
                        else (
                            last_sample.arrival_epoch_ns
                            - last_sample.frame_time_ns
                        )
                        / 1e6
                    ),
                    cad_arrival_monotonic_ns=(
                        "" if last_sample is None else last_sample.arrival_monotonic_ns
                    ),
                    cad_arrival_epoch_ns=(
                        "" if last_sample is None else last_sample.arrival_epoch_ns
                    ),
                    cad_source_age_ms=""
                    if last_sample is None
                    else (time.time_ns() - last_sample.source_time_ns) / 1e6,
                    cad_arrival_age_ms=""
                    if last_sample is None
                    else (now_ns - last_sample.arrival_monotonic_ns) / 1e6,
                    cad_root_locked=False
                    if last_sample is None
                    else last_sample.root_locked,
                    receiver_packets=receiver.received_packets,
                    receiver_valid_packets=receiver.valid_packets,
                    receiver_invalid_packets=receiver.invalid_packets,
                    receiver_last_error=receiver.last_error or "",
                    cad_fresh=freshness.fresh,
                    recovery_streak=freshness.recovery_streak,
                    discontinuity_generation=freshness.discontinuity_generation,
                    last_discontinuity=freshness.last_discontinuity,
                    deadman_pressed=deadman_pressed,
                    deadman_engaged=controller.engaged,
                    deadman_needs_release=controller.needs_release,
                    deadman_source=(
                        "automatic_after_rest"
                        if automatic_activation
                        else (
                            "linux_evdev_key"
                            if physical_deadman is not None
                            else "dry_terminal_latch"
                        )
                    ),
                    deadman_device=deadman_device,
                    deadman_key_code=(
                        deadman_key_code if physical_deadman is not None else ""
                    ),
                    deadman_state_age_ms=(
                        ""
                        if deadman_sample is None
                        else (
                            time.perf_counter_ns() - deadman_sample.monotonic_ns
                        )
                        / 1e6
                    ),
                    deadman_press_generation=(
                        deadman_sample.press_generation
                        if deadman_sample is not None
                        else console.press_generation
                    ),
                    deadman_release_generation=(
                        deadman_sample.release_generation
                        if deadman_sample is not None
                        else console.release_generation
                    ),
                    reanchor_generation=mapper.reanchor_generation,
                    commission_joint=""
                    if commission_joints is None
                    else "|".join(str(joint) for joint in commission_joints),
                    hardware_live=args.live,
                    command_send_monotonic_ns=send_finished_ns,
                    command_send_epoch_ns=time.time_ns(),
                    command_send_duration_ms=(
                        send_finished_ns - send_started_ns
                    )
                    / 1e6,
                    feedback_read_monotonic_ns=feedback_read_ns,
                    feedback_sample_fresh=feedback_sample_fresh,
                    feedback_reference_state=feedback_state,
                    feedback_newest_command_age_ms=feedback_newest_age_ms,
                    feedback_history_span_ms=feedback_history_span_ms,
                    q_source="" if last_sample is None else last_sample.q,
                    q_source_filtered=""
                    if last_filtered_sample is None
                    else last_filtered_sample.q,
                    source_filter_alpha=source_filter.last_alpha,
                    source_filter_cutoff_hz=source_filter.last_cutoff_hz,
                    q_des=q_des,
                    q_cmd=q_command,
                    q_feedback=q_feedback,
                    q_feedback_reference=q_feedback_reference,
                    q_feedback_error=q_feedback_error,
                    gripper_feedback_m=gripper_feedback,
                    limiter_flags="|".join(limiter_flags),
                    event=last_event,
                    fault_reason="" if pending_fault is None else str(pending_fault),
                )
            if pending_fault is not None:
                raise pending_fault

            if now - last_status >= 1.0:
                print(
                    "CAD_HW "
                    f"live={int(args.live)} fresh={int(freshness.fresh)} "
                    "commission_joints="
                    f"{'all' if commission_joints is None else ','.join(str(joint) for joint in commission_joints)} "
                    f"age_ms={freshness.age_s * 1000.0:.1f} "
                    f"deadman={int(deadman_pressed)} engaged={int(controller.engaged)} "
                    f"q_cmd={np.array2string(q_command, precision=4, suppress_small=True)} "
                    f"max_feedback_error={np.max(np.abs(q_feedback_error)):.4f}"
                )
                last_status = now

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0.0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()
    except KeyboardInterrupt:
        print("operator stopped CAD follower")
    except (CadSafetyError, HardwareSafetyError, HardwareUnavailableError) as exc:
        shutdown_warnings.append(str(exc))
        print(f"CAD SAFETY STOP: {exc}")
    finally:
        console.set_pressed(False)
        if controller is not None:
            controller.fault(False)
        receiver.stop()
        if physical_deadman is not None:
            physical_deadman.close()
        if connected and motion_started:
            try:
                return_to_rest(
                    backend,
                    model,
                    config,
                    control_gripper=False,
                    locked_arm_joints=(
                        None
                        if locked_joint5_q is None
                        else {5: locked_joint5_q}
                    ),
                    locked_gripper_position_m=locked_gripper_m,
                )
            except Exception as exc:  # Cleanup must continue even if the driver is lost.
                shutdown_warnings.append(f"return to rest failed: {exc}")
                try:
                    backend.safe_hold()
                except Exception as hold_exc:
                    shutdown_warnings.append(f"measured hold failed: {hold_exc}")
        backend.close()
        if telemetry is not None:
            telemetry.close()
            print(f"telemetry: {telemetry.run_dir}")
        if shutdown_warnings:
            print("CAD FOLLOWER ENDED WITH SAFETY WARNINGS:")
            for warning in shutdown_warnings:
                print(f"  - {warning}")
            if args.live:
                print("Confirm the arm is at rest; cut controller power if it is not.")
    if shutdown_warnings:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
