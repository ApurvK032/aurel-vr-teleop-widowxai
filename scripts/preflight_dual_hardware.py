from __future__ import annotations

import argparse

import numpy as np

from widowxai_quest_teleop.config import (
    DUAL_ARM_SIDES,
    DualArmConfigError,
    dual_live_confirmation_token,
    load_config,
    parse_dual_arm_config,
    require_live_dual_arm_config,
)
from widowxai_quest_teleop.dual_arm_coordinator import build_dual_arm_system
from widowxai_quest_teleop.hardware import (
    END_EFFECTOR_PROFILE_TO_VARIANT,
    HardwareSafetyError,
    HardwareUnavailableError,
    TrossenArmBackend,
)


def offline_checks(config: dict, arms_config: dict) -> None:
    """Everything that must pass before either controller is contacted."""

    print("=== offline dual-arm checks ===")
    for side in DUAL_ARM_SIDES:
        arm = arms_config[side]
        print(
            f"{side:>5} arm: controller={arm.controller_hand} mapping={arm.mapping_mode} "
            f"ip={arm.robot_ip} calibration={arm.calibration_status} "
            f"base={arm.placement.measurement_status}"
        )
    print(f"base separation: {config['_dual_arm']['base_separation_m']:.3f} m")

    home_q = np.asarray(config["model"]["simulation_start_q_rad"], dtype=float).reshape(6)
    hardware = config["hardware"]
    gripper_open = float(hardware["gripper_open_m"])
    rest_q = np.asarray(hardware["rest_q_rad"], dtype=float).reshape(6)
    rest_gripper = float(hardware["rest_gripper_m"])

    arms, collision_model, _ = build_dual_arm_system(
        config,
        arms_config,
        initial_q=home_q,
        initial_gripper_m=gripper_open,
        control_gripper=bool(hardware.get("control_gripper", True)),
    )
    home_q = arms["left"].model.clamp_joints(home_q)
    samples = int(hardware["startup_collision_samples"])

    both = lambda value: {side: value for side in DUAL_ARM_SIDES}  # noqa: E731

    # Simultaneous rest -> home and home -> rest. Both arms are interpolated
    # together, so a path that is clear for each arm alone but collides when
    # they move at the same time is rejected here rather than on the bench.
    for name, start_q, end_q, start_g, end_g in (
        ("rest -> home (simultaneous)", rest_q, home_q, rest_gripper, rest_gripper),
        ("gripper open at home", home_q, home_q, rest_gripper, gripper_open),
        ("home -> rest (simultaneous)", home_q, rest_q, gripper_open, gripper_open),
        ("gripper close at rest", rest_q, rest_q, gripper_open, rest_gripper),
    ):
        found = collision_model.first_collision_on_path(
            both(start_q),
            both(end_q),
            start_gripper=both(start_g),
            end_gripper=both(end_g),
            samples=samples,
        )
        if found is not None:
            alpha, report = found
            raise HardwareSafetyError(
                f"combined scene rejects {name} near {alpha * 100:.1f}%: {report.describe()}"
            )
        print(f"  combined path clear: {name}")

    # One arm moving while the other holds, in both directions.
    for moving in DUAL_ARM_SIDES:
        holding = [side for side in DUAL_ARM_SIDES if side != moving][0]
        found = collision_model.first_collision_on_path(
            {moving: rest_q, holding: home_q},
            {moving: home_q, holding: home_q},
            start_gripper=both(gripper_open),
            end_gripper=both(gripper_open),
            samples=samples,
        )
        if found is not None:
            alpha, report = found
            raise HardwareSafetyError(
                f"combined scene rejects {moving} rest->home while {holding} holds home "
                f"near {alpha * 100:.1f}%: {report.describe()}"
            )
        print(f"  combined path clear: {moving} moves while {holding} holds")

    report = collision_model.check(both(home_q), both(gripper_open))
    print(
        f"  home/home clearance: {report.separation_m:.3f} m "
        f"(configured margin {config['safety']['cross_arm_clearance_m']:.3f} m)"
    )
    print("offline dual-arm checks PASSED")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Validate a dual-arm profile and read both WidowXAI controllers "
            "without enabling position mode or sending any command"
        )
    )
    parser.add_argument("--config", default="configs/dual_widowxai.yaml")
    parser.add_argument(
        "--end-effector-profile",
        choices=tuple(END_EFFECTOR_PROFILE_TO_VARIANT),
        help="defaults to the selected config; legacy_1_8 matches the proven local setup",
    )
    parser.add_argument(
        "--contact-arms",
        action="store_true",
        help=(
            "additionally open a read-only driver session to each controller; "
            "omit to run the offline checks only"
        ),
    )
    args = parser.parse_args()

    config = load_config(args.config)
    try:
        arms_config = parse_dual_arm_config(config)
    except DualArmConfigError as exc:
        raise SystemExit(f"PREFLIGHT FAILED: invalid dual-arm configuration: {exc}") from None

    try:
        offline_checks(config, arms_config)
    except (HardwareSafetyError, ValueError) as exc:
        raise SystemExit(f"PREFLIGHT FAILED: {exc}") from None

    print()
    print(f"dual live confirmation token: {dual_live_confirmation_token(arms_config)}")
    try:
        require_live_dual_arm_config(config, arms_config)
        print("live dual-arm gates: SATISFIED")
    except DualArmConfigError as exc:
        print(f"live dual-arm gates: BLOCKED — {exc}")

    if not args.contact_arms:
        print()
        print("no controller was contacted; pass --contact-arms for the read-only state preflight")
        return

    hardware = config["hardware"]
    profile = args.end_effector_profile or hardware.get("end_effector_profile")
    if not profile:
        raise SystemExit("PREFLIGHT FAILED: an end-effector profile is required")
    variant = END_EFFECTOR_PROFILE_TO_VARIANT[str(profile)]

    print()
    print("=== read-only controller state (no position mode, no commands) ===")
    backends = {}
    try:
        for side in DUAL_ARM_SIDES:
            arm = arms_config[side]
            backend = TrossenArmBackend(
                arm.robot_ip,
                end_effector_variant=variant,
                required_driver_version=str(hardware["driver_version_tested"]),
            )
            backends[side] = backend
            state = backend.connect()
            print(f"{side:>5} arm @ {arm.robot_ip}")
            print(f"        driver={state.driver_version} firmware={state.firmware_version}")
            print(f"        q_rad={np.round(state.q_arm, 6).tolist()}")
            print(f"        gripper_m={state.gripper_position_m:.6f}")
        print("NO-MOTION DUAL PREFLIGHT PASSED for both controllers")
        print("position mode was not enabled on either arm; no command was sent")
    except (HardwareSafetyError, HardwareUnavailableError) as exc:
        connected = sorted(backends)
        raise SystemExit(
            f"PREFLIGHT FAILED: {exc}; sessions opened for {connected} were closed"
        ) from None
    finally:
        for backend in backends.values():
            try:
                backend.close()
            except Exception as exc:  # noqa: BLE001 - report, never mask cleanup
                print(f"WARNING: driver cleanup failed: {exc}")


if __name__ == "__main__":
    main()
