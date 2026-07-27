from __future__ import annotations

import argparse
import time

import mujoco
import numpy as np

from widowxai_quest_teleop.config import (
    DUAL_ARM_SIDES,
    DualArmConfigError,
    load_config,
    parse_dual_arm_config,
)
from widowxai_quest_teleop.dual_arm_coordinator import build_dual_arm_system
from widowxai_quest_teleop.motion_limiter import (
    bounded_command_period,
    configured_command_spacing_stage,
    configured_minimum_command_interval,
    minimum_command_spacing_wait,
)
from widowxai_quest_teleop.telemetry import DUAL_ARM_TELEMETRY_COLUMNS, TelemetryLogger
from widowxai_quest_teleop.transport import BimanualQuestReceiver
from widowxai_quest_teleop.viewer import close_passive_viewer


def dual_arm_telemetry_record(
    tick,
    *,
    consume_ns: int,
    send_ns_by_side: dict[str, int],
    send_epoch_ns_by_side: dict[str, int],
    sample,
    overwrite_count: int,
    pre_consume_wait_s: float,
    pre_send_wait_s: float,
    arms,
) -> dict[str, object]:
    """Flatten one coordinated tick into a single dual-arm telemetry row."""

    sends = [value for value in send_ns_by_side.values() if value]
    skew_ms = (max(sends) - min(sends)) / 1e6 if len(sends) == 2 else ""
    record: dict[str, object] = {
        "pc_epoch_ns": time.time_ns(),
        "pc_monotonic_ns": time.perf_counter_ns(),
        "quest_sequence": "" if sample is None else sample.sequence,
        "quest_capture_monotonic_ms": "" if sample is None else sample.capture_monotonic_ms,
        "quest_capture_epoch_ms": "" if sample is None else sample.capture_epoch_ms,
        "quest_send_monotonic_ms": "" if sample is None else sample.send_monotonic_ms,
        "pc_socket_arrival_monotonic_ns": "" if sample is None else sample.pc_arrival_monotonic_ns,
        "pc_socket_arrival_epoch_ns": "" if sample is None else sample.pc_arrival_epoch_ns,
        "control_consume_monotonic_ns": consume_ns,
        "command_spacing_wait_ms": (pre_consume_wait_s + pre_send_wait_s) * 1000.0,
        "command_pre_consume_wait_ms": pre_consume_wait_s * 1000.0,
        "command_pre_send_wait_ms": pre_send_wait_s * 1000.0,
        "mailbox_overwrite_count": overwrite_count,
        "reconnect_generation": "" if sample is None else sample.reconnect_generation,
        "head_quaternion_wxyz": "" if sample is None else sample.head_quaternion_wxyz,
        "command_skew_ms": skew_ms,
        "cross_arm_collision": tick.collision.colliding,
        "cross_arm_collision_kind": tick.collision.kind,
        "cross_arm_collision_bodies": (
            "" if tick.collision.bodies is None else "|".join(tick.collision.bodies)
        ),
        "cross_arm_separation_m": (
            "" if not np.isfinite(tick.collision.separation_m) else tick.collision.separation_m
        ),
        "coordinated_hold": "|".join(tick.held_sides),
        "fault_reason": tick.fault_reason,
    }
    for side in DUAL_ARM_SIDES:
        proposal = tick.proposals[side]
        arm = arms[side]
        arm_sample = proposal.sample
        diagnostics = proposal.diagnostics
        record.update(
            {
                f"{side}_controller_hand": arm.controller_hand,
                f"{side}_mapping_mode": arm.mapping_mode,
                f"{side}_tracked": proposal.tracked,
                f"{side}_grip": "" if arm_sample is None else arm_sample.grip,
                f"{side}_trigger": "" if arm_sample is None else arm_sample.trigger,
                f"{side}_stream_fresh": proposal.freshness.fresh,
                f"{side}_stream_age_s": proposal.freshness.age_s,
                f"{side}_clutch_engaged": proposal.engaged,
                f"{side}_reanchor_generation": arm.mapper.reanchor_generation,
                f"{side}_raw_controller_position": (
                    "" if arm_sample is None else arm_sample.controller_pose.position
                ),
                f"{side}_raw_controller_quaternion_wxyz": (
                    "" if arm_sample is None else arm_sample.controller_pose.quaternion_wxyz
                ),
                f"{side}_pose_filter_rotation_alpha": arm.controller_filter.rotation_alpha,
                f"{side}_pose_filter_rotation_cutoff_hz": arm.controller_filter.rotation_cutoff_hz,
                f"{side}_engage_head_yaw_rad": (
                    "" if arm.mapper.engage_head_yaw_rad is None else arm.mapper.engage_head_yaw_rad
                ),
                f"{side}_mapped_target_position": (
                    "" if proposal.target_pose is None else proposal.target_pose.position
                ),
                f"{side}_mapped_target_quaternion_wxyz": (
                    "" if proposal.target_pose is None else proposal.target_pose.quaternion_wxyz
                ),
                f"{side}_ik_start_monotonic_ns": proposal.ik_start_monotonic_ns,
                f"{side}_ik_end_monotonic_ns": proposal.ik_end_monotonic_ns,
                f"{side}_command_send_monotonic_ns": send_ns_by_side.get(side, 0),
                f"{side}_command_send_epoch_ns": send_epoch_ns_by_side.get(side, 0),
                f"{side}_q_des": proposal.q_des,
                f"{side}_q_cmd": arm.q_command,
                f"{side}_q_feedforward_velocity": arm.feedforward_velocity,
                f"{side}_q_feedback": arm.q_feedback,
                f"{side}_gripper_des_m": proposal.gripper_des_m,
                f"{side}_gripper_cmd_m": arm.gripper_command_m,
                f"{side}_gripper_feedback_m": arm.gripper_feedback_m,
                f"{side}_position_residual_m": (
                    "" if diagnostics is None else diagnostics.position_residual_m
                ),
                f"{side}_orientation_residual_rad": (
                    "" if diagnostics is None else diagnostics.orientation_residual_rad
                ),
                f"{side}_ik_step_norm_rad": "" if diagnostics is None else diagnostics.step_norm_rad,
                f"{side}_minimum_joint_limit_margin_rad": (
                    "" if diagnostics is None else diagnostics.minimum_joint_limit_margin_rad
                ),
                f"{side}_ik_status": "" if diagnostics is None else diagnostics.status,
                f"{side}_limiter_flags": "|".join(proposal.limiter_flags),
                f"{side}_held": side in tick.held_sides,
                f"{side}_rejected": not tick.accepted,
            }
        )
    return record


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Drive two WidowXAI arms in one MuJoCo scene from both Quest controllers"
    )
    parser.add_argument("--config", default="configs/dual_widowxai.yaml")
    parser.add_argument("--duration", type=float, default=0.0, help="0 runs until Ctrl+C")
    parser.add_argument("--label", default="dual-widowxai-sim")
    parser.add_argument(
        "--inline-viewer",
        action="store_true",
        help="diagnostic only; shows both arms in the combined scene",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    try:
        arms_config = parse_dual_arm_config(config)
    except DualArmConfigError as exc:
        raise SystemExit(f"invalid dual-arm configuration: {exc}") from None

    for side in DUAL_ARM_SIDES:
        arm = arms_config[side]
        if not arm.calibration_accepted:
            print(
                f"NOTE: {side} arm calibration is {arm.calibration_status!r}. "
                "Simulation is permitted; live output is not."
            )

    home_q = np.asarray(config["model"]["simulation_start_q_rad"], dtype=float).reshape(6)
    hardware = config.get("hardware", {}) or {}
    gripper_open = float(hardware.get("gripper_open_m", 0.044))
    control_gripper = bool(hardware.get("control_gripper", True))

    arms, collision_model, coordinator = build_dual_arm_system(
        config,
        arms_config,
        initial_q=home_q,
        initial_gripper_m=gripper_open,
        control_gripper=control_gripper,
    )
    home_q = arms["left"].model.clamp_joints(home_q)

    start_report = collision_model.check(
        {side: home_q for side in DUAL_ARM_SIDES},
        {side: gripper_open for side in DUAL_ARM_SIDES},
    )
    if start_report.colliding:
        raise SystemExit(
            "the configured home pose already fails the combined scene: "
            f"{start_report.describe()}; re-measure the base transforms"
        )
    print(
        f"combined scene: home pose clear, closest cross-arm separation "
        f"{start_report.separation_m:.3f} m at "
        f"{config['_dual_arm']['base_separation_m']:.3f} m base separation"
    )

    receiver = BimanualQuestReceiver(
        config["quest"]["websocket_url"],
        mapping_modes={side: arms_config[side].mapping_mode for side in DUAL_ARM_SIDES},
    )
    receiver.start()

    sim_data = mujoco.MjData(collision_model.model)
    collision_model.set_viewer_qpos(
        sim_data,
        {side: home_q for side in DUAL_ARM_SIDES},
        {side: gripper_open for side in DUAL_ARM_SIDES},
    )
    viewer = None
    if args.inline_viewer:
        from mujoco import viewer as mujoco_viewer

        viewer = mujoco_viewer.launch_passive(collision_model.model, sim_data)

    control = config["control"]
    loop_hz = float(control["loop_rate_hz"])
    dt = 1.0 / loop_hz
    quest_synchronized = control.get("update_mode", "fixed_rate") == "quest_synchronized"
    try:
        minimum_command_interval_s = configured_minimum_command_interval(control)
        command_spacing_stage = configured_command_spacing_stage(control)
    except ValueError as exc:
        receiver.stop()
        raise SystemExit(f"invalid command spacing: {exc}") from None

    started = time.perf_counter()
    next_tick_s = started
    last_command_send_s = started
    mailbox_generation = 0

    try:
        with TelemetryLogger(
            args.label,
            config,
            config["telemetry"]["output_dir"],
            columns=DUAL_ARM_TELEMETRY_COLUMNS,
            ik_status_columns=tuple(f"{side}_ik_status" for side in DUAL_ARM_SIDES),
            strict_columns=True,
        ) as telemetry:
            while args.duration <= 0.0 or time.perf_counter() - started < args.duration:
                pre_consume_wait_s = 0.0
                pre_send_wait_s = 0.0
                if quest_synchronized:
                    if command_spacing_stage == "before_consume":
                        pre_consume_wait_s = minimum_command_spacing_wait(
                            last_command_send_s, minimum_command_interval_s, time.perf_counter()
                        )
                        if pre_consume_wait_s > 0.0:
                            time.sleep(pre_consume_wait_s)
                    sample, mailbox_generation = receiver.mailbox.wait_take_latest(
                        mailbox_generation, dt
                    )
                else:
                    sample, _ = receiver.mailbox.take_latest()
                consume_ns = time.perf_counter_ns()

                limiter_elapsed_s = time.perf_counter() - last_command_send_s
                if quest_synchronized and command_spacing_stage == "before_send":
                    limiter_elapsed_s = max(limiter_elapsed_s, minimum_command_interval_s)
                limiter_dt = bounded_command_period(limiter_elapsed_s, loop_hz)

                tick = coordinator.step(sample, limiter_dt=limiter_dt)

                if quest_synchronized and sample is None and not any(
                    proposal.active for proposal in tick.proposals.values()
                ):
                    continue

                if quest_synchronized and command_spacing_stage == "before_send":
                    pre_send_wait_s = minimum_command_spacing_wait(
                        last_command_send_s, minimum_command_interval_s, time.perf_counter()
                    )
                    if pre_send_wait_s > 0.0:
                        time.sleep(pre_send_wait_s)

                collision_model.set_viewer_qpos(
                    sim_data,
                    {side: arms[side].q_command for side in DUAL_ARM_SIDES},
                    {side: arms[side].gripper_command_m for side in DUAL_ARM_SIDES},
                )
                send_ns_by_side = {}
                send_epoch_ns_by_side = {}
                for side in DUAL_ARM_SIDES:
                    send_ns_by_side[side] = time.perf_counter_ns()
                    send_epoch_ns_by_side[side] = time.time_ns()
                    arms[side].q_feedback = sim_data.qpos[
                        collision_model.arm_qpos_indices[side]
                    ].copy()
                    arms[side].gripper_feedback_m = arms[side].gripper_command_m
                last_command_send_s = max(send_ns_by_side.values()) / 1e9

                telemetry.log(
                    **dual_arm_telemetry_record(
                        tick,
                        consume_ns=consume_ns,
                        send_ns_by_side=send_ns_by_side,
                        send_epoch_ns_by_side=send_epoch_ns_by_side,
                        sample=sample,
                        overwrite_count=receiver.mailbox.overwrite_count,
                        pre_consume_wait_s=pre_consume_wait_s,
                        pre_send_wait_s=pre_send_wait_s,
                        arms=arms,
                    )
                )

                if viewer is not None:
                    viewer.sync()
                    if not viewer.is_running():
                        break
                if quest_synchronized:
                    continue
                next_tick_s += dt
                sleep_for = next_tick_s - time.perf_counter()
                if sleep_for > 0.0:
                    time.sleep(sleep_for)
                else:
                    next_tick_s = time.perf_counter()
            print(
                f"telemetry: {telemetry.run_dir} "
                f"(rejected ticks {coordinator.rejected_ticks}, "
                f"stream-loss ticks {coordinator.stream_loss_ticks})"
            )
    except KeyboardInterrupt:
        print("stopped; both arms hold their last commanded pose")
    finally:
        receiver.stop()
        if viewer is not None:
            close_passive_viewer(viewer)


if __name__ == "__main__":
    main()
