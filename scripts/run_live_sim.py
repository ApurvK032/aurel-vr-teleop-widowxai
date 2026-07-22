from __future__ import annotations

import argparse
import time

import mujoco
import numpy as np

from widowxai_quest_teleop.clutch import ClutchController
from widowxai_quest_teleop.config import load_config
from widowxai_quest_teleop.decoupled_ik import DecoupledIK
from widowxai_quest_teleop.gripper import trigger_to_gripper_position
from widowxai_quest_teleop.mapping import ClutchPoseMapper
from widowxai_quest_teleop.model import WidowXAIModel
from widowxai_quest_teleop.motion_limiter import bounded_command_period, limiter_from_config
from widowxai_quest_teleop.pose_filter import ControllerPoseFilter
from widowxai_quest_teleop.safety import FreshSequenceWatchdog
from widowxai_quest_teleop.telemetry import TelemetryLogger
from widowxai_quest_teleop.transport import QuestReceiver
from widowxai_quest_teleop.viewer import close_passive_viewer


def main() -> None:
    parser = argparse.ArgumentParser(description="Drive WidowXAI MuJoCo simulation from a Quest 3")
    parser.add_argument("--config", default="configs/baseline.yaml")
    parser.add_argument(
        "--inline-viewer",
        action="store_true",
        help="diagnostic only; reference operation uses the separate viewer_client.py",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="0 runs until Ctrl+C")
    parser.add_argument("--label", default="quest-live-sim")
    args = parser.parse_args()

    config = load_config(args.config)
    quest_config = config["quest"]
    model = WidowXAIModel(config["model"]["xml_path"])
    solver = DecoupledIK.from_config(model, config)
    q_start = model.clamp_joints(np.asarray(config["model"]["simulation_start_q_rad"], dtype=float))
    mapper = ClutchPoseMapper(
        quest_config["calibration"],
        translation_scale=quest_config["translation_scale"],
        rotation_scale=quest_config["rotation_scale"],
        position_reach_limit_m=quest_config["position_reach_limit_m"],
        rotation_reach_limit_rad=quest_config["rotation_reach_limit_rad"],
    )
    clutch = ClutchController(mapper)
    watchdog = FreshSequenceWatchdog(
        quest_config["stale_timeout_s"], quest_config["fresh_samples_to_recover"]
    )
    receiver = QuestReceiver(quest_config["websocket_url"])
    receiver.start()

    sim_data = mujoco.MjData(model.model)
    hardware_config = config.get("hardware", {})
    gripper_open = float(hardware_config.get("gripper_open_m", 0.044))
    gripper_min = float(hardware_config.get("gripper_min_demo_m", 0.0))
    gripper_q = gripper_open
    model.set_viewer_qpos(sim_data, q_start, gripper_q)
    viewer = None
    if args.inline_viewer:
        from mujoco import viewer as mujoco_viewer

        viewer = mujoco_viewer.launch_passive(model.model, sim_data)

    loop_hz = float(config["control"]["loop_rate_hz"])
    dt = 1.0 / loop_hz
    update_mode = config["control"].get("update_mode", "fixed_rate")
    if update_mode not in ("fixed_rate", "quest_synchronized"):
        raise SystemExit(f"unknown control.update_mode: {update_mode}")
    quest_synchronized = update_mode == "quest_synchronized"
    controller_filter = ControllerPoseFilter(config["control"])
    joint_limiter = limiter_from_config(
        config["control"].get("joint_command_limits"),
        q_start,
    )
    gripper_limiter = limiter_from_config(
        config["control"].get("gripper_command_limits"),
        np.array([gripper_q]),
    )
    q_des = q_start.copy()
    q_command = q_start.copy()
    gripper_des = gripper_q
    target_pose = None
    last_sample = None
    filtered_controller = None
    diagnostics = None
    started = time.perf_counter()
    next_tick_s = started
    last_command_send_s = started
    mailbox_generation = 0

    try:
        with TelemetryLogger(args.label, config, config["telemetry"]["output_dir"]) as telemetry:
            while args.duration <= 0.0 or time.perf_counter() - started < args.duration:
                if quest_synchronized:
                    sample, mailbox_generation = receiver.mailbox.wait_take_latest(
                        mailbox_generation,
                        dt,
                    )
                else:
                    sample, _ = receiver.mailbox.take_latest()
                control_consume_ns = time.perf_counter_ns()
                if sample is not None:
                    last_sample = sample
                    freshness = watchdog.observe(sample)
                else:
                    freshness = watchdog.poll()

                if quest_synchronized and sample is None:
                    if last_sample is not None and not freshness.fresh:
                        robot_pose, wrist_pose = model.fk(q_command)
                        target_pose = clutch.update(
                            grip=last_sample.grip,
                            stream_fresh=False,
                            controller_pose=filtered_controller or last_sample.controller_pose,
                            robot_pose=robot_pose,
                            wrist_pivot=wrist_pose.position,
                            head_quaternion_wxyz=last_sample.head_quaternion_wxyz,
                        )
                        if joint_limiter is not None:
                            joint_limiter.reset(q_command)
                        if gripper_limiter is not None:
                            gripper_limiter.reset(np.array([gripper_q]))
                        if not mapper.engaged:
                            filtered_controller = None
                            controller_filter.reset()
                    continue

                robot_pose, wrist_pose = model.fk(q_command)
                pose_sample = sample if quest_synchronized else last_sample
                if pose_sample is not None:
                    filtered_controller = controller_filter.update(
                        pose_sample.controller_pose,
                        pose_sample.capture_monotonic_ms / 1000.0,
                    )
                    target_pose = clutch.update(
                        grip=pose_sample.grip,
                        stream_fresh=freshness.fresh,
                        controller_pose=filtered_controller,
                        robot_pose=robot_pose,
                        wrist_pivot=wrist_pose.position,
                        head_quaternion_wxyz=pose_sample.head_quaternion_wxyz,
                    )
                else:
                    target_pose = None

                target_active = target_pose is not None and freshness.fresh and mapper.engaged
                limiter_flags: list[str] = []
                if target_active:
                    ik_start = time.perf_counter_ns()
                    q_des, diagnostics = solver.solve(target_pose, q_command)
                    ik_end = time.perf_counter_ns()
                    gripper_des = trigger_to_gripper_position(
                        last_sample.trigger,
                        gripper_open,
                        gripper_min,
                    )
                else:
                    ik_start = ik_end = 0
                    diagnostics = None
                    q_des = q_command.copy()
                    gripper_des = gripper_q
                    if joint_limiter is not None:
                        joint_limiter.reset(q_command)
                    if gripper_limiter is not None:
                        gripper_limiter.reset(np.array([gripper_q]))
                if not mapper.engaged:
                    filtered_controller = None
                    controller_filter.reset()
                if target_active:
                    limiter_dt = bounded_command_period(
                        time.perf_counter() - last_command_send_s,
                        loop_hz,
                    )
                    if joint_limiter is None:
                        q_command = q_des.copy()
                    else:
                        joint_result = joint_limiter.step(q_des, limiter_dt)
                        q_command = joint_result.command
                        limiter_flags.extend(joint_result.flags("joint"))
                    if gripper_limiter is None:
                        gripper_q = gripper_des
                    else:
                        gripper_result = gripper_limiter.step(np.array([gripper_des]), limiter_dt)
                        gripper_q = float(gripper_result.command[0])
                        limiter_flags.extend(gripper_result.flags("gripper"))
                model.set_viewer_qpos(sim_data, q_command, gripper_q)
                q_feedback = sim_data.qpos[model.qpos_indices].copy()
                _, wrist_pose = model.fk(q_feedback)
                receiver.publish(
                    {
                        "type": "ik_state",
                        "teleop_id": args.label,
                        "q_arm": q_command.tolist(),
                        "gripper_q": float(gripper_q),
                    }
                )
                command_send_ns = time.perf_counter_ns()
                last_command_send_s = command_send_ns / 1e9

                telemetry.log(
                    pc_epoch_ns=time.time_ns(),
                    pc_monotonic_ns=time.perf_counter_ns(),
                    quest_sequence="" if last_sample is None else last_sample.sequence,
                    quest_capture_monotonic_ms="" if last_sample is None else last_sample.capture_monotonic_ms,
                    quest_capture_epoch_ms="" if last_sample is None else last_sample.capture_epoch_ms,
                    quest_send_monotonic_ms="" if last_sample is None else last_sample.send_monotonic_ms,
                    pc_socket_arrival_monotonic_ns="" if last_sample is None else last_sample.pc_arrival_monotonic_ns,
                    control_consume_monotonic_ns=control_consume_ns,
                    ik_start_monotonic_ns=ik_start,
                    ik_end_monotonic_ns=ik_end,
                    command_send_monotonic_ns=command_send_ns,
                    reconnect_generation="" if last_sample is None else last_sample.reconnect_generation,
                    quest_grip="" if last_sample is None else last_sample.grip,
                    quest_trigger="" if last_sample is None else last_sample.trigger,
                    stream_fresh=freshness.fresh,
                    clutch_engaged=mapper.engaged,
                    reanchor_generation=mapper.reanchor_generation,
                    raw_controller_position="" if last_sample is None else last_sample.controller_pose.position,
                    raw_controller_quaternion_wxyz="" if last_sample is None else last_sample.controller_pose.quaternion_wxyz,
                    pose_filter_rotation_alpha=controller_filter.rotation_alpha,
                    pose_filter_rotation_cutoff_hz=controller_filter.rotation_cutoff_hz,
                    head_quaternion_wxyz="" if last_sample is None else last_sample.head_quaternion_wxyz,
                    engage_head_yaw_rad="" if mapper.engage_head_yaw_rad is None else mapper.engage_head_yaw_rad,
                    mapped_target_position="" if target_pose is None else target_pose.position,
                    mapped_target_quaternion_wxyz="" if target_pose is None else target_pose.quaternion_wxyz,
                    wrist_current_position=wrist_pose.position,
                    q_des=q_des,
                    q_cmd=q_command,
                    q_feedback=q_feedback,
                    gripper_des_m=gripper_des,
                    gripper_cmd_m=gripper_q,
                    gripper_feedback_m=gripper_q,
                    position_residual_m="" if diagnostics is None else diagnostics.position_residual_m,
                    orientation_residual_rad="" if diagnostics is None else diagnostics.orientation_residual_rad,
                    position_manipulability="" if diagnostics is None else diagnostics.position_manipulability,
                    rotation_manipulability="" if diagnostics is None else diagnostics.rotation_manipulability,
                    position_damping="" if diagnostics is None else diagnostics.position_damping,
                    rotation_damping="" if diagnostics is None else diagnostics.rotation_damping,
                    ik_step_norm_rad="" if diagnostics is None else diagnostics.step_norm_rad,
                    minimum_joint_limit_margin_rad="" if diagnostics is None else diagnostics.minimum_joint_limit_margin_rad,
                    ik_status="" if diagnostics is None else diagnostics.status,
                    limiter_flags="|".join(limiter_flags),
                )
                if viewer is not None:
                    viewer.sync()
                    if not viewer.is_running():
                        break
                if quest_synchronized:
                    continue
                # Match the reference workflow: send immediately after IK, then
                # wait for the next absolute control deadline.
                next_tick_s += dt
                sleep_for = next_tick_s - time.perf_counter()
                if sleep_for > 0.0:
                    time.sleep(sleep_for)
                else:
                    next_tick_s = time.perf_counter()
            print(f"telemetry: {telemetry.run_dir}")
    except KeyboardInterrupt:
        print("stopped; viewer held its last commanded pose")
    finally:
        receiver.stop()
        if viewer is not None:
            close_passive_viewer(viewer)


if __name__ == "__main__":
    main()
