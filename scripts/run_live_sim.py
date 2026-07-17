from __future__ import annotations

import argparse
import time

import mujoco
import numpy as np

from widowxai_quest_teleop.clutch import ClutchController
from widowxai_quest_teleop.command_shaper import JointCommandShaper
from widowxai_quest_teleop.config import load_config
from widowxai_quest_teleop.decoupled_ik import DecoupledIK
from widowxai_quest_teleop.mapping import ClutchPoseMapper
from widowxai_quest_teleop.model import WidowXAIModel
from widowxai_quest_teleop.safety import FreshSequenceWatchdog, wait_for_cycle_period
from widowxai_quest_teleop.telemetry import TelemetryLogger
from widowxai_quest_teleop.transport import QuestReceiver


def main() -> None:
    parser = argparse.ArgumentParser(description="Drive WidowXAI MuJoCo simulation from a Quest 3")
    parser.add_argument("--config", default="configs/baseline.yaml")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0, help="0 runs until Ctrl+C")
    parser.add_argument("--label", default="quest-live-sim")
    args = parser.parse_args()

    config = load_config(args.config)
    quest_config = config["quest"]
    model = WidowXAIModel(config["model"].get("simulation_scene_xml_path", config["model"]["xml_path"]))
    solver = DecoupledIK.from_config(model, config)
    q_start = model.clamp_joints(np.asarray(config["model"]["simulation_start_q_rad"], dtype=float))
    shaper = JointCommandShaper.from_config(q_start, model.joint_limits, config)
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
    sim_data.qpos[model.qpos_indices] = q_start
    sim_data.ctrl[:6] = q_start
    sim_data.ctrl[6] = 0.044
    mujoco.mj_forward(model.model, sim_data)
    viewer = None
    if not args.headless:
        from mujoco import viewer as mujoco_viewer

        viewer = mujoco_viewer.launch_passive(model.model, sim_data)

    command_hz = float(config["control"]["command_rate_hz"])
    ik_hz = float(config["control"]["ik_rate_hz"])
    dt = 1.0 / command_hz
    ik_stride = max(1, round(command_hz / ik_hz))
    physics_steps = max(1, round(dt / model.model.opt.timestep))
    q_des = q_start.copy()
    target_pose = None
    last_sample = None
    diagnostics = None
    motion_active = False
    tick = 0
    started = time.perf_counter()

    try:
        with TelemetryLogger(args.label, config, config["telemetry"]["output_dir"]) as telemetry:
            while args.duration <= 0.0 or time.perf_counter() - started < args.duration:
                tick_started = time.perf_counter()
                q_feedback = sim_data.qpos[model.qpos_indices].copy()
                robot_pose, wrist_pose = model.fk(q_feedback)
                sample, _ = receiver.mailbox.take_latest()
                if sample is not None:
                    last_sample = sample
                    freshness = watchdog.observe(sample)
                    target_pose = clutch.update(
                        grip=sample.grip,
                        stream_fresh=freshness.fresh,
                        controller_pose=sample.controller_pose,
                        robot_pose=robot_pose,
                        wrist_pivot=wrist_pose.position,
                    )
                else:
                    freshness = watchdog.poll()
                    if not freshness.fresh and mapper.engaged:
                        clutch.force_reanchor()
                        target_pose = None

                target_active = target_pose is not None and freshness.fresh and mapper.engaged
                just_engaged = target_active and not motion_active
                if target_active:
                    motion_active = True
                    if not just_engaged and tick % ik_stride == 0:
                        ik_start = time.perf_counter_ns()
                        q_des, diagnostics = solver.solve(target_pose, q_des)
                        ik_end = time.perf_counter_ns()
                    else:
                        ik_start = ik_end = 0
                    q_cmd = shaper.step(q_des, dt)
                else:
                    ik_start = ik_end = 0
                    if motion_active:
                        q_des = shaper.hold()
                        diagnostics = None
                    motion_active = False
                    q_cmd = shaper.q_command.copy()
                sim_data.ctrl[:6] = q_cmd
                if last_sample is not None:
                    sim_data.ctrl[6] = 0.044 * (1.0 - np.clip(last_sample.trigger, 0.0, 1.0))
                for _ in range(physics_steps):
                    mujoco.mj_step(model.model, sim_data)

                telemetry.log(
                    pc_epoch_ns=time.time_ns(),
                    pc_monotonic_ns=time.perf_counter_ns(),
                    quest_sequence="" if last_sample is None else last_sample.sequence,
                    quest_capture_monotonic_ms="" if last_sample is None else last_sample.capture_monotonic_ms,
                    quest_capture_epoch_ms="" if last_sample is None else last_sample.capture_epoch_ms,
                    quest_send_monotonic_ms="" if last_sample is None else last_sample.send_monotonic_ms,
                    pc_socket_arrival_monotonic_ns="" if last_sample is None else last_sample.pc_arrival_monotonic_ns,
                    control_consume_monotonic_ns=time.perf_counter_ns(),
                    ik_start_monotonic_ns=ik_start,
                    ik_end_monotonic_ns=ik_end,
                    command_send_monotonic_ns=time.perf_counter_ns(),
                    reconnect_generation="" if last_sample is None else last_sample.reconnect_generation,
                    quest_grip="" if last_sample is None else last_sample.grip,
                    quest_trigger="" if last_sample is None else last_sample.trigger,
                    stream_fresh=freshness.fresh,
                    clutch_engaged=mapper.engaged,
                    reanchor_generation=mapper.reanchor_generation,
                    raw_controller_position="" if last_sample is None else last_sample.controller_pose.position,
                    raw_controller_quaternion_wxyz="" if last_sample is None else last_sample.controller_pose.quaternion_wxyz,
                    mapped_target_position="" if target_pose is None else target_pose.position,
                    mapped_target_quaternion_wxyz="" if target_pose is None else target_pose.quaternion_wxyz,
                    wrist_current_position=wrist_pose.position,
                    q_des=q_des,
                    q_cmd=q_cmd,
                    q_feedback=q_feedback,
                    position_residual_m="" if diagnostics is None else diagnostics.position_residual_m,
                    orientation_residual_rad="" if diagnostics is None else diagnostics.orientation_residual_rad,
                    position_manipulability="" if diagnostics is None else diagnostics.position_manipulability,
                    rotation_manipulability="" if diagnostics is None else diagnostics.rotation_manipulability,
                    position_damping="" if diagnostics is None else diagnostics.position_damping,
                    rotation_damping="" if diagnostics is None else diagnostics.rotation_damping,
                    ik_step_norm_rad="" if diagnostics is None else diagnostics.step_norm_rad,
                    minimum_joint_limit_margin_rad="" if diagnostics is None else diagnostics.minimum_joint_limit_margin_rad,
                    ik_status="" if diagnostics is None else diagnostics.status,
                    limiter_flags=shaper.last_limiter_flags,
                )
                if viewer is not None:
                    viewer.sync()
                    if not viewer.is_running():
                        break
                tick += 1
                wait_for_cycle_period(tick_started, dt)
            print(f"telemetry: {telemetry.run_dir}")
    except KeyboardInterrupt:
        print("stopped; simulation held its last safe command")
    finally:
        receiver.stop()
        if viewer is not None:
            viewer.close()


if __name__ == "__main__":
    main()
