from __future__ import annotations

import argparse
import time

import mujoco
import numpy as np

from widowxai_quest_teleop.command_shaper import JointCommandShaper
from widowxai_quest_teleop.config import load_config
from widowxai_quest_teleop.decoupled_ik import DecoupledIK
from widowxai_quest_teleop.math3d import quat_to_matrix
from widowxai_quest_teleop.model import WidowXAIModel
from widowxai_quest_teleop.safety import wait_for_cycle_period
from widowxai_quest_teleop.telemetry import TelemetryLogger


def reference_q(q_start: np.ndarray, elapsed_s: float) -> np.ndarray:
    phase = 2.0 * np.pi * elapsed_s / 4.0
    offsets = np.array(
        [
            0.10 * np.sin(phase),
            0.08 * np.sin(phase + 0.3),
            0.07 * np.sin(phase - 0.5),
            0.10 * np.sin(phase * 0.7),
            0.08 * np.sin(phase * 0.8 + 0.2),
            0.12 * np.sin(phase * 0.6 - 0.2),
        ]
    )
    return q_start + offsets


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the decoupled WidowXAI pipeline in MuJoCo")
    parser.add_argument("--config", default="configs/baseline.yaml")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--label", default="synthetic-decoupled-sim")
    args = parser.parse_args()

    config = load_config(args.config)
    kinematics = WidowXAIModel(config["model"].get("simulation_scene_xml_path", config["model"]["xml_path"]))
    solver = DecoupledIK.from_config(kinematics, config)
    q_start = kinematics.clamp_joints(np.asarray(config["model"]["simulation_start_q_rad"], dtype=float))
    shaper = JointCommandShaper.from_config(q_start, kinematics.joint_limits, config)

    sim_data = mujoco.MjData(kinematics.model)
    sim_data.qpos[kinematics.qpos_indices] = q_start
    sim_data.ctrl[:6] = q_start
    sim_data.ctrl[6] = 0.044
    mujoco.mj_forward(kinematics.model, sim_data)

    command_hz = float(config["control"]["command_rate_hz"])
    ik_hz = float(config["control"]["ik_rate_hz"])
    dt = 1.0 / command_hz
    ik_stride = max(1, round(command_hz / ik_hz))
    physics_steps = max(1, round(dt / kinematics.model.opt.timestep))
    q_des = q_start.copy()
    diagnostics = None
    viewer = None
    if args.viewer:
        from mujoco import viewer as mujoco_viewer

        viewer = mujoco_viewer.launch_passive(kinematics.model, sim_data)

    started = time.perf_counter()
    tick = 0
    with TelemetryLogger(args.label, config, config["telemetry"]["output_dir"]) as telemetry:
        while time.perf_counter() - started < args.duration:
            tick_started = time.perf_counter()
            elapsed = tick * dt
            q_feedback = sim_data.qpos[kinematics.qpos_indices].copy()
            target_q = kinematics.clamp_joints(reference_q(q_start, elapsed))
            target_pose, _ = kinematics.fk(target_q)
            if tick % ik_stride == 0:
                ik_start = time.perf_counter_ns()
                q_des, diagnostics = solver.solve(target_pose, q_des)
                ik_end = time.perf_counter_ns()
            else:
                ik_start = ik_end = 0

            q_cmd = shaper.step(q_des, dt)
            sim_data.ctrl[:6] = q_cmd
            kinematics.set_target_pose(sim_data, target_pose)
            for _ in range(physics_steps):
                mujoco.mj_step(kinematics.model, sim_data)
            q_feedback = sim_data.qpos[kinematics.qpos_indices].copy()
            current_ee, current_wrist = kinematics.fk(q_feedback)
            target_rotation = quat_to_matrix(target_pose.quaternion_wxyz)
            current_rotation = quat_to_matrix(current_ee.quaternion_wxyz)
            local_offset = current_rotation.T @ (current_wrist.position - current_ee.position)
            wrist_target = target_pose.position + target_rotation @ local_offset

            telemetry.log(
                pc_epoch_ns=time.time_ns(),
                pc_monotonic_ns=time.perf_counter_ns(),
                control_consume_monotonic_ns=int(tick_started * 1e9),
                ik_start_monotonic_ns=ik_start,
                ik_end_monotonic_ns=ik_end,
                command_send_monotonic_ns=time.perf_counter_ns(),
                stream_fresh=True,
                clutch_engaged=False,
                mapped_target_position=target_pose.position,
                mapped_target_quaternion_wxyz=target_pose.quaternion_wxyz,
                wrist_target_position=wrist_target,
                wrist_current_position=current_wrist.position,
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
            if args.realtime or args.viewer:
                wait_for_cycle_period(tick_started, dt)
        run_dir = telemetry.run_dir
    if viewer is not None:
        viewer.close()
    print(f"simulation complete: {tick} command ticks")
    print(f"telemetry: {run_dir}")


if __name__ == "__main__":
    main()
