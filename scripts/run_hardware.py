from __future__ import annotations

import argparse
import platform
import time

import numpy as np

from widowxai_quest_teleop.clutch import ClutchController
from widowxai_quest_teleop.config import load_config
from widowxai_quest_teleop.decoupled_ik import DecoupledIK
from widowxai_quest_teleop.hardware import (
    CommandGate,
    DryRunBackend,
    HardwareSafetyError,
    HardwareUnavailableError,
    TrossenArmBackend,
)
from widowxai_quest_teleop.mapping import ClutchPoseMapper
from widowxai_quest_teleop.model import WidowXAIModel
from widowxai_quest_teleop.pose_filter import pose_ema
from widowxai_quest_teleop.safety import FreshSequenceWatchdog
from widowxai_quest_teleop.telemetry import TelemetryLogger
from widowxai_quest_teleop.transport import QuestReceiver


def wait_for_released_quest(receiver: QuestReceiver, timeout_s: float, stale_timeout_s: float) -> None:
    """Require live tracking and a released deadman before any position mode is enabled."""
    deadline = time.perf_counter() + float(timeout_s)
    watchdog = FreshSequenceWatchdog(stale_timeout_s, fresh_samples_to_recover=3)
    while time.perf_counter() < deadline:
        sample, _ = receiver.mailbox.take_latest()
        if sample is not None:
            freshness = watchdog.observe(sample)
            if freshness.fresh and sample.grip < 0.65:
                return
        time.sleep(0.01)
    raise HardwareSafetyError(
        "Quest preflight failed: enter WebXR, keep the grip released, and provide fresh tracking"
    )


def ramp_to_rest(backend, gate: CommandGate, start, rest_q: np.ndarray, config: dict) -> None:
    hardware = config["hardware"]
    duration = float(hardware["startup_ramp_duration_s"])
    rate = float(hardware["startup_ramp_rate_hz"])
    steps = max(1, round(duration * rate))
    period = 1.0 / rate
    start_q = start.q_arm.copy()
    start_gripper = float(start.gripper_position_m)
    goal_gripper = float(hardware["gripper_open_m"])
    next_tick = time.perf_counter()
    for index in range(1, steps + 1):
        alpha = index / steps
        q = start_q + alpha * (rest_q - start_q)
        gripper = start_gripper + alpha * (goal_gripper - start_gripper)
        gate.validate(q, gripper)
        backend.send_positions(q, gripper)
        next_tick += period
        sleep_for = next_tick - time.perf_counter()
        if sleep_for > 0.0:
            time.sleep(sleep_for)
        else:
            next_tick = time.perf_counter()


def main() -> None:
    parser = argparse.ArgumentParser(description="Fail-closed WidowXAI physical demo")
    parser.add_argument("--config", default="configs/hardware_demo.yaml")
    parser.add_argument("--live", action="store_true", help="use the official Trossen hardware driver")
    parser.add_argument("--robot-ip", help="required for --live; never inferred")
    parser.add_argument("--confirm-live", default="", help="must equal LIVE-WIDOWXAI-<robot-ip>")
    parser.add_argument("--duration", type=float, default=0.0, help="0 uses the configured demo limit")
    parser.add_argument("--label", default="widowxai-hardware-demo")
    args = parser.parse_args()

    config = load_config(args.config)
    hardware = config["hardware"]
    if not hardware.get("enabled") or not hardware.get("require_explicit_enable"):
        raise SystemExit("hardware demo configuration is not explicitly gated")

    model = WidowXAIModel(config["model"]["xml_path"])
    solver = DecoupledIK.from_config(model, config)
    rest_q = model.clamp_joints(np.asarray(config["model"]["simulation_start_q_rad"], dtype=float))
    max_delta = np.asarray(config["ik"]["max_dq_per_joint_rad"], dtype=float)

    robot_ip = args.robot_ip or hardware.get("robot_ip")
    if args.live:
        if platform.system() not in ("Linux", "Darwin"):
            raise SystemExit(
                "live hardware is unavailable on native Windows: Trossen's official driver "
                "supports Ubuntu and macOS only; no arm connection was attempted"
            )
        if not robot_ip:
            raise SystemExit("--robot-ip is required for a live demo")
        expected = f"LIVE-WIDOWXAI-{robot_ip}"
        if args.confirm_live != expected:
            raise SystemExit(f"live output remains disabled; pass --confirm-live {expected}")
        backend = TrossenArmBackend(robot_ip)
    else:
        backend = DryRunBackend(rest_q, model.joint_limits)

    quest = config["quest"]
    mapper = ClutchPoseMapper(
        quest["calibration"],
        translation_scale=quest["translation_scale"],
        rotation_scale=quest["rotation_scale"],
        position_reach_limit_m=quest["position_reach_limit_m"],
        rotation_reach_limit_rad=quest["rotation_reach_limit_rad"],
    )
    clutch = ClutchController(mapper)
    watchdog = FreshSequenceWatchdog(quest["stale_timeout_s"], quest["fresh_samples_to_recover"])
    receiver = QuestReceiver(quest["websocket_url"])
    receiver.start()

    connected = False
    try:
        print("preflight: waiting for fresh Quest tracking with grip released")
        wait_for_released_quest(receiver, 10.0, quest["stale_timeout_s"])
        state = backend.connect()
        connected = True
        print(f"preflight: firmware={state.firmware_version} q={np.round(state.q_arm, 3).tolist()}")

        combined_limits = state.joint_limits.copy()
        combined_limits[:6, 0] = np.maximum(combined_limits[:6, 0], model.joint_limits[:, 0])
        combined_limits[:6, 1] = np.minimum(combined_limits[:6, 1], model.joint_limits[:, 1])
        ramp_gate = CommandGate(
            state.q_arm,
            state.gripper_position_m,
            combined_limits,
            max_delta,
            # The official sleep pose lies on lower limits; the ramp only moves inward.
            joint_limit_margin_rad=0.0,
            gripper_limits_m=(state.joint_limits[6, 0], state.joint_limits[6, 1]),
            max_gripper_delta_m=hardware["max_gripper_delta_m"],
        )

        if args.live:
            phrase = f"ARM READY {robot_ip}"
            typed = input(
                f"Clear the workspace, be ready to cut controller power, and type '{phrase}': "
            ).strip()
            if typed != phrase:
                raise HardwareSafetyError("operator confirmation did not match")

        backend.enable_position_control()
        backend.send_positions(state.q_arm, state.gripper_position_m)
        print("startup: ramping from measured pose to the official WidowXAI staged pose")
        ramp_to_rest(backend, ramp_gate, state, rest_q, config)
        settled = backend.read_state()
        if np.max(np.abs(settled.q_arm - rest_q)) > hardware["max_feedback_error_rad"]:
            raise HardwareSafetyError("arm did not reach the staged posture within the feedback limit")
        print("preflight: release grip again before teleoperation")
        wait_for_released_quest(receiver, 10.0, quest["stale_timeout_s"])

        command_gate = CommandGate(
            rest_q,
            hardware["gripper_open_m"],
            combined_limits,
            max_delta,
            joint_limit_margin_rad=hardware["joint_limit_margin_rad"],
            gripper_limits_m=(hardware["gripper_min_demo_m"], hardware["gripper_open_m"]),
            max_gripper_delta_m=hardware["max_gripper_delta_m"],
        )
        q_command = rest_q.copy()
        gripper_command = float(hardware["gripper_open_m"])
        q_feedback = q_command.copy()
        last_sample = None
        filtered_controller = None
        target_pose = None
        diagnostics = None

        loop_hz = float(config["control"]["loop_rate_hz"])
        period = 1.0 / loop_hz
        feedback_stride = max(1, round(loop_hz / float(hardware["feedback_check_rate_hz"])))
        duration_limit = float(hardware["max_demo_duration_s"])
        duration = duration_limit if args.duration <= 0.0 else min(float(args.duration), duration_limit)
        started = time.perf_counter()
        next_tick = started
        tick = 0

        mode = "LIVE" if args.live else "DRY RUN"
        print(f"{mode}: 200 Hz direct commands, 50% spatial gain, grip is the deadman")
        with TelemetryLogger(args.label, config, config["telemetry"]["output_dir"]) as telemetry:
            while time.perf_counter() - started < duration:
                sample, _ = receiver.mailbox.take_latest()
                if sample is not None:
                    last_sample = sample
                    freshness = watchdog.observe(sample)
                else:
                    freshness = watchdog.poll()

                robot_pose, wrist_pose = model.fk(q_command)
                if last_sample is not None:
                    filtered_controller = pose_ema(
                        filtered_controller,
                        last_sample.controller_pose,
                        config["control"]["pose_filter_alpha"],
                    )
                    target_pose = clutch.update(
                        grip=last_sample.grip,
                        stream_fresh=freshness.fresh,
                        controller_pose=filtered_controller,
                        robot_pose=robot_pose,
                        wrist_pivot=wrist_pose.position,
                        head_quaternion_wxyz=last_sample.head_quaternion_wxyz,
                    )
                else:
                    target_pose = None

                target_active = target_pose is not None and freshness.fresh and mapper.engaged
                if target_active:
                    ik_start = time.perf_counter_ns()
                    q_command, diagnostics = solver.solve(target_pose, q_command)
                    ik_end = time.perf_counter_ns()
                    trigger = float(np.clip(last_sample.trigger, 0.0, 1.0))
                    gripper_command = hardware["gripper_open_m"] - trigger * (
                        hardware["gripper_open_m"] - hardware["gripper_min_demo_m"]
                    )
                else:
                    ik_start = ik_end = 0
                    diagnostics = None
                if not mapper.engaged:
                    filtered_controller = None

                command_gate.validate(q_command, gripper_command)
                backend.send_positions(q_command, gripper_command)
                command_send_ns = time.perf_counter_ns()

                if tick % feedback_stride == 0:
                    feedback = backend.read_state()
                    q_feedback = feedback.q_arm
                    if np.max(np.abs(q_feedback - q_command)) > hardware["max_feedback_error_rad"]:
                        raise HardwareSafetyError("measured joint tracking error exceeded the demo limit")

                telemetry.log(
                    pc_epoch_ns=time.time_ns(),
                    pc_monotonic_ns=time.perf_counter_ns(),
                    quest_sequence="" if last_sample is None else last_sample.sequence,
                    quest_capture_monotonic_ms="" if last_sample is None else last_sample.capture_monotonic_ms,
                    pc_socket_arrival_monotonic_ns="" if last_sample is None else last_sample.pc_arrival_monotonic_ns,
                    control_consume_monotonic_ns=time.perf_counter_ns(),
                    ik_start_monotonic_ns=ik_start,
                    ik_end_monotonic_ns=ik_end,
                    command_send_monotonic_ns=command_send_ns,
                    quest_grip="" if last_sample is None else last_sample.grip,
                    quest_trigger="" if last_sample is None else last_sample.trigger,
                    stream_fresh=freshness.fresh,
                    clutch_engaged=mapper.engaged,
                    reanchor_generation=mapper.reanchor_generation,
                    mapped_target_position="" if target_pose is None else target_pose.position,
                    mapped_target_quaternion_wxyz="" if target_pose is None else target_pose.quaternion_wxyz,
                    q_des=q_command,
                    q_cmd=q_command,
                    q_feedback=q_feedback,
                    position_residual_m="" if diagnostics is None else diagnostics.position_residual_m,
                    orientation_residual_rad="" if diagnostics is None else diagnostics.orientation_residual_rad,
                    ik_step_norm_rad="" if diagnostics is None else diagnostics.step_norm_rad,
                    minimum_joint_limit_margin_rad="" if diagnostics is None else diagnostics.minimum_joint_limit_margin_rad,
                    ik_status="" if diagnostics is None else diagnostics.status,
                    limiter_flags="",
                )

                tick += 1
                next_tick += period
                sleep_for = next_tick - time.perf_counter()
                if sleep_for > 0.0:
                    time.sleep(sleep_for)
                else:
                    next_tick = time.perf_counter()
            print(f"telemetry: {telemetry.run_dir}")
    except KeyboardInterrupt:
        print("operator stop: commanding measured hold before driver cleanup")
    except (HardwareSafetyError, HardwareUnavailableError) as exc:
        raise SystemExit(f"SAFETY STOP: {exc}") from None
    finally:
        receiver.stop()
        if connected:
            backend.safe_hold()
        backend.close()


if __name__ == "__main__":
    main()
