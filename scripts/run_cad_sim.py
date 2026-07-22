from __future__ import annotations

import argparse
import csv
import json
import re
import select
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

from widowxai_quest_teleop.cad_input import (
    CadDeadmanController,
    CadFreshnessWatchdog,
    CadRestMapper,
    CadSafetyError,
    CadUdpReceiver,
    rest_command_limits,
)
from widowxai_quest_teleop.config import load_config, resolve_project_path
from widowxai_quest_teleop.hardware import CommandGate, HardwareSafetyError
from widowxai_quest_teleop.model import WidowXAIModel
from widowxai_quest_teleop.motion_limiter import limiter_from_config
from widowxai_quest_teleop.viewer import close_passive_viewer


CAD_TELEMETRY_COLUMNS = (
    "pc_epoch_ns",
    "pc_monotonic_ns",
    "cad_sequence",
    "cad_source_time_ns",
    "cad_source_age_ms",
    "cad_arrival_age_ms",
    "cad_fresh",
    "recovery_streak",
    "discontinuity_generation",
    "last_discontinuity",
    "deadman_pressed",
    "deadman_engaged",
    "deadman_needs_release",
    "reanchor_generation",
    "q_source",
    "q_des",
    "q_cmd",
    "q_feedback",
    "limiter_flags",
    "event",
)


def _encode(value):
    if isinstance(value, np.ndarray):
        return json.dumps(value.tolist(), separators=(",", ":"))
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(value, separators=(",", ":"))
    return value


class CadTelemetryLogger:
    def __init__(self, label: str, config: dict, output_dir: str | Path) -> None:
        safe_label = re.sub(r"[^A-Za-z0-9_.-]+", "-", label).strip("-") or "cad-rest-sim"
        stamp = time.strftime("%Y%m%d-%H%M%S")
        suffix = time.time_ns() % 1_000_000_000
        self.run_dir = resolve_project_path(output_dir) / f"{stamp}-{suffix:09d}_{safe_label}"
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.csv_path = self.run_dir / "cad_telemetry.csv"
        self._handle = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=CAD_TELEMETRY_COLUMNS)
        self._writer.writeheader()
        self.rows = 0
        (self.run_dir / "config_snapshot.json").write_text(
            json.dumps(config, indent=2, default=str), encoding="utf-8"
        )

    def log(self, **record) -> None:
        self._writer.writerow(
            {name: _encode(record.get(name, "")) for name in CAD_TELEMETRY_COLUMNS}
        )
        self.rows += 1

    def close(self) -> None:
        if self._handle.closed:
            return
        self._handle.flush()
        self._handle.close()
        (self.run_dir / "summary.json").write_text(
            json.dumps(
                {
                    "rows": self.rows,
                    "telemetry_csv": str(self.csv_path),
                    "hardware_output": False,
                    "start_and_end_pose": "rest",
                },
                indent=2,
            ),
            encoding="utf-8",
        )


class SimulationDeadmanConsole:
    """Simulation-only latch used to exercise deadman/re-anchor semantics."""

    def __init__(self) -> None:
        self._pressed = False
        self._quit = False
        self._interactive = False

    @property
    def pressed(self) -> bool:
        return self._pressed

    @property
    def quit_requested(self) -> bool:
        return self._quit

    def set_pressed(self, value: bool) -> None:
        self._pressed = bool(value)

    def start(self) -> None:
        if not sys.stdin.isatty():
            print("stdin is not interactive; simulation deadman remains released")
            return
        self._interactive = True

    def poll(self) -> None:
        if not self._interactive:
            return
        readable, _writable, _exceptional = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return
        command = sys.stdin.readline().strip().lower()
        if command in ("e", "engage"):
            self._pressed = True
            print("SIM deadman ENGAGED; type 'r' + Enter to release")
        elif command in ("r", "release"):
            self._pressed = False
            print("SIM deadman released")
        elif command in ("q", "quit", ""):
            self._pressed = False
            self._quit = True
        else:
            print("simulation commands: e=engage, r=release, q=quit")


def make_command_gate(
    rest_q: np.ndarray,
    command_limits: np.ndarray,
    model: WidowXAIModel,
    config: dict,
    gripper_q: float,
) -> CommandGate:
    combined = np.vstack([command_limits, np.array([0.0, 0.044])])
    return CommandGate(
        rest_q,
        gripper_q,
        combined,
        np.asarray(config["commissioning"]["max_command_step_rad"], dtype=float),
        # J1/J2 intentionally start at their official lower limits. The much
        # tighter rest-relative commissioning envelope supplies the guard.
        joint_limit_margin_rad=0.0,
        gripper_limits_m=(0.0, 0.044),
        max_gripper_delta_m=0.044,
    )


def return_simulation_to_rest(
    *,
    model: WidowXAIModel,
    sim_data: mujoco.MjData,
    viewer,
    limiter,
    gate: CommandGate,
    q_command: np.ndarray,
    rest_q: np.ndarray,
    gripper_q: float,
    loop_rate_hz: float,
    timeout_s: float,
) -> np.ndarray:
    if np.max(np.abs(q_command - rest_q)) <= 1e-8:
        model.set_viewer_qpos(sim_data, rest_q, gripper_q)
        if viewer is not None:
            viewer.sync()
        return rest_q.copy()

    print("returning MuJoCo command to all-zero rest")
    limiter.reset(q_command)
    period = 1.0 / loop_rate_hz
    deadline = time.perf_counter() + timeout_s
    next_tick = time.perf_counter()
    while time.perf_counter() < deadline:
        result = limiter.step(rest_q, period)
        candidate = result.command
        if model.in_self_collision(candidate, gripper_q):
            raise CadSafetyError("self-collision predicted while returning simulation to rest")
        gate.validate(candidate, gripper_q)
        q_command = candidate
        model.set_viewer_qpos(sim_data, q_command, gripper_q)
        if viewer is not None:
            viewer.sync()
        if np.max(np.abs(q_command - rest_q)) <= 1e-6:
            model.set_viewer_qpos(sim_data, rest_q, gripper_q)
            return rest_q.copy()
        next_tick += period
        sleep_for = next_tick - time.perf_counter()
        if sleep_for > 0.0:
            time.sleep(sleep_for)
        else:
            next_tick = time.perf_counter()
    raise CadSafetyError("simulation did not return to rest before the configured timeout")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Preview validated M3T leader joints from all-zero rest in MuJoCo"
    )
    parser.add_argument("--config", default="configs/cad_rest_commissioning.yaml")
    parser.add_argument("--udp-host")
    parser.add_argument("--udp-port", type=int)
    parser.add_argument("--duration", type=float, default=0.0, help="0 runs until q/Ctrl+C")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--label", default="cad-rest-mujoco")
    parser.add_argument("--no-telemetry", action="store_true")
    parser.add_argument(
        "--simulation-auto-deadman",
        action="store_true",
        help=(
            "simulation smoke tests only: engage once after the first fresh packet; "
            "it remains fail-closed after any stale/discontinuity"
        ),
    )
    args = parser.parse_args()

    config = load_config(args.config)
    if config.get("hardware", {}).get("enabled", False):
        raise SystemExit("CAD MuJoCo launcher refuses configurations with hardware.enabled=true")
    if config.get("hardware", {}).get("live_output_implemented", False):
        raise SystemExit("CAD MuJoCo launcher cannot enable physical output")
    if not np.isfinite(args.duration) or args.duration < 0.0:
        raise SystemExit("--duration must be finite and nonnegative")

    model = WidowXAIModel(config["model"]["xml_path"])
    rest_q = np.asarray(config["model"]["rest_q_rad"], dtype=float).reshape(6)
    if not np.all(np.isfinite(rest_q)):
        raise SystemExit("rest position must contain six finite values")
    if model.in_self_collision(rest_q, float(config["model"]["gripper_preview_m"])):
        raise SystemExit("official MuJoCo model reports a self-collision at rest")

    commissioning = config["commissioning"]
    command_limits = rest_command_limits(
        model.joint_limits,
        rest_q,
        np.asarray(commissioning["minimum_delta_rad"], dtype=float),
        np.asarray(commissioning["maximum_delta_rad"], dtype=float),
    )
    mapper = CadRestMapper(
        rest_q=rest_q,
        signs=np.asarray(config["cad"]["signs"], dtype=float),
        scales=np.asarray(config["cad"]["scales"], dtype=float),
        source_deadband_rad=np.asarray(config["cad"]["source_deadband_rad"], dtype=float),
        command_limits=command_limits,
    )
    controller = CadDeadmanController(mapper)
    watchdog = CadFreshnessWatchdog(
        config["cad"]["stale_timeout_s"],
        config["cad"]["fresh_samples_to_recover"],
        np.asarray(config["cad"]["max_source_step_rad"], dtype=float),
    )

    udp_host = args.udp_host or config["cad"]["udp_host"]
    udp_port = config["cad"]["udp_port"] if args.udp_port is None else args.udp_port
    receiver = CadUdpReceiver(
        udp_host,
        udp_port,
        expected_names=config["cad"]["expected_names"],
        max_packet_age_s=config["cad"]["max_packet_age_s"],
        max_future_skew_s=config["cad"]["max_future_skew_s"],
    )

    gripper_q = float(config["model"]["gripper_preview_m"])
    gate = make_command_gate(rest_q, command_limits, model, config, gripper_q)
    limiter = limiter_from_config(config["control"]["joint_command_limits"], rest_q)
    if limiter is None:
        raise SystemExit("CAD commissioning requires the joint command limiter")

    sim_data = mujoco.MjData(model.model)
    model.set_viewer_qpos(sim_data, rest_q, gripper_q)
    viewer = None
    if not args.headless:
        from mujoco import viewer as mujoco_viewer

        viewer = mujoco_viewer.launch_passive(model.model, sim_data)
        viewer.cam.lookat[:] = [0.25, 0.0, 0.18]
        viewer.cam.distance = 1.0
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20

    console = SimulationDeadmanConsole()
    if not args.simulation_auto_deadman:
        console.start()
    print("PHYSICAL OUTPUT DISABLED: this process never imports or opens the Trossen driver")
    print(f"starting and ending at rest q={rest_q.tolist()}")
    print(f"listening for M3T joints on udp://{udp_host}:{udp_port}")
    if args.simulation_auto_deadman:
        print("simulation auto-deadman will engage once after three fresh packets")
    else:
        print("simulation commands: e + Enter=engage, r + Enter=release, q + Enter=quit")

    telemetry = None
    if not args.no_telemetry:
        telemetry = CadTelemetryLogger(
            args.label, config, config["telemetry"]["output_dir"]
        )

    q_command = rest_q.copy()
    q_des = rest_q.copy()
    last_sample = None
    loop_hz = float(config["control"]["loop_rate_hz"])
    period = 1.0 / loop_hz
    started = time.perf_counter()
    previous_tick = started
    next_tick = started
    last_log = started
    auto_deadman_engaged = False
    last_event = "startup_at_rest"

    try:
        receiver.start()
        while args.duration <= 0.0 or time.perf_counter() - started < args.duration:
            console.poll()
            if console.quit_requested:
                break
            if viewer is not None and not viewer.is_running():
                break
            sample, _generation = receiver.mailbox.take_latest()
            now_ns = time.perf_counter_ns()
            if sample is not None:
                last_sample = sample
                freshness = watchdog.observe(sample, now_ns)
            else:
                freshness = watchdog.poll(now_ns)

            if args.simulation_auto_deadman:
                if freshness.fresh and not auto_deadman_engaged:
                    console.set_pressed(True)
                    auto_deadman_engaged = True
                deadman_pressed = console.pressed
            else:
                deadman_pressed = console.pressed

            try:
                target = controller.update(
                    deadman_pressed=deadman_pressed,
                    stream_fresh=freshness.fresh,
                    sample=last_sample,
                    robot_q=q_command,
                )
                last_event = ""
            except CadSafetyError as exc:
                target = None
                last_event = f"mapping_fault:{exc}"
                print(f"CAD SAFETY HOLD: {exc}; release and re-engage the simulation deadman")

            now = time.perf_counter()
            dt = max(1e-6, now - previous_tick)
            previous_tick = now
            limiter_flags: list[str] = []
            if target is None:
                q_des = q_command.copy()
                limiter.reset(q_command)
                candidate = q_command.copy()
            else:
                q_des = target
                result = limiter.step(q_des, dt)
                candidate = result.command
                limiter_flags = result.flags("joint")

            try:
                if model.in_self_collision(candidate, gripper_q):
                    raise CadSafetyError("official model predicts a self-collision")
                gate.validate(candidate, gripper_q)
                q_command = candidate
            except (CadSafetyError, HardwareSafetyError) as exc:
                controller.fault(deadman_pressed)
                limiter.reset(q_command)
                q_des = q_command.copy()
                last_event = f"command_fault:{exc}"
                print(f"CAD SAFETY HOLD: {exc}; release and re-engage the simulation deadman")

            model.set_viewer_qpos(sim_data, q_command, gripper_q)
            if viewer is not None:
                viewer.sync()

            if telemetry is not None:
                source_age_ms = (
                    ""
                    if last_sample is None
                    else (time.time_ns() - last_sample.source_time_ns) / 1e6
                )
                arrival_age_ms = (
                    ""
                    if last_sample is None
                    else (now_ns - last_sample.arrival_monotonic_ns) / 1e6
                )
                telemetry.log(
                    pc_epoch_ns=time.time_ns(),
                    pc_monotonic_ns=time.perf_counter_ns(),
                    cad_sequence="" if last_sample is None else last_sample.sequence,
                    cad_source_time_ns="" if last_sample is None else last_sample.source_time_ns,
                    cad_source_age_ms=source_age_ms,
                    cad_arrival_age_ms=arrival_age_ms,
                    cad_fresh=freshness.fresh,
                    recovery_streak=freshness.recovery_streak,
                    discontinuity_generation=freshness.discontinuity_generation,
                    last_discontinuity=freshness.last_discontinuity,
                    deadman_pressed=deadman_pressed,
                    deadman_engaged=controller.engaged,
                    deadman_needs_release=controller.needs_release,
                    reanchor_generation=mapper.reanchor_generation,
                    q_source="" if last_sample is None else last_sample.q,
                    q_des=q_des,
                    q_cmd=q_command,
                    q_feedback=sim_data.qpos[model.qpos_indices].copy(),
                    limiter_flags="|".join(limiter_flags),
                    event=last_event,
                )

            if now - last_log >= 1.0:
                print(
                    "CAD_SIM "
                    f"recv/valid/invalid={receiver.received_packets}/"
                    f"{receiver.valid_packets}/{receiver.invalid_packets} "
                    f"fresh={int(freshness.fresh)} age_ms={freshness.age_s * 1000:.1f} "
                    f"deadman={int(deadman_pressed)} engaged={int(controller.engaged)} "
                    f"needs_release={int(controller.needs_release)} "
                    f"discontinuity={freshness.last_discontinuity or 'none'} "
                    f"q_cmd={np.array2string(q_command, precision=4, suppress_small=True)}"
                )
                last_log = now

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0.0:
                time.sleep(sleep_for)
            else:
                # Never generate catch-up command bursts after an overrun.
                next_tick = time.perf_counter()
    except KeyboardInterrupt:
        print("operator stopped CAD MuJoCo commissioning")
    finally:
        console.set_pressed(False)
        controller.update(
            deadman_pressed=False,
            stream_fresh=False,
            sample=last_sample,
            robot_q=q_command,
        )
        receiver.stop()
        try:
            q_command = return_simulation_to_rest(
                model=model,
                sim_data=sim_data,
                viewer=viewer,
                limiter=limiter,
                gate=gate,
                q_command=q_command,
                rest_q=rest_q,
                gripper_q=gripper_q,
                loop_rate_hz=loop_hz,
                timeout_s=float(commissioning["return_to_rest_timeout_s"]),
            )
            print(f"rest reached: q={np.round(q_command, 8).tolist()}")
        finally:
            if telemetry is not None:
                telemetry.close()
                print(f"telemetry: {telemetry.run_dir}")
            if viewer is not None:
                close_passive_viewer(viewer)


if __name__ == "__main__":
    main()
