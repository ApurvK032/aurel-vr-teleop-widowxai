from __future__ import annotations

import csv
import json
import re
import subprocess
import time
from pathlib import Path
from typing import Any

import numpy as np

from .config import PROJECT_ROOT, resolve_project_path


TELEMETRY_COLUMNS = [
    "pc_epoch_ns",
    "pc_monotonic_ns",
    "quest_sequence",
    "quest_capture_monotonic_ms",
    "quest_capture_epoch_ms",
    "quest_send_monotonic_ms",
    "pc_socket_arrival_monotonic_ns",
    "control_consume_monotonic_ns",
    "ik_start_monotonic_ns",
    "ik_end_monotonic_ns",
    "command_send_monotonic_ns",
    "reconnect_generation",
    "quest_grip",
    "quest_trigger",
    "stream_fresh",
    "clutch_engaged",
    "reanchor_generation",
    "raw_controller_position",
    "raw_controller_quaternion_wxyz",
    "mapped_target_position",
    "mapped_target_quaternion_wxyz",
    "wrist_target_position",
    "wrist_current_position",
    "q_des",
    "q_cmd",
    "q_feedback",
    "position_residual_m",
    "orientation_residual_rad",
    "position_manipulability",
    "rotation_manipulability",
    "position_damping",
    "rotation_damping",
    "ik_step_norm_rad",
    "minimum_joint_limit_margin_rad",
    "ik_status",
    "limiter_flags",
]


def _encode(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return json.dumps(value.tolist(), separators=(",", ":"))
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(value, separators=(",", ":"))
    return value


class TelemetryLogger:
    def __init__(self, label: str, config: dict[str, Any], output_dir: str | Path = "runs") -> None:
        safe_label = re.sub(r"[^A-Za-z0-9_.-]+", "-", label).strip("-") or "run"
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.run_dir = resolve_project_path(output_dir) / f"{stamp}_{safe_label}"
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.csv_path = self.run_dir / "telemetry.csv"
        self._handle = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=TELEMETRY_COLUMNS, extrasaction="ignore")
        self._writer.writeheader()
        self.rows = 0
        self.ik_failures = 0
        snapshot = dict(config)
        try:
            snapshot["git_commit"] = subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=PROJECT_ROOT, text=True, stderr=subprocess.DEVNULL
            ).strip()
        except (subprocess.CalledProcessError, FileNotFoundError):
            snapshot["git_commit"] = "uncommitted-initial-state"
        (self.run_dir / "config_snapshot.json").write_text(
            json.dumps(snapshot, indent=2, default=str), encoding="utf-8"
        )

    def log(self, **record: Any) -> None:
        row = {name: _encode(record.get(name, "")) for name in TELEMETRY_COLUMNS}
        self._writer.writerow(row)
        self.rows += 1
        if record.get("ik_status") not in (None, "", "ok", "orientation_parked"):
            self.ik_failures += 1

    def close(self) -> None:
        if self._handle.closed:
            return
        self._handle.flush()
        self._handle.close()
        summary = {
            "rows": self.rows,
            "ik_failures": self.ik_failures,
            "telemetry_csv": str(self.csv_path),
        }
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    def __enter__(self) -> "TelemetryLogger":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
