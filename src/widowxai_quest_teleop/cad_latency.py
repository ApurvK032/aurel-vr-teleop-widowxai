from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class Distribution:
    count: int
    p50_ms: float
    p95_ms: float
    p99_ms: float
    maximum_ms: float


@dataclass(frozen=True)
class LagEstimate:
    lag_ms: float
    correlation: float
    samples: int


def _float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if np.isfinite(parsed) else None


def _bool(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _vector(value: object, joint: int) -> float | None:
    if value in (None, ""):
        return None
    try:
        parsed = json.loads(str(value))
        result = float(parsed[joint])
    except (json.JSONDecodeError, TypeError, ValueError, IndexError):
        return None
    return result if np.isfinite(result) else None


def _distribution(values: Iterable[float]) -> Distribution | None:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return None
    return Distribution(
        count=int(array.size),
        p50_ms=float(np.percentile(array, 50)),
        p95_ms=float(np.percentile(array, 95)),
        p99_ms=float(np.percentile(array, 99)),
        maximum_ms=float(np.max(array)),
    )


def estimate_lag(
    source_time_s: Sequence[float],
    source_value: Sequence[float],
    response_time_s: Sequence[float],
    response_value: Sequence[float],
    *,
    sample_period_ms: float = 5.0,
    maximum_lag_ms: float = 750.0,
    velocity: bool = False,
) -> LagEstimate | None:
    """Estimate the positive response lag using normalized cross-correlation.

    This is a phase/following estimate, not an onset detector.  Positive lag
    means the response follows the source.  Absolute correlation is optimized
    because a commissioned joint mapping may intentionally invert its sign.
    """

    source_t = np.asarray(source_time_s, dtype=float)
    source_v = np.asarray(source_value, dtype=float)
    response_t = np.asarray(response_time_s, dtype=float)
    response_v = np.asarray(response_value, dtype=float)
    if source_t.size < 4 or response_t.size < 4:
        return None
    source_ok = np.isfinite(source_t) & np.isfinite(source_v)
    response_ok = np.isfinite(response_t) & np.isfinite(response_v)
    source_t, source_v = source_t[source_ok], source_v[source_ok]
    response_t, response_v = response_t[response_ok], response_v[response_ok]
    if source_t.size < 4 or response_t.size < 4:
        return None

    source_order = np.argsort(source_t)
    response_order = np.argsort(response_t)
    source_t, source_v = source_t[source_order], source_v[source_order]
    response_t, response_v = response_t[response_order], response_v[response_order]
    source_t, source_unique = np.unique(source_t, return_index=True)
    response_t, response_unique = np.unique(response_t, return_index=True)
    source_v = source_v[source_unique]
    response_v = response_v[response_unique]

    dt_s = float(sample_period_ms) / 1000.0
    if not np.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("sample period must be finite and positive")
    start = max(float(source_t[0]), float(response_t[0]))
    stop = min(float(source_t[-1]), float(response_t[-1]))
    if stop - start < max(0.25, 10.0 * dt_s):
        return None
    grid = np.arange(start, stop, dt_s)
    if grid.size < 50:
        return None
    source_grid = np.interp(grid, source_t, source_v)
    response_grid = np.interp(grid, response_t, response_v)

    if velocity:
        source_grid = np.gradient(source_grid, dt_s)
        response_grid = np.gradient(response_grid, dt_s)
        window = max(1, int(round(0.025 / dt_s)))
        if window > 1:
            kernel = np.ones(window, dtype=float) / window
            source_grid = np.convolve(source_grid, kernel, mode="same")
            response_grid = np.convolve(response_grid, kernel, mode="same")

    maximum_steps = min(
        int(round(float(maximum_lag_ms) / float(sample_period_ms))),
        grid.size // 3,
    )
    best: tuple[float, int, int] | None = None
    for step in range(maximum_steps + 1):
        if step:
            left, right = source_grid[:-step], response_grid[step:]
        else:
            left, right = source_grid, response_grid
        left = left - np.mean(left)
        right = right - np.mean(right)
        denominator = float(np.linalg.norm(left) * np.linalg.norm(right))
        if denominator <= 1.0e-12:
            continue
        correlation = float(np.dot(left, right) / denominator)
        score = abs(correlation)
        if best is None or score > best[0]:
            best = (score, step, len(left))
            best_correlation = correlation
    if best is None:
        return None
    return LagEstimate(
        lag_ms=float(best[1] * sample_period_ms),
        correlation=float(best_correlation),
        samples=int(best[2]),
    )


def _signal(
    rows: Sequence[dict[str, str]],
    column: str,
    joint: int,
    *,
    time_column: str,
    fresh_feedback_only: bool = False,
    source_arrival_fallback: bool = False,
) -> tuple[list[float], list[float]]:
    times: list[float] = []
    values: list[float] = []
    seen: set[float] = set()
    for row in rows:
        if fresh_feedback_only and not _bool(row.get("feedback_sample_fresh")):
            continue
        value = _vector(row.get(column), joint)
        timestamp_ns = _float(row.get(time_column))
        if timestamp_ns is None and source_arrival_fallback:
            pc_ns = _float(row.get("pc_monotonic_ns"))
            arrival_age_ms = _float(row.get("cad_arrival_age_ms"))
            if pc_ns is not None and arrival_age_ms is not None:
                timestamp_ns = pc_ns - arrival_age_ms * 1.0e6
        if value is None or timestamp_ns is None or timestamp_ns <= 0.0:
            continue
        timestamp_s = timestamp_ns / 1.0e9
        if timestamp_s in seen:
            continue
        seen.add(timestamp_s)
        times.append(timestamp_s)
        values.append(value)
    return times, values


def analyze_cad_telemetry(
    telemetry_path: str | Path,
    *,
    joint: int = 0,
    target_ms: float = 35.0,
) -> dict[str, object]:
    path = Path(telemetry_path).expanduser().resolve()
    if path.is_dir():
        path = path / "telemetry.csv"
    if not path.is_file():
        raise FileNotFoundError(f"CAD telemetry not found: {path}")
    if not 0 <= int(joint) < 5:
        raise ValueError("joint must be in the range [0, 4]")
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"CAD telemetry has no rows: {path}")
    engaged = [row for row in rows if _bool(row.get("deadman_engaged"))]
    analysis_rows = engaged or rows

    unique_packets: list[dict[str, str]] = []
    seen_sequences: set[str] = set()
    for row in rows:
        sequence = row.get("cad_sequence", "")
        if not sequence or sequence in seen_sequences:
            continue
        seen_sequences.add(sequence)
        unique_packets.append(row)

    capture_to_publish = _distribution(
        value
        for row in unique_packets
        if (value := _float(row.get("cad_capture_to_publish_ms"))) is not None
    )
    publish_to_arrival_values: list[float] = []
    for row in unique_packets:
        value = _float(row.get("cad_publish_to_arrival_ms"))
        if value is None:
            source_age = _float(row.get("cad_source_age_ms"))
            arrival_age = _float(row.get("cad_arrival_age_ms"))
            if source_age is not None and arrival_age is not None:
                value = source_age - arrival_age
        if value is not None:
            publish_to_arrival_values.append(value)
    publish_to_arrival = _distribution(publish_to_arrival_values)
    frame_to_arrival = _distribution(
        value
        for row in unique_packets
        if (value := _float(row.get("cad_frame_to_arrival_ms"))) is not None
    )
    control_source_age = _distribution(
        value
        for row in analysis_rows
        if (value := _float(row.get("cad_source_age_ms"))) is not None
    )
    send_duration = _distribution(
        value
        for row in analysis_rows
        if (value := _float(row.get("command_send_duration_ms"))) is not None
    )

    source = _signal(
        analysis_rows,
        "q_source",
        joint,
        time_column="cad_arrival_monotonic_ns",
        source_arrival_fallback=True,
    )
    filtered = _signal(
        analysis_rows,
        "q_source_filtered",
        joint,
        time_column="cad_arrival_monotonic_ns",
        source_arrival_fallback=True,
    )
    desired = _signal(
        analysis_rows,
        "q_des",
        joint,
        time_column="command_send_monotonic_ns",
    )
    command = _signal(
        analysis_rows,
        "q_cmd",
        joint,
        time_column="command_send_monotonic_ns",
    )
    feedback = _signal(
        analysis_rows,
        "q_feedback",
        joint,
        time_column="feedback_read_monotonic_ns",
        fresh_feedback_only=True,
    )

    def lag_pair(left: tuple[list[float], list[float]], right: tuple[list[float], list[float]]) -> dict[str, object] | None:
        position = estimate_lag(*left, *right, velocity=False)
        velocity = estimate_lag(*left, *right, velocity=True)
        if position is None and velocity is None:
            return None
        return {
            "position": None if position is None else asdict(position),
            "velocity": None if velocity is None else asdict(velocity),
        }

    lag_stages = {
        "source_to_filtered": lag_pair(source, filtered),
        "source_to_desired": lag_pair(source, desired),
        "desired_to_command": lag_pair(desired, command),
        "command_to_feedback": lag_pair(command, feedback),
        "source_arrival_to_feedback": lag_pair(source, feedback),
    }
    source_feedback = lag_stages["source_arrival_to_feedback"]
    velocity_lag_ms: float | None = None
    if isinstance(source_feedback, dict) and isinstance(
        source_feedback.get("velocity"), dict
    ):
        velocity_lag_ms = float(source_feedback["velocity"]["lag_ms"])
    capture_to_feedback_estimate_ms: float | None = None
    if frame_to_arrival is not None and velocity_lag_ms is not None:
        capture_to_feedback_estimate_ms = (
            frame_to_arrival.p50_ms + velocity_lag_ms
        )

    return {
        "telemetry": str(path),
        "joint": int(joint),
        "target_ms": float(target_ms),
        "rows": len(rows),
        "engaged_rows": len(engaged),
        "unique_packets": len(unique_packets),
        "direct_timing_ms": {
            "capture_to_publish": None
            if capture_to_publish is None
            else asdict(capture_to_publish),
            "publish_to_arrival": None
            if publish_to_arrival is None
            else asdict(publish_to_arrival),
            "frame_to_arrival": None
            if frame_to_arrival is None
            else asdict(frame_to_arrival),
            "source_age_at_control": None
            if control_source_age is None
            else asdict(control_source_age),
            "driver_send_duration": None
            if send_duration is None
            else asdict(send_duration),
        },
        "phase_lag_ms": lag_stages,
        "estimated_capture_to_encoder_phase_ms": capture_to_feedback_estimate_ms,
        "meets_internal_phase_target": (
            None
            if capture_to_feedback_estimate_ms is None
            else capture_to_feedback_estimate_ms < float(target_ms)
        ),
        "physical_target_verified": False,
        "limitations": [
            "Cross-correlation reports following/phase lag, not motion-onset latency.",
            "Encoder feedback is not an optical measurement of follower-link motion.",
            "Physical leader motion may begin up to one camera frame before frame_time_ns.",
        ],
    }
