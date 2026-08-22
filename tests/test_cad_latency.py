from __future__ import annotations

import csv
import json

import numpy as np

from widowxai_quest_teleop.cad_latency import (
    analyze_cad_telemetry,
    estimate_lag,
)
from widowxai_quest_teleop.telemetry import CAD_TELEMETRY_COLUMNS


def test_lag_estimator_recovers_a_known_following_delay() -> None:
    time_s = np.arange(0.0, 5.0, 0.005)
    source = (
        np.sin(2.0 * np.pi * 0.7 * time_s)
        + 0.35 * np.sin(2.0 * np.pi * 1.9 * time_s)
    )
    delay_s = 0.085
    response = (
        np.sin(2.0 * np.pi * 0.7 * (time_s - delay_s))
        + 0.35 * np.sin(2.0 * np.pi * 1.9 * (time_s - delay_s))
    )
    position = estimate_lag(time_s, source, time_s, response)
    velocity = estimate_lag(time_s, source, time_s, response, velocity=True)
    assert position is not None
    assert velocity is not None
    assert position.lag_ms == 85.0
    assert velocity.lag_ms == 85.0
    assert position.correlation > 0.99
    assert velocity.correlation > 0.98


def test_analyzer_reports_capture_timing_and_target_result(tmp_path) -> None:
    telemetry = tmp_path / "telemetry.csv"
    time_s = np.arange(0.0, 3.0, 0.01)
    fieldnames = list(CAD_TELEMETRY_COLUMNS)
    with telemetry.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for index, t_s in enumerate(time_s):
            arrival_ns = 2_000_000_000 + int(t_s * 1e9)
            publish_ns = 1_000_000_000 + int(t_s * 1e9)
            frame_ns = publish_ns - 8_000_000
            command_ns = arrival_ns + 1_000_000
            feedback_ns = command_ns + 20_000_000
            source = np.sin(2.0 * np.pi * 0.8 * t_s)
            feedback = source
            vector = lambda value: json.dumps([value, 0.0, 0.0, 0.0, 0.0, 0.0])
            row = {name: "" for name in fieldnames}
            row.update(
                pc_epoch_ns=publish_ns + 10_000_000,
                pc_monotonic_ns=command_ns,
                cad_sequence=index,
                cad_source_time_ns=publish_ns,
                cad_frame_time_ns=frame_ns,
                cad_frame_timestamp_domain="global_time",
                cad_frame_skew_ms=0.2,
                cad_capture_to_publish_ms=8.0,
                cad_publish_to_arrival_ms=1.0,
                cad_frame_to_arrival_ms=9.0,
                cad_arrival_monotonic_ns=arrival_ns,
                cad_arrival_epoch_ns=publish_ns + 1_000_000,
                cad_source_age_ms=10.0,
                cad_arrival_age_ms=9.0,
                deadman_engaged=True,
                command_send_monotonic_ns=command_ns,
                command_send_duration_ms=0.2,
                feedback_read_monotonic_ns=feedback_ns,
                feedback_sample_fresh=True,
                q_source=vector(source),
                q_source_filtered=vector(source),
                q_des=vector(source),
                q_cmd=vector(source),
                q_feedback=vector(feedback),
            )
            writer.writerow(row)

    report = analyze_cad_telemetry(telemetry, joint=0, target_ms=35.0)
    direct = report["direct_timing_ms"]
    assert direct["capture_to_publish"]["p50_ms"] == 8.0
    assert direct["frame_to_arrival"]["p50_ms"] == 9.0
    assert report["estimated_capture_to_encoder_phase_ms"] == 29.0
    assert report["meets_internal_phase_target"] is True
    assert report["physical_target_verified"] is False
