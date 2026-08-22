#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

from widowxai_quest_teleop.cad_latency import analyze_cad_telemetry


def _fmt_distribution(value: object) -> str:
    if not isinstance(value, dict):
        return "unavailable"
    return (
        f"p50={value['p50_ms']:.2f} p95={value['p95_ms']:.2f} "
        f"p99={value['p99_ms']:.2f} max={value['maximum_ms']:.2f} "
        f"n={value['count']}"
    )


def _fmt_lag(value: object) -> str:
    if not isinstance(value, dict):
        return "unavailable"
    position = value.get("position")
    velocity = value.get("velocity")
    parts: list[str] = []
    if isinstance(position, dict):
        parts.append(
            f"position={position['lag_ms']:.1f} ms (r={position['correlation']:.3f})"
        )
    if isinstance(velocity, dict):
        parts.append(
            f"velocity={velocity['lag_ms']:.1f} ms (r={velocity['correlation']:.3f})"
        )
    return ", ".join(parts) if parts else "unavailable"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze camera-CAD-follower timing from one telemetry run."
    )
    parser.add_argument("run", type=Path, help="run directory or telemetry.csv")
    parser.add_argument("--joint", type=int, default=0, choices=range(5))
    parser.add_argument("--target-ms", type=float, default=35.0)
    parser.add_argument("--json", action="store_true", help="print machine-readable JSON")
    args = parser.parse_args()
    report = analyze_cad_telemetry(
        args.run, joint=args.joint, target_ms=args.target_ms
    )
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0

    print(f"telemetry={report['telemetry']}")
    print(
        f"joint={report['joint']} rows={report['rows']} "
        f"engaged_rows={report['engaged_rows']} unique_packets={report['unique_packets']}"
    )
    direct = report["direct_timing_ms"]
    assert isinstance(direct, dict)
    for name, value in direct.items():
        print(f"direct {name}: {_fmt_distribution(value)} ms")
    stages = report["phase_lag_ms"]
    assert isinstance(stages, dict)
    for name, value in stages.items():
        print(f"phase {name}: {_fmt_lag(value)}")
    estimate = report["estimated_capture_to_encoder_phase_ms"]
    if estimate is None:
        print(
            "RESULT INCOMPLETE: this run predates capture timestamps; "
            "only post-tracker phase lag is measurable"
        )
    else:
        status = "PASS" if report["meets_internal_phase_target"] else "FAIL"
        print(
            f"capture-to-encoder phase estimate={estimate:.1f} ms "
            f"target<{report['target_ms']:.1f} ms INTERNAL PHASE {status}"
        )
        print("PHYSICAL RESULT UNVERIFIED: synchronized visual onset is not measured")
    print(
        "Note: use synchronized high-speed video for true physical "
        "leader-motion to follower-link-motion ground truth."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
