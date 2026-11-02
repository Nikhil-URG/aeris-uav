#!/usr/bin/env python3
"""AERIS mission log evaluator (Week 7 placeholder).

This entry point will consume mission.jsonl, telemetry.csv and rosbag2
artifacts produced by the mission manager. For now it validates the
artifact directory and reports what is present, so the interface is
stable before the metrics are implemented.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path("results/logs"),
        help="Directory containing one subdirectory per mission run",
    )
    return parser.parse_args()


def summarize(run_dir: Path) -> None:
    artifacts = {
        "mission events": run_dir / "mission.jsonl",
        "telemetry": run_dir / "telemetry.csv",
        "rosbag2": run_dir / "rosbag2",
    }
    print(f"run: {run_dir.name}")
    for label, path in artifacts.items():
        state = "present" if path.exists() else "missing"
        print(f"  {label:16s} {state:8s} {path}")
    print()


def main() -> int:
    args = parse_args()
    if not args.log_dir.is_dir():
        print(f"error: log directory not found: {args.log_dir}", file=sys.stderr)
        return 2

    runs = sorted(p for p in args.log_dir.iterdir() if p.is_dir())
    if not runs:
        print(f"No mission runs found in {args.log_dir}.")
        print("This evaluator is a Week 7 placeholder; metrics will be added then.")
        return 0

    for run in runs:
        summarize(run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
