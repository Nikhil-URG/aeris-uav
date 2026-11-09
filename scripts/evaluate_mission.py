#!/usr/bin/env python3
"""Evaluate AERIS mission JSONL logs.

The mission manager writes one JSONL file per run. This tool turns those
events into comparable metrics and optional plots/reports.

Examples:
    python3 scripts/evaluate_mission.py docs/experiments/week3_mission.jsonl
    python3 scripts/evaluate_mission.py docs/experiments/week*_mission.jsonl \
        --json results/reports/benchmark.json \
        --csv results/reports/benchmark.csv \
        --plot-dir docs/experiments/plots
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any


def parse_timestamp(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def load_events(path: Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError as exc:
                print(f"warning: skipping invalid JSON in {path}: {exc}",
                      file=sys.stderr)
    return events


def first_event(events: list[dict[str, Any]], name: str) -> dict[str, Any] | None:
    return next((event for event in events if event.get("event") == name), None)


def events_of(events: list[dict[str, Any]], name: str) -> list[dict[str, Any]]:
    return [event for event in events if event.get("event") == name]


def seconds_between(start: dict[str, Any] | None,
                    end: dict[str, Any] | None) -> float | None:
    if start is None or end is None:
        return None
    delta = parse_timestamp(end["timestamp"]) - parse_timestamp(start["timestamp"])
    return round(delta.total_seconds(), 3)


def derive_scenario(mission_name: str,
                    detections: int,
                    replans: int,
                    safety_faults: list[str]) -> str:
    if mission_name == "safety_test":
        if safety_faults:
            return "safety_" + safety_faults[0]
        return "safety_nominal"
    if replans:
        return "obstacle"
    if detections:
        return "perception"
    return "nominal"


def evaluate_run(path: Path) -> dict[str, Any]:
    events = load_events(path)
    if not events:
        return {
            "run": path.stem,
            "evidence": str(path),
            "mission": "",
            "scenario": "empty",
            "outcome": "empty",
            "success": False,
        }

    loaded = first_event(events, "mission_loaded")
    start = first_event(events, "mission_start") or events[0]
    takeoff = first_event(events, "takeoff_confirmed")
    complete = first_event(events, "mission_complete")
    safety = first_event(events, "safety_abort")
    landing = first_event(events, "landing")

    mission_name = ""
    for event in events:
        if event.get("mission"):
            mission_name = str(event["mission"])
            break

    waypoints_total = 0
    if loaded is not None:
        waypoints_total = int(loaded.get("waypoints", 0))

    transit_events = {event.get("id"): event
                      for event in events_of(events, "waypoint_transit")}
    reached_events = {event.get("id"): event
                      for event in events_of(events, "waypoint_reached")}
    inspect_events = {event.get("id"): event
                      for event in events_of(events, "inspection_complete")}

    waypoints: list[dict[str, Any]] = []
    for waypoint_id, transit in transit_events.items():
        reached = reached_events.get(waypoint_id)
        inspected = inspect_events.get(waypoint_id)
        waypoints.append(
            {
                "id": waypoint_id,
                "transit_s": seconds_between(transit, reached),
                "hold_s": seconds_between(reached, inspected),
                "reached": reached is not None,
                "inspected": inspected is not None,
            }
        )

    detections = events_of(events, "target_detected")
    detection_confidences = [
        float(item["confidence"]) for item in detections
        if "confidence" in item
    ]
    detection_ranges = [
        float(item["range_m"]) for item in detections
        if "range_m" in item
    ]
    obstacle_events = events_of(events, "obstacle_detected")
    replans = events_of(events, "replan")
    safety_faults: list[str] = []
    safety_action = ""
    if safety is not None:
        safety_faults = [str(fault) for fault in safety.get("faults", [])]
        safety_action = str(safety.get("action", ""))

    rtl_transit = first_event(events, "rtl_transit")
    rtl_reached = first_event(events, "rtl_reached")

    if complete is not None:
        outcome = "complete"
        success = True
    elif safety is not None:
        outcome = "safety_abort"
        success = False
    else:
        outcome = "aborted"
        success = False

    duration_s = seconds_between(start, complete or safety or events[-1])
    takeoff_s = seconds_between(start, takeoff)
    landing_s = seconds_between(landing, complete) if complete else None

    reached_waypoints = sum(1 for item in waypoints if item["reached"])
    completed_waypoints = sum(1 for item in waypoints if item["inspected"])
    transit_values = [item["transit_s"] for item in waypoints
                      if item["transit_s"] is not None]
    hold_values = [item["hold_s"] for item in waypoints
                   if item["hold_s"] is not None]

    first_detour = replans[0] if replans else None
    scenario = derive_scenario(
        mission_name, len(detections), len(replans), safety_faults
    )

    return {
        "run": path.stem,
        "evidence": str(path),
        "mission": mission_name,
        "scenario": scenario,
        "outcome": outcome,
        "success": success,
        "duration_s": duration_s,
        "takeoff_s": takeoff_s,
        "landing_s": landing_s,
        "rtl_transit_s": seconds_between(rtl_transit, rtl_reached),
        "waypoints_total": waypoints_total,
        "waypoints_reached": reached_waypoints,
        "waypoints_completed": completed_waypoints,
        "waypoints": waypoints,
        "mean_transit_s": round(statistics.mean(transit_values), 3)
        if transit_values else None,
        "mean_hold_s": round(statistics.mean(hold_values), 3)
        if hold_values else None,
        "detections": len(detections),
        "detection_rate": round(len(detections) / reached_waypoints, 3)
        if reached_waypoints else 0.0,
        "mean_confidence": round(statistics.mean(detection_confidences), 3)
        if detection_confidences else None,
        "min_range_m": round(min(detection_ranges), 3)
        if detection_ranges else None,
        "max_range_m": round(max(detection_ranges), 3)
        if detection_ranges else None,
        "obstacle_detections": len(obstacle_events),
        "replans": len(replans),
        "detour": {
            "x": round(float(first_detour.get("detour_x")), 3),
            "y": round(float(first_detour.get("detour_y")), 3),
            "z": round(float(first_detour.get("detour_z")), 3),
        } if first_detour else None,
        "safety_aborts": 1 if safety is not None else 0,
        "safety_faults": safety_faults,
        "safety_action": safety_action,
        "state_transitions": len(events_of(events, "state_transition")),
    }


def aggregate(runs: list[dict[str, Any]]) -> dict[str, Any]:
    complete_runs = [run for run in runs if run["outcome"] == "complete"]
    safety_runs = [run for run in runs if run["outcome"] == "safety_abort"]
    durations = [run["duration_s"] for run in complete_runs
                 if run["duration_s"] is not None]
    confidences = [run["mean_confidence"] for run in runs
                   if run["mean_confidence"] is not None]
    detection_rates = [run["detection_rate"] for run in runs
                       if run["detections"] > 0]

    return {
        "runs": len(runs),
        "completed": len(complete_runs),
        "safety_aborts": len(safety_runs),
        "aborted_other": len(runs) - len(complete_runs) - len(safety_runs),
        "planned_completion_success_rate": round(
            len(complete_runs) /
            max(1, len([run for run in runs
                        if run["outcome"] != "safety_abort"])), 3),
        "mean_duration_s": round(statistics.mean(durations), 3)
        if durations else None,
        "min_duration_s": round(min(durations), 3) if durations else None,
        "max_duration_s": round(max(durations), 3) if durations else None,
        "total_detections": sum(run["detections"] for run in runs),
        "mean_confidence": round(statistics.mean(confidences), 3)
        if confidences else None,
        "mean_detection_rate": round(statistics.mean(detection_rates), 3)
        if detection_rates else None,
        "total_obstacle_detections": sum(run["obstacle_detections"]
                                         for run in runs),
        "total_replans": sum(run["replans"] for run in runs),
        "safety_faults": {
            fault: sum(1 for run in safety_runs
                       if fault in run["safety_faults"])
            for fault in sorted({
                fault
                for run in safety_runs
                for fault in run["safety_faults"]
            })
        },
    }


def write_json(path: Path, runs: list[dict[str, Any]],
               summary: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "aggregate": summary,
        "runs": runs,
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def write_csv(path: Path, runs: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "run", "scenario", "mission", "outcome", "duration_s", "takeoff_s",
        "waypoints_total", "waypoints_reached", "waypoints_completed",
        "mean_transit_s", "mean_hold_s", "rtl_transit_s", "landing_s",
        "detections",
        "detection_rate", "mean_confidence", "min_range_m", "max_range_m",
        "obstacle_detections", "replans", "safety_faults", "safety_action",
        "evidence",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for run in runs:
            row = {field: run.get(field) for field in fields}
            row["safety_faults"] = ";".join(run.get("safety_faults", []))
            writer.writerow(row)


def make_plots(path: Path, runs: list[dict[str, Any]]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        print(f"warning: matplotlib unavailable, skipping plots: {exc}",
              file=sys.stderr)
        return

    path.mkdir(parents=True, exist_ok=True)
    labels = [run["run"].replace("_mission", "").replace("week", "w")
              for run in runs]
    colors = {
        "complete": "#2e7d32",
        "safety_abort": "#c62828",
        "aborted": "#ef6c00",
        "empty": "#757575",
    }

    fig, axis = plt.subplots(figsize=(11, 5))
    durations = [run.get("duration_s") or 0.0 for run in runs]
    axis.bar(range(len(runs)), durations,
             color=[colors.get(run["outcome"], "#455a64") for run in runs])
    axis.set_xticks(range(len(runs)))
    axis.set_xticklabels(labels, rotation=35, ha="right", fontsize=8)
    axis.set_ylabel("Duration (s)")
    axis.set_title("AERIS mission duration by recorded run")
    axis.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(path / "mission_durations.png", dpi=160)
    plt.close(fig)

    complete_runs = [run for run in runs if run["outcome"] == "complete"]
    if complete_runs:
        fig, axis = plt.subplots(figsize=(9, 5))
        takeoff = [run.get("takeoff_s") or 0.0 for run in complete_runs]
        transit = [sum(item["transit_s"] or 0.0 for item in run["waypoints"])
                   for run in complete_runs]
        hold = [sum(item["hold_s"] or 0.0 for item in run["waypoints"])
                for run in complete_runs]
        rtl = [run.get("rtl_transit_s") or 0.0 for run in complete_runs]
        landing = [run.get("landing_s") or 0.0 for run in complete_runs]
        overhead = [
            max(
                0.0,
                (run.get("duration_s") or 0.0)
                - takeoff[i] - transit[i] - hold[i] - rtl[i] - landing[i],
            )
            for i, run in enumerate(complete_runs)
        ]
        x = range(len(complete_runs))
        bottom = [0.0] * len(complete_runs)
        for values, label in (
            (takeoff, "takeoff"),
            (transit, "transit"),
            (hold, "inspection hold"),
            (rtl, "RTL"),
            (landing, "landing"),
            (overhead, "other"),
        ):
            axis.bar(x, values, bottom=bottom, label=label)
            bottom = [bottom[i] + values[i] for i in range(len(complete_runs))]
        axis.set_xticks(list(x))
        axis.set_xticklabels(
            [run["run"].replace("_mission", "") for run in complete_runs],
            rotation=25, ha="right", fontsize=8,
        )
        axis.set_ylabel("Seconds")
        axis.set_title("Completed mission phase breakdown")
        axis.legend(fontsize=8)
        axis.grid(axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(path / "phase_breakdown.png", dpi=160)
        plt.close(fig)


def collect_paths(inputs: list[Path]) -> list[Path]:
    files: list[Path] = []
    for path in inputs:
        if path.is_dir():
            files.extend(sorted(path.rglob("mission.jsonl")))
        elif path.is_file():
            files.append(path)
        else:
            print(f"warning: path not found: {path}", file=sys.stderr)
    return sorted(set(files))


def default_paths() -> list[Path]:
    root = Path("docs/experiments")
    return sorted(root.glob("week*_mission.jsonl"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate AERIS mission JSONL logs."
    )
    parser.add_argument(
        "paths", nargs="*", type=Path,
        help="JSONL files or directories containing mission.jsonl",
    )
    parser.add_argument("--json", type=Path,
                        help="Write the full metrics report as JSON")
    parser.add_argument("--csv", type=Path,
                        help="Write one row per run as CSV")
    parser.add_argument("--plot-dir", type=Path,
                        help="Directory for generated plots")
    parser.add_argument("--no-plots", action="store_true",
                        help="Skip plot generation")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    inputs = args.paths or default_paths()
    files = collect_paths(inputs)
    if not files:
        print("error: no mission JSONL files found", file=sys.stderr)
        return 2

    runs = [evaluate_run(path) for path in files]
    summary = aggregate(runs)

    header = (
        f"{'run':28s} {'scenario':26s} {'outcome':12s} "
        f"{'dur':>7s} {'wp':>5s} {'det':>4s} {'obs':>4s} {'replan':>6s}"
    )
    print(header)
    print("-" * len(header))
    for run in runs:
        duration = run.get("duration_s")
        duration_text = f"{duration:.1f}" if duration is not None else "-"
        print(
            f"{run['run'][:28]:28s} {run['scenario'][:26]:26s} "
            f"{run['outcome']:12s} {duration_text:>7s} "
            f"{run.get('waypoints_completed', 0)}/{run.get('waypoints_total', 0):<3d} "
            f"{run.get('detections', 0):>4d} "
            f"{run.get('obstacle_detections', 0):>4d} "
            f"{run.get('replans', 0):>6d}"
        )

    print()
    print(
        f"runs={summary['runs']} completed={summary['completed']} "
        f"safety_aborts={summary['safety_aborts']} "
        f"planned_completion_success={summary['planned_completion_success_rate']:.1%}"
    )
    if summary["mean_duration_s"] is not None:
        print(
            f"completed duration mean={summary['mean_duration_s']:.1f}s "
            f"range={summary['min_duration_s']:.1f}-"
            f"{summary['max_duration_s']:.1f}s"
        )
    if summary["total_detections"]:
        print(
            f"detections={summary['total_detections']} "
            f"mean_confidence={summary['mean_confidence']} "
            f"mean_detection_rate={summary['mean_detection_rate']:.1%}"
        )
    if summary["total_obstacle_detections"]:
        print(
            f"obstacle_detections={summary['total_obstacle_detections']} "
            f"replans={summary['total_replans']}"
        )
    if summary["safety_faults"]:
        print(f"safety_faults={summary['safety_faults']}")

    if args.json:
        write_json(args.json, runs, summary)
        print(f"wrote {args.json}")
    if args.csv:
        write_csv(args.csv, runs)
        print(f"wrote {args.csv}")
    if args.plot_dir and not args.no_plots:
        make_plots(args.plot_dir, runs)
        print(f"wrote plots to {args.plot_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
