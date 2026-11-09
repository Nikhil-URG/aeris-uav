"""Tests for the AERIS mission-log evaluator."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "evaluate_mission", ROOT / "scripts" / "evaluate_mission.py"
)
assert SPEC is not None and SPEC.loader is not None
evaluate_mission = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluate_mission)


def write_events(path: Path, events: list[dict]) -> Path:
    path.write_text(
        "\n".join(json.dumps(event) for event in events) + "\n",
        encoding="utf-8",
    )
    return path


def test_complete_perception_run(tmp_path):
    path = write_events(
        tmp_path / "perception.jsonl",
        [
            {"timestamp": "2026-01-01T00:00:00+00:00", "event": "mission_loaded",
             "mission": "demo", "waypoints": 1},
            {"timestamp": "2026-01-01T00:00:01+00:00", "event": "mission_start",
             "mission": "demo"},
            {"timestamp": "2026-01-01T00:00:06+00:00", "event": "takeoff_confirmed",
             "mission": "demo"},
            {"timestamp": "2026-01-01T00:00:06+00:00", "event": "waypoint_transit",
             "mission": "demo", "id": "wp_01"},
            {"timestamp": "2026-01-01T00:00:09+00:00", "event": "waypoint_reached",
             "mission": "demo", "id": "wp_01"},
            {"timestamp": "2026-01-01T00:00:10+00:00", "event": "target_detected",
             "mission": "demo", "id": "wp_01", "confidence": 0.9,
             "range_m": 4.0},
            {"timestamp": "2026-01-01T00:00:12+00:00",
             "event": "inspection_complete", "mission": "demo", "id": "wp_01"},
            {"timestamp": "2026-01-01T00:00:12+00:00", "event": "rtl_transit",
             "mission": "demo"},
            {"timestamp": "2026-01-01T00:00:15+00:00", "event": "rtl_reached",
             "mission": "demo"},
            {"timestamp": "2026-01-01T00:00:15+00:00", "event": "landing",
             "mission": "demo"},
            {"timestamp": "2026-01-01T00:00:21+00:00", "event": "mission_complete",
             "mission": "demo"},
        ],
    )

    run = evaluate_mission.evaluate_run(path)

    assert run["outcome"] == "complete"
    assert run["scenario"] == "perception"
    assert run["duration_s"] == 20.0
    assert run["takeoff_s"] == 5.0
    assert run["waypoints_completed"] == 1
    assert run["mean_transit_s"] == 3.0
    assert run["mean_hold_s"] == 3.0
    assert run["rtl_transit_s"] == 3.0
    assert run["landing_s"] == 6.0
    assert run["detections"] == 1
    assert run["detection_rate"] == 1.0
    assert run["mean_confidence"] == 0.9


def test_safety_abort_run(tmp_path):
    path = write_events(
        tmp_path / "safety.jsonl",
        [
            {"timestamp": "2026-01-01T00:00:00+00:00", "event": "mission_loaded",
             "mission": "safety_test", "waypoints": 1},
            {"timestamp": "2026-01-01T00:00:01+00:00", "event": "mission_start",
             "mission": "safety_test"},
            {"timestamp": "2026-01-01T00:00:06+00:00", "event": "takeoff_confirmed",
             "mission": "safety_test"},
            {"timestamp": "2026-01-01T00:00:06+00:00", "event": "waypoint_transit",
             "mission": "safety_test", "id": "wp_safety"},
            {"timestamp": "2026-01-01T00:00:08+00:00", "event": "waypoint_reached",
             "mission": "safety_test", "id": "wp_safety"},
            {"timestamp": "2026-01-01T00:00:10+00:00", "event": "safety_abort",
             "mission": "safety_test", "faults": ["low_battery"],
             "action": "rtl"},
        ],
    )

    run = evaluate_mission.evaluate_run(path)

    assert run["outcome"] == "safety_abort"
    assert run["scenario"] == "safety_low_battery"
    assert run["waypoints_reached"] == 1
    assert run["waypoints_completed"] == 0
    assert run["safety_faults"] == ["low_battery"]
    assert run["safety_action"] == "rtl"
    assert run["success"] is False


def test_real_week4_evidence():
    run = evaluate_mission.evaluate_run(
        ROOT / "docs" / "experiments" / "week4_mission.jsonl"
    )

    assert run["outcome"] == "complete"
    assert run["waypoints_completed"] == 2
    assert run["detections"] == 2
    assert run["detection_rate"] == 1.0
    assert run["mean_confidence"] == 1.0


def test_aggregate_marks_fault_runs_as_planned():
    runs = [
        {
            "outcome": "complete", "duration_s": 30.0, "detections": 0,
            "mean_confidence": None, "detection_rate": 0.0,
            "obstacle_detections": 0, "replans": 0, "safety_faults": [],
        },
        {
            "outcome": "safety_abort", "duration_s": 12.0, "detections": 0,
            "mean_confidence": None, "detection_rate": 0.0,
            "obstacle_detections": 0, "replans": 0,
            "safety_faults": ["stale_pose"],
        },
    ]

    summary = evaluate_mission.aggregate(runs)

    assert summary["runs"] == 2
    assert summary["completed"] == 1
    assert summary["safety_aborts"] == 1
    assert summary["planned_completion_success_rate"] == 1.0
    assert summary["safety_faults"] == {"stale_pose": 1}
