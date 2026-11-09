"""Mission model, YAML loader, and validation for AERIS."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import math

import yaml

MIN_ALTITUDE_M = 1.0
MAX_ALTITUDE_M = 10.0
MAX_RADIUS_M = 20.0
MIN_TOLERANCE_M = 0.1
MAX_TOLERANCE_M = 2.0
MAX_HOLD_S = 60.0
MIN_MISSION_TIMEOUT_S = 10.0
MAX_MISSION_TIMEOUT_S = 3600.0
ALLOWED_ACTIONS = {"inspect", "transit", "photo"}


class MissionValidationError(ValueError):
    """Raised when a mission file is structurally or semantically invalid."""


@dataclass(frozen=True)
class Waypoint:
    id: str
    x: float
    y: float
    z: float
    hold_s: float
    action: str
    target_id: str
    tolerance_m: float


@dataclass(frozen=True)
class Mission:
    name: str
    frame: str
    home_x: float
    home_y: float
    home_z: float
    cruise_altitude_m: float
    max_speed_mps: float
    waypoint_tolerance_m: float
    mission_timeout_s: float
    waypoints: tuple[Waypoint, ...]
    return_to_launch: bool
    land_on_complete: bool


def _mapping(value: Any, where: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise MissionValidationError(f"{where} must be a mapping")
    return value


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionValidationError(f"{where} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise MissionValidationError(f"{where} must be finite")
    return value


def _bool(value: Any, where: str, default: bool) -> bool:
    if value is None:
        return default
    if not isinstance(value, bool):
        raise MissionValidationError(f"{where} must be a boolean")
    return value


def _text(value: Any, where: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise MissionValidationError(f"{where} must be a non-empty string")
    return value.strip()


def _altitude(value: float, where: str) -> None:
    if value < MIN_ALTITUDE_M or value > MAX_ALTITUDE_M:
        raise MissionValidationError(
            f"{where} must be between {MIN_ALTITUDE_M} and {MAX_ALTITUDE_M} m"
        )


def parse_mission(data: Any) -> Mission:
    root = _mapping(data, "root")
    mission_raw = _mapping(root.get("mission"), "mission")

    name = _text(mission_raw.get("name"), "mission.name")
    frame = _text(mission_raw.get("frame", "local_ned"), "mission.frame")
    if frame != "local_ned":
        raise MissionValidationError("mission.frame must be 'local_ned'")

    home_raw = _mapping(mission_raw.get("home", {}), "mission.home")
    home_x = _number(home_raw.get("x", 0.0), "mission.home.x")
    home_y = _number(home_raw.get("y", 0.0), "mission.home.y")
    home_z = _number(home_raw.get("z", 0.0), "mission.home.z")

    defaults_raw = _mapping(mission_raw.get("defaults", {}), "mission.defaults")
    cruise_altitude_m = _number(
        defaults_raw.get("cruise_altitude_m", 5.0),
        "mission.defaults.cruise_altitude_m",
    )
    _altitude(cruise_altitude_m, "mission.defaults.cruise_altitude_m")

    max_speed_mps = _number(
        defaults_raw.get("max_speed_mps", 1.5),
        "mission.defaults.max_speed_mps",
    )
    if not 0.1 <= max_speed_mps <= 10.0:
        raise MissionValidationError(
            "mission.defaults.max_speed_mps must be between 0.1 and 10.0"
        )

    waypoint_tolerance_m = _number(
        defaults_raw.get("waypoint_tolerance_m", 0.4),
        "mission.defaults.waypoint_tolerance_m",
    )
    if not MIN_TOLERANCE_M <= waypoint_tolerance_m <= MAX_TOLERANCE_M:
        raise MissionValidationError(
            "mission.defaults.waypoint_tolerance_m must be between "
            f"{MIN_TOLERANCE_M} and {MAX_TOLERANCE_M} m"
        )

    mission_timeout_s = _number(
        defaults_raw.get("mission_timeout_s", 180.0),
        "mission.defaults.mission_timeout_s",
    )
    if not MIN_MISSION_TIMEOUT_S <= mission_timeout_s <= MAX_MISSION_TIMEOUT_S:
        raise MissionValidationError(
            "mission.defaults.mission_timeout_s must be between "
            f"{MIN_MISSION_TIMEOUT_S} and {MAX_MISSION_TIMEOUT_S} s"
        )

    waypoints_raw = mission_raw.get("waypoints")
    if not isinstance(waypoints_raw, list) or not waypoints_raw:
        raise MissionValidationError("mission.waypoints must be a non-empty list")

    waypoints: list[Waypoint] = []
    seen_ids: set[str] = set()

    for index, item in enumerate(waypoints_raw):
        where = f"mission.waypoints[{index}]"
        wp = _mapping(item, where)
        wp_id = _text(wp.get("id"), f"{where}.id")
        if wp_id in seen_ids:
            raise MissionValidationError(f"duplicate waypoint id: {wp_id}")
        seen_ids.add(wp_id)

        x = _number(wp.get("x"), f"{where}.x")
        y = _number(wp.get("y"), f"{where}.y")
        z = _number(wp.get("z"), f"{where}.z")
        if z > 0.0:
            raise MissionValidationError(
                f"{where}.z must be <= 0 in PX4 local NED (down is positive)"
            )
        _altitude(-z, f"{where}.z altitude")

        radius = math.hypot(x - home_x, y - home_y)
        if radius > MAX_RADIUS_M:
            raise MissionValidationError(
                f"{where} is {radius:.1f} m from home; maximum is {MAX_RADIUS_M:.1f} m"
            )

        hold_s = _number(wp.get("hold_s", 0.0), f"{where}.hold_s")
        if not 0.0 <= hold_s <= MAX_HOLD_S:
            raise MissionValidationError(
                f"{where}.hold_s must be between 0 and {MAX_HOLD_S} s"
            )

        action = _text(wp.get("action", "inspect"), f"{where}.action")
        if action not in ALLOWED_ACTIONS:
            raise MissionValidationError(
                f"{where}.action must be one of {sorted(ALLOWED_ACTIONS)}"
            )

        target_id = str(wp.get("target_id", "")).strip()
        tolerance_m = _number(
            wp.get("tolerance_m", waypoint_tolerance_m),
            f"{where}.tolerance_m",
        )
        if not MIN_TOLERANCE_M <= tolerance_m <= MAX_TOLERANCE_M:
            raise MissionValidationError(
                f"{where}.tolerance_m must be between "
                f"{MIN_TOLERANCE_M} and {MAX_TOLERANCE_M} m"
            )

        waypoints.append(
            Waypoint(
                id=wp_id,
                x=x,
                y=y,
                z=z,
                hold_s=hold_s,
                action=action,
                target_id=target_id,
                tolerance_m=tolerance_m,
            )
        )

    return Mission(
        name=name,
        frame=frame,
        home_x=home_x,
        home_y=home_y,
        home_z=home_z,
        cruise_altitude_m=cruise_altitude_m,
        max_speed_mps=max_speed_mps,
        waypoint_tolerance_m=waypoint_tolerance_m,
        mission_timeout_s=mission_timeout_s,
        waypoints=tuple(waypoints),
        return_to_launch=_bool(
            mission_raw.get("return_to_launch"), "mission.return_to_launch", True
        ),
        land_on_complete=_bool(
            mission_raw.get("land_on_complete"), "mission.land_on_complete", True
        ),
    )


def load_mission(path: str | Path) -> Mission:
    path = Path(path)
    if not path.is_file():
        raise MissionValidationError(f"mission file not found: {path}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise MissionValidationError(f"invalid YAML in {path}: {exc}") from exc
    return parse_mission(data)
