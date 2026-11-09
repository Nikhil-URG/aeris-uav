from copy import deepcopy

import pytest
import yaml

from aeris_mission_manager.mission import (
    MissionValidationError,
    load_mission,
    parse_mission,
)


VALID = {
    "mission": {
        "name": "test_mission",
        "frame": "local_ned",
        "home": {"x": 0.0, "y": 0.0, "z": 0.0},
        "defaults": {
            "cruise_altitude_m": 5.0,
            "max_speed_mps": 1.5,
            "waypoint_tolerance_m": 0.4,
            "mission_timeout_s": 180.0,
        },
        "waypoints": [
            {
                "id": "wp_01",
                "x": 4.0,
                "y": 0.0,
                "z": -5.0,
                "hold_s": 3.0,
                "action": "inspect",
                "target_id": "valve_a",
            },
            {
                "id": "wp_02",
                "x": 4.0,
                "y": 4.0,
                "z": -5.0,
                "hold_s": 3.0,
                "action": "inspect",
                "target_id": "gauge_b",
            },
        ],
        "return_to_launch": True,
        "land_on_complete": True,
    }
}


def test_valid_mission_parses():
    mission = parse_mission(deepcopy(VALID))
    assert mission.name == "test_mission"
    assert mission.frame == "local_ned"
    assert len(mission.waypoints) == 2
    assert mission.waypoints[0].id == "wp_01"
    assert mission.return_to_launch is True


def test_missing_waypoints_rejected():
    data = deepcopy(VALID)
    data["mission"]["waypoints"] = []
    with pytest.raises(MissionValidationError):
        parse_mission(data)


def test_duplicate_ids_rejected():
    data = deepcopy(VALID)
    data["mission"]["waypoints"][1]["id"] = "wp_01"
    with pytest.raises(MissionValidationError, match="duplicate"):
        parse_mission(data)


def test_bad_frame_rejected():
    data = deepcopy(VALID)
    data["mission"]["frame"] = "enu"
    with pytest.raises(MissionValidationError, match="local_ned"):
        parse_mission(data)


def test_altitude_limit_rejected():
    data = deepcopy(VALID)
    data["mission"]["waypoints"][0]["z"] = -15.0
    with pytest.raises(MissionValidationError, match="altitude"):
        parse_mission(data)


def test_positive_z_rejected():
    data = deepcopy(VALID)
    data["mission"]["waypoints"][0]["z"] = 5.0
    with pytest.raises(MissionValidationError, match="NED"):
        parse_mission(data)


def test_geofence_rejected():
    data = deepcopy(VALID)
    data["mission"]["waypoints"][0]["x"] = 30.0
    with pytest.raises(MissionValidationError, match="home"):
        parse_mission(data)


def test_bad_action_rejected():
    data = deepcopy(VALID)
    data["mission"]["waypoints"][0]["action"] = "fly"
    with pytest.raises(MissionValidationError, match="action"):
        parse_mission(data)


def test_negative_hold_rejected():
    data = deepcopy(VALID)
    data["mission"]["waypoints"][0]["hold_s"] = -1.0
    with pytest.raises(MissionValidationError, match="hold_s"):
        parse_mission(data)


def test_load_mission_file(tmp_path):
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(yaml.safe_dump(VALID), encoding="utf-8")
    mission = load_mission(mission_file)
    assert mission.name == "test_mission"


def test_missing_file_rejected(tmp_path):
    with pytest.raises(MissionValidationError, match="not found"):
        load_mission(tmp_path / "does_not_exist.yaml")
