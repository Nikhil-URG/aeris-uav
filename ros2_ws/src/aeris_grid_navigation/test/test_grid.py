"""Unit tests for the AERIS grid planner."""

from __future__ import annotations

from copy import deepcopy

import pytest

from aeris_grid_navigation.grid import (
    Cell,
    GridValidationError,
    parse_grid,
)


def base_grid():
    return {
        "grid": {
            "name": "test_grid",
            "width": 5,
            "height": 5,
            "cell_size_m": 4.0,
            "start": {"x": 0, "y": 0},
            "cruise_altitude_m": 5.0,
            "buildings": [],
        }
    }


def test_cell_to_ned_uses_start_as_origin():
    data = base_grid()
    data["grid"]["start"] = {"x": 2, "y": 2}
    grid = parse_grid(data)

    assert grid.cell_to_ned(Cell(2, 2)) == (0.0, 0.0)
    assert grid.cell_to_ned(Cell(3, 2)) == (4.0, 0.0)
    assert grid.cell_to_ned(Cell(2, 3)) == (0.0, 4.0)
    assert grid.ned_to_cell(3.9, -0.1) == Cell(3, 2)


def test_straight_path_is_shortest():
    grid = parse_grid(base_grid())
    path = grid.plan(Cell(0, 0), Cell(0, 4))

    assert path == [Cell(0, y) for y in range(5)]
    assert grid.path_length_m(path) == 16.0


def test_path_avoids_occupied_cells():
    data = base_grid()
    data["grid"]["buildings"] = [
        {"x": 0, "y": 2, "width": 1, "height": 1, "height_m": 10.0}
    ]
    grid = parse_grid(data)
    path = grid.plan(Cell(0, 0), Cell(0, 4))

    assert path is not None
    assert Cell(0, 2) not in path
    assert all(grid.is_free(cell) for cell in path)
    # A detour around one blocked cell adds distance.
    assert grid.path_length_m(path) > 16.0


def test_no_path_when_goal_is_walled_in():
    data = base_grid()
    data["grid"]["buildings"] = [
        {"x": 1, "y": 3, "width": 1, "height": 1, "height_m": 10.0},
        {"x": 3, "y": 3, "width": 1, "height": 1, "height_m": 10.0},
        {"x": 2, "y": 4, "width": 1, "height": 1, "height_m": 10.0},
        {"x": 2, "y": 2, "width": 1, "height": 1, "height_m": 10.0},
    ]
    grid = parse_grid(data)

    assert grid.plan(Cell(0, 0), Cell(2, 3)) is None


def test_no_corner_cutting():
    data = base_grid()
    data["grid"]["buildings"] = [
        {"x": 1, "y": 0, "width": 1, "height": 1, "height_m": 10.0}
    ]
    grid = parse_grid(data)
    path = grid.plan(Cell(0, 0), Cell(1, 1))

    assert path is not None
    # The direct diagonal (0,0)->(1,1) would cut past the building at
    # (1,0); the planner must take at least three cells.
    assert len(path) >= 3
    assert Cell(1, 0) not in path


def test_start_equals_goal():
    grid = parse_grid(base_grid())
    assert grid.plan(Cell(2, 2), Cell(2, 2)) == [Cell(2, 2)]


def test_ned_to_cell_clamps_to_grid():
    grid = parse_grid(base_grid())
    assert grid.ned_to_cell(-100.0, -100.0) == Cell(0, 0)
    assert grid.ned_to_cell(100.0, 100.0) == Cell(4, 4)


def test_overlapping_buildings_rejected():
    data = base_grid()
    data["grid"]["buildings"] = [
        {"x": 1, "y": 1, "width": 2, "height": 2, "height_m": 10.0},
        {"x": 2, "y": 2, "width": 2, "height": 2, "height_m": 10.0},
    ]
    with pytest.raises(GridValidationError, match="overlaps"):
        parse_grid(data)


def test_building_on_start_rejected():
    data = deepcopy(base_grid())
    data["grid"]["buildings"] = [
        {"x": 0, "y": 0, "width": 1, "height": 1, "height_m": 10.0}
    ]
    with pytest.raises(GridValidationError, match="start"):
        parse_grid(data)
