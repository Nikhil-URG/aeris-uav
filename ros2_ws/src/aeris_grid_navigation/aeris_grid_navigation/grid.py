"""Grid map, occupancy and shortest-path planning for AERIS.

Grid cells are indexed ``(x, y)`` where x is north and y is east. The cell
centres map to PX4 local NED as::

    ned_x = (cell.x - start.x) * cell_size_m
    ned_y = (cell.y - start.y) * cell_size_m

so the vehicle's start cell is always local NED ``(0, 0)``.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
from pathlib import Path
from typing import Any

import yaml

SQRT2 = math.sqrt(2.0)


class GridValidationError(ValueError):
    """Raised when a grid YAML file is structurally or semantically invalid."""


@dataclass(frozen=True, order=True)
class Cell:
    x: int
    y: int


@dataclass(frozen=True)
class Building:
    x: int
    y: int
    width: int
    height: int
    height_m: float

    def cells(self) -> list[Cell]:
        return [
            Cell(x, y)
            for x in range(self.x, self.x + self.width)
            for y in range(self.y, self.y + self.height)
        ]


@dataclass(frozen=True)
class GridMap:
    name: str
    width: int
    height: int
    cell_size_m: float
    start: Cell
    cruise_altitude_m: float
    buildings: tuple[Building, ...]
    occupied: frozenset[Cell]

    # ------------------------------------------------------------------
    # Geometry
    # ------------------------------------------------------------------
    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell.x < self.width and 0 <= cell.y < self.height

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and cell not in self.occupied

    def cell_to_ned(self, cell: Cell) -> tuple[float, float]:
        return (
            (cell.x - self.start.x) * self.cell_size_m,
            (cell.y - self.start.y) * self.cell_size_m,
        )

    def ned_to_cell(self, x: float, y: float) -> Cell:
        gx = round(self.start.x + x / self.cell_size_m)
        gy = round(self.start.y + y / self.cell_size_m)
        gx = min(max(gx, 0), self.width - 1)
        gy = min(max(gy, 0), self.height - 1)
        return Cell(gx, gy)

    def path_length_m(self, path: list[Cell]) -> float:
        total = 0.0
        for first, second in zip(path, path[1:]):
            dx = (second.x - first.x) * self.cell_size_m
            dy = (second.y - first.y) * self.cell_size_m
            total += math.hypot(dx, dy)
        return total

    # ------------------------------------------------------------------
    # A* planning
    # ------------------------------------------------------------------
    def neighbors(self, cell: Cell) -> list[tuple[Cell, float]]:
        result: list[tuple[Cell, float]] = []
        for dx, dy, cost in (
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, SQRT2),
            (1, -1, SQRT2),
            (-1, 1, SQRT2),
            (-1, -1, SQRT2),
        ):
            neighbor = Cell(cell.x + dx, cell.y + dy)
            if not self.is_free(neighbor):
                continue
            # Do not cut through the corner of an occupied cell.
            if dx != 0 and dy != 0:
                if not self.is_free(Cell(cell.x + dx, cell.y)):
                    continue
                if not self.is_free(Cell(cell.x, cell.y + dy)):
                    continue
            result.append((neighbor, cost))
        return result

    @staticmethod
    def _heuristic(first: Cell, second: Cell) -> float:
        return math.hypot(first.x - second.x, first.y - second.y)

    def plan(self, start: Cell, goal: Cell) -> list[Cell] | None:
        """Return the shortest free-cell path including start and goal."""
        if not self.is_free(start) or not self.is_free(goal):
            return None
        if start == goal:
            return [start]

        counter = itertools.count()
        open_set: list[tuple[float, int, Cell]] = [
            (self._heuristic(start, goal), next(counter), start)
        ]
        came_from: dict[Cell, Cell] = {}
        best_cost: dict[Cell, float] = {start: 0.0}

        while open_set:
            _, _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for neighbor, step_cost in self.neighbors(current):
                tentative = best_cost[current] + step_cost
                if tentative < best_cost.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    best_cost[neighbor] = tentative
                    priority = tentative + self._heuristic(neighbor, goal)
                    heapq.heappush(
                        open_set, (priority, next(counter), neighbor)
                    )

        return None


def _mapping(value: Any, where: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise GridValidationError(f"{where} must be a mapping")
    return value


def _integer(value: Any, where: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise GridValidationError(f"{where} must be an integer")
    return value


def _number(value: Any, where: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise GridValidationError(f"{where} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise GridValidationError(f"{where} must be finite")
    return value


def parse_grid(data: Any) -> GridMap:
    root = _mapping(data, "root")
    raw = _mapping(root.get("grid"), "grid")

    name = str(raw.get("name", "grid"))
    width = _integer(raw.get("width"), "grid.width")
    height = _integer(raw.get("height"), "grid.height")
    cell_size_m = _number(raw.get("cell_size_m"), "grid.cell_size_m")
    cruise_altitude_m = _number(
        raw.get("cruise_altitude_m", 5.0), "grid.cruise_altitude_m"
    )
    if width < 2 or height < 2:
        raise GridValidationError("grid width and height must be at least 2")
    if cell_size_m <= 0.0:
        raise GridValidationError("grid.cell_size_m must be positive")
    if cruise_altitude_m <= 0.0:
        raise GridValidationError("grid.cruise_altitude_m must be positive")

    start_raw = _mapping(raw.get("start"), "grid.start")
    start = Cell(
        _integer(start_raw.get("x"), "grid.start.x"),
        _integer(start_raw.get("y"), "grid.start.y"),
    )

    buildings_raw = raw.get("buildings", [])
    if not isinstance(buildings_raw, list):
        raise GridValidationError("grid.buildings must be a list")

    buildings: list[Building] = []
    occupied: set[Cell] = set()
    for index, item in enumerate(buildings_raw):
        where = f"grid.buildings[{index}]"
        item = _mapping(item, where)
        building = Building(
            x=_integer(item.get("x"), f"{where}.x"),
            y=_integer(item.get("y"), f"{where}.y"),
            width=_integer(item.get("width", 1), f"{where}.width"),
            height=_integer(item.get("height", 1), f"{where}.height"),
            height_m=_number(item.get("height_m", 10.0), f"{where}.height_m"),
        )
        if building.width < 1 or building.height < 1:
            raise GridValidationError(f"{where} width/height must be positive")
        for cell in building.cells():
            if cell.x < 0 or cell.x >= width or cell.y < 0 or cell.y >= height:
                raise GridValidationError(f"{where} extends outside the grid")
            if cell == start:
                raise GridValidationError(f"{where} covers the start cell")
            if cell in occupied:
                raise GridValidationError(
                    f"{where} overlaps another building at ({cell.x}, {cell.y})"
                )
            occupied.add(cell)
        buildings.append(building)

    if start.x < 0 or start.x >= width or start.y < 0 or start.y >= height:
        raise GridValidationError("grid.start is outside the grid")
    if start in occupied:
        raise GridValidationError("grid.start is occupied")

    return GridMap(
        name=name,
        width=width,
        height=height,
        cell_size_m=cell_size_m,
        start=start,
        cruise_altitude_m=cruise_altitude_m,
        buildings=tuple(buildings),
        occupied=frozenset(occupied),
    )


def load_grid(path: str | Path) -> GridMap:
    path = Path(path)
    if not path.is_file():
        raise GridValidationError(f"grid file not found: {path}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise GridValidationError(f"invalid YAML in {path}: {exc}") from exc
    return parse_grid(data)
