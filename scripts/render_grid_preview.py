#!/usr/bin/env python3
"""Render a static GUI preview of an AERIS grid path (no simulation needed)."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication  # noqa: E402

from aeris_grid_gui.gui import GridCanvas  # noqa: E402
from aeris_grid_navigation.grid import load_grid  # noqa: E402
from aeris_interfaces.msg import FlightStatus, GridStatus  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--grid",
        type=Path,
        default=None,
        help="Grid YAML path (default: aeris_bringup city_grid.yaml)",
    )
    parser.add_argument("--goal-x", type=int, default=10)
    parser.add_argument("--goal-y", type=int, default=10)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/experiments/plots/grid_gui_spread.png"),
    )
    args = parser.parse_args()

    if args.grid is None:
        root = os.environ.get("AERIS_ROOT")
        if root:
            args.grid = Path(root) / (
                "ros2_ws/src/aeris_bringup/config/grids/city_grid.yaml"
            )
        else:
            from ament_index_python.packages import (
                get_package_share_directory,
            )
            args.grid = Path(get_package_share_directory("aeris_bringup")) / (
                "config/grids/city_grid.yaml"
            )

    grid = load_grid(args.grid)
    goal = (args.goal_x, args.goal_y)
    from aeris_grid_navigation.grid import Cell
    path = grid.plan(grid.start, Cell(*goal))
    if path is None:
        print("error: no free path to the requested goal", file=sys.stderr)
        return 2

    current_index = max(1, int((len(path) - 1) * 0.55))
    current_cell = path[current_index]
    ned_x, ned_y = grid.cell_to_ned(current_cell)
    next_cell = path[min(current_index + 1, len(path) - 1)]
    next_x, next_y = grid.cell_to_ned(next_cell)
    yaw = math.atan2(next_y - ned_y, next_x - ned_x)

    status = GridStatus()
    status.grid_name = grid.name
    status.state = "TRANSIT"
    status.active = True
    status.success = False
    status.message = "navigating to goal"
    status.start_x = grid.start.x
    status.start_y = grid.start.y
    status.goal_x = goal[0]
    status.goal_y = goal[1]
    status.current_x = current_cell.x
    status.current_y = current_cell.y
    status.path_x = [cell.x for cell in path]
    status.path_y = [cell.y for cell in path]
    status.current_index = current_index
    status.progress = current_index / max(1, len(path) - 1)
    status.path_length_m = grid.path_length_m(path)
    status.remaining_m = status.path_length_m * (1.0 - status.progress)
    status.drone_x = ned_x
    status.drone_y = ned_y
    status.drone_z = -grid.cruise_altitude_m

    flight = FlightStatus()
    flight.state = "TRANSIT"
    flight.armed = True
    flight.offboard = True
    flight.x = ned_x
    flight.y = ned_y
    flight.z = -grid.cruise_altitude_m
    flight.yaw = yaw

    app = QApplication(sys.argv)
    canvas = GridCanvas(grid)
    canvas.resize(900, 803)
    canvas.set_data(status, flight)
    canvas.show()
    app.processEvents()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not canvas.grab().save(str(args.output)):
        print("error: could not save preview", file=sys.stderr)
        return 1
    print(
        f"wrote {args.output} "
        f"(grid={grid.name}, goal={goal}, path={len(path)} cells, "
        f"{status.path_length_m:.1f} m)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
