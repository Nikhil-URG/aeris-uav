#!/usr/bin/env python3
"""Spawn the buildings described by the AERIS grid map into Gazebo."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from aeris_grid_navigation.grid import GridMap, load_grid

PALETTE = [
    ("0.22 0.24 0.28 1", "0.33 0.36 0.41 1"),
    ("0.24 0.22 0.20 1", "0.38 0.35 0.31 1"),
    ("0.18 0.23 0.25 1", "0.30 0.38 0.41 1"),
    ("0.25 0.22 0.25 1", "0.40 0.35 0.40 1"),
]


def building_sdf(
    name: str,
    width_m: float,
    depth_m: float,
    height_m: float,
    palette_index: int,
) -> str:
    ambient, diffuse = PALETTE[palette_index % len(PALETTE)]
    return f"""
<sdf version='1.9'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <box><size>{width_m:.3f} {depth_m:.3f} {height_m:.3f}</size></box>
        </geometry>
        <material>
          <ambient>{ambient}</ambient>
          <diffuse>{diffuse}</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>
          <box><size>{width_m:.3f} {depth_m:.3f} {height_m:.3f}</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--grid",
        type=Path,
        default=None,
        help="Grid YAML path (default: aeris_bringup share city_grid.yaml)",
    )
    parser.add_argument(
        "--world", default="default", help="Gazebo world name"
    )
    parser.add_argument(
        "--gap-m",
        type=float,
        default=1.0,
        help="Visual/collision gap from the cell block boundary",
    )
    args = parser.parse_args()

    grid_path = args.grid
    if grid_path is None:
        grid_path = Path(
            get_package_share_directory("aeris_bringup")
        ) / "config" / "grids" / "city_grid.yaml"
    if not grid_path.is_file():
        print(f"ERROR: grid file not found: {grid_path}", file=sys.stderr)
        return 2

    grid: GridMap = load_grid(grid_path)
    print(
        f"[buildings] Spawning {len(grid.buildings)} buildings from "
        f"'{grid.name}' ({grid.width}x{grid.height}, "
        f"cell={grid.cell_size_m:.1f} m)."
    )

    print("[buildings] Waiting for Gazebo create service ...")
    for _ in range(30):
        result = subprocess.run(
            ["gz", "service", "-l"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        if f"/world/{args.world}/create" in result.stdout:
            break
        time.sleep(1)
    else:
        print(
            "ERROR: Gazebo create service not available; "
            "is the simulation running?",
            file=sys.stderr,
        )
        return 1

    for index, building in enumerate(grid.buildings):
        center_x = building.x + (building.width - 1) / 2.0
        center_y = building.y + (building.height - 1) / 2.0
        ned_x = (center_x - grid.start.x) * grid.cell_size_m
        ned_y = (center_y - grid.start.y) * grid.cell_size_m
        width_m = building.width * grid.cell_size_m - args.gap_m
        depth_m = building.height * grid.cell_size_m - args.gap_m
        name = f"building_{index:02d}"

        sdf = building_sdf(
            name, width_m, depth_m, building.height_m, index
        )
        print(
            f"[buildings] {name}: cell=({building.x},{building.y}) "
            f"size={width_m:.1f}x{depth_m:.1f}x{building.height_m:.1f} m "
            f"ned=({ned_x:.1f},{ned_y:.1f})"
        )
        subprocess.run(
            [
                "ros2", "run", "ros_gz_sim", "create",
                "-world", args.world,
                "-string", sdf,
                "-name", name,
                "-allow_renaming", "true",
                "-x", f"{ned_x:.3f}",
                "-y", f"{ned_y:.3f}",
                "-z", f"{building.height_m / 2.0:.3f}",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        time.sleep(0.25)

    print("[buildings] Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
