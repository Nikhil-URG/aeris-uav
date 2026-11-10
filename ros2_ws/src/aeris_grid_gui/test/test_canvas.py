"""Headless tests for the grid GUI canvas click mapping."""

from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtCore import QPoint, Qt  # noqa: E402
from PyQt5.QtTest import QTest  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

from aeris_grid_gui.gui import GridCanvas  # noqa: E402
from aeris_grid_navigation.grid import Cell, parse_grid  # noqa: E402


def make_grid():
    return parse_grid(
        {
            "grid": {
                "name": "test",
                "width": 5,
                "height": 5,
                "cell_size_m": 4.0,
                "start": {"x": 2, "y": 2},
                "cruise_altitude_m": 5.0,
                "buildings": [
                    {"x": 0, "y": 0, "width": 1, "height": 1,
                     "height_m": 10.0}
                ],
            }
        }
    )


def test_click_maps_to_grid_cell():
    app = QApplication.instance() or QApplication([])
    canvas = GridCanvas(make_grid())
    canvas.resize(700, 700)
    canvas.show()
    QTest.qWait(50)

    clicked: list[Cell] = []
    canvas.cell_clicked.connect(clicked.append)

    target = Cell(3, 1)
    canvas._origin_x, canvas._origin_y, canvas._cell_px = canvas._geometry()
    rect = canvas._cell_rect(target)
    point = QPoint(int(rect.center().x()), int(rect.center().y()))
    QTest.mouseClick(canvas, Qt.LeftButton, pos=point)
    app.processEvents()

    assert clicked == [target]
    canvas.close()
