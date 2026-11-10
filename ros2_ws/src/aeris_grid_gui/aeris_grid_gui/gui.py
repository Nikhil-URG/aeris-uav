"""AERIS grid goal-selection GUI.

Click a free grid cell to send the drone there. The grid navigation node plans
the shortest free path, flies it, returns to the start cell and lands. The
canvas shows buildings, the planned path, the goal, and the live drone pose.
"""

from __future__ import annotations

import math
import os
import queue
import sys
import threading

from aeris_grid_navigation.grid import Cell, GridMap, load_grid
from aeris_interfaces.msg import FlightStatus, GridStatus, SafetyStatus
from aeris_interfaces.srv import GridGoal
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class RosBridge:
    """Owns the rclpy node and a background executor."""

    def __init__(self) -> None:
        rclpy.init(args=sys.argv)
        self.node = Node("aeris_grid_gui")
        self.node.declare_parameter("grid_file", "")
        grid_file = str(self.node.get_parameter("grid_file").value).strip()
        if not grid_file:
            root = os.environ.get("AERIS_ROOT")
            if not root:
                raise RuntimeError(
                    "grid_file parameter is required "
                    "(or set AERIS_ROOT)"
                )
            grid_file = os.path.join(
                root, "ros2_ws", "src", "aeris_bringup",
                "config", "grids", "city_grid.yaml",
            )
        self.grid = load_grid(grid_file)

        self._lock = threading.Lock()
        self._grid_status: GridStatus | None = None
        self._flight_status: FlightStatus | None = None
        self._safety_status: SafetyStatus | None = None
        self._message = "Click a free cell to send the drone there."
        self._shutdown = False

        self._goal_queue: queue.Queue[Cell] = queue.Queue()
        self._land_queue: queue.Queue[bool] = queue.Queue()

        self._goal_client = self.node.create_client(
            GridGoal, "/aeris_grid_navigation/set_goal"
        )
        self._land_client = self.node.create_client(
            Trigger, "/aeris_offboard_controller/land"
        )
        self.node.create_subscription(
            GridStatus, "/aeris/grid/status", self._on_grid_status, 10
        )
        self.node.create_subscription(
            FlightStatus,
            "/aeris_offboard_controller/flight_status",
            self._on_flight_status,
            10,
        )
        self.node.create_subscription(
            SafetyStatus, "/aeris/safety/status", self._on_safety_status, 10
        )
        self.node.create_timer(0.1, self._process_queues)

        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

        self.node.get_logger().info(
            f"AERIS grid GUI connected to '{self.grid.name}' "
            f"({self.grid.width}x{self.grid.height}, "
            f"cell={self.grid.cell_size_m:.1f} m)."
        )

    # ------------------------------------------------------------------
    def _spin(self) -> None:
        try:
            rclpy.spin(self.node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().error(f"ROS spin stopped: {exc}")

    def _on_grid_status(self, msg: GridStatus) -> None:
        with self._lock:
            self._grid_status = msg

    def _on_flight_status(self, msg: FlightStatus) -> None:
        with self._lock:
            self._flight_status = msg

    def _on_safety_status(self, msg: SafetyStatus) -> None:
        with self._lock:
            self._safety_status = msg

    def _set_message(self, message: str) -> None:
        with self._lock:
            self._message = message

    # ------------------------------------------------------------------
    def queue_goal(self, cell: Cell) -> None:
        self._goal_queue.put(cell)

    def queue_land(self) -> None:
        self._land_queue.put(True)

    def _process_queues(self) -> None:
        if not self._goal_queue.empty():
            cell = self._goal_queue.get_nowait()
            if not self._goal_client.service_is_ready():
                self._set_message(
                    "Grid navigation service is not available."
                )
            else:
                request = GridGoal.Request()
                request.x = cell.x
                request.y = cell.y
                future = self._goal_client.call_async(request)
                future.add_done_callback(self._goal_done)
                self._set_message(
                    f"Requesting goal ({cell.x}, {cell.y})..."
                )

        if not self._land_queue.empty():
            self._land_queue.get_nowait()
            if not self._land_client.service_is_ready():
                self._set_message("Offboard land service is not available.")
            else:
                future = self._land_client.call_async(Trigger.Request())
                future.add_done_callback(self._land_done)
                self._set_message("Landing requested.")

    def _goal_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._set_message(f"Goal request failed: {exc}")
            return
        if response.accepted:
            self._set_message(
                f"Goal accepted: {response.path_cells} cells, "
                f"{response.path_length_m:.1f} m path."
            )
        else:
            self._set_message(f"Goal rejected: {response.message}")

    def _land_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._set_message(f"Land request failed: {exc}")
            return
        self._set_message(
            "Landing accepted." if response.success
            else f"Landing rejected: {response.message}"
        )

    def snapshot(self):
        with self._lock:
            return (
                self._grid_status,
                self._flight_status,
                self._safety_status,
                self._message,
            )

    def shutdown(self) -> None:
        if self._shutdown:
            return
        self._shutdown = True
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self._thread.join(timeout=2.0)


class GridCanvas(QWidget):
    cell_clicked = pyqtSignal(object)

    def __init__(self, grid: GridMap) -> None:
        super().__init__()
        self.grid = grid
        self.grid_status: GridStatus | None = None
        self.flight_status: FlightStatus | None = None
        self.setMinimumSize(620, 620)
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._cell_px = 1.0

    def set_data(
        self,
        grid_status: GridStatus | None,
        flight_status: FlightStatus | None,
    ) -> None:
        self.grid_status = grid_status
        self.flight_status = flight_status

    # ------------------------------------------------------------------
    def _geometry(self) -> tuple[float, float, float]:
        margin = 34.0
        available_w = max(1.0, self.width() - 2.0 * margin)
        available_h = max(1.0, self.height() - 2.0 * margin)
        cell_px = min(
            available_w / self.grid.width,
            available_h / self.grid.height,
        )
        board_w = cell_px * self.grid.width
        board_h = cell_px * self.grid.height
        return (
            (self.width() - board_w) / 2.0,
            (self.height() - board_h) / 2.0,
            cell_px,
        )

    def _cell_rect(self, cell: Cell) -> QRectF:
        x = self._origin_x + cell.y * self._cell_px
        y = self._origin_y + (self.grid.height - 1 - cell.x) * self._cell_px
        return QRectF(x, y, self._cell_px, self._cell_px)

    def _cell_center(self, cell: Cell) -> QPointF:
        rect = self._cell_rect(cell)
        return rect.center()

    def _ned_to_point(self, x: float, y: float) -> QPointF:
        grid_x = self.grid.start.x + x / self.grid.cell_size_m
        grid_y = self.grid.start.y + y / self.grid.cell_size_m
        px = self._origin_x + (grid_y + 0.5) * self._cell_px
        py = (
            self._origin_y
            + (self.grid.height - 1 - grid_x + 0.5) * self._cell_px
        )
        return QPointF(px, py)

    # ------------------------------------------------------------------
    def paintEvent(self, event) -> None:  # noqa: N802 - Qt API
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#20242b"))

        self._origin_x, self._origin_y, self._cell_px = self._geometry()

        goal = None
        path: list[Cell] = []
        status = self.grid_status
        if status is not None:
            if status.goal_x >= 0 and status.goal_y >= 0:
                goal = Cell(status.goal_x, status.goal_y)
            path = list(zip(status.path_x, status.path_y))
            path = [Cell(x, y) for x, y in path]

        path_set = set(path)
        for x in range(self.grid.width):
            for y in range(self.grid.height):
                cell = Cell(x, y)
                rect = self._cell_rect(cell)
                if cell in self.grid.occupied:
                    color = QColor("#41464f")
                elif cell in path_set:
                    color = QColor("#ffe08a")
                elif cell == self.grid.start:
                    color = QColor("#bfe3ca")
                elif goal is not None and cell == goal:
                    color = QColor("#bcd7ff")
                else:
                    color = QColor("#f3f1ea")
                painter.fillRect(rect, color)
                painter.setPen(QPen(QColor("#8b8f96"), 1))
                painter.drawRect(rect)

        # Building height labels.
        painter.setPen(QColor("#e8eaf0"))
        painter.setFont(QFont("Sans", 8, QFont.Bold))
        for building in self.grid.buildings:
            center_x = building.x + (building.width - 1) / 2.0
            center_y = building.y + (building.height - 1) / 2.0
            rect = self._cell_rect(Cell(int(center_x), int(center_y)))
            painter.drawText(
                rect, Qt.AlignCenter, f"{building.height_m:.0f}m"
            )

        # Path line.
        if len(path) > 1:
            painter.setPen(QPen(QColor("#d99a00"), 4))
            points = [self._cell_center(cell) for cell in path]
            for first, second in zip(points, points[1:]):
                painter.drawLine(first, second)

        # Start and goal markers.
        start_center = self._cell_center(self.grid.start)
        painter.setPen(QPen(QColor("#1e7b45"), 2))
        painter.setBrush(QColor("#1e7b45"))
        painter.drawEllipse(start_center, 7, 7)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QColor("#173b2a"))
        painter.setFont(QFont("Sans", 9, QFont.Bold))
        painter.drawText(
            self._cell_rect(self.grid.start), Qt.AlignTop | Qt.AlignLeft,
            " S",
        )

        if goal is not None:
            goal_center = self._cell_center(goal)
            painter.setPen(QPen(QColor("#1f5fa8"), 2))
            painter.setBrush(QColor("#1f5fa8"))
            painter.drawEllipse(goal_center, 7, 7)
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QColor("#173b6b"))
            painter.drawText(
                self._cell_rect(goal), Qt.AlignTop | Qt.AlignLeft, " G"
            )

        # Drone.
        if self.flight_status is not None:
            point = self._ned_to_point(
                float(self.flight_status.x), float(self.flight_status.y)
            )
            radius = max(7.0, self._cell_px * 0.16)
            painter.setPen(QPen(QColor("#ffffff"), 2))
            painter.setBrush(QColor("#d62728"))
            painter.drawEllipse(point, radius, radius)
            yaw = float(self.flight_status.yaw)
            heading = QPointF(
                point.x() + radius * 1.8 * math.sin(yaw),
                point.y() - radius * 1.8 * math.cos(yaw),
            )
            painter.setPen(QPen(QColor("#ffffff"), 3))
            painter.drawLine(point, heading)

        # North arrow.
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.setFont(QFont("Sans", 9, QFont.Bold))
        ax = self._origin_x + 18
        ay = self._origin_y + 34
        painter.drawLine(QPointF(ax, ay), QPointF(ax, ay - 20))
        painter.drawLine(QPointF(ax, ay - 20), QPointF(ax - 5, ay - 12))
        painter.drawLine(QPointF(ax, ay - 20), QPointF(ax + 5, ay - 12))
        painter.drawText(QPointF(ax + 8, ay - 8), "N")

        # Cell coordinate labels.
        painter.setPen(QColor("#c7cbd2"))
        painter.setFont(QFont("Sans", 7))
        for index in range(self.grid.width):
            rect = self._cell_rect(Cell(index, 0))
            painter.drawText(rect, Qt.AlignCenter, str(index))
        for index in range(self.grid.height):
            rect = self._cell_rect(Cell(0, index))
            painter.drawText(rect, Qt.AlignTop | Qt.AlignHCenter, str(index))

    # ------------------------------------------------------------------
    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt API
        if event.button() != Qt.LeftButton:
            return
        x = event.x() - self._origin_x
        y = event.y() - self._origin_y
        board_w = self._cell_px * self.grid.width
        board_h = self._cell_px * self.grid.height
        if x < 0 or y < 0 or x >= board_w or y >= board_h:
            return
        column = int(x / self._cell_px)
        row_from_bottom = int(y / self._cell_px)
        grid_x = self.grid.height - 1 - row_from_bottom
        grid_y = column
        if 0 <= grid_x < self.grid.width and 0 <= grid_y < self.grid.height:
            self.cell_clicked.emit(Cell(grid_x, grid_y))


class MainWindow(QMainWindow):
    def __init__(self, bridge: RosBridge) -> None:
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("AERIS — Grid Navigation")
        self.resize(900, 860)

        central = QWidget()
        layout = QVBoxLayout(central)

        title = QLabel("AERIS Grid Navigation")
        title.setStyleSheet(
            "font-size: 22px; font-weight: 700; color: #f0f2f5;"
        )
        layout.addWidget(title)

        self.status_label = QLabel("Waiting for grid navigation...")
        self.status_label.setStyleSheet("font-size: 12px; color: #c7cbd2;")
        layout.addWidget(self.status_label)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setFormat("%p% of path")
        layout.addWidget(self.progress)

        self.canvas = GridCanvas(bridge.grid)
        self.canvas.cell_clicked.connect(self.on_cell_clicked)
        layout.addWidget(self.canvas, stretch=1)

        row = QHBoxLayout()
        self.safety_label = QLabel("Safety: waiting")
        self.safety_label.setStyleSheet("color: #c7cbd2; font-weight: 600;")
        row.addWidget(self.safety_label)
        row.addStretch(1)

        home_button = QPushButton("Go Home")
        home_button.clicked.connect(self.on_go_home)
        row.addWidget(home_button)

        land_button = QPushButton("Land")
        land_button.clicked.connect(self.on_land)
        row.addWidget(land_button)

        quit_button = QPushButton("Quit")
        quit_button.clicked.connect(self.close)
        row.addWidget(quit_button)
        layout.addLayout(row)

        self.message_label = QLabel(
            "Click a free cell. Gray cells are buildings."
        )
        self.message_label.setWordWrap(True)
        self.message_label.setStyleSheet("color: #e5c07b;")
        layout.addWidget(self.message_label)

        legend = QLabel(
            "gray = building   green = start   blue = goal   "
            "yellow = planned path   red = drone"
        )
        legend.setStyleSheet("color: #9aa0a6; font-size: 10px;")
        layout.addWidget(legend)

        central.setStyleSheet("background-color: #2b3038;")
        self.setCentralWidget(central)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def on_cell_clicked(self, cell: Cell) -> None:
        if not self.bridge.grid.is_free(cell):
            self.message_label.setText(
                f"Cell ({cell.x}, {cell.y}) is occupied by a building."
            )
            return
        self.bridge.queue_goal(cell)
        self.message_label.setText(
            f"Goal requested at ({cell.x}, {cell.y})."
        )

    def on_go_home(self) -> None:
        self.bridge.queue_goal(self.bridge.grid.start)

    def on_land(self) -> None:
        self.bridge.queue_land()

    def refresh(self) -> None:
        grid_status, flight_status, safety_status, message = (
            self.bridge.snapshot()
        )
        self.canvas.set_data(grid_status, flight_status)
        self.canvas.update()

        if grid_status is None:
            self.status_label.setText(
                "Waiting for /aeris/grid/status — is grid navigation running?"
            )
            self.progress.setValue(0)
        else:
            state = grid_status.state or "IDLE"
            goal = (
                f"({grid_status.goal_x}, {grid_status.goal_y})"
                if grid_status.goal_x >= 0 else "—"
            )
            drone = (
                f"({grid_status.drone_x:.1f}, {grid_status.drone_y:.1f})"
            )
            self.status_label.setText(
                f"State: {state}   Goal: {goal}   "
                f"Path: {len(grid_status.path_x)} cells / "
                f"{grid_status.path_length_m:.1f} m   "
                f"Remaining: {grid_status.remaining_m:.1f} m   "
                f"Drone NED: {drone}"
            )
            self.progress.setValue(
                int(max(0.0, min(1.0, grid_status.progress)) * 100.0)
            )

        if safety_status is None:
            self.safety_label.setText("Safety: not connected")
            self.safety_label.setStyleSheet(
                "color: #9aa0a6; font-weight: 600;"
            )
        elif safety_status.safe:
            self.safety_label.setText("Safety: OK")
            self.safety_label.setStyleSheet(
                "color: #7dda9a; font-weight: 700;"
            )
        else:
            faults = ", ".join(safety_status.active_faults)
            self.safety_label.setText(
                f"SAFETY: {faults} → {safety_status.action}"
            )
            self.safety_label.setStyleSheet(
                "color: #ff6b6b; font-weight: 700;"
            )

        self.message_label.setText(message)

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API
        self.timer.stop()
        self.bridge.shutdown()
        event.accept()


def main() -> None:
    bridge = RosBridge()
    app = QApplication(sys.argv)
    window = MainWindow(bridge)
    window.show()
    try:
        app.exec_()
    finally:
        if rclpy.ok():
            bridge.shutdown()


if __name__ == "__main__":
    main()
