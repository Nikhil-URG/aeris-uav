"""AERIS grid navigation: shortest-path flight to a selected cell and return."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import threading
import time

from aeris_grid_navigation.grid import (
    Cell,
    GridMap,
    GridValidationError,
    load_grid,
)
from aeris_interfaces.msg import FlightStatus, GridStatus, SafetyStatus
from aeris_interfaces.srv import GoTo, GridGoal
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class SafetyAbort(RuntimeError):
    """Raised when the safety supervisor declares the flight unsafe."""

    def __init__(self, faults: list[str], action: str) -> None:
        self.faults = list(faults)
        self.action = action
        reason = ", ".join(self.faults) if self.faults else "unknown"
        super().__init__(f"safety fault ({reason}); action={action}")


class GridNavigation(Node):
    def __init__(self) -> None:
        super().__init__("aeris_grid_navigation")

        self.declare_parameter("grid_file", "")
        self.declare_parameter(
            "flight_status_topic", "/aeris_offboard_controller/flight_status"
        )
        self.declare_parameter("safety_topic", "/aeris/safety/status")
        self.declare_parameter("status_topic", "/aeris/grid/status")
        self.declare_parameter(
            "start_service", "/aeris_offboard_controller/start"
        )
        self.declare_parameter("goto_service", "/aeris_offboard_controller/go_to")
        self.declare_parameter("land_service", "/aeris_offboard_controller/land")
        self.declare_parameter("abort_service", "/aeris_offboard_controller/abort")
        self.declare_parameter("takeoff_timeout_s", 90.0)
        self.declare_parameter("waypoint_timeout_s", 45.0)
        self.declare_parameter("goal_hold_s", 3.0)
        self.declare_parameter("waypoint_tolerance_m", 0.8)
        self.declare_parameter("mission_timeout_s", 300.0)
        self.declare_parameter("land_timeout_s", 60.0)
        self.declare_parameter("safety_timeout_s", 2.0)
        self.declare_parameter("return_to_start", True)
        self.declare_parameter("land_on_return", True)
        self.declare_parameter("log_dir", "results/logs")

        self.grid = self._load_grid()
        self._lock = threading.RLock()
        self._state = "IDLE"
        self._message = "waiting for a goal"
        self._active = False
        self._success = False
        self._goal: Cell | None = None
        self._path: list[Cell] = []
        self._current_index = 0
        self._flight_status: FlightStatus | None = None
        self._latest_safety: SafetyStatus | None = None
        self._latest_safety_time = 0.0
        self._thread: threading.Thread | None = None
        self._abort_event = threading.Event()
        self._run_dir: Path | None = None
        self._log_file = None

        self._takeoff_timeout_s = float(
            self.get_parameter("takeoff_timeout_s").value
        )
        self._waypoint_timeout_s = float(
            self.get_parameter("waypoint_timeout_s").value
        )
        self._goal_hold_s = float(self.get_parameter("goal_hold_s").value)
        self._waypoint_tolerance_m = float(
            self.get_parameter("waypoint_tolerance_m").value
        )
        self._mission_timeout_s = float(
            self.get_parameter("mission_timeout_s").value
        )
        self._land_timeout_s = float(
            self.get_parameter("land_timeout_s").value
        )
        self._safety_timeout_s = float(
            self.get_parameter("safety_timeout_s").value
        )

        self._create_interfaces()

        self.get_logger().info(
            f"Grid navigation loaded '{self.grid.name}': "
            f"{self.grid.width}x{self.grid.height} cells, "
            f"cell={self.grid.cell_size_m:.1f} m, "
            f"start=({self.grid.start.x},{self.grid.start.y}), "
            f"buildings={len(self.grid.buildings)}, "
            f"occupied={len(self.grid.occupied)}."
        )
        self.get_logger().info(
            f"Service: {self.get_name()}/set_goal; "
            f"status: {self.get_parameter('status_topic').value}"
        )

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def _load_grid(self) -> GridMap:
        path = str(self.get_parameter("grid_file").value).strip()
        if not path:
            root = os.environ.get("AERIS_ROOT")
            if not root:
                raise GridValidationError(
                    "grid_file parameter is required "
                    "(or set AERIS_ROOT)"
                )
            path = os.path.join(
                root, "ros2_ws", "src", "aeris_bringup",
                "config", "grids", "city_grid.yaml",
            )
        try:
            return load_grid(path)
        except GridValidationError as exc:
            self.get_logger().error(f"Grid validation failed: {exc}")
            raise

    def _create_interfaces(self) -> None:
        self._flight_status_sub = self.create_subscription(
            FlightStatus,
            str(self.get_parameter("flight_status_topic").value),
            self._on_flight_status,
            10,
        )
        self._safety_sub = self.create_subscription(
            SafetyStatus,
            str(self.get_parameter("safety_topic").value),
            self._on_safety_status,
            10,
        )
        self._status_pub = self.create_publisher(
            GridStatus, str(self.get_parameter("status_topic").value), 10
        )
        self._goal_srv = self.create_service(
            GridGoal, "~/set_goal", self._on_set_goal
        )
        self._start_client = self.create_client(
            Trigger, str(self.get_parameter("start_service").value)
        )
        self._goto_client = self.create_client(
            GoTo, str(self.get_parameter("goto_service").value)
        )
        self._land_client = self.create_client(
            Trigger, str(self.get_parameter("land_service").value)
        )
        self._abort_client = self.create_client(
            Trigger, str(self.get_parameter("abort_service").value)
        )
        self._status_timer = self.create_timer(0.2, self._publish_status)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_flight_status(self, msg: FlightStatus) -> None:
        with self._lock:
            self._flight_status = msg

    def _on_safety_status(self, msg: SafetyStatus) -> None:
        with self._lock:
            self._latest_safety = msg
            self._latest_safety_time = time.monotonic()

    def _on_set_goal(
        self,
        request: GridGoal.Request,
        response: GridGoal.Response,
    ) -> GridGoal.Response:
        goal = Cell(int(request.x), int(request.y))

        with self._lock:
            if self._active:
                response.accepted = False
                response.message = "a grid mission is already active"
                return response
            if not self.grid.is_free(goal):
                response.accepted = False
                response.message = f"cell ({goal.x}, {goal.y}) is occupied"
                return response

            current = self._current_cell_locked()
            path = self.grid.plan(current, goal)
            if path is None:
                response.accepted = False
                response.message = (
                    f"no free path from ({current.x}, {current.y}) "
                    f"to ({goal.x}, {goal.y})"
                )
                return response

            self._goal = goal
            self._path = path
            self._current_index = 0
            self._active = True
            self._success = False
            self._abort_event.clear()
            self._state = "PREFLIGHT"
            self._message = "goal accepted"
            self._thread = None

        self._spawn_thread()

        response.accepted = True
        response.message = "goal accepted"
        response.path_cells = len(path)
        response.path_length_m = self.grid.path_length_m(path)
        return response

    # ------------------------------------------------------------------
    # Mission thread
    # ------------------------------------------------------------------
    def _spawn_thread(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def _run(self) -> None:
        goal = self._goal
        path = list(self._path)
        if goal is None:
            return

        self._open_run_log()
        deadline = time.monotonic() + self._mission_timeout_s
        try:
            self._log(
                "grid_loaded",
                grid=self.grid.name,
                width=self.grid.width,
                height=self.grid.height,
                cell_size_m=self.grid.cell_size_m,
                buildings=len(self.grid.buildings),
            )
            self._log("goal_selected", x=goal.x, y=goal.y)
            self._set_state("PREFLIGHT", "starting offboard controller")
            self._ensure_airborne(deadline)

            self._set_state(
                "TRANSIT", f"navigating to ({goal.x}, {goal.y})"
            )
            self._log(
                "path_planned",
                phase="outbound",
                cells=len(path),
                length_m=self.grid.path_length_m(path),
                path=[(cell.x, cell.y) for cell in path],
            )
            self._execute_path(path, "TRANSIT", deadline)

            self._set_state("AT_GOAL", f"goal ({goal.x}, {goal.y}) reached")
            self._log("goal_reached", x=goal.x, y=goal.y)
            self._sleep_abortable(self._goal_hold_s, deadline)

            if bool(self.get_parameter("return_to_start").value):
                return_path = self.grid.plan(goal, self.grid.start)
                if return_path is None:
                    raise RuntimeError("no free return path to start")
                with self._lock:
                    self._path = return_path
                    self._current_index = 0
                self._set_state("RETURNING", "returning to start")
                self._log(
                    "path_planned",
                    phase="return",
                    cells=len(return_path),
                    length_m=self.grid.path_length_m(return_path),
                    path=[(cell.x, cell.y) for cell in return_path],
                )
                self._execute_path(return_path, "RETURNING", deadline)
                self._log("returned_to_start")

            if bool(self.get_parameter("land_on_return").value):
                self._set_state("LANDING", "landing at start")
                self._log("landing")
                self._call_trigger(self._land_client, "land", 10.0)
                self._wait_flight(
                    lambda status: (
                        not status.armed
                        or status.state in ("DONE", "ABORTED")
                    ),
                    self._land_timeout_s,
                    "disarm after landing",
                    deadline,
                )

            self._set_state("DONE", "grid mission complete")
            with self._lock:
                self._success = True
            self._log("mission_complete")
        except SafetyAbort as exc:
            self.get_logger().error(f"Safety abort: {exc}")
            with self._lock:
                self._state = "ABORTED"
                self._success = False
                self._message = str(exc)
            self._log(
                "safety_abort",
                faults=exc.faults,
                action=exc.action,
                reason=str(exc),
            )
        except Exception as exc:  # noqa: BLE001 - thread reports all failures
            self.get_logger().error(f"Grid mission failed: {exc}")
            with self._lock:
                self._state = "ABORTED"
                self._success = False
                self._message = str(exc)
            self._log("abort", reason=str(exc))
            try:
                self._call_trigger(self._abort_client, "abort", 5.0)
            except Exception as abort_exc:  # noqa: BLE001
                self.get_logger().warning(
                    f"Abort service call failed: {abort_exc}"
                )
        finally:
            with self._lock:
                self._active = False
                self._thread = None
            self._close_run_log()

    # ------------------------------------------------------------------
    # Flight sequencing
    # ------------------------------------------------------------------
    def _ensure_airborne(self, deadline: float) -> None:
        with self._lock:
            status = self._flight_status
        if status is not None and status.armed and status.state == "HOLD":
            self._log("takeoff_confirmed", reused=True)
            return

        response = self._call_service(
            self._start_client, Trigger.Request(), 10.0, "start"
        )
        if not response.success:
            raise RuntimeError(f"offboard start rejected: {response.message}")

        self._wait_flight(
            lambda current: current.state == "HOLD" and current.armed,
            self._takeoff_timeout_s,
            "takeoff and HOLD",
            deadline,
        )
        self._log("takeoff_confirmed")

    def _execute_path(
        self, path: list[Cell], phase: str, deadline: float
    ) -> None:
        for index, cell in enumerate(path[1:], start=1):
            self._check(deadline)
            with self._lock:
                self._current_index = index
            self._log(
                "cell_transit",
                phase=phase,
                index=index,
                x=cell.x,
                y=cell.y,
                ned_x=self.grid.cell_to_ned(cell)[0],
                ned_y=self.grid.cell_to_ned(cell)[1],
            )
            self._send_goal(cell, deadline)
            self._log(
                "cell_reached", phase=phase, index=index, x=cell.x, y=cell.y
            )

    def _send_goal(self, cell: Cell, deadline: float) -> None:
        ned_x, ned_y = self.grid.cell_to_ned(cell)
        request = GoTo.Request()
        request.x = ned_x
        request.y = ned_y
        request.z = -self.grid.cruise_altitude_m
        request.yaw = math.nan
        request.tolerance_m = self._waypoint_tolerance_m
        request.timeout_s = self._waypoint_timeout_s
        response = self._call_service(
            self._goto_client, request, 10.0,
            f"go_to ({cell.x}, {cell.y})",
        )
        if not response.accepted:
            raise RuntimeError(
                f"go_to rejected for ({cell.x}, {cell.y}): {response.message}"
            )

        sequence = response.sequence
        self._wait_flight(
            lambda status, seq=sequence: (
                status.goal_sequence == seq
                and (
                    status.goal_active
                    or status.goal_reached
                    or status.goal_failed
                )
            ),
            5.0,
            f"goal acceptance for ({cell.x}, {cell.y})",
            deadline,
        )
        self._wait_flight(
            lambda status, seq=sequence: (
                status.goal_sequence == seq
                and (status.goal_reached or status.goal_failed)
            ),
            self._waypoint_timeout_s + 10.0,
            f"cell ({cell.x}, {cell.y})",
            deadline,
        )
        with self._lock:
            status = self._flight_status
        if status is None or status.goal_failed:
            raise RuntimeError(f"cell ({cell.x}, {cell.y}) was not reached")

    def _wait_flight(self, predicate, timeout: float, description: str,
                     mission_deadline: float):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._check(mission_deadline)
            with self._lock:
                status = self._flight_status
            if status is not None and predicate(status):
                return status
            time.sleep(0.05)
        raise TimeoutError(f"timed out waiting for {description}")

    def _call_service(self, client, request, timeout: float, name: str):
        self._check(float("inf"))
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f"service '{name}' unavailable")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            self._check(float("inf"))
            if time.monotonic() > deadline:
                raise TimeoutError(f"service '{name}' timed out")
            time.sleep(0.02)
        return future.result()

    def _call_trigger(self, client, name: str, timeout: float):
        response = self._call_service(
            client, Trigger.Request(), timeout, name
        )
        if not response.success:
            raise RuntimeError(
                f"service '{name}' rejected: {response.message}"
            )
        return response

    def _sleep_abortable(self, seconds: float, mission_deadline: float) -> None:
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end:
            self._check(mission_deadline)
            time.sleep(min(0.1, end - time.monotonic()))

    def _check(self, mission_deadline: float) -> None:
        if not rclpy.ok():
            raise RuntimeError("ROS shutdown")
        if self._abort_event.is_set():
            raise RuntimeError("grid mission abort requested")
        if time.monotonic() > mission_deadline:
            raise TimeoutError("grid mission timeout exceeded")

        with self._lock:
            safety = self._latest_safety
            safety_time = self._latest_safety_time

        if safety is not None:
            if not safety.safe and safety.armed:
                raise SafetyAbort(safety.active_faults, safety.action)
            if time.monotonic() - safety_time > self._safety_timeout_s:
                raise RuntimeError("safety supervisor heartbeat lost")

    # ------------------------------------------------------------------
    # State, status and logging
    # ------------------------------------------------------------------
    def _current_cell_locked(self) -> Cell:
        if self._flight_status is None:
            return self.grid.start
        return self.grid.ned_to_cell(
            float(self._flight_status.x), float(self._flight_status.y)
        )

    def _set_state(self, state: str, message: str) -> None:
        with self._lock:
            self._state = state
            self._message = message
        self.get_logger().info(f"[{state}] {message}")
        self._log("state_transition", state=state, message=message)

    def _publish_status(self) -> None:
        with self._lock:
            state = self._state
            message = self._message
            active = self._active
            success = self._success
            goal = self._goal
            path = list(self._path)
            current_index = self._current_index
            flight = self._flight_status

        msg = GridStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.grid_name = self.grid.name
        msg.state = state
        msg.active = active
        msg.success = success
        msg.message = message
        msg.start_x = self.grid.start.x
        msg.start_y = self.grid.start.y
        msg.goal_x = goal.x if goal is not None else -1
        msg.goal_y = goal.y if goal is not None else -1
        msg.current_x = self.grid.ned_to_cell(
            flight.x if flight is not None else 0.0,
            flight.y if flight is not None else 0.0,
        ).x
        msg.current_y = self.grid.ned_to_cell(
            flight.x if flight is not None else 0.0,
            flight.y if flight is not None else 0.0,
        ).y
        msg.path_x = [cell.x for cell in path]
        msg.path_y = [cell.y for cell in path]
        msg.current_index = current_index
        path_length = self.grid.path_length_m(path) if path else 0.0
        if not path:
            progress = 0.0
        elif len(path) == 1:
            progress = 1.0
        else:
            progress = min(1.0, current_index / (len(path) - 1))
        msg.progress = progress
        msg.path_length_m = path_length
        msg.remaining_m = max(0.0, path_length * (1.0 - progress))
        if flight is not None:
            msg.drone_x = float(flight.x)
            msg.drone_y = float(flight.y)
            msg.drone_z = float(flight.z)
        self._status_pub.publish(msg)

    def _open_run_log(self) -> None:
        log_dir = Path(str(self.get_parameter("log_dir").value))
        run_id = f"grid_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self._run_dir = log_dir / run_id
        self._run_dir.mkdir(parents=True, exist_ok=True)
        self._log_file = (self._run_dir / "grid.jsonl").open(
            "w", encoding="utf-8"
        )
        self.get_logger().info(
            f"Grid mission log: {self._run_dir / 'grid.jsonl'}"
        )

    def _close_run_log(self) -> None:
        if self._log_file is not None:
            self._log_file.flush()
            self._log_file.close()
            self._log_file = None

    def _log(self, event: str, **fields) -> None:
        if self._log_file is None:
            return
        record = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "event": event,
            "grid": self.grid.name,
        }
        record.update(fields)
        self._log_file.write(json.dumps(record) + "\n")
        self._log_file.flush()

    def destroy_node(self) -> bool:
        self._abort_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)
        self._close_run_log()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GridNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
