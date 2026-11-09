"""ROS 2 mission manager for AERIS.

The node owns mission-level state and sequencing. It never publishes PX4
topics directly; all flight authority remains in aeris_flight_control.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import threading
import time

from aeris_interfaces.msg import FlightStatus, MissionStatus
from aeris_interfaces.srv import GoTo
from aeris_mission_manager.mission import Mission, MissionValidationError, load_mission
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__("aeris_mission_manager")

        self.declare_parameter("mission_file", "")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("auto_start_delay_s", 5.0)
        self.declare_parameter("log_dir", "results/logs")
        self.declare_parameter("flight_status_topic",
                               "/aeris_offboard_controller/flight_status")
        self.declare_parameter("start_service",
                               "/aeris_offboard_controller/start")
        self.declare_parameter("goto_service",
                               "/aeris_offboard_controller/go_to")
        self.declare_parameter("land_service",
                               "/aeris_offboard_controller/land")
        self.declare_parameter("abort_service",
                               "/aeris_offboard_controller/abort")
        self.declare_parameter("takeoff_timeout_s", 60.0)
        self.declare_parameter("waypoint_timeout_s", 45.0)
        self.declare_parameter("waypoint_accept_timeout_s", 5.0)
        self.declare_parameter("land_timeout_s", 60.0)

        self._lock = threading.RLock()
        self._state = "IDLE"
        self._current_waypoint = ""
        self._waypoints_completed = 0
        self._message = ""
        self._active = False
        self._success = False
        self._flight_status: FlightStatus | None = None
        self._mission: Mission | None = None
        self._mission_error = ""
        self._run_dir: Path | None = None
        self._log_file = None
        self._start_event = threading.Event()
        self._abort_event = threading.Event()
        self._mission_thread: threading.Thread | None = None

        self._load_mission()
        self._create_interfaces()

        self._status_timer = self.create_timer(0.5, self._publish_status)

        if self._mission is not None and bool(
            self.get_parameter("auto_start").value
        ):
            delay = float(self.get_parameter("auto_start_delay_s").value)
            timer = threading.Timer(delay, self._auto_start)
            timer.daemon = True
            timer.start()

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def _load_mission(self) -> None:
        path = str(self.get_parameter("mission_file").value).strip()
        if not path:
            self._mission_error = "mission_file is empty"
            self.get_logger().warning(
                "No mission_file supplied; mission manager will stay idle."
            )
            return

        try:
            self._mission = load_mission(path)
        except MissionValidationError as exc:
            self._mission_error = str(exc)
            self.get_logger().error(f"Mission validation failed: {exc}")
            return

        self.get_logger().info(
            f"Loaded mission '{self._mission.name}' with "
            f"{len(self._mission.waypoints)} waypoints from {path}"
        )
        self._message = "mission loaded"

    def _create_interfaces(self) -> None:
        self._flight_status_topic = str(
            self.get_parameter("flight_status_topic").value
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

        self._flight_status_sub = self.create_subscription(
            FlightStatus,
            self._flight_status_topic,
            self._on_flight_status,
            10,
        )
        self._status_pub = self.create_publisher(
            MissionStatus, "/aeris_mission_manager/status", 10
        )
        self._start_srv = self.create_service(
            Trigger, "~/start", self._on_start_service
        )
        self._abort_srv = self.create_service(
            Trigger, "~/abort", self._on_abort_service
        )

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------
    def _on_start_service(self, request, response):
        del request
        if self._mission is None:
            response.success = False
            response.message = f"no valid mission: {self._mission_error}"
            return response

        with self._lock:
            if self._active:
                response.success = False
                response.message = "mission already active"
                return response
            self._active = True
            self._success = False
            self._state = "PREFLIGHT"
            self._message = "start accepted"
            self._abort_event.clear()
            self._start_event.set()

        self._spawn_mission_thread()
        response.success = True
        response.message = "mission start accepted"
        return response

    def _on_abort_service(self, request, response):
        del request
        self._abort_event.set()
        response.success = True
        response.message = "mission abort requested"
        return response

    def _auto_start(self) -> None:
        if not rclpy.ok():
            return
        with self._lock:
            if self._active or self._mission is None:
                return
            self._active = True
            self._success = False
            self._state = "PREFLIGHT"
            self._message = "auto-start"
            self._abort_event.clear()
            self._start_event.set()
        self._spawn_mission_thread()

    def _spawn_mission_thread(self) -> None:
        if self._mission_thread is None or not self._mission_thread.is_alive():
            self._mission_thread = threading.Thread(
                target=self._run_mission, daemon=True
            )
            self._mission_thread.start()

    # ------------------------------------------------------------------
    # Flight status / mission status
    # ------------------------------------------------------------------
    def _on_flight_status(self, msg: FlightStatus) -> None:
        with self._lock:
            self._flight_status = msg

    def _publish_status(self) -> None:
        msg = MissionStatus()
        msg.stamp = self.get_clock().now().to_msg()
        with self._lock:
            mission = self._mission
            msg.mission_name = mission.name if mission else ""
            msg.state = self._state
            msg.current_waypoint = self._current_waypoint
            msg.waypoints_total = len(mission.waypoints) if mission else 0
            msg.waypoints_completed = self._waypoints_completed
            msg.active = self._active
            msg.success = self._success
            msg.message = self._message
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Mission thread
    # ------------------------------------------------------------------
    def _run_mission(self) -> None:
        mission = self._mission
        if mission is None:
            return

        self._start_event.wait()
        if not rclpy.ok():
            return

        self._open_run_log(mission)
        deadline = time.monotonic() + mission.mission_timeout_s

        try:
            self._execute_mission(mission, deadline)
        except Exception as exc:  # noqa: BLE001 - mission thread reports all failures
            self.get_logger().error(f"Mission failed: {exc}")
            self._abort_mission(mission, str(exc))
        finally:
            with self._lock:
                self._active = False
                if self._state not in ("COMPLETE", "ABORTED"):
                    self._state = "ABORTED"
                    self._success = False
            self._close_run_log()

    def _execute_mission(self, mission: Mission, deadline: float) -> None:
        self._log("mission_start", mission_file=str(
            self.get_parameter("mission_file").value
        ))

        self._set_state("PREFLIGHT", "starting offboard controller")
        response = self._call_service(
            self._start_client, Trigger.Request(), 10.0, "start"
        )
        if not response.success:
            raise RuntimeError(f"offboard start rejected: {response.message}")

        self._set_state("TAKEOFF", "waiting for takeoff and HOLD")
        self._wait_for_flight(
            lambda status: status.state == "HOLD" and status.armed,
            float(self.get_parameter("takeoff_timeout_s").value),
            "takeoff and HOLD",
            deadline,
        )
        self._log("takeoff_confirmed")
        self._message = "airborne"

        for index, waypoint in enumerate(mission.waypoints):
            self._check(deadline)
            self._set_waypoint(waypoint.id)
            self._set_state(
                "TRANSIT", f"transiting to {waypoint.id} ({waypoint.action})"
            )
            self._log(
                "waypoint_transit",
                index=index,
                id=waypoint.id,
                x=waypoint.x,
                y=waypoint.y,
                z=waypoint.z,
                hold_s=waypoint.hold_s,
                action=waypoint.action,
                target_id=waypoint.target_id,
            )

            request = GoTo.Request()
            request.x = waypoint.x
            request.y = waypoint.y
            request.z = waypoint.z
            request.yaw = math.nan
            request.tolerance_m = waypoint.tolerance_m
            request.timeout_s = float(
                self.get_parameter("waypoint_timeout_s").value
            )
            response = self._call_service(
                self._goto_client, request, 10.0, "go_to"
            )
            if not response.accepted:
                raise RuntimeError(
                    f"go_to rejected for {waypoint.id}: {response.message}"
                )

            sequence = response.sequence
            self._wait_for_flight(
                lambda status, seq=sequence: (
                    status.goal_sequence == seq
                    and (
                        status.goal_active
                        or status.goal_reached
                        or status.goal_failed
                    )
                ),
                float(self.get_parameter("waypoint_accept_timeout_s").value),
                f"goal acceptance for {waypoint.id}",
                deadline,
            )
            self._wait_for_flight(
                lambda status, seq=sequence: (
                    status.goal_sequence == seq
                    and (status.goal_reached or status.goal_failed)
                ),
                float(self.get_parameter("waypoint_timeout_s").value) + 10.0,
                f"waypoint {waypoint.id}",
                deadline,
            )

            with self._lock:
                status = self._flight_status
            if status is None or status.goal_failed:
                raise RuntimeError(f"waypoint {waypoint.id} was not reached")

            self._set_state("INSPECT", f"inspecting {waypoint.id}")
            self._log("waypoint_reached", index=index, id=waypoint.id)
            self._sleep_abortable(waypoint.hold_s, deadline)
            self._log(
                "inspection_complete",
                index=index,
                id=waypoint.id,
                target_id=waypoint.target_id,
            )
            with self._lock:
                self._waypoints_completed += 1

        if mission.return_to_launch:
            self._check(deadline)
            self._set_waypoint("rtl")
            self._set_state("TRANSIT", "returning to launch")
            self._log("rtl_transit", x=mission.home_x, y=mission.home_y,
                      z=-mission.cruise_altitude_m)
            request = GoTo.Request()
            request.x = mission.home_x
            request.y = mission.home_y
            request.z = -mission.cruise_altitude_m
            request.yaw = math.nan
            request.tolerance_m = mission.waypoint_tolerance_m
            request.timeout_s = float(
                self.get_parameter("waypoint_timeout_s").value
            )
            response = self._call_service(
                self._goto_client, request, 10.0, "go_to (RTL)"
            )
            if not response.accepted:
                raise RuntimeError(f"RTL rejected: {response.message}")
            sequence = response.sequence
            self._wait_for_flight(
                lambda status, seq=sequence: (
                    status.goal_sequence == seq
                    and (
                        status.goal_active
                        or status.goal_reached
                        or status.goal_failed
                    )
                ),
                float(self.get_parameter("waypoint_accept_timeout_s").value),
                "RTL goal acceptance",
                deadline,
            )
            self._wait_for_flight(
                lambda status, seq=sequence: (
                    status.goal_sequence == seq
                    and (status.goal_reached or status.goal_failed)
                ),
                float(self.get_parameter("waypoint_timeout_s").value) + 10.0,
                "RTL waypoint",
                deadline,
            )
            with self._lock:
                status = self._flight_status
            if status is None or status.goal_failed:
                raise RuntimeError("RTL waypoint was not reached")
            self._log("rtl_reached")

        self._set_waypoint("")
        self._set_state("LAND", "landing at current position")
        self._log("landing")
        response = self._call_service(
            self._land_client, Trigger.Request(), 10.0, "land"
        )
        if not response.success:
            raise RuntimeError(f"landing rejected: {response.message}")

        self._wait_for_flight(
            lambda status: (
                not status.armed or status.state in ("DONE", "ABORTED")
            ),
            float(self.get_parameter("land_timeout_s").value),
            "disarm after landing",
            deadline,
        )
        self._set_state("COMPLETE", "mission complete")
        with self._lock:
            self._success = True
        self._log("mission_complete")

    def _abort_mission(self, mission: Mission, reason: str) -> None:
        with self._lock:
            self._state = "ABORTED"
            self._success = False
            self._message = reason
            self._current_waypoint = ""
        self._log("mission_abort", reason=reason)
        try:
            self._abort_event.set()
            self._call_service(
                self._abort_client, Trigger.Request(), 5.0, "abort"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Abort service call failed: {exc}")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _set_state(self, state: str, message: str) -> None:
        with self._lock:
            self._state = state
            self._message = message
        self.get_logger().info(f"[{state}] {message}")
        self._log("state_transition", state=state, message=message)

    def _set_waypoint(self, waypoint_id: str) -> None:
        with self._lock:
            self._current_waypoint = waypoint_id

    def _wait_for_flight(self, predicate, timeout: float, description: str,
                         mission_deadline: float) -> FlightStatus:
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

    def _sleep_abortable(self, seconds: float, mission_deadline: float) -> None:
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end:
            self._check(mission_deadline)
            time.sleep(min(0.1, end - time.monotonic()))

    def _check(self, mission_deadline: float) -> None:
        if not rclpy.ok():
            raise RuntimeError("ROS shutdown")
        if self._abort_event.is_set():
            raise RuntimeError("mission abort requested")
        if time.monotonic() > mission_deadline:
            raise TimeoutError("mission timeout exceeded")

    def _open_run_log(self, mission: Mission) -> None:
        log_dir = Path(str(self.get_parameter("log_dir").value))
        run_id = f"{mission.name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self._run_dir = log_dir / run_id
        self._run_dir.mkdir(parents=True, exist_ok=True)
        self._log_file = (self._run_dir / "mission.jsonl").open(
            "w", encoding="utf-8"
        )
        self.get_logger().info(f"Mission log: {self._run_dir / 'mission.jsonl'}")
        self._log(
            "mission_loaded",
            name=mission.name,
            waypoints=len(mission.waypoints),
            mission_timeout_s=mission.mission_timeout_s,
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
        }
        mission = self._mission
        if mission is not None:
            record["mission"] = mission.name
        record.update(fields)
        self._log_file.write(json.dumps(record) + "\n")
        self._log_file.flush()

    def destroy_node(self) -> bool:
        self._abort_event.set()
        thread = self._mission_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)
        self._close_run_log()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
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
