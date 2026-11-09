"""ROS 2 perception node for the AERIS simulated inspection camera."""

from __future__ import annotations

import math

from aeris_interfaces.msg import Detection, DetectionArray
from aeris_perception.detector import (
    DetectorConfig,
    bearing_from_pixel,
    detect_red_target,
    focal_from_hfov,
    pixel_to_local_ned,
    range_from_altitude,
)
from cv_bridge import CvBridge
from px4_msgs.msg import VehicleLocalPosition
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("aeris_perception")

        self.declare_parameter("image_topic", "/aeris/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/aeris/camera/camera_info")
        self.declare_parameter(
            "position_topic", "/fmu/out/vehicle_local_position_v1"
        )
        self.declare_parameter(
            "detections_topic", "/aeris/perception/detections"
        )
        self.declare_parameter("class_name", "red_target")
        self.declare_parameter("min_area_px", 500)
        self.declare_parameter("hfov_rad", 1.74)
        self.declare_parameter("log_every_n", 15)

        self._bridge = CvBridge()
        self._class_name = str(self.get_parameter("class_name").value)
        self._hfov_rad = float(self.get_parameter("hfov_rad").value)
        self._min_area = int(self.get_parameter("min_area_px").value)
        self._log_every_n = max(1, int(self.get_parameter("log_every_n").value))
        self._counter = 0

        self._fx = focal_from_hfov(1280, self._hfov_rad)
        self._fy = self._fx
        self._cx = 640.0
        self._cy = 480.0
        self._width = 1280
        self._height = 960
        self._have_camera_info = False

        self._position: VehicleLocalPosition | None = None

        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        position_topic = str(self.get_parameter("position_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)

        self._image_sub = self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self._position_sub = self.create_subscription(
            VehicleLocalPosition,
            position_topic,
            self._on_position,
            qos_profile_sensor_data,
        )
        self._detections_pub = self.create_publisher(
            DetectionArray, detections_topic, 10
        )

        self.get_logger().info(
            f"AERIS perception initialized: image='{image_topic}', "
            f"detections='{detections_topic}', class='{self._class_name}'."
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if msg.k[0] > 0.0:
            first = not self._have_camera_info
            self._fx = float(msg.k[0])
            self._fy = float(msg.k[4])
            self._cx = float(msg.k[2])
            self._cy = float(msg.k[5])
            self._width = int(msg.width)
            self._height = int(msg.height)
            self._have_camera_info = True
            if first:
                self.get_logger().info(
                    f"Camera info: {self._width}x{self._height} "
                    f"fx={self._fx:.1f} fy={self._fy:.1f} "
                    f"cx={self._cx:.1f} cy={self._cy:.1f}"
                )

    def _on_position(self, msg: VehicleLocalPosition) -> None:
        self._position = msg

    def _on_image(self, msg: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"cv_bridge conversion failed: {exc}")
            return

        if not self._have_camera_info:
            self._width = int(image.shape[1])
            self._height = int(image.shape[0])
            self._fx = focal_from_hfov(self._width, self._hfov_rad)
            self._fy = self._fx
            self._cx = float(self._width) / 2.0
            self._cy = float(self._height) / 2.0

        candidates = detect_red_target(
            image, DetectorConfig(min_area_px=self._min_area)
        )
        altitude = float("nan")
        drone_x = float("nan")
        drone_y = float("nan")
        yaw = 0.0
        if self._position is not None:
            altitude = -float(self._position.z)
            drone_x = float(self._position.x)
            drone_y = float(self._position.y)
            yaw = float(self._position.heading)

        output = DetectionArray()
        output.stamp = msg.header.stamp
        output.frame_id = "local_ned" if self._position is not None else "camera_link"

        for candidate in candidates:
            detection = Detection()
            detection.stamp = msg.header.stamp
            detection.target_id = self._class_name
            detection.class_name = self._class_name
            detection.confidence = float(candidate.confidence)
            detection.bbox_x = candidate.bbox_x
            detection.bbox_y = candidate.bbox_y
            detection.bbox_width = candidate.bbox_width
            detection.bbox_height = candidate.bbox_height
            detection.bearing_rad = bearing_from_pixel(
                candidate.center_x, self._cx, self._fx
            )
            detection.range_m = range_from_altitude(
                candidate.center_x,
                candidate.center_y,
                self._cx,
                self._cy,
                self._fx,
                altitude,
            )
            local_x, local_y, local_z = pixel_to_local_ned(
                candidate.center_x,
                candidate.center_y,
                self._width,
                self._height,
                self._hfov_rad,
                altitude,
                drone_x,
                drone_y,
                yaw,
            )
            detection.local_x = local_x
            detection.local_y = local_y
            detection.local_z = local_z
            output.detections.append(detection)

        self._detections_pub.publish(output)

        self._counter += 1
        if self._counter % self._log_every_n == 0 and candidates:
            best = candidates[0]
            self.get_logger().info(
                f"Detected {self._class_name}: conf={best.confidence:.2f} "
                f"bbox=({best.bbox_x},{best.bbox_y},{best.bbox_width},"
                f"{best.bbox_height}) altitude={altitude:.2f} m"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
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
