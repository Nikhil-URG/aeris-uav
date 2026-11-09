"""OpenCV red-target detection and nadir ground projection."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectorConfig:
    min_area_px: int = 500
    hsv_red_lower1: tuple[int, int, int] = (0, 120, 70)
    hsv_red_upper1: tuple[int, int, int] = (10, 255, 255)
    hsv_red_lower2: tuple[int, int, int] = (170, 120, 70)
    hsv_red_upper2: tuple[int, int, int] = (180, 255, 255)
    morphology_kernel: int = 5


@dataclass(frozen=True)
class Candidate:
    bbox_x: int
    bbox_y: int
    bbox_width: int
    bbox_height: int
    center_x: float
    center_y: float
    area_px: float
    confidence: float


def detect_red_target(
    image_bgr: np.ndarray,
    config: DetectorConfig | None = None,
) -> list[Candidate]:
    """Return red target candidates, largest first."""
    if config is None:
        config = DetectorConfig()
    if image_bgr is None or image_bgr.ndim != 3:
        return []

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, config.hsv_red_lower1, config.hsv_red_upper1)
    mask |= cv2.inRange(hsv, config.hsv_red_lower2, config.hsv_red_upper2)

    kernel_size = max(1, int(config.morphology_kernel))
    if kernel_size > 1:
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    candidates: list[Candidate] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < config.min_area_px:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            continue
        candidates.append(
            Candidate(
                bbox_x=int(x),
                bbox_y=int(y),
                bbox_width=int(w),
                bbox_height=int(h),
                center_x=float(x) + float(w) / 2.0,
                center_y=float(y) + float(h) / 2.0,
                area_px=area,
                confidence=float(min(1.0, area / 5000.0)),
            )
        )

    candidates.sort(key=lambda c: c.area_px, reverse=True)
    return candidates


def focal_from_hfov(width: int, hfov_rad: float) -> float:
    if width <= 0 or not 0.0 < hfov_rad < math.pi:
        return float("nan")
    return (float(width) / 2.0) / math.tan(float(hfov_rad) / 2.0)


def bearing_from_pixel(u: float, cx: float, fx: float) -> float:
    if not math.isfinite(fx) or fx <= 0.0:
        return float("nan")
    return math.atan2(float(u) - float(cx), float(fx))


def range_from_altitude(
    u: float,
    v: float,
    cx: float,
    cy: float,
    fx: float,
    altitude_m: float,
) -> float:
    if altitude_m <= 0.2 or not math.isfinite(fx) or fx <= 0.0:
        return float("nan")
    radial = math.hypot(float(u) - float(cx), float(v) - float(cy))
    theta = math.atan2(radial, float(fx))
    return float(altitude_m) / max(0.1, math.cos(theta))


def pixel_to_local_ned(
    u: float,
    v: float,
    width: int,
    height: int,
    hfov_rad: float,
    altitude_m: float,
    drone_x: float,
    drone_y: float,
    yaw_rad: float,
) -> tuple[float, float, float]:
    """Approximate ground-plane projection for the nadir camera.

    The X500 mono camera points down. This uses a small-angle/pinhole ground
    intersection at the current altitude and rotates the body-frame offset
    into local NED using the vehicle yaw. It is intended for the simulated
    inspection target and should be calibrated for other camera mounts.
    """
    if width <= 0 or height <= 0 or altitude_m <= 0.2:
        return (float("nan"), float("nan"), float("nan"))
    fx = focal_from_hfov(width, hfov_rad)
    if not math.isfinite(fx) or fx <= 0.0:
        return (float("nan"), float("nan"), float("nan"))

    cx = float(width) / 2.0
    cy = float(height) / 2.0
    x_cam = (float(u) - cx) / fx
    y_cam = (float(v) - cy) / fx

    # Gazebo camera: image right corresponds to body right (negative body-left),
    # image down corresponds to body backward for the downward-mounted camera.
    forward = -y_cam * float(altitude_m)
    left = -x_cam * float(altitude_m)

    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    north = forward * cos_yaw - left * sin_yaw
    east = forward * sin_yaw + left * cos_yaw

    return (float(drone_x) + north, float(drone_y) + east, 0.0)
