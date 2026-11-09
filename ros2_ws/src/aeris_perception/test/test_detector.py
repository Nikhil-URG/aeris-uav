"""Unit tests for the AERIS red-target detector."""

import cv2
import numpy as np

from aeris_perception.detector import (
    DetectorConfig,
    detect_red_target,
    pixel_to_local_ned,
)


def test_detects_red_rectangle():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (300, 200), (380, 280), (0, 0, 255), -1)

    candidates = detect_red_target(image, DetectorConfig(min_area_px=100))

    assert len(candidates) == 1
    candidate = candidates[0]
    assert 290 <= candidate.bbox_x <= 310
    assert 190 <= candidate.bbox_y <= 210
    assert 70 <= candidate.bbox_width <= 90
    assert 70 <= candidate.bbox_height <= 90
    assert candidate.confidence > 0.0


def test_blank_image_has_no_detection():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    assert detect_red_target(image, DetectorConfig(min_area_px=100)) == []


def test_small_red_mark_is_ignored():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(image, (10, 10), (14, 14), (0, 0, 255), -1)
    assert detect_red_target(image, DetectorConfig(min_area_px=100)) == []


def test_center_projection_matches_vehicle_position():
    x, y, z = pixel_to_local_ned(
        u=640.0,
        v=480.0,
        width=1280,
        height=960,
        hfov_rad=1.74,
        altitude_m=5.0,
        drone_x=4.0,
        drone_y=4.0,
        yaw_rad=0.0,
    )
    assert abs(x - 4.0) < 1e-6
    assert abs(y - 4.0) < 1e-6
    assert z == 0.0
