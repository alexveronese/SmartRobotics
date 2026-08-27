"""OpenCV board localisation and red-X / blue-O classification."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    board: Tuple[str, ...]
    bounds: Tuple[int, int, int, int]
    annotated: np.ndarray


def _largest_board_bounds(hsv: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
    # The board is deliberately low-saturation and bright; the table is brown.
    # Keep the cream board while rejecting the light-gray Gazebo background.
    mask = cv2.inRange(hsv, np.array((0, 0, 190)), np.array((179, 75, 255)))
    # Close over the deliberately thick grid lines so nine cells become one
    # board contour before occupancy is classified separately.
    kernel = np.ones((15, 15), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = hsv.shape[:2]
    candidates = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area_ratio = (w * h) / float(width * height)
        aspect = w / float(h)
        if 0.04 < area_ratio < 0.65 and 0.72 < aspect < 1.38:
            candidates.append((w * h, (x, y, w, h)))
    return max(candidates, default=(0, None), key=lambda item: item[0])[1]


def detect_board(image_bgr: np.ndarray, min_colour_ratio: float = 0.018) -> Optional[Detection]:
    if image_bgr is None or image_bgr.ndim != 3:
        return None
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    bounds = _largest_board_bounds(hsv)
    if bounds is None:
        return None
    x, y, w, h = bounds

    red_a = cv2.inRange(hsv, np.array((0, 100, 80)), np.array((12, 255, 255)))
    red_b = cv2.inRange(hsv, np.array((168, 100, 80)), np.array((179, 255, 255)))
    red = cv2.bitwise_or(red_a, red_b)
    blue = cv2.inRange(hsv, np.array((92, 90, 65)), np.array((135, 255, 255)))

    annotated = image_bgr.copy()
    cells = []
    for row in range(3):
        for col in range(3):
            # Ignore cell borders/grid lines and inspect the central 70%.
            x0 = int(x + (col + 0.15) * w / 3.0)
            x1 = int(x + (col + 0.85) * w / 3.0)
            y0 = int(y + (row + 0.15) * h / 3.0)
            y1 = int(y + (row + 0.85) * h / 3.0)
            pixels = max(1, (x1 - x0) * (y1 - y0))
            red_ratio = cv2.countNonZero(red[y0:y1, x0:x1]) / pixels
            blue_ratio = cv2.countNonZero(blue[y0:y1, x0:x1]) / pixels
            value = "x" if red_ratio > min_colour_ratio and red_ratio > blue_ratio else ""
            if blue_ratio > min_colour_ratio and blue_ratio > red_ratio:
                value = "o"
            cells.append(value)
            cv2.rectangle(annotated, (x0, y0), (x1, y1), (80, 220, 80), 1)
            cv2.putText(
                annotated, value.upper() or str(row * 3 + col + 1),
                (x0 + 5, y0 + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 255), 2, cv2.LINE_AA,
            )
    cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 255), 2)
    return Detection(tuple(cells), bounds, annotated)
