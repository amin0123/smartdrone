"""Replaceable RGB detector. Color classes are synthetic, not crop diagnoses."""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass(frozen=True)
class Blob:
    classification: str
    confidence: float
    center: tuple[float, float]
    box: tuple[int, int, int, int]


class ColorDetector:
    def __init__(self, healthy_lower=(35, 60, 35), healthy_upper=(95, 255, 255),
                 stressed_lower=(25, 80, 40), stressed_upper=(34, 255, 255),
                 min_area: int = 60, max_area_fraction: float = .25):
        self.ranges = {'healthy': (healthy_lower, healthy_upper),
                       'stressed': (stressed_lower, stressed_upper)}
        self.min_area, self.max_fraction = min_area, max_area_fraction

    def detect(self, rgb: np.ndarray) -> list[Blob]:
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError('Detector expects H×W×3 uint8 RGB')
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        output = []
        for label, (lower, upper) in self.ranges.items():
            mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if not self.min_area <= area <= rgb.shape[0] * rgb.shape[1] * self.max_fraction:
                    continue
                moments = cv2.moments(contour)
                if moments['m00'] <= 0:
                    continue
                x, y, w, h = cv2.boundingRect(contour)
                # Shape/mask confidence is a heuristic, never a disease probability.
                confidence = min(.99, .5 + .5 * area / (w * h))
                output.append(Blob(label, confidence,
                    (moments['m10']/moments['m00'], moments['m01']/moments['m00']), (x, y, w, h)))
        return output


class PlantTracker:
    """Associate approximate map positions without reading simulator truth labels."""
    def __init__(self, radius: float = .8):
        self.radius = radius
        self.positions: dict[str, np.ndarray] = {}

    def assign(self, position) -> str:
        p = np.asarray(position, dtype=float)
        if self.positions:
            nearest = min(self.positions, key=lambda key: np.linalg.norm(p-self.positions[key]))
            if np.linalg.norm(p-self.positions[nearest]) <= self.radius:
                self.positions[nearest] = .8 * self.positions[nearest] + .2 * p
                return nearest
        key = f'plant_{len(self.positions)+1:04d}'
        self.positions[key] = p
        return key
