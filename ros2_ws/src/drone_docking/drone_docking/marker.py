"""Metric ArUco marker pose; all camera/world conversion lives in drone_core."""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass(frozen=True)
class MarkerPose:
    translation_optical: tuple[float, float, float]
    reprojection_error: float


class MarkerDetector:
    def __init__(self, marker_id: int = 7, size: float = .8, max_error: float = 2.):
        if not 0 <= marker_id < 250 or not np.isfinite(size) or size <= 0:
            raise ValueError('Invalid marker id or physical size')
        self.marker_id, self.size, self.max_error = marker_id, size, max_error
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250))
        h = size/2
        self.points = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    def detect(self, rgb, intrinsics, distortion=None) -> MarkerPose | None:
        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        if ids is None:
            return None
        distortion = np.zeros(5) if distortion is None else distortion
        for points, marker_id in zip(corners, ids.flatten()):
            if marker_id != self.marker_id:
                continue
            pixels = points.reshape(4, 2).astype(np.float64)
            ok, rotation, translation = cv2.solvePnP(self.points, pixels, intrinsics, distortion,
                                                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or not np.isfinite(translation).all() or translation[2, 0] <= 0:
                continue
            projected, _ = cv2.projectPoints(self.points, rotation, translation, intrinsics, distortion)
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2)-pixels)**2, axis=1))))
            if error <= self.max_error:
                return MarkerPose(tuple(map(float, translation.flatten())), error)
        return None
