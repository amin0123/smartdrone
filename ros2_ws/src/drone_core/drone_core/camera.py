"""Image-time pose association and calibrated downward-camera geometry."""
from bisect import bisect_left
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .frames import FLU_FROM_DOWN_OPTICAL, rotation_xyzw, vector3
from .models import Point


@dataclass(frozen=True)
class TimedPose:
    stamp: float
    position: Point
    quaternion: tuple[float, float, float, float]


class PoseBuffer:
    def __init__(self, horizon: float = 3.):
        self.horizon = horizon
        self.samples: list[TimedPose] = []

    def add(self, pose: TimedPose) -> None:
        if not math.isfinite(pose.stamp):
            raise ValueError('Invalid pose time')
        vector3(pose.position)
        rotation_xyzw(pose.quaternion)
        i = bisect_left([p.stamp for p in self.samples], pose.stamp)
        if i < len(self.samples) and self.samples[i].stamp == pose.stamp:
            self.samples[i] = pose
        else:
            self.samples.insert(i, pose)
        newest = self.samples[-1].stamp
        self.samples = [p for p in self.samples if newest - p.stamp <= self.horizon]

    def at(self, stamp: float, tolerance: float = .2) -> TimedPose | None:
        if not self.samples or not math.isfinite(stamp):
            return None
        i = bisect_left([p.stamp for p in self.samples], stamp)
        nearest = min(self.samples[max(0, i-1):i+1], key=lambda p: abs(p.stamp-stamp))
        if abs(nearest.stamp - stamp) > tolerance:
            return None
        if 0 < i < len(self.samples):
            a, b = self.samples[i-1:i+1]
            if max(stamp-a.stamp, b.stamp-stamp) <= tolerance:
                alpha = (stamp-a.stamp) / (b.stamp-a.stamp)
                p = tuple((1-alpha) * x + alpha*y for x, y in zip(a.position, b.position))
                q = Slerp([a.stamp, b.stamp], Rotation.from_quat([a.quaternion, b.quaternion]))([stamp]).as_quat()[0]
                return TimedPose(stamp, p, tuple(q))
        return nearest


def optical_point_to_map(point, pose: TimedPose, camera_translation=(0., 0., -.04)) -> Point:
    body_point = vector3(camera_translation) + FLU_FROM_DOWN_OPTICAL @ vector3(point)
    return tuple(vector3(pose.position) + rotation_xyzw(pose.quaternion).apply(body_point))


def ground_intersection(pixel: tuple[float, float], intrinsics, pose: TimedPose,
                        plane_z: float, camera_translation=(0., 0., -.04),
                        max_range: float = 30.) -> Point | None:
    k = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('Invalid camera intrinsics')
    ray_optical = np.linalg.solve(k, [pixel[0], pixel[1], 1.])
    body_rotation = rotation_xyzw(pose.quaternion)
    origin = vector3(pose.position) + body_rotation.apply(vector3(camera_translation))
    direction = body_rotation.apply(FLU_FROM_DOWN_OPTICAL @ ray_optical)
    direction /= np.linalg.norm(direction)
    if not np.isfinite(direction).all() or direction[2] >= -1e-6:
        return None
    distance = (plane_z - origin[2]) / direction[2]
    if not 0 < distance <= max_range:
        return None
    return tuple(origin + distance * direction)
