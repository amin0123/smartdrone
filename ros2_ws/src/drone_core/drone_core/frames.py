"""Explicit REP-103 ENU/FLU ↔ PX4 NED/FRD transformations.

Quaternions in this library are XYZW. PX4's WXYZ layout is converted at the
transport boundary. A rotation maps body coordinates into its world frame.
"""
import math
from collections.abc import Sequence
import numpy as np
from scipy.spatial.transform import Rotation

ENU_FROM_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FLU_FROM_FRD = np.diag([1., -1., -1.])
# Downward USD camera with identity local orientation: right=+body X,
# image down=-body Y, optical forward=-body Z. See COORDINATE_FRAMES.md.
FLU_FROM_DOWN_OPTICAL = np.diag([1., -1., -1.])


def vector3(value: Sequence[float]) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError('Expected a finite three-dimensional vector')
    return result


def enu_to_ned(value: Sequence[float]) -> tuple[float, float, float]:
    return tuple(ENU_FROM_NED.T @ vector3(value))


def ned_to_enu(value: Sequence[float]) -> tuple[float, float, float]:
    return tuple(ENU_FROM_NED @ vector3(value))


def flu_to_frd(value: Sequence[float]) -> tuple[float, float, float]:
    return tuple(FLU_FROM_FRD.T @ vector3(value))


def frd_to_flu(value: Sequence[float]) -> tuple[float, float, float]:
    return tuple(FLU_FROM_FRD @ vector3(value))


def wrap_pi(angle: float) -> float:
    if not math.isfinite(angle):
        raise ValueError('Angle must be finite')
    return (angle + math.pi) % (2 * math.pi) - math.pi


def yaw_enu_to_ned(yaw: float) -> float:
    return wrap_pi(math.pi / 2 - yaw)


def yaw_ned_to_enu(yaw: float) -> float:
    return wrap_pi(math.pi / 2 - yaw)


def rotation_xyzw(q: Sequence[float]) -> Rotation:
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid quaternion')
    return Rotation.from_quat(q)


def attitude_ned_frd_to_enu_flu(q_xyzw: Sequence[float]) -> tuple[float, ...]:
    matrix = ENU_FROM_NED @ rotation_xyzw(q_xyzw).as_matrix() @ FLU_FROM_FRD.T
    return tuple(Rotation.from_matrix(matrix).as_quat())


def attitude_enu_flu_to_ned_frd(q_xyzw: Sequence[float]) -> tuple[float, ...]:
    matrix = ENU_FROM_NED.T @ rotation_xyzw(q_xyzw).as_matrix() @ FLU_FROM_FRD
    return tuple(Rotation.from_matrix(matrix).as_quat())
