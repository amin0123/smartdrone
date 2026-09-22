"""Small immutable data objects, independent of rclpy and the simulator."""
from dataclasses import dataclass
from enum import Enum
import math

Point = tuple[float, float, float]


@dataclass(frozen=True)
class Vehicle:
    position: Point = (0., 0., 0.)
    velocity: Point = (0., 0., 0.)
    attitude_xyzw: tuple[float, float, float, float] = (0., 0., 0., 1.)
    connected: bool = False
    localized: bool = False
    armed: bool = False
    landed: bool = False
    offboard: bool = False
    failsafe: bool = False
    reset_counter: int = 0

    @property
    def speed(self) -> float:
        return math.dist(self.velocity, (0., 0., 0.))


class Severity(str, Enum):
    OK = 'OK'
    RETURN = 'RETURN'
    LAND = 'LAND'
    INHIBIT = 'INHIBIT'


@dataclass(frozen=True)
class Decision:
    severity: Severity
    reason: str


@dataclass(frozen=True)
class Target:
    position: Point
    yaw: float = 0.


def near(a: Point, b: Point, tolerance: float) -> bool:
    return math.dist(a, b) <= tolerance
