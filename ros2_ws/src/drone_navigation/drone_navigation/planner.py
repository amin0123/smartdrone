"""Deterministic coverage planning and collision checks, never flight control."""
from dataclasses import dataclass
import math
from collections.abc import Sequence
from drone_core.models import Point


@dataclass(frozen=True)
class Area:
    origin_x: float = 4.
    origin_y: float = -3.
    length: float = 8.
    width: float = 6.
    row_spacing: float = 3.
    altitude: float = 3.
    speed: float = 1.

    def __post_init__(self) -> None:
        values = (self.origin_x, self.origin_y, self.length, self.width,
                  self.row_spacing, self.altitude, self.speed)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Area parameters must be finite')
        if min(self.length, self.width, self.row_spacing, self.altitude, self.speed) <= 0:
            raise ValueError('Dimensions, spacing, altitude and speed must be positive')
        if self.width / self.row_spacing > 10000:
            raise ValueError('Route is too dense; check units')


def lawnmower(area: Area) -> list[Point]:
    """Cover both boundaries, including a final row when width is not divisible."""
    rows = [i * area.row_spacing for i in range(math.floor(area.width / area.row_spacing) + 1)]
    if not math.isclose(rows[-1], area.width):
        rows.append(area.width)
    result: list[Point] = []
    for index, row in enumerate(rows):
        ends = (0., area.length) if index % 2 == 0 else (area.length, 0.)
        result.extend((area.origin_x + x, area.origin_y + row, area.altitude) for x in ends)
    return result


@dataclass(frozen=True)
class Box:
    minimum: Point
    maximum: Point

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) for v in (*self.minimum, *self.maximum)):
            raise ValueError('Box must be finite')
        if any(a >= b for a, b in zip(self.minimum, self.maximum)):
            raise ValueError('Box minimum must be below maximum')

    def contains(self, point: Point, margin: float = 0.) -> bool:
        return all(a + margin <= p <= b - margin for p, a, b in zip(point, self.minimum, self.maximum))

    def intersects_segment(self, start: Point, end: Point, margin: float = 0.) -> bool:
        """Slab intersection against a vehicle-radius-inflated obstacle."""
        low, high = 0., 1.
        for a, b, minimum, maximum in zip(start, end, self.minimum, self.maximum):
            minimum, maximum = minimum - margin, maximum + margin
            delta = b - a
            if abs(delta) < 1e-12:
                if a < minimum or a > maximum:
                    return False
            else:
                t0, t1 = sorted(((minimum - a) / delta, (maximum - a) / delta))
                low, high = max(low, t0), min(high, t1)
                if low > high:
                    return False
        return True


def validate_route(points: Sequence[Point], fence: Box, obstacles: Sequence[Box], radius: float) -> None:
    if len(points) < 2 or radius < 0 or not math.isfinite(radius):
        raise ValueError('Invalid route or vehicle radius')
    if not all(fence.contains(p) for p in points):
        raise ValueError('Route leaves geofence')
    for start, end in zip(points, points[1:]):
        if any(box.intersects_segment(start, end, radius) for box in obstacles):
            raise ValueError(f'Blocked segment {start} → {end}')


class RouteFollower:
    """Generate speed-limited position references, bounded ahead of the aircraft.

    PX4 remains responsible for tracking and acceleration limits. Never skip a
    corner merely because elapsed wall time has advanced past it.
    """
    def __init__(self, start: Point, waypoints: Sequence[Point], speed: float,
                 tolerance: float = .35, max_lead: float = 1.5):
        if not waypoints or not all(math.isfinite(v) for p in [start, *waypoints] for v in p):
            raise ValueError('A finite route is required')
        if min(speed, tolerance, max_lead) <= 0 or tolerance >= max_lead:
            raise ValueError('Invalid tracking limits')
        self.waypoints = list(waypoints)
        self.reference = start
        self.speed, self.tolerance, self.max_lead = speed, tolerance, max_lead
        self.index = 0

    @property
    def complete(self) -> bool:
        return self.index >= len(self.waypoints)

    def step(self, actual: Point, dt: float) -> Point:
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('Invalid elapsed time')
        if self.complete:
            return self.reference
        destination = self.waypoints[self.index]
        remaining = math.dist(self.reference, destination)
        # Cap jumps after executor stalls; freshness handling belongs to safety.
        step = min(remaining, self.speed * min(dt, .2))
        fraction = step / remaining if remaining > 1e-9 else 0.
        candidate = tuple(a + fraction * (b - a) for a, b in zip(self.reference, destination))
        if math.dist(candidate, actual) <= self.max_lead:
            self.reference = candidate
        if math.dist(self.reference, destination) < 1e-6 and math.dist(actual, destination) <= self.tolerance:
            self.index += 1
        return self.reference
