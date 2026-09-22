"""Dock contact and bounded marker-guided descent in ENU."""
from dataclasses import dataclass
import math
from drone_core.models import Point, Vehicle


def dock_contact(vehicle: Vehicle, station: Point, radius: float = .65,
                 height_tolerance: float = .25) -> bool:
    return (vehicle.connected and vehicle.localized and vehicle.landed and not vehicle.armed and
            math.dist(vehicle.position[:2], station[:2]) <= radius and
            abs(vehicle.position[2] - station[2]) <= height_tolerance)


@dataclass(frozen=True)
class DockCommand:
    target: Point
    land: bool
    source: str


def descent_step(vehicle: Vehicle, known_station: Point, marker_station: Point | None,
                 marker_age: float, dt: float, *, use_marker: bool,
                 marker_timeout: float = 2., tolerance: float = .18,
                 descent_speed: float = .25, land_height: float = .6,
                 reference_z: float | None = None, max_vertical_lead: float = .4) -> DockCommand:
    if not vehicle.localized or not math.isfinite(dt) or dt < 0:
        raise ValueError('Descent needs valid localization and elapsed time')
    station, source = known_station, 'known_station'
    if use_marker and marker_station is not None and 0 <= marker_age <= marker_timeout:
        if all(math.isfinite(v) for v in marker_station) and math.dist(marker_station, known_station) < 1.:
            station, source = marker_station, 'visual_marker'
    aligned = math.dist(vehicle.position[:2], station[:2]) <= tolerance
    height = vehicle.position[2] - known_station[2]
    # A stale or rejected marker falls back to the established Stage-1 station
    # approach; it never commands a descent toward an arbitrary marker jump.
    reference_z = vehicle.position[2] if reference_z is None else reference_z
    target_z = max(known_station[2] + land_height,
                   vehicle.position[2] - max_vertical_lead,
                   reference_z - descent_speed * min(dt, .2)) if aligned else vehicle.position[2]
    return DockCommand((station[0], station[1], target_z), aligned and height <= land_height + .1, source)
