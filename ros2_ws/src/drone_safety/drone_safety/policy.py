"""Independent safety policy. Native PX4 failsafes remain below this layer."""
from dataclasses import dataclass
import math
from drone_core.models import Decision, Severity, Vehicle
from .battery import BatteryPolicy


@dataclass(frozen=True)
class SafetyConfig:
    minimum: tuple[float, float, float] = (-5., -6., -.5)
    maximum: tuple[float, float, float] = (17., 6., 8.)
    state_timeout: float = 1.
    battery_timeout: float = 2.
    mission_timeout: float = 300.
    controller_timeout: float = 1.
    perception_timeout: float = 3.

    def __post_init__(self):
        if not all(math.isfinite(x) for x in (*self.minimum, *self.maximum)):
            raise ValueError('Geofence must be finite')
        if any(a >= b for a, b in zip(self.minimum, self.maximum)):
            raise ValueError('Geofence bounds are reversed')
        if min(self.state_timeout, self.battery_timeout, self.mission_timeout,
               self.controller_timeout, self.perception_timeout) <= 0:
            raise ValueError('Safety timeouts must be positive')


@dataclass(frozen=True)
class SafetyInput:
    vehicle: Vehicle
    battery_percent: float
    phase: str
    state_age: float = 0.
    battery_age: float = 0.
    mission_age: float = 0.
    control_age: float = 0.
    perception_age: float = 0.
    recorder_age: float = 0.
    elapsed: float = 0.
    return_seconds: float = 0.


class SafetyMonitor:
    def __init__(self, config: SafetyConfig, battery: BatteryPolicy):
        self.config, self.battery = config, battery
        self._origin_revision: int | None = None
        self._latched: Decision | None = None

    def evaluate(self, data: SafetyInput) -> Decision:
        v, c = data.vehicle, self.config
        airborne = v.armed or not v.landed
        preflight = data.phase in {'IDLE', 'READY', 'PRE_FLIGHT_CHECK', 'MISSION_ABORTED'} and not v.armed
        if preflight:
            self._latched = None
            self._origin_revision = v.reset_counter

        def fault(reason: str) -> Decision:
            return Decision(Severity.INHIBIT if preflight else Severity.LAND, reason)

        decision = Decision(Severity.OK, 'all_monitors_healthy')
        if data.state_age > c.state_timeout or not v.connected:
            decision = fault('px4_or_ros_state_stale')
        elif not v.localized or not all(math.isfinite(x) for x in (*v.position, *v.velocity)):
            decision = fault('localization_invalid')
        elif v.failsafe:
            decision = fault('px4_native_failsafe')
        elif self._origin_revision is not None and v.reset_counter != self._origin_revision:
            decision = fault('estimator_origin_reset')
        elif not all(a <= p <= b for p, a, b in zip(v.position, c.minimum, c.maximum)):
            decision = fault('geofence_or_altitude_limit')
        elif data.battery_age > c.battery_timeout:
            decision = fault('battery_stale')
        elif airborne and data.mission_age > c.controller_timeout:
            decision = fault('mission_controller_lost')
        elif v.armed and v.offboard and data.control_age > c.controller_timeout:
            decision = fault('offboard_stream_lost')
        elif data.elapsed > c.mission_timeout and data.phase in {'TAKEOFF', 'TRANSIT_TO_INSPECTION_AREA', 'INSPECTION'}:
            decision = Decision(Severity.RETURN, 'mission_timeout')
        elif data.phase == 'INSPECTION' and data.perception_age > c.perception_timeout:
            decision = Decision(Severity.RETURN, 'perception_lost')
        elif data.phase == 'INSPECTION' and data.recorder_age > c.perception_timeout:
            decision = Decision(Severity.RETURN, 'inspection_recorder_lost')
        else:
            decision = self.battery.decide(data.battery_percent, data.return_seconds)
            if preflight and decision.severity != Severity.OK:
                decision = Decision(Severity.INHIBIT, decision.reason)
        if self._origin_revision is None:
            self._origin_revision = v.reset_counter
        # Recovery of a noisy signal cannot resume a mission after a landing or
        # return decision. The mission also has no RETURN→INSPECTION transition.
        if not preflight and airborne:
            rank = {Severity.OK: 0, Severity.RETURN: 1, Severity.INHIBIT: 2, Severity.LAND: 3}
            if self._latched and rank[self._latched.severity] > rank[decision.severity]:
                return self._latched
            if decision.severity in {Severity.RETURN, Severity.LAND}:
                self._latched = decision
        return decision
