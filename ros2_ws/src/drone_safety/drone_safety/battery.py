"""Battery abstraction; simulated energy is explicitly separate from PX4 battery."""
from dataclasses import dataclass
import math
from drone_core.models import Decision, Severity


@dataclass(frozen=True)
class BatteryPolicy:
    return_percent: float = 30.
    emergency_percent: float = 15.
    ready_percent: float = 95.
    reserve_percent: float = 5.
    discharge_per_second: float = .08
    speed_load_per_second: float = .01
    charge_per_second: float = 5.

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('Battery settings must be finite')
        if not 0 < self.emergency_percent < self.return_percent < self.ready_percent <= 100:
            raise ValueError('Require 0 < emergency < return < ready <= 100')
        if min(self.reserve_percent, self.discharge_per_second, self.speed_load_per_second) < 0 or self.charge_per_second <= 0:
            raise ValueError('Invalid battery rates')

    def decide(self, percent: float, return_seconds: float) -> Decision:
        if not math.isfinite(percent) or not 0 <= percent <= 100:
            return Decision(Severity.LAND, 'battery_unavailable')
        if not math.isfinite(return_seconds) or return_seconds < 0:
            return Decision(Severity.LAND, 'return_time_unavailable')
        reserve = return_seconds * (self.discharge_per_second + self.speed_load_per_second) + self.reserve_percent
        if percent <= self.emergency_percent or percent <= reserve:
            return Decision(Severity.LAND, 'battery_critical_or_return_unreachable')
        if percent <= max(self.return_percent, reserve + self.emergency_percent):
            return Decision(Severity.RETURN, 'battery_return_reserve')
        return Decision(Severity.OK, 'battery_normal')


class SimulatedBattery:
    def __init__(self, policy: BatteryPolicy, initial_percent: float = 100.):
        self.policy = policy
        self.set_percent(initial_percent)

    def set_percent(self, value: float) -> None:
        if not math.isfinite(value) or not 0 <= value <= 100:
            raise ValueError('Battery percentage must be in [0, 100]')
        self.percent = value

    def step(self, dt: float, *, armed: bool, speed: float, dock_contact: bool,
             charging_requested: bool) -> float:
        if not math.isfinite(dt) or dt < 0 or not math.isfinite(speed) or speed < 0:
            raise ValueError('Invalid battery update')
        if armed:
            self.percent -= dt * (self.policy.discharge_per_second + self.policy.speed_load_per_second * speed)
        elif dock_contact and charging_requested:
            self.percent += dt * self.policy.charge_per_second
        self.percent = max(0., min(100., self.percent))
        return self.percent
