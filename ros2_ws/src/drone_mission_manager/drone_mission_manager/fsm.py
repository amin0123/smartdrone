"""Explicit, auditable mission transition table with reasoned events."""
from dataclasses import dataclass
from enum import Enum


class State(str, Enum):
    IDLE = 'IDLE'
    PRE_FLIGHT_CHECK = 'PRE_FLIGHT_CHECK'
    ARMING = 'ARMING'
    TAKEOFF = 'TAKEOFF'
    TRANSIT_TO_INSPECTION_AREA = 'TRANSIT_TO_INSPECTION_AREA'
    INSPECTION = 'INSPECTION'
    RETURN_HOME = 'RETURN_HOME'
    PRECISION_LANDING = 'PRECISION_LANDING'
    DOCKED = 'DOCKED'
    CHARGING = 'CHARGING'
    READY = 'READY'
    FAILSAFE = 'FAILSAFE'
    EMERGENCY_LANDING = 'EMERGENCY_LANDING'
    MISSION_ABORTED = 'MISSION_ABORTED'


class Event(str, Enum):
    START = 'START'
    CHECKS_OK = 'CHECKS_OK'
    ARMED = 'ARMED'
    AT_ALTITUDE = 'AT_ALTITUDE'
    AT_INSPECTION = 'AT_INSPECTION'
    ROUTE_COMPLETE = 'ROUTE_COMPLETE'
    RECALL = 'RECALL'
    AT_HOME = 'AT_HOME'
    DOCK_CONFIRMED = 'DOCK_CONFIRMED'
    CHARGE = 'CHARGE'
    CHARGED = 'CHARGED'
    CRITICAL = 'CRITICAL'
    LAND = 'LAND'
    LANDED_AWAY = 'LANDED_AWAY'
    REJECT = 'REJECT'


TRANSITIONS: dict[tuple[State, Event], State] = {
    (State.IDLE, Event.START): State.PRE_FLIGHT_CHECK,
    (State.READY, Event.START): State.PRE_FLIGHT_CHECK,
    (State.MISSION_ABORTED, Event.START): State.PRE_FLIGHT_CHECK,
    (State.PRE_FLIGHT_CHECK, Event.CHECKS_OK): State.ARMING,
    (State.PRE_FLIGHT_CHECK, Event.REJECT): State.MISSION_ABORTED,
    (State.ARMING, Event.REJECT): State.MISSION_ABORTED,
    (State.ARMING, Event.ARMED): State.TAKEOFF,
    (State.TAKEOFF, Event.AT_ALTITUDE): State.TRANSIT_TO_INSPECTION_AREA,
    (State.TRANSIT_TO_INSPECTION_AREA, Event.AT_INSPECTION): State.INSPECTION,
    (State.INSPECTION, Event.ROUTE_COMPLETE): State.RETURN_HOME,
    (State.RETURN_HOME, Event.AT_HOME): State.PRECISION_LANDING,
    (State.PRECISION_LANDING, Event.DOCK_CONFIRMED): State.DOCKED,
    (State.PRECISION_LANDING, Event.LANDED_AWAY): State.MISSION_ABORTED,
    (State.DOCKED, Event.CHARGE): State.CHARGING,
    (State.DOCKED, Event.REJECT): State.MISSION_ABORTED,
    (State.CHARGING, Event.CHARGED): State.READY,
    (State.CHARGING, Event.REJECT): State.MISSION_ABORTED,
    (State.FAILSAFE, Event.LAND): State.EMERGENCY_LANDING,
    (State.EMERGENCY_LANDING, Event.LANDED_AWAY): State.MISSION_ABORTED,
}
FLIGHT_STATES = frozenset({State.ARMING, State.TAKEOFF, State.TRANSIT_TO_INSPECTION_AREA,
                          State.INSPECTION, State.RETURN_HOME, State.PRECISION_LANDING})
for state in FLIGHT_STATES:
    TRANSITIONS[state, Event.CRITICAL] = State.FAILSAFE
for state in (State.TAKEOFF, State.TRANSIT_TO_INSPECTION_AREA, State.INSPECTION):
    TRANSITIONS[state, Event.RECALL] = State.RETURN_HOME


@dataclass(frozen=True)
class Transition:
    at: float
    previous: State
    current: State
    event: Event
    reason: str


class MissionMachine:
    def __init__(self, now: float = 0.):
        self.state = State.IDLE
        self.entered_at = now
        self.reason = 'awaiting_mission'
        self.history: list[Transition] = []

    def accepts(self, event: Event) -> bool:
        return (self.state, event) in TRANSITIONS

    def trigger(self, event: Event, reason: str, now: float) -> Transition:
        if not reason.strip():
            raise ValueError('Every transition requires an explanation')
        if now < self.entered_at:
            raise ValueError('Monotonic transition time moved backward')
        if not self.accepts(event):
            raise ValueError(f'Illegal transition {self.state.value} + {event.value}')
        target = TRANSITIONS[self.state, event]
        transition = Transition(now, self.state, target, event, reason)
        self.state, self.entered_at, self.reason = target, now, reason
        self.history.append(transition)
        return transition
