import pytest
from drone_mission_manager.fsm import Event, MissionMachine, State


def advance(machine, events):
    for event in events:
        machine.trigger(event, 'test_evidence', machine.entered_at + 1)


def test_full_lifecycle_and_second_mission_requires_request():
    m = MissionMachine()
    advance(m, [Event.START, Event.CHECKS_OK, Event.ARMED, Event.AT_ALTITUDE,
                Event.AT_INSPECTION, Event.ROUTE_COMPLETE, Event.AT_HOME,
                Event.DOCK_CONFIRMED, Event.CHARGE, Event.CHARGED])
    assert m.state == State.READY
    assert m.accepts(Event.START)
    assert len(m.history) == 10
    assert all(t.reason for t in m.history)


def test_battery_return_cannot_resume_inspection():
    m = MissionMachine()
    advance(m, [Event.START, Event.CHECKS_OK, Event.ARMED, Event.RECALL])
    assert m.state == State.RETURN_HOME
    with pytest.raises(ValueError, match='Illegal'):
        m.trigger(Event.AT_INSPECTION, 'battery_recovered', 5)


def test_fault_lands_then_aborts():
    m = MissionMachine()
    advance(m, [Event.START, Event.CHECKS_OK, Event.ARMED, Event.CRITICAL, Event.LAND, Event.LANDED_AWAY])
    assert m.state == State.MISSION_ABORTED


def test_empty_reason_and_invalid_time_rejected():
    m = MissionMachine(10)
    with pytest.raises(ValueError):
        m.trigger(Event.START, '', 11)
    with pytest.raises(ValueError):
        m.trigger(Event.START, 'requested', 9)
    assert m.state == State.IDLE
