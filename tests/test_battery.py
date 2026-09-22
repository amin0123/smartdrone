import pytest
from drone_core.models import Severity
from drone_safety.battery import BatteryPolicy, SimulatedBattery


def test_battery_decisions_and_return_cost():
    policy = BatteryPolicy()
    assert policy.decide(90, 20).severity == Severity.OK
    assert policy.decide(30, 20).severity == Severity.RETURN
    assert policy.decide(15, 20).severity == Severity.LAND
    assert policy.decide(40, 500).severity == Severity.LAND
    assert policy.decide(float('nan'), 20).severity == Severity.LAND


def test_charge_requires_contact_and_disarm():
    b = SimulatedBattery(BatteryPolicy(), 70)
    b.step(2, armed=False, speed=0, dock_contact=False, charging_requested=True)
    assert b.percent == 70
    b.step(2, armed=True, speed=1, dock_contact=True, charging_requested=True)
    assert b.percent < 70
    b.step(200, armed=False, speed=0, dock_contact=True, charging_requested=True)
    assert b.percent == 100
    b.step(20000, armed=True, speed=1, dock_contact=False, charging_requested=False)
    assert b.percent == 0


def test_bad_thresholds_rejected():
    with pytest.raises(ValueError):
        BatteryPolicy(return_percent=10, emergency_percent=15)
