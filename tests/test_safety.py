from dataclasses import replace
from drone_core.models import Severity, Vehicle
from drone_safety.battery import BatteryPolicy
from drone_safety.policy import SafetyConfig, SafetyInput, SafetyMonitor


def sample(**kwargs):
    v = Vehicle(position=(4, 0, 3), connected=True, localized=True, armed=True, offboard=True)
    return replace(SafetyInput(v, 80, 'INSPECTION'), **kwargs)


def test_normal_then_stale_connection_latches_land():
    s = SafetyMonitor(SafetyConfig(), BatteryPolicy())
    assert s.evaluate(sample()).severity == Severity.OK
    assert s.evaluate(sample(state_age=2)).severity == Severity.LAND
    assert s.evaluate(sample()).severity == Severity.LAND


def test_low_battery_and_lost_perception_return():
    for input in (sample(battery_percent=25), sample(perception_age=5), sample(elapsed=400)):
        assert SafetyMonitor(SafetyConfig(), BatteryPolicy()).evaluate(input).severity == Severity.RETURN


def test_preflight_fault_inhibits_and_finite_geofence_required():
    monitor = SafetyMonitor(SafetyConfig(), BatteryPolicy())
    input = sample(phase='PRE_FLIGHT_CHECK', vehicle=Vehicle(landed=True))
    assert monitor.evaluate(input).severity == Severity.INHIBIT
    bad = replace(sample().vehicle, position=(0, 0, 10))
    assert monitor.evaluate(sample(vehicle=bad)).severity == Severity.LAND


def test_origin_reset_and_controller_loss():
    monitor = SafetyMonitor(SafetyConfig(), BatteryPolicy())
    monitor.evaluate(sample())
    changed = replace(sample().vehicle, reset_counter=1)
    assert monitor.evaluate(sample(vehicle=changed)).reason == 'estimator_origin_reset'
    assert SafetyMonitor(SafetyConfig(), BatteryPolicy()).evaluate(sample(mission_age=3)).severity == Severity.LAND
