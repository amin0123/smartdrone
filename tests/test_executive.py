"""ROS executive contract tests with an ideal API test double, never flight evidence."""
from concurrent.futures import Future
import math
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('drone_interfaces.msg')
from rclpy.time import Time
from std_srvs.srv import Trigger
from drone_interfaces.msg import VehicleState, SafetyStatus
from drone_interfaces.srv import VehicleCommand
from drone_mission_manager.node import MissionNode
from drone_mission_manager.fsm import State


@pytest.fixture
def executive(monkeypatch, tmp_path):
    now = [1000.]
    monkeypatch.setattr('drone_mission_manager.node.time.monotonic', lambda: now[0])
    rclpy.init(args=[], domain_id=93)  # Isolate tests from the actual flight stack.
    node = MissionNode()
    node.log_dir = tmp_path
    class Clock:
        def now(self):
            return Time(nanoseconds=round(now[0]*1e9))
    monkeypatch.setattr(node, 'get_clock', lambda: Clock())
    state = VehicleState()
    state.connected = state.localized = state.preflight_ok = state.landed = True
    state.pose.orientation.w = 1.
    node.client._state(state)
    node.percent = 100.
    node.safety = SafetyStatus(severity=SafetyStatus.OK, reason='test_inputs_healthy')
    commands = []
    def command(name):
        commands.append(name)
        if name == 'offboard': state.offboard = True
        elif name == 'arm': state.armed = True
        elif name == 'land':
            state.armed, state.offboard, state.landed = False, False, True
            state.pose.position.z = 0.
        elif name == 'disarm':
            assert state.landed
            state.armed = False
        future = Future()
        future.set_result(VehicleCommand.Response(accepted=True, message='Test double'))
        return future
    def target(position, yaw=0.):
        assert state.connected and state.localized
        state.pose.position.x, state.pose.position.y, state.pose.position.z = map(float, position)
        if state.armed and position[2] > .1:
            state.landed = False
    monkeypatch.setattr(node.client, 'command', command)
    monkeypatch.setattr(node.client, 'goto_position', target)
    def tick():
        now[0] += .1
        node.client._state(state)
        for k in node.seen:
            if k != 'marker': node.seen[k] = now[0]
        node._tick()
    yield node, state, commands, tick
    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize('recall', [False, True])
def test_complete_executive_cycle_and_low_battery_return(executive, recall):
    node, state, commands, tick = executive
    assert node._start(None, Trigger.Response()).success
    assert not node._start(None, Trigger.Response()).success
    for _ in range(2000):
        if recall and node.machine.state == State.INSPECTION:
            node.safety = SafetyStatus(severity=SafetyStatus.RETURN, reason='battery_return_threshold')
        tick()
        if node.machine.state == State.READY:
            break
    assert node.machine.state == State.READY
    assert not state.armed and state.landed
    assert math.hypot(state.pose.position.x, state.pose.position.y) < .2
    assert commands[:2] == ['offboard', 'arm']
    assert 'land' in commands
    states = [t.current for t in node.machine.history]
    assert State.CHARGING in states and State.INSPECTION in states
    if recall:
        assert any(t.reason == 'battery_return_threshold' for t in node.machine.history)


def test_critical_fault_never_resumes_inspection(executive):
    node, state, commands, tick = executive
    node._start(None, Trigger.Response())
    for _ in range(600):
        if node.machine.state == State.TAKEOFF:
            node.safety = SafetyStatus(severity=SafetyStatus.LAND, reason='localization_fault_test')
        tick()
        if node.machine.state == State.MISSION_ABORTED:
            break
    assert node.machine.state == State.MISSION_ABORTED
    assert State.FAILSAFE in [t.current for t in node.machine.history]
    assert State.INSPECTION not in [t.current for t in node.machine.history]
    assert not state.armed and state.landed
