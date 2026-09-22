import math
import time
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty
from drone_interfaces.msg import VehicleState, MissionStatus, SafetyStatus
from drone_core.models import Vehicle, Severity
from drone_core.ros_support import SENSOR_QOS, LATCHED_QOS, parameter, steady_timer, vehicle_from_msg, spin
from .parameters import battery_policy
from .policy import SafetyConfig, SafetyInput, SafetyMonitor


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        config = SafetyConfig(
            minimum=tuple(parameter(self, 'fence.minimum', [-5., -6., -.5])),
            maximum=tuple(parameter(self, 'fence.maximum', [17., 6., 8.])),
            state_timeout=parameter(self, 'safety.state_timeout', 2.),
            battery_timeout=parameter(self, 'safety.battery_timeout', 2.),
            mission_timeout=parameter(self, 'mission.flight_timeout', 300.),
            controller_timeout=parameter(self, 'safety.controller_timeout', 1.),
            perception_timeout=parameter(self, 'safety.perception_timeout', 3.))
        self.monitor = SafetyMonitor(config, battery_policy(self))
        self.station = tuple(parameter(self, 'dock.position', [0., 0., 0.]))
        self.speed = parameter(self, 'inspection.velocity', 1.)
        self.vehicle, self.percent, self.phase = Vehicle(), math.nan, 'IDLE'
        self.at = {k: float('-inf') for k in ('state', 'battery', 'mission', 'control', 'perception')}
        self.mission_id, self.started_at = '', time.monotonic()
        self.previous_reason = ''
        self.pub = self.create_publisher(SafetyStatus, '/drone/safety', 10)
        self.create_subscription(VehicleState, '/drone/state', self._state, SENSOR_QOS)
        self.create_subscription(BatteryState, '/drone/battery', self._battery, SENSOR_QOS)
        self.create_subscription(MissionStatus, '/drone/mission_status', self._mission, LATCHED_QOS)
        self.create_subscription(Empty, '/drone/offboard_heartbeat', lambda _: self._seen('control'), 10)
        self.create_subscription(Empty, '/drone/perception_heartbeat', lambda _: self._seen('perception'), 10)
        steady_timer(self, .1, self._tick)

    def _seen(self, key):
        self.at[key] = time.monotonic()

    def _state(self, msg):
        self.vehicle = vehicle_from_msg(msg)
        self._seen('state')

    def _battery(self, msg):
        self.percent = msg.percentage * 100 if msg.present else math.nan
        self._seen('battery')

    def _mission(self, msg):
        if msg.mission_id != self.mission_id:
            self.mission_id, self.started_at = msg.mission_id, time.monotonic()
        self.phase = msg.state
        self._seen('mission')

    def _tick(self):
        now = time.monotonic()
        distance = math.dist(self.vehicle.position[:2], self.station[:2])
        # Reserve includes a conservative descent and docking allowance.
        return_seconds = distance / max(.1, self.speed) + max(0., self.vehicle.position[2] - self.station[2]) / .3 + 15
        data = SafetyInput(self.vehicle, self.percent, self.phase,
            state_age=now-self.at['state'], battery_age=now-self.at['battery'],
            mission_age=now-self.at['mission'], control_age=now-self.at['control'],
            perception_age=now-self.at['perception'], elapsed=now-self.started_at,
            return_seconds=return_seconds)
        decision = self.monitor.evaluate(data)
        msg = SafetyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.severity = {Severity.OK: msg.OK, Severity.RETURN: msg.RETURN,
                        Severity.LAND: msg.LAND, Severity.INHIBIT: msg.INHIBIT}[decision.severity]
        msg.reason = decision.reason
        self.pub.publish(msg)
        if decision.reason != self.previous_reason:
            self.get_logger().info(f'{decision.severity.value}: {decision.reason}')
            self.previous_reason = decision.reason


def main():
    spin(SafetyNode)
