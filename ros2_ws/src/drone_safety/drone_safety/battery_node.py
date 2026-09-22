"""Replaceable battery source and simulation-only charging/fault injection."""
import math
import time
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from drone_interfaces.msg import VehicleState, MissionStatus
from drone_interfaces.srv import SetBattery
from drone_core.models import Vehicle
from drone_core.ros_support import SENSOR_QOS, LATCHED_QOS, parameter, steady_timer, vehicle_from_msg, spin
from drone_docking.controller import dock_contact
from .battery import SimulatedBattery
from .parameters import battery_policy


class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_manager')
        self.policy = battery_policy(self)
        self.source = parameter(self, 'battery.source', 'simulated')
        if self.source not in {'simulated', 'px4'}:
            raise ValueError('battery.source must be simulated or px4')
        self.model = SimulatedBattery(self.policy, parameter(self, 'battery.initial_percent', 100.))
        self.station = tuple(parameter(self, 'dock.position', [0., 0., 0.]))
        self.radius = parameter(self, 'dock.contact_radius', .65)
        self.height_tolerance = parameter(self, 'dock.contact_height', .25)
        self.timeout = parameter(self, 'safety.state_timeout', 2.)
        self.vehicle, self.raw = Vehicle(), None
        self.state_at = self.mission_at = float('-inf')
        self.phase = 'IDLE'
        self.last_time = None
        self.pub = self.create_publisher(BatteryState, '/drone/battery', SENSOR_QOS)
        self.create_subscription(VehicleState, '/drone/state', self._state, SENSOR_QOS)
        self.create_subscription(MissionStatus, '/drone/mission_status', self._mission, LATCHED_QOS)
        if self.source == 'simulated':
            self.create_service(SetBattery, '/drone/sim/set_battery', self._set_battery)
        steady_timer(self, .1, self._tick)

    def _state(self, msg):
        self.raw, self.vehicle, self.state_at = msg, vehicle_from_msg(msg), time.monotonic()

    def _mission(self, msg):
        self.phase, self.mission_at = msg.state, time.monotonic()

    def _set_battery(self, request, response):
        try:
            self.model.set_percent(request.percentage)
            response.accepted, response.message = True, 'Simulation battery updated'
        except ValueError as exc:
            response.accepted, response.message = False, str(exc)
        return response

    def _tick(self):
        wall = time.monotonic()
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0. if self.last_time is None else max(0., min(now - self.last_time, 1.))
        self.last_time = now
        fresh = wall - self.state_at <= self.timeout and self.vehicle.connected
        contact = fresh and dock_contact(self.vehicle, self.station, self.radius, self.height_tolerance)
        charging = contact and self.phase == 'CHARGING' and wall - self.mission_at < 1.
        if self.source == 'simulated':
            percent = self.model.step(dt, armed=self.vehicle.armed, speed=self.vehicle.speed,
                                      dock_contact=contact, charging_requested=charging)
            valid = fresh
        else:
            valid = fresh and self.raw is not None and self.raw.battery_valid
            percent = self.raw.battery_percent if valid else math.nan
        msg = BatteryState()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'base_link'
        msg.percentage, msg.present = float(percent / 100.) if valid else math.nan, bool(valid)
        msg.voltage = msg.current = msg.charge = msg.capacity = msg.design_capacity = math.nan
        msg.power_supply_status = (msg.POWER_SUPPLY_STATUS_CHARGING if charging else
                                  msg.POWER_SUPPLY_STATUS_DISCHARGING if self.vehicle.armed else
                                  msg.POWER_SUPPLY_STATUS_NOT_CHARGING)
        self.pub.publish(msg)


def main():
    spin(BatteryNode)
