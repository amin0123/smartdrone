"""Single owner of PX4 DDS setpoints, commands, state and frame conversions."""
import math
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    VehicleStatus, VehicleLocalPosition, VehicleAttitude, BatteryStatus,
    VehicleLandDetected, VehicleCommandAck, VehicleCommand as Px4Command,
    OffboardControlMode, TrajectorySetpoint,
)
from drone_interfaces.msg import VehicleState, FlightTarget, SafetyStatus, CommandResult
from drone_interfaces.srv import VehicleCommand
from drone_core.frames import enu_to_ned, ned_to_enu, yaw_enu_to_ned, attitude_ned_frd_to_enu_flu, rotation_xyzw
from drone_core.ros_support import SENSOR_QOS, parameter, steady_timer, spin
from .lease import ControllerLease


class Px4Gateway(Node):
    def __init__(self):
        super().__init__('px4_gateway')
        self.timeout = parameter(self, 'safety.state_timeout', 1.)
        self.safety_timeout = parameter(self, 'safety.controller_timeout', 1.)
        self.lease = ControllerLease(parameter(self, 'px4.target_timeout', .5))
        self.minimum = parameter(self, 'fence.minimum', [-5., -6., -.5])
        self.maximum = parameter(self, 'fence.maximum', [17., 6., 8.])
        self.max_speed = parameter(self, 'navigation.max_velocity', 2.)
        self.target_system = parameter(self, 'px4.system_id', 1)
        self.source_system = parameter(self, 'px4.source_system', 245)
        self.source_component = parameter(self, 'px4.source_component', 191)
        self.data, self.received = {}, {}
        self.target = None
        self.safety, self.safety_at = None, float('-inf')
        self.streaming_since = None
        self.native_landing = False
        self.was_armed = False
        self.last_command = {}
        self.state_pub = self.create_publisher(VehicleState, '/drone/state', SENSOR_QOS)
        self.odom_pub = self.create_publisher(Odometry, '/drone/odometry', SENSOR_QOS)
        self.pulse_pub = self.create_publisher(Empty, '/drone/offboard_heartbeat', 10)
        self.result_pub = self.create_publisher(CommandResult, '/drone/command_result', 10)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.target_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(Px4Command, '/fmu/in/vehicle_command', 10)
        for key, cls, topic in (
            ('status', VehicleStatus, 'vehicle_status'),
            ('position', VehicleLocalPosition, 'vehicle_local_position'),
            ('attitude', VehicleAttitude, 'vehicle_attitude'),
            ('landed', VehicleLandDetected, 'vehicle_land_detected'),
            ('battery', BatteryStatus, 'battery_status'),
        ):
            self.create_subscription(cls, '/fmu/out/' + topic,
                lambda msg, key=key: self._receive(key, msg), SENSOR_QOS)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack', self._ack, SENSOR_QOS)
        self.create_subscription(FlightTarget, '/drone/target', self._target, 10)
        self.create_subscription(SafetyStatus, '/drone/safety', self._safety, 10)
        self.create_service(VehicleCommand, '/drone/vehicle_command', self._request)
        steady_timer(self, .05, self._tick)

    def _receive(self, key, msg):
        self.data[key], self.received[key] = msg, time.monotonic()

    def _fresh(self, key, now):
        return now - self.received.get(key, float('-inf')) <= self.timeout

    def _safety(self, msg):
        self.safety, self.safety_at = msg, time.monotonic()

    def _target(self, msg):
        values = (msg.position.x, msg.position.y, msg.position.z) if msg.mode == msg.POSITION else (msg.velocity.x, msg.velocity.y, msg.velocity.z)
        valid = msg.header.frame_id == 'map' and msg.mode in (msg.POSITION, msg.VELOCITY)
        valid = valid and all(math.isfinite(x) for x in (*values, msg.yaw))
        if msg.mode == msg.POSITION:
            valid = valid and all(a <= v <= b for a, v, b in zip(self.minimum, values, self.maximum))
        else:
            valid = valid and math.dist(values, (0., 0., 0.)) <= self.max_speed
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        valid = valid and -.1 <= age <= self.lease.timeout
        if valid and self.lease.accept(msg.sequence, time.monotonic()):
            self.target = msg
        elif not valid:
            self.get_logger().error('Rejected invalid, stale or out-of-fence target', throttle_duration_sec=2.)

    def _state(self, now):
        msg = VehicleState()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'map'
        msg.pose.orientation.w = 1.
        msg.connected = all(self._fresh(k, now) for k in ('status', 'position', 'attitude', 'landed'))
        status, pos, att, land, battery = (self.data.get(k) for k in ('status', 'position', 'attitude', 'landed', 'battery'))
        if status:
            msg.armed = status.arming_state == status.ARMING_STATE_ARMED
            msg.offboard = status.nav_state == status.NAVIGATION_STATE_OFFBOARD
            msg.nav_state, msg.failsafe = status.nav_state, status.failsafe
        msg.landed = bool(land and self._fresh('landed', now) and land.landed)
        if pos and att:
            try:
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = ned_to_enu((pos.x, pos.y, pos.z))
                msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = ned_to_enu((pos.vx, pos.vy, pos.vz))
                q = attitude_ned_frd_to_enu_flu((att.q[1], att.q[2], att.q[3], att.q[0]))
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
                msg.localized = bool(msg.connected and pos.xy_valid and pos.z_valid and pos.v_xy_valid and pos.v_z_valid and pos.heading_good_for_control)
            except ValueError:
                msg.localized = False
            msg.reset_counter = int(pos.xy_reset_counter) * 65536 + int(pos.z_reset_counter) * 256 + int(pos.heading_reset_counter)
        if battery and self._fresh('battery', now) and battery.connected:
            msg.battery_valid = bool(math.isfinite(battery.remaining) and 0 <= battery.remaining <= 1)
            msg.battery_percent = float(battery.remaining * 100) if msg.battery_valid else 0.
        return msg

    def _permitted(self, state, now):
        return (state.connected and state.localized and not state.failsafe and
                self.safety is not None and now - self.safety_at <= self.safety_timeout and
                self.safety.severity in (SafetyStatus.OK, SafetyStatus.RETURN) and
                self.target is not None and self.lease.fresh(now) and not self.native_landing)

    def _tick(self):
        now = time.monotonic()
        state = self._state(now)
        if self.was_armed and not state.armed and state.landed:
            self.lease.reset()
            self.target = None
            self.native_landing = False
        self.was_armed = state.armed
        self.state_pub.publish(state)
        if state.localized:
            odom = Odometry()
            odom.header, odom.child_frame_id = state.header, 'base_link'
            odom.pose.pose, odom.twist.twist = state.pose, state.twist
            # Odometry.twist is expressed in child_frame_id; state.twist is ENU.
            q, v = state.pose.orientation, state.twist.linear
            body_velocity = rotation_xyzw((q.x, q.y, q.z, q.w)).inv().apply((v.x, v.y, v.z))
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, body_velocity)
            self.odom_pub.publish(odom)
        if not self._permitted(state, now):
            self.streaming_since = None
            if state.armed and state.offboard and not self.native_landing:
                self._send(Px4Command.VEHICLE_CMD_NAV_LAND)
                self.native_landing = True
                self.get_logger().error('Offboard permission lost: requested native LAND and stopped stream')
            return
        if self.streaming_since is None:
            self.streaming_since = now
        stamp = self.get_clock().now().nanoseconds // 1000
        mode = OffboardControlMode()
        mode.timestamp = stamp
        mode.position = self.target.mode == FlightTarget.POSITION
        mode.velocity = self.target.mode == FlightTarget.VELOCITY
        target = TrajectorySetpoint()
        target.timestamp = stamp
        # NaN disables unused control components; zero would be a real command.
        target.position = np.array([math.nan] * 3, dtype=np.float32)
        target.velocity = np.array([math.nan] * 3, dtype=np.float32)
        target.acceleration = np.array([math.nan] * 3, dtype=np.float32)
        target.jerk = np.array([math.nan] * 3, dtype=np.float32)
        target.yawspeed = math.nan
        if mode.position:
            p = self.target.position
            target.position = np.array(enu_to_ned((p.x, p.y, p.z)), dtype=np.float32)
        else:
            v = self.target.velocity
            target.velocity = np.array(enu_to_ned((v.x, v.y, v.z)), dtype=np.float32)
        target.yaw = float(yaw_enu_to_ned(self.target.yaw))
        self.mode_pub.publish(mode)
        self.target_pub.publish(target)
        self.pulse_pub.publish(Empty())

    def _request(self, request, response):
        now = time.monotonic()
        state = self._state(now)
        command = request.command
        warmed = self.streaming_since is not None and now - self.streaming_since >= 1.2
        if command in {'arm', 'offboard'} and (not self._permitted(state, now) or not warmed):
            response.accepted, response.message = False, 'Fresh state, safety and 1.2 s offboard warmup required'
            return response
        if command == 'disarm' and (not state.connected or not state.landed):
            response.accepted, response.message = False, 'Disarm requires fresh landed confirmation'
            return response
        table = {
            'arm': (Px4Command.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1., 0.),
            'disarm': (Px4Command.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0., 0.),
            'offboard': (Px4Command.VEHICLE_CMD_DO_SET_MODE, 1., 6.),
            'land': (Px4Command.VEHICLE_CMD_NAV_LAND, 0., 0.),
            'return_home': (Px4Command.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, 0., 0.),
        }
        if command not in table:
            response.accepted, response.message = False, 'Unknown high-level command'
            return response
        if command in {'land', 'return_home'}:
            self.native_landing = True
        self._send(*table[command])
        response.accepted, response.message = True, 'Queued; confirm /drone/command_result and vehicle state'
        return response

    def _send(self, command: int, param1: float = 0., param2: float = 0.):
        now = time.monotonic()
        key = (command, param1, param2)
        if now - self.last_command.get(key, float('-inf')) < .5:
            return
        msg = Px4Command()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command, msg.param1, msg.param2 = command, param1, param2
        msg.param5, msg.param6, msg.param7 = math.nan, math.nan, math.nan
        msg.target_system, msg.target_component = self.target_system, 1
        msg.source_system, msg.source_component = self.source_system, self.source_component
        msg.from_external = True
        self.command_pub.publish(msg)
        self.last_command[key] = now

    def _ack(self, msg):
        if msg.target_system != self.source_system or msg.target_component != self.source_component:
            return
        out = CommandResult()
        out.header.stamp = self.get_clock().now().to_msg()
        out.command, out.result = msg.command, msg.result
        out.detail = f'PX4 acknowledgement: command={msg.command} result={msg.result}'
        self.result_pub.publish(out)
        self.get_logger().info(out.detail)


def main():
    spin(Px4Gateway)
