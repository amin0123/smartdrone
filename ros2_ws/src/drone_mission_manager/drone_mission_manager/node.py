"""ROS executive: state-specific handlers over a tested transition table."""
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
import uuid
import yaml
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from drone_interfaces.msg import MissionStatus, SafetyStatus, DockObservation, CommandResult
from drone_core.models import Vehicle, near
from drone_core.ros_support import SENSOR_QOS, LATCHED_QOS, parameter, steady_timer, vehicle_from_msg, spin
from drone_px4_interface.client import Px4Client
from drone_navigation.planner import Area, Box, RouteFollower, lawnmower, validate_route
from drone_docking.controller import dock_contact, descent_step
from .fsm import Event, State, MissionMachine, FLIGHT_STATES


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.client = Px4Client(self)
        p = lambda key, default: parameter(self, key, default)
        self.area = Area(p('inspection.origin_x', 4.), p('inspection.origin_y', -3.),
                         p('inspection.length', 8.), p('inspection.width', 6.),
                         p('inspection.row_spacing', 3.), p('inspection.altitude', 3.),
                         p('inspection.velocity', 1.))
        self.coverage = lawnmower(self.area)
        self.station = tuple(p('dock.position', [0., 0., 0.]))
        self.fence = Box(tuple(p('fence.minimum', [-5., -6., -.5])), tuple(p('fence.maximum', [17., 6., 8.])))
        self.radius = p('navigation.vehicle_radius', .4)
        self.tolerance = p('navigation.waypoint_tolerance', .35)
        self.max_lead = p('navigation.max_lead', 1.)
        self.max_speed = p('navigation.max_velocity', 2.)
        if self.area.speed > self.max_speed:
            raise ValueError('Inspection speed exceeds navigation.max_velocity')
        self.world_file = Path(p('environment_file', 'config/world.yaml'))
        self.obstacles = self._read_obstacles()
        self.preflight_timeout = p('mission.preflight_timeout', 30.)
        self.arming_timeout = p('mission.arming_timeout', 15.)
        self.landing_timeout = p('mission.landing_timeout', 60.)
        self.state_timeout = p('safety.state_timeout', 2.)
        self.start_percent = p('battery.ready_percent', 95.)
        self.contact_radius = p('dock.contact_radius', .65)
        self.contact_height = p('dock.contact_height', .25)
        self.approach_altitude = p('dock.approach_altitude', 1.5)
        self.precision = p('dock.precision_enabled', False)
        self.marker_timeout = p('dock.marker_timeout', 2.)
        self.align_tolerance = p('dock.alignment_tolerance', .18)
        self.descent_speed = p('dock.descent_speed', .25)
        self.land_height = p('dock.land_height', .6)
        self.auto_start, self.auto_repeat = p('mission.auto_start', False), p('mission.auto_repeat', False)
        self.log_dir = Path(p('inspection_records.directory', 'logs/inspection')).expanduser().resolve()
        self.machine = MissionMachine(time.monotonic())
        self.mission_id = ''
        self.safety, self.percent, self.marker = None, math.nan, None
        self.seen = {key: float('-inf') for key in ('safety', 'battery', 'perception', 'recorder', 'marker')}
        self.route = None
        self.native_landing = False
        self.last_command, self.hold_since = float('-inf'), None
        self.hold = self.station
        self.descent_z = None
        self.docking_source = ''
        self.last_sim_time = None
        self.landed_since = None
        self.pub = self.create_publisher(MissionStatus, '/drone/mission_status', LATCHED_QOS)
        self.path_pub = self.create_publisher(RosPath, '/drone/planned_path', LATCHED_QOS)
        path = RosPath()
        path.header.frame_id = 'map'
        for point in self.coverage:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.create_subscription(SafetyStatus, '/drone/safety', self._safety, 10)
        self.create_subscription(BatteryState, '/drone/battery', self._battery, SENSOR_QOS)
        self.create_subscription(DockObservation, '/drone/dock_observation', self._marker, SENSOR_QOS)
        self.create_subscription(Empty, '/drone/perception_heartbeat', lambda _: self._seen('perception'), 10)
        self.create_subscription(Empty, '/drone/recorder_heartbeat', lambda _: self._seen('recorder'), 10)
        self.create_subscription(CommandResult, '/drone/command_result', self._ack, 10)
        self.create_service(Trigger, '/drone/start_mission', self._start)
        self.create_service(Trigger, '/drone/abort_mission', self._abort)
        self.handlers = {
            State.IDLE: self._idle, State.READY: self._ready,
            State.PRE_FLIGHT_CHECK: self._preflight, State.ARMING: self._arming,
            State.TAKEOFF: lambda v, dt, now: self._follow(v, dt, Event.AT_ALTITUDE, 'takeoff_altitude_reached'),
            State.TRANSIT_TO_INSPECTION_AREA: lambda v, dt, now: self._follow(v, dt, Event.AT_INSPECTION, 'inspection_entry_reached'),
            State.INSPECTION: lambda v, dt, now: self._follow(v, dt, Event.ROUTE_COMPLETE, 'coverage_complete'),
            State.RETURN_HOME: lambda v, dt, now: self._follow(v, dt, Event.AT_HOME, 'above_station'),
            State.PRECISION_LANDING: self._landing, State.DOCKED: self._docked,
            State.CHARGING: self._charging, State.FAILSAFE: self._failsafe,
            State.EMERGENCY_LANDING: self._emergency, State.MISSION_ABORTED: self._idle,
        }
        steady_timer(self, .1, self._tick)

    def _read_obstacles(self):
        world = yaml.safe_load(self.world_file.read_text())
        boxes = [Box(tuple(a-b/2 for a, b in zip(o['position'], o['scale'])),
                     tuple(a+b/2 for a, b in zip(o['position'], o['scale']))) for o in world['obstacles']]
        if not near(tuple(world['dock']), self.station, .001):
            raise ValueError('Mission dock differs from environment dock')
        # Conservative canopy volume: planned crop passes must remain above it.
        f = world['field']
        boxes.append(Box((f['origin'][0]-.4, f['origin'][1]-.4, -.05),
                         (f['origin'][0]+f['length']+.4, f['origin'][1]+f['width']+.4, f['canopy_height']+.4)))
        return boxes

    def _seen(self, key):
        self.seen[key] = time.monotonic()

    def _safety(self, msg):
        self.safety = msg
        self._seen('safety')

    def _battery(self, msg):
        self.percent = msg.percentage*100 if msg.present else math.nan
        self._seen('battery')

    def _marker(self, msg):
        if msg.visible and msg.header.frame_id == 'map':
            self.marker = (msg.station_position.x, msg.station_position.y, msg.station_position.z)
            self._seen('marker')

    def _ack(self, msg):
        if msg.result not in {0, 5}:  # MAV_RESULT_ACCEPTED / IN_PROGRESS; gateway owns raw commands.
            self.get_logger().warning(f'PX4 command response: {msg.detail}')

    def _start(self, request, response):
        if not self.machine.accepts(Event.START):
            response.success, response.message = False, f'Mission already in {self.machine.state.value}'
            return response
        self.mission_id = uuid.uuid4().hex
        self.native_landing, self.hold_since, self.route = False, None, None
        self.last_command = float('-inf')
        self._transition(Event.START, 'mission_requested')
        response.success, response.message = True, f'Preflight started: {self.mission_id}'
        return response

    def _abort(self, request, response):
        if self.machine.accepts(Event.RECALL):
            self._transition(Event.RECALL, 'operator_abort_return_to_station')
        elif self.machine.state == State.PRE_FLIGHT_CHECK:
            self._transition(Event.REJECT, 'operator_cancelled_preflight')
        elif self.machine.accepts(Event.CRITICAL):
            self._transition(Event.CRITICAL, 'operator_abort_landing')
        else:
            response.success, response.message = False, f'No airborne mission to abort in {self.machine.state.value}'
            return response
        response.success, response.message = True, self.machine.reason
        return response

    def _transition(self, event, reason):
        t = self.machine.trigger(event, reason, time.monotonic())
        self.get_logger().info(f'{t.previous.value} -> {t.current.value}: {reason}')
        directory = self.log_dir / self.mission_id
        directory.mkdir(parents=True, exist_ok=True)
        with (directory / 'executive.jsonl').open('a') as out:
            out.write(json.dumps(asdict(t)) + '\n')
        v = self._vehicle()
        destinations = {
            State.TAKEOFF: [(self.station[0], self.station[1], self.area.altitude)],
            State.TRANSIT_TO_INSPECTION_AREA: [self.coverage[0]],
            State.INSPECTION: self.coverage[1:],
            State.RETURN_HOME: [(v.position[0], v.position[1], max(v.position[2], self.area.altitude)),
                                (self.station[0], self.station[1], max(v.position[2], self.area.altitude))],
            State.PRECISION_LANDING: [(self.station[0], self.station[1], self.station[2]+self.approach_altitude)],
        }
        if t.current in destinations:
            self._route(v.position, destinations[t.current])
        if t.current in {State.ARMING, State.PRE_FLIGHT_CHECK}:
            self.hold = v.position
        if t.current == State.PRECISION_LANDING:
            self.descent_z = None
            self.docking_source = ''
        self._publish()

    def _route(self, start, waypoints):
        validate_route([start, *waypoints], self.fence, self.obstacles, self.radius)
        self.route = RouteFollower(start, waypoints, self.area.speed, self.tolerance, self.max_lead)

    def _vehicle(self):
        return vehicle_from_msg(self.client.state) if self.client.state else Vehicle()

    def _fresh(self, now):
        return now-self.client.state_at <= self.state_timeout and self._vehicle().connected

    def _contact(self, v, now):
        return self._fresh(now) and dock_contact(v, self.station, self.contact_radius, self.contact_height)

    def _command(self, name, now):
        if now-self.last_command < 1.:
            return
        self.last_command = now
        future = self.client.command(name)
        if future is None:
            return
        def completed(result):
            try:
                reply = result.result()
                if reply.accepted and name == 'land':
                    self.native_landing = True
                elif not reply.accepted:
                    self.get_logger().warning(f'{name}: {reply.message}')
            except Exception as exc:
                self.get_logger().error(f'{name} service failed: {exc}')
        future.add_done_callback(completed)

    def _tick(self):
        now = time.monotonic()
        sim = self.get_clock().now().nanoseconds*1e-9
        dt = 0. if self.last_sim_time is None else max(0., min(.2, sim-self.last_sim_time))
        self.last_sim_time = sim
        v = self._vehicle()
        if v.landed and self._fresh(now):
            self.landed_since = self.landed_since or now
        else:
            self.landed_since = None
        try:
            self._apply_safety(v, now)
            self.handlers[self.machine.state](v, dt, now)
        except (ValueError, OSError) as exc:
            self.get_logger().error(f'Executive error: {exc}')
            if self.machine.accepts(Event.CRITICAL):
                self._transition(Event.CRITICAL, f'executive_error:{exc}')
            elif self.machine.accepts(Event.REJECT):
                self._transition(Event.REJECT, f'executive_error:{exc}')
        self._publish()

    def _apply_safety(self, v, now):
        if self.machine.state not in FLIGHT_STATES:
            return
        reason = None
        if not self._fresh(now) or now-self.seen['safety'] > 1.:
            reason = 'state_or_safety_monitor_lost'
        elif self.safety and self.safety.severity in {SafetyStatus.LAND, SafetyStatus.INHIBIT}:
            reason = self.safety.reason
        elif (v.armed and not v.offboard and self.machine.state != State.ARMING and not self.native_landing):
            reason = 'offboard_mode_lost'
        if reason:
            self._transition(Event.CRITICAL, reason)
        elif self.safety and self.safety.severity == SafetyStatus.RETURN and self.machine.accepts(Event.RECALL):
            self._transition(Event.RECALL, self.safety.reason)

    def _idle(self, v, dt, now):
        if self.auto_start and self.machine.state == State.IDLE:
            self.auto_start = False
            self._start(None, Trigger.Response())

    def _ready(self, v, dt, now):
        if self.auto_repeat and now-self.machine.entered_at > 2.:
            self._start(None, Trigger.Response())

    def _preflight(self, v, dt, now):
        good = (self._contact(v, now) and self.safety is not None and
                self.safety.severity == SafetyStatus.OK and now-self.seen['safety'] < 1. and
                now-self.seen['battery'] < 2. and self.percent >= self.start_percent and
                now-self.seen['perception'] < 2. and now-self.seen['recorder'] < 2.)
        if good:
            if self.hold_since is None:
                validate_route([v.position, (self.station[0], self.station[1], self.area.altitude),
                                *self.coverage, (self.station[0], self.station[1], self.area.altitude),
                                (self.station[0], self.station[1], self.approach_altitude)],
                               self.fence, self.obstacles, self.radius)
                self.hold_since, self.hold = now, v.position
            self.client.goto_position(self.hold)
            if now-self.hold_since >= 1.6:
                # PX4 may remain in AUTO_LAND after disarming; its preflight
                # checks depend on the selected mode. Select OFFBOARD before
                # asking whether it is ready to arm in that mode.
                if not v.offboard:
                    self._command('offboard', now)
                elif self.client.state.preflight_ok:
                    self._transition(Event.CHECKS_OK, 'state_battery_camera_recorder_route_and_stream_ready')
        else:
            self.hold_since = None
        if self.machine.state == State.PRE_FLIGHT_CHECK and now-self.machine.entered_at > self.preflight_timeout:
            reason = self.safety.reason if self.safety else 'no_safety_status'
            self._transition(Event.REJECT, f'preflight_timeout:{reason}; require dock/camera/recorder/charged battery')

    def _arming(self, v, dt, now):
        self.client.goto_position(self.hold)
        if v.offboard and v.armed:
            self._transition(Event.ARMED, 'PX4_confirmed_offboard_and_armed')
        elif now-self.machine.entered_at > self.arming_timeout:
            self._transition(Event.CRITICAL, 'arming_timeout')
        else:
            self._command('arm' if v.offboard else 'offboard', now)

    def _follow(self, v, dt, event, reason):
        target = self.route.step(v.position, dt)
        self.client.goto_position(target)
        if self.route.complete:
            self._transition(event, reason)

    def _landing(self, v, dt, now):
        if self._contact(v, now):
            self._transition(Event.DOCK_CONFIRMED, 'landed_disarmed_at_station')
            return
        if self._fresh(now) and v.landed and not v.armed:
            self._transition(Event.LANDED_AWAY, 'landed_outside_station_tolerance')
            return
        if now-self.machine.entered_at > self.landing_timeout:
            self._transition(Event.CRITICAL, 'docking_timeout_native_landing')
            return
        if self.native_landing:
            self._command('land', now)
            return
        if not self.route.complete:
            self.client.goto_position(self.route.step(v.position, dt))
            return
        command = descent_step(v, self.station, self.marker, now-self.seen['marker'], dt,
            use_marker=self.precision, marker_timeout=self.marker_timeout, tolerance=self.align_tolerance,
            descent_speed=self.descent_speed, land_height=self.land_height, reference_z=self.descent_z)
        self.descent_z = command.target[2]
        if command.source != self.docking_source:
            self.get_logger().info(f'Docking alignment source: {command.source}')
            self.docking_source = command.source
        with (self.log_dir / self.mission_id / 'docking.jsonl').open('a') as out:
            out.write(json.dumps({'time': self.get_clock().now().nanoseconds*1e-9,
                'source': command.source, 'position': v.position, 'target': command.target,
                'request_land': command.land})+'\n')
        self.client.goto_position(command.target)
        if command.land:
            self._command('land', now)

    def _docked(self, v, dt, now):
        if not self._contact(v, now):
            self._transition(Event.REJECT, 'dock_contact_lost')
        elif now-self.machine.entered_at >= 1.:
            self._transition(Event.CHARGE, 'stable_dock_contact')

    def _charging(self, v, dt, now):
        if not self._contact(v, now):
            self._transition(Event.REJECT, 'dock_contact_lost_while_charging')
        elif now-self.seen['battery'] < 2. and self.percent >= self.start_percent:
            self._transition(Event.CHARGED, 'battery_ready')

    def _failsafe(self, v, dt, now):
        self._command('land', now)
        self._transition(Event.LAND, self.machine.reason)

    def _emergency(self, v, dt, now):
        if v.armed and self.landed_since is not None and now-self.landed_since >= 2.:
            self._command('disarm', now)
        else:
            self._command('land', now)
        if self._fresh(now) and v.landed and not v.armed and now-self.machine.entered_at > 2.:
            self._transition(Event.LANDED_AWAY, 'failsafe_landed_disarmed:' + self.machine.reason)

    def _publish(self):
        msg = MissionStatus()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'map'
        msg.mission_id, msg.state, msg.reason = self.mission_id, self.machine.state.value, self.machine.reason
        msg.waypoint_count = len(self.route.waypoints) if self.route else 0
        msg.waypoint_index = self.route.index if self.route else 0
        self.pub.publish(msg)


def main():
    spin(MissionNode)
