"""High-level ENU client; no upstream PX4 message dependency here."""
import time
from drone_interfaces.msg import FlightTarget
from drone_interfaces.srv import VehicleCommand
from drone_core.ros_support import SENSOR_QOS
from drone_interfaces.msg import VehicleState


class Px4Client:
    def __init__(self, node):
        self.node = node
        self.sequence = 0
        self.state = None
        self.state_at = float('-inf')
        self.target_pub = node.create_publisher(FlightTarget, '/drone/target', 10)
        self.commands = node.create_client(VehicleCommand, '/drone/vehicle_command')
        self.subscription = node.create_subscription(VehicleState, '/drone/state', self._state, SENSOR_QOS)

    def _state(self, msg):
        self.state = msg
        self.state_at = time.monotonic()

    def get_vehicle_state(self):
        return self.state

    def command(self, name: str):
        if not self.commands.service_is_ready():
            return None
        return self.commands.call_async(VehicleCommand.Request(command=name))

    def arm(self):
        return self.command('arm')

    def disarm(self):
        return self.command('disarm')

    def engage_offboard(self):
        return self.command('offboard')

    def land(self):
        return self.command('land')

    def return_home(self):
        """PX4 native RTL; the mission normally plans its own known-dock approach."""
        return self.command('return_home')

    def goto_position(self, position, yaw: float = 0.) -> None:
        """Refresh at 10 Hz. A one-shot request intentionally expires the lease."""
        msg = self._target(FlightTarget.POSITION, yaw)
        msg.position.x, msg.position.y, msg.position.z = map(float, position)
        self.target_pub.publish(msg)

    def takeoff(self, altitude: float, yaw: float = 0.) -> None:
        if self.state is None:
            raise RuntimeError('No vehicle position for takeoff')
        p = self.state.pose.position
        self.goto_position((p.x, p.y, altitude), yaw)

    def set_velocity(self, velocity, yaw: float = 0.) -> None:
        msg = self._target(FlightTarget.VELOCITY, yaw)
        msg.velocity.x, msg.velocity.y, msg.velocity.z = map(float, velocity)
        self.target_pub.publish(msg)

    def _target(self, mode: int, yaw: float) -> FlightTarget:
        self.sequence += 1
        msg = FlightTarget()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.mode, msg.yaw, msg.sequence = mode, float(yaw), self.sequence
        return msg
