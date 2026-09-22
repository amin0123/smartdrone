"""Common ROS adapters; core algorithms never import this module."""
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from drone_core.models import Vehicle

SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


def parameter(node: Node, name: str, default):
    return node.declare_parameter(name, default).value


def steady_timer(node: Node, period: float, callback):
    return node.create_timer(period, callback, clock=Clock(clock_type=ClockType.STEADY_TIME))


def vehicle_from_msg(msg) -> Vehicle:
    p, v, q = msg.pose.position, msg.twist.linear, msg.pose.orientation
    return Vehicle((p.x, p.y, p.z), (v.x, v.y, v.z), (q.x, q.y, q.z, q.w),
                   msg.connected, msg.localized, msg.armed, msg.landed,
                   msg.offboard, msg.failsafe, msg.reset_counter)


def spin(factory) -> None:
    rclpy.init()
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
