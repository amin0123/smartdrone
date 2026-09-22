"""ROS camera input with explicit encoding, stride and image-time pose checks."""
from dataclasses import dataclass
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy
from drone_interfaces.msg import VehicleState
from .camera import PoseBuffer, TimedPose
from .ros_support import SENSOR_QOS, parameter


def seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def image_to_rgb(msg) -> np.ndarray:
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(msg.encoding)
    if channels is None or min(msg.height, msg.width) <= 0:
        raise ValueError(f'Unsupported image: {msg.encoding}')
    if msg.step < msg.width * channels or len(msg.data) != msg.height * msg.step:
        raise ValueError('Image stride or payload size is invalid')
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, :msg.width*channels].reshape(msg.height, msg.width, channels)
    if channels == 1:
        return np.repeat(pixels, 3, axis=2)
    rgb = pixels[:, :, :3]
    if msg.encoding in {'bgr8', 'bgra8'}:
        rgb = rgb[:, :, ::-1]
    return np.ascontiguousarray(rgb)


@dataclass(frozen=True)
class CameraFrame:
    message: object
    rgb: np.ndarray
    pose: TimedPose
    intrinsics: np.ndarray
    distortion: np.ndarray


class CameraInput:
    def __init__(self, node, callback):
        self.node, self.callback = node, callback
        self.poses, self.info = PoseBuffer(), None
        self.reset_counter = None
        self.last_stamp = float('-inf')
        self.accepted_at = time.monotonic()
        self.pose_tolerance = parameter(node, 'perception.pose_tolerance', .2)
        self.translation = tuple(parameter(node, 'perception.camera_translation', [0., 0., -.04]))
        reliable = parameter(node, 'perception.reliable_images', True)
        image_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT)
        self.image_sub = node.create_subscription(Image, parameter(node, 'perception.image_topic',
            '/drone/camera/image_raw'), self._image, image_qos)
        self.info_sub = node.create_subscription(CameraInfo, parameter(node, 'perception.camera_info_topic',
            '/drone/camera/camera_info'), self._info, SENSOR_QOS)
        self.pose_sub = node.create_subscription(VehicleState, '/drone/state', self._state, SENSOR_QOS)

    def _info(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0 and np.isfinite(msg.k).all():
            self.info = msg

    def _state(self, msg):
        if not msg.connected or not msg.localized:
            return
        if self.reset_counter != msg.reset_counter:
            self.poses = PoseBuffer()
            self.reset_counter = msg.reset_counter
        p, q = msg.pose.position, msg.pose.orientation
        self.poses.add(TimedPose(seconds(msg.header.stamp), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))

    def _image(self, msg):
        stamp = seconds(msg.header.stamp)
        age = self.node.get_clock().now().nanoseconds * 1e-9 - stamp
        if not -.1 <= age <= .5 or stamp <= self.last_stamp:
            self._reject(f'time: image_age={age:.3f}s stamp={stamp:.3f} previous={self.last_stamp:.3f}')
            return
        if self.info is None or (msg.width, msg.height, msg.header.frame_id) != (
                self.info.width, self.info.height, self.info.header.frame_id):
            self._reject('calibration/frame identity unavailable')
            return
        pose = self.poses.at(stamp, self.pose_tolerance)
        if pose is None:
            nearest = min((abs(p.stamp-stamp) for p in self.poses.samples), default=float('inf'))
            self._reject(f'no image-time pose: nearest={nearest:.3f}s')
            return
        try:
            rgb = image_to_rgb(msg)
        except ValueError as exc:
            self.node.get_logger().error(str(exc), throttle_duration_sec=5.)
            return
        self.last_stamp = stamp
        self.accepted_at = time.monotonic()
        k = np.asarray(self.info.k).reshape(3, 3)
        d = np.asarray(self.info.d)
        if d.size and np.any(d):
            # Downstream pixel geometry uses the same rectified K.
            rgb = cv2.undistort(rgb, k, d)
        self.callback(CameraFrame(msg, rgb, pose, k, np.zeros(5)))

    def _reject(self, reason):
        if time.monotonic()-self.accepted_at > 2.:
            self.node.get_logger().warning('Camera input rejected: '+reason, throttle_duration_sec=5.)
