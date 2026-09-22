"""Isaac's native camera API → standard ROS messages, with no PX4 dependencies."""
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, CameraInfo
from isaacsim.sensors.camera import Camera


def stamp(seconds: float) -> Time:
    ns = round(seconds * 1_000_000_000)
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class RosCamera:
    def __init__(self, world, config: dict, enabled: bool = True):
        self.config, self.enabled = config, enabled
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('isaac_sensor_bridge')
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        self.rgb_pub = self.node.create_publisher(Image, '/drone/camera/image_raw', qos)
        self.depth_pub = self.node.create_publisher(Image, '/drone/camera/depth', qos)
        self.info_pub = self.node.create_publisher(CameraInfo, '/drone/camera/camera_info', qos)
        self.last_clock = self.last_capture = -1.
        self.camera = None
        if enabled:
            self.camera = Camera('/World/quadrotor/body/down_camera', name='down_camera',
                frequency=config['fps'], resolution=(config['width'], config['height']))
            self.camera.set_local_pose(translation=np.array(config['translation']),
                                      orientation=np.array([1., 0., 0., 0.]), camera_axes='usd')
            self.camera.set_focal_length(config['focal_length_mm'])
            self.camera.set_horizontal_aperture(config['horizontal_aperture_mm'])
            self.camera.set_vertical_aperture(config['horizontal_aperture_mm'] * config['height'] / config['width'])
            self.camera.set_clipping_range(.05, 50.)

    def initialize(self):
        if self.camera:
            self.camera.initialize()
            self.camera.add_distance_to_image_plane_to_frame()

    def step(self, sim_time: float, rendered: bool):
        if sim_time - self.last_clock >= .019:
            self.clock_pub.publish(Clock(clock=stamp(sim_time)))
            self.last_clock = sim_time
        rclpy.spin_once(self.node, timeout_sec=0.)
        if not self.camera or not rendered:
            return
        frame = self.camera.get_current_frame()
        capture_time = frame.get('rendering_time', sim_time)
        if capture_time <= self.last_capture or capture_time - self.last_capture < .95 / self.config['fps']:
            return
        rgba = self.camera.get_rgba()
        if rgba is None or rgba.size == 0:
            return
        self.last_capture = capture_time
        rgb = np.ascontiguousarray(rgba[:, :, :3], dtype=np.uint8)
        image = self._image(rgb, 'rgb8', capture_time)
        self.rgb_pub.publish(image)
        info = CameraInfo()
        info.header, info.width, info.height = image.header, image.width, image.height
        k = self.camera.get_intrinsics_matrix().astype(float)
        info.k = k.flatten().tolist()
        info.r = np.eye(3).flatten().tolist()
        info.p = np.column_stack((k, np.zeros(3))).flatten().tolist()
        info.distortion_model, info.d = 'plumb_bob', [0.] * 5
        self.info_pub.publish(info)
        depth = frame.get('distance_to_image_plane')
        if depth is not None and depth.size:
            depth = np.asarray(depth, dtype=np.float32)
            self.depth_pub.publish(self._image(depth, '32FC1', capture_time))

    def _image(self, array, encoding, capture_time):
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp(capture_time), 'down_camera_optical'
        msg.height, msg.width, msg.encoding = array.shape[0], array.shape[1], encoding
        msg.is_bigendian = False
        msg.step = int(array.strides[0])
        msg.data = array.tobytes()
        return msg

    def close(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
