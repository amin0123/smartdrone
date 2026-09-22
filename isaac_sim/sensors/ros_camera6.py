"""Isaac 6 Replicator RGB/depth → ROS; isolated from the ROS autonomy process."""
import numpy as np
import omni.replicator.core as rep
import omni.usd
from pxr import Gf, UsdGeom
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, CameraInfo


def stamp(seconds):
    ns = round(seconds*1e9)
    return Time(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)


class RosCamera6:
    def __init__(self, world, config, enabled=True):
        self.config, self.enabled = config, enabled
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('isaac_sensor_bridge')
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.clock = self.node.create_publisher(Clock, '/clock', 10)
        self.rgb = self.node.create_publisher(Image, '/drone/camera/image_raw', qos)
        self.depth = self.node.create_publisher(Image, '/drone/camera/depth', qos)
        self.info = self.node.create_publisher(CameraInfo, '/drone/camera/camera_info', qos)
        self.last_clock = self.last_image = -1.
        if enabled:
            path = '/World/quadrotor/body/down_camera'
            camera = UsdGeom.Camera.Define(world.stage, path)
            camera.AddTranslateOp().Set(Gf.Vec3d(*config['translation']))
            # USD identity camera: -Z forward, +Y up. Optical axes are +X,-Y,-Z.
            camera.CreateFocalLengthAttr(config['focal_length_mm'])
            camera.CreateHorizontalApertureAttr(config['horizontal_aperture_mm'])
            camera.CreateVerticalApertureAttr(config['horizontal_aperture_mm']*config['height']/config['width'])
            camera.CreateClippingRangeAttr(Gf.Vec2f(.005, 50.))
            self.product = rep.create.render_product(path, (config['width'], config['height']))
            self.rgb_annotator = rep.AnnotatorRegistry.get_annotator('rgb')
            self.depth_annotator = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
            self.rgb_annotator.attach([self.product])
            self.depth_annotator.attach([self.product])

    def initialize(self):
        pass  # Replicator initializes its render product on the first rendered frame.

    def step(self, sim_time, rendered):
        if sim_time-self.last_clock >= .019:
            self.clock.publish(Clock(clock=stamp(sim_time)))
            self.last_clock = sim_time
        rclpy.spin_once(self.node, timeout_sec=0.)
        if not self.enabled or not rendered or sim_time-self.last_image < 1/self.config['fps']:
            return
        rgba = self.rgb_annotator.get_data()
        if not isinstance(rgba, np.ndarray) or rgba.size == 0:
            return
        rgb = np.ascontiguousarray(rgba[:, :, :3], dtype=np.uint8)
        msg = self._image(rgb, 'rgb8', sim_time)
        self.rgb.publish(msg)
        info = CameraInfo()
        info.header, info.width, info.height = msg.header, msg.width, msg.height
        focal_pixels = self.config['width']*self.config['focal_length_mm']/self.config['horizontal_aperture_mm']
        k = np.array([[focal_pixels, 0., msg.width/2], [0., focal_pixels, msg.height/2], [0., 0., 1.]])
        info.k, info.r = k.flatten().tolist(), np.eye(3).flatten().tolist()
        info.p = np.column_stack((k, np.zeros(3))).flatten().tolist()
        info.distortion_model, info.d = 'plumb_bob', [0.]*5
        self.info.publish(info)
        depth = self.depth_annotator.get_data()
        if isinstance(depth, np.ndarray) and depth.size:
            self.depth.publish(self._image(np.ascontiguousarray(depth, dtype=np.float32), '32FC1', sim_time))
        self.last_image = sim_time

    def _image(self, data, encoding, sim_time):
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp(sim_time), 'down_camera_optical'
        msg.height, msg.width, msg.encoding = data.shape[0], data.shape[1], encoding
        msg.is_bigendian, msg.step, msg.data = False, int(data.strides[0]), data.tobytes()
        return msg

    def close(self):
        if self.enabled:
            self.rgb_annotator.detach([self.product])
            self.depth_annotator.detach([self.product])
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
