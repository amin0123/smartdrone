"""RGB inspection adapter; replace ColorDetector while preserving the output contract."""
from pathlib import Path
import re
import time
import cv2
from rclpy.node import Node
from std_msgs.msg import Empty
from drone_interfaces.msg import DetectionArray, InspectionDetection, MissionStatus
from drone_core.camera import ground_intersection
from drone_core.ros_camera import CameraInput
from drone_core.ros_support import LATCHED_QOS, parameter, spin
from .detector import ColorDetector, PlantTracker


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('plant_perception')
        self.detector = ColorDetector(
            parameter(self, 'perception.healthy_hsv_lower', [35, 60, 35]),
            parameter(self, 'perception.healthy_hsv_upper', [95, 255, 255]),
            parameter(self, 'perception.stressed_hsv_lower', [25, 80, 40]),
            parameter(self, 'perception.stressed_hsv_upper', [34, 255, 255]),
            parameter(self, 'perception.min_area_pixels', 60))
        self.radius = parameter(self, 'inspection_records.dedup_distance', .8)
        self.tracker = PlantTracker(self.radius)
        self.plane_z = parameter(self, 'perception.ground_height', .65)
        self.interval = parameter(self, 'perception.save_interval', 1.)
        self.directory = Path(parameter(self, 'inspection_records.directory', 'logs/inspection')).expanduser().resolve()
        self.mission_id, self.phase = '', 'IDLE'
        self.mission_at = self.last_saved = float('-inf')
        self.pub = self.create_publisher(DetectionArray, '/drone/detections', 10)
        self.heartbeat = self.create_publisher(Empty, '/drone/perception_heartbeat', 10)
        self.create_subscription(MissionStatus, '/drone/mission_status', self._mission, LATCHED_QOS)
        self.camera = CameraInput(self, self._frame)

    def _mission(self, msg):
        if msg.mission_id and not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', msg.mission_id):
            return
        if self.mission_id != msg.mission_id:
            self.tracker = PlantTracker(self.radius)
            self.last_saved = float('-inf')
        self.mission_id, self.phase, self.mission_at = msg.mission_id, msg.state, time.monotonic()

    def _frame(self, frame):
        # A live process with a stopped camera is unhealthy: publish only after
        # an actual fresh image, calibration and associated pose arrive.
        self.heartbeat.publish(Empty())
        if self.phase != 'INSPECTION' or not self.mission_id or time.monotonic()-self.mission_at > 1.:
            return
        if frame.pose.stamp - self.last_saved < self.interval:
            return
        blobs = self.detector.detect(frame.rgb)
        output = DetectionArray()
        output.header = frame.message.header
        for blob in blobs:
            position = ground_intersection(blob.center, frame.intrinsics, frame.pose,
                                           self.plane_z, self.camera.translation)
            if position is None:
                continue
            msg = InspectionDetection()
            msg.header, msg.mission_id = output.header, self.mission_id
            msg.plant_id = self.tracker.assign(position)
            msg.classification, msg.confidence = blob.classification, float(blob.confidence)
            msg.drone_pose.position.x, msg.drone_pose.position.y, msg.drone_pose.position.z = map(float, frame.pose.position)
            q = msg.drone_pose.orientation
            q.x, q.y, q.z, q.w = map(float, frame.pose.quaternion)
            msg.approximate_position.x, msg.approximate_position.y, msg.approximate_position.z = map(float, position)
            msg.position_valid = True
            output.detections.append(msg)
        if not output.detections:
            return
        directory = self.directory / self.mission_id / 'images'
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f'{round(frame.pose.stamp*1e9):019d}.jpg'
        if not cv2.imwrite(str(path), cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)):
            raise OSError(f'Failed to save inspection image: {path}')
        for detection in output.detections:
            detection.image_path = str(path)
        self.pub.publish(output)
        self.last_saved = frame.pose.stamp


def main():
    spin(PerceptionNode)
