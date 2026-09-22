from rclpy.node import Node
from drone_interfaces.msg import DockObservation
from drone_core.camera import optical_point_to_map
from drone_core.ros_camera import CameraInput
from drone_core.ros_support import SENSOR_QOS, parameter, spin
from .marker import MarkerDetector


class DockMarkerNode(Node):
    def __init__(self):
        super().__init__('dock_marker')
        self.detector = MarkerDetector(parameter(self, 'dock.marker_id', 7),
                                       parameter(self, 'dock.marker_size', .8))
        self.pub = self.create_publisher(DockObservation, '/drone/dock_observation', SENSOR_QOS)
        self.camera = CameraInput(self, self._frame)

    def _frame(self, frame):
        marker = self.detector.detect(frame.rgb, frame.intrinsics, frame.distortion)
        msg = DockObservation()
        msg.header.stamp, msg.header.frame_id = frame.message.header.stamp, 'map'
        msg.visible, msg.marker_id = marker is not None, self.detector.marker_id
        if marker:
            position = optical_point_to_map(marker.translation_optical, frame.pose, self.camera.translation)
            msg.station_position.x, msg.station_position.y, msg.station_position.z = map(float, position)
            msg.reprojection_error = marker.reprojection_error
        self.pub.publish(msg)


def main():
    spin(DockMarkerNode)
