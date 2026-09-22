"""Persist observations immediately; summarize after landing and after charging."""
from pathlib import Path
from rclpy.node import Node
from std_msgs.msg import Empty
from drone_interfaces.msg import DetectionArray, MissionStatus
from drone_core.ros_support import LATCHED_QOS, parameter, steady_timer, spin
from .store import InspectionRecord, MissionStore, utc_now


def position_dict(point):
    return {'x': point.x, 'y': point.y, 'z': point.z}


class RecorderNode(Node):
    def __init__(self):
        super().__init__('inspection_recorder')
        self.store = MissionStore(Path(parameter(self, 'inspection_records.directory', 'logs/inspection')))
        self.last_status = None
        self.pub = self.create_publisher(Empty, '/drone/recorder_heartbeat', 10)
        self.create_subscription(DetectionArray, '/drone/detections', self._detections, 10)
        self.create_subscription(MissionStatus, '/drone/mission_status', self._mission, LATCHED_QOS)
        steady_timer(self, .5, self._heartbeat)

    def _heartbeat(self):
        self.store.db.execute('SELECT 1').fetchone()
        self.pub.publish(Empty())

    def _mission(self, msg):
        key = (msg.mission_id, msg.state, msg.reason)
        if not msg.mission_id or self.last_status == key:
            return
        self.store.start(msg.mission_id)
        self.store.transition(*key)
        self.last_status = key
        if msg.state in {'DOCKED', 'READY', 'MISSION_ABORTED'}:
            path = self.store.summarize(*key)
            self.get_logger().info(f'Mission report: {path}')

    def _detections(self, msg):
        for d in msg.detections:
            if not d.position_valid:
                continue
            record = InspectionRecord(utc_now(), d.header.stamp.sec + d.header.stamp.nanosec*1e-9,
                d.plant_id, d.classification, d.confidence, position_dict(d.drone_pose.position),
                position_dict(d.approximate_position), d.image_path, d.mission_id)
            self.store.add(record)

    def destroy_node(self):
        self.store.close()
        return super().destroy_node()


def main():
    spin(RecorderNode)
