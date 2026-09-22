import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from drone_core.camera import PoseBuffer, TimedPose, ground_intersection, optical_point_to_map


def test_downward_camera_center_and_optical_axes():
    p = TimedPose(1, (4, 2, 3.04), (0, 0, 0, 1))
    k = [[100, 0, 50], [0, 100, 50], [0, 0, 1]]
    assert ground_intersection((50, 50), k, p, 0) == pytest.approx((4, 2, 0))
    assert ground_intersection((60, 50), k, p, 0) == pytest.approx((4.3, 2, 0))
    assert ground_intersection((50, 60), k, p, 0) == pytest.approx((4, 1.7, 0))
    assert optical_point_to_map((0, 0, 3), p) == pytest.approx((4, 2, 0))


def test_horizon_and_plane_behind_camera_are_rejected():
    p = TimedPose(1, (0, 0, 3), tuple(Rotation.from_euler('x', 180, degrees=True).as_quat()))
    assert ground_intersection((0, 0), np.eye(3), p, 0) is None
    assert ground_intersection((0, 0), np.eye(3), TimedPose(1, (0, 0, 0), (0, 0, 0, 1)), 1) is None


def test_timestamp_pairing_interpolates_and_rejects_stale_pose():
    b = PoseBuffer()
    b.add(TimedPose(1, (0, 0, 3), (0, 0, 0, 1)))
    b.add(TimedPose(1.2, (2, 0, 3), (0, 0, 0, 1)))
    assert b.at(1.1).position == pytest.approx((1, 0, 3))
    assert b.at(3) is None
    b.add(TimedPose(10, (4, 0, 3), (0, 0, 0, 1)))
    assert b.at(1.2) is None
