import cv2
import numpy as np
from drone_docking.marker import MarkerDetector


def test_marker_pixel_size_gives_metric_distance_and_wrong_id_is_rejected():
    image = np.full((480, 640), 255, np.uint8)
    image[160:320, 240:400] = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250), 7, 160)
    rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    k = np.array([[400., 0., 320.], [0., 400., 240.], [0., 0., 1.]])
    pose = MarkerDetector().detect(rgb, k)
    assert pose is not None
    assert abs(pose.translation_optical[2] - 2.) < .03
    assert max(abs(v) for v in pose.translation_optical[:2]) < .01
    assert MarkerDetector(marker_id=8).detect(rgb, k) is None
