import cv2
import numpy as np
import pytest
from drone_perception.detector import ColorDetector, PlantTracker


def test_rgb_pipeline_detects_plants_but_not_soil():
    image = np.full((240, 320, 3), (80, 55, 25), np.uint8)
    cv2.circle(image, (80, 100), 20, (20, 180, 20), -1)
    cv2.circle(image, (230, 100), 20, (220, 220, 10), -1)
    blobs = ColorDetector().detect(image)
    assert {b.classification for b in blobs} == {'healthy', 'stressed'}
    assert len(blobs) == 2
    with pytest.raises(ValueError):
        ColorDetector().detect(image.astype(float))


def test_tracking_uses_positions_not_ground_truth_ids():
    t = PlantTracker(.5)
    a = t.assign((4, 0, .65))
    assert t.assign((4.1, 0, .65)) == a
    assert t.assign((6, 0, .65)) != a
