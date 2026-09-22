# Perception and inspection records

The first detector proves the complete RGB pipeline. It recognizes deliberately
green and yellow simulated plants; it does not diagnose agricultural disease.
Yellow means `stressed` only in this synthetic experiment. The detector never
reads the simulator's `inspection:classification` USD attributes or label JSON.

```text
USD downward camera → RTX RGB + CameraInfo → CameraInput
                                      PX4 ENU poses → image-time interpolation
                                                     ↓
                               HSV masks → connected contours → plant IDs
                                                     ↓
                                calibrated canopy-plane ray intersection
                                                     ↓
                              DetectionArray → SQLite + JPEG → summary.json
```

`drone_core.ros_camera.CameraInput` checks encoding, row stride, dimensions,
frame identity, finite intrinsics, image age and pose availability. It handles
RGB/BGR/RGBA/BGRA/mono and rectifies calibrated distorted images before processing.
`PoseBuffer` interpolates positions and uses quaternion SLERP when both bounding
poses are close enough; otherwise it accepts only a nearby pose. An unavailable
pose causes an observation to be skipped, never assigned to an invented location.

`ColorDetector` converts RGB into OpenCV HSV (hue 0–179), thresholds configured
healthy/stressed ranges, removes small mask noise, and rejects very small or
very large contours. Its confidence describes mask fill, not a calibrated
disease probability. Tune thresholds against saved **rendered** frames because
lighting and tone mapping change material colors.

The pixel centroid and camera intrinsics define an optical ray. The known
camera-to-body transform and PX4 attitude rotate it into ENU. Intersecting this
ray with the configured canopy height gives an approximate plant position.
This is better than labelling every detection with the drone's location, but
height variation, occlusion, segmentation errors, image latency and localization
errors still affect accuracy. Store both drone position and target position.
Depth is published separately and can replace the plane assumption after its
registration, invalid pixels and uncertainty are validated.

`PlantTracker` associates nearby map positions and creates mission-local IDs;
these are observation clusters, not persistent biological plant identities.
Change `inspection_records.dedup_distance` when plant spacing or localization
accuracy changes. Observations are saved at `perception.save_interval` during
INSPECTION, while camera health continues to be monitored outside that state.
The heartbeat requires an actual fresh image, calibration and associated pose.
A running process with a frozen camera does not look healthy.

The recorder commits each detection to SQLite in WAL mode. Mission transitions
and their reasons are also saved. JPEGs live under a unique mission directory.
On DOCKED or MISSION_ABORTED it writes an atomic JSON summary, and updates the
summary on READY. It keeps repeated observations but counts unique plant IDs,
preferring the most severe observed class for each summary entry. A recorder
crash leaves already committed rows recoverable; loss of recorder heartbeat
requests return during inspection. Check disk space before long campaigns.

Example record (ENU metres, image time in simulation seconds):

```json
{
  "timestamp": "2026-09-22T21:00:00+00:00",
  "image_timestamp": 45.2,
  "plant_id": "plant_0003",
  "classification": "stressed",
  "confidence": 0.91,
  "drone_position": {"x": 6.1, "y": -3.0, "z": 3.0},
  "approximate_position": {"x": 6.0, "y": -3.1, "z": 0.65},
  "image_path": "/project/logs/inspection/mission_id/images/frame.jpg",
  "mission_id": "mission_id",
  "location_method": "calibrated_camera_canopy_plane_intersection"
}
```

To replace the detector, preserve the `DetectionArray` contract: original image
timestamp, ENU `map` coordinates, mission identifier, classification, confidence,
validity and image evidence. A YOLO/segmentation adapter can replace blob
generation without changing mission logic or storage. For real disease models,
version the model and taxonomy, calibrate confidence, define an unknown class,
and validate on the intended crops and sensors rather than transferring these
color thresholds.

## NIR and multispectral extension

An RGB material color or grayscale image is **not** a physically accurate NIR
measurement. Useful future options include separately synthesized bands with
explicit assumptions; measured leaf/soil spectral reflectances and illumination
fed to a spectral renderer; replay of calibrated, registered multispectral
datasets; or a learned synthetic sensor validated against real paired data.
Keep bands, wavelength ranges, radiometric units, exposure and calibration
metadata distinct. Synchronize them with RGB and geometry. Do not claim physical
NDVI from arbitrary RGB channel values. The present mission remains RGB-only.
