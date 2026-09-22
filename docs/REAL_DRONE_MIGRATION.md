# From simulation to a physical drone

The reusable boundary is the high-level ROS API, not the Isaac script. Mission
states, route generation, records, safety policy and detector adapters can stay.
PX4 continues to own motor control on the real flight controller.

```text
SIMULATION                              HARDWARE
Isaac RGB/depth + CameraInfo             Camera driver RGB/depth + calibration
         ↓                                          ↓
      perception → geolocation → inspection records (same contracts)
         ↓                                          ↓
           mission / navigation / safety / docking
         ↓                                          ↓
PX4 gateway → XRCE UDP → SITL            PX4 gateway → XRCE serial/UDP → autopilot
         ↓                                          ↓
Pegasus rotors + PhysX                   ESCs + motors + airframe
```

| Component | Hardware change |
|---|---|
| Flight controller | A PX4-compatible controller with appropriate IMU, barometer, power module and supported maintained firmware. Re-audit firmware, px4_msgs and Agent together. |
| Companion computer | A Linux computer such as a Jetson-class device, sized for the actual model/camera workload. Audit JetPack, Ubuntu, ROS and Python compatibility separately; desktop Jazzy assumptions do not automatically hold. |
| Camera | A calibrated downward RGB camera with a stable mount, driver timestamps and measured body extrinsic. Depth/LiDAR may be necessary for obstacles and height. |
| Agricultural sensing | Add NIR/multispectral cameras and radiometric calibration only when the application needs them; preserve band metadata and registration. |
| Positioning | GNSS outdoors; greenhouse GNSS may be inadequate. Evaluate visual-inertial odometry, motion capture, UWB or another measured solution and fuse it using PX4's supported estimator interfaces. |
| Battery | Use `battery.source=px4` with a calibrated physical power module, measured discharge under load and reserve for wind/return/landing. Remove the simulation injection service from deployment. |
| Dock/charger | Add physical contact detection, charger control, electrical isolation, temperature/current checks and a charging handshake. Position plus landed flags is not proof that charging contacts connected. |
| QGC | Keep an independent monitored MAVLink connection and an operator recovery path. |

Before installing hardware software, create a new compatibility profile. The
historical PX4 1.14 reference in this repository is a reproducible simulator
baseline, not a deployment recommendation. Match consumed message definitions,
QoS, command IDs and failsafe semantics again after upgrading firmware.

In hardware use actual sensor clocks and validated time synchronization. Set
`use_sim_time=false`, configure the appropriate gateway timestamp mode (normally
`system` with Agent synchronization), and check end-to-end timestamp errors.
The simulation's zero timestamp requests PX4 receive time to avoid slow-renderer
clock drift; it is not a replacement for sensor synchronization on hardware.
Match `perception.reliable_images` to the camera driver's offered QoS. Disable
simulation-only scene/charging launch components.

Calibrate ENU/NED and FLU/FRD with named-axis experiments before flight. Verify
camera optical axes and translation, station coordinates, GNSS/map origin and
height datum. The current known-dock planner assumes one consistent local map;
it cannot use arbitrary latitude/longitude values as metres. Estimator position
resets invalidate a stored local route. Use a surveyed map transform or explicit
relocalization policy for persistent maps across power cycles.

Replace synthetic color classes with an agricultural model validated on the
intended crops, lighting and camera. Keep uncertainty and unknown observations;
do not equate high mask confidence with disease probability. Replace the canopy
plane approximation when height variation or localization error exceeds the
inspection requirements. Validate calibrated depth/terrain registration first.

An appropriate qualification sequence is: bench communication and timestamp
checks; motors disabled arming/failsafe checks; manual stabilized flight and
position hold; bounded offboard hover; short routes; operator-triggered return;
mission return; sensor/link/controller failure trials; known-station landing;
marker landing; finally charger interlocks. Keep physical recovery controls and
an operational boundary appropriate to the site. A passing SITL mission cannot
establish real aerodynamics, sensor reliability, wind margins or docking safety.
