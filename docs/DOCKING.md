# Landing, docking and simulated charging

Stage 1 uses the known ENU station position. The return route first establishes
safe transit altitude and reaches the point above the dock. The executive then
descends to approach altitude while retaining position control in PX4. A bounded
reference moves downward only after horizontal alignment. Near the surface it
hands landing to PX4's native LAND mode, stops offboard intent and waits for
land detection and disarming. No ROS node commands individual rotor speeds.

Dock contact requires all of: fresh connection/localization, landed flag,
disarmed flag, horizontal position inside the station radius and vertical
position inside its height tolerance. A touchdown elsewhere does not charge.
These conditions are centralized in `dock_contact`, shared by mission and battery
nodes. They are simulation criteria, not proof of real electrical contact.

Stage 2 adds the downward RGB camera and an ArUco **DICT_6X6_250, ID 7**, 0.8 m
outer black square. `config/dock_marker.json` contains its exact cells; the
simulator renders those cells as geometry on a white pad. `MarkerDetector` uses
calibrated intrinsics and metric square PnP to estimate marker translation in the
optical frame. Reprojection error, positive depth and ID checks reject bad poses.
The shared extrinsic/attitude conversion yields station position in ENU.

```text
RGB + CameraInfo + image-time pose
             ↓
ArUco corners → metric PnP → optical station translation
             ↓
body extrinsic + vehicle attitude → map station estimate
             ↓
freshness + consistency gate → align XY → bounded descent → native LAND
```

`dock.precision_enabled` now defaults to true after both landing stages passed
actual simulator trials. Set it to false to exercise the established Stage 1. When
enabled, the controller accepts only recent marker positions close to the known
station. It falls back to the known station if the marker is missing, stale or
inconsistent; a visual outlier cannot redirect landing arbitrarily. Descent stops
while horizontal error exceeds tolerance. It maintains a descending reference
with bounded vertical lead, rather than repeatedly setting a tiny offset from
the current altitude and inadvertently making descent almost stop.

This is a bounded visual refinement of a known dock, not general blind landing
on an unknown moving platform. The marker can leave the field of view near the
ground; the final LAND handover is deliberate. Evaluate alignment error, marker
visibility and touchdown position in the flight evidence. For real docking add
physical target pose calibration, contact/charger sensing, wind compensation,
landing-gear geometry and a charger interlock.

After stable contact, DOCKED transitions to CHARGING. In simulation only, the
battery adapter increases percentage at `battery.charge_per_second` while all
contact checks remain true and the CHARGING status remains fresh. It clamps at
100%. `battery.ready_percent` determines READY. The next mission does not start
automatically by default. The ROS battery model's percentage is independent of
PX4's native simulated battery displayed in QGC; both remain visible and are
explicitly labelled. For hardware, use `battery.source=px4` and replace charging
simulation with a real charger/contact interface.
