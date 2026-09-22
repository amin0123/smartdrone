# Architecture

```text
Isaac Sim 5.1 / PhysX / RTX
  field + dock + physical Iris
    ├── Pegasus sensor models ── MAVLink HIL TCP 4560 ── PX4 SITL
    │                                                │
    │                    rotor forces ← actuators ───┘
    └── RGB / depth / CameraInfo / clock ── ROS 2 bridge ──┐
                                                        v
PX4 uORB ←→ uXRCE client ← UDP 8888 → Agent ← DDS → drone_px4_interface
  │                                                       │ ENU state/API
  └── MAVLink UDP 14550 → QGroundControl                    v
                                               drone_mission_manager
                           drone_navigation ── route ──────┤
                           drone_safety ── decision ────────┤
                           drone_docking ── alignment ──────┤
                           drone_perception ── detections ──┴→ drone_inspection
                                                               JSON/SQLite
```

Isaac owns rigid-body physics, collisions and rendered sensors. Pegasus supplies
the vehicle, rotor model and simulated avionics bridge rather than a new flight
controller. PX4 owns estimation, stabilization, mixing, arming and native
failsafes. ROS owns mission decisions, planning, perception and high-level
position/velocity requests. QGC observes MAVLink and offers operator recovery.

Only `drone_px4_interface` uses PX4 topic layouts and frame conventions. Its
continuous offboard stream is conditional on fresh upstream intent and fresh
state. If it crashes, stops publishing, or loses its controller lease, PX4's
offboard-loss behavior must take over. Independent ROS safety decisions can
request return or land but never replace PX4 stabilization.

Use ENU `map`, FLU `base_link`, calibrated optical camera frames. Associate
observations with image-time poses, not the latest unrelated pose. Keep simulator
truth labels out of the RGB classifier. Reports distinguish approximate camera
ground intersection from drone position. Marker landing is optional until the
known-station approach passes its own gate.

The simulator uses its bundled Python. ROS nodes and pure-logic tests use the
`drone` environment and the native Jazzy ABI. Standard DDS messages isolate the
two Python versions. Simulation-only battery/charging and scene construction are
replaceable adapters; mission behavior must consume the same state on hardware.
