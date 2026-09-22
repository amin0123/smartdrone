# Autonomous agricultural drone inspection

A PX4-controlled quadrotor inspects synthetic crop rows in Isaac Sim, records
RGB plant observations with approximate locations, returns to its station, lands,
and simulates recharging. ROS 2 supplies autonomy; Pegasus supplies the multirotor
and sensor bridge; **PX4 controls the aircraft**.

The project has flown the complete mission on this workstation: 15 plants
(11 healthy, 4 simulated stressed), autonomous return, marker-assisted landing,
disarm and recharge to READY. Separate Stage 1 and low/critical-battery trials
also passed. This is an engineering simulation with explicit test evidence,
not a flight-qualified hardware system. See [current status](docs/PROJECT_STATUS.md)
for the recorded simulator and failure-trial results.

**Runtime choice matters.** The original documented Isaac 5.1/Pegasus 5.1 pairing
crashes with this host's NVIDIA R595 driver. The working profile uses the existing
Isaac 6 build and an explicitly **experimental, pinned Pegasus port**. No driver
was replaced. Read [COMPATIBILITY.md](docs/COMPATIBILITY.md) before installing or
changing versions.

## Architecture

```text
Isaac Sim / PhysX / RTX
  ├─ agricultural field, station, physical Iris
  ├─ Pegasus sensors ── MAVLink HIL ── PX4 SITL ── actuators ── Pegasus rotors
  └─ RGB + depth + calibration + clock ────────────────┐
                                                    v
PX4 uORB ↔ XRCE client ↔ Micro XRCE-DDS Agent ↔ ROS 2 gateway
  └─ MAVLink → QGroundControl                    ↕ ENU state/targets
                               navigation → mission manager ← safety/battery
                               camera → perception → inspection SQLite/JSON
                               marker detector → docking → simulated charging
```

[Architecture](docs/ARCHITECTURE.md) explains ownership and data flow.
[Coordinate frames](docs/COORDINATE_FRAMES.md) explains ENU/FLU ↔ NED/FRD and camera
geometry. Simulator-specific code is isolated from the ROS mission API.

## Stack and installation

Ubuntu 24.04, ROS 2 Jazzy, conda **drone** / Python 3.12.11, PX4 1.14.3 with
matching `px4_msgs`, XRCE Agent 2.4.2, QGC 5.0.8, OpenCV 4.10.0.84. Source commits,
ABI boundaries, build repairs and simulator profiles are in the
[compatibility audit](docs/COMPATIBILITY.md).

Follow [INSTALLATION.md](docs/INSTALLATION.md) for a new machine. On this prepared
workstation, source the project environment in each ROS/development terminal:

```bash
cd /home/amin24/orca/smartdrone
source scripts/activate.sh
python scripts/doctor.py --profile 6
```

Isaac must use its own bundled Python in an isolated child process. Its wrapper
can be invoked from `drone`; ROS nodes, build tools and tests use `drone`.

## First launch and autonomous demonstration

From the repository root, in separate terminals:

```bash
# Terminal 1: explicit experimental Isaac 6 profile, cameras and PX4 child.
# Omit --headless to see the field in the Isaac window.
bash scripts/run_sim6.sh --headless --cameras
```

```bash
# Terminal 2: all ROS nodes plus the DDS Agent. Wait for PX4/localization readiness.
source scripts/activate.sh
ros2 launch drone_bringup full_system.launch.py
```

```bash
# Terminal 3: QGroundControl monitor. No flight sticks are required.
bash scripts/run_qgc.sh
```

```bash
# Terminal 4: starts exactly one mission, observes it, writes pass/fail evidence.
source scripts/activate.sh
python scripts/mission_demo.py --output logs/my-mission.json
```

The autonomous sequence is preflight → arm → takeoff → crop-row coverage → return
→ station approach/landing → disarm → charge → READY. Starting another mission
requires another request unless `mission.auto_repeat` is enabled. The demo is an
observer after its start request; closing it does **not** abort the mission.

The same start request without an observer:

```bash
ros2 service call /drone/start_mission std_srvs/srv/Trigger '{}'
# Request a controlled return/abort while the mission node is running:
ros2 service call /drone/abort_mission std_srvs/srv/Trigger '{}'
```

Results are saved under `logs/inspection/<mission_id>/`: images, SQLite records,
`summary.json`, state transitions and docking traces. Read
[PERCEPTION.md](docs/PERCEPTION.md) for projection assumptions and plant IDs.
[DOCKING.md](docs/DOCKING.md) explains known-station and marker-assisted landing.
The ROS simulated battery is separate from the native PX4 battery shown in QGC.

Stop the ROS launch and simulator after landed/disarmed confirmation. A ROS
controller/gateway failure hands control to the documented PX4 native landing
failsafe. Stopping the simulator itself stops the physics world; it is not an
in-flight recovery command. The simulator owns and terminates only its PX4 child.

## Configuration and extension

Edit `config/mission.yaml` for area, altitude, speed, geofence, battery thresholds,
dock and detector settings; edit `config/world.yaml` for scene/sensors. Restart
nodes after changing configuration. Parameters are read at startup. To preserve
the default, copy the YAML and pass `params:=/absolute/path/to/copy.yaml` to launch.
Keep the scene and navigation geometry consistent. Routes crossing the inflated
obstacles are rejected before arming; dynamic obstacle avoidance is future work.

`dock.precision_enabled` selects marker refinement; the known-station fallback
remains available. The RGB detector uses actual rendered pixels. Synthetic labels
are only evaluation truth. Replace its detector through the stable detection
message contract to add a trained model. RGB is not a physically accurate NIR
simulation; [future imaging strategies](docs/PERCEPTION.md#nir-and-multispectral-extension)
are documented separately.

## Repository

```text
config/             parameter YAML, exact source/Python/ROS inventories
ros2_ws/src/        interfaces, PX4 gateway, mission, navigation, perception,
                    inspection, docking, safety, bringup and shared drone_core
isaac_sim/          scene, vehicle/physics bridge and camera adapters
px4/config/         SITL-only PX4 startup parameters
scripts/            isolated setup, builds, launch wrappers, diagnostics
patches/            small, explained patches to pinned upstream sources
tests/              pure logic and ROS executive contract tests
docs/               design explanations, runbooks, evidence and status
logs/               local flight/inspection data (ignored by Git)
vendor/, .local/    downloaded sources/runtimes/builds (ignored by Git)
```

## Testing and study guide

```bash
source scripts/activate.sh
python -m pytest -q
python scripts/check_px4_schemas.py
```

[TESTING.md](docs/TESTING.md) defines all 15 gates and real-simulator fault trials.
Use `bash scripts/debug.sh rviz`, `bash scripts/debug.sh record`, or
`bash scripts/debug.sh topics` for observation. See [troubleshooting](docs/TROUBLESHOOTING.md)
for the actual failures diagnosed during development.

Subsystem explanations:
[PX4/ROS](docs/PX4_ROS2.md), [Isaac](docs/ISAAC_SIM.md),
[Pegasus](docs/PEGASUS.md), [navigation](docs/NAVIGATION.md),
[perception](docs/PERCEPTION.md), [mission states](docs/MISSION_MANAGER.md),
[safety](docs/SAFETY.md), [docking](docs/DOCKING.md),
[real-drone migration](docs/REAL_DRONE_MIGRATION.md), and
[implementation roadmap](docs/ROADMAP.md).
