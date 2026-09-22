# Testing and acceptance evidence

Tests are layered. Pure/ROS-contract tests check algorithms and decisions.
Actual Isaac/Pegasus/PX4 trials check sensor/actuator timing and aircraft response.
Neither substitutes for the other. Current observed results and immutable source
versions are in PROJECT_STATUS.md and COMPATIBILITY.md. Raw runtime evidence is
kept in ignored `logs/`; concise summaries are checked in under `docs/evidence/`.

## Automated tests

```bash
source scripts/activate.sh
python -m pytest -q
python scripts/check_px4_schemas.py
bash scripts/build_ros.sh
```

The suite covers ENU/NED and FLU/FRD axes and quaternion roundtrips, malformed
coordinates, lawnmower boundary/spacing/speed/collision behavior, battery reserve
and charge decisions, legal/illegal mission transitions, safety escalation and
latching, controller lease expiry/replay, calibrated camera projection and
image-time interpolation, RGB masks, marker PnP, station contact/descent and
SQLite/JSON serialization. The process-lifetime test starts its own harmless
child and proves it cannot survive its parent.

`test_executive.py` runs the real mission handlers with an explicitly ideal
vehicle API test double and controllable clock on ROS domain 93. It checks the
complete state sequence and battery abort/return decisions. It does **not**
validate dynamics, PX4, rendering or landing accuracy. Generated ROS interfaces
must already be built; otherwise those ROS-specific tests are skipped. Check the
reported skip count, not just the exit status.

## Fifteen simulator gates

Use the **explicit profile 6** on this workstation. Re-qualify the gates in order
when changing the simulator, aircraft, PX4/schema version or timing settings.

| Gate | Acceptance and verification |
|---|---|
| 1. Spawn | `run_sim6.sh --headless --no-px4 --duration 5`: physical Iris rests on the dock, scene builds, no physics callback error. |
| 2. PX4 link | Start normal simulator. PX4 initializes estimator from Pegasus sensors and returns HIL actuators; state changes track physical motion. |
| 3. QGC | Run QGC 5.0.8. Observe connected PX4 1.14.3, mode, local/global position and native battery. A running process alone is insufficient. |
| 4. Native flight | With ROS flight controllers stopped, run `python scripts/flight_smoke.py`. Normal arm → 3 m relative takeoff within 0.3 m → native land → disarm. No force arm. |
| 5. ROS state | Start Agent and gateway; `/drone/state` reports connected/localized and native preflight state. Ten wire schemas must match. |
| 6. Offboard | Mission prestreams, requests OFFBOARD, receives confirmation, arms and takes off without offboard-loss. Inspect ULog/ACKs on failure. |
| 7. Waypoints | Actual estimated position reaches route waypoints within tolerance; targets do not skip corners just because time elapsed. |
| 8. Cameras | RGB, 32FC1 metric depth and CameraInfo arrive with optical frame and simulation timestamps; image-time pose is available. |
| 9. Perception | Real rendered RGB detects green/yellow plants, saves images and approximate map positions. Truth labels are excluded from runtime perception. |
| 10. Coverage | A normal mission records `coverage_complete`, covers the three crop rows and resolves 15 plant clusters. |
| 11. Low battery | `mission_demo.py --scenario low-battery` injects 25% at inspection entry. `battery_return_reserve` causes return, station landing and recharge. |
| 12. Stage 1 docking | With `dock.precision_enabled: false`, complete known-station approach/native landing and confirm fresh landed + disarmed + station contact. |
| 13. Stage 2 docking | Enable precision. `docking.jsonl` must contain actual `visual_marker` control updates, not merely marker detections, and end landed/disarmed near the station. |
| 14. Charging | With depleted battery, percentage rises only while contact is confirmed and CHARGING is fresh; READY occurs at configured threshold. |
| 15. Complete mission | Normal request → coverage/images/report → visual docking → disarm → recharge → READY; repeat a sortie without resetting the simulator. |

Run the complete mission with:

```bash
python scripts/mission_demo.py --output logs/normal-evidence.json
```

The JSON contains every observed transition, sampled aircraft state and battery,
marker visibility and explicit pass/fail. Normal success requires
`coverage_complete`, final READY, landed and disarmed; a safe return caused by
camera loss does not pass normal coverage. The final report's healthy/stressed
counts are synthetic classifications, not biological diagnoses. Plant IDs are
mission-local clusters; compare locations with `logs/scene_labels.json` for
synthetic evaluation.

## Battery and process failures in the simulator

Run only one trial/controller at a time on the isolated project SITL. Start the
normal simulator and ROS launch, with full initial simulated battery.

```bash
python scripts/mission_demo.py --scenario low-battery --output logs/low-battery.json
python scripts/mission_demo.py --scenario emergency-battery --output logs/emergency.json
```

The critical test injects 10% after takeoff reaches 1 m. It requires emergency
native landing and MISSION_ABORTED, landed/disarmed. It does not pretend an
emergency landing away from a charger can recharge. After investigating a fault,
restart the ROS launch on the ground to clear the mission supervisor, or use the
explicit simulated-battery service where appropriate. This service only exists
for `battery.source: simulated`:

```bash
ros2 service call /drone/sim/set_battery drone_interfaces/srv/SetBattery '{percentage: 100.0}'
```

To test actual process loss, identify the exact node PID from the launch output.
The tool checks that PID belongs to the requested package in this workspace and
that this project's actual SITL binary is running. It starts a mission, kills
that one selected node at 1.5 m during takeoff, and observes **PX4 output directly**
so a dead gateway cannot hide the result:

```bash
python scripts/fault_trial.py --target mission --pid <mission_node_pid> --output logs/mission-crash.json
# After confirmed landing, restart the ROS launch before the separate test:
python scripts/fault_trial.py --target gateway --pid <gateway_node_pid> --output logs/gateway-crash.json
```

Success requires PX4 AUTO_LAND and fresh landed/disarmed state. The first exercises
the gateway lease; the second exercises native PX4 offboard loss. Automatic ROS
node respawn is disabled. See SAFETY.md for the independent safety-node watchdog.
A camera dropout was also observed during development and caused a safe return;
reliable image QoS resolved that failure in subsequent full missions.

## Timing, pause and diagnostics

Use `debug.sh topics|state|mission|rviz|record` to inspect or record standard topics.
`ros2 bag info <bag-directory>` verifies a completed recording. Bag playback must
use a separate ROS domain and stopped flight command processes. The stock
recording excludes command/setpoint topics; do not publish a recorded clock into
a live simulation.

Pausing Isaac pauses physics/PX4 lockstep. ROS wall-time watchdogs should invalidate
state within 2 s and abort/land on resume; elapsed wall time is not flight time.
A controlled pause/resume trial uses
`python scripts/fault_trial.py --target pause --pid <isaac_python_pid> --output logs/pause.json`.
It SIGSTOPs only the selected simulator process for 3 wall seconds during takeoff,
always resumes it, and requires native landing/disarm. Check PROJECT_STATUS.md
for the actual trial result.
Never infer it passed from the unit clock tests. Validate different real-time
factors, CPU contention and packet loss before increasing mission complexity.

On every failed gate, preserve PX4 ULog (`.local/px4-instance/log`), simulator/ROS
logs, parameter YAML, source commits and observer output. State what failed,
its likely cause, attempted fixes and the next verification. Do not relabel a
kinematic mock as an Isaac/PX4 result.
