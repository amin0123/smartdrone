# Project status

Updated 2026-09-22. **No flight milestone has passed yet.** A unit test or mock
mission is not evidence of an Isaac/PX4 flight. Evidence is recorded below as
work proceeds.

| Subsystem | Status | Evidence / next gate |
|---|---|---|
| Machine and compatibility audit | DONE | COMPATIBILITY.md; version sources checked before installation |
| Architecture and roadmap | IN PROGRESS | Define runtime ownership and milestone acceptance |
| Conda drone environment | DONE | Python 3.12.11; OpenCV/NumPy/rclpy imported; actual Jazzy node created |
| Isaac 5.1 / Pegasus 5.1 | IN PROGRESS | Download in progress; Pegasus source pinned and Iris asset present |
| PX4 1.14.3 SITL | IN PROGRESS | Build DONE with documented portability fixes; real sensor/flight validation next |
| QGroundControl | NOT STARTED | Confirm vehicle heartbeat and state |
| XRCE Agent and px4_msgs | IN PROGRESS | Agent built and listening UDP 8888; interfaces and gateway built; 10 schemas match |
| Coordinate transforms | DONE | Six tests include named axes, invalid values and 100 attitude roundtrips |
| Navigation | IN PROGRESS | Eight pure planner/tracker checks; physical flight still unverified |
| Mission state machine | NOT STARTED | Explicit transitions and reasons |
| Perception and records | NOT STARTED | Camera → detections → pose association → local report |
| Battery and safety | NOT STARTED | Stale data, loss of controller, reserve decisions |
| Landing and docking | NOT STARTED | Known station before marker refinement |
| Recharge | NOT STARTED | Only landed, disarmed and within station tolerance |
| Documentation / migration | IN PROGRESS | Explain assumptions, limitations and exact commands |

## Milestone gates

| # | Acceptance | Status |
|---|---|---|
| 1 | Isaac spawns a physical Pegasus aircraft | NOT STARTED |
| 2 | PX4 receives sensors and returns actuator commands | NOT STARTED |
| 3 | QGC displays connected vehicle | NOT STARTED |
| 4 | PX4 arms, takes off and lands | NOT STARTED |
| 5 | ROS receives real PX4 status/position | NOT STARTED |
| 6 | ROS offboard setpoints control the aircraft | NOT STARTED |
| 7 | Aircraft flies commanded waypoints | NOT STARTED |
| 8 | Isaac RGB/depth reaches ROS | NOT STARTED |
| 9 | RGB perception detects simulated plants | NOT STARTED |
| 10 | Aircraft completes crop-row route | NOT STARTED |
| 11 | Low battery causes return | NOT STARTED |
| 12 | Known-station autonomous landing | NOT STARTED |
| 13 | Marker alignment and precision landing | NOT STARTED |
| 14 | Docked aircraft recharges to READY | NOT STARTED |
| 15 | Full mission including records and fault trials | NOT STARTED |

## Change / failure log

* Audit: host already has Jazzy and Isaac 6.0 RC. Selected a separate Isaac 5.1
  install matching Pegasus instead of altering the existing simulator.
* System-level changes: none. Downloads, source builds and Python dependencies
  will remain in the repository's ignored directories and conda environment.
* PX4 first configuration failed: `CMakeLists.txt:129 list index: 1 out of
  range`, version printed as `1dacb4c..NOTFOUND`. Cause: shallow SHA-only fetch
  omitted the annotated v1.14.3 tag expected by PX4's version parser. Fix:
  fetch that exact tag without changing HEAD; make fetch script preserve it.
* Pure logic suite: 17 passed. These do not advance the flight milestone table.
* ROS interface build: `drone_core`, `drone_interfaces`, `px4_msgs` passed in
  conda drone; all 10 gateway wire definitions match the firmware sources.
* PX4 next build exposed `px_update_git_header.py:153 IndexError` because the
  shallow NuttX checkout has no version tags. Fetch the existing upstream
  `nuttx-10.3.0` metadata tag without changing its submodule commit. NuttX is not
  executed in SITL.
* Agent build failed `fatal: invalid reference: 2.12.x`. The upstream 2.4.2
  superbuild refers to a deleted Fast DDS branch. Apply the tracked one-line
  `patches/xrce-agent-fastdds-pin.patch` selecting official Fast DDS v2.12.2 by
  SHA. No protocol version or runtime code was changed. Rebuild pending.
* Host GCC 13 then exposed missing `<cstdint>` includes in PX4's POSIX shell and
  Fast DDS generated statistics code. Tracked header/build patches supply the
  standard header. PX4's optional legacy LPE also triggers `-Warray-bounds` in
  generic matrix inversion; retain this diagnostic as a warning using the one
  scoped `-Wno-error=array-bounds` flag. Stock EKF2 remains the active estimator.
* Inspection of firmware DDS configuration found landed, battery and command ACK
  outputs absent from the stock export list. Add these existing uORB topics via
  the supported `dds_topics.yaml` mechanism, using a tracked patch; no message
  schema changes are required. Rebuild is required before flight tests.
* Pure logic suite: 25 passed after mission FSM and independent safety policy.
* Build repairs verified: PX4 linked successfully; XRCE Agent installed locally
  and started on UDP 8888. The ROS gateway initializes in conda drone. No flight
  claim follows from these checks.
* Pure logic suite: 29 passed, including lease expiry/replay and dock calculations.
  One initial lease test used 1.4 < 14×0.1 due to floating representation; changed
  the observation time to 1.45, preserving the strict backward-time rejection.
