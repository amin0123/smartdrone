# Project status

Updated 2026-09-22. **No flight milestone has passed yet.** A unit test or mock
mission is not evidence of an Isaac/PX4 flight. Evidence is recorded below as
work proceeds.

| Subsystem | Status | Evidence / next gate |
|---|---|---|
| Machine and compatibility audit | DONE | COMPATIBILITY.md; version sources checked before installation |
| Architecture and roadmap | IN PROGRESS | Define runtime ownership and milestone acceptance |
| Conda drone environment | NOT STARTED | Create Python 3.12 environment after audit |
| Isaac 5.1 / Pegasus 5.1 | NOT STARTED | Separate install; preserve existing Isaac 6 RC |
| PX4 1.14.3 SITL | NOT STARTED | Build and validate sensor/actuator connection |
| QGroundControl | NOT STARTED | Confirm vehicle heartbeat and state |
| XRCE Agent and px4_msgs | NOT STARTED | Verify schemas and actual DDS state |
| Coordinate transforms | NOT STARTED | Pure tests including attitudes and yaw |
| Navigation | NOT STARTED | Boustrophedon planner, bounded position setpoints |
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
