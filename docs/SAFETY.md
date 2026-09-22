# Safety responsibilities

PX4 remains the flight controller: attitude/rate stabilization, motor allocation,
estimation, arming checks, land detection and native failsafes run there. ROS
provides a second supervisory layer, not an alternative stabilization loop.
All initial tests use the private SITL instance; do not upload its parameters
wholesale to an aircraft.

| Monitor | Response |
|---|---|
| Invalid/stale PX4 state or localization | Inhibit preflight; request native LAND in flight. |
| Position-origin reset | LAND: stored map waypoints cannot be trusted after a changed position reference. Ordinary in-flight yaw alignment is handled through current attitude. |
| Native PX4 failsafe or unexpected loss of offboard | Stop mission intent and request LAND; do not repeatedly re-enter offboard. |
| Low battery / insufficient reserve | RETURN, or LAND when the emergency threshold/reserve makes returning unsafe. |
| Geofence or altitude violation | Inhibit before flight; LAND in flight. |
| Missing mission or offboard heartbeat | LAND and stop the stream. |
| Perception or recorder health lost during inspection | RETURN; do not silently complete a survey without evidence. |
| Mission timeout | RETURN. Landing timeout escalates to native LAND. |
| Dock contact lost | Stop charging and mark the mission aborted. |

RETURN and LAND decisions latch while airborne. Recovery of a sensor cannot
restart crop inspection midway through return. New preflight clears the latch
only on a disarmed vehicle and rechecks every prerequisite. The mission records
transition reasons, so a return due to camera loss is distinguishable from
completed coverage even if both end at READY.

There are three independent failure paths:

1. The mission stops or crashes: its target lease expires after 0.5 wall seconds.
   The gateway requests native LAND and stops the offboard stream.
2. The gateway or DDS link fails: PX4 receives no offboard heartbeat and applies
   `COM_OF_LOSS_T=0.5`, `COM_OBL_RC_ACT=4` (LAND in the pinned firmware).
3. The supervisor requests RETURN/LAND: the executive follows that decision;
   the gateway also refuses unsafe intent independently of the executive.

The safety node itself has a freshness watchdog in the gateway. Respawning a
controller automatically while airborne is intentionally avoided. If simulation
pauses, physical motion and PX4 lockstep pause, but ROS communication watchdogs
continue on monotonic wall time. On resume, an abort/landing may be pending;
restart the mission from the ground rather than assuming the old sortie resumes.

Slow PX4 status and land-detection topics are periodic in **simulation time**
(2 Hz and 1 Hz respectively). Their age is evaluated in ROS time. High-rate
position/attitude age is independently checked in wall time, so a stopped clock
cannot keep a disconnected vehicle marked healthy. This avoids treating a
healthy slow renderer as a lost land detector.

The simulator's PX4 actuator wait is bounded. If PX4 exits or stops supplying
actuators, the wrapper ends the simulation instead of hanging forever in the
upstream lockstep wait. This protects the development process; it is not a
hardware failsafe.

Known scope: planning checks static obstacle volumes with vehicle clearance. It
does not detect or avoid newly moving obstacles. Localization validity flags
cannot prove freedom from all estimator errors. Simulated battery consumption
is a configurable development model, not an energy guarantee. Real operation
needs obstacle sensing, measured energy reserve, validated localization, operator
recovery and flight-controller failsafe tests under the actual hardware setup.
