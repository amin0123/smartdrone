# Mission executive

The executive runs in `drone_mission_manager.node.MissionNode`. Legal transitions
are declared in `fsm.py`; every event requires a reason and monotonic transition
time. State-specific handlers perform work and raise events. There is no motor
controller or simulator position override in this package.

```text
IDLE/READY/ABORTED --request--> PRE_FLIGHT_CHECK --> ARMING --> TAKEOFF
                                                               ↓
                                                      TRANSIT_TO_INSPECTION_AREA
                                                               ↓
                                                           INSPECTION
                                                               ↓
RETURN_HOME <---- completed route / low battery / abort / perception loss
     ↓
PRECISION_LANDING --> DOCKED --> CHARGING --> READY

Flight state --critical fault--> FAILSAFE --> EMERGENCY_LANDING
                                               ↓ confirmed landed + disarmed
                                         MISSION_ABORTED
```

| State | Work and completion condition |
|---|---|
| IDLE | Publishes health/status, awaits a start request. |
| PRE_FLIGHT_CHECK | Requires a fresh localized, landed, disarmed vehicle at the dock; charged battery; working camera/recorder; safe complete route; fresh safety decision. Streams a hold reference before checking PX4 readiness. Timeout rejects the mission. |
| ARMING | Requests offboard, confirms the mode, requests normal arming, confirms both actual states. PX4 command ACKs alone are insufficient. Timeout initiates failsafe handling. |
| TAKEOFF | Ramps the ENU position reference up to inspection altitude over the dock. |
| TRANSIT_TO_INSPECTION_AREA | Flies to the first coverage waypoint. |
| INSPECTION | Follows remaining crop-row waypoints while perception records observations. |
| RETURN_HOME | Maintains at least inspection altitude, then flies above the known station. There is no transition back to inspection. |
| PRECISION_LANDING | Descends to approach altitude, optionally refines station alignment with a marker, then requests native PX4 LAND. Waits for landed/disarmed/contact confirmation. |
| DOCKED | Confirms stable station contact before charging. A landing elsewhere is an abort, not a successful dock. |
| CHARGING | Waits for configured ready percentage; rejects loss of dock contact. |
| READY | Mission complete. Another mission requires a request unless auto-repeat is explicitly configured. |
| FAILSAFE | Records the cause and requests native LAND. |
| EMERGENCY_LANDING | Stops offboard intent, retries LAND, and waits for actual landing/disarming. Never claims completion while airborne. A disarm request is allowed only after stable, fresh landed confirmation. |
| MISSION_ABORTED | Publishes the cause and awaits a new start request. New preflight must pass from the dock. |

The transition table is the authoritative list of legal edges. Preflight and
charging can reject without entering a flight state. RETURN requests are accepted
only during takeoff/transit/inspection; critical decisions escalate any flight
phase. Explicit operator abort uses the same safe return/landing paths.

Mission, safety and gateway are separate processes. The executive refreshes
high-level intent at 10 Hz, and the gateway emits PX4 offboard messages at 20 Hz
only while the intent lease, state and safety status remain valid. Killing the
executive therefore stops permission to fly even if the gateway remains alive.
Killing the gateway removes PX4's offboard heartbeat, invoking native failsafe.

Trajectory progression uses simulation time, so paused time does not advance
waypoints. Communication watchdogs and executive phase deadlines use monotonic
wall time so a paused simulator cannot mask a dead connection. Slow rendering
therefore needs generous mission/landing deadlines; do not enlarge heartbeat
timeouts to hide a frozen controller. Review the configured flight timeout when
changing field size or measured real-time factor.

Start once with `ros2 service call /drone/start_mission std_srvs/srv/Trigger '{}'`.
Its response means preflight was accepted, not that the aircraft is armed.
Observe `/drone/mission_status` and `/drone/command_result`. Abort with the
equivalent `/drone/abort_mission` Trigger service. `auto_start` and `auto_repeat`
default to false. Each request receives a unique ID, preventing mixed records
across sorties. `executive.jsonl` and the recorder's transition table explain
every state change even when the sortie aborts.
