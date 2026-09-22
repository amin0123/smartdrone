# PX4 and ROS 2 communication

SITL is the normal PX4 flight stack compiled as a host process. Its estimator,
controllers, arming checks and mode logic still run. The simulated sensor and
actuator boundary replaces physical devices; a ROS node does not replace the
flight controller.

There are two independent protocols in this project:

```text
Pegasus IMU/GPS/barometer/magnetometer
    → MAVLink HIL sensor messages → PX4 → HIL actuator messages → Pegasus rotors

ROS planner → ENU FlightTarget → gateway → PX4 TrajectorySetpoint
    → DDS Agent → XRCE client → uORB → PX4 position/attitude/rate control
```

MAVLink also supplies QGroundControl's vehicle link on UDP 14550. XRCE on UDP
8888 carries the DDS topics for autonomy. QGC can observe the mission without
participating in the ROS command path. The initial `scripts/flight_smoke.py`
uses MAVLink for a diagnostic arm/takeoff/land sequence; the mission uses DDS.
Do not run two mission controllers at the same time.

uORB is PX4's internal publish/subscribe bus. The XRCE client serializes selected
uORB types and the Agent represents them as DDS entities. `px4_msgs` generates
the corresponding ROS types. Matching field order and types is essential:
similar-looking message names are insufficient. `scripts/check_px4_schemas.py`
compares the nine live wire types plus VehicleOdometry, retained as a checked
optional telemetry schema. Standard `/drone/odometry` is currently composed from
local position and attitude. `patches/px4-inspection-dds-topics.patch`
adds the stock battery, land detector and command ACK outputs omitted from the
1.14 default list. Rebuild firmware after altering this export configuration.

## Gateway contract

| Interface | Meaning |
|---|---|
| `/drone/state` | Freshness-qualified ENU pose/velocity, FLU attitude, arming, landed, battery, navigation state |
| `/drone/odometry` | Standard ENU pose with FLU body velocity, stamped when the PX4 sample is received |
| `/drone/target` | Position or velocity in map ENU, ENU yaw and increasing sequence; refresh at 10 Hz |
| `/drone/vehicle_command` | Queue arm, disarm, offboard, land or native return-home; response is not proof of flight-state change |
| `/drone/command_result` | Actual PX4 acknowledgment directed to this gateway's source IDs |
| `/drone/offboard_heartbeat` | Evidence that the gateway just published a valid offboard stream |

`Px4Client` exposes arm/disarm/takeoff/land/goto_position/set_velocity/return_home
and get_vehicle_state. Its positional requests require periodic refresh. Mission
management owns the route and durations; a one-shot target is intentionally not
an unlimited authorization to fly.

The gateway prestreams at 20 Hz before requesting offboard or arming. It checks
the controller lease, independent safety heartbeat and PX4 telemetry. Unused
position, velocity, acceleration and yaw-rate fields are NaN rather than zero.
Zero would request an actual velocity/acceleration or heading component.
Position targets use only position; velocity mode uses only velocity.

Subscriptions to PX4 telemetry use best-effort/volatile QoS, compatible with the
bridge's sensor-style writers. Project command/mission channels use reliable
QoS. Mission status is transient-local so a newly started observer receives the
current state. Heartbeats are volatile: an old retained sample must never prove
that a controller is alive.

## Native failsafe ownership

The SITL-only `px4/config/px4-rc.params` is loaded through PX4's existing PATH
customization hook. `COM_OF_LOSS_T=0.5` and `COM_OBL_RC_ACT=4` select native land
on offboard loss in this pinned release. `COM_RC_IN_MODE=4` disables absent stick
input for the autonomous simulation. Arming checks and stabilization remain
active; there is no force-arm flag. A native cylindrical geofence complements
the ROS rectangular fence. The ROS battery adapter is separate from PX4's own
simulated battery and QGC display; do not confuse their percentages.

If the mission node stops, the gateway's wall-clock lease expires and it requests
native LAND, then stops offboard. If the gateway itself dies, PX4 detects missing
offboard messages. If the DDS Agent disappears, PX4 does the same. During an
Isaac lockstep pause, simulated PX4 time may pause too; test behavior after
resuming, and distinguish simulator time from wall-clock process liveness.

Official protocol and mode references are recorded in COMPATIBILITY.md. Parameter
enums here were checked against the **v1.14.3 source**, not current-main docs.


## Clock and freshness contract

PX4 1.14 has no `UXRCE_DDS_SYNCT` parameter. On this renderer's approximately
0.3 real-time factor, Agent time synchronization drift caused genuine offboard
loss despite a continuous 20 Hz publisher. The pinned generated PX4 decoder
(`build/px4_sitl_default/uORB/ucdr/offboard_control_mode.h`)
explicitly maps **timestamp 0** to `hrt_absolute_time()` on receipt. The simulation
config uses `px4.timestamp_mode: receive_time` to select that existing path;
there is no firmware clock patch. Command stream QoS is reliable, depth 1, with
0.2 s lifespan; the independent 0.5 s wall-time intent lease still rejects stale
or replayed controller intent. A zero timestamp alone is never a liveness check.

For hardware, select `timestamp_mode: system`, `use_sim_time: false`, and validate
Agent/client clock synchronization. Do not copy the simulation setting blindly.
Slow PX4 status (2 Hz), landed (1 Hz) and battery are aged in ROS time with a
2.5 s limit. High-rate local position and attitude must arrive within 2 s wall
time as well, so pausing the simulator or losing the link invalidates state even
when `/clock` stops. This distinction avoids false disconnects at low real-time
factor while retaining independent process supervision.

`preflight_ok` comes from PX4's actual `pre_flight_checks_pass`. PX4 1.14's
`heading_good_for_control` represents final in-flight magnetic alignment and
must not be required before ground arming. Expected yaw alignment resets are
not map-origin changes; XY/Z estimator reset counters still invalidate a mission.
