# Isaac simulation

The simulator is responsible for PhysX rigid bodies, collisions, rendering and
sensor generation. Pegasus supplies the Iris asset, four-rotor force model and
IMU/GPS/barometer/magnetometer models. PX4 receives those measurements through
the Pegasus MAVLink HIL backend and sends actuator outputs back. The project
never animates the aircraft along the mission waypoints.

`config/world.yaml` describes the primitive field: ground, crop rows, open
transit/docking space, two obstacles, synthetic healthy/stressed canopies and a
white docking pad with marker geometry. Local USD primitives avoid downloading a
large decorative scene before flight validation. Source labels are saved solely
for evaluation; perception consumes rendered RGB. Plant colliders are conservative
and not biologically realistic. Keep planned flight above the canopy clearance.

`run_simulation.py` creates SimulationApp before importing Isaac extensions,
constructs the field/vehicle, initializes sensors and starts PX4 as a child.
Cleanup stops PX4. The PX4 runtime is the same compiled SITL binary used by the
manual smoke test. `--external-px4` allows a separately managed instance; never
launch two instances on the same ports. `--no-px4 --duration 5` is a spawn-only
test, not a flight demonstration.

Two explicitly different profiles exist:

* `scripts/run_sim.sh`: documented Isaac 5.1 / Pegasus 5.1 pairing. It is blocked
  on this host by the R595 renderer crash documented in COMPATIBILITY.md.
* `scripts/run_sim6.sh`: installed Isaac 6.0 build plus pinned upstream Pegasus
  PR 144 and the tracked physics-attitude correction. This is an unreleased
  integration profile qualified locally, not an upstream stable Pegasus release.

Isaac 5 uses World; Isaac 6 uses SimulationManager/RenderingManager through the
small `runtime6.py` adapter. Both keep simulator-specific imports out of ROS
autonomy. The launcher removes conda/host ROS binary paths from the simulator
process and uses bundled Python/ROS libraries. The ROS portion remains in conda
`drone`. DDS messages cross the process boundary without sharing Python objects.

The downward pinhole camera has explicit focal length, aperture, resolution and
body translation. In the 6 profile Replicator produces RGB and image-plane depth;
the bridge publishes Image, CameraInfo and `/clock`. Camera geometry follows
COORDINATE_FRAMES.md. Frames are stamped after the corresponding synchronous
render update. A renderer/host-copy pipeline can add approximately one rendered
frame of latency; the first implementation makes approximate location claims,
not calibrated metric mapping accuracy. Validate latency before adding visual
odometry or precision measurements. No standard RGB rendering is claimed to be NIR.

Physics runs at 250 Hz with rendering every fourth step by default. The simple
scene may run below real time on this machine, especially with camera readback.
Measure `/clock` advancement versus wall time rather than assuming real-time
performance. Keep IMU/actuator lockstep; tune render rate and image resolution
before changing flight dynamics or widening safety timeouts. Pausing simulation
freezes physical motion, while ROS wall-clock watchdogs intentionally detect loss
of fresh state. Restart cleanly after a paused-flight abort.

Flight trials should run without a fixed `--duration`: a short simulator timeout
can stop physics during landing. Stop the process after landed/disarmed state or
use the documented test harness. Native PX4 ULogs are under the private runtime's
`log/` directory; simulator output and mission evidence are under `logs/`.


For the GUI, omit `--headless`. `config/world.yaml` → `viewport.eye` and
`viewport.target` set the spectator view of the whole field. This is independent
of the vehicle's calibrated RGB/depth camera. The Isaac 6 adapter uses
`ViewportCameraState(viewport=...)`; passing the viewport as its first positional
argument incorrectly treats that object as a camera path in the pinned Kit API.
