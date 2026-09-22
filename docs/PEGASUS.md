# Pegasus integration

Pegasus supplies the stock Iris USD aircraft, four-rotor thrust/drag dynamics,
IMU, GPS, barometer and magnetometer. Its PX4 MAVLink backend consumes PX4 HIL
actuator outputs and applies them through the vehicle model. This project never
uses Pegasus's direct ROS rotor-control backend. Rigid-body motion, collisions
and rendered cameras remain Isaac's responsibility.

Two source checkouts make the compatibility choice explicit:

| Profile | Upstream boundary |
|---|---|
| Pegasus v5.1.0 | Documented Isaac 5.1 pairing. Uses World and the older dynamic-control API. Installed but blocked by this host's renderer/driver failure. |
| Pegasus PR 144, immutable commit in `config/dependencies.repos` | Open Isaac 6 port. Uses SimulationManager and experimental RigidPrim/Articulation APIs. This profile passed the recorded local flight trials, but remains unreleased. |

The standalone API is initialized only after SimulationApp loads Kit. The 5.1
world ownership pattern assigns `PegasusInterface`'s world backing member; the
6 port uses its simulation manager. These version-specific dependencies stay in
`isaac_sim/`, outside reusable ROS autonomy. The 6 physics-attitude patch reads
position and quaternion from the same PhysX update instead of mixing a current
physics position with a delayed USD attitude. See `patches/README.md` and the
upstream issue linked in COMPATIBILITY.md.

Pegasus's setup.py would modify Isaac's base application file to enable the
extension. The project exposes pinned source on PYTHONPATH, installs extra Python
dependencies privately (`.local/isaac-python` or `.local/isaac6-python`) and enables
extensions in the launcher. The simulator's existing installation and global
extension-manager preferences remain intact.

PX4 runs as a separate process through `scripts/run_px4.sh`, with private
parameters and flight logs. The simulation starts it by default and binds that
child's lifetime to the simulator, including native Kit shutdown/crash.
`--external-px4` supports a separately managed terminal; `--no-px4` is a spawn-only
diagnostic. Do not launch a second PX4 instance on the same ports.

`BoundedPX4Backend` adds a five-second wall-time limit to the existing lockstep
actuator receive loop. On PX4 death/transport failure, the wrapper reports the
error and ends the simulation rather than hanging indefinitely. It retains the
upstream HIL actuator decoder and sensor/rotor models. A three-second simulator
pause/resume trial separately verified ROS watchdogs and native landing.

Changes to rotor geometry, mass, motor constants, sensor noise or estimator
settings require another native hover/landing gate before autonomous missions.
Changing the visual mesh alone does not establish a valid aircraft dynamics
model. Any integration change belongs in a tracked patch or narrow adapter;
a kinematic test double is never labelled as a physics-flight result.
