# Installation and process isolation

Read [COMPATIBILITY.md](COMPATIBILITY.md) first. The existing workstation is already
prepared. These steps explain how to reproduce it; they do not replace the OS or
driver. Ignored `.local/` contains binary runtimes/builds and `vendor/` contains
pinned external sources. Allow approximately 60 GB for both profiles, build trees,
archives and initial recordings; large rosbag/image collections need more.

## Host prerequisites

Use Ubuntu 24.04 x86_64 with native ROS 2 Jazzy, its Fast DDS RMW, ROS development
tools and RViz2. Required build utilities include Git, GCC/G++, CMake, Make,
Ninja, zip/unzip, curl and pkg-config. The inspected machine already supplied
these, NVIDIA 595.84 and Miniforge. Exact installed ROS package versions are in
`config/ros-packages.lock`. On a fresh host use the
[official Jazzy installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
then verify `/opt/ros/jazzy/setup.bash`. System package/driver installation is a
separate host setup step, not hidden in a project script.

## Conda and source versions

```bash
cd /path/to/smartdrone
conda env create -f environment.yml       # skip when drone already exists
source scripts/activate.sh
python --version                        # 3.12.11
python scripts/fetch_sources.py --experimental-isaac6
```

For an existing environment, inspect its differences before choosing
`conda env update -f environment.yml`. `activate.sh` sources Jazzy and the built
workspace without editing the login profile. Set `DRONE_CONDA_ROOT` if Miniforge
is not under `~/miniforge3`. Do not create a second environment named drone.

`fetch_sources.py` verifies exact commits and fetches PX4 submodules and the
version tags needed by its generators. It refuses mismatched existing checkouts.
Build scripts apply tracked patches idempotently; vendor dirty status is expected
only for those patches. See `patches/README.md` for each change.

## Simulator profile 6: working on this workstation, experimental Pegasus port

Keep the existing standalone Isaac build identified in COMPATIBILITY.md.
`DRONE_ISAAC6_PATH` can select its installation directory; the launcher default is
`$HOME/isaacsim`. NVIDIA distribution/license terms apply; Isaac is not vendored
into this repository. Reproduce this exact build before substituting a later one.

```bash
source scripts/activate.sh
bash scripts/install_pegasus6_deps.sh
```

This installs only the required CPython 3.12 MAVLink dependencies into the private
`.local/isaac6-python` folder and applies the documented Pegasus attitude timing
patch. The simulator retains its bundled NumPy/SciPy/Kit libraries. `run_sim6.sh`
isolates host ROS/conda library paths and uses the internal Jazzy bridge.
Custom ROS interfaces remain outside Isaac, so there is no duplicate interface
build inside its Python environment.

## Simulator profile 5: original documented pairing, blocked here

```bash
bash scripts/install_isaac.sh
bash scripts/install_pegasus_deps.sh
```

The installer downloads the versioned approximately 8.8 GB Isaac 5.1 archive,
checks ZIP integrity, records its SHA256 and extracts privately. The hash records
local artifact identity; it is not an independently published NVIDIA signature.
Its Python is **3.11**, so the camera bridge uses bundled ROS libraries and sends
standard DDS messages to external Python 3.12 Jazzy nodes. Never pip-install Isaac
into `drone` or import the host's rclpy into its Python 3.11 interpreter.

On driver 595.84 this renderer crashes before the scene starts. Do not treat
reinstalling Python as a remedy. An R580 driver change/reboot requires separate
machine-level planning. The installer does not run NVIDIA's desktop-shortcut
post-install script; an early attempt altered the shortcut and is documented in
PROJECT_STATUS.md.

## Build the flight and ROS stack

```bash
source scripts/activate.sh
bash scripts/build_agent.sh
bash scripts/build_px4.sh
bash scripts/build_ros.sh
source scripts/activate.sh                 # pick up generated interfaces
python scripts/check_px4_schemas.py
python -m pytest -q
bash scripts/install_qgc.sh
python scripts/doctor.py --profile 6
```

Builds are local; no `sudo`, global pip, driver replacement or ROS package changes
are performed. PX4's stock battery/landed/ACK uORB topics are explicitly exported
to DDS and their message layouts checked against `px4_msgs`. A successful build
proves neither rendering nor flight: follow the gates in TESTING.md.

QGC 5.0.8 is extracted from its pinned official AppImage and launched with private
settings/cache paths, so it works without FUSE. Its missing serial-permission
notification can be dismissed for UDP SITL. No serial/group/ModemManager changes
are needed for this simulation. `run_qgc.sh` removes conflicting conda Qt paths.

## Runtime order

In separate terminals at the repository root:

```bash
bash scripts/run_sim6.sh --headless --cameras
```

```bash
source scripts/activate.sh
ros2 launch drone_bringup full_system.launch.py
```

```bash
bash scripts/run_qgc.sh
```

Wait for `DRONE_SIM_READY` and PX4 estimator readiness. Then:

```bash
source scripts/activate.sh
python scripts/mission_demo.py --output logs/my-mission.json
```

The simulator normally starts/owns PX4. To debug it in a separate terminal use
`bash scripts/run_sim6.sh --external-px4 --cameras`, then
`bash scripts/run_px4.sh`. External PX4 is then your terminal's responsibility.
For an already running Agent, pass `start_agent:=false` to `full_system.launch.py`.
Do not start duplicate agents, simulators, gateways or mission managers.

`simulation.launch.py experimental_isaac6:=true` is the GUI launch equivalent.
`px4_bridge.launch.py`, `perception.launch.py` and `mission.launch.py` start subsets
for incremental diagnosis. The canonical launch files live in `drone_bringup`.
Set the same `ROS_DOMAIN_ID` in every terminal before launching; default is 0.
PX4's XRCE client uses that value. MAVLink TCP 4560 and UDP 14550, and XRCE UDP 8888
must be available. This configuration is a single-vehicle localhost baseline.

For normal shutdown, confirm `/drone/state` says landed and disarmed, then stop
the ROS launch and simulator. Isaac owns only the child it created; the Linux
parent-death signal also prevents an orphan PX4 after native Kit shutdown/crash.
A simulator restart resets the world clock and estimator: restart ROS with it.
Do not replay rosbag commands into a running vehicle's domain.
