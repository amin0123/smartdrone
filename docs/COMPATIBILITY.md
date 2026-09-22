# Compatibility audit

Audited on 2026-09-22, **before installing project dependencies**. This is a
candidate integration baseline; a documented component pairing is not proof
that the entire aircraft has passed the milestones in PROJECT_STATUS.md.

## Machine inspected

Ubuntu 24.04.4 LTS x86_64, kernel 7.0.0-31-generic; Intel i7-14700KF,
32 GiB RAM, RTX 5070 Ti with 16 GiB VRAM, NVIDIA driver 595.84, about
738 GiB disk available. Miniforge and ROS 2 Jazzy are already installed.
`~/isaacsim/VERSION` is **6.0.0-rc.59**; it is not the selected runtime.
No existing project files or commits were present. No driver, OS, ROS,
shell profile, or existing Isaac installation will be replaced.

## Selected versions

| Component | Selection | Reason / constraint |
|---|---|---|
| Ubuntu | Existing 24.04.4 LTS | Native Jazzy platform; supported by Isaac 5.1. Pegasus documents its tests on 22.04, so the complete Noble pairing needs local qualification. |
| NVIDIA driver | Existing 595.84 | Keep the working Blackwell driver; Isaac 5.1 documentation lists 580.65.06 as tested, not this exact version. Rendering is a mandatory gate. |
| Isaac Sim | **5.1.0 standalone** | Release explicitly paired with Pegasus 5.1.0. Install privately under `.local/isaac-sim-5.1.0`. NVIDIA now labels this release unsupported; use as a reproducible simulation baseline, track migration. |
| Pegasus Simulator | **v5.1.0**, commit `644da37e9d5268e5f9a34e78bdcfd57a8bab82b4` | Upstream explicitly requires Isaac 5.1.0. Supplies aircraft dynamics, sensor models and PX4 MAVLink bridge. |
| PX4 SITL | **v1.14.3**, commit `1dacb4cdef2d7145754fc788fa8dc482eed74b40` | Pegasus's documented development version; keep firmware/schema/protocol consistent. Not a recommendation to deploy old firmware to hardware. |
| px4_msgs | **release/1.14**, pinned `ffb6e80e1c17e5714395611a020c282a87af8fa4` | Match the firmware's message definitions. Do not use main with 1.14.3. Verify consumed messages against PX4 sources. |
| Micro XRCE-DDS Agent | **v2.4.2**, commit `57d086216d01ec43121845d385894a25987f8a2c` | 2.x agent for PX4 1.14's 2.x client. Do not substitute a 3.x agent. Build privately. |
| Agent's Fast DDS | **v2.12.2**, commit `092848725b8425e4f05a8ccf7b3b8d513fabf733` | Tracked build-only patch replaces Agent's deleted `2.12.x` branch reference. Other transitive dependencies remain its upstream Fast CDR 1.1.1, foonathan memory 0.7-3 and spdlog 1.9.2. |
| ROS 2 | **Jazzy Jalisco**, existing Noble Debian packages | Reuse native supported host installation; record exact Debian versions in `config/ros-packages.lock`. Fast DDS middleware (`rmw_fastrtps_cpp`). |
| Development Python | **3.12** in conda **drone** | Matches Jazzy's CPython ABI. Pin package resolution in environment lock after installation. ROS shared libraries still come from `/opt/ros/jazzy`. |
| Isaac Python | **Bundled 3.11** | Required by Kit/Isaac 5.1. Run in an isolated child process, never import host Jazzy's CPython 3.12 bindings there. |
| QGroundControl | **v4.4.3** Linux AppImage | Stable MAVLink observer for PX4 1.14; pin release asset. Run extracted if FUSE unavailable. No serial/group changes needed for SITL. |
| OpenCV | **4.10.0.84** contrib headless wheel | RGB masks and ArUco support, no Qt collision with QGC/Isaac. |
| NumPy / SciPy | **1.26.4 / 1.13.1** in drone | Python 3.12 compatible; retain NumPy 1.x ABI for host ROS extensions. Isaac keeps its bundled libraries. |
| PyYAML / pytest | **6.0.2 / 8.3.5** | Configuration and pure-logic verification independent of simulation. |
| PX4 Python generator | **empy 3.3.4**, pyros-genmsg **0.5.8** | PX4 1.14 templates predate incompatible EmPy 4 changes. Build tools run from drone; do not modify system pip. |

## Process and ABI boundaries

Development happens with `conda activate drone`. Native ROS uses Python 3.12.
Isaac is launched by a wrapper that removes sourced host ROS/conda library paths
and invokes its own `python.sh`, using the internal Jazzy bridge. Only standard
ROS image, camera info, clock and transform messages cross that boundary. Custom
`drone_interfaces` and `px4_msgs` remain outside Isaac; no second interface build
for Python 3.11 is necessary. DDS transports bytes between processes and does not
require matching Python minor versions.

Pegasus ↔ PX4 uses MAVLink TCP 4560 for simulated sensors and actuators. PX4 ↔
ROS uses XRCE UDP 8888. QGC receives MAVLink on UDP 14550. These are distinct links.
All use localhost for this single-machine baseline; ROS domain is configured
consistently. PX4's simulation clock and ROS camera timestamps need explicit
validation, including a paused-simulator test.

## Alternatives considered

* Isaac 6.0 RC already present: rejected as the default because no corresponding
  documented Pegasus release was found. Leave it intact for other projects.
* Isaac 4.5 + Pegasus 4.5.1 + Humble/22.04: documented historical pairing, but
  would add OS isolation and older renderer/GPU risk on this Blackwell machine.
* Ubuntu 22.04 + ROS Humble: closest to Pegasus's own test machine; retain as a
  container/secondary-host fallback only if Noble qualification fails.
* Newest PX4 / px4_msgs main: potentially useful fixes, but changes message
  contracts and bridge behavior before proving the reference aircraft.
* ROS directly driving rotor forces: rejected; flight stabilization must remain
  in PX4. A kinematic test double can only validate logic, never flight physics.
* Conda-only ROS or putting every runtime in one Python: unnecessary ABI risk.

## Official sources consulted

1. [Pegasus installation](https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)
   and [release source](https://github.com/PegasusSimulator/PegasusSimulator/tree/v5.1.0).
2. [Isaac 5.1 requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html),
   [ROS/Python interoperability](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html),
   [standalone archive](https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip).
3. [PX4 1.14 ROS guide](https://docs.px4.io/v1.14/en/ros/ros2_comm),
   [XRCE bridge](https://docs.px4.io/v1.14/en/middleware/uxrce_dds),
   [offboard mode](https://docs.px4.io/v1.14/en/flight_modes/offboard),
   [firmware source](https://github.com/PX4/PX4-Autopilot/tree/v1.14.3),
   [message branch](https://github.com/PX4/px4_msgs/tree/release/1.14).
4. [XRCE Agent 2.4.2](https://github.com/eProsima/Micro-XRCE-DDS-Agent/tree/v2.4.2).
5. [ROS Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
   (web viewer encountered an anti-bot page; native package inventory and NVIDIA's
   Jazzy/Noble matrix independently confirm the installed distribution).
6. [QGC 4.4.3 release](https://github.com/mavlink/qgroundcontrol/releases/tag/v4.4.3)
   and [Linux installation](https://docs.qgroundcontrol.com/Stable_V4.4/en/qgc-user-guide/getting_started/download_and_install.html).

The NVIDIA live skill catalog was checked; no skill specifically addressing the
Pegasus/PX4/Isaac integration was found. No additional skill was installed.

## Upgrade path

Record source commits, downloads and package inventory first. Qualify the
baseline through spawn → PX4 sensors → QGC → takeoff/land → DDS → offboard →
mission → injected failures. Upgrade Isaac and Pegasus together in a separate
environment when Pegasus supports a maintained Isaac release. Upgrade PX4 and
px4_msgs together, selecting the matching XRCE generation; inspect uORB schema,
command IDs, failsafe parameters and sensor timing. Repeat all flight and
failure gates before changing the baseline. Real aircraft require maintained
firmware and a separate hardware qualification campaign.
