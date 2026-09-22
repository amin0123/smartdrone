# Compatibility audit

Audited on 2026-09-22, **before installing project dependencies**. The original audit selected the released 5.1 pairing before installation. Its
renderer failed on this host. The explicitly experimental 6 profile subsequently
passed actual PX4 flight and a complete inspection mission. Local qualification
is recorded in [PROJECT_STATUS.md](PROJECT_STATUS.md); it does not convert an
open upstream port into a supported release.

## Machine inspected

Ubuntu 24.04.4 LTS x86_64, kernel 7.0.0-31-generic; Intel i7-14700KF,
32 GiB RAM, RTX 5070 Ti with 16 GiB VRAM, NVIDIA driver 595.84, about
738 GiB disk available. Miniforge and ROS 2 Jazzy are already installed.
`~/isaacsim/VERSION` is **6.0.0-rc.59+release.41464.5f2772bc.gl**; it was not the initially selected runtime. The `rc` string alone is not evidence of a prerelease: the official 5.1 standalone archive also uses an internal `rc` build identifier.
No existing project files or commits were present. No driver, OS, ROS,
shell profile, or existing Isaac installation will be replaced.

## Simulator profiles

| Profile | Exact pairing | Local result / use |
|---|---|---|
| Original released pairing | Isaac 5.1.0 (`5.1.0-rc.19+release.26219.9c81211b.gl`), Pegasus v5.1.0 SHA `644da37e9d5268e5f9a34e78bdcfd57a8bab82b4`, bundled Python 3.11 | **BLOCKED** on R595 before scene construction. Retained for a separately qualified R580 host. |
| Working local experimental pairing | Existing Isaac `6.0.0-rc.59+release.41464.5f2772bc.gl`, Python 3.12.13; Pegasus PR 144 SHA `fcb99c061460c0000144e526e6384a49291b6e77` plus tracked physics-attitude fix | **Actual flight verified** with PX4 1.14.3, including a full RGB inspection/landing/recharge. Select explicitly with `run_sim6.sh`. This is an unreleased upstream port. |

## Dependency decisions

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
| Development Python | **3.12.11** in conda **drone** | Matches Jazzy's CPython ABI. Pin package resolution in environment lock after installation. ROS shared libraries still come from `/opt/ros/jazzy`. |
| Isaac Python | **Bundled 3.11 for 5.1; 3.12.13 for 6** | Keep each runtime isolated even when its Python minor matches the external ROS ABI. |
| QGroundControl | **v5.0.8** Linux AppImage, extracted | Stable Noble-supported observer, actual PX4 1.14.3 connection verified. Replaces initial 4.4.3, whose fatal serial-group check blocked UDP-only startup. No host group changes. |
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
This is a single-machine baseline; ROS domain is configured consistently. Some
middleware sockets bind all host interfaces by default: localhost testing is not
a network-security boundary. See PX4_ROS2.md for the receive-time timestamp mode
needed by PX4 1.14 under sub-real-time lockstep and separate wall-time watchdogs.

## Alternatives considered

* Existing Isaac 6: initially rejected because no matching released Pegasus was
  found. After the 5.1 renderer failure, qualified a separate pinned open port;
  existing Isaac files remain intact. Explicit profile selection preserves the decision.
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

## Local rendering gate: failed on the selected driver

The installed official Isaac 5.1 archive starts Kit, then exits with SIGSEGV in
`librtx.scenedb.plugin.so` before project scene construction. Reproduction:
`bash scripts/run_sim.sh --headless --no-px4 --duration 5`; evidence:
`logs/isaac-spawn.log`. Profile 5 did **not** spawn the aircraft; profile 6 is independently verified.

NVIDIA maintainers identify driver R595 as incompatible with Kit 107.3.3 in
[issue 619](https://github.com/isaac-sim/IsaacSim/issues/619) and
[issue 517](https://github.com/isaac-sim/IsaacSim/issues/517). Their Linux remedy
is an R580 driver. Disabling rendering would invalidate the camera mission;
the D3D12 workaround is Windows-only. No driver has been changed.

An isolated alternative qualification uses the already installed Isaac 6 build
and [Pegasus PR 144](https://github.com/PegasusSimulator/PegasusSimulator/pull/144),
head `fcb99c061460c0000144e526e6384a49291b6e77`. This PR is **open/unreleased**;
its author reports flight with this exact Isaac build, Jazzy and PX4 1.16.
Our PX4 1.14.3 pairing subsequently passed native and ROS offboard flight, full
RGB inspection, return, landing and recharge. This is an explicitly
experimental alternative, not a silent change to the stable baseline. It
preserves Pegasus physics and PX4 motor control. Driver replacement remains a
separate machine-level decision because it affects other projects and needs a
reboot. The installed native package candidate is R580 open 580.178.04, which
also needs qualification rather than assuming the documented 580.65.06 result.

## QGroundControl qualification adjustment

The 4.4.3 process did not reach its main window: its source makes missing serial
`dialout` group membership a fatal startup error, even for UDP-only SITL.
Select **QGroundControl 5.0.8 stable** for this host (not the newer 5.1 series).
Its [source](https://github.com/mavlink/qgroundcontrol/blob/v5.0.8/src/QGCApplication.cc)
changes this check to a notification; its [platform matrix](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html)
explicitly supports Ubuntu 24.04. MAVLink monitoring of PX4 1.14.3 was verified in the actual QGC window
(`logs/qgc5-after-dialog.png`). Install the pinned official AppImage privately.
No serial-group or ModemManager changes are required for this UDP-only use.

## Tracked integration fixes and repeatability

`config/dependencies.repos` pins source commits. `environment.yml` pins direct
Python dependencies; the pip/conda and native ROS inventories in `config/` record
the installed resolution. `patches/README.md` explains build portability and DDS
export changes. Pegasus's physics quaternion is sampled with its PhysX position,
following the issue described in [PR 147](https://github.com/PegasusSimulator/PegasusSimulator/pull/147).
The experimental port retains upstream rotor dynamics, IMU/GPS models and
MAVLink actuator handling; ROS never generates rotor forces.

The real-time factor is about 0.3 on this host with rendering. ROS motion/battery
integration uses `/clock`; controller/process liveness uses wall time. Reliable
raw-image DDS QoS was required to eliminate observed fragmented-image loss. A
faster machine or different RMW is another timing qualification, not merely a
package-version substitution.
