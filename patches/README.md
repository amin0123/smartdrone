# Patches to pinned dependencies

External code remains in ignored `vendor/`. Build/install scripts apply these
small patches with `git apply --check`, or verify that they are already applied.
Never apply them to an unreviewed upstream version.

| Patch | Reason and scope |
|---|---|
| `px4-gcc13-cstdint.patch` | Include the standard integer header needed by PX4 1.14's POSIX shell with GCC 13. No flight behavior change. |
| `px4-inspection-dds-topics.patch` | Export existing battery, landed-state and command-ACK uORB messages through the supported DDS topic configuration. No wire schema change. |
| `xrce-agent-fastdds-pin.patch` | Agent 2.4.2 referred to deleted branch `2.12.x`; pin the compatible official Fast DDS 2.12.2 commit instead. |
| `xrce-agent-gcc13.patch` | Supply `<cstdint>` to old generated Fast DDS statistics sources when building on GCC 13. No XRCE protocol change. |
| `pegasus6-physics-attitude.patch` | Read position and quaternion from the same PhysX sample; the port otherwise mixed physics position with delayed USD attitude. The upstream issue is described in Pegasus PR 147. |

`build_px4.sh` also retains GCC's legacy LPE matrix array-bounds diagnostic as a
warning using one scoped compiler option. The selected estimator remains stock
EKF2. The PX4 annotated release tag and a NuttX version-metadata tag are fetched
without changing their selected commits because the version generator expects
tagged history even in a shallow checkout.

`isaac_sim/px4_backend.py` subclasses the existing Pegasus MAVLink bridge only to
bound a lockstep receive wait. Actuator decoding, rotor dynamics and avionics
sensors remain upstream implementations. The simulator owns its local PX4 child;
`scripts/owned_process.py` prevents a native Kit crash from leaving it orphaned.

Upstream sources retain their respective licenses. Pegasus is BSD-3-Clause,
PX4 is BSD-3-Clause, and eProsima Micro XRCE-DDS Agent/Fast DDS use Apache-2.0.
Check each pinned repository's LICENSE for complete attribution and terms.
NVIDIA Isaac binaries/assets and QGC are distributed separately under their own
terms. This repository does not redistribute their binary runtimes.
