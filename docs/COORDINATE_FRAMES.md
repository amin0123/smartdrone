# Coordinate frames

Lengths are metres, angles radians, velocities metres/second. ROS world `map`
is ENU; aircraft `base_link` is FLU. PX4 local world is NED and body is FRD.
`drone_core.frames` owns these transformations; the PX4 gateway calls it once
on ingress and once on egress. Pegasus already converts its simulated sensor
data for PX4; do not apply our conversion inside Pegasus a second time.

```text
World ENU                  World NED
  +Z up                      +Z down
  +Y north                   +X north
  +X east                    +Y east

Body FLU                   Body FRD
  +X forward                 +X forward
  +Y left                    +Y right
  +Z up                      +Z down
```

For any world position or world velocity, `(N,E,D) = (y,x,-z)`.
For body vectors, `(F,R,D) = (x,-y,-z)`. Both matrices are proper rotations,
their own inverse, and have determinant +1. An ENU waypoint `(3,5,4)` becomes
PX4 `(5,3,-4)`. Never just negate altitude while leaving horizontal axes alone.

ENU yaw is counterclockwise about up, zero east. NED yaw is clockwise viewed
from above, zero north. `yaw_ned = wrap(pi/2 - yaw_enu)`. A ROS eastward yaw of
zero is PX4 +pi/2. Use full rotation matrices/quaternions for attitudes:

```text
R_ENU_FLU = R_ENU_NED × R_NED_FRD × R_FRD_FLU
```

Quaternion layout in core and ROS is **x,y,z,w**. PX4's attitude messages use
**w,x,y,z**. Layout reordering and frame rotation are different operations.
Random rotation roundtrip tests supplement named-axis examples.

`map` is anchored to PX4's initial local origin for a single mission. The scene
spawns at the configured dock origin. An estimator origin reset invalidates the
current route and observation association: safety must stop the mission instead
of interpreting a sudden coordinate jump as real motion. Hardware requires an
explicit surveyed map-to-local transform, including altitude datum.

The downward camera uses an identity USD local orientation, looking along -Z.
Its ROS optical axes are right +X, image-down +Y, forward +Z. Relative to the
body this is `diag(1,-1,-1)`; image-right points along body-forward and
image-down along body-right. This valid mounting is unusual, so the extrinsic
must be retained when replacing the camera. Image projection uses CameraInfo,
that extrinsic and an image-time body pose. Ground intersection is approximate
when canopy height is uncertain. Do not label it centimetre-accurate GPS.
