# Implementation roadmap

1. Audit the machine and dependency releases. Save the compatibility decision
   before installing dependencies. Establish isolated environments and locks.
2. Prove the physical simulator: primitive field, stock Pegasus Iris, PX4 sensor
   loop, QGC, basic takeoff/land. Diagnose a failed gate before moving forward.
3. Build matching DDS messages and agent. Confirm actual PX4 state; add a single
   owner for offboard streaming, coordinate transforms and command acknowledgments.
4. Add tested route generation and an explicit mission state machine. Exercise
   takeoff, one waypoint, return and confirmed landing before the full route.
5. Publish camera calibration/images, detect colored vegetation, associate image
   timestamps with vehicle poses and persist records plus post-landing summary.
6. Add independent safety monitoring, battery abstraction and fault injection.
   Test controller loss as well as normal and low-battery missions.
7. Prove known-station landing/recharge; add marker refinement with a configured
   fallback policy. Repeat the complete mission and document the evidence.

The repository root itself is `autonomous_drone_inspection` conceptually; do not
nest another Git repository beneath `smartdrone`. Packages separate transport,
autonomy and simulation. Pure algorithms can be tested without a GPU, but their
tests do not advance physical flight milestones.
