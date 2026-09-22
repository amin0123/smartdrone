# Navigation

`drone_navigation.planner` creates a boustrophedon route in ENU. Length follows
east (+X), width follows north (+Y). Rows alternate direction and adjacent row
endpoints are connected. A non-divisible width adds the final boundary row;
positive dimensions and finite values are required.

```text
North +Y
  ^     ─────────────→
  |     ↑
  |     ←─────────────
  |                   ↑
  |     ─────────────→
  +----------------------> East +X
```

The route generator never publishes PX4 messages. `RouteFollower` advances a
position reference at configured speed and pauses advancement if the aircraft
lags too far behind. It requires measured arrival at a corner before advancing
to the next leg. Executor delays cannot jump the reference across the route.
PX4 enforces acceleration and velocity limits while tracking these positions.

Static obstacle checks test whole line segments against inflated boxes, not just
waypoint endpoints. The field's obstacles are deliberately outside the initial
flight corridor. This is not a dynamic obstacle avoidance system: moving people,
unexpected objects or dense greenhouse structures require a local 3D planner
and suitable depth/LiDAR sensing before deployment. A nominal crop-row route is
not a proof that every leaf was seen; camera footprint and image quality remain
separate coverage criteria.

Inspection altitude, speed, field dimensions, row spacing, tracking tolerance,
lead bound and geofence are in config/mission.yaml. Keep the scene geometry in
config/world.yaml consistent with the configured route. Tests cover partial
rows, invalid units, obstacles between endpoints, corner order and stalled
executors.
