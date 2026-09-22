# Launch entry points

Canonical, installed ROS launch files live in
[`ros2_ws/src/drone_bringup/launch`](../ros2_ws/src/drone_bringup/launch).
Keeping them in the ROS package lets `ros2 launch drone_bringup ...` work after
installation. This directory intentionally avoids a second drifting copy.
