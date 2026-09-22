cmake_minimum_required(VERSION 3.16)
project(drone_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/VehicleState.msg"
  "msg/FlightTarget.msg"
  "msg/SafetyStatus.msg"
  "msg/MissionStatus.msg"
  "msg/InspectionDetection.msg"
  "msg/DetectionArray.msg"
  "msg/DockObservation.msg"
  "msg/CommandResult.msg"
  "srv/VehicleCommand.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()

