cmake_minimum_required(VERSION 3.8)
project(aeris_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/FlightStatus.msg"
  "msg/GridStatus.msg"
  "msg/MissionStatus.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/SafetyStatus.msg"
  "srv/GoTo.srv"
  "srv/GridGoal.srv"
  "srv/InjectFault.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
