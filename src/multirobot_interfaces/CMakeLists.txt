cmake_minimum_required(VERSION 3.8)
project(multirobot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotPose.msg"
  "msg/RobotState.msg"
  "msg/CubeDetection.msg"
  "msg/CubeDetectionArray.msg"
  "srv/RequestZone.srv"
  "srv/ReleaseZone.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
