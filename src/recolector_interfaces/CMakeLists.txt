cmake_minimum_required(VERSION 3.8)
project(recolector_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# TODO: add here every .msg the team defines in msg/
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineSensors.msg"
  "msg/CubeDetection.msg"
  "msg/CubeDetectionArray.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
