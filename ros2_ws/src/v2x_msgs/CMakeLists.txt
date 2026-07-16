cmake_minimum_required(VERSION 3.8)
project(v2x_msgs)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleState.msg"
  "msg/BrakeCommand.msg"
  DEPENDENCIES std_msgs)
ament_package()
