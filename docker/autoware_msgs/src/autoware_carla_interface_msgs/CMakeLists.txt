cmake_minimum_required(VERSION 3.14)
project(autoware_carla_interface_msgs)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BridgeState.msg
  msg/EgoConfig.msg
  msg/SensorConfig.msg
  srv/AutowareTick.srv
  DEPENDENCIES builtin_interfaces std_msgs)

ament_auto_package()
