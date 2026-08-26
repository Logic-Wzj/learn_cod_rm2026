cmake_minimum_required(VERSION 3.8)
project(cod_referee_simulator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cod_referee_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(referee_simulator_node
  src/referee_simulator_node.cpp
)
ament_target_dependencies(referee_simulator_node
  rclcpp
  cod_referee_interfaces
  std_msgs
)

add_executable(referee_keyboard
  src/referee_keyboard.cpp
)
ament_target_dependencies(referee_keyboard
  rclcpp
  std_msgs
)

install(TARGETS
  referee_simulator_node
  referee_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
