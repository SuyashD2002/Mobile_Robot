cmake_minimum_required(VERSION 3.8)

project(robot_system)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(lidar_node
  src/lidar_node.cpp
)

ament_target_dependencies(
  lidar_node
  rclcpp
  std_msgs
  
)

add_executable(robot_controller_node
  src/robot_controller_node.cpp
  src/Robot.cpp
  src/StateManager.cpp
)
add_executable(cmd_vel_watchdog
  src/cmd_vel_watchdog.cpp
)

ament_target_dependencies(
  cmd_vel_watchdog
  rclcpp
  geometry_msgs
)
add_executable(gazebo_lidar_node
  src/gazebo_lidar_node.cpp
)

ament_target_dependencies(
  gazebo_lidar_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(
  robot_controller_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
)
add_executable(battery_node
  src/battery_node.cpp
)

ament_target_dependencies(
  battery_node
  rclcpp
  std_msgs
)

install(
  TARGETS
  lidar_node
  robot_controller_node
  battery_node
  cmd_vel_watchdog
   gazebo_lidar_node
  DESTINATION lib/${PROJECT_NAME}
 
)
install(
  DIRECTORY
    launch
    urdf
    worlds
    config
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/dashboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dashboard
)


ament_package()

