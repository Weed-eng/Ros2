cmake_minimum_required(VERSION 3.8)
project(team_safety)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(team_kalman REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

install(
  PROGRAMS
    scripts/safety_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
