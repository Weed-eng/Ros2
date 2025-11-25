cmake_minimum_required(VERSION 3.8)
project(team_lidar)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ------------------------------------------
# Message generation
# ------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MovingObject.msg"
)

ament_export_dependencies(
  rosidl_default_runtime
  rclpy
  sensor_msgs
  std_msgs
)

# ------------------------------------------
# Install Python scripts
# ------------------------------------------
install(
  PROGRAMS
    scripts/lidar_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------
# Install launch files
# ------------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------
# Final
# ------------------------------------------
ament_package()
