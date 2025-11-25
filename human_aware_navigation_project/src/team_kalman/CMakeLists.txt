cmake_minimum_required(VERSION 3.8)
project(team_kalman)

# ---------------------------------------
# Find dependencies
# ---------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(team_lidar REQUIRED)

# ---------------------------------------
# Message generation
# ---------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PredictedObject.msg"
)

ament_export_dependencies(
  rosidl_default_runtime
  team_lidar
)

# ---------------------------------------
# Install Python node
# ---------------------------------------
install(
  PROGRAMS
    scripts/human_predictor.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------
# Install launch files
# ---------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------
# Package
# ---------------------------------------
ament_package()
