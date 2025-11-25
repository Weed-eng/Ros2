cmake_minimum_required(VERSION 3.8)
project(team_slam)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# ------------------------------------------
# Install Python scripts
# ------------------------------------------
install(
  PROGRAMS
    scripts/save_map.py
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
# Install config files
# ------------------------------------------
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------------------
# Final
# ------------------------------------------
ament_package()

