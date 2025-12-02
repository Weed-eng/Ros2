================================================================================
HOW TO RUN: Human-Aware Navigation Project
================================================================================

This guide will walk you through setting up and running the Human-Aware 
Navigation project from scratch on a fresh Ubuntu Linux VM.

================================================================================
TABLE OF CONTENTS
================================================================================

1. Prerequisites
2. Ubuntu VM Setup
3. ROS 2 Installation
4. System Dependencies
5. Workspace Setup
6. Building the Workspace
7. Running the System
8. Verification
9. Troubleshooting

================================================================================
1. PREREQUISITES
================================================================================

- Ubuntu Linux VM (Ubuntu 22.04 recommended for ROS 2 Humble)
- At least 4GB RAM (8GB recommended)
- At least 20GB free disk space
- Internet connection for downloading packages
- Basic familiarity with Linux terminal commands

================================================================================
2. UBUNTU VM SETUP
================================================================================

2.1 Update System Packages
---------------------------
Open a terminal and run:

    sudo apt update
    sudo apt upgrade -y

2.2 Install Essential Build Tools
----------------------------------
    sudo apt install -y \
        build-essential \
        git \
        wget \
        curl \
        vim \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool

2.3 Set Locale (if needed)
---------------------------
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

================================================================================
3. ROS 2 INSTALLATION
================================================================================

3.1 Add ROS 2 Repository
--------------------------
For Ubuntu 22.04 (ROS 2 Humble):

    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

3.2 Install ROS 2 Humble Desktop
---------------------------------
    sudo apt update
    sudo apt install -y ros-humble-desktop

3.3 Source ROS 2 Setup
-----------------------
Add to your ~/.bashrc file:

    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Verify installation:

    ros2 --help

================================================================================
4. SYSTEM DEPENDENCIES
================================================================================

4.1 Install ROS 2 Dependencies
-------------------------------
    sudo apt install -y \
        ros-humble-slam-toolbox \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-turtlebot3* \
        python3-rosdep

4.2 Initialize rosdep
----------------------
    sudo rosdep init
    rosdep update

4.3 Install Python Dependencies
--------------------------------
    pip3 install numpy

Note: If you encounter permission issues, use:
    pip3 install --user numpy

================================================================================
5. WORKSPACE SETUP
================================================================================

5.1 Create ROS 2 Workspace
---------------------------
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src

5.2 Clone/Copy Project
-----------------------
If you have the project in a git repository:

    git clone <your-repository-url> .

Or if you have the project files locally, copy them to ~/ros2_ws/src/:

    cp -r /path/to/human_aware_navigation_project/* ~/ros2_ws/src/

Verify the structure:

    ls ~/ros2_ws/src/

You should see directories like:
    - team_adaptive_dwa
    - team_integration
    - team_kalman
    - team_lidar
    - team_safety
    - team_slam

5.3 Install Package Dependencies
----------------------------------
From the workspace root:

    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y

This will automatically install all ROS 2 dependencies declared in package.xml files.

================================================================================
6. BUILDING THE WORKSPACE
================================================================================

6.1 Navigate to Workspace Root
-------------------------------
    cd ~/ros2_ws

6.2 Build the Workspace
------------------------
    colcon build --symlink-install

The --symlink-install flag allows you to edit Python scripts without rebuilding.

6.3 Source the Workspace
-------------------------
Add to your ~/.bashrc:

    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Or source manually each time:

    source ~/ros2_ws/install/setup.bash

6.4 Verify Build
-----------------
Check that packages are recognized:

    ros2 pkg list | grep team

You should see:
    team_adaptive_dwa
    team_integration
    team_kalman
    team_lidar
    team_safety
    team_slam

================================================================================
7. RUNNING THE SYSTEM
================================================================================

7.1 Prerequisites for Running
-------------------------------
Before running the navigation system, you need:

a) A robot simulation or real robot providing:
   - /scan topic (sensor_msgs/LaserScan)
   - /odom topic (nav_msgs/Odometry)
   - TF transforms: odom -> base_link

b) For simulation: Webots (if using Webots simulation)
   OR Gazebo (if using Gazebo simulation)

7.2 Webots Setup (if using Webots simulation)
-----------------------------------------------
Install Webots:

    wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -
    sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'
    sudo apt update
    sudo apt install -y webots

Or download from: https://cyberbotics.com/

7.3 Launch the Complete System
-------------------------------
Open a terminal and source the workspace:

    source ~/ros2_ws/install/setup.bash

Launch the main navigation system:

    ros2 launch team_integration human_aware_navigation.launch.py

7.4 Launch Options
------------------
With SLAM enabled (default):

    ros2 launch team_integration human_aware_navigation.launch.py use_slam:=true

Without SLAM (if you have a pre-built map):

    ros2 launch team_integration human_aware_navigation.launch.py use_slam:=false

With fixed goal (instead of RViz goal):

    ros2 launch team_integration human_aware_navigation.launch.py \
        use_rviz_goal:=false \
        fixed_goal_x:=2.0 \
        fixed_goal_y:=0.0

7.5 Launch Individual Components (for testing)
-----------------------------------------------
Launch only LiDAR detector:

    ros2 launch team_lidar lidar.launch.py

Launch only SLAM:

    ros2 launch team_slam slam.launch.py

Launch only Kalman predictor:

    ros2 launch team_kalman kalman.launch.py

7.6 Using RViz for Visualization
---------------------------------
The launch file should automatically start RViz. If not, manually launch:

    rviz2 -d ~/ros2_ws/src/team_integration/rviz/navigation.rviz

In RViz:
- Use "2D Nav Goal" tool to set navigation goals
- Visualize /scan, /map, robot model, etc.

7.7 Running with Simulation
-----------------------------
If using Webots:

Terminal 1 - Start Webots world:
    cd ~/ros2_ws/src/webots_world/worlds
    webots turtlebot3_burger.wbt

Terminal 2 - Start ROS 2 bridge (if needed):
    source ~/ros2_ws/install/setup.bash
    # Follow Webots ROS 2 bridge setup instructions

Terminal 3 - Launch navigation:
    source ~/ros2_ws/install/setup.bash
    ros2 launch team_integration human_aware_navigation.launch.py

================================================================================
8. VERIFICATION
================================================================================

8.1 Check Running Nodes
------------------------
In a new terminal:

    source ~/ros2_ws/install/setup.bash
    ros2 node list

You should see nodes like:
    /lidar_detector
    /kalman_predictor
    /adaptive_dwa
    /safety_node
    /team_integration
    /slam_toolbox (if use_slam:=true)

8.2 Check Topics
-----------------
    ros2 topic list

You should see topics like:
    /scan
    /moving_objects
    /predicted_object
    /cmd_vel
    /goal_pose
    /map (if SLAM is running)
    /odom

8.3 Check TF Tree
-----------------
    ros2 run tf2_tools view_frames

This creates a frames.pdf showing the TF tree. Verify:
    map -> odom -> base_link

8.4 Monitor Topic Data
-----------------------
Check LiDAR scan:

    ros2 topic echo /scan

Check moving objects:

    ros2 topic echo /moving_objects

Check velocity commands:

    ros2 topic echo /cmd_vel

8.5 Test Navigation
--------------------
1. Launch the system
2. In RViz, use "2D Nav Goal" to set a goal
3. Watch the robot navigate (or check /cmd_vel for velocity commands)
4. Verify the robot avoids obstacles and humans

================================================================================
9. TROUBLESHOOTING
================================================================================

9.1 Build Errors
----------------

Error: "Package 'team_xxx' not found"
Solution: Make sure you've sourced the workspace:
    source ~/ros2_ws/install/setup.bash

Error: "CMake Error: Could not find ament_cmake"
Solution: Install ROS 2 development tools:
    sudo apt install -y ros-humble-ament-cmake

Error: "No module named 'numpy'"
Solution: Install numpy:
    pip3 install numpy
    # Or: pip3 install --user numpy

Error: Custom messages not found
Solution: Build message packages first:
    cd ~/ros2_ws
    colcon build --packages-select team_lidar team_kalman
    source install/setup.bash
    colcon build

9.2 Runtime Errors
------------------

Error: "Could not find package 'slam_toolbox'"
Solution: Install slam_toolbox:
    sudo apt install -y ros-humble-slam-toolbox

Error: "TF lookup failed"
Solution: Check TF tree:
    ros2 run tf2_tools view_frames
Ensure odom -> base_link transform exists (from robot/simulator)

Error: "No /scan topic"
Solution: Ensure robot/simulation is running and publishing /scan
Check with: ros2 topic list
Monitor with: ros2 topic echo /scan

Error: "use_sim_time is true but no simulation"
Solution: Either:
    a) Start your simulation (Webots/Gazebo)
    b) Set use_sim_time:=false in slam_params.yaml

9.3 Python Script Errors
-------------------------

Error: "Permission denied" when running scripts
Solution: Make scripts executable:
    chmod +x ~/ros2_ws/src/team_*/scripts/*.py

Error: "ModuleNotFoundError: No module named 'team_xxx'"
Solution: Rebuild and source:
    cd ~/ros2_ws
    colcon build
    source install/setup.bash

9.4 SLAM Issues
---------------

Error: "SLAM not building map"
Solution: 
    - Check /scan topic is publishing: ros2 topic echo /scan
    - Check robot is moving (odom should change)
    - Verify slam_params.yaml scan_topic matches your /scan topic

Error: "Map frame not found"
Solution: Ensure SLAM node is running:
    ros2 node list | grep slam

9.5 Navigation Issues
---------------------

Error: "Robot not moving"
Solution:
    - Check /cmd_vel topic: ros2 topic echo /cmd_vel
    - Verify goal is set: ros2 topic echo /goal_pose
    - Check for errors in node logs

Error: "Robot stuck or oscillating"
Solution:
    - Check for obstacles in /scan
    - Verify DWA parameters are reasonable
    - Check safety node isn't blocking movement

9.6 Common Fixes Summary
-------------------------

If nothing works, try a clean rebuild:

    cd ~/ros2_ws
    rm -rf build install log
    colcon build --symlink-install
    source install/setup.bash

Check all dependencies:

    rosdep install --from-paths src --ignore-src -r -y

Verify ROS 2 installation:

    printenv | grep ROS
    # Should show ROS_DISTRO=humble

================================================================================
10. QUICK START SUMMARY
================================================================================

For experienced users, here's the condensed version:

1. Install ROS 2 Humble:
   sudo apt install -y ros-humble-desktop
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

2. Install dependencies:
   sudo apt install -y ros-humble-slam-toolbox python3-colcon-common-extensions
   pip3 install numpy

3. Setup workspace:
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   # Copy/clone your project here

4. Build:
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc

5. Run:
   ros2 launch team_integration human_aware_navigation.launch.py

================================================================================
END OF GUIDE
================================================================================

For additional help, check:
- ROS 2 documentation: https://docs.ros.org/en/humble/
- slam_toolbox: https://github.com/SteveMacenski/slam_toolbox
- Webots documentation: https://cyberbotics.com/doc/guide/

Good luck with your Human-Aware Navigation project!

