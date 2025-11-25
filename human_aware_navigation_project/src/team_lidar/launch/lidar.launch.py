from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='team_lidar',
            executable='lidar_detector.py',
            name='lidar_detector',
            output='screen'
        )
    ])
