from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="team_kalman",
            executable="human_predictor.py",
            name="kalman_predictor",
            output="screen"
        )
    ])
