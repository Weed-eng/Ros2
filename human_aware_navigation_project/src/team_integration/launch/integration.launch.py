from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="team_integration",
            executable="integration_node.py",
            name="team_integration",
            output="screen",
        )
    ])
