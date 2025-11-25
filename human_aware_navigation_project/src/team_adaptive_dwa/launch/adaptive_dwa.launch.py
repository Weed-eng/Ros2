from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="team_adaptive_dwa",
            executable="adaptive_dwa_node.py",
            name="adaptive_dwa",
            output="screen",
            parameters=[]
        )
    ])
