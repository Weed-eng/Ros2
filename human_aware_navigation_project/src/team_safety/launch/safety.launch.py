from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="team_safety",
            executable="safety_node.py",
            name="safety_node",
            output="screen",
            parameters=[]
        )
    ])
