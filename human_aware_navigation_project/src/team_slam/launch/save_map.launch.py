from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    save_map_node = Node(
        package='team_slam',
        executable='save_map',
        name='save_map',
        output='screen'
    )

    return LaunchDescription([save_map_node])
