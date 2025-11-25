from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Whether to launch SLAM node'
    )
    
    use_rviz_goal_arg = DeclareLaunchArgument(
        'use_rviz_goal',
        default_value='true',
        description='Use RViz goal input (true) or fixed goal (false)'
    )
    
    fixed_goal_x_arg = DeclareLaunchArgument(
        'fixed_goal_x',
        default_value='2.0',
        description='Fixed goal X coordinate (if use_rviz_goal=false)'
    )
    
    fixed_goal_y_arg = DeclareLaunchArgument(
        'fixed_goal_y',
        default_value='0.0',
        description='Fixed goal Y coordinate (if use_rviz_goal=false)'
    )

    # Get launch configuration
    use_slam = LaunchConfiguration('use_slam')
    use_rviz_goal = LaunchConfiguration('use_rviz_goal')
    fixed_goal_x = LaunchConfiguration('fixed_goal_x')
    fixed_goal_y = LaunchConfiguration('fixed_goal_y')

    # ============================================
    # PERCEPTION PIPELINE
    # ============================================
    
    # LiDAR Detector Node
    lidar_node = Node(
        package='team_lidar',
        executable='lidar_detector.py',
        name='lidar_detector',
        output='screen'
    )

    # Kalman Predictor Node
    kalman_node = Node(
        package='team_kalman',
        executable='human_predictor.py',
        name='kalman_predictor',
        output='screen'
    )

    # ============================================
    # PLANNING PIPELINE
    # ============================================
    
    # Adaptive DWA Node
    dwa_node = Node(
        package='team_adaptive_dwa',
        executable='adaptive_dwa_node.py',
        name='adaptive_dwa',
        output='screen',
        parameters=[]
    )

    # Safety Node
    safety_node = Node(
        package='team_safety',
        executable='safety_node.py',
        name='safety_node',
        output='screen',
        parameters=[]
    )

    # Integration Node
    integration_node = Node(
        package='team_integration',
        executable='integration_node.py',
        name='team_integration',
        output='screen',
        parameters=[{
            'use_rviz_goal': use_rviz_goal,
            'fixed_goal_x': fixed_goal_x,
            'fixed_goal_y': fixed_goal_y
        }]
    )

    # ============================================
    # SLAM (Optional)
    # ============================================
    
    # SLAM Node (conditional)
    slam_param_file = os.path.join(
        get_package_share_directory('team_slam'),
        'config',
        'slam_params.yaml'
    )
    
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_param_file, {'use_sim_time': True}],
        condition=IfCondition(use_slam)
    )

    return LaunchDescription([
        # Launch arguments
        use_slam_arg,
        use_rviz_goal_arg,
        fixed_goal_x_arg,
        fixed_goal_y_arg,
        
        # Perception pipeline
        lidar_node,
        kalman_node,
        
        # Planning pipeline
        dwa_node,
        safety_node,
        integration_node,
        
        # SLAM (optional)
        slam_node,
    ])

