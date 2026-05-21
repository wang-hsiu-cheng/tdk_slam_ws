import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    localization_mode = LaunchConfiguration('localization_mode', default='mapping')
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('wildbot_slam_manager'), 'config', 'ekf_config.yaml']),
            {'use_sim_time': use_sim_time}]
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('tdk_slam_manager'), 'config', 'filter_config.yaml']),
            {'use_sim_time': use_sim_time}]
    )

    mapping_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'mapping'"])),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('wildbot_slam_manager'), 'config', 'mapper_params_online_async.yaml']),
            {'use_sim_time': use_sim_time}]
    )

    localization_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'slam_toolbox'"])),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('wildbot_slam_manager'), 'config', 'slam_toolbox_params.yaml']),
            {
                'mode': PythonExpression(["'mapping' if '", localization_mode, "' == 'mapping' else 'localization'"]),
                'use_sim_time': use_sim_time
            }
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization_mode', default_value='mapping'),

        ekf_node,
        laser_filter_node,
        mapping_node,
        localization_node
    ])