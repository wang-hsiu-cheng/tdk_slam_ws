import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # define path
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    navigation_pkg = get_package_share_directory('tdk_nav2_manager')
    # define launch params
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    planner_type = LaunchConfiguration('planner', default='smac_2d') # smac_2d, smac_lattice, theta_star
    controller_type = LaunchConfiguration('controller', default='mppi') # dwb, mppi
    bt_file = os.path.join(navigation_pkg, 'config', 'bt_nav.yaml')
    global_costmap_file = os.path.join(navigation_pkg, 'costmap_config', 'global_costmap.yaml')
    local_costmap_file = os.path.join(navigation_pkg, 'costmap_config', 'local_costmap.yaml')
    # read config file
    planner_params = PathJoinSubstitution([
        navigation_pkg,
        'config',
        PythonExpression([planner_type, "' + '_params.yaml'"])
    ])
    controller_params = PathJoinSubstitution([
        navigation_pkg,
        'config',
        PythonExpression([controller_type, "' + '_params.yaml'"])
    ])

    # use official bringup pkg
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': [bt_file, ' ', global_costmap_file, ' ', local_costmap_file, ' ', planner_params, ' ', controller_params],
            'use_lifecycle_mgr': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('planner_type', default_value='smac_2d'),
        DeclareLaunchArgument('controller_type', default_value='mppi'),
        nav2_launch
    ])