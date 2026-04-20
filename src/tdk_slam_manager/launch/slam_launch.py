import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    localization_pkg = get_package_share_directory('tdk_slam_manager')
    sllidar_pkg = get_package_share_directory('rplidar_ros')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    predict_mode = LaunchConfiguration('predict_mode', default='na') # na or odometry or imu
    localization_mode = LaunchConfiguration('localization_mode', default='slam_toolbox') # amcl, slam_toolbox, cartographer, mapping, carto_mapping
    map_yaml_file = os.path.join(localization_pkg, 'maps', 'tdk_map_0.yaml')
    xacro_file = os.path.join(get_package_share_directory('tdk_slam_manager'), 'urdf', 'sensors.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_raw}]
    )
    
    # start RPLiDAR C1
    lidar_front = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sllidar_pkg, 'launch', 'sllidar_s3_launch.py')),
        launch_arguments={'serial_port': '/dev/ttyUSB0', 'frame_id': 'laser_front', 'inverted': 'true'}.items(),
        namespace='front'
    )
    lidar_rear = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sllidar_pkg, 'launch', 'sllidar_s3_launch.py')),
        launch_arguments={'serial_port': '/dev/ttyUSB1', 'frame_id': 'laser_rear', 'inverted': 'true'}.items(),
        namespace='rear'
    )
    # laser_filters
    filter_front = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='filter_front',
        parameters=[os.path.join(localization_pkg, 'config', 'laser_filter_params.yaml')],
        namespace='front',
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_front_filtered')]
    )
    filter_rear = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='filter_rear',
        parameters=[os.path.join(localization_pkg, 'config', 'laser_filter_params.yaml')],
        namespace='rear',
        remappings=[('scan', '/scan'), ('scan_filtered', '/scan_rear_filtered')]
    )

    # ira_laser_tools
    merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laser_merger',
        parameters=[os.path.join(localization_pkg, 'config', 'laser_merger_params.yaml')]
    )

    # mapping
    mapping_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'mapping'"])),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(localization_pkg, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )
    # Cartographer mapping
    cartographer_mapping_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'carto_mapping'"])),
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(localization_pkg, 'cartographer_config'),
            '-configuration_basename', 'cartographer_2d.lua'
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/odom')
        ]
    )
    # Convert Submap to OccupancyGrid
    occupancy_grid_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' in ['carto_mapping', 'cartographer']"])),
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05']
    )
    fake_ft_publisher = Node(
        condition=IfCondition(PythonExpression(["'", predict_mode, "' == 'na'"])),
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )

    # Cartographer localization
    cartographer_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'cartographer'"])),
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', os.path.join(localization_pkg, 'cartographer_config'),
            '-configuration_basename', 'localization.lua',
            '-load_state_filename', os.path.join(localization_pkg, 'maps', 'tdk_map_0.pbstream')
        ],
    )

    # SLAM Toolbox
    slam_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'slam_toolbox'"])),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(localization_pkg, 'config', 'slam_toolbox_params.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

    # map_server
    map_server_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"])),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file},
                    {'use_sim_time': use_sim_time}]
    )
    # Lifecycle Manager: activate map server
    lifecycle_manager_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"])),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}] # the node need to configured
    )
    # AMCL
    amcl_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"])),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(localization_pkg, 'config', 'amcl_params.yaml'),
                    {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('predict_mode', default_value='odometry'),
        DeclareLaunchArgument('localization_mode', default_value='slam_toolbox'),
        robot_state_publisher,
        lidar_front,
        lidar_rear,
        filter_front,
        filter_rear,
        merger_node,
        mapping_node,
        cartographer_mapping_node,
        occupancy_grid_node,
        fake_ft_publisher,
        cartographer_node,
        slam_node,
        map_server_node,
        lifecycle_manager_node,
        amcl_node
    ])