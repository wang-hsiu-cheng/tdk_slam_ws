import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    localization_pkg = os.path.join('/home/tdk/tdk_slam_ws/src/tdk_slam_manager')
    sllidar_pkg = get_package_share_directory('rplidar_ros')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    localization_mode = LaunchConfiguration('localization_mode', default='mapping')

    xacro_file = os.path.join(localization_pkg, 'urdf', 'sensors.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    slam_map_file_name = os.path.join(localization_pkg, 'maps', 'slam_map_3')
    carto_map_file_name = os.path.join(localization_pkg, 'maps', 'carto_map_3')

    world_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_static_publisher',
        output='screen',
        arguments=[
            '--x', '0.425',
            '--y', '1.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'map'
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('wildbot_slam_manager'), 'config', 'ekf_config.yaml']),
            {'use_sim_time': use_sim_time}]
    )

    # start RPLiDAR S3
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

    filter_front = Node(
        package='tdk_slam_manager',
        executable='laser_angle_filter_node',
        name='filter_front',
        parameters=[{
            'lower_angle': -3.1415,
            'upper_angle': -1.5708,
            'input_topic': '/front/scan',
            'output_topic': '/front/scan_filtered'
        }]
    )

    filter_rear = Node(
        package='tdk_slam_manager',
        executable='laser_angle_filter_node',
        name='filter_rear',
        parameters=[{
            'lower_angle': -3.1415,
            'upper_angle': -1.5708,
            'input_topic': '/rear/scan',
            'output_topic': '/rear/scan_filtered'
        }]
    )

    merger_node = Node(
        package='ira_laser_tools',
        executable='laserscan_multi_merger',
        name='laser_merger',
        parameters=[PathJoinSubstitution([FindPackageShare('tdk_slam_manager'), 'config', 'laser_merger_params.yaml']),
            {'use_sim_time': use_sim_time}],
        output='screen'
    )

    mapping_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'mapping'"])),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('tdk_slam_manager'), 'config', 'mapper_params_online_async.yaml']),
            {'use_sim_time': use_sim_time}]
    )

    robot_pose_publisher_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' in ['slam_toolbox', 'cartographer']"])),
        package='tdk_slam_manager',
        executable='robot_pose_publisher_node',
        name='robot_pose_pub',
        parameters=[{
            'parent_frame': "world",
            'child_frame': "base_footprint"
        }]
    )

    localization_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'slam_toolbox'"])),
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('tdk_slam_manager'), 'config', 'slam_toolbox_params.yaml']),
            {
                'mode': "localization",
                'map_start_pose': [0.0, 0.0, 0.0],
                'map_file_name': slam_map_file_name,
                'use_sim_time': use_sim_time
            }
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
            ('odom', '/odometry/filtered')
        ]
    )
    # Convert Submap to OccupancyGrid
    occupancy_grid_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'carto_mapping'"])),
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05']
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
            '-load_state_filename', carto_map_file_name + '.pbstream'
        ],
        remappings=[
            ('odom', '/odometry/filtered')
        ]
    )

    # map_server
    map_server_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'cartographer'"])),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': carto_map_file_name + ".yaml"},
            {'use_sim_time': use_sim_time}
        ]
    )
    lifecycle_manager_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'cartographer'"])),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},           
            {'node_names': ['map_server']} 
        ]
    )

    localization_manager_node = Node(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' in ['slam_toolbox', 'cartographer']"])),
        package='tdk_slam_manager',
        executable='localization_manager_node',
        name='localization_manager',
        output='screen',
        parameters=[{
            'slam_type': localization_mode,
            'world_to_map_x': 0.425,
            'world_to_map_y': 1.0,
            'tolerance_dist': 0.05,
            'tolerance_yaw': 0.05
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization_mode', default_value='carto_mapping'),

        world_tf_pub,
        robot_state_publisher,
        ekf_node,
        lidar_front,
        lidar_rear,
        filter_front,
        filter_rear,  
        merger_node,

        mapping_node,
        localization_node,

        cartographer_mapping_node,
        occupancy_grid_node,
        cartographer_node,
        map_server_node,
        
        robot_pose_publisher_node,
        localization_manager_node
    ])