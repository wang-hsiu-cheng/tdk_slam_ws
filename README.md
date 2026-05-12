# Wildbot Localization

## Environment
### pre-installed pkgs and tools
- LiDAR plugin in Gazebo [source](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)
- rplidar_ros: S3 LiDAR firmware driver [source](https://github.com/Slamtec/rplidar_ros)
- phidgets_drivers: IMU firmware driver [source](https://github.com/ros-drivers/phidgets_drivers)

### How to setup
- `git clone -b feature-wildbot https://github.com/wang-hsiu-cheng/tdk_slam_ws.git`
- `cd docker && chmod +x run_dev.sh && ./run_dev.sh`
- `docker exec -it wildbot_slam bash`

### notice
- 以下檔案需要根據 wildbot 場地環境與機器人規格修改
```bash
/wildbot_slam_manager
├── config
│   ├── mapper_params_online_async.yaml
│   ├── robot_params.yaml    # 機器人大小、輪徑、LiDAR 位置 & 方向
│   └── slam_toolbox_params.yaml # 定位起始點、要使用的地圖檔案位置、各種定位參數
├── launch
│   ├── sim_spawn_launch.py  # 在 spawn_entity 修改模擬時機器的生成位置
│   └── spawn_launch.py      # 在 lidar_node 修改 serial_port 與 LiDAR 型號 (根據實際情況修改)
├── sim
│   ├── models
│   │   └── maze
│   │       ├── meshes       # 放場地的 CAD 圖檔
│   │       ├── model.config # 匯入 gazebo 所需的設定檔
│   │       └── model.sdf    # 匯入 gazebo 所需的設定檔
│   └── worlds
│       └── maze.world       # 場地匯入 gazebo 後的位置與方向
└── urdf
    └── sensors.urdf.xacro   # 換成 wildbot 機器人的 urdf (或是拿這份 urdf 去調整參數)
```
- 用teleop玩車的時候小心翻車。

## Localization
### pre-mapping mode
1. Launch all the programs for mapping: lidar filter, lidar merger, slam pkg, mapping pkg
    ```
    colcon build --symlink-install
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ```
    - if you're using simulation mode
        ```
        ros2 launch wildbot_slam_manager maze_world_launch.py
        ros2 launch wildbot_slam_manager sim_spawn_launch.py
        ```
        - remember to `ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'wildbot_robot'}"` brfore reopen sim_spawn_launch.py
    - if you're using real world mode:
        `ros2 launch wildbot_slam_manager spawn_launch.py`
2. Stop mapping and save map
    - save map for slam_toolbox: `ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: 'slam_map_0'}"`
    - save map for nav_amcl: `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'amcl_map_0'}}"`
    - save map for catographer: 
        1. 停止軌跡: `ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"`
        2. 序列化地圖 (類似 slam_toolbox 的 serialize): `ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/wildbot/wildbot_slam_ws/src/wildbot_slam_manager/maps/carto_map_0.pbstream'}"`
        3. 將地圖轉換成 nav2 可以使用的圖片: 
            ```
            ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
                -pbstream_filename /home/wildbot/wildbot_slam_ws/src/wildbot_slam_manager/carto_map_0.pbstream \
                -map_filestem /home/wildbot/wildbot_slam_ws/src/wildbot_slam_manager/maps/carto_map_0 \
                -resolution 0.05
            ```

### localization mode
1. modify localization_mode in `sim_spawn_launch.py` (or `spawn_launch.py`)
    ```py
    DeclareLaunchArgument('localization_mode', default_value='slam_toolbox')
    ```
2. Launch all the programs for mapping: lidar filter, lidar merger, slam pkgs
    - `source install/setup.bash`
    - if you're using simulation mode
        ```
        ros2 launch wildbot_slam_manager maze_world_launch.py
        ros2 launch wildbot_slam_manager sim_spawn_launch.py
        ```
        - execute `ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'wildbot_robot'}"` brfore reopen `sim_spawn_launch.py`
    - if you're using real world mode:
        `ros2 launch wildbot_slam_manager spawn_launch.py`

### Interact & Visulization
   - `rivz2`: can visualize every ros info (robot pose, tf, lidar scan)
   - `ros2 run teleop_twist_keyboard teleop_twist_keyboard`: move robot around. Can use to create map or test the localization quality

## Navigation
