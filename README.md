# 定位與導航
  
## 專案架構


## 技術整合
### 預先掃描建圖
1. 雙 LiDAR fusion
    - LiDAR 型號: RPLiDAR C1, [Driver](https://github.com/Slamtec/rplidar_ros/tree/ros2)
    - laser_filters
    - ira_laser_tools: laserscan_multi_merger [source](https://github.com/nakai-omer/ira_laser_tools/tree/humble)
2. 建圖
    - package: SLAM Toolbox
    - execute: `ros2 launch tdk_slam_manager slam_localization.launch.py mode:=mapping`
    - save map for slam_toolbox: `ros2 service call /slam_toolbox/serialize_map slam_toolbox_msgs/srv/SerializePoseGraph "{filename: 'slam_map_0'}"`
    - save map for nav_amcl: `ros2 service call /slam_toolbox/save_map slam_toolbox_msgs/srv/SaveMap "{name: {data: 'amcl_map_0'}}"`
    - save map for catographer: 
        1. 停止軌跡: `ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"`
        2. 序列化地圖 (類似 slam_toolbox 的 serialize): `ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/ted/tdk_slam_ws/src/tdk_slam_manager/maps/carto_map_0.pbstream'}"`
        3. 將地圖轉換成 nav2 可以使用的圖片: 
            ```
            ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
                -pbstream_filename /home/ted/tdk_slam_ws/src/tdk_slam_manager/carto_map_0.pbstream \
                -map_filestem /home/ted/tdk_slam_ws/src/tdk_slam_manager/maps/carto_map_0 \
                -resolution 0.05
            ```
### SLAM 定位
- tf 關係說明:
    - tf: map->odom->base_footprint->base_link
    - ground truth: map->base_footprint (虛擬的 tf 關係)
    > 有回授的情況
    - odom->base_footprint: odometry 積分提供位移資訊
    - map->odom: 用 SLAM 校正，讓兩者盡量重疊
    > 沒有回授的情況
    - odom->base_footprint: 發送假的 tf 讓兩者完全重疊
    - map->odom: odom 跟隨機器移動，用 SLAM 計算當前 odom 偏離 map 多少

1. 雙 LiDAR fusion: 與建圖時相同
2. map->odom 更新: 
    - 純 SLAM 定位測試: 
        - SLAM Toolbox: `ros2 launch tdk_slam_manager slam_localization.launch.py mode:=slam_toolbox`
        - nav2_amcl: `ros2 launch tdk_slam_manager slam_localization.launch.py mode:=amcl`
    - 有底盤 odometry 資訊: 
        - 接收底盤 odometry 資訊作為預測資訊
        - SLAM Toolbox 或 nav2_amcl

### nav2 導航
- navigation2 工作流程: user -> nav_main -> planner -> controller

1. planner: nav2_smac_planner/SmacPlanner2d
2. controller: MPPI (調大 vx_std, vy_std)
3. nav_main: 使用 action 呼叫 BT root

## 實驗流程
1. 雙 LiDAR 掃描預先建圖: slam_toolbox, catographer
    - 項目:
    1. 純 LiDAR 掃描，調整 LiDAR fusion 角度與設定
    2. slam_toolbox: 比較 odometry 回授 & odometry+imu (EKF) 回授
    3. catographer: 比較 imu 回授 & odometry+imu (EKF) 回授
    - 驗證: 地圖形狀的精確度
2. 使用預先建圖結果進行純 LiDAR SLAM 定位: 
    - 比較 amcl, slam_toolbox, catographer 定位效果
    - 驗證:
    1. 靜止時的抖動
    2. 瞬間移動後的回復速度
    3. 遮擋容忍程度
    4. 更新速度與延遲
3. 使用上述較優秀方案進行有回授的 SLAM 定位+導航: 
    - 項目: 
    1. 根據定位方式選擇對應的回授: amcl(odometry), slam_toolbox(odometry), catographer(imu)
    2. 比較 SmacLattice+MPPI, Smac2D+DWB, ThetaStar+MPPI 導航效果
    - 驗證:
    1. 軌跡平滑程度
    2. 機器人是否按照 planner path 走動
    3. 電腦 CPU 使用量
4. 使用雙 LiDAR 進行即時建圖純 LiDAR SLAM 定位: 
    1. 比較 slam_toolbox, catographer 定位效果

### 數據回放
- rosbag: /tf, /front/scan, /rear/scan, /scan, /odom, /initialpose
- foxglove: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`

## 環境
<!-- ### 需要手動安裝的工具
- install: 
  - ros-humble-pcl-ros
- build from source:
  - https://github.com/Slamtec/rplidar_ros/tree/ros2
  - https://github.com/nakai-omer/ira_laser_tools/tree/humble
  - https://github.com/ros-drivers/phidgets_drivers.git -->