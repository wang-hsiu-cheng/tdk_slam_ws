include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint", -- frame where imu locate
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,    -- close when using odometry
  use_odometry = true,           -- open when using odometry
  use_nav_sat = false,
  use_landmarks = false,
  use_pose_extrapolator = true,
  publish_frame_projected_to_2d = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 3e-2,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2.  -- 降低對 EKF 位移(底盤)的信心
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10.   -- 降低對 EKF 角度(底盤)的信心
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10. -- 加強對齊牆壁特徵
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 2.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15  -- 允許在 15 公分範圍內窮舉找牆壁
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.) -- 允許在 20 度範圍內窮舉旋轉對齊
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0

return options