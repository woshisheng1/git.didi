include "map_builder.lua"                 -- 引入 Cartographer 默认的地图构建器配置，生成全局表 MAP_BUILDER
include "trajectory_builder.lua"          -- 引入 Cartographer 默认的轨迹构建器配置，生成全局表 TRAJECTORY_BUILDER

options = {                               -- 定义最终返回给 Cartographer 的配置表
  map_builder = MAP_BUILDER,              -- 使用上面引入的 MAP_BUILDER 配置
  trajectory_builder = TRAJECTORY_BUILDER,-- 使用上面引入的 TRAJECTORY_BUILDER 配置
  map_frame = "map",                      -- 地图坐标系名称
  tracking_frame = "gyro_link",      -- SLAM 内部跟踪使用的坐标系，传感器数据会转换到此坐标系
  published_frame = "odom_combined",      -- 对外发布位姿时使用的机器人坐标系
  odom_frame = "odom_combined",           -- 里程计坐标系名称
  provide_odom_frame = false,             -- 由 Cartographer 提供 odom 坐标系，并发布 map->odom 与 odom->published_frame
  publish_frame_projected_to_2d = true,   -- 发布位姿时强制投影到 2D 平面
  use_pose_extrapolator = true,           -- 启用位姿外推器，用于短期位姿预测
  use_odometry = true,                    -- 使用轮式里程计
  use_nav_sat = false,                    -- 不使用卫星导航数据
  use_landmarks = false,                  -- 不使用路标数据
  num_laser_scans = 1,                    -- 使用普通单回波 LaserScan
  num_multi_echo_laser_scans = 0,         -- 不使用多回波 MultiEchoLaserScan 话题
  num_subdivisions_per_laser_scan = 1,   --不把来打拆封，单线程激光雷达每个线时间一致拆封脑部时间会导致精度降低
  num_point_clouds = 0,                   -- 不使用 PointCloud2 点云
  lookup_transform_timeout_sec = 0.2,     -- 查询 TF 变换的超时时间，单位秒
  submap_publish_period_sec = 0.3,        -- 子图发布周期，单位秒
  pose_publish_period_sec =5e-2,         -- 位姿发布周期，5e-3 秒 = 0.005 秒，即 200 Hz
  trajectory_publish_period_sec = 30e-3,  -- 轨迹发布周期，30e-3 秒 = 0.03 秒，约 33.3 Hz
  rangefinder_sampling_ratio = 1.,        -- 测距数据采样比例，1. 表示全部使用
  odometry_sampling_ratio = 1.,           -- 里程计数据采样比例，1. 表示全部使用
  fixed_frame_pose_sampling_ratio = 1.,   -- 固定坐标系位姿采样比例，1. 表示全部使用
  imu_sampling_ratio = 1.,                -- IMU 数据采样比例，1. 表示全部使用
  landmarks_sampling_ratio = 1.,          -- 路标数据采样比例，1. 表示全部使用
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- 【核心修正 3】强制开启 IMU 数据融合
TRAJECTORY_BUILDER_2D.use_imu_data = true
--激光雷达裁剪
TRAJECTORY_BUILDER_2D.min_range = 0.16
TRAJECTORY_BUILDER_2D.max_range = 8.0

TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 30.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 70.0

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.max_constraint_distance = 8.0
POSE_GRAPH.constraint_builder.min_score = 0.45
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
return options