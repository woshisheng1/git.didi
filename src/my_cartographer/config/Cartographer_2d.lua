include "map_builder.lua"                 -- 引入 Cartographer 默认的地图构建器配置，生成全局表 MAP_BUILDER
include "trajectory_builder.lua"          -- 引入 Cartographer 默认的轨迹构建器配置，生成全局表 TRAJECTORY_BUILDER

options = {                               -- 定义最终返回给 Cartographer 的配置表
  map_builder = MAP_BUILDER,              -- 使用上面引入的 MAP_BUILDER 配置
  trajectory_builder = TRAJECTORY_BUILDER,-- 使用上面引入的 TRAJECTORY_BUILDER 配置
  map_frame = "map",                      -- 地图坐标系名称
  tracking_frame = "base_link",           -- SLAM 内部跟踪使用的坐标系，传感器数据会转换到此坐标系
  published_frame = "base_link",          -- 对外发布位姿时使用的机器人坐标系
  odom_frame = "odom",                    -- 里程计坐标系名称
  provide_odom_frame = true,              -- 由 Cartographer 提供 odom 坐标系，并发布 map->odom 与 odom->published_frame
  publish_frame_projected_to_2d = false,  -- 发布位姿时不强制投影到 2D 平面
  use_pose_extrapolator = true,           -- 启用位姿外推器，用于短期位姿预测
  use_odometry = true,                   -- 不使用轮式里程计
  use_nav_sat = false,                    -- 不使用卫星导航数据
  use_landmarks = false,                  -- 不使用路标数据
  num_laser_scans = 0,                    -- 不使用普通单回波 LaserScan
  num_multi_echo_laser_scans = 1,         -- 使用 1 个多回波 MultiEchoLaserScan 话题
  num_subdivisions_per_laser_scan = 10,   -- 每个激光扫描拆分为 10 个子扫描
  num_point_clouds = 0,                   -- 不使用 PointCloud2 点云
  lookup_transform_timeout_sec = 0.2,     -- 查询 TF 变换的超时时间，单位秒
  submap_publish_period_sec = 0.3,        -- 子图发布周期，单位秒
  pose_publish_period_sec = 5e-3,         -- 位姿发布周期，5e-3 秒 = 0.005 秒，即 200 Hz
  trajectory_publish_period_sec = 30e-3,  -- 轨迹发布周期，30e-3 秒 = 0.03 秒，约 33.3 Hz
  rangefinder_sampling_ratio = 1.,        -- 测距数据采样比例，1. 表示全部使用
  odometry_sampling_ratio = 1.,           -- 里程计数据采样比例，1. 表示全部使用
  fixed_frame_pose_sampling_ratio = 1.,   -- 固定坐标系位姿采样比例，1. 表示全部使用
  imu_sampling_ratio = 1.,                -- IMU 数据采样比例，1. 表示全部使用
  landmarks_sampling_ratio = 1.,          -- 路标数据采样比例，1. 表示全部使用
}

MAP_BUILDER.use_trajectory_builder_2d = true              -- 启用 2D 轨迹构建器，运行 2D SLAM
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10     -- 累积 10 份测距数据后进行一次扫描匹配

return options                                            -- 返回配置表给 Cartographer