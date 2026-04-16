# Offline Nav2 (rosbag playback, no Gazebo)

## 机器人平台

- 履带式车辆，车体 1023 × 778 mm，接地长度 560 mm，履带宽 150 mm
- 整备重量 130 kg，2 × 650 W 无刷伺服电机，克里斯蒂悬挂
- 最高速度 1.5 m/s，最大爬坡度 20°
- 导航限速：前进 1.0 m/s，后退 0.5 m/s，最大角速度 1.5 rad/s
- 足迹（base_link 在几何中心）：`[[-0.512, -0.389], [-0.512, 0.389], [0.512, 0.389], [0.512, -0.389]]`
  - 若 base_link 不在几何中心，需调整 x 方向偏移量

## Files

- `launch/offline_nav2.launch.py`
- `config/nav2_params_offline.yaml`
- `rviz/offline_nav2.rviz`

## Required TF tree

FastLIO 维护完整 TF 链，Nav2 直接使用，无需 AMCL：

```
map → odom → camera_init → body → base_link
                                 → cam_left
                                 → camera_left
                                 → imu_link
```

- `map → odom`：外部定位节点（FastLIO + 全局定位）负责发布
- `odom → camera_init`：静态 identity TF 或由 FastLIO 提供
- `use_localization` 在 launch 里硬编码为 `False`，Nav2 不启动 AMCL

## Required topics

| 话题 | 类型 | 说明 |
|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | bag 播放时加 `--clock` |
| `/tf`, `/tf_static` | TF | FastLIO 提供 |
| `/odom` | `nav_msgs/Odometry` | FastLIO 里程计 |
| `/accumulated_map_points` | `sensor_msgs/PointCloud2` | 局部代价地图障碍源，`frame_id=body` |
| `/similarity_costmap` | `nav_msgs/OccupancyGrid` | 全局代价地图输入，`frame_id=map` |

## 全局代价地图（/similarity_costmap）

`global_costmap.static_layer` 订阅 `/similarity_costmap`：

- 类型：`nav_msgs/OccupancyGrid`，`header.frame_id = map`
- QoS：**Volatile**（`map_subscribe_transient_local: False`），兼容 ros_bridge 转发
- 时间戳：跟随仿真时间 `/clock`
- 数据约定：`-1` 未知，`0` 自由，`100` 占据
- 发布方需**持续以一定频率重发**（建议 ≥ 1 Hz），因为去掉 transient local 后 Nav2 启动时可能错过第一帧

## 局部代价地图（/accumulated_map_points）

`local_costmap.voxel_layer` 订阅 `/accumulated_map_points`：

- `clearing: False`：积累点云不做射线清除，避免 ray-tracing 把障碍物清掉
- `marking: True`：正常标记障碍物
- rolling window 滚动时自动清除移出视野的格子，不会产生鬼影
- 局部地图范围：10 × 10 m，分辨率 0.05 m

## MPPI 控制器关键参数

| 参数 | 值 | 说明 |
|---|---|---|
| `motion_model` | `DiffDrive` | 履带车等效差速驱动 |
| `consider_footprint` | `true` | 使用完整车体轮廓做代价评估 |
| `vx_max / vx_min` | 1.0 / -0.5 m/s | 导航限速（硬件上限 1.5 m/s） |
| `wz_max` | 1.5 rad/s | 保守转向（130 kg 惯量） |
| `prune_distance` | 2.5 m | 大于车体长度 |

## Run

Terminal 1（FastLIO + 自定义地图节点，需 `use_sim_time:=true`）：
```bash
# 确保这些节点都开启了仿真时间
```

Terminal 2（Nav2 + RViz）：
```bash
ros2 launch /home/whd/offline_nav2/launch/offline_nav2.launch.py
```

Terminal 3（bag 播放）：
```bash
ros2 bag play <your_bag_path> --clock --loop
```

## RViz 使用

- 打开 Nav2 面板，用 `2D Goal Pose` 发送目标点
- **Sensors 组**：`/accumulated_map_points` 点云（按 Z 高度着色）
- **Global Planner 组**：原始相似度地图 + 全局代价地图 + 全局路径
- **Controller 组**：局部代价地图 + 局部路径 + MPPI 采样轨迹（`/trajectories`）

## 注意事项

- `use_localization: False`（硬编码）：`map → odom` TF 断开时规划器立即报错
- `min_y_velocity_threshold: 0.001`：履带车横向速度为 0，原始值 0.5 是遗留 Bug
- TF Marker Scale 0.5（0.5 m 轴长），适合当前车体尺寸
- `base_footprint` 未使用，所有节点统一使用 `base_link`

## 自检命令

```bash
# TF 链完整性
ros2 run tf2_tools view_frames

# 全局地图是否收到
ros2 topic echo /global_costmap/costmap --once --no-arr

# 局部地图是否有障碍物标记
ros2 topic hz /local_costmap/voxel_marked_cloud

# 确认 accumulated_map_points 订阅者
ros2 topic info /accumulated_map_points
```
