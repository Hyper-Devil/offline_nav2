# Offline Nav2 (rosbag playback, no Gazebo)

## 机器人平台

- 履带式车辆，车体 1023 × 778 mm，接地长度 560 mm，履带宽 150 mm
- 整备重量 130 kg，2 × 650 W 无刷伺服电机，克里斯蒂悬挂
- 最高速度 1.5 m/s，最大爬坡度 20°
- 导航限幅：前进 1.5 m/s，后退 1.0 m/s，线加速度/减速度 ±1.5 m/s²，最大角速度 1.5 rad/s
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
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | 局部代价地图障碍源，`frame_id=body` |
| `/similarity_costmap` | `nav_msgs/OccupancyGrid` | 全局代价地图输入，`frame_id=map` |

## 全局代价地图（/similarity_costmap）

`global_costmap.static_layer` 订阅 `/similarity_costmap`：

- launch 文件本身不约束 global costmap 分辨率；分辨率由参数文件配置项决定
- 当前 `global_costmap.global_costmap.ros__parameters.resolution = 0.10`（`config/nav2_params_offline.yaml`）
- 建议发布端 `/similarity_costmap` 的分辨率与上面保持一致，避免重采样带来的边界锯齿或路径贴边误差

- 类型：`nav_msgs/OccupancyGrid`，`header.frame_id = map`
- QoS：**Volatile**（`map_subscribe_transient_local: False`），兼容 ros_bridge 转发
- 时间戳：跟随仿真时间 `/clock`
- 数据约定：`-1` 未知，`0` 自由，`100` 占据
- 发布方需**持续以一定频率重发**（建议 ≥ 1 Hz），因为去掉 transient local 后 Nav2 启动时可能错过第一帧

## 局部代价地图（/cloud_registered_body）

`local_costmap.voxel_layer` 订阅 `/cloud_registered_body`：

- `clearing: False`：积累点云不做射线清除，避免 ray-tracing 把障碍物清掉
- `marking: True`：正常标记障碍物
- rolling window 滚动时自动清除移出视野的格子，不会产生鬼影
- 局部地图范围：10 × 10 m，分辨率 0.05 m

## MPPI 控制器关键参数

| 参数 | 值 | 说明 |
|---|---|---|
| `motion_model` | `DiffDrive` | 履带车等效差速驱动 |
| `consider_footprint` | `true` | 使用完整车体轮廓做代价评估 |
| `vx_max / vx_min` | 1.5 / -1.0 m/s | 导航限速（硬件上限 1.5 m/s） |
| `ax_max / ax_min` | 1.5 / -1.5 m/s² | 线加速度限制 |
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
- **Sensors 组**：`/cloud_registered_body` 点云（按 Z 高度着色）
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

# 确认 cloud_registered_body 订阅者
ros2 topic info /cloud_registered_body

# 若 local_costmap 仍全 0，检查点云 frame_id 是否为 body
ros2 topic echo /cloud_registered_body --once
```

---

## 调试记录（ros1_bridge + 仿真时间场景）

以下问题在 ROS1 容器运行 FastLIO + rosbag，本容器通过 ros1_bridge 接收数据的场景中被发现并修复。

### 问题 1：local_costmap 激活失败（TF 时序竞争）

**现象**

```
[local_costmap]: Timed out waiting for transform from base_link to odom,
    tf error: Invalid frame ID "odom" - frame does not exist
[local_costmap]: Timed out waiting for transform from base_link to odom,
    tf error: Lookup would require extrapolation at time T, but only time T+0.08s is in the buffer
[local_costmap]: Failed to activate local_costmap ... did not become available before timeout
[lifecycle_manager]: Failed to bring up all requested nodes. Aborting bringup.
```

**根本原因**

Nav2 节点刚启动时，自身 TF listener 尚未接收到任何 `/tf` 消息，lifecycle_manager 就触发激活检查，必然失败。即使 bag 和 FastLIO 已在运行，新启动的节点 buffer 为空。

第二条 extrapolation 错误是 ros1_bridge 的固有偏差：bag 中 `/clock` 消息比 `/tf` 消息早约一帧（75~100 ms），导致 ROS2 侧仿真时钟比 TF 时间戳略早，nav2 以"当前仿真时间"查询 TF 时找不到对应历史数据。

是下面这个时序竞争导致的，任何配置都解决不了：

nav2 节点在 TimerAction(5s) 后才启动 → TF listener 才开始订阅
DDS 发现 + 订阅建立需要 1-2 秒 → TF listener 真正开始收数据时已经是 t=7s
lifecycle_manager 立刻进入 activate 阶段，检查 TF
此时 TF buffer 里第一帧时间戳 = sim_time + 3ms（ros1_bridge 的固有偏序）
nav2 请求 sim_time T，buffer 最早数据是 T+3ms
"extrapolation into the past" → nav2 对这类错误有单独的立即退出路径，initial_transform_timeout: 60s 完全无效

是 ros1_bridge 的固有问题。 同一包 /clock 先于 /tf 到达 ROS2 端约 3ms，导致 sim_time 始终比 TF buffer 里最早的帧略早。原生 ROS2 不存在此问题。

**修复**：`launch/offline_nav2.launch.py` 中对 bringup 和 rviz 均加 5 秒 `TimerAction` 延迟，让节点的 TF listener 完成订阅并积累数据后，lifecycle_manager 再触发激活。

```python
from launch.actions import TimerAction
ld.add_action(TimerAction(period=5.0, actions=[bringup_cmd]))
ld.add_action(TimerAction(period=5.0, actions=[rviz_cmd]))
```

尝试记录（已回滚）：
根本修法曾尝试：先启动 nav2 节点（TF listener 立即开始积累数据），10 秒后再触发 lifecycle 激活。

结论：该策略在本项目当前环境中未达预期，已从 `launch/offline_nav2.launch.py` 回滚。
当前启动逻辑恢复为默认自动激活（autostart=true），local_costmap 问题仍存在，等待后续修复。


**注意**：`initial_transform_timeout` 参数（已加入 yaml）被 nav2 读取但对重试总时长影响有限，nav2 对 extrapolation 类型错误有单独的提前退出路径，不建议依赖该参数解决此问题。

---

### 问题 2：planner_server SIGSEGV 崩溃，拖垮整个 stack（含 local_costmap）

**现象**

```
[planner_server]: process has died [exit code -11]
[lifecycle_manager]: CRITICAL FAILURE: SERVER planner_server IS DOWN
[lifecycle_manager]: Failed to bring up all requested nodes. Aborting bringup.
```

local_costmap 成功激活后也随之被关闭。

**根本原因**

`nav2_params_offline_local_only.yaml` 仅配置了 controller 侧参数，缺少 `global_costmap` 和 `planner_server` 配置项。Nav2 bringup 仍会启动 planner_server，它使用默认配置（`global_costmap` 订阅 `/map` 的 static_layer）。本场景中 `/map` 话题不存在，NavfnPlanner 在空/未初始化的全局地图上尝试规划时发生段错误。lifecycle_manager 检测到心跳断开后杀掉所有节点。

**修复**：在 `config/nav2_params_offline_local_only.yaml` 中补充 `global_costmap`（rolling window，不使用 static_layer）和 `planner_server` 的完整配置：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      rolling_window: true
      width: 200
      height: 200
      global_frame: odom
      robot_base_frame: base_link
      transform_tolerance: 0.5
      initial_transform_timeout: 60.0
      plugins: ["inflation_layer"]   # 不依赖 /map
      ...

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      allow_unknown: true
```

---

### 问题 3：VoxelLayer sensor origin out of map bounds，raytrace 失效

**现象**

```
[local_costmap]: Sensor origin at (x, y, -4.43) is out of map bounds
    (-2.00) to (2.00). The costmap cannot raytrace for it.
```

障碍物只进不出，代价地图持续累积噪点。

**根本原因**

VoxelLayer 的 `origin_z` 是 odom 帧中 voxel grid 底部的**绝对 z 值**，不随机器人位置在 Z 方向滚动（local costmap 只在 XY 方向 rolling window）。

本车坐标系关系：
- body（LiDAR）随地形升降，在 odom 帧中 z 值持续变化
- 原配置 `origin_z: -2.0`，覆盖 -2.0m ~ +2.0m
- 下坡后 body z 达到 -4.43m，超出范围，sensor origin 超界

**z 范围计算**（本车参数）：
- body 预计 z 变化范围：±1.0m（最大地形高差），加安全裕量 0.5m → -1.5m ~ +1.5m
- 最低障碍（body 下方 0.88m 的 base_link 附近）：-1.5 - 0.88 ≈ **-2.4m** → 取 -2.5m
- 最高障碍（body 上方 2.0m）：+1.5 + 2.0 = **+3.5m**
- 总范围 6m → `z_voxels = 120`（120 × 0.05m = 6.0m）

**修复**（`config/nav2_params_offline_local_only.yaml`）：

```yaml
voxel_layer:
  origin_z: -2.5          # 原 -2.0
  z_voxels: 120            # 原 80
  max_obstacle_height: 3.5  # 原 2.0（VoxelLayer 级别）
  pointcloud_mark:
    max_obstacle_height: 3.5   # 原 2.0
    min_obstacle_height: -2.5  # 原 -0.75
  pointcloud_clear:
    max_obstacle_height: 3.5   # 原 2.0
    min_obstacle_height: -2.5  # 原 -2.0
```

---

### 待处理：坡地场景下地面点误标为障碍物

**问题描述**

当前 `min_obstacle_height` 使用 odom 帧绝对高度过滤地面点，在平地有效，坡地失效（地面绝对 z 随坡度变化）。

**正确方案**（暂未实现）

在 nav2 订阅之前，在 **body 帧**中做高度 passthrough 过滤：body 帧随车倾斜，无论坡度如何，地面相对 LiDAR 的高度基本恒定（z_body ≈ -0.88m）。

```
/cloud_registered_body → [body帧 passthrough 过滤] → /cloud_registered_body_filtered → nav2
```

过滤参数（body 帧，相对于 LiDAR）：
- `min_z_body: -1.1m`（保留 base_link 附近及以上的近地面障碍）
- `max_z_body: +2.0m`（过滤树冠等高处无效点）

实现后，`min/max_obstacle_height` 可设为宽松绝对范围（如 ±20m），地面过滤由前级完成。

---

### 清理残留进程

多次测试或异常退出后，nav2 子进程可能残留，导致新启动时 DDS 服务调用立即失败。

```bash
# 强制清理所有 nav2 相关进程
pkill -9 -f "nav2_controller|nav2_planner|nav2_smoother|nav2_bt_navigator|nav2_behaviors|nav2_route|nav2_waypoint|nav2_velocity|nav2_collision|opennav_docking|nav2_lifecycle"

# 重启 ROS2 daemon 清除僵尸 DDS 节点
ros2 daemon stop && ros2 daemon start

# 验证清理结果（应只剩 ros_bridge 等非 nav2 节点）
ros2 node list
```
