# Bunker Nav2 实车导航

当前可用基线：ROS1 Noetic 传感器 / FAST-LIO-SAM，经 `bridge_env` 桥接到 ROS2 Jazzy Nav2；Bunker 驱动运行在 `ros2_ugv`。目录名保留 `offline_nav2`，当前默认使用实车时间，不是 rosbag 回放模式。

## 机器人平台

- 履带式车辆，车体 1023 × 778 mm，接地长度 560 mm，履带宽 150 mm。
- 整备重量 130 kg，2 × 650 W 无刷伺服电机，克里斯蒂悬挂。
- 最高速度 1.5 m/s，最大爬坡度 20°；遥控最高速度 1.5 m/s 已由操作员确认。
- 足迹（base_link 在几何中心）：`[[-0.512, -0.389], [-0.512, 0.389], [0.512, 0.389], [0.512, -0.389]]`，当前 footprint_padding 为 0.02 m。若 base_link 不在几何中心，需调整 x 方向偏移量。

原文的导航限幅设计记录为：前进 / 后退 1.5 m/s，线加速度 / 减速度 ±2.0 m/s²，最大角速度 1.5 rad/s。这是早期设计值；当前实际配置见下方参数表。

## 文件

- `launch/nav2_no_collision.launch.py`：已实测可用的最小启动链路。
- `config/nav2_params_offline_livox_50hz.yaml`：当前唯一维护的导航参数。
- `rviz/offline_nav2.rviz`：RViz 配置。
- `scripts/record_nav_test.sh`：录制导航测试数据。
- `old_config/`：旧启动文件、旧 YAML 与历史备份，不用于当前启动。

本目录不是带 package.xml 的独立 ROS2 包，使用启动文件绝对路径，无需为修改 YAML 或 launch 编译。

## 执行链路与参数

`controller_server / behavior_server → /cmd_vel_nav → velocity_smoother → /cmd_vel → bunker`

速度平滑器的输出 `cmd_vel_smoothed` 在 launch 内重映射到 `cmd_vel`，因此当前不存在独立的 `/cmd_vel_smoothed` 输出话题。

启动节点：controller_server、planner_server、behavior_server、bt_navigator、velocity_smoother、lifecycle_manager_navigation，以及可选 RViz。局部和全局 costmap 分别由 controller 和 planner 管理。未启动 collision_monitor、route_server、waypoint_follower、docking_server、路径 smoother_server。

| 参数 | 当前值 |
|---|---|
| controller / velocity_smoother 频率 | 50 Hz / 50 Hz |
| MPPI motion_model | DiffDrive |
| MPPI model_dt / time_steps / batch_size | 0.02 s / 100 / 2000（预测 2 s） |
| 最大前进 / 倒车速度 | 1.5 / −0.3 m/s |
| 线加速度 / 制动减速度 | 3.0 / −2.0 m/s² |
| 最大角速度 | 1.0 rad/s |
| 角加速度 / 减速度（平滑器） | 1.5 / −1.5 rad/s² |
| controller 速度反馈 | /bunker_odom |
| velocity_smoother.feedback | OPEN_LOOP |
| velocity_timeout | 0.3 s |
| CostCritic.consider_footprint | false |
| 目标位置 / 航向容差 | 0.35 m / 0.35 rad |

MPPI 与平滑器的线速度、加减速限制保持一致。OPEN_LOOP 用上一条输出作为限加速度的起点；controller 仍读取真实底盘速度反馈。参数是指令 / 模型边界，不代表底盘实测一定达到该加速度。

## 定位与 costmap

外部 FAST-LIO-SAM 提供定位 / TF；该 launch 不启动 AMCL、map_server 或 SLAM。Nav2 必须能查询 `map → base_link` 和 `odom → base_link`，中间帧与静态外参由外部项目维护；不能仅根据话题名假定 TF 完整。

| 输入 | 用途 |
|---|---|
| /tf、/tf_static | 位姿变换 |
| /bunker_odom | controller 底盘速度反馈 |
| /Odometry | FAST-LIO-SAM 里程计，诊断记录 |
| /livox_costmap | local / global StaticLayer 的 OccupancyGrid 输入 |

BT 配置仍保留 `odom_topic: /odom`，这是本次整理前的已验证值；它不改变 controller 使用 `/bunker_odom` 的设置，需在后续功能扩展时检查该话题是否提供。

两层 costmap 均使用 StaticLayer + InflationLayer，没有 VoxelLayer。local 为 odom 帧、20 × 20 m rolling window，更新 / 发布 5 / 2 Hz；global 为 map 帧，更新 / 发布 1 / 1 Hz，尺寸和原点来自输入地图。分辨率 0.1 m，膨胀半径 0.6 m，cost_scaling_factor 3.0。StaticLayer 使用 volatile 订阅，地图生成器需持续发布。

`generate_livox_costmap.py` 在 ros1_ugv 中运行。名称保留 Livox，但实际点云源由 ROS 参数 `~cloud_topic` 决定，脚本默认 /livox/points；速腾 SLAM 点云存在不等于此地图生成器已收到输入。默认生成 odom 帧、20 m 边长滚动地图，以机器人位置为中心，每帧重建；高度过滤基于 odom 中相对于机器人 z 的 0.2–2.0 m，至少 2 点 / 格标记障碍。默认在传感器 15 m 范围内将无障碍点的格子置自由，范围外置未知；此逻辑并不等于完整的遮挡 / 射线可见性判断。

collision_monitor 已从启动链路移除。costmap、MPPI CostCritic 与行为服务器自身的碰撞检查仍存在；它们不能替代被移除的独立碰撞监视器。

## 启动

每个容器项目在新终端执行。bridge_env 自动启动，无需额外启动命令。

### 时间同步与 CAN（宿主机）

```bash
sudo /home/ugv/start_ptp.sh
ps aux | grep -E '[p]tp4l|[p]hc2sys'
bash /home/ugv/colcon_ws/src/ugv_sdk/scripts/bringup_can2usb_500k.bash
ip -details link show can0
```

### ROS1 master

```bash
docker start ros1_ugv
docker exec -it ros1_ugv bash
source /opt/ros/noetic/setup.bash
roscore
```

### 传感器

```bash
docker exec -it ros1_ugv bash
source /opt/ros/noetic/setup.bash
source /home/ugv/catkin_ws/devel/setup.bash
roslaunch ugv_bringup start_all.launch
```

### FAST-LIO-SAM

```bash
docker exec -it ros1_ugv bash
source /opt/ros/noetic/setup.bash
source /home/ugv/catkin_ws/devel/setup.bash
roslaunch fast_lio_sam mapping_rs.launch use_sim_time:=false rviz:=false
```

### 点云地图生成器

```bash
docker exec -it ros1_ugv bash
source /opt/ros/noetic/setup.bash
source /home/ugv/catkin_ws/devel/setup.bash
python3 /home/ugv/emcupy_ws/src/elevation_mapping_cupy/elevation_mapping_cupy/script/generate_livox_costmap.py
```

### Bunker 驱动

```bash
docker start ros2_ugv
docker exec -it ros2_ugv bash
source /opt/ros/jazzy/setup.bash
source /home/ugv/colcon_ws/install/setup.bash
ros2 launch bunker_base bunker_base.launch.py use_sim_time:=false
```

### Nav2 + RViz

先在原终端 Ctrl+C 停止旧 Nav2，再执行：

```bash
docker exec -it ros2_ugv bash
source /opt/ros/jazzy/setup.bash
source /home/ugv/colcon_ws/install/setup.bash
ros2 launch /home/ugv/colcon_ws/src/offline_nav2/launch/nav2_no_collision.launch.py \
  params_file:=/home/ugv/colcon_ws/src/offline_nav2/config/nav2_params_offline_livox_50hz.yaml \
  use_sim_time:=false \
  use_rviz:=true
```

启动命令本身不发布目标。RViz 中由操作员使用 Nav2 面板 / 2D Goal Pose 发送任务。修改参数后重启 Nav2 生效。回放数据时需统一所有相关节点的仿真时间并提供 /clock；当前实车模式不需要 /clock。

## 检查与记录

在 ROS2 新终端：

```bash
docker exec -it ros2_ugv bash
source /opt/ros/jazzy/setup.bash
source /home/ugv/colcon_ws/install/setup.bash
ros2 node list
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
ros2 topic info -v /cmd_vel
ros2 run tf2_ros tf2_echo map base_link
```

以下命令分别执行，hz / tf2_echo 持续运行，Ctrl+C 结束：

```bash
ros2 topic hz /Odometry
ros2 topic hz /bunker_odom
ros2 topic hz /livox_costmap
ros2 topic echo /global_costmap/costmap --once --no-arr
ros2 param get /controller_server odom_topic
ros2 param get /velocity_smoother feedback
```

无任务时 /cmd_vel_nav 和 /cmd_vel 可能无消息，不能据此判定频率故障。执行导航时才比较原始指令、最终指令与底盘反馈。

记录测试（新终端）：

```bash
docker exec -it ros2_ugv bash
source /opt/ros/jazzy/setup.bash
source /home/ugv/colcon_ws/install/setup.bash
bash /home/ugv/colcon_ws/src/offline_nav2/scripts/record_nav_test.sh 180
```

脚本只记录，不发布目标或速度；出现 MCAP recording is active 后手动发布目标。输出默认位于脚本同目录下的 `scripts/nav_test_时间戳/`，不受运行命令时所在目录影响；第二个参数可指定其他输出根目录。已按当前链路记录 /cmd_vel_nav、/cmd_vel、/bunker_odom、/Odometry、TF、路径和地图。

## 调试记录

### 2026-10-03：车辆蠕动、一走一停，关闭 collision_monitor 后恢复正常

此前车辆出现很慢的移动、一走一停，能听到底盘执行声但未正常行走。早期记录中 controller 使用 FAST-LIO /Odometry，速度估计会影响 MPPI 输出；随后将 controller 反馈切换至 /bunker_odom，速度平滑器调整为 OPEN_LOOP，并保留加减速约束。

在此基线上移除 collision_monitor，平滑器输出直接重映射至 /cmd_vel 后，操作员实测反馈“非常好，正常了”。这记录了关闭该环节后的实际改善；具体触发限速的点云、多边形、TF 或 source_timeout 条件尚未通过同一组数据的对照实验确认。不同早期日志中碰撞监视器未必是相同的故障来源，不能据此把全部历史震荡归于它。

仅清空 collision_monitor 的 polygons / observation_sources 曾使启动卡在 `Waiting for service collision_monitor/get_state...`。因此最终方案是从 launch 的节点与生命周期名单中完全移除，同时修改速度平滑器输出重映射。

恢复正常后按操作员要求同步调整 MPPI 与平滑器：最高前进速度 1.5 m/s、加速度 3.0 m/s²、制动减速度 −2.0 m/s²。角速度与倒车限制保持原值。

### costmap 越界与 TF 启动问题

遇到 `Robot is out of bounds of the costmap!` 时检查 /livox_costmap 的 frame、原点、尺寸与机器人当前坐标；地图生成器未运行、地图与定位原点不一致或机器人超出地图均需排查，不能直接归因于 MPPI。

TF 超时需检查外部 TF 链、时间同步及数据时间戳。旧 README 将若干时序故障认定为 ros1_bridge 固有缺陷，并描述了与代码不一致的 TimerAction 修复；当前启动文件没有该延迟逻辑，不再保留这些未经核实的结论。当前 costmap 不使用 VoxelLayer，旧版 z_voxels / voxel_marked_cloud 调试说明不适用。

### 2026-10-09：配置整理

将已验证启动文件从 config 移入 launch；只保留当前基线 YAML，移除未启动节点的参数及已安装 Jazzy MPPI 不生效的 open_loop 字段（平滑器 OPEN_LOOP 保留）。旧 launch、五份旧 YAML 和两份历史备份移到 `old_config/`，保留未提交改动，可恢复。同步更新 /home/ugv/启动nav2调试.md。监控脚本默认将测试记录保存到脚本所在目录。
