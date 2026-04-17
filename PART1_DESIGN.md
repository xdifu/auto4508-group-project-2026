# Part 1 — Simulation (Individual) 实施设计文档

> 本文档描述仓库中 Part 1 实现的最终架构与实现边界。
> 对应 `AUT4508 - Group Project 2026 - V1_0.pdf` 中 `Part 1 – Simulation (due in week 6) - Individual` 的全部 8 项任务。

---

## 1. 最终架构

### 1.1 三包结构

本实现采用三包结构，而不是早期草案中的单 `ament_python` 包。

```text
project/src/
├── p3at_description/   # ament_cmake: xacro, meshes, rviz
├── p3at_bringup/       # ament_python: world, launch, config, mission/path 工具
└── p3at_nav_plugins/   # ament_cmake: 自定义 Nav2 local controller plugin (C++)
```

### 1.2 各包职责

**`p3at_description`**
- 提供 `urdf/pioneer.urdf.xacro`
- 提供 Gazebo / RViz 共用的 meshes
- 提供 RViz 配置

**`p3at_bringup`**
- 提供 `worlds/james_oval.sdf`
- 提供 `sim.launch.py` 与 `nav.launch.py`
- 提供 bridge / EKF / slam_toolbox / Nav2 / waypoints 配置
- 提供 Python 工具节点:
  - `mission_manager`
  - `path_recorder`

**`p3at_nav_plugins`**
- 提供 `p3at_nav_plugins::P3ATController`
- 该插件实现 `nav2_core::Controller`
- 用于路径跟踪、接近减速、终点姿态对齐与局部碰撞预测

### 1.3 运行分层

```text
Gazebo Harmonic
  └─ James Oval world + Pioneer 3-AT robot
      ├─ ros_gz_bridge
      ├─ robot_state_publisher
      ├─ clock_sanitizer / odom_sanitizer / joint_state_sanitizer
      ├─ slam_toolbox
      ├─ Nav2 (planner + costmaps + BT + waypoint follower)
      ├─ mission_manager
      └─ path_recorder
```

### 1.4 TF 职责

TF 职责严格分离，不允许重复发布:

```text
map -> odom               : slam_toolbox
odom -> base_link         : odom_sanitizer
base_link -> sensors/...  : robot_state_publisher
```

`DiffDrive` 插件只发布原始 `/odom`，不发布 `odom -> base_link` TF。

---

## 2. 与 8 项任务的一一对应

### Task 1 — 完成 Pioneer URDF + Teleop 驱动

当前实现不再直接编辑原始 `project/Resources/robots/pioneer.urdf`，而是基于其结构重建为 `xacro`。

实现要点:
- 四轮通过 `xacro:macro` 生成，补全 `front_right / back_left / back_right`
- 所有轮子 collision 统一只保留 `cylinder`
- `wheel_separation` 默认从 `0.394` 起步，匹配实际 P3AT 轮距
- `odom_publish_frequency` 实际为 `20Hz`（model.sdf DiffDrive 插件设定）
- `JointStatePublisher` 使用 Gazebo Harmonic 插件命名并放入 `<gazebo>` 标签内
- Teleop 用于验证模型与动力学是否正确，不再依赖 Gazebo GUI 内置 Teleop 作为正式 ROS2 控制链

### Task 2 — James Oval 世界

世界文件为 `p3at_bringup/worlds/james_oval.sdf`。

实现目标:
- 草地平面
- 四周围栏 / 路缘
- 北侧建筑立面
- 边缘灯杆 / 树
- cricket nets 框架
- 若干 cone / bucket 障碍物

设计原则:
- 世界不仅要“像草坪”，还必须给 `slam_toolbox` 足够的几何特征
- 所有 cones / buckets 都是静态障碍物
- 障碍物 collision 高度必须穿过 lidar 扫描平面

### Task 3 — 传感器添加且非理想化

机器人描述中加入:

**Lidar**
- `270° FOV`
- `810 samples`
- `10Hz`
- `max range 20m`
- Gaussian noise

**Camera**
- RGB camera
- `/camera/image` + `/camera/camera_info`
- 正确的 `cam_optical_link`

**IMU**
- 挂在 `base_link`
- 有角速度与线加速度噪声
- 通过 bridge 发布到 `/imu`，供后续 Part 2/3 传感器融合扩展使用

### Task 4 — ROS2 Stack 自主驱动架构

正式 ROS2 栈由两条 launch 组成:

**`sim.launch.py`**
- 设置 `GZ_SIM_RESOURCE_PATH`
- 全局 `use_sim_time=true`
- 启动 Gazebo
- 生成机器人
- 启动 `robot_state_publisher`
- 启动 `ros_gz_bridge`
- 启动 `clock_sanitizer / odom_sanitizer / joint_state_sanitizer`
- 可选启动 RViz
- 可选启动 `teleop_twist_keyboard`（`teleop:=true`）

**`nav.launch.py`**
- 全局 `use_sim_time=true`
- 启动 `slam_toolbox`
- 启动 Nav2
- 启动 `mission_manager`
- 启动 `path_recorder`
- 可选启动 rosbag2 录制

### Task 5 — 自定义 local controller

自定义控制器不是独立 Python 节点，而是 C++ Nav2 plugin:

`p3at_nav_plugins::P3ATController`

它实现三段行为:
1. 路径跟踪
2. 接近目标时减速
3. 到达目标位置后原地朝向对齐

额外能力:
- 将全局路径从 `map` 坐标系变换到局部控制所用坐标系后再跟踪
- 从 local costmap 前向预测碰撞
- 若局部不可行，则抛出失败，交给 BT 执行 recovery / replan

### Task 6 — 多航点路径生成

任务航点来自 `config/waypoints.yaml`。

关键定义:
- YAML 只定义“任务输入航点”
- 真正的可执行路径由 Nav2 planner 在 costmap 上逐段生成
- `mission_manager` 使用 `FollowWaypoints` action client 顺序执行航点
- `/mission_waypoints` 用于 RViz 显示任务航点
- `/plan` 是实际规划路径

### Task 7 — 避开未知静态障碍

主方案不再是纯扇区法。

当前实现采用:
- lidar → local/global costmap
- planner 生成新路径
- custom controller 跟踪路径并做局部碰撞预测
- behavior tree 执行 `clear costmap / spin / backup` 等 recovery

因此，未知静态障碍的处理链是:

```text
scan -> costmap update -> planner replan -> controller follow -> recovery if blocked
```

纯扇区反应式避障只保留为“设计演化历史 / fallback 思路”，不再是正式交付主链路。

### Task 8 — 建图与路径记录

**建图**
- `slam_toolbox`
- online async mapping
- `map -> odom`
- `max_laser_range = 18.0m`

**路径记录**
- `path_recorder` 默认订阅 `/odometry/filtered`
- 发布 `/driven_path`
- 导出 CSV

**完整数据记录**
- rosbag2 录制:
  - `/tf`
  - `/tf_static`
  - `/clock`
  - `/odom`
  - `/odometry/filtered`
  - `/scan`
  - `/imu`
  - `/map`
  - `/cmd_vel`
  - `/camera/image`
  - `/camera/camera_info`
  - `/plan`

---

## 3. 关键配置决策

### 3.1 Bridge Topic 清单

正式桥接 topic:

| Topic | 方向 |
|------|------|
| `/clock` | GZ → ROS2 |
| `/cmd_vel` | ROS2 → GZ |
| `/scan` | GZ → ROS2 |
| `/imu` | GZ → ROS2 |
| `/odom` | GZ → ROS2 |
| `/joint_states` | GZ → ROS2 |
| `/camera/image` | GZ → ROS2 |
| `/camera/camera_info` | GZ → ROS2 |

### 3.2 Nav2 关键选择

本实现锁定以下选择:
- Planner: `NavFn`
- Controller: `p3at_nav_plugins::P3ATController`
- Waypoint execution: `FollowWaypoints`
- Costmaps: `obstacle_layer + inflation_layer`
- Recovery: `clear costmap + spin + backup`

### 3.3 use_sim_time

`use_sim_time: true` 必须全局一致。

适用对象:
- `robot_state_publisher`
- `clock_sanitizer` / `odom_sanitizer` / `joint_state_sanitizer`
- `slam_toolbox`
- Nav2 全部节点
- `mission_manager`
- `path_recorder`
- RViz

### 3.4 Gazebo 资源路径

由于 robot description 与 world 都需要解析包内资源，`sim.launch.py` 中显式设置:

- `GZ_SIM_RESOURCE_PATH`

这样 Gazebo 才能稳定找到:
- `install/share` 下的包资源根目录
- `p3at_description/meshes`
- `p3at_bringup/worlds`

---

## 4. 验证计划

### 4.1 模型层
- `xacro` 结构完整
- Gazebo 可加载机器人
- 四轮朝向正确
- Teleop 前进 / 后退 / 左转 / 右转 / 原地转正常

### 4.2 传感器层
- `/scan /imu /odom /joint_states /camera/image /camera/camera_info /clock` 均可桥接
- RViz 可显示 RobotModel、LaserScan、Camera、TF、Map

### 4.3 TF 层
- 无 TF 冲突
- `odom_sanitizer` 是 `odom -> base_link` 唯一发布者
- `slam_toolbox` 是 `map -> odom` 唯一发布者

### 4.4 导航层
- 单航点可到达并在终点对齐朝向
- `FollowWaypoints` 可逐个航点停靠
- 当 cones / buckets 初始未被观察到时，机器人能在感知到它们后重规划或 recovery

### 4.5 记录层
- `/driven_path` 与实际路径一致
- rosbag2 可正常录制
- `slam_toolbox` 地图可保存
- CSV 可导出

---

## 5. 参数与待调项

以下参数为实际运行时的值（以 `nav2.yaml` / `model.sdf` 为准）:

| 参数 | 运行值 | 说明 |
|------|-------|------|
| `wheel_separation` | `0.394` | 匹配实际 P3AT 轮距 |
| `odom_publish_frequency` | `20Hz` | model.sdf DiffDrive 实际频率 |
| `slam_toolbox.max_laser_range` | `18.0m` | 略低于激光物理量程，用于抑制边缘噪声 |
| `lookahead_distance` | `0.8m` | controller 路径跟踪前视距离 (nav2.yaml) |
| `slowdown_distance` | `1.2m` | 接近目标线性减速起始距离 (nav2.yaml) |
| `max_linear_speed` | `0.45 m/s` | 最大直线速度 (nav2.yaml) |
| `collision_lookahead_time` | `1.0s` | 前向碰撞预测时间窗 (nav2.yaml) |
| `goal_dist_tolerance` | `0.25m` | 终点位置容差 |
| `goal_yaw_tolerance` | `0.087rad` | 约 5° |

---

## 6. 实现边界与假设

- 默认目标机器已安装 `ROS2 Jazzy + Gazebo Harmonic`
- 本仓库不负责系统级安装脚本或容器化
- Camera 在 Part 1 中已建模和桥接，但不作为控制主闭环
- 当前实现优先完成 Part 1，不提前扩展到 Part 2/3 的视觉识别和手柄接管

---

## 7. 最终结论

本实现从“单 Python 包 + 纯 topic 链 + 扇区避障”的早期方案，升级为:

```text
三包 ROS2 工程
+ Xacro 机器人描述
+ James Oval 结构化世界
+ clock/odom/joint_state sanitizer
+ slam_toolbox
+ Nav2 planner/costmaps/BT
+ 自定义 C++ local controller
+ FollowWaypoints 任务调度
+ rosbag2 + CSV 双记录
```

这套方案覆盖了 Part 1 的全部 8 项任务，并且每个设计决策都能直接映射到仓库中的实际实现。
