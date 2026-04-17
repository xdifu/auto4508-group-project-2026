# Part 1 Demo 满分指南

> 全程只开 1 个 Gazebo 渲染窗口，不开 RViz，保证 WSL2 流畅运行。
> 3 个终端，约 5-8 分钟。

---

## 前置准备（仅首次 / 改了 C++ 才需要）

```bash
cd /home/god/auto4508/project
colcon build --symlink-install
```

修改 Python / YAML / launch / SDF **不需要**重新编译（`--symlink-install` 自动生效）。
只有改 C++（`p3at_controller.cpp`）才需要重新 `colcon build`。

---

## 展示前清理（每次展示前执行一次）

```bash
cd /home/god/auto4508/project

# 1. 杀残留进程
pkill -f "gz sim" ; pkill -f "ros_gz_bridge" ; pkill -f "robot_state_publisher" ; \
pkill -f "sanitizer" ; pkill -f "slam_toolbox" ; pkill -f "planner_server" ; \
pkill -f "controller_server" ; pkill -f "behavior_server" ; pkill -f "bt_navigator" ; \
pkill -f "waypoint_follower" ; pkill -f "lifecycle_manager" ; \
pkill -f "mission_manager" ; pkill -f "path_recorder" 2>/dev/null

# 2. 删除旧输出文件（防止 rosbag 报目录已存在）
rm -rf part1_demo_run part1_driven_path.csv part1_slam_map.*
```

---

## 完整展示流程

### 步骤 1 — Terminal 1：启动仿真 (Tasks 1, 2)

```bash
cd /home/god/auto4508/project
source install/setup.bash
ros2 launch p3at_bringup sim.launch.py
```

**等 Gazebo 窗口出现后，向老师展示场景（鼠标操作：左键旋转 | 中键平移 | 滚轮缩放）：**

| 向老师指出 | 对应 Task |
|---|---|
| **Pioneer 机器人**：粉色底盘 + 黄色顶板 + 4 个黑色轮子 + 青色旗杆 | Task 1 — URDF 模型 |
| **椭圆形草地**（23m×30m，1:6 比例模拟真实 James Oval） | Task 2 — James Oval |
| **椭圆边界墙**（32段红褐色围墙，模拟 laterite 步行道） | Task 2 |
| **北楼**（砂岩色大建筑，模拟 UWA 建筑） | Task 2 |
| **板球场**（中央棕色长方形） | Task 2 |
| **3 面板球网**（东南区域） | Task 2 |
| **4 根灯杆**（场内四象限） | Task 2 |
| **5 个橙色圆锥 (cones)** — 必须指出 | Task 2 — 障碍物 |
| **3 个蓝色桶 (buckets)** — 必须指出 | Task 2 — 障碍物 |

---

### 步骤 2 — Terminal 2：键盘遥控 (Task 1)

```bash
cd /home/god/auto4508/project
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

按键操作（在此终端窗口内按键）：

| 按键 | 效果 |
|---|---|
| `i` | 前进 |
| `,` | 后退 |
| `j` | 左转 |
| `l` | 右转 |
| `k` | 停止 |

在 Gazebo 中看到机器人移动 → **按 `k` 停车 → `Ctrl+C` 退出 teleop**。

> **向老师说明**："Pioneer 3-AT URDF 模型完成，四轮差速驱动，通过 ROS2 teleop_twist_keyboard 键盘遥控。"

> **⚠️ 重要：teleop 会使机器人偏离原点。步骤 3 验证传感器后，必须重启仿真让机器人回到原点，否则导航航点会超出 costmap 范围导致规划失败。**

---

### 步骤 3 — Terminal 2：传感器验证 (Task 3)

**3.1 Lidar 数据（SICK TIM781 仿真）：**
```bash
ros2 topic echo /scan --once 2>/dev/null | head -15
```
**预期输出（向老师指出关键字段）：**
```yaml
header:
  stamp: ...
  frame_id: laser_frame
angle_min: -2.356...          # ← 270° FOV（±135°）
angle_max: 2.356...           # ← 270° FOV
angle_increment: 0.00581...   # ← 810 samples（270°/810=0.33°/ray）
range_min: 0.05
range_max: 20.0
ranges:
- 19.87...                    # ← 带高斯噪声，不是整数
- 20.02...                    # ← 噪声使值超过 max 也正常
```

**3.2 传感器频率验证（每个看到数字就 Ctrl+C）：**
```bash
ros2 topic hz /scan
```
预期：`average rate: 10.0xx Hz` → Lidar 10Hz ✅ — Ctrl+C

```bash
ros2 topic hz /imu
```
预期：`average rate: 20.0xx Hz` → IMU 20Hz ✅ — Ctrl+C

```bash
ros2 topic hz /camera/image
```
预期：`average rate: 2.0xx Hz` → Camera 2Hz ✅ — Ctrl+C

> **向老师说明**："三种传感器全部配置了高斯噪声——Lidar 距离噪声 stddev=0.02m，IMU 角速度噪声 0.001 rad/s、加速度噪声 0.05 m/s²，Camera 像素噪声 0.0075。模拟真实 SICK TIM781、Phidget Spatial 3/3/3、OAK-D V2 的测量不确定性。"

---

### 步骤 3.5 — Terminal 1：重启仿真（重置机器人到原点）

> **⚠️ 必做！** Teleop 已经把机器人从原点 (0,0) 移走。导航航点定义在原点附近，必须重启仿真让机器人回到 (0,0)。

**Terminal 1：Ctrl+C 关闭当前 sim.launch.py，然后重新启动：**

```bash
ros2 launch p3at_bringup sim.launch.py
```

**等待 Gazebo 窗口重新出现，确认机器人回到场地中央原点。**

> **向老师说明**："Teleop 演示完毕。重启仿真重置机器人位置，准备自主导航演示。两条 launch 完全解耦：sim 负责仿真层，nav 负责决策层，可以独立重启。"

---

### 步骤 4 — Terminal 2：启动自主导航 (Tasks 4, 5, 6, 7, 8)

```bash
cd /home/god/auto4508/project
source install/setup.bash
ros2 launch p3at_bringup nav.launch.py record:=true
```

> ⚠️ **不加 `rviz:=true`**，只用 Gazebo 观察，省 GPU。

**Terminal 2 关键日志时间线（按顺序出现）：**

```
[lifecycle_manager]: Configuring controller_server
[lifecycle_manager]: Configuring planner_server
[lifecycle_manager]: Configuring behavior_server
[lifecycle_manager]: Configuring bt_navigator
[lifecycle_manager]: Configuring waypoint_follower
...
[lifecycle_manager]: Managed nodes are active          ← Nav2 栈就绪
[mission_manager]: Waiting for first /map message...   ← 等 SLAM 建图
[mission_manager]: Sent mission with 5 waypoints.      ← 任务开始！
[mission_manager]: Current waypoint index: 0           ← 前往航点 1
[mission_manager]: Current waypoint index: 1           ← 前往航点 2
[mission_manager]: Current waypoint index: 2           ← 前往航点 3
[mission_manager]: Current waypoint index: 3           ← 前往航点 4
[mission_manager]: Current waypoint index: 4           ← 前往航点 5
[mission_manager]: Mission completed successfully.     ← ✅ 任务全部完成
[mission_manager]: Map saved: .../part1_slam_map.pgm   ← ✅ SLAM 地图已保存
```

**在 Gazebo 中向老师指出（对应 Task）：**

| 观察现象 | 对应 Task |
|---|---|
| 机器人开始自动移动，沿路径平滑行驶 | Task 4 — ROS2 自主驾驶栈 |
| 路径跟踪平滑，无大幅抖动 | Task 5 — 自定义 lookahead controller |
| 到达航点后**原地旋转**对齐目标朝向 | Task 5 — heading alignment |
| 依次前往 5 个不同位置 | Task 6 — 多航点路径 |
| 遇到 cone/bucket/cricket net 时**自动绕行** | Task 7 — 静态障碍物避障 |

---

### 步骤 5 — Terminal 3：运行中架构验证 (Tasks 4, 5)

**在机器人导航过程中**，新开 Terminal 3：

```bash
cd /home/god/auto4508/project
source install/setup.bash
```

**5.1 验证完整 ROS2 节点架构 (Task 4)：**
```bash
ros2 node list
```
**预期输出（关键节点标注）：**
```
/async_slam_toolbox_node       ← SLAM 建图
/behavior_server               ← Nav2 恢复行为（spin/backup/wait）
/bt_navigator                  ← 行为树导航器
/clock_sanitizer               ← 时钟清洗器（Gazebo→ROS）
/controller_server             ← Nav2 局部控制器
/joint_state_sanitizer         ← 关节状态清洗器
/lifecycle_manager_navigation  ← 生命周期管理
/mission_manager               ← 航点任务管理器（自编）
/odom_sanitizer                ← 里程计清洗器
/path_recorder                 ← 路径 CSV 记录器（自编）
/planner_server                ← Nav2 全局规划器
/robot_state_publisher         ← URDF→TF
/ros_gz_bridge                 ← Gazebo↔ROS 桥接
/waypoint_follower             ← 航点跟随器
```

> **向老师说明**："`sim.launch.py` 负责仿真和桥接层，`nav.launch.py` 负责感知决策层，完全解耦，可独立重启。"

**5.2 验证自定义 Controller (Task 5) — 关键命令：**
```bash
ros2 param get /controller_server FollowPath.plugin
```
**预期输出：**
```
String value is: p3at_nav_plugins::P3ATController
```

> **这一条命令直接证明**使用的是自定义控制器 `P3ATController`，而不是 DWB 等默认插件。

> **向老师说明**："自定义 `P3ATController` 实现了 `nav2_core::Controller` 接口。核心功能：lookahead 路径跟踪（前方 0.8m）、接近减速（1.2m 内线性减速）、终点朝向对齐、前向碰撞预测（模拟 1 秒轨迹检查 costmap）。"

---

### 步骤 6 — 任务完成后验证输出 (Task 8)

等 Terminal 2 显示 **"Mission completed successfully"** 和 **"Map saved"** 后：

**Terminal 2: `Ctrl+C` 关闭 nav**

**在 Terminal 3 验证输出文件：**

**6.1 SLAM 地图：**
```bash
ls -lh /home/god/auto4508/project/part1_slam_map.*
```
预期：
```
-rw-r--r-- 1 god god  XXK ... part1_slam_map.pgm    ← 占位格栅地图图像
-rw-r--r-- 1 god god  XXX ... part1_slam_map.yaml   ← 地图元数据
```

```bash
cat /home/god/auto4508/project/part1_slam_map.yaml
```
预期包含：`resolution: 0.050000`, `origin: [...]`, `image: part1_slam_map.pgm`

**6.2 行驶路径 CSV：**
```bash
head -5 /home/god/auto4508/project/part1_driven_path.csv
```
预期：
```
timestamp,x,y,z,roll,pitch,yaw
1234567.89,0.05,0.02,0.0,0.001,-0.001,0.01
1234568.39,0.15,0.04,0.0,0.001,-0.001,0.03
...
```

```bash
wc -l /home/god/auto4508/project/part1_driven_path.csv
```
预期：几百行数据（含表头），证明完整记录了行驶路径。

**6.3 Rosbag2 录制：**
```bash
ros2 bag info /home/god/auto4508/project/part1_demo_run
```
预期包含：Duration、Message count、录制的 topics 列表（/tf, /scan, /imu, /cmd_vel, /map, /camera/image 等）。

> **向老师说明**："slam_toolbox 在线建图，任务完成后自动调用 map_saver 保存地图。path_recorder 持续订阅 /odometry/filtered 输出 CSV。rosbag2 录制了完整话题数据供回放分析。"

---

## 如何指定机器人去某个坐标，以及输出保存到哪里

### 1. 当前项目里“指定目标位置”的正式方式

当前实现**不是**在 Gazebo 里点击地面，也**不是**靠 RViz 的 2D Goal 手动发点。

正式方式是编辑：

```bash
/home/god/auto4508/project/src/p3at_bringup/config/waypoints.yaml
```

当前文件内容：

```yaml
waypoints:
  frame_id: map
  poses:
    - {x: 1.5, y: 1.0, yaw: 0.0}
    - {x: 4.5, y: 1.0, yaw: 0.0}
    - {x: 6.0, y: 2.5, yaw: 0.8}
    - {x: 6.3, y: 6.3, yaw: 1.6}
    - {x: 2.5, y: 7.4, yaw: 3.14}
```

字段含义：

| 字段 | 含义 | 单位 / 说明 |
|---|---|---|
| `frame_id` | 航点坐标系 | 当前固定为 `map` |
| `x` | 地图坐标系下前后方向位置 | 米 |
| `y` | 地图坐标系下左右方向位置 | 米 |
| `yaw` | 到达该点后希望朝向的角度 | 弧度，逆时针为正 |

### 2. 这些坐标是如何进入导航系统的

链路如下：

1. `nav.launch.py` 读取 launch 参数 `waypoints_file`
2. 默认指向 `p3at_bringup/config/waypoints.yaml`
3. `mission_manager` 从 YAML 读取 `x/y/yaw`
4. 把 `yaw` 转成 quaternion
5. 等待第一帧 `/map`
6. 再延迟 6 秒，保证 SLAM / costmap 稳定
7. 通过 Nav2 的 `FollowWaypoints` action 一次性发送全部航点
8. `waypoint_follower` 逐个调用 `bt_navigator`
9. planner 生成路径，controller 跟踪路径，到点后转到目标 `yaw`

### 3. 如果你只想让机器人去“一个特定位置”

最简单的方法：把 `poses:` 里只保留一个点。

例如，让机器人只去 `(x=3.0, y=2.0)`，并在到达后朝北（`yaw=1.57`）：

```yaml
waypoints:
  frame_id: map
  poses:
    - {x: 3.0, y: 2.0, yaw: 1.57}
```

然后重新启动：

```bash
cd /home/god/auto4508/project
source install/setup.bash
ros2 launch p3at_bringup nav.launch.py
```

### 4. 如果你想保留默认文件，再单独指定另一组坐标

可以自己新建一个 YAML，例如：

```bash
/home/god/auto4508/project/my_waypoints.yaml
```

内容示例：

```yaml
waypoints:
  frame_id: map
  poses:
    - {x: 2.0, y: 1.0, yaw: 0.0}
    - {x: 2.0, y: 4.0, yaw: 1.57}
```

启动时显式指定：

```bash
cd /home/god/auto4508/project
source install/setup.bash
ros2 launch p3at_bringup nav.launch.py \
  waypoints_file:=/home/god/auto4508/project/my_waypoints.yaml
```

### 5. 这些坐标到底相对于哪里

这些坐标是相对于 `map` 坐标系，而不是屏幕像素，也不是 Gazebo 相机视角。

要点：

- 单位是**米**
- `yaw` 单位是**弧度**
- 地图原点附近就是机器人重启仿真后的初始位置
- 所以 **teleop 演示后必须重启 sim**，否则机器人当前位置和你设定的 waypoint 会错位，planner 可能报 out of bounds / failed to create plan

常用朝向参考：

| yaw | 朝向 |
|---|---|
| `0.0` | 朝 +x |
| `1.57` | 朝 +y |
| `3.14` | 朝 -x |
| `-1.57` | 朝 -y |

### 6. 改了 YAML 后要不要重新编译

**不用。**

当前 workspace 是 `colcon build --symlink-install` 工作流。

修改以下文件都不需要重新 build：

- `waypoints.yaml`
- `nav2.yaml`
- `slam_toolbox.yaml`
- launch 文件
- world / SDF 文件

改完后重新 `source install/setup.bash` 并重新启动对应 launch 即可。

### 7. 地图、路径、rosbag 是怎么保存的

当前项目有三种输出：

| 输出 | 由谁保存 | 触发时机 |
|---|---|---|
| `part1_slam_map.pgm/.yaml` | `mission_manager` 调 `map_saver_cli` | **全部 waypoint 完成后自动保存** |
| `part1_driven_path.csv` | `path_recorder` | **导航运行过程中持续写入** |
| `part1_demo_run/` | `ros2 bag record` | 只有 `record:=true` 时录制 |

### 8. 输出保存在哪里

在当前 DEMO_GUIDE 的标准启动方式里：

```bash
cd /home/god/auto4508/project
source install/setup.bash
ros2 launch p3at_bringup nav.launch.py record:=true
```

输出会保存在：

```bash
/home/god/auto4508/project
```

具体就是：

- `/home/god/auto4508/project/part1_slam_map.pgm`
- `/home/god/auto4508/project/part1_slam_map.yaml`
- `/home/god/auto4508/project/part1_driven_path.csv`
- `/home/god/auto4508/project/part1_demo_run/`

**重要：这些输出路径本质上取决于你启动 `nav.launch.py` 时的当前工作目录。**

所以如果你在别的目录执行 `ros2 launch ...`，文件也会落到那个目录，不一定是 `project/`。
本指南前面每一步都先 `cd /home/god/auto4508/project`，就是为了固定输出位置。

### 9. 如果任务还没结束，想手动保存地图

可以单独执行：

```bash
cd /home/god/auto4508/project
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f part1_slam_map \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0
```

### 10. 如何确认你设定的目标真的被加载了

运行 `nav.launch.py` 后，看 Terminal 2 日志：

```text
[mission_manager]: Sent mission with N waypoints.
```

其中 `N` 就是你 YAML 里 `poses:` 的个数。

如果你只写了一个点，日志应显示：

```text
[mission_manager]: Sent mission with 1 waypoints.
```

---

### 步骤 7 — 关闭

1. Terminal 2：已 Ctrl+C
2. Terminal 1：`Ctrl+C` 关闭 Gazebo
3. Terminal 3：关闭

---

## 8 项 Task 满分证据总表

| Task | 要求原文 | 满分证据 | 在哪个步骤 |
|---|---|---|---|
| **1** | Finish pioneer URDF + teleop from ROS2 | Gazebo 中四轮 Pioneer 可见 + 键盘遥控能动 | 步骤 1-2 |
| **2** | World SDF looks like James Oval, include cones and buckets | Gazebo 中：草坪、北楼、围栏、灯杆、板球网、5 cones、3 buckets | 步骤 1 |
| **3** | Relevant sensors, not idealised, real-world limitations | 3 种传感器频率正常 + echo 看到高斯噪声数据 | 步骤 3 |
| **4** | Set up ROS2 stack to drive autonomously | `ros2 node list` 显示 14+ 节点完整架构 | 步骤 5.1 |
| **5** | Create own local controller, correct position and orientation | `ros2 param get` 确认 P3ATController + 机器人平滑导航 + 到点旋转对齐 | 步骤 4-5.2 |
| **6** | Generate a path of multiple waypoints | 日志 "5 waypoints" + index 0→4 + Gazebo 看到依次到达 | 步骤 4 |
| **7** | Avoid previously unseen static obstacles | Gazebo 看到机器人绕开 cones/buckets/nets | 步骤 4 |
| **8** | Build a map and record the driven path | .pgm/.yaml + .csv + rosbag2 三种文件全部存在且有内容 | 步骤 6 |

---

## 老师常见问题与回答

### Q: Controller 怎么实现的？和 DWB/RPP 有什么不同？

> 自定义 `P3ATController`，实现 `nav2_core::Controller` 接口（C++），在 `p3at_nav_plugins` 包中编译为插件。
>
> 核心算法：
> 1. **Lookahead 路径跟踪** — 在全局路径上找前方 0.8m 处的目标点，计算航向误差驱动转向
> 2. **接近减速** — 距终点 1.2m 内线性降速，避免冲过目标
> 3. **终点朝向对齐** — 到达航点位置后，原地旋转对齐目标 yaw（tolerance 5°）
> 4. **前向碰撞预测** — 向前模拟 1 秒轨迹，逐步检查 local costmap；检测到 LETHAL/INSCRIBED 时抛出异常，触发 planner replan
>
> DWB 是采样式搜索（遍历速度空间评分选最优），RPP 是 Regulated Pure Pursuit。我的实现更简洁、专门优化了差速驱动的朝向对齐需求。

### Q: 传感器为什么是非理想的？噪声参数怎么定的？

> 三种传感器全部配置了高斯噪声：
> - **Lidar** (SICK TIM781): 距离噪声 stddev=0.02m, 270° FOV, 810 samples, 10Hz
> - **IMU** (Phidget Spatial): 角速度 stddev=0.001 rad/s, 线加速度 stddev=0.05 m/s², 20Hz
> - **Camera** (OAK-D V2): 像素噪声 stddev=0.0075, 160x120, 2Hz
>
> 参数参考真实传感器 datasheet 的典型测量不确定性，不是零噪声理想化模型。

### Q: SLAM 用的什么？为什么选它？

> `slam_toolbox` 的 `online_async` 模式。它基于图优化（Ceres Solver），异步更新地图，CPU 开销低，适合实时导航。分辨率 0.05m，scan 最大范围 18m。

### Q: 路径规划怎么做的？

> - **全局规划**: `NavfnPlanner`（Dijkstra 搜索基于 global costmap）
> - **局部控制**: 自定义 `P3ATController`（lookahead + 碰撞预测）
> - **恢复行为**: `behavior_server` 提供 spin/backup/wait，卡住时自动触发

### Q: 避障原理？

> 三层机制：
> 1. **Global costmap** — obstacle_layer 从 /scan 标记障碍 + inflation_layer（半径 0.6m）膨胀 → planner 生成的路径**自动绕开**
> 2. **Local costmap** — 8m×8m 滚动窗口实时更新，检测新出现的障碍
> 3. **Controller 碰撞预测** — 前向模拟 1 秒轨迹检查 costmap cost，检测到碰撞时触发 replan

### Q: 什么是 sanitizer？为什么需要？

> Gazebo Harmonic 桥接到 ROS2 的数据可能存在：时钟时间戳倒退、里程计零协方差、关节状态空白帧。
> 三个 sanitizer 节点（clock/odom/joint_state）过滤掉这些异常，保证 TF 树单调递增、Nav2 栈收到合法数据。
> 如果不过滤，slam_toolbox 和 Nav2 会报 TF extrapolation 错误并崩溃。

### Q: 两条 launch 分别干什么？

> | Launch | 职责 | 节点 |
> |---|---|---|
> | `sim.launch.py` | 仿真 + 桥接 | Gazebo, bridge, robot_state_publisher, 3 sanitizers |
> | `nav.launch.py` | 感知 + 决策 | SLAM, Nav2 (5 servers + lifecycle), mission_manager, path_recorder |
>
> 完全解耦。可以 Ctrl+C nav 后重新启动，不需要重启 Gazebo。

### Q: 航点是怎么定义和执行的？

> `waypoints.yaml` 定义 5 个 (x, y, yaw) 坐标。`mission_manager` 等 /map 话题就绪后延迟 6 秒，通过 Nav2 `FollowWaypoints` action 一次性发送全部航点。waypoint_follower 按顺序调用 bt_navigator 逐个导航。

### Q: robot_localization / EKF 在哪里？

> 当前实现中，`odom_sanitizer` 直接将清洗后的 Gazebo 里程计发布到 `/odometry/filtered`，同时广播 odom→base_link TF。这在仿真中足够精确。Part 2/3 切换到真实机器人时会接入 `robot_localization` EKF 融合 IMU + 轮式里程计。

---

## 故障快速处理

| 现象 | 原因 | 解决 |
|---|---|---|
| Gazebo 打开后看不到机器人 | 相机视角未对准原点 | 滚轮缩小 + 左键旋转找到原点 (0,0) |
| **Planner 报 "Failed to create plan"** | **teleop 后未重启 sim，机器人偏离原点太远** | **Ctrl+C sim.launch.py → 重新启动 → 再启动 nav** |
| **"Robot is out of bounds of the costmap"** | **同上，机器人不在 costmap 范围内** | **同上，重启 sim 重置机器人到 (0,0)** |
| 导航启动后机器人长时间不动 | mission_manager 在等 /map + 6 秒延迟 | 正常，等 "Sent mission with 5 waypoints" 日志 |
| 机器人导航中途停顿几秒 | behavior_server 触发恢复行为（spin/backup） | 正常，Nav2 自动恢复 |
| `ros2 bag record` 报目录已存在 | 之前的 part1_demo_run/ 未删除 | `rm -rf part1_demo_run` 后重试 |
| 地图文件不存在 | nav 提前 Ctrl+C 或 map_saver 超时 | 手动：`ros2 run nav2_map_server map_saver_cli -f part1_slam_map --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0` |
| `ros2 param get` 报节点不存在 | nav 已关闭 | 需要在 nav 运行期间执行 |
| Gazebo 卡顿 | 残留进程占用资源 | 执行"展示前清理"命令 |
