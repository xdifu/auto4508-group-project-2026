# Part 1 Presentation Guide / Part 1 展示答辩指南

> **Purpose / 用途**: Bilingual reference for the 10-minute TA demo. Follow the demo steps in order; each step includes commands, what to show, what to say (EN for TA, CN for yourself), code highlights, and potential Q&A.
>
> **用途**: 中英双语的10分钟助教展示参考。按步骤顺序进行，每步包含命令、展示要点、解说词（英文给助教，中文给自己）、代码亮点和潜在问答。

---

## Architecture Overview / 架构概览

```
project/src/
├── p3at_description/     # ament_cmake: URDF xacro, meshes, RViz configs
├── p3at_bringup/         # ament_python: world, launch, config, Python nodes
└── p3at_nav_plugins/     # ament_cmake: custom C++ Nav2 controller plugin
```

**EN**: "Three ROS2 packages. Description handles the robot model. Bringup handles world, launch files, configs, and Python utility nodes. Nav plugins is a C++ package containing my custom Nav2 controller."

**CN**: 三个 ROS2 包。description 管机器人模型，bringup 管世界/launch/配置/Python工具节点，nav_plugins 是自定义C++控制器插件。

### TF Tree / TF 树

```
map ──→ odom ──→ base_link ──→ laser_frame
 (slam)  (odom_     (robot_     cam_frame
          sanitizer)  state_pub)  cam_optical_link
                                  p3at_*_wheel (x4)
```

**EN**: "Strict TF responsibility separation. slam_toolbox publishes map→odom. odom_sanitizer publishes odom→base_link. robot_state_publisher publishes all static frames from URDF. No duplicates, no conflicts."

**CN**: TF 严格分工，无冲突。slam 管 map→odom，odom_sanitizer 管 odom→base_link，robot_state_publisher 管 base_link 以下所有静态帧。

### Launch Architecture / Launch 架构

```
sim.launch.py                    nav.launch.py
├── Gazebo Harmonic              ├── slam_toolbox (online_async)
├── robot_state_publisher        ├── Nav2 (5 servers + lifecycle)
├── ros_gz_bridge                │   ├── planner_server (NavfnPlanner)
├── clock_sanitizer              │   ├── controller_server (P3ATController)
├── odom_sanitizer               │   ├── behavior_server (spin/backup/wait)
└── joint_state_sanitizer        │   ├── bt_navigator
                                 │   └── waypoint_follower
                                 ├── mission_manager
                                 ├── path_recorder
                                 └── rosbag2 (optional)
```

**EN**: "Two completely decoupled launch files. sim handles simulation infrastructure. nav handles perception and decision. You can restart nav without touching Gazebo."

**CN**: 两条 launch 完全解耦。sim 管仿真基础设施，nav 管感知决策层，可以独立重启 nav 不影响 Gazebo。

---

## Step 1 — Launch Simulation (Tasks 1, 2) / 步骤1：启动仿真

### Commands / 命令

```bash
cd /home/god/auto4508/project
source install/setup.bash
ros2 launch p3at_bringup sim.launch.py
```

### What to Show in Gazebo / 在 Gazebo 中展示

| Show to TA / 向助教指出 | Task | EN Explanation | CN 说明 |
|---|---|---|---|
| Pioneer robot: pink chassis + yellow top + 4 black wheels | Task 1 | "Complete Pioneer 3-AT URDF model with four-wheel differential drive" | 完整 P3AT URDF 模型，四轮差速 |
| Oval green grass field (23m x 30m) | Task 2 | "James Oval recreation with grass field and elliptical boundary" | James Oval 草地 + 椭圆边界 |
| 32-segment reddish-brown oval boundary | Task 2 | "32 wall segments forming the oval walking path boundary" | 32段围墙模拟 laterite 步行道 |
| Sandstone north building | Task 2 | "North building representing UWA architecture" | 北侧 UWA 建筑 |
| Cricket pitch (brown rectangle at center) | Task 2 | "Central cricket pitch" | 中央板球场 |
| 3 cricket nets (south-east) | Task 2 | "Three cricket net frames in the south-east area" | 东南区板球网 |
| 4 light poles (four quadrants) | Task 2 | "Four light poles placed around the oval" | 四根灯杆 |
| **5 orange cones** | Task 2 | "Five orange traffic cones as obstacles — these are static obstacles the robot must avoid" | **5个橙色圆锥障碍物** |
| **3 blue buckets** | Task 2 | "Three blue buckets as obstacles" | **3个蓝色桶障碍物** |

### Code Highlights for Task 1 / Task 1 代码亮点

**File: `p3at_description/urdf/pioneer.urdf.xacro`**

```
EN: "I used a xacro macro to generate all four wheel modules. Each wheel has
inertial, visual, collision, and a continuous joint with damping."
```

- **Xacro macro** `wheel_module` (line 47): generates axle → hub → wheel for each corner
- **4 wheels** instantiated at lines 144-147: `front_left`, `front_right`, `back_left`, `back_right`
- **DiffDrive plugin** (lines 296-311): 4 wheel joints, `wheel_separation=0.394`, `publish_odom_tf=false`
- **JointStatePublisher** (lines 293-295): publishes `/joint_states` for `robot_state_publisher`

**CN**: xacro 宏生成四个轮模块（轴→轮毂→轮），DiffDrive 差速插件控制4个轮关节，`publish_odom_tf=false` 避免 TF 冲突。

### Code Highlights for Task 2 / Task 2 代码亮点

**File: `p3at_bringup/worlds/james_oval.sdf`**

- **Grass field**: 30x36m green plane (line 75)
- **Oval boundary**: 32-segment ellipse with collision walls, 0.5m tall (lines 94-386)
- **Obstacles**: 5 cones (radius 0.12m, height 0.5m) + 3 buckets (radius 0.2m, height 0.5m)
- All obstacles are `<static>true</static>` with `<collision>` geometry intersecting lidar plane (z=0.281)
- **Robot**: included as `model://pioneer` in world (line 574), not spawned dynamically — avoids WSL2 rendering bug

**EN**: "The world has a grass ground plane, 32-segment oval boundary matching James Oval's shape, a north building, cricket pitch and nets, light poles, and 8 obstacle objects — 5 cones and 3 buckets — all with collision geometry at the lidar scan height."

**CN**: 世界包含草地、32段椭圆围墙、北楼、板球场/网、灯杆、8个障碍物（5锥+3桶），所有障碍物 collision 都穿过 lidar 扫描平面。

### Potential Questions / 可能的提问

**Q: Why include the robot in the world SDF instead of spawning it?**
- EN: "On WSL2, dynamic spawning with `ros2 run ros_gz_sim create` sometimes causes a rendering pipeline crash in Gazebo Harmonic. Including the robot directly in the world SDF is more reliable and the robot appears immediately when Gazebo loads."
- CN: WSL2 上动态 spawn 会导致 Gazebo Harmonic 渲染崩溃，直接在世界 SDF 里 include 更稳定。

**Q: How accurate is the James Oval model?**
- EN: "It's a 1:6 scale approximation. The real James Oval is about 140m x 180m. Key features are preserved: oval shape, surrounding path, north building, cricket pitch, cricket nets, light poles. The focus is on providing enough geometric features for SLAM while being small enough for efficient simulation."
- CN: 1:6 缩比近似，保留关键特征给 SLAM 提供几何特征，同时控制仿真规模。

---

## Step 2 — Teleop Demo (Task 1) / 步骤2：遥控演示

### Commands / 命令

```bash
cd /home/god/auto4508/project && source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keys: `i`=forward, `,`=backward, `j`=left turn, `l`=right turn, `k`=stop

### What to Say / 解说词

**EN**: "The Pioneer 3-AT URDF model is complete with four-wheel differential drive. I'm driving it with teleop_twist_keyboard publishing to /cmd_vel, which the ros_gz_bridge forwards to Gazebo's DiffDrive plugin."

**CN**: Pioneer 3-AT URDF 模型完成，四轮差速驱动，teleop_twist_keyboard 发布 /cmd_vel 通过 bridge 转发到 Gazebo DiffDrive 插件。

### After Teleop / 遥控后

**Press `k` to stop, then `Ctrl+C` to exit teleop.**

> **IMPORTANT**: Teleop moves the robot away from origin. Must restart sim before nav!
> **重要**：遥控会把机器人从原点移走，必须重启 sim 再启动 nav！

---

## Step 3 — Sensor Verification (Task 3) / 步骤3：传感器验证

### Commands / 命令

```bash
# 3.1 Lidar data
ros2 topic echo /scan --once 2>/dev/null | head -15

# 3.2 Sensor frequencies (Ctrl+C each after seeing rate)
ros2 topic hz /scan        # expect ~10 Hz
ros2 topic hz /imu         # expect ~20 Hz
ros2 topic hz /camera/image  # expect ~2 Hz
```

### What to Say / 解说词

**EN**: "Three sensors are modelled with Gaussian noise to simulate real hardware limitations:
- **Lidar** simulating SICK TIM781: 270-degree FOV, 810 samples, 10Hz, max range 20m, distance noise stddev 0.02m
- **IMU** simulating Phidget Spatial 3/3/3: angular velocity noise 0.001 rad/s, linear acceleration noise 0.05 m/s², 20Hz
- **Camera** simulating OAK-D V2: 160x120 RGB, 2Hz, pixel noise stddev 0.0075

None of the sensors are idealised — they all have noise parameters based on real sensor datasheets."

**CN**: 三种传感器全部配置高斯噪声模拟真实硬件：
- Lidar (SICK TIM781): 270° FOV, 810采样, 10Hz, 距离噪声0.02m
- IMU (Phidget Spatial): 角速度噪声0.001, 加速度噪声0.05, 20Hz
- Camera (OAK-D V2): 160x120, 2Hz, 像素噪声0.0075

### Code Highlights / 代码亮点

**File: `p3at_description/urdf/pioneer.urdf.xacro`**

- Lidar sensor (lines 216-242): `<type>gpu_lidar</type>`, `<samples>810</samples>`, `<min_angle>-2.35619</min_angle>` (±135° = 270°)
- IMU sensor (lines 272-290): `<type>imu</type>`, noise on all 6 axes (3 angular + 3 linear)
- Camera sensor (lines 244-269): `<type>camera</type>`, proper `cam_optical_link` (line 210: rpy `-1.5708 0 -1.5708`)

**Bridge**: `bridge.yaml` maps 8 topics (clock_raw, cmd_vel, scan, imu, odom, joint_states_raw, camera/image, camera/camera_info)

### Potential Questions / 可能的提问

**Q: Why is the camera only 2Hz and 160x120?**
- EN: "For Part 1, the camera is not in the control loop — it's modelled and bridged for future use in Parts 2 and 3 where we need image recognition. Low resolution and frame rate keeps GPU load minimal on WSL2."
- CN: Part 1 摄像头不参与控制闭环，低分辨率/帧率节省 WSL2 GPU 资源，Part 2/3 做视觉识别时会用。

**Q: Why 270° and not 360° for the lidar?**
- EN: "The real SICK TIM781 has a 270° scanning angle. I matched the simulation to the real sensor's specification to avoid over-optimistic results."
- CN: 真实 SICK TIM781 就是 270° 扫描角，仿真匹配真实规格避免过于理想化。

---

## Step 3.5 — Restart Simulation / 步骤3.5：重启仿真

### Commands / 命令

Terminal 1: `Ctrl+C` to stop sim, then:
```bash
ros2 launch p3at_bringup sim.launch.py
```

**EN**: "I'm restarting the simulation to reset the robot to the origin before autonomous navigation. The two launch files are fully decoupled — sim handles infrastructure, nav handles decision-making."

**CN**: 重启仿真让机器人回到原点 (0,0)，准备自主导航。两条 launch 完全解耦。

---

## Step 4 — Autonomous Navigation (Tasks 4, 5, 6, 7, 8) / 步骤4：自主导航

### Commands / 命令

```bash
cd /home/god/auto4508/project && source install/setup.bash
ros2 launch p3at_bringup nav.launch.py record:=true
```

### Expected Log Timeline / 预期日志时间线

```
[lifecycle_manager]: Configuring controller_server / planner_server / ...
[lifecycle_manager]: Managed nodes are active          ← Nav2 ready
[mission_manager]: Waiting for first /map message...   ← Waiting for SLAM
[mission_manager]: Sent mission with 5 waypoints.      ← Mission started!
[mission_manager]: Current waypoint index: 0→1→2→3→4   ← Sequential progress
[mission_manager]: Mission completed successfully.     ← All 5 done!
[mission_manager]: Map saved: .../part1_slam_map.pgm   ← SLAM map saved
```

### What to Show in Gazebo / 在 Gazebo 中观察

| Observation | Task | EN Explanation |
|---|---|---|
| Robot moves smoothly along path | Task 4 | "Complete ROS2 autonomous driving stack running" |
| Smooth path tracking, no oscillation | Task 5 | "Custom lookahead controller with heading-proportional steering" |
| Robot rotates in-place at waypoints | Task 5 | "Heading alignment — controller switches to pure rotation at goal" |
| Robot visits 5 different locations | Task 6 | "Five waypoints from waypoints.yaml executed via FollowWaypoints" |
| Robot detours around cones/buckets | Task 7 | "Costmap obstacle detection → planner replanning around obstacles" |
| Spin recovery if stuck near obstacle | Task 7 | "Behavior tree triggers spin/backup recovery when controller reports collision" |

### What to Say / 解说词

**EN**: "The nav launch starts SLAM, the full Nav2 stack, and my mission manager. The mission manager waits for the first /map message from slam_toolbox, then waits 6 seconds for the map to stabilize, then sends all 5 waypoints to Nav2's FollowWaypoints action. Nav2's waypoint follower calls the behavior tree navigator for each waypoint, which uses the global planner to plan a path and my custom controller to follow it."

**CN**: nav launch 启动 SLAM + Nav2 全栈 + mission_manager。mission_manager 等第一帧 /map 后延迟6秒，然后通过 FollowWaypoints action 发送5个航点。waypoint_follower 逐个调用 bt_navigator，用全局 planner 规划路径，用自定义 controller 跟踪。

### Code Highlights for Task 4 — Autonomous Stack / Task 4 代码亮点

**File: `p3at_bringup/launch/nav.launch.py`**

- **SLAM**: `slam_toolbox` online_async mode (line 39-45)
- **Nav2 servers**: planner, controller, behavior, bt_navigator, waypoint_follower (lines 53-101)
- **Lifecycle manager**: manages startup/shutdown order (lines 103-120)
- **Mission manager**: auto-starts mission after map ready (lines 122-133)
- **Path recorder**: records driven path to CSV (lines 135-145)
- **Rosbag2**: optional recording of 11 topics (lines 157-179)

### Code Highlights for Task 5 — Custom Controller / Task 5 代码亮点

**File: `p3at_nav_plugins/src/p3at_controller.cpp`**

This is the most important file for the demo — it's the **custom-written** controller.

**Core algorithm in `computeVelocityCommands()` (lines 101-168):**

```
1. Transform global plan to local frame (line 119)
2. Prune passed waypoints from plan (line 124)
3. If within goal distance tolerance (0.25m):
   → Pure rotation to align heading (lines 134-139)
   → angular_cmd = align_gain * yaw_error (clamped)
4. Else (path tracking mode):
   → Find lookahead point 0.8m ahead on path (line 141)
   → Calculate heading error to lookahead point (lines 142-145)
   → Linear speed with slowdown near goal (lines 147-151)
   → Angular speed proportional to heading error (line 153)
   → If heading error > 45°: stop linear, rotate only (lines 154-156)
5. Collision check before sending command (lines 159-163)
```

**EN**: "My P3ATController implements the nav2_core::Controller interface in C++. The core is a three-phase algorithm:
1. **Path tracking**: lookahead-based steering — I find a point 0.8m ahead on the planned path and steer toward it proportionally.
2. **Approach slowdown**: when within 1.2m of the goal, linear speed decreases linearly.
3. **Heading alignment**: when within 0.25m of the goal position, the robot stops moving forward and rotates in-place to match the target heading within 5 degrees.

Additionally, the controller does forward collision prediction — it simulates the robot's trajectory 1 second ahead, checking the local costmap at each step. If it detects a lethal or inscribed obstacle, it throws a ControllerException which triggers the behavior tree's recovery."

**CN**: 自定义 P3ATController 实现 nav2_core::Controller 接口（C++），三段算法：
1. **路径跟踪**: lookahead 前视0.8m，航向误差比例转向
2. **接近减速**: 距终点1.2m内线性减速
3. **朝向对齐**: 距终点0.25m内停车，原地旋转对齐目标航向（容差5°）
附加：前向碰撞预测模拟1秒轨迹检查 costmap。

**Collision prediction** (`isCollisionImminent`, lines 254-288):
- Simulates forward trajectory at 0.1s intervals for `collision_lookahead_time` (1.0s from yaml)
- Checks each point against local costmap
- Returns true on LETHAL or INSCRIBED cost → triggers recovery

### Code Highlights for Task 6 — Multiple Waypoints / Task 6 代码亮点

**File: `p3at_bringup/config/waypoints.yaml`**

```yaml
waypoints:
  frame_id: map
  poses:
    - {x: 1.5, y: 1.0, yaw: 0.0}    # WP1: south-east
    - {x: 4.5, y: 1.0, yaw: 0.0}    # WP2: east along south fence
    - {x: 6.0, y: 2.5, yaw: 0.8}    # WP3: turning north-east
    - {x: 6.3, y: 6.3, yaw: 1.6}    # WP4: deep north-east
    - {x: 2.5, y: 7.4, yaw: 3.14}   # WP5: north, facing west
```

**File: `p3at_bringup/p3at_bringup/mission_manager.py`**

- Loads waypoints from YAML, converts yaw to quaternion (lines 45-70)
- Waits for `/map` topic + 6 second delay (lines 89-107)
- Sends all waypoints via `FollowWaypoints` action (lines 109-113)
- Tracks progress via feedback callback (lines 116-120)
- After completion: auto-saves SLAM map via `map_saver_cli` (lines 142-172)

**EN**: "The mission manager loads 5 waypoints from a YAML config, waits for the SLAM map to be ready, then sends them all at once to Nav2's FollowWaypoints action. The waypoint follower executes them sequentially. After all waypoints are completed, the mission manager automatically saves the SLAM map."

**CN**: mission_manager 从 YAML 加载5个航点，等 SLAM 就绪后通过 FollowWaypoints 一次性发送，waypoint_follower 顺序执行。全部完成后自动调用 map_saver 保存地图。

### Code Highlights for Task 7 — Obstacle Avoidance / Task 7 代码亮点

**File: `p3at_bringup/config/nav2.yaml`**

Three-layer obstacle avoidance:

1. **Global costmap** (lines 91-123): `obstacle_layer` marks obstacles from `/scan` + `inflation_layer` (0.6m radius) → planner generates paths that automatically avoid known obstacles
2. **Local costmap** (lines 125-153): 8m x 8m rolling window, same layers → real-time detection of newly-seen obstacles
3. **Controller collision prediction** (p3at_controller.cpp lines 254-288): 1-second forward trajectory check against local costmap
4. **Recovery behaviors** (lines 62-78): `spin`, `backup`, `drive_on_heading`, `wait`

**EN**: "Obstacle avoidance works at three levels:
1. The global costmap obstacle layer marks obstacles detected by the lidar and inflates them by 0.6m. The NavFn planner generates paths that avoid these inflated areas.
2. The local costmap is an 8-by-8 meter rolling window that updates at 10Hz, catching newly-discovered obstacles.
3. My custom controller predicts collision 1 second ahead — if a lethal cell is detected, it throws an exception that triggers the behavior tree's recovery sequence: clear costmap, spin, backup.

So even if an obstacle wasn't visible when the path was planned, the robot will detect it, replan, or recover."

**CN**: 避障三层机制：
1. 全局 costmap obstacle_layer + inflation(0.6m) → planner 自动绕开
2. 局部 costmap 8x8m 滚动窗口 10Hz 更新 → 实时发现新障碍
3. 自定义 controller 前向碰撞预测1秒 → 检测到碰撞触发 recovery（spin/backup）

### Code Highlights for Task 8 — Mapping & Recording / Task 8 代码亮点

**SLAM mapping**: `slam_toolbox` online_async, Ceres solver, resolution 0.05m, max laser range 18m

**File: `p3at_bringup/p3at_bringup/path_recorder.py`**

- Subscribes to `/odometry/filtered` (line 39)
- Publishes `/driven_path` as nav_msgs/Path (line 38)
- Exports CSV with columns: `timestamp, x, y, z, roll, pitch, yaw` (line 44)
- Samples every 0.5s with 0.05m minimum translation threshold (lines 21-22)
- Flushes to file in real-time (line 103)

**Rosbag2** (nav.launch.py lines 157-179): Records 11 topics including `/tf`, `/scan`, `/imu`, `/map`, `/cmd_vel`, `/camera/image`

**EN**: "Three types of data recording:
1. **SLAM map** — slam_toolbox builds the map online, and mission_manager auto-saves it as .pgm + .yaml after mission completion
2. **Driven path CSV** — path_recorder samples odometry every 0.5 seconds and writes timestamp, position, and orientation to a CSV file
3. **Rosbag2** — full ROS2 bag recording of 11 topics for offline replay and analysis"

**CN**: 三种数据记录：
1. SLAM 地图 — slam_toolbox 在线建图，任务完成后自动保存 .pgm/.yaml
2. 行驶路径 CSV — path_recorder 每0.5秒采样 odometry 输出 CSV
3. Rosbag2 — 11个 topic 完整录制供回放分析

---

## Step 5 — Runtime Verification (Tasks 4, 5) / 步骤5：运行时验证

### Commands (in Terminal 3, while robot is navigating) / 命令

```bash
cd /home/god/auto4508/project && source install/setup.bash

# 5.1 Node list (Task 4)
ros2 node list

# 5.2 Verify custom controller (Task 5) — KEY COMMAND
ros2 param get /controller_server FollowPath.plugin
# Expected: String value is: p3at_nav_plugins::P3ATController
```

### What to Say / 解说词

**EN**: "`ros2 node list` shows 14 nodes running — the complete autonomous stack. And `ros2 param get` confirms the controller is my custom `P3ATController`, not a default plugin like DWB or RPP."

**CN**: `ros2 node list` 显示14个节点完整架构，`ros2 param get` 确认使用自定义 P3ATController 而非默认 DWB/RPP。

---

## Step 6 — Output Verification (Task 8) / 步骤6：输出验证

After "Mission completed" → `Ctrl+C` nav. Then in Terminal 3:

```bash
# 6.1 SLAM map
ls -lh part1_slam_map.*
cat part1_slam_map.yaml

# 6.2 Driven path CSV
head -5 part1_driven_path.csv
wc -l part1_driven_path.csv

# 6.3 Rosbag2
ros2 bag info part1_demo_run
```

### What to Say / 解说词

**EN**: "Three output artifacts prove Task 8 completion:
1. SLAM map — .pgm image and .yaml metadata with resolution 0.05m
2. Driven path CSV — about 100 rows of timestamped position data
3. Rosbag2 — full recording of all sensor and control topics"

**CN**: 三种输出文件证明 Task 8 完成：SLAM 地图(.pgm/.yaml)、行驶路径 CSV(~100行)、rosbag2 完整录制。

---

## Comprehensive Q&A / 完整问答

### Controller Questions / 控制器相关

**Q: How is your controller different from DWB or Regulated Pure Pursuit?**

EN: "DWB (Dynamic Window Approach) is a sampling-based method — it samples velocity pairs, simulates trajectories, and scores them against multiple criteria. RPP (Regulated Pure Pursuit) uses a curvature-based geometric approach. My controller is simpler and purpose-built: it uses a single lookahead point for steering, has an explicit three-phase behavior (track → slowdown → align), and directly checks the costmap along the predicted trajectory for collision rather than scoring candidate paths. It's designed specifically for differential-drive robots like the Pioneer."

CN: DWB 是采样式（遍历速度空间评分），RPP 是曲率几何方法。我的控制器更简洁：单前视点转向 + 明确三段行为（跟踪→减速→对齐）+ 直接沿预测轨迹检查 costmap 碰撞。专为差速驱动设计。

---

**Q: Why did you choose lookahead distance 0.8m?**

EN: "0.8m was tuned experimentally. Too short (0.3-0.5m) causes oscillation as the robot chases near points. Too long (1.5m+) cuts corners and misses tight turns around obstacles. 0.8m gives smooth tracking while still allowing the robot to navigate around the obstacles in our world."

CN: 实验调参。太短会振荡（追近点），太长会切弯。0.8m 在平滑跟踪和绕障精度之间取得平衡。

---

**Q: What happens when the controller detects a collision?**

EN: "The `isCollisionImminent` function simulates the robot's trajectory 1 second ahead at 0.1s intervals. If any point lands on a LETHAL or INSCRIBED cell in the local costmap, it throws a `nav2_core::ControllerException`. This exception propagates up to the behavior tree, which triggers the recovery sequence: first it tries clearing the costmap, then spinning in place, then backing up. After recovery, the planner replans a new path."

CN: `isCollisionImminent` 前向模拟1秒轨迹，逐步检查 local costmap。检测到 LETHAL/INSCRIBED 则抛出 ControllerException，触发行为树 recovery（清 costmap → spin → backup），之后 planner 重规划。

---

### Sensor & Bridge Questions / 传感器与桥接

**Q: What are the sanitizer nodes? Why do you need them?**

EN: "Gazebo Harmonic's ros_gz_bridge can produce problematic data: clock timestamps that go backward, odometry with zero covariance matrices, and joint state messages with empty frames. These break slam_toolbox and Nav2. The three sanitizers — clock, odom, and joint_state — filter out bad messages and republish clean data. For example, `clock_sanitizer` subscribes to `/clock_raw` (remapped from bridge) and only publishes to `/clock` if the timestamp is strictly increasing. `odom_sanitizer` adds proper covariance values and publishes the `odom→base_link` TF transform."

CN: Gazebo Harmonic 桥接数据有问题：时钟倒退、里程计零协方差、关节状态空帧。三个 sanitizer 过滤异常数据，保证 TF 单调递增、Nav2 收到合法数据。不过滤的话 slam_toolbox 和 Nav2 会报 TF extrapolation 错误。

---

**Q: Why remap /clock to /clock_raw and /joint_states to /joint_states_raw?**

EN: "The bridge publishes raw Gazebo data which may have issues. By remapping to `_raw` topics, the sanitizer nodes can subscribe to raw data, clean it, and republish on the standard topic names. This way, all other ROS2 nodes (SLAM, Nav2) receive clean data on the expected topic names without any special configuration."

CN: bridge 发布原始数据到 `_raw` topic，sanitizer 订阅原始数据、清洗后发布到标准 topic 名。其他 ROS2 节点（SLAM、Nav2）无需特殊配置就能收到干净数据。

---

**Q: Why is odom_sanitizer publishing the TF instead of the DiffDrive plugin?**

EN: "The DiffDrive plugin's `publish_odom_tf` is set to false in the URDF. This is intentional — if Gazebo published the TF directly, it could conflict with the sanitized odometry or create race conditions. By having the odom_sanitizer as the single source of truth for `odom→base_link`, we ensure TF consistency."

CN: DiffDrive 的 `publish_odom_tf=false`，避免与 sanitizer 发布的 TF 冲突。odom_sanitizer 是 odom→base_link TF 的唯一发布者，保证 TF 一致性。

---

### SLAM & Navigation Questions / SLAM 与导航

**Q: Why slam_toolbox and not gmapping or cartographer?**

EN: "slam_toolbox uses graph-based SLAM with Ceres solver, which is more accurate than gmapping's particle filter approach and lighter-weight than Cartographer for our use case. Its `online_async` mode processes scans asynchronously, reducing CPU load. It also natively integrates with Nav2's lifecycle management. slam_toolbox is the recommended SLAM solution in the Nav2 ecosystem."

CN: slam_toolbox 用图优化（Ceres Solver），比 gmapping 粒子滤波更准确，比 Cartographer 更轻量。online_async 异步处理降低 CPU 开销，原生支持 Nav2 生命周期管理。

---

**Q: Why NavfnPlanner and not Smac or Theta*?**

EN: "NavfnPlanner uses Dijkstra's algorithm on the costmap grid, which is simple, reliable, and computationally efficient. For our relatively small world with simple obstacles, it produces good paths. SmacPlanner is better for complex environments with narrow passages, and Theta* produces smoother paths but at higher computational cost. NavFn is the right trade-off for this simulation."

CN: NavfnPlanner 用 Dijkstra 搜索，简单可靠效率高。对我们这个相对简单的世界足够好。SmacPlanner 适合复杂窄通道，Theta* 更平滑但计算更贵。

---

**Q: How does the waypoint system work?**

EN: "The mission_manager node is an action client for Nav2's `FollowWaypoints` action. It loads 5 waypoint coordinates from `waypoints.yaml`, converts yaw angles to quaternions, waits for the SLAM map to be ready, then sends all waypoints in a single FollowWaypoints goal. The waypoint_follower server processes them one at a time, calling bt_navigator for each. When the behavior tree confirms goal reached, it moves to the next waypoint."

CN: mission_manager 是 FollowWaypoints action client。从 YAML 加载5个航点（yaw→quaternion），等 SLAM 就绪后一次性发送。waypoint_follower 逐个处理，每个调用 bt_navigator 导航到目标。

---

**Q: What's in the behavior tree?**

EN: "I'm using Nav2's default `navigate_to_pose` behavior tree. It first calls the planner to generate a global path, then the controller to follow it. If the controller reports failure, it tries recovery behaviors in order: clear costmap, spin, backup. If all recoveries fail, it reports the waypoint as failed."

CN: 使用 Nav2 默认 navigate_to_pose 行为树。先规划路径，然后 controller 跟踪。如果 controller 失败，按顺序尝试 recovery：清 costmap → spin → backup。

---

### Architecture Questions / 架构相关

**Q: Why three packages instead of one?**

EN: "Separation of concerns. `p3at_description` is pure robot model — it can be reused in different projects. `p3at_nav_plugins` is a C++ ament_cmake package because Nav2 plugins must be compiled shared libraries. `p3at_bringup` is an ament_python package containing all the mission-specific configuration, launch files, and Python utility nodes. This structure follows ROS2 best practices."

CN: 关注点分离。description 纯模型（可复用），nav_plugins 是 C++ cmake 包（Nav2 插件必须是编译的共享库），bringup 是 Python 包（配置/launch/工具节点）。遵循 ROS2 最佳实践。

---

**Q: Where is robot_localization / EKF?**

EN: "In the simulation, `odom_sanitizer` directly publishes the cleaned Gazebo odometry to `/odometry/filtered` and broadcasts the `odom→base_link` transform. Gazebo's DiffDrive odometry is already quite accurate in simulation. For Parts 2 and 3 with the real robot, we would plug in `robot_localization`'s EKF to fuse wheel odometry with IMU data. The `/odometry/filtered` topic name is kept consistent so that downstream nodes like path_recorder and Nav2 don't need any changes."

CN: 仿真中 odom_sanitizer 直接将清洗后的 Gazebo 里程计发布到 `/odometry/filtered`。Gazebo 里程计在仿真中已经足够精确。Part 2/3 真实机器人时接入 robot_localization EKF 融合 IMU + 轮式里程计。topic 名保持一致，下游节点无需修改。

---

**Q: What is use_sim_time and why is it everywhere?**

EN: "In simulation, all nodes must synchronize to Gazebo's simulated clock rather than wall clock. `use_sim_time: true` tells each node to subscribe to `/clock` for its time source. Without this, TF lookups would fail because nodes would have timestamps in different time domains. Every node — robot_state_publisher, SLAM, Nav2, mission_manager — must have this set consistently."

CN: 仿真中所有节点必须同步到 Gazebo 模拟时钟。`use_sim_time: true` 让节点订阅 `/clock` 作为时间源。不设置的话 TF 查找会因时间域不匹配而失败。所有节点必须一致设置。

---

**Q: How does the costmap inflation work?**

EN: "The inflation layer takes lethal obstacle cells and creates a cost gradient radiating outward. With `inflation_radius: 0.6m` and `cost_scaling_factor: 5.0`, any cell within 0.6m of an obstacle has elevated cost. The robot radius is 0.34m, so the planner ensures paths stay at least one robot-width from obstacles. The scaling factor controls how quickly the cost drops — higher values make the cost drop faster, allowing closer passes."

CN: inflation 层从致命障碍格向外辐射代价梯度。膨胀半径0.6m，机器人半径0.34m，所以 planner 保证路径至少离障碍一个机器人宽度。cost_scaling_factor=5.0 控制代价衰减速度。

---

### Troubleshooting / 故障排除

**Q: What if the planner fails to create a plan?**

EN: "Most likely the robot's position is outside the global costmap bounds — this happens if teleop moved the robot far from the origin and sim wasn't restarted. The fix is to restart sim.launch.py to reset the robot to (0,0). If the costmap is too small, the `width` and `height` parameters in nav2.yaml can be increased."

CN: 大概率是机器人位置超出 costmap 范围（teleop 移走后没重启 sim）。重启 sim 重置到 (0,0) 即可。

---

**Q: What if the map saver fails?**

EN: "We set `save_map_timeout: 10.0` seconds — this gives DDS enough time to discover the map topic on WSL2 where DDS discovery is slow (3-5 seconds). If it still fails, run manually: `ros2 run nav2_map_server map_saver_cli -f part1_slam_map --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0`"

CN: 设置 `save_map_timeout: 10.0` 秒给 DDS 足够发现时间（WSL2 上 DDS 发现需要3-5秒）。失败则手动执行命令。

---

## Task-to-Evidence Summary / 任务证据总表

| Task | Requirement | Evidence | Demo Step |
|---|---|---|---|
| **1** | Finish pioneer URDF + teleop | 4-wheel xacro model + DiffDrive + teleop_twist_keyboard | Steps 1-2 |
| **2** | World looks like James Oval + cones/buckets | Grass, oval boundary, building, cricket pitch/nets, poles, 5 cones, 3 buckets | Step 1 |
| **3** | Sensors not idealised, real-world limitations | Lidar/IMU/Camera all with Gaussian noise, realistic specs | Step 3 |
| **4** | ROS2 stack to drive autonomously | 14+ nodes: SLAM + Nav2 + lifecycle + sanitizers + mission/path tools | Steps 4-5 |
| **5** | Custom local controller, correct position+orientation | P3ATController (C++): lookahead tracking + slowdown + heading alignment | Steps 4-5 |
| **6** | Generate path of multiple waypoints | 5 waypoints in YAML, FollowWaypoints action, sequential execution | Step 4 |
| **7** | Avoid unseen static obstacles | Costmap obstacle/inflation layers + controller collision prediction + recovery | Step 4 |
| **8** | Build map + record driven path | SLAM map (.pgm/.yaml) + CSV path + rosbag2 (11 topics) | Step 6 |
