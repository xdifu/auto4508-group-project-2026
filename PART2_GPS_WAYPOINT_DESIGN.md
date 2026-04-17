# Part 2 — GPS 与 Waypoint Handling 详细设计规划（v2）

> 本文档仅覆盖 Part 2 中与 **GPS 定位** 和 **航点（waypoint）执行** 直接相关的子系统。
> 视觉识别（marker cone / 形状物体）、Lidar 反应式避障内部实现、Bluetooth 手柄与 dead-man 的硬件层，只在 §12《接口契约》中给出输入/输出契约，不展开内部实现。
> 本文档延续 Part 1（`PART1_DESIGN.md` / `DEMO_GUIDE.md`）的三包结构、`use_sim_time` 一致性、TF 唯一发布者约束、Nav2 + `p3at_nav_plugins::P3ATController` 主链路等约定。
> 目标：文档单独交给另一支设计团队时，他们能据此复现同一个可靠方案；文档内不出现任何代码。

---

## 0. 文档约定与不变量

**单位与坐标约定**
- 长度单位统一为米，角度单位统一为弧度，时间戳单位统一为秒（ROS2 `builtin_interfaces/Time`）。
- 航向 yaw 约定：在 `map` 系中 `yaw = 0` 指向 `+x`（ENU 下为东），正方向为从 `+x` 向 `+y` 的逆时针旋转（右手系）。
- 地理坐标使用 WGS84：纬度正为北，经度正为东，海拔为正高于椭球面。
- 协方差单位：`NavSatFix.position_covariance` 为 m²；IMU orientation 的协方差单位为 rad²。

**全局不变量**（在任何状态下都必须成立，违反即视为系统错误）
1. **TF 唯一发布者**：每一条父→子 TF 边在任意时刻只能由一个节点发布。Part 2 各帧的唯一发布者表见 §3.3。
2. **use_sim_time 一致**：仿真 launch 中所有节点（包含 `mission_orchestrator` / `robot_localization` 套件 / Nav2 全栈）必须 `use_sim_time: true`；真机 launch 必须全部 `false`。
3. **Datum 稳定**：一次任务启动后，`map` 系原点对应的地理 datum 不再变化，直到 `mission_orchestrator` 进入 `COMPLETED` 或 `ABORTED`。
4. **cmd_vel 唯一主人**：`cmd_vel` 在任一时刻只能由一个“已被仲裁的源”发布。Part 2 中仲裁层由 `safety_node` 维护（优先级：estop > manual_teleop > Nav2），`mission_orchestrator` 不直接发布 `cmd_vel`，只通过 Nav2 action 间接控制。
5. **无硬编码航点**：除 `home` 回落项外，`mission_orchestrator` 不允许在源码里写任何具体坐标；所有航点必须来自 YAML。
6. **fail-closed**：GPS / 安全层 / 视觉任意一项状态未确认时，orchestrator 必须处于非驱动状态（不下发 goal），而不是假设“默认 OK 先跑”。

**术语表**
- **Waypoint**：赛题“GPS 航点”，由 marker cone 标记，容差 1–2m。
- **Goal**：发给 Nav2 `NavigateToPose` 的目标位姿，通常 = waypoint + `pass_side` 偏移。
- **Arrival**：机器人与 waypoint（真实 cone 位置）距离 ≤ `arrival_radius` 并连续若干帧，由 `arrival_judge` 判定。
- **Segment / Leg**：两个相邻航点之间的一段任务，例如 `WP1 → WP2`；策略以 segment 为单位切换。
- **Datum**：`map` 系原点对应的地理点 `(lat₀, lon₀, alt₀)`，由 YAML 指定或由首帧有效 fix 确定。

---

## 1. 范围、与 Part 1 的关系、交付边界

### 1.1 从 Part 1 复用（不重新设计）
- 三包结构 `p3at_description` / `p3at_bringup` / `p3at_nav_plugins`。
- URDF、Gazebo 世界、sanitizer 节点、ros_gz_bridge、IMU / Lidar / Camera 传感器建模。
- Nav2 主链路：`NavfnPlanner` + `p3at_nav_plugins::P3ATController` + `nav2_costmap_2d` + `behavior_server` + `bt_navigator`。
- `path_recorder` 的 `/odometry/filtered` → CSV 链路（schema 复用，文件名换）。
- rosbag2 录制框架（topic 列表需扩展，见 §14.3）。

### 1.2 需要新增 / 改造
| 维度 | Part 1 现状 | Part 2 目标 |
|------|-------------|-------------|
| 定位 | `slam_toolbox` 发布 `map→odom` | `robot_localization` 双 EKF + `navsat_transform_node`，GPS 对齐的 `map` 系 |
| 航点 | YAML 中 `map` 系 `(x, y, yaw)` | YAML 中地理 `(lat, lon, …)`，运行时投影到 `map` 系 |
| 航点容差 | 严格 0.25m + yaw 对齐 | 软 1–2m，无 yaw 约束，但有“通过侧”几何约束 |
| 任务调度 | `mission_manager` 顺序 `FollowWaypoints` | `mission_orchestrator` 显式状态机 + 逐点 `NavigateToPose` |
| 航段参数 | 全局一套 Nav2 参数 | 按 segment 切换 `mission_policy` |
| 视觉交互 | 无 | 到点 marker 照片 + 形状物体识别（仅消费，不实现） |
| 安全层 | 无 | X/O/dead-man → `safety_node` → 仲裁 cmd_vel |
| 任务总结 | CSV + 地图 | JSON / markdown summary + 照片 + 扩充 rosbag |
| Sim/Real 兼容 | 仅 sim | sim 与 real 双 launch，接口完全镜像 |

### 1.3 本文档**不覆盖**的部分（仅在 §12 给出接口）
- Marker cone 检测、shape 识别、距离估计的内部算法（OpenCV / OAK-D 逻辑）。
- Bluetooth 手柄驱动、dead-man 硬件级实现。
- Lidar 避障的具体参数调优（沿用 Part 1 并在 §10 给出覆盖字段）。
- 机器人真机的上电序列、IP 配置、电池管理。

---

## 2. 系统架构与节点拓扑

### 2.1 扩展的包结构

```text
project/src/
├── p3at_description/                # 沿用
├── p3at_nav_plugins/                # 沿用
├── p3at_bringup/
│   ├── config/
│   │   ├── waypoints_gps.yaml       # 任务输入：地理航点
│   │   ├── mission_policy.yaml      # 航段策略表
│   │   ├── ekf_local.yaml           # 局部 EKF：odom+IMU → odom→base_link
│   │   ├── ekf_global.yaml          # 全局 EKF：odom+IMU+GPS → map→odom
│   │   ├── navsat_transform.yaml    # GPS ↔ map 的投影桥
│   │   ├── nav2_part2.yaml          # Part 2 调参后的 Nav2 参数基线
│   │   └── mission_bounds.yaml      # 任务安全围栏多边形
│   ├── launch/
│   │   ├── sim.launch.py            # 扩展：fake_gps + 双 EKF + navsat_transform
│   │   ├── real.launch.py           # 新：真机硬件驱动 + 双 EKF + navsat_transform
│   │   └── nav.launch.py            # 扩展：mission_orchestrator 替换 mission_manager
│   └── p3at_bringup/                # Python 节点
│       ├── mission_orchestrator.py  # 新：状态机
│       ├── waypoint_loader.py       # 新：YAML → 内部对象 + 校验 + 投影
│       ├── geo_utils.py             # 新：WGS84 ↔ ENU 工具
│       ├── arrival_judge.py         # 新：到达 / 通过侧几何判定
│       ├── goal_builder.py          # 新：waypoint → Nav2 goal（含 pass_side offset）
│       ├── mission_summary.py       # 新：事件聚合 → JSON / markdown
│       ├── policy_switcher.py       # 新：按 segment 切换 Nav2 / costmap 参数
│       ├── safety_node.py           # 新：手柄解释 + cmd_vel 仲裁 + estop
│       └── fake_gps_node.py         # 新（仅 sim）：ground-truth → /fix
└── p3at_localization/               # 新包：集中管理 GPS / EKF / navsat_transform
    ├── launch/localization.launch.py
    └── (无源码，组合 robot_localization 现成节点)
```

### 2.2 顶层数据流

```text
[ 硬件/仿真层 ]
  /fix         (NavSatFix, 5–10Hz)
  /imu         (Imu, 20Hz)
  /odom        (Odometry, 20Hz, 来自 odom_sanitizer)
  /scan /camera/image /camera/camera_info   (沿用 Part 1)

[ 定位层 p3at_localization ]
  ekf_local         ──► odom → base_link TF + /odometry/filtered/local
  navsat_transform  ──► /odometry/gps (gps in map frame)
  ekf_global        ──► map → odom TF   + /odometry/filtered/global

[ 视觉层 ]
  cone / shape detector ──► /perception/cone_pose,
                            /perception/shape_result,
                            /perception/marker_photo_done

[ 安全层 ]
  joy / bluetooth ──► safety_node ──► /safety/automated_enabled,
                                      /safety/deadman_pressed,
                                      /safety/estop
                                      (并对 cmd_vel 做前置仲裁)

[ 任务层 ]
  waypoint_loader ──► 航点列表（原始 GPS + map 投影）
                         │
                         ▼
  mission_orchestrator (状态机)
        │
        ├──► policy_switcher   ──► Nav2 / costmap 参数运行时覆盖
        ├──► goal_builder      ──► NavigateToPose goal (含 pass_side offset)
        ├──► arrival_judge     ──► /mission/arrived 事件
        ├──► mission_summary   ──► part2_mission_summary.{json,md}
        └──► /mission/state, /mission/event  （日志 + rviz）

[ 行为层 ] sh
  Nav2 (NavigateToPose / BT / planner / costmap / controller)  ──► /cmd_vel_nav
                         │
                         ▼（经 safety_node 仲裁）
                       /cmd_vel
```

### 2.3 关键解耦原则
1. **Orchestrator 不知道 GPS**：它接收的是 `map` 系 `(x, y)` goal，由 `waypoint_loader` 在启动时一次性投影。这让 Part 1 的所有 Nav2 组件零改动。
2. **Orchestrator 不写 cmd_vel**：所有速度指令都走 Nav2 → safety → `/cmd_vel`。Estop 只需掐断 safety_node 的输出，不涉及 orchestrator。
3. **Localization 不知道任务**：EKF 与 navsat_transform 对航点/状态机完全无感知，任务层崩溃不影响 TF。
4. **Sim/Real 同构**：sim launch 与 real launch 节点图在“定位层 / 视觉层 / 任务层 / 行为层”完全相同，只有“硬件/仿真层”节点不同。

---

## 3. 坐标系与 TF 拓扑

### 3.1 坐标帧清单
| 帧 | 含义 | 是否 TF 参与 |
|----|------|--------------|
| `map` | 任务局部 ENU 帧，原点在 datum `(lat₀, lon₀, alt₀)`，+x 指东，+y 指北，+z 指天 | 是 |
| `odom` | 连续的里程计帧，任务启动时与 base_link 对齐，之后不保证与 `map` 同步 | 是 |
| `base_link` | 机器人机械中心 | 是 |
| `imu_link` / `laser_frame` / `cam_optical_link` / 四轮 | 沿用 Part 1 | 是（robot_state_publisher） |
| `utm` | UTM 全局帧，仅用于跨区域讨论；**本项目不作为 TF 节点参与**（小场景单 datum 即可） | 否 |
| `earth` | ECEF 根帧；**本项目不使用** | 否 |

**关键决策**：因为 James Oval 场景 < 100m 尺度，不引入 `utm` / `earth` 作为 TF 节点，统一使用单一 `map` 为 ENU 根。`navsat_transform_node` 以“相对 datum 的 ENU”方式直接输出 `/odometry/gps`，不发布额外 TF。

### 3.2 TF 拓扑

```text
map ──► odom ──► base_link ──► { imu_link, laser_frame, cam_optical_link, wheel_* }
```

### 3.3 TF 唯一发布者表
| 边 | 唯一发布者 | 说明 |
|----|-----------|------|
| `map → odom` | `ekf_global`（robot_localization） | 融合 wheel + IMU + GPS 的漂移校正 |
| `odom → base_link` | `ekf_local`（robot_localization） | 只融合 wheel + IMU，保证短期连续 |
| `base_link → sensors/wheels` | `robot_state_publisher` | 沿用 Part 1 |

Part 1 的 `odom_sanitizer` 在 Part 2 **退化为只发布 `/odom` topic、不再发布 TF**；`slam_toolbox` 在 Part 2 默认不启用，调试模式启用时必须以 `publish_tf=false` 方式运行，仅用于可视化地图。

### 3.4 仿真与真机的 frame 差异
- 仿真中所有传感器 frame 与 Part 1 URDF 保持一致。
- 真机上需增加 `gps_link` 作为 GPS 天线相对 `base_link` 的偏移（在 URDF xacro 中以参数暴露）。`navsat_transform_node` 的 `gps_offset_on_base` 对应该偏移。

---

## 4. GPS 摄入（sim + real）

### 4.1 输入契约
| topic | type | 频率要求 | 质量要求 |
|-------|------|---------|---------|
| `/fix` | `sensor_msgs/NavSatFix` | ≥ 5Hz | `status.status ≥ STATUS_FIX`；`position_covariance_type ≠ COVARIANCE_TYPE_UNKNOWN`；水平方差 ≤ `gps.max_acceptable_horizontal_var`（默认 9 m²，即 3m std） |
| `/imu` | `sensor_msgs/Imu` | ≥ 20Hz | `orientation_covariance[0] ≥ 0` 表示 orientation 可用；否则 EKF 忽略 orientation，仅用角速度 |

### 4.2 仿真侧：`fake_gps_node`
**目的**：在 Gazebo 中跑通 GPS 路径，且与真机接口同构。

**输入**：机器人在 Gazebo 世界系下的真值位姿。采集方式优先级：
1. `/world/.../model/p3at/pose`（Gazebo Harmonic pose publisher）经 ros_gz_bridge 映射为 `/sim/ground_truth_pose`；
2. 若未桥接 ground truth，则退化为订阅 `/odometry/filtered/local`（假设仿真零打滑）。

**转换**：
- 读取 YAML 中的 `origin_lat / origin_lon / origin_alt`（仿真世界原点对应的地理点）。
- 用 ENU→WGS84 逆投影把 `(x, y, z)` 转换为 `(lat, lon, alt)`。
- 叠加高斯噪声 `N(0, σ²)`，默认 `σ_h = 1.5m`、`σ_v = 3.0m`，可配置。
- 填充 `status.status = STATUS_FIX`，`position_covariance = diag(σ_h², σ_h², σ_v²)`，`position_covariance_type = COVARIANCE_TYPE_DIAGONAL_KNOWN`。

**扩展能力**（用于故障注入测试）：
- 按时间表临时将 status 置为 `STATUS_NO_FIX` 或提升协方差，模拟信号丢失。
- 同时以 `/gps/sim_truth` 发布无噪声真值，仅用于事后回放对比。

### 4.3 真机侧
- GPS 驱动必须将 NMEA/UBX 统一转发为 `/fix`（推荐 `nmea_navsat_driver` 或厂商 ROS2 驱动）。
- 若 GPS 模块原生提供双天线航向，应同时发布 `/gps/heading`（`std_msgs/Float64` 或 `Imu`），供 navsat_transform 使用（见 §5）。
- 上电后最长 `gps.cold_fix_timeout`（默认 120s）内等待首帧有效 fix；超时则 orchestrator 保持 `WAITING_FOR_GPS` 并在 UI 显示超时。

### 4.4 GPS 健康度分级
`gps_health_monitor`（可作为 `safety_node` 的子模块或独立节点）维护一个枚举发布到 `/mission/gps_health`：
| 等级 | 触发条件 | Orchestrator 反应 |
|------|---------|-------------------|
| `HEALTHY` | 有 fix，horizontal std ≤ `warn_std`（默认 2.0m） | 正常 |
| `DEGRADED` | std ∈ (`warn_std`, `max_std`]，或短时（< `short_loss_seconds`，默认 5s）丢失 | 继续当前 segment，但禁止跨 segment 切换 |
| `LOST` | 丢失 ≥ `short_loss_seconds` 且 ≤ `hard_loss_seconds`（默认 30s） | 进入 `DEGRADED` 态，广播警告；若到达判定需要 GPS，则延后 |
| `FATAL` | 丢失 > `hard_loss_seconds` 或 std > `max_std`（默认 9m²） | 触发 `SAFE_STOP`；恢复后可回退到原状态 |

---

## 5. 初始航向获取

`navsat_transform_node` 把 `/fix` 投影到 `map` 系需要已知的“机器人当前朝向”。这是真机侧最常见的失败源，必须显式处理。

### 5.1 可选航向来源（按优先级）
1. **双天线 GPS / RTK heading**：若可用，直接用 `/gps/heading`。这是唯一免运动的可靠来源。
2. **带磁力计融合的 IMU orientation**（Phidget Spatial 的 AHRS 输出）：可用但**必须做磁偏角补偿**。James Oval 2026 磁偏角约为 -1.7° ~ -2.0°（具体值在任务启动时通过 NOAA 模型按 datum 重新计算一次，保存至 `navsat_transform.yaml.magnetic_declination`）。
3. **运动推断**：机器人在 `INIT → WAITING_FOR_GPS → HEADING_ACQUISITION` 阶段以 ≥ 0.2 m/s 直线前进 3–5m，由 `ekf_global` 根据 GPS 航迹推断初始航向，再锁定。适用于无磁力计或磁环境不可信的情况。
4. **人工初始化**：UI 中提供 `set_initial_yaw` 服务，允许操作员手动输入（例如机器人朝向已知地标时），仅在上面三种全部失败时使用。

### 5.2 航向获取状态（独立子状态）
- 进入 `HEADING_ACQUISITION` 前 orchestrator 必须确认 GPS `HEALTHY` 且 EKF 在 `odom` 帧稳定（`/odometry/filtered/local` 协方差收敛）。
- 航向置信度通过 `navsat_transform_node` 的 `yaw_offset_confidence`（或等价自定义指标）判定：连续 `heading.confirm_count`（默认 10 帧）高于阈值才认为可用。
- 置信度不足：orchestrator 回退到运动推断流程；运动推断失败 3 次后转 `ABORTED`。

### 5.3 冷启动 UX
- 启动后第一屏必须显示：GPS 状态 / 首帧 fix 等待 / 航向获取方法 / 当前 yaw 估计 / 置信度。
- 任何一项未绿灯，`WAITING_FOR_SAFETY` 状态无法通过（即用户按下 X 也不会进入 `PLANNING_NEXT`）。

---

## 6. 定位融合管线

### 6.1 双 EKF 模式（robot_localization 标准模式）
采用 robot_localization 提供的两实例 EKF + `navsat_transform_node` 组合：

- **ekf_local**：`world_frame: odom`，`publish_tf: true`
  - 输入：`/odom`（wheel）的 x/y/yaw + 协方差；`/imu` 的角速度（绕 z）+ 线加速度（x）。
  - 输出：`/odometry/filtered/local`（`odom` 帧），`odom → base_link` TF。
  - 作用：保证短期连续性、抗 GPS 抖动。
- **ekf_global**：`world_frame: map`，`publish_tf: true`
  - 输入：`/odom` 同上；`/imu` 的角速度；`/odometry/gps`（来自 navsat_transform）x/y 位置 + 协方差。
  - 输出：`/odometry/filtered/global`（`map` 帧），`map → odom` TF。
  - 作用：在 GPS 有效时修正 odom 漂移。
- **navsat_transform_node**：
  - 输入：`/fix`、`/imu`（或 `/gps/heading`）、`/odometry/filtered/global`。
  - 输出：`/odometry/gps`（`map` 帧）、`/gps/filtered`（经过滤的 GPS 位姿）。
  - 参数：`magnetic_declination_radians`、`yaw_offset`（若航向来自其他源）、`zero_altitude: true`（平面任务忽略高程以减少 EKF 抖动）、`broadcast_utm_transform: false`（不发 TF）、`publish_filtered_gps: true`。

### 6.2 启动时序
```text
t0  节点起步
t1  ekf_local 收到首帧 odom+imu，开始发布 odom→base_link
t2  /fix 首帧有效（status≥FIX、协方差达标）
t3  航向获取完成（§5）
t4  navsat_transform 完成 datum 锁定，开始发布 /odometry/gps
t5  ekf_global 收到首帧 /odometry/gps，开始发布 map→odom
t6  orchestrator 检测到 /mission/gps_health = HEALTHY 且 TF tree 完整
    → 允许进入 WAITING_FOR_SAFETY
```

orchestrator 的 `WAITING_FOR_GPS` 状态等价于 `not (t4 && t5 && t6)`。

### 6.3 关键参数建议值（初值，真机需现场调）
| 参数 | 推荐值 | 理由 |
|------|--------|------|
| `ekf_local.frequency` | 30 Hz | 高于 IMU 下游消费速率 |
| `ekf_global.frequency` | 10 Hz | 与 GPS 同阶即可，防止空跑 |
| `navsat_transform.magnetic_declination_radians` | 从 NOAA IGRF 按 datum 计算 | 不能写死 0 |
| `navsat_transform.yaw_offset` | 真机 GPS 天线朝向偏移，默认 0 | 安装校准决定 |
| `navsat_transform.zero_altitude` | `true` | 平面任务，减少 z 抖动 |
| `navsat_transform.wait_for_datum` | `false`（用首帧 fix）或 `true`（用 YAML datum） | 与 §8.2 datum 策略一致 |
| `ekf_global.process_noise_covariance[x,y]` | 稍大于 ekf_local | 允许 GPS 慢修正 |

### 6.4 Sim / Real 切换
- 仿真中：`ekf_global.process_noise` 可调小，因为 fake_gps 噪声可控。
- 真机中：必须先跑 `rosbag` 静态采集 10 分钟，估计 GPS 噪声谱，再设置协方差；这一步在 §17 测试计划里作为“真机第一日任务”。

---

## 7. GPS 健康度与降级策略（完整策略表）

| 情形 | orchestrator 反应 | Nav2 | 安全层 | 数据记录 |
|------|------------------|------|--------|----------|
| `HEALTHY` | 正常 | 正常 | 正常 | 按常态录制 |
| `DEGRADED`（短时抖动） | 维持当前状态，禁用 `AT_WAYPOINT → LEAVING` 跨 segment 切换 | 允许 | 允许 | `/mission/event` 写 `gps_degraded_start` |
| `LOST`（> 5s 丢失） | 进入 `DEGRADED` 态；若 5s 内需要做到达判定，推迟该判定 | 当前 goal 继续执行（odom 仍有效） | 允许 | 写 `gps_lost` |
| `FATAL`（> 30s 丢失 / 方差越限 / 坐标跳变 > `jump_threshold`） | 进入 `SAFE_STOP` | Cancel 所有活动 goal | `/cmd_vel = 0` 一次，safety 锁 autopilot | 写 `gps_fatal`；summary 标 `waypoint_failed` |
| 跳变但未越 FATAL 阈值 | 保持状态 | 继续 | 正常 | 写 `gps_jump_<Δm>m`，供事后分析 |
| datum 被误触重置请求 | 拒绝，记录 `datum_override_rejected` | — | — | — |

`jump_threshold` 默认 `max(3·current_std, 5m)`，确保噪声分布的 3σ 不被误判。

---

## 8. Waypoint 数据模型与加载

### 8.1 `waypoints_gps.yaml` 结构（字段规范，不贴代码）
顶层字段：
- `mission_id`：字符串，单次任务唯一标识（也用于输出文件名）。
- `datum`（可选）：`{lat, lon, alt}`。若提供，orchestrator 启动时将 `navsat_transform` 的 datum 硬锁到此值，保证多次运行坐标可比；若缺省，则用首帧有效 fix。
- `origin_lat / origin_lon / origin_alt`：仿真世界原点对应的地理点，fake_gps 反投影使用。真机 launch 可忽略。
- `home`（可选）：`{lat, lon, yaw_hint?}`。缺省时，orchestrator 启动时将当前 `/gps/filtered` 位置作为 home。
- `mission_bounds`：经纬度多边形（≥ 3 个点），loader 校验所有航点与 home 必须在多边形内。
- `return_home`：bool，默认 true。
- `waypoints`：有序列表。

每个 `waypoints[i]` 字段：
- `id`：字符串，例 `"WP1"`，summary / 日志 / photo 文件名都用它。
- `lat`, `lon`：必填。
- `arrival_radius`：可选，默认 1.5m，约束范围 [1.0, 2.0]。
- `pass_side`：枚举 `right` / `left` / `none`，默认按任务类型；marker 型航点默认 `right`。
- `yaw_hint`：可选，作为 `goal_builder` 构造通过姿态时的“优先朝向”；缺省时由“前一航点→本航点”的 bearing 推算。
- `policy`：引用 `mission_policy.yaml` 中的 id；缺省 `default`。
- `photo_required`：bool，默认 true（marker 必拍）。
- `expected_secondary_object`：`{color?, shape_prior?}`，缺省为空。视觉子系统作为先验。
- `notes`：自由文本，仅用于 summary 展示。

### 8.2 Datum 选择策略
- **Sim**：推荐 YAML 显式指定 datum（= origin），保证 fake_gps 与 navsat_transform 完全对齐，便于单元测试。
- **Real**：推荐 YAML 显式指定 datum 为“场地西南角 GPS 测点”，防止每次首帧偏移导致跨次轨迹不可比。
- 缺省：退化到首帧 fix，同时在 summary 中记录实际 datum。

### 8.3 加载器职责（`waypoint_loader`）
1. **语义校验**：字段齐全、枚举合法、经纬度范围、arrival_radius 范围。
2. **地理合理性**：相邻航点大圆距离不超过 `max_segment_distance`（默认 200m）；所有航点在 `mission_bounds` 内。
3. **唯一性校验**：`id` 不重复。
4. **策略引用校验**：每个 `policy` 在 `mission_policy.yaml` 中存在。
5. **投影**：对每个航点调用 `geo_utils` 将 `(lat, lon)` 投影到 `map` 系，保存双份（原始 GPS + `map` 局部）。`geo_utils` 内部使用 ENU 切平面近似（< 1km 场景足够，误差 < 1mm）。
6. **隐式 home 追加**：若 `return_home=true`，在列表尾追加 `WP_HOME`（`id="HOME_RETURN"`, `pass_side=none`, `photo_required=false`, `arrival_radius=1.0`）。
7. **失败模式**：任一校验失败 → 打印精确错误（字段名 + 期望 + 实际）并 fail-fast，orchestrator 不进入 `INIT` 之后的状态。

### 8.4 运行时不可变性
- 航点列表在 orchestrator 启动后**不可修改**。若需要修改，必须完全重启任务。
- 未来 Part 3 的“动态追加探索目标”需求通过**独立 topic（`/exploration/goal`）**实现，不复用 waypoint YAML。

---

## 9. Mission Orchestrator 状态机

### 9.1 状态清单
| 状态 | 含义 | 终态条件 |
|------|------|---------|
| `INIT` | 加载参数、订阅建立 | 参数加载成功 |
| `WAITING_FOR_GPS` | 等首帧 fix + datum + 航向 | GPS_HEALTHY 且 TF 完整 |
| `HEADING_ACQUISITION` | 特殊态，若需运动推断航向（见 §5） | 航向置信度达标 |
| `WAITING_FOR_SAFETY` | 等 X + dead-man | `/safety/automated_enabled=true` 且 `deadman_pressed=true` |
| `PLANNING_NEXT` | 应用 segment policy，构造 goal | Nav2 accept |
| `NAVIGATING` | 执行单航点导航 | 进入 arrival_radius / Nav2 failed / estop |
| `ARRIVING` | 已进入航点半径，发拍照请求、等几何确认 | photo_done + pass_side_verified |
| `AT_WAYPOINT` | 到点后处理：等 shape 识别（带超时） | shape_result OR timeout |
| `LEAVING` | 驶离本航点，缓冲圈确认 | 与 waypoint 距离 > `arrival_radius + buffer` |
| `RETURNING_HOME` | 执行 HOME_RETURN 航点 | 与 home 距离 ≤ 1m |
| `DEGRADED` | GPS / 视觉子健康，但未终止 | 子系统恢复 → 回退上一个非 DEGRADED 状态 |
| `SAFE_STOP` | 强制停车，等待恢复 | 安全与 GPS 恢复 → 回退 |
| `COMPLETED` | 全部完成 | 写 summary |
| `ABORTED` | 不可恢复错误 | 写 summary（部分） |

### 9.2 转移表（核心路径）
| From | 触发 | To | Guard | Side effect |
|------|------|----|-------|-------------|
| INIT | params_loaded | WAITING_FOR_GPS | - | 订阅 /fix /imu /odom |
| WAITING_FOR_GPS | gps_health=HEALTHY ∧ tf_ready | HEADING_ACQUISITION | 需要运动推断时 | 发起直线前进序列 |
| WAITING_FOR_GPS | 同上 | WAITING_FOR_SAFETY | 航向来源为 GPS/IMU | - |
| HEADING_ACQUISITION | heading_confidence_ok | WAITING_FOR_SAFETY | - | 停车 |
| WAITING_FOR_SAFETY | automated=true ∧ deadman=true | PLANNING_NEXT | 有下一航点 | idx=0 |
| PLANNING_NEXT | nav2_accept_goal | NAVIGATING | - | 应用 policy / 构造 goal |
| NAVIGATING | dist_to_wp ≤ arrival_radius for N frames | ARRIVING | gps_health ≥ DEGRADED | 发 photo_request |
| NAVIGATING | Nav2 result = failed | (recovery branch, §13) | - | - |
| ARRIVING | photo_done ∧ pass_side_ok | AT_WAYPOINT | - | 记录 arrival_pose |
| ARRIVING | pass_side_violated | (recovery branch, §13) | - | - |
| AT_WAYPOINT | shape_result OR timeout | LEAVING | - | 保存 shape/photo 到 summary |
| LEAVING | dist_to_wp > arrival_radius + buffer | PLANNING_NEXT | 有下一航点 | 恢复 policy |
| LEAVING | 同上 | RETURNING_HOME | 无下一航点且 return_home | - |
| LEAVING | 同上 | COMPLETED | 无下一航点且 ¬return_home | - |
| RETURNING_HOME | dist_to_home ≤ 1m | COMPLETED | - | - |
| 任意非终态 | safety_violation | SAFE_STOP | - | cancel Nav2 goal |
| SAFE_STOP | safety_restored | 原状态 | 保持 stack | - |
| 任意 | gps_fatal / unrecoverable_nav_failure | ABORTED | - | 写 summary |

### 9.3 状态发布
- `/mission/state`：`String` 或自定义 msg，1Hz 心跳，状态变化时立即触发。
- `/mission/event`：关键事件（arrival / photo / degraded / failed / segment_switch / datum_set），带时间戳、waypoint id、data 字段。
- 两个 topic 都被 rosbag 录制，构成 summary 的事实来源。

### 9.4 arrival_radius 与 GPS 协方差的自适应
如果 `current_gps_horizontal_std ≥ 0.8 × arrival_radius`，单凭 GPS 距离难以可靠触发 `ARRIVING`。此时 orchestrator 必须满足下述**任一**附加条件才允许进入 `ARRIVING`：
- 视觉已确认 cone pose 且置信度 ≥ 阈值；
- 机器人速度已降至 < 0.15 m/s 并保持 ≥ 1.5s（Nav2 认为已到 goal）；

否则 orchestrator 保持 `NAVIGATING`，即便 GPS 显示已在半径内。

### 9.5 拍照握手
- 进入 `ARRIVING` 瞬间发布 `/perception/marker_photo_request`（含 waypoint id、当前位姿）。
- Orchestrator 等待 `/perception/marker_photo_done`（含 photo 路径、success bool、timestamp）最多 `photo_timeout`（默认 5s）。
- 超时处理：记录 `photo_missing`，不阻塞状态推进；summary 中该航点 `marker_photo_path = null`。
- 拍照期间保持机器人软停车（Nav2 goal 已到达，`/cmd_vel = 0`）。Dead-man 释放会中断拍照并转 `SAFE_STOP`。

---

## 10. 航段策略与运行时参数切换

### 10.1 为什么分段
| 航段 | 典型需求 | 策略 id |
|------|---------|---------|
| home → WP1 | 常规导航，GPS 主导 | `default` |
| WP1 → WP2 | 需 weave through cones，窄缝通过 | `weave_through_cones` |
| WPi → WPi+1 (i≥2) | 常规 + pass-side offset | `default_marker` |
| WP_last → home | 常规，容差宽松 | `final_return` |

`waypoints[i].policy` 指定“**到达** waypoint i 所使用的 segment 策略”，即“**from previous to waypoints[i]**”。

### 10.2 `mission_policy.yaml` 字段
每条 policy：
- `id`
- `controller_overrides`：`P3ATController` 可动态调参字段，例如 `lookahead_distance`、`max_linear_speed`、`slowdown_distance`、`collision_lookahead_time`、`goal_dist_tolerance`、`approach_slow_factor`。
- `local_costmap_overrides`：`inflation_layer.inflation_radius`、`inflation_layer.cost_scaling_factor`、`obstacle_layer.obstacle_max_range`。
- `global_costmap_overrides`（可选）：一般不动。
- `arrival_overrides`：`radius`（覆盖 YAML 默认值）、`confirm_frames`（默认 3）。
- `goal_offset`：`{lateral: m, longitudinal: m}`，构造 goal 时相对 cone 的偏移（见 §11.2）。
- `replan_on_entry`：bool，进入 segment 时是否强制 planner 重新规划。

### 10.3 运行时参数切换机制（`policy_switcher`）
1. 启动时把所有 policy 的基线值快照一份，以便恢复。
2. 进入 `PLANNING_NEXT` 时，计算 “将要应用的差量” = policy_overrides - baseline。
3. 通过 ROS2 参数服务把差量写入 `controller_server`、`local_costmap` 等节点。**必须验证**这些参数是 `rclcpp::ParameterDescriptor::dynamic_typing` / non-read_only 的；在 Nav2 当前版本，controller 插件的主要参数与 `inflation_layer.inflation_radius` 都可动态修改，costmap plugins 列表本身不可动态修改（因此 policy **不允许**改变 costmap plugins 列表，只能改参数）。
4. 写入失败 → 记录 `policy_apply_failed` 事件，回退基线，状态切到 `ABORTED`（fail-closed）。
5. 离开 segment（`LEAVING → PLANNING_NEXT` 边界）时恢复 baseline，再写入下一 policy 的差量。

### 10.4 Weave-through-cones 的具体策略
- `lookahead_distance: 0.45m`（短前视，允许锐转）。
- `max_linear_speed: 0.25m/s`。
- `collision_lookahead_time: 1.3s`。
- `inflation_radius: 0.35m`（小于默认，允许更靠近 cone）、`cost_scaling_factor` 提升以保持梯度。
- `replan_on_entry: true`，进入时强制 planner 重规划，避免使用带旧 inflation 的缓存。
- 不硬编码中间 sub-goals：依赖 costmap 让 planner 自然选择绕行路径。如果现场实测发现 planner 倾向于“绕整个 cone 阵列外侧”而不是真正“穿插”，则退化到“按 yaw_hint 手工插入 1–2 个中间航点”（在 YAML 中作为普通 waypoint，pass_side=none）。**这一退化方案在文档级别预留，不在 v2 默认实现**。

---

## 11. 到达判定与 “cone-on-right” 通过几何

### 11.1 到达判定（`arrival_judge`）
- 输入：`/odometry/filtered/global`（`map` 帧）+ 当前 waypoint 的 map 坐标 + 当前 waypoint 的 `arrival_radius` + `gps_health`。
- **严格区分 `arrival` 与 `goal`**：`arrival_judge` 的度量目标是**真实 cone 位置**（即 waypoint 本身），不是 `goal_builder` 构造的偏移后 goal。否则“设 offset 之后机器人到达了 offset goal 但其实离 cone 很远”时会被误判。
- 判据：
  - 主：`dist(robot, waypoint) ≤ arrival_radius`。
  - 辅：连续 `confirm_frames` 成立（默认 3，约 0.3s @10Hz）。
  - 辅：§9.4 的 GPS 协方差自适应条件。
- 输出：`/mission/arrived` 事件（`{waypoint_id, t, pose, source="gps|vision|nav2"}`，表明是哪类信号首先达成）。

### 11.2 “cone-on-right” 几何模型

定义：
- `C` = waypoint 在 `map` 系的 (x, y)（= cone 真实位置的最佳估计）。
- `θ_in` = 进入方向，定义为“前一航点 → 本航点 的 bearing”；若没有前航点（即 WP1），则退化为“当前机器人位置 → 本航点”。
- `pass_side ∈ {right, left, none}`，来自 YAML。
- `lat = policy.goal_offset.lateral`（默认 = arrival_radius）。
- `lon = policy.goal_offset.longitudinal`（默认 = arrival_radius，即 goal 设在 cone “过去一点”）。

`goal_builder` 输出：
- `G.x = C.x + lon·cos(θ_in) + lat·sin(θ_in)·s`
- `G.y = C.y + lon·sin(θ_in) − lat·cos(θ_in)·s`
- `G.yaw = θ_in`
- 其中 `s = +1 if pass_side=right`，`−1 if left`，`0 if none`（同时 lat 也置 0）。

几何含义：
- `lon·(cos, sin)` 将 goal 推到 cone **正前方**（沿进入方向），保证机器人到达后已经“越过”cone。
- `lat·(sin, −cos)` 在进入方向的**左侧**偏移（因为 `+x→+y` 的 90° 旋转向量为 `(-sin, cos)`，负该向量即右侧）；这里当 `s=+1` 时向进入方向的左侧偏，这样机器人到达 goal 时 cone 在其右侧。

> 符号约定再次确认：若 `θ_in = 0`（朝东），`s=+1`（pass_side=right），则 `G = C + (lon, -lat)` — 即 goal 在 cone 的南侧（向南是东向前进的右侧），这与“cone 在机器人右侧”一致 ❌ 错误。
>
> 修正：若朝东 θ_in=0 前进，机器人右侧是南（y 减小），因此若要“cone 在机器人右侧”，goal 应在 cone 的**北侧**，即 `y` 方向 + 偏移。因此 lat 项的符号应为 `s·(+)·cos, s·(+)·sin 旋转 90° 逆时针`：即 `(-sin, cos)`。重算：
>
> - `G.x = C.x + lon·cos(θ_in) − lat·sin(θ_in)·s`
> - `G.y = C.y + lon·sin(θ_in) + lat·cos(θ_in)·s`
>
> 复核：`θ_in=0, s=+1`：`G = C + (lon, +lat)` → goal 在 cone 北侧 → 机器人在 goal 处面朝东时，cone 在其南侧 = 右侧。✓
>
> `θ_in = π/2` 朝北，`s=+1`：`G = C + (0, 0) + (−lat, 0)` → goal 在 cone 西侧 → 机器人面朝北时右侧是东 → cone 在机器人东侧 = 右侧。✓
>
> 最终公式以此为准。

### 11.3 Pass-side 验证（`pass_side_verified`）
- 在 `ARRIVING` 期间采集机器人近 `window_seconds`（默认 2s）内的 `map` 系位置轨迹。
- 对每个采样点计算其相对 cone 的侧位：`cross = (C.x - p.x)·sin(p.yaw) − (C.y - p.y)·cos(p.yaw)`；`cross > 0` 代表 cone 在机器人左侧，`< 0` 代表右侧（推导略）。
- 若 `pass_side=right`：要求 `cross < 0` 在 ≥ 80% 样本上成立。
- 若 `pass_side=left`：要求 `cross > 0` 在 ≥ 80% 样本上成立。
- 若 `pass_side=none`：跳过验证。
- 不满足：发 `pass_side_violated` 事件，进入 §13 的“几何违约恢复”分支。

### 11.4 视觉修正（GPS→Vision 覆盖）
- 当 orchestrator 处于 `NAVIGATING` 且距离当前 waypoint ≤ `vision_override_trigger_radius`（默认 6m）时，开始消费 `/perception/cone_pose`。
- 覆盖条件：
  - 视觉 confidence ≥ `vision_confidence_threshold`（默认 0.7）。
  - 视觉位置与 YAML 投影位置距离 ≤ `vision_confirm_distance`（默认 4m，防误识别）。
  - 连续 `vision_persist_frames`（默认 5）满足。
- 覆盖动作：
  - 更新 `arrival_judge` 的 waypoint 参考位置为视觉位置（只影响到达判定与 pass-side 判定）。
  - 更新 `goal_builder` 的 `C`，重新计算 goal；通过 `NavigateToPose` 的 `update_goal` 机制（或 cancel+resend）重发。
  - 在 `/mission/event` 写 `vision_override{offset_m=...}`。
- 不覆盖 datum、不覆盖 YAML；所有覆盖只是运行时内存值，任务重启即回落到原始 GPS 值。

### 11.5 Costmap 与 offset 兼容性
- 确保：`goal_offset.lateral² + goal_offset.longitudinal² >  inflation_radius²`，否则 Nav2 会把 goal 判为 “in_collision”，规划失败。
- `policy_switcher` 在应用 policy 时必须先校验该不等式，失败则拒绝应用并回退 `ABORTED`。

---

## 12. 接口契约

以下表即本模块对外的**全部**接口。任何内部细节变化不得破坏此接口。

### 12.1 订阅 topic
| Topic | Type | Provider | 消费者 | 说明 |
|-------|------|----------|-------|------|
| `/fix` | NavSatFix | fake_gps / 真 GPS 驱动 | navsat_transform, gps_health_monitor | 必备 |
| `/imu` | Imu | Part 1 桥接 / Phidget | ekf_*, navsat_transform | 必备 |
| `/odom` | Odometry | odom_sanitizer | ekf_local, ekf_global | 必备 |
| `/gps/heading` | Float64 或 Imu | 双天线 GPS（可选） | navsat_transform | 可选 |
| `/perception/cone_pose` | geometry_msgs/PoseWithCovarianceStamped | 视觉 | orchestrator, arrival_judge | - |
| `/perception/shape_result` | 自定义 msg: `{shape, photo_path, distance_to_marker, confidence}` | 视觉 | orchestrator | - |
| `/perception/marker_photo_done` | 自定义 msg: `{waypoint_id, photo_path, success, t}` | 视觉 | orchestrator | 对应 request |
| `/safety/automated_enabled` | Bool | safety_node | orchestrator | - |
| `/safety/deadman_pressed` | Bool | safety_node | orchestrator | - |
| `/safety/estop` | Bool | safety_node | orchestrator | 单向 latch |

### 12.2 发布 topic
| Topic | Type | Provider | 说明 |
|-------|------|----------|------|
| `/mission/state` | String | orchestrator | 1Hz 心跳 + 变化即发 |
| `/mission/event` | 自定义: `{t, type, waypoint_id?, data}` | orchestrator | 事件流 |
| `/mission/arrived` | PoseStamped | arrival_judge | 到达事件 |
| `/mission/current_waypoint` | 自定义: `{id, lat, lon, x, y, pass_side, policy}` | orchestrator | rviz 与 UI |
| `/mission/gps_health` | String 枚举 | gps_health_monitor | `HEALTHY/DEGRADED/LOST/FATAL` |
| `/perception/marker_photo_request` | 自定义: `{waypoint_id, pose}` | orchestrator | 拍照触发 |
| `/fake_gps/sim_truth` | NavSatFix | fake_gps（仅 sim） | 无噪声真值 |
| `/viz/mission_markers` | MarkerArray | orchestrator | rviz 可视化 |

### 12.3 Action
| Action | 客户端 | 服务端 | 说明 |
|--------|-------|-------|------|
| `navigate_to_pose` | orchestrator | bt_navigator | 每个 waypoint 发一次 |

### 12.4 Service
| Service | Provider | 用途 |
|---------|----------|------|
| `/mission/pause` | orchestrator | 软暂停，进入 SAFE_STOP |
| `/mission/resume` | orchestrator | 从 SAFE_STOP 回退 |
| `/mission/abort` | orchestrator | 显式终止，写 summary |
| `/mission/set_initial_yaw` | orchestrator | 航向人工初始化（§5.1.4） |
| `/mission/override_datum` | orchestrator | 仅调试启用，默认拒绝 |

### 12.5 参数（运行时可写）
| 节点 | 参数 | 允许运行时改 | 说明 |
|------|------|-------------|------|
| controller_server | FollowPath.* | 是 | policy_switcher 使用 |
| local_costmap | inflation_layer.* | 是 | policy_switcher 使用 |
| local_costmap | plugin list | 否 | 不允许 policy 改 |
| navsat_transform | magnetic_declination_radians, yaw_offset | 启动时 | 不在任务中途改 |
| ekf_* | 大部分协方差矩阵 | 启动时 | 不在任务中途改 |

---

## 13. 失败处理与恢复策略

### 13.1 失败分类矩阵
| 类别 | 失败事件 | 检测者 | 首次处理 | 升级 |
|------|---------|-------|---------|------|
| 硬件感知 | IMU 掉线 ≥ 2s | ekf 诊断 | 进入 SAFE_STOP | 持续 10s → ABORTED |
| GPS | `FATAL` 等级 | gps_health_monitor | SAFE_STOP | 回落再尝试 1 次，仍失败 → ABORTED |
| 导航 | Nav2 action result = `FAILED`（BT recovery 已耗尽） | orchestrator | 对当前 waypoint **重试** 1 次，strategy = `replan_with_cleared_costmap` | 第二次仍失败：该 waypoint 记 `failed_waypoint`，summary 标记，**跳过**进入下一 waypoint（除非是 home，home 失败 → ABORTED） |
| 导航 | Nav2 反复进入 recovery（> `recovery_budget_per_segment`，默认 3 次 spin/backup） | orchestrator 监听 BT recovery topic | 强制 cancel + 重试 | 同上 |
| 导航 | goal 被 costmap 判 in_collision | orchestrator（goal rejection） | 根据 `goal_offset` 自动缩放 20%，重试 1 次 | 2 次缩放失败 → failed_waypoint |
| 视觉 | 拍照超时 | orchestrator | 标 `photo_missing`，继续 | 不升级 |
| 视觉 | shape 识别超时 | orchestrator | 标 `shape_missing`，继续 | 不升级 |
| 几何 | `pass_side_violated` | arrival_judge | 执行“回绕”小机动：在 cone 周围增加一个临时 subgoal，在相反侧绕回，pass_side 再验证 1 次 | 失败 → 标 `pass_side_violated`，继续 |
| 安全 | Dead-man 释放 | safety_node | SAFE_STOP | — |
| 安全 | Estop latch | safety_node | SAFE_STOP + 锁定 | 仅人工 reset |

### 13.2 跳过 waypoint 的语义
“跳过”只是在 summary 上标记该点 `status=failed`，机器人物理上仍前往下一个 waypoint；home 永远不跳过。

### 13.3 恢复时的状态回退栈
- `SAFE_STOP` 保留进入时的上一个状态指针 `prev_state`。
- 恢复条件全部满足后回退到 `prev_state`（不跨过 PLANNING_NEXT）。
- 若 `prev_state = NAVIGATING` 且 Nav2 goal 已被 cancel，恢复时需要重新下发 goal（走一次 PLANNING_NEXT→NAVIGATING）。

---

## 14. Mission Summary 与数据记录

### 14.1 JSON Schema
每条 waypoint 记录字段（所有字段非 null 即有语义）：
- `id`, `index`
- `status`: `completed | failed | skipped`
- `target_gps`: `{lat, lon, alt?}`
- `target_map`: `{x, y}`
- `arrival_pose_map`: `{x, y, yaw}`
- `arrival_pose_gps`: `{lat, lon, alt?}`
- `arrival_time`, `leave_time`, `dwell_seconds`
- `pass_side_expected`, `pass_side_actual`, `pass_side_verified` (bool)
- `vision_override_applied` (bool), `vision_override_offset_m` (float, 可空)
- `marker_photo_path` (path, 可空)
- `secondary_object`: `{shape, color, photo_path, distance_to_marker, confidence}` 可为 null
- `gps_events`: 该 waypoint 期间的 `gps_degraded / lost / jump` 事件列表
- `nav_events`: `replans`, `recoveries`, `goal_in_collision_retries` 计数
- `notes`: 自由文本

顶层字段：
- `mission_id`, `started_at`, `ended_at`, `total_seconds`
- `datum`: `{lat, lon, alt, source: "yaml|first_fix"}`
- `mission_outcome`: `completed | completed_with_failures | aborted`
- `waypoints`: 列表
- `gps_stats`: `{mean_std_h, max_std_h, total_lost_seconds}`

### 14.2 输出布局
所有输出放在 `output_dir`（orchestrator 参数，默认 `${workspace}/part2_runs/${mission_id}/`，**绝对路径**，避免 cwd 陷阱）：
```text
part2_runs/<mission_id>/
  mission_summary.json
  mission_summary.md
  driven_path.csv
  rosbag/ (rosbag2 目录)
  photos/
    <waypoint_id>_marker.jpg
    <waypoint_id>_secondary.jpg
```

### 14.3 rosbag2 topic 列表（在 Part 1 基础上扩展）
新增：`/fix`, `/gps/filtered`, `/odometry/gps`, `/odometry/filtered/local`, `/odometry/filtered/global`, `/mission/state`, `/mission/event`, `/mission/arrived`, `/mission/gps_health`, `/mission/current_waypoint`, `/perception/*`, `/safety/*`, `/viz/mission_markers`。

### 14.4 流式写出
- 每个 waypoint 离开即追加写入 JSON（原子 rename 避免半写）。
- markdown 在 `COMPLETED` / `ABORTED` 时一次性渲染。
- 崩溃 recovery：节点重启时能从部分写入的 JSON 恢复上下文供人工检查（但不支持自动续跑）。

---

## 15. Launch 组成

### 15.1 启动参数（统一）
| 参数 | 默认 | 说明 |
|------|------|------|
| `localization_mode` | `gps` | `gps` / `slam_debug` / `gps_with_slam_view` |
| `waypoints_file` | `config/waypoints_gps.yaml` | 绝对路径或相对 share |
| `policy_file` | `config/mission_policy.yaml` | 同上 |
| `bounds_file` | `config/mission_bounds.yaml` | 同上 |
| `record` | `false` | 触发 rosbag2 |
| `output_dir` | `${workspace}/part2_runs/${now}` | 绝对路径 |
| `use_sim_time` | sim=true / real=false | 由各 launch 硬设 |
| `autostart` | `false` | 若 true，orchestrator 启动后直接进入 WAITING_FOR_SAFETY；否则停在 INIT 等 service 触发 |

### 15.2 `sim.launch.py`（扩展）
节点：Part 1 sim 全部 + `fake_gps_node` + `p3at_localization` launch + `safety_node`（可选 headless 模式，测试时自动 set automated=true）。

### 15.3 `real.launch.py`（新）
节点：硬件驱动（GPS / IMU / Lidar / Camera / Pioneer base）+ `p3at_localization` launch + Nav2 + orchestrator + safety_node + 视觉节点。**绝不**启动 Gazebo / fake_gps。

### 15.4 `nav.launch.py`
加载 `nav2_part2.yaml` 基线（覆盖 Part 1），并启动 orchestrator、arrival_judge、policy_switcher、mission_summary、waypoint_loader 等任务层节点。默认不启动 slam_toolbox。

### 15.5 Sim / Real 镜像校验
提供一个 `check_parity.sh`（非代码交付，仅文档约定）：对比两条 launch 的 `ros2 node list` 差集，必须 ⊆ `{gazebo, bridge, fake_gps, hardware_drivers}`。

---

## 16. 可视化与操作员 UX

### 16.1 RViz Marker 清单
- 所有 waypoints：球体 + 文本标签（id），颜色按 status（未到 / 当前 / 已完成 / 失败）。
- Pass-side offset 的 goal 点：小箭头。
- `mission_bounds` 多边形：线框。
- 当前 waypoint 的 `arrival_radius` 圆圈：半透明填充。
- 视觉 cone pose（如果有）：不同颜色球体。
- GPS 历史位置 `/gps/filtered` 折线。

### 16.2 文本 UI（由 Part 3 延伸，Part 2 仅占位）
- Part 2 阶段，HUD 只需要在终端日志定期打印（5s 一次）：当前状态、当前 waypoint、到目标距离、GPS 健康度、安全层布尔。
- Part 3 需要触摸屏图形 UI，本文档不展开。

---

## 17. 测试与验证

### 17.1 测试等级
- **L1 单元**：`geo_utils`（ENU↔WGS84）、`goal_builder`（pass-side 几何）、`arrival_judge`（含噪声注入）。
- **L2 子系统**：定位栈（fake_gps → EKF → TF）；orchestrator 状态机（用 mock topics 驱动）。
- **L3 仿真集成**：`sim.launch.py` 全栈，多场景 YAML。
- **L4 真机**：按 §17.6 清单分阶段上场。

### 17.2 L1 关键单测 pass 条件
| 测试 | 输入 | 期望 |
|------|------|------|
| ENU↔WGS84 往返 | 100 个随机点，r ≤ 500m | 误差 ≤ 1mm |
| goal_builder pass_side=right, θ_in ∈ {0, π/2, π, -π/2} | 单 cone (0,0) | goal 与手算一致（见 §11.2 复核表） |
| arrival_judge 噪声 | 机器人距 waypoint 1.3m 但 GPS std=2.0 | 不触发 arrival（触发 §9.4 守护） |
| arrival_judge confirm_frames | 2 帧 in / 1 帧 out / 2 帧 in | 不触发（连续性） |

### 17.3 L2 状态机干跑
- 用 rosbag 回放预录的 `/fix /imu /odom /perception/* /safety/*` 序列，驱动 orchestrator 运行，断言：
  - 状态转移序列与预期文本匹配；
  - 所有 `/mission/event` 字段齐全；
  - summary JSON schema 合法。

### 17.4 L3 场景集
| 场景 | YAML | 期望 |
|------|------|------|
| S1 单航点回家 | 1 wp + home | 完成、summary 2 项、home 距离 ≤ 1m |
| S2 5 航点全 right | 5 wp | 所有 pass_side_verified=true |
| S3 WP1→WP2 weave | 中间插 6 个 cones（作为世界障碍） | weave 段完成，无碰撞 |
| S4 GPS 抖动 | 注入 6s `NO_FIX` 一次 | 仅 DEGRADED，任务继续 |
| S5 GPS fatal | 注入 35s 丢失 | SAFE_STOP，恢复后续跑成功 |
| S6 视觉覆盖 | YAML cone 位置故意偏 3m，视觉正确 | vision_override 被应用，pass_side 通过 |
| S7 goal in collision | policy.goal_offset 设为 0.1m，inflation 0.5m | 触发自动缩放重试或 failed_waypoint |
| S8 中途 dead-man | 每 30s 按一次“释放” | SAFE_STOP + RESUME 均正常 |
| S9 pass_side 违规 | 强制从 cone 的右侧通过 | `pass_side_violated` 事件 + 绕回验证 |
| S10 返回 home 失败 | mission_bounds 不含 home | loader fail-fast |

### 17.5 性能指标
- 端到端（5 wp + home）完成时间 ≤ 仿真 8 分钟。
- `/mission/state` 发布 jitter < 50ms。
- summary.json 写入延迟 < 100ms / waypoint。

### 17.6 真机上场分阶段
1. **Day 1：静态 GPS**：机器人不动，采 rosbag 10 分钟，估计 GPS 噪声谱；核对 magnetic_declination。
2. **Day 2：直线 heading**：关 autopilot，只跑 HEADING_ACQUISITION 运动推断，对比 IMU/磁/GPS 三源。
3. **Day 3：单航点 manual approach**：手动驾驶靠近 WP1，观察 arrival_judge 是否正确触发。
4. **Day 4：单航点 auto + 无障碍**：开 autopilot，WP1→home。
5. **Day 5：完整 5 航点** 按仿真同一 YAML 执行。
6. **Day 6：weave 段** 在真实 cones 场地完整重跑。

---

## 18. 风险与开放问题

| 风险 | 影响 | 概率 | 缓解 |
|------|------|------|------|
| James Oval 附近建筑多径致 GPS 跳变 | Summary 不可靠 / 跨航段误触 | 中 | §4.4 健康分级 + §11.4 视觉覆盖 |
| IMU 磁场受机器人电机干扰 | 磁航向错 5–15° | 高 | §5.1.3 运动推断优先于磁航向 |
| Nav2 在 weave 段选“绕外侧”而非穿插 | 看起来没 weave | 中 | §10.4 预留手工 subgoal 退化 |
| 视觉误把装饰 cone 识别为 marker | 错误覆盖 goal | 中 | §11.4 距离阈 4m + 置信度门限 |
| EKF 协方差默认值致 GPS 修正过慢或过激 | 轨迹震荡或漂移 | 中 | §17.6 Day1 先采 bag 再调 |
| Nav2 controller 参数 `goal_dist_tolerance` 与 YAML `arrival_radius` 冲突 | Nav2 认为未到 goal，orchestrator 认为已到 | 中 | policy_switcher 同步写 controller.goal_dist_tolerance = YAML.arrival_radius·0.8 |
| 仿真中 fake_gps 与 EKF 的 z 高度扰动 | map→odom 抖动 | 低 | `zero_altitude: true` |
| Policy 切换时 costmap plugin 列表错误地被尝试修改 | 参数写入失败 | 低 | §10.3 校验：只允许改值不改 plugin 列表 |
| Summary 写入在崩溃中损坏 | 数据丢失 | 低 | §14.4 原子 rename |
| home 点恰好在障碍物 inflation 内 | 返程永远到不了 | 中 | §8.3 校验 `home ∉ inflated_obstacles`（启动时做 costmap 抽样） |

开放问题（留给团队实施时决策）：
- 是否采购或租用一台 RTK 基站以把 GPS std 降到 < 0.3m。若能，`arrival_radius` 可直接收紧到 1.0m。
- 真机摄像头帧率是否够用于 6m 距离识别 cone（影响 §11.4 的覆盖时机）。
- 失败 waypoint 是否应该在当天第二轮补跑（流程问题，非算法）。

---

## 19. 实现里程碑

| 里程碑 | 交付物 | 退出条件 |
|-------|--------|---------|
| M1 定位栈 | ekf_local + ekf_global + navsat_transform + fake_gps；`sim.launch.py` 可启动并看到 map→odom | §17.4 S1 跑通 |
| M2 Waypoint 加载 | waypoint_loader + geo_utils + bounds 校验 + goal_builder | L1 单测全过 |
| M3 Orchestrator 骨架 | 状态机（不含 policy / vision / safety），NavigateToPose 单航点 | S1, S2 跑通 |
| M4 Policy 切换 | policy_switcher + mission_policy.yaml | S3 weave 跑通 |
| M5 Arrival + Pass-side | arrival_judge + pass_side 验证 + visual override stub | S2, S6 跑通 |
| M6 Safety 接入 | safety_node + dead-man | S8 跑通 |
| M7 GPS 降级 | gps_health_monitor + DEGRADED/SAFE_STOP | S4, S5 跑通 |
| M8 Summary + Records | mission_summary + rosbag 扩展 + photos 路径 | summary JSON schema 通过校验 |
| M9 真机第一次 | 真机 Day 1–3 达标 | 静态 bag + 航向采集数据入库 |
| M10 真机端到端 | 真机 Day 4–6 达标 | 真机 5 航点 + weave 完整跑 |

---

## 20. 与 Part 3 的衔接（前瞻）

- **Localization** 在 Part 3 需要重新引入 SLAM，但 `map` 帧定义（ENU + datum）必须保留。建议 Part 3 使用 slam_toolbox 的 **localization mode**，载入 Part 2 建过的地图 + 保留 navsat_transform 作为全局锚点。
- **Waypoint 模型**：Part 3 的“探索目标”可以复用本 YAML schema 并新增 `type: exploration | revisit`，不破坏 Part 2 字段。
- **Summary schema**：保留 Part 2 的字段，Part 3 在 `secondary_object` 之外追加 `greek_letter`, `red_yellow_obstacle_list`。
- **Phase 切换**：Part 3 需要 mapping / waypoint driving 两个 service；orchestrator 可被扩展为持有 `phase` 状态，不必重写。

---

## 21. 交付清单

**配置**：
- `config/waypoints_gps.yaml`（含 origin / datum / home / waypoints / bounds）
- `config/mission_policy.yaml`（至少 `default` / `weave_through_cones` / `final_return` / `default_marker` 四条）
- `config/ekf_local.yaml`, `config/ekf_global.yaml`, `config/navsat_transform.yaml`
- `config/nav2_part2.yaml`
- `config/mission_bounds.yaml`

**节点（本文档范围）**：
- `mission_orchestrator`、`waypoint_loader`、`arrival_judge`、`goal_builder`、`policy_switcher`、`mission_summary`、`gps_health_monitor`、`safety_node`、`fake_gps_node`

**包**：
- `p3at_localization`（launch 组合包）

**Launch**：
- `sim.launch.py`, `real.launch.py`, `nav.launch.py`

**输出**：
- `part2_runs/<mission_id>/mission_summary.{json,md}`
- `part2_runs/<mission_id>/driven_path.csv`
- `part2_runs/<mission_id>/rosbag/`
- `part2_runs/<mission_id>/photos/*.jpg`

**文档**：
- 本文档 `PART2_GPS_WAYPOINT_DESIGN.md`
- 预留 `PART2_DEMO_GUIDE.md`（实施后补，流程不属于本文档）

**不在本交付范围（由其他子系统提供）**：
- 视觉识别节点（marker 拍照 / shape 检测 / 距离估计）
- 手柄驱动与 Bluetooth 配对细节
- 真机硬件驱动具体版本选择
