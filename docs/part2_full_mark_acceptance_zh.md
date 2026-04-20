# Part 2 满分导向验收标准

## 1. 目的

本文档用于把当前仓库的 Part 2 真实机器人实现，收敛成一套可执行、可核查、可留证的验收标准。

本文档覆盖两类输入要求：

- 课程任务要求
- 19/04/2026 公布的 Part 2 marking guide / rubric

本文档的设计目标是：如果系统在最终演示时稳定满足本文所有“必须项”，并且能展示本文要求的自动产物与现场行为证据，则它已经覆盖所有 Part 2 满分评分点。

必须明确一点：

- 本文档是“满分导向验收标准”，不是对老师最终评分结果的法律式保证。
- 老师仍可能基于现场稳定性、天气、场地、演示质量和主观观感打分。
- 但如果本文所有必须项都稳定成立，那么从技术覆盖面和证据链上，系统已经满足满分要求。

## 2. 任务范围

Part 2 必须完成以下真实机器人任务：

1. 机器人按照给定 GPS waypoints 自主行驶，并最终返回起点。
2. 每个 waypoint 由橙色 traffic cone 标记；机器人到达后必须拍摄 marker，且离开时 marker 始终位于机器人右侧。
3. waypoint 1 与 waypoint 2 之间有未知间距的锥桶，机器人必须 weaving through cones。
4. 每个 waypoint 周围有一个额外彩色物体；机器人必须识别其颜色与形状，并计算它与 waypoint marker 的距离。
5. 任务结束后必须输出完整 journey summary。
6. 必须使用 LiDAR 对 marker、object、墙、人、自行车、车辆等静态和动态障碍物进行避障。
7. 必须通过 Bluetooth gamepad 实现安全链：
   - `X` 开启 automated mode
   - automated mode 下 back pedals 是 deadman，松开即停
   - `O` 开启 manual mode
   - manual mode 下可前后左右人工驾驶

## 3. 与评分 Rubric 的一一对应

### 3.1 autonomous drive

满分要求：

- 机器人能高效驶向目标点，不出现明显蛇形、来回摆动、重复冲刺刹车。
- 自动驾驶状态下，路径连续，方向修正属于正常闭环控制，不需要人工接管。
- waypoint 间移动过程没有长期停滞、无目标空转或反复重规划抖动。

必须证据：

- 现场自动行驶演示
- `summary/path.png`
- `summary/path.csv`

### 3.2 manual drive

满分要求：

- manual mode 可以平滑前进、后退、左转、右转。
- deadman 开关真实有效。
- 控制无可感知迟滞，至少不能影响正常操控。
- `X` / `O` 模式切换明确有效。

必须证据：

- 现场人工驾驶演示
- `/mission/safety`
- `/safety/automated_enabled`
- `/safety/deadman_pressed`
- `/safety/estop`

### 3.3 waypoints

满分要求：

- 所有 waypoint 都被访问。
- 机器人返回 home。
- 每个 waypoint 到达满足课程要求的 1–2 m 范围。
- rubric 对 waypoint 的满分文字是 “visits all waypoints within 5m before returning home”，但任务描述更严格，当前系统必须按任务要求执行到 1–2 m standoff。

必须证据：

- 现场完整路线演示
- `/mission/navigation`
- `summary/summary.json`
- `summary/path.png`

### 3.4 photo of marker and object

满分要求：

- 每个 waypoint 都拍到 marker
- 每个 waypoint 都拍到额外 object
- 系统记录 marker 位置
- 机器人离开 waypoint 时 marker 位于机器人右侧

必须证据：

- `photos/marker/*.jpg`
- `photos/object/*.jpg`
- `photos/annotated/*.jpg`
- `/mission/navigation` 中 `pass_side_verified=true`
- `summary/summary.html`

### 3.5 weave through the waypoint markers

满分要求：

- waypoint 1 到 waypoint 2 之间，机器人必须明显执行 weaving through cones
- 不能只是直线通过或仅依赖普通避障绕开
- 必须展示交替穿越 cone corridor 的行为

必须证据：

- 现场 weaving 演示
- `summary/path.png`
- 如有 bag，则保留 `/scan` 与路径回放

### 3.6 distance calculation

满分要求：

- 能计算未知 object 与 waypoint marker 的距离
- 满分标准要求误差控制在 1 m 内

必须证据：

- `/mission/vision`
- `summary/summary.json`
- `summary/summary.html`
- 现场抽查若干 waypoint 的实测对比

### 3.7 identify object

满分要求：

- 能识别未知障碍物的颜色和形状
- 现场展示应尽量覆盖多个颜色与形状类别

当前系统目标类别：

- 颜色：`dark_green`, `yellow`, `red`, `orange`, `blue`
- 类别：`cube`, `cuboid`, `cylinder`
- 显示形状：`square`, `rectangle`, `circle`

必须证据：

- `/mission/vision`
- `summary/summary.html`
- `summary/summary.json`
- 保存的 object 图片

### 3.8 obstacle avoidance

满分要求：

- 能流畅避开静态障碍物和动态障碍物
- 避障不是完全中断任务，而是尽量平滑减速、停车、绕行和恢复

必须证据：

- 现场静态障碍演示
- 现场动态障碍演示
- Nav2 costmap / collision monitor 行为正常

### 3.9 journey summary

满分要求：

- 能展示完整 journey summary
- 可以看出 driven path
- 可以查看 identified object details
- 已构建地图
- 能回顾 waypoint 相关图片和识别结果

必须证据：

- `summary/summary.html`
- `summary/summary.json`
- `summary/path.csv`
- `summary/path.png`
- `map/map.yaml`
- `map/map.pgm`

## 4. 满分验收必须项

以下项目全部满足，才可以认为“技术上覆盖满分要求”。

### 4.1 环境与构建

- 最终机器人仓库位于正确路径
- 使用最新 `main`
- `git status` 干净
- `./start_robot_docker.sh --mode prep` 成功
- `./start_robot_docker.sh --mode build` 成功
- `install/setup.bash` 可用

### 4.2 硬件链路

- 底盘串口识别正确
- GPS 串口识别正确
- IMU 驱动与 launch 识别正确
- LiDAR 型号、IP、frame、topic 全部识别正确
- OAK-D RGB、depth、camera_info topics 正常
- Bluetooth gamepad 可稳定连接

### 4.3 基础 topics

必须存在且内容合理：

- `/odom`
- `/fix`
- `/imu/data`
- `/scan`
- `/camera/camera/color/image_raw`
- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/color/camera_info`
- `/joy`
- `/mission/navigation`
- `/mission/vision`
- `/mission/safety`
- `/odometry/filtered/local`
- `/odometry/filtered/global`

### 4.4 安全链

- `X` 切 AUTO
- `O` 切 MANUAL
- deadman 必须生效
- 松开 deadman 即停
- estop 能触发
- estop 不会错误自清除
- manual drive 无明显 lag

### 4.5 定位与导航

- GPS + navsat_transform + EKF + Nav2 正常
- robot 不会因为 TF 错误或 frame 缺失而无法规划
- 自动导航至各 waypoint 时不需要人工救场
- return-home 正常
- 室内测试时若 GPS health 不可用，只能把该阶段视为“环境预检查”，不能据此判定 waypoint 主链通过；waypoint / return-home / 满分验收必须在室外或 GPS 条件满足的场地完成

### 4.6 waypoint / marker / right-side 规则

- 每个 waypoint 到达后都进入 inspection
- marker cone 被拍照并保存
- 每个 waypoint 离开前都验证 marker 在右侧
- 到达距离必须收敛到 1–2 m standoff

### 4.7 视觉识别

- marker 能稳定检测
- object 能稳定检测
- object 颜色和形状识别基本正确
- 深度有效时 `status=ok`
- 距离可计算且尽量控制到 1 m 误差内

### 4.8 weaving

- WP1 -> WP2 段必须出现明显 weaving 行为
- 不能被普通直行导航替代
- 若触发 fallback，也不能完全失去 weaving 展示

### 4.9 避障

- LiDAR 进入 Nav2 和 collision monitor 主链
- 能对静态障碍物避障
- 能对动态障碍物减速、停车、恢复

### 4.10 summary

必须自动生成：

- `summary/summary.html`
- `summary/summary.json`
- `summary/path.csv`
- `summary/path.png`
- `map/map.yaml`
- `map/map.pgm`
- 每个 waypoint 的 marker/object/annotated 图片

## 5. 现场验收流程

必须按以下顺序执行，不允许第一次就直接 full demo：

1. 环境盘点
2. prep
3. build
4. 单设备测试
5. 单节点测试
6. 单 waypoint 测试
7. 双 waypoint 测试
8. weaving 专项测试
9. 全流程 rehearsal
10. 正式展示

## 6. 不允许的错误做法

- 在未确认最终机器人硬件前，把临时机器人参数写死进默认值
- 用旧 README、旧配置、旧 `global_topics.yaml` 作为运行真理源
- 跳过 `prep/build` 直接赌现成 build 可用
- 不做单 waypoint / weaving 专项测试就直接 full demo
- 人工补 summary、人工补图片、人工补结果
- 用口头解释代替系统真实实现

## 7. 最终展示前清单

最终展示前，必须逐条打勾：

- 最新 main 已同步
- 最终机器人硬件实配表已确认
- 最终 GPS waypoints 文件已替换完成
- 单设备链路全部通过
- vision bench 通过
- safety/manual 通过
- single waypoint 通过
- waypoint 1 -> 2 weaving 通过
- full demo rehearsal 至少成功 1 次
- summary 可在现场直接打开

## 8. 结论

如果系统在最终机器人上稳定满足本文全部“必须项”，并且现场证据与自动产物完整可展示，那么它已经覆盖 Part 2 全部任务要求与 rubric 满分项，是一套满分导向的可验收实现。
