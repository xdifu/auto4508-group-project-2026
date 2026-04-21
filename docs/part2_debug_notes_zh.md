# Part 2 最终机器人调试记录

## 1. 用途

本文档用于记录最终机器人上的真实硬件映射、调试结果、问题、修复方法和演示前注意事项。

本文档是现场联调记录，不替代正式验收标准。

正式验收标准见：

- `/home/god/auto4508-group-project-2026/docs/part2_full_mark_acceptance_zh.md`

## 2. 记录规则

每次调试必须记录：

- 日期时间
- 机器人 IP / 机器人编号
- 当前 git commit
- 当前 waypoint 文件
- 当前 launch 参数覆盖
- 成功项
- 失败项
- 证据路径
- 下一步动作

## 3. 最终机器人实配表

当前默认正式方案（Lakibeam 主链）：

- 机器人 IP：`192.168.2.213`
- 机器人编号：`pioneer7-NUC11PHi7`
- SSH 用户：`team17`
- 仓库路径：`/home/team17/auto4508-group-project-2026`
- git commit：以正式演示当日最终 commit 为准
- Pioneer base 串口：
- GPS 串口：
- GPS 波特率：
- IMU 驱动包：
- IMU launch：
- LiDAR 型号：`Lakibeam`
- LiDAR 包：`lakibeam1`
- LiDAR launch：`p3at_bringup/launch/lidar.launch.py`（`lidar_driver:=lakibeam`）
- LiDAR Host IP：`192.168.198.50`
- LiDAR Sensor IP：`192.168.198.2`
- LiDAR UDP Port：`2368`
- LiDAR frame：`laser_frame`
- LiDAR topic：`/scan`
- OAK-D RGB topic：
- OAK-D depth topic：
- OAK-D camera_info topic：
- 手柄型号：

历史调试记录仍保留下文中的 `192.168.2.105 / pioneer5-NUC11PHi7` 与 SICK 路线结论；若与当前正式默认值冲突，以本节和当前 README 为准。

## 4. 当前已知经验

- 不要第一次就 full demo，必须按单设备 -> 单节点 -> 单 waypoint -> weaving -> 全流程的顺序推进。
- LiDAR 对 obstacle avoidance 和 weaving 都是主链关键项，不能只保证 `/scan` 存在，还必须保证 frame、costmap 和 collision monitor 真正吃到数据。
- 当前仓库默认 LiDAR 主链已切到 Lakibeam；若现场临时回退 SICK，必须显式使用 `lidar_driver:=sick`，不能再依赖旧默认值。
- vision 的满分不只是能框出来，还要稳定得到 `status=ok`、颜色、形状、距离和三类图片。
- journey summary 必须自动产出，不能赛后手工补。
- final robot 的硬件如果与之前临时机器人不同，必须优先做实配表，不允许直接套旧参数。
- Lakibeam 真机前置项：
  - 当前最终演示 Pioneer 用于接收 LiDAR UDP 的网卡实配为 `192.168.198.50/24`
  - 若现场网卡地址变化，必须同步修改 `lidar_host_ip` 与 Lakibeam 设备端 `host.ip`
  - 启动前先检查 `ip -o -4 addr show`
  - 启动后必须同时检查 `/scan`、`frame_id=laser_frame`、`base_link -> laser_frame`
  - 必须确认 Nav2 costmap、collision monitor、supervisor 都真实订阅 `/scan`
- 当前最终机器人与之前临时机器人不同：
  - 最终机器人网口是 `192.168.0.50/24`
  - `192.168.0.1` 可达
  - `192.168.198.2` 不可达
  - 这是之前 SICK 路线调试阶段的历史结论，不再代表当前 Lakibeam 默认主链。
- 首次连接最终机器人时，仓库并不是最新主线，而是旧提交 `afeaaff`。在任何正式测试前，必须先 `git fetch + git pull --ff-only origin main`。
- `setup_env.sh` 原先把 `ros2-testing-apt-source` 当成硬依赖，这会在最终机器人的课程镜像中直接失败；现已改成“可装则装，不可装则继续使用现有 apt 源”。
- 最终机器人默认课程镜像 `docker-t5_paneer:latest` 本身不带 AriaCoda/libAria，因此在未执行修正后的 `setup_env.sh` 前，`aria_node` 无法通过编译。
- 课程镜像 `docker-t5_paneer:latest` 混入旧 ROS 二进制后，与新安装的 Jazzy deb 会发生 `fastcdr` ABI 冲突；表现在 `gps.launch.py` 和 `lidar.launch.py` 启动时出现 `symbol lookup error`。最终解决方案不是继续修补旧镜像，而是改用干净 `ros:jazzy` 基底重建 `Dockerfile.team17`。
- `colcon build` 生成的 `build/install/log` 目录默认由容器内 root 写入，宿主机上直接 `rm -rf` 会失败；现场若需要彻底清理缓存，必须使用容器自身执行：
  - `docker run --rm -v /home/team17/auto4508-group-project-2026:/workspace -w /workspace auto4508_team17_env:latest bash -lc 'rm -rf build install log'`
- `start_robot_docker.sh` 不能强制写死 `docker run -it`，否则在 SSH 后台、nohup 或自动化调试场景下会直接报 `the input device is not a TTY`；现已改为“检测到真实 TTY 才附加 `-it`”。
- 当前最终机器人上已经存在老师预置镜像：
  - `pioneer_ws-aria_driver`
  - `pioneer_ws-gps_driver`
  - `pioneer_ws-teleop`
  但当前主链仍以统一的 `start_robot_docker.sh` + `real.launch.py` 为主，不应在正式方案里重新拆回多容器人工拼装。
- 最终机器人上虽然存在老师或其他人留下的多个 Docker 镜像，但不能默认拿来直接作为正式基线：
  - 已实测 `docker-t5_paneer:latest` 会触发 Jazzy 相关 ABI 冲突
  - 其他 `pioneer_ws-*` 镜像更偏向分拆式临时调试，不保证与当前仓库主链一致
  - 当前最可复用、最可重建的正式方案仍然是仓库内的 `Dockerfile.team17` + `start_robot_docker.sh`
- `real.launch.py` 在最终机器人上的 `aria_node` 集成并不是简单“单链可用就整链可用”：
  - standalone `aria_base.launch.py` 可以连上 Pioneer
  - 但整链默认并发启动时，`aria_node` 会在串口握手阶段失败
  - 通过把 `robot_state_publisher` 以及其他非底盘节点一起延后，并把 `startup_delay_sec` 调到 `12.0`，整链默认启动才稳定连上最终机器人底盘
- 在最终机器人室内测试时，`/fix` 可能仍然有消息，但 `GPS health -> FATAL` 与 supervisor 停在 `WAITING_FOR_GPS` 是正常现象：
  - 这是室内卫星条件不足的环境限制
  - 不能把室内 GPS 不达标误判为当前代码主链 bug
  - 室外再做 waypoint / return-home / 满分任务闭环验证
- 2026-04-21 室内整链验证已确认下列项目在最终机器人上可起：
  - `/odom`
  - `/scan`，且 `frame_id=laser_frame`
  - `/odometry/filtered/local`
  - `/odometry/filtered/global`
  - `/imu/data`
  - `/fix`
  - `/camera/camera/color/image_raw`
  - `/camera/camera/depth/image_rect_raw`
  - `/camera/camera/color/camera_info`
  - `/mission/safety`
  - `base_link -> laser_frame` TF
- 如果担心机器人本机文件明天被清理或不可持久保存，必须把所有有效改动保留在仓库中，而不是只留在机器人本地容器里：
  - 以仓库文件为准
  - 用 `Dockerfile.team17` 重新构建镜像
  - 用 `start_robot_docker.sh --mode prep/build/real/vision` 重现
  - 不依赖机器人上现成旧容器长期保存状态

## 5. 调试日志

### Round 1

- 时间：2026-04-20 22:3x AWST
- 机器人：`192.168.2.105 / pioneer5-NUC11PHi7`
- commit：初始为 `afeaaff`
- waypoint 文件：未开始 mission 级测试
- 启动命令：SSH 盘点 + git 状态检查
- 成功：
  - 成功连接最终机器人
  - 成功确认最终机器人与临时机器人 LiDAR 网络不同
  - 成功确认最终机器人是 `192.168.0.1` SICK 路线
  - 成功确认串口设备存在：`/dev/ttyUSB0`、`/dev/ttyACM0`
  - 成功确认 OAK-D、u-blox、Phidgets 设备都在 USB 枚举里
- 失败：
  - 机器人仓库不是最新 `main`
- 产物：无
- 下一步：
  - 先同步到最新 `main`
  - 再跑 `prep/build`

### Round 2

- 时间：2026-04-20 22:4x AWST
- 机器人：`192.168.2.105 / pioneer5-NUC11PHi7`
- commit：`6353ab2`
- waypoint 文件：未开始 mission 级测试
- 启动命令：
  - `git fetch origin`
  - `git pull --ff-only origin main`
  - `./start_robot_docker.sh --mode build`
- 成功：
  - 已 fast-forward 到最新 `main`
  - build 已经把真正 blocker 缩小到 `aria_node`
- 失败：
  - `aria_node` 因缺少 AriaCoda/libAria 失败
  - `setup_env.sh` 首轮尝试因 `ros2-testing-apt-source` 不存在而提前失败
- 产物：build 日志
- 下一步：
  - 修复 `setup_env.sh`
  - 重新执行 `prep`
  - 再次执行 `build`

### Round 3

- 时间：2026-04-20 22:4x AWST
- 机器人：`192.168.2.105 / pioneer5-NUC11PHi7`
- commit：`6353ab2`
- waypoint 文件：未开始 mission 级测试
- 启动命令：
  - 修正后的 `./start_robot_docker.sh --mode prep`
- 成功：
  - 已确认容器内 apt 源实际上能找到：
    - `ros-jazzy-phidgets-spatial`
    - `ros-jazzy-sick-scan-xd`
    - `ros-jazzy-depthai-ros-v3`
  - 已确认 `prep` 正在安装这些缺失依赖
- 失败：无最终结论，安装仍在进行
- 产物：进程状态记录
- 下一步：
  - 等待 `prep` 完成
  - 重新 `build`
  - 进入单设备链路测试

### Round 4

- 时间：2026-04-20 23:1x AWST
- 机器人：`192.168.2.105 / pioneer5-NUC11PHi7`
- commit：`6353ab2`
- waypoint 文件：未开始 mission 级测试
- 启动命令：
  - 使用基于干净 `ros:jazzy` 的 `Dockerfile.team17` 重建 `auto4508_team17_env:latest`
  - `./start_robot_docker.sh --mode build`
- 成功：
  - 新镜像已成功构建
  - workspace 已在最终机器人上重新完整 build 通过
- 失败：
  - 首次 build 仍引用旧镜像缓存里的 `/opt/ros/jazzy/lib/libfastcdr.so.2.2.5`
- 原因：
  - 宿主机保留了旧镜像时期生成的 `build/install/log`，里面烘进了具体 `.so` 路径
- 修复：
  - 用容器自身清掉 `build/install/log`
  - 再次 `./start_robot_docker.sh --mode build`
- 产物：
  - 新镜像 `auto4508_team17_env:latest`
  - 最终机器人上的 clean build 日志
- 下一步：
  - 进入 base/GPS/IMU/LiDAR/vision 单设备测试

### Round 5

- 时间：2026-04-21 00:0x AWST
- 机器人：`192.168.2.105 / pioneer5-NUC11PHi7`
- commit：`6353ab2 + 本地未提交最终机器人适配修补`
- waypoint 文件：未开始 mission 级室外测试
- 启动命令：
  - `./start_robot_docker.sh --mode shell --name t17-base -- ros2 launch p3at_bringup aria_base.launch.py`
  - `./start_robot_docker.sh --mode real --name t17-fulldefault`
- 成功：
  - standalone `aria_base.launch.py` 能稳定连上 Pioneer
  - `gps.launch.py`、`imu.launch.py`、`lidar.launch.py`、`vision.launch.py` 单链均通过
  - `real.launch.py` 经过最终机器人专用时序修正后，整链已可在室内跑到：
    - `/odom`
    - `/scan`
    - `/odometry/filtered/local`
    - `/odometry/filtered/global`
    - `/imu/data`
    - `/fix`
    - `/camera/...`
    - `/mission/safety`
- 失败：
  - 室内环境下 GPS health 会降到 `FATAL`
  - supervisor 因室内 GPS 条件不足停在 `WAITING_FOR_GPS`
- 产物：
  - `/tmp/t17-base*.log`
  - `/tmp/t17-full*.log`
  - 机器人上的室内 artifacts 目录
- 修复：
  - `real.launch.py` 新增并调高 `startup_delay_sec`
  - 默认值从 `5.0` 调到 `12.0`
  - `robot_state_publisher` 被放入延后启动组，与其他非底盘节点一起后移，避免干扰 Pioneer 底盘串口握手
- 下一步：
  - 到室外后先确认 GPS health 恢复正常
  - 替换为最终 waypoint 文件
  - 做 manual drive / AUTO-MANUAL / deadman
  - 跑单 waypoint、双 waypoint、weave、full mission

### Round 6（蜂鸣 + 手柄无效紧急修复）

- 时间：2026-04-21 01:5x AWST
- 机器人：`192.168.2.213 / pioneer7-NUC11PHi7`
- 现象：
  - 机器人持续蜂鸣
  - 手柄按键有 `/joy` 数据，但车不动
- 直接证据：
  - `docker logs t17-manual` 显示 `aria_node` 启动后报错并退出：
    - `Could not connect ... /dev/ttyUSB0`
    - `No packet`
    - `process has died`
  - 此时 `ros2 topic info /cmd_vel -v` 只有发布者（`safety_node`），`Subscription count: 0`（底盘未接入）
- 根因（本次实测）：
  - 手动容器里底盘驱动只尝试一次串口握手，首次失败后 `aria_node` 退出；
  - 其他节点（`joy/teleop/safety`）仍在运行，造成“看起来系统在跑、但底盘完全不接收 `/cmd_vel`”。
- 修复动作（现场生效）：
  - 在容器内手工重试底盘连接，确认串口可恢复握手：
    - `timeout 8 /workspace/install/aria_node/lib/aria_node/ariaNode -rp /dev/ttyUSB0`
    - 出现 `Syncing 0/1/2` + `Connected to robot`。
  - 随后重新拉起 `aria_base.launch.py`，恢复 `/aria_node` 常驻。
  - 蜂鸣在底盘重新建立连接后结束。
- 避坑结论（必须执行）：
  - 不能只看 `docker ps` 判断“系统正常”，必须同时检查：
    - `ros2 node list` 里有 `/aria_node`
    - `ros2 topic info /cmd_vel -v` 的 `Subscription count >= 1`
  - 一旦出现“蜂鸣 + 手柄无效”，先查 `aria_node` 是否已退出，再做其他调参。

### Round 7（最终展示前室内手动过渡启动）

- 时间：2026-04-21 13:4x AWST
- 机器人：`192.168.2.213 / pioneer7-NUC11PHi7`
- 场景：
  - 最终展示开始前，需先在室内用蓝牙手柄 `O -> MANUAL` 把 Pioneer 开到室外；
  - 此阶段不应先武装自动任务，优先保证 `aria + joy + teleop + safety` 手动最小链稳定。
- 新发现的坑：
  - 在最终机器人上直接执行 `./start_robot_docker.sh --mode shell ...`，脚本会在真正进入容器前失败：
    - `/opt/ros/jazzy/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable`
  - 即使绕过外层脚本，在容器里若继续用 `set -u` 再 `source /workspace/install/setup.bash`，也会失败：
    - `/opt/ros/jazzy/local_setup.sh: line 23: AMENT_PYTHON_EXECUTABLE: unbound variable`
- 根因（本次实测）：
  - 这台最终机器人当前工作区/镜像里的 ROS setup 脚本在 `nounset` 环境下并不安全；
  - 因此任何 `set -u` + `source /opt/ros/...` 或 `source /workspace/install/setup.bash` 的启动链，都会在现场直接中断。
- 现场可行修复（已验证生效）：
  - 室内手动过渡阶段不走 `start_robot_docker.sh --mode shell`；
  - 直接用 `docker run -d` 启动 `auto4508_team17_env:latest` 最小手动容器：
    - 挂载 `/dev`
    - `--network host`
    - 挂载工作区到 `/workspace`
  - 容器内启动脚本改为 `set -eo pipefail`，不使用 `set -u`；
  - 手动容器只启动：
    - `aria_base.launch.py`
    - `joy_node`
    - `teleop_twist_joy`
    - `safety_node`
  - `aria_base.launch.py` 用循环重试包装，因为首次串口握手仍可能短暂出现：
    - `Syncing 0`
    - `No packet`
    - 随后再次重试才 `Connected to robot`
- 直接证据（现场实测）：
  - `joy_node` 日志：
    - `Opened joystick: PS4 Controller`
  - `teleop_twist_joy` 日志：
    - 线速度轴 `x -> axis 1`
    - 角速度轴 `yaw -> axis 2`
  - `safety_node` 日志：
    - `Safety node ready. joy=/joy nav=/cmd_vel_nav manual=/cmd_vel_manual`
  - `aria_base.log`：
    - `Connected to robot`
    - `Aria base driver ready. cmd_vel_topic=/cmd_vel`
  - `ros2 topic info /cmd_vel -v`：
    - Publisher: `safety_node`
    - Subscription count: `1`
    - Subscriber: `aria_node`
  - `ros2 topic echo /mission/safety --once`：
    - `mode = MANUAL`
    - `automated_enabled = false`
    - `deadman_pressed = true`
    - `estop = false`
    - `joy_connected = true`
  - `ros2 topic hz /joy`：
    - 现场约 `15-17 Hz`
- 本轮结论：
  - 当前这台 Pioneer 的蓝牙手柄链、`O -> MANUAL`、deadman、安全仲裁和底盘订阅都已正常；
  - 室内可以先以手动模式安全开到室外，再切换完整 `real.launch.py` 做最终自动展示。
- 必须记住：
  - 现场一旦只是“想先手动挪车”，不要先纠结完整 mission 栈，先保住最小手动链；
  - 只要 `start_robot_docker.sh --mode shell` 仍报 `AMENT_TRACE_SETUP_FILES`，就直接走最小手动容器方案；
  - 判断“现在能不能手动开车”必须同时满足：
    - `joy_node` 已打开 PS4 手柄
    - `safety_node` 正常发布 `/mission/safety`
    - `ros2 topic info /cmd_vel -v` 的 `Subscription count >= 1`
    - `aria_base.log` 已出现 `Connected to robot`

### Round 8（最终展示前 full chain 最终修正）

- 时间：2026-04-21 13:5x-14:0x AWST
- 机器人：`192.168.2.213 / pioneer7-NUC11PHi7`
- 目标：
  - 最终展示时不再依赖额外终端命令；
  - 直接使用同一个蓝牙手柄完成：
    - `O -> MANUAL`
    - `X -> AUTO`
    - back pedal = deadman
- 新发现的高优先级问题：
  - 仅启动“最小手动链”时，`X` 只会把 safety mode 切到 `AUTO`，但不会真正进入自动导航，因为没有 `/cmd_vel_nav` 发布者；
  - full chain 初次切换到 `real.launch.py` 后，`part2_supervisor` 在自动运行中若被 `O` 切回 MANUAL 或 deadman 松开，只是底盘停下，后台 goal 不一定被取消；
  - ROS 图里出现了外部幽灵节点：
    - `/mission_controller`
    - `/Aria_node`
  - 这些节点来自同网段自动发现，污染了 `/cmd_vel` 的发布/订阅图，属于真实安全风险。
- 根因（本次实测）：
  - 之前的最小手动链没有 Nav2/supervisor，因此 `X` 不可能真正触发自动任务；
  - `part2_supervisor` 旧逻辑只在 `estop` 时显式 `_cancel_goal()`，没有把 `MANUAL` / deadman release 视为自动任务暂停事件；
  - 容器默认 discovery 范围会看到同子网 ROS 节点，不适合最终演示现场。
- 代码修复（已落地）：
  - `part2_supervisor.py`
    - 新增 `_pause_for_safety()`
    - 当 `require_safety_topics=true` 且自动运行中收到：
      - `automated_enabled=false`
      - 或 deadman release
    - 会：
      - cancel 当前 goal
      - 清空当前导航序列
      - 重置 arrival/vision 状态
      - 进入 `WAITING_FOR_SAFETY`
    - 这样 `O -> MANUAL` / deadman 松开 不再只是“底盘停住”，而是“自动任务被安全暂停”；
    - 重新按 `X` 并满足 deadman 后，再从 `WAITING_FOR_SAFETY -> PLAN_SEGMENT` 恢复。
  - `start_robot_docker.sh`
    - ROS setup source 前统一 `set +u`
    - 修复最终机器人上 `AMENT_TRACE_SETUP_FILES` / `AMENT_PYTHON_EXECUTABLE` 引起的启动崩溃；
    - 默认把 `ROS_AUTOMATIC_DISCOVERY_RANGE` 设为 `LOCALHOST`
    - 切断同网段外部幽灵节点对本机 `/cmd_vel` 图的污染。
  - `aria_base.launch.py`
    - `aria_node` 增加 `respawn=True`
    - `respawn_delay=2.0`
    - 避免“首次串口握手失败后 full chain 整轮报废”。
- 现场恢复流程（本次有效）：
  - 当 full chain 因底盘串口状态异常无法第一次连上时：
    - 先停掉 `auto4508_real_ros`
    - 运行一次独立 `ariaNode -rp /dev/ttyUSB0`
    - 等看到 `Syncing 0/1/2 + Connected to robot`
    - 再重新拉起 `real.launch.py`
  - 本轮最终成功日志：
    - `Connected to robot`
    - `Aria base driver ready`
    - `Lakibeam UDP packets are arriving`
    - `Published first Lakibeam LaserScan`
    - `Safety node ready`
    - `Part 2 supervisor initialized`
    - `State -> WAITING_FOR_GPS`
- 当前最终待命状态（本轮最终实测）：
  - full chain 容器：`auto4508_real_ros`
  - `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
  - `/mission/safety`：
    - `mode = MANUAL`
    - `deadman_pressed = true`
    - `estop = false`
    - `joy_connected = true`
  - `/safety/automated_enabled = false`
  - `/joy` 持续有数据（约 `14-16 Hz`）
  - GPS 室内仍为 `FATAL`，因此 supervisor 正常停在 `WAITING_FOR_GPS`
- 直接结论：
  - 现在已经不是“手动链”和“自动链”分开两套入口；
  - 当前 full chain 就绪后，展示现场应按同一只手柄操作：
    - `O` 保持 / 切回手动
    - `X` 切自动
    - back pedal 始终作为 deadman
  - 室内阶段 GPS 不健康时，`X` 不会真正开始 mission，这是预期；
  - 到室外 GPS 恢复后，再按 `X` 才会进入自动任务。
