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

待填写：

- 机器人 IP：`192.168.2.105`
- 机器人编号：`pioneer5-NUC11PHi7`
- SSH 用户：`team17`
- 仓库路径：`/home/team17/auto4508-group-project-2026`
- git commit：首次连接时是 `afeaaff`，已 fast-forward 到最新 `main = 6353ab2`
- Pioneer base 串口：
- GPS 串口：
- GPS 波特率：
- IMU 驱动包：
- IMU launch：
- LiDAR 型号：
- LiDAR 包：
- LiDAR launch：
- LiDAR IP：
- LiDAR frame：
- OAK-D RGB topic：
- OAK-D depth topic：
- OAK-D camera_info topic：
- 手柄型号：

## 4. 当前已知经验

- 不要第一次就 full demo，必须按单设备 -> 单节点 -> 单 waypoint -> weaving -> 全流程的顺序推进。
- LiDAR 对 obstacle avoidance 和 weaving 都是主链关键项，不能只保证 `/scan` 存在，还必须保证 frame、costmap 和 collision monitor 真正吃到数据。
- vision 的满分不只是能框出来，还要稳定得到 `status=ok`、颜色、形状、距离和三类图片。
- journey summary 必须自动产出，不能赛后手工补。
- final robot 的硬件如果与之前临时机器人不同，必须优先做实配表，不允许直接套旧参数。
- 当前最终机器人与之前临时机器人不同：
  - 最终机器人网口是 `192.168.0.50/24`
  - `192.168.0.1` 可达
  - `192.168.198.2` 不可达
  - 说明最终机器人更符合当前仓库默认的 SICK 路线，而不是之前临时机器人那条 `192.168.198.x` 路线。
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
