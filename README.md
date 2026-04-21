# AUTO4508 Group Project - Team 17

## 概览

- 当前仓库以 `p3at_bringup/launch/real.launch.py` 作为 Part 2 真机主入口
- 推荐使用根目录脚本 `start_robot_docker.sh` 进入容器、准备环境、构建和启动
- 运行产物统一输出到 `artifacts/part2_runs/<run_id>`
- 当前主链包含：`aria_node`、GPS、IMU、LiDAR、Nav2、`part2_supervisor`、`vision.launch.py`、`safety_node`、`logger_node`

## 推荐流程

```bash
./start_robot_docker.sh --mode prep
./start_robot_docker.sh --mode build
./start_robot_docker.sh --mode vision
./start_robot_docker.sh --mode real
```

## 常用命令

- 进入交互容器：

```bash
./start_robot_docker.sh --mode shell
```

- 准备依赖环境：

```bash
./start_robot_docker.sh --mode prep
```

- 构建工作区：

```bash
./start_robot_docker.sh --mode build
```

- 单独测试视觉链：

```bash
./start_robot_docker.sh --mode vision
```

- 启动完整真机链：

```bash
./start_robot_docker.sh --mode real
```

## 默认硬件假设

- 最终演示 Pioneer IP：`192.168.2.213`
- Pioneer 底盘串口：`/dev/ttyUSB0`
- GPS 串口：`/dev/ttyACM0`
- GPS 波特率：`9600`
- LiDAR 默认驱动：`lakibeam1`
- LiDAR 默认模式：Lakibeam RJ45 UDP
- LiDAR Host IP（机器人网卡监听地址）：`192.168.198.50`
- LiDAR Sensor IP：`192.168.198.2`
- LiDAR UDP Port：`2368`
- LiDAR topic / frame：`/scan` + `laser_frame`
- SICK 回退：`--lidar-driver sick -- --lidar_hostname:=192.168.0.1`
- IMU：`phidgets_spatial`
- 相机：OAK-D，使用 `depthai_ros_v3`
- 手柄：DualShock 类蓝牙手柄

## 覆盖默认参数

如设备端口或驱动与默认值不一致，可以直接覆盖：

```bash
./start_robot_docker.sh --mode real \
  --robot-port /dev/ttyUSB1 \
  --gps-port /dev/ttyUSB0 \
  --lidar-driver lakibeam \
  --lidar-host-ip 192.168.198.50 \
  --lidar-sensor-ip 192.168.198.2 \
  --lidar-port 2368 \
  --lidar-frame-id laser_frame
```

- 如需回退到 SICK：

```bash
./start_robot_docker.sh --mode real \
  --lidar-driver sick \
  -- --lidar_hostname:=192.168.0.1
```

- 如需自定义外部 LiDAR bringup：

```bash
./start_robot_docker.sh --mode real \
  --lidar-driver custom \
  -- --lidar_bringup_package:=<pkg> --lidar_bringup_launch:=<launch.py>
```

## 说明

- `global_topics.yaml` 不再是当前 Part 2 主链的运行时配置源
- 当前 `safety_node`、`vision_node`、`logger_node` 的参数接口以各自节点源码和 launch 文件为准
- 当前最终演示 Pioneer 的 LiDAR 网卡实配为 `192.168.198.50/24`；若现场网卡地址变更，必须同步覆盖 `--lidar-host-ip`
- 若需 Lakibeam 真机联调，必须先确认机器人网卡上已经配置用于接收 UDP 的 `192.168.198.50/24`，或明确改用 USB-C `192.168.8.x` 路线
- Lakibeam 真机验收不能只看 `/scan` 存在；还必须核对 `frame_id=laser_frame`、`base_link -> laser_frame`、Nav2 costmap、collision monitor 和 supervisor 都真实吃到数据
- 若需真机联调，请优先核对串口映射、LiDAR 型号/IP、IMU launch 名称和 OAK-D topic 是否与默认值一致
