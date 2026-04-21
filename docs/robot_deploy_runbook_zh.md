# Pioneer 现场部署与运行手册

## 1. 适用场景

- 需要把本机工作区代码手动同步到 Pioneer 机器人
- 需要在 Pioneer 上管理 `t17-real` 容器
- 需要在 Pioneer 上构建、启动、检查 Part 2 演示链路
- 需要把本地改动推送到 GitHub

默认机器人信息：

- 机器人 SSH 用户：`team17`
- 仓库路径：`/home/team17/auto4508-group-project-2026`
- 常用运行容器名：`t17-real`

## 2. 本机代码如何手动推到机器人

推荐方式是 `rsync`，因为它会增量同步，适合频繁迭代。

在本机仓库根目录执行：

```bash
rsync -avz --delete \
  --exclude '.git' \
  --exclude 'build' \
  --exclude 'install' \
  --exclude 'log' \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  /home/god/auto4508-group-project-2026/ \
  team17@<ROBOT_IP>:/home/team17/auto4508-group-project-2026/
```

如果只想推单个文件：

```bash
scp /home/god/auto4508-group-project-2026/src/p3at_bringup/config/nav2_part2.yaml \
  team17@<ROBOT_IP>:/home/team17/auto4508-group-project-2026/src/p3at_bringup/config/nav2_part2.yaml
```

同步完成后登录机器人：

```bash
ssh team17@<ROBOT_IP>
cd /home/team17/auto4508-group-project-2026
```

## 3. 容器怎么管理

查看当前容器：

```bash
docker ps -a
```

查看 `t17-real` 日志：

```bash
docker logs --tail 200 t17-real
docker logs -f t17-real
```

进入容器：

```bash
docker exec -it t17-real bash
```

停止并删除运行容器：

```bash
docker rm -f t17-real
```

如果历史上也起过 `auto4508_real_ros`，一起清掉：

```bash
docker rm -f t17-real auto4508_real_ros
```

## 4. 在机器人上怎么构建

先进入仓库：

```bash
cd /home/team17/auto4508-group-project-2026
```

全量构建：

```bash
./start_robot_docker.sh --mode build
```

只构建某个包，缩短时间：

```bash
./start_robot_docker.sh --mode build --name t17-build -- --packages-select p3at_bringup
```

如果改了多个包：

```bash
./start_robot_docker.sh --mode build --name t17-build -- --packages-select p3at_bringup auto4508_project
```

## 5. 在机器人上怎么运行

启动完整真机链路：

```bash
./start_robot_docker.sh --mode real
```

如果报容器名冲突：

```bash
docker rm -f t17-real
./start_robot_docker.sh --mode real
```

## 6. 怎么检查容器和链路是否正常

### 6.1 宿主机检查

```bash
docker ps
docker logs --tail 120 t17-real
```

关键日志应尽量看到：

- `Connected to robot`
- `Aria base driver ready`
- `Published first Lakibeam LaserScan`
- `Part 2 supervisor initialized`

### 6.2 容器内检查

进入容器：

```bash
docker exec -it t17-real bash
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
```

检查节点：

```bash
ros2 node list
```

至少应看到这些主链节点中的大部分：

- `/aria_node`
- `/richbeam_lidar_node0`
- `/joy_node`
- `/safety_node`
- `/part2_supervisor`
- `/planner_server`
- `/controller_server`
- `/bt_navigator`

检查自动状态：

```bash
ros2 topic echo /mission/state --once
ros2 topic echo /safety/automated_enabled --once
ros2 topic echo /safety/deadman_pressed --once
ros2 topic echo /safety/estop --once
```

检查底盘是否真正接上 `/cmd_vel`：

```bash
ros2 topic info /cmd_vel -v
```

正确情况应看到：

- Publisher: `safety_node`
- Subscriber: `aria_node`

检查 LiDAR：

```bash
ros2 topic echo /scan --once
```

检查自动导航是否真在给速度：

```bash
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel --once
```

## 7. 手柄和演示操作

默认安全链约定：

- `O`：手动模式
- `X`：自动模式
- `R2`：deadman

建议现场操作顺序：

1. 先按 `O`，确认可以手动
2. 手动把车挪到起点
3. 室外 GPS 稳定后，按住 `R2`
4. 再按 `X` 切自动

如果出现 `SAFE_STOP`：

```bash
ros2 topic echo /safety/estop --once
```

若 `estop=true`，先解锁急停，再切模式，不要只反复按 `O/X`。

## 8. 常见故障处理

### 8.1 容器活着，但车不动

先查：

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo /cmd_vel --once
ros2 topic echo /cmd_vel_nav --once
```

- `/cmd_vel_nav = 0`：问题在 Nav2 / TF / controller
- `/cmd_vel_nav` 非零但 `/cmd_vel = 0`：问题在 safety / collision monitor
- `/cmd_vel` 非零但车不动：问题在 `aria_node` / 底盘连接

### 8.2 导航显示 `NAVIGATING` 但车不走

重点查日志：

```bash
docker logs --tail 120 t17-real | grep -E "RPPPathHandler|Unable to transform|Goal failed|dropping message"
```

如果看到：

- `Lookup would require extrapolation into the future`
- `dropping message: frame 'laser_frame'`

说明主要是 TF / 时间戳容错问题，不是模式切换问题。

### 8.3 容器名冲突

```bash
docker rm -f t17-real
```

### 8.4 想强制停机

```bash
docker rm -f t17-real auto4508_real_ros
```

## 9. 怎么推送到 GitHub

在本机仓库根目录执行：

查看改动：

```bash
git status
```

添加需要提交的文件：

```bash
git add README.md docs/robot_deploy_runbook_zh.md src/
```

不要提交缓存文件：

```bash
git restore --staged src/auto4508_project/auto4508_project/__pycache__/safety_multiplexer.cpython-312.pyc 2>/dev/null || true
```

提交：

```bash
git commit -m "补充机器人部署运行手册并更新自动导航容错修复"
```

推送到 `main`：

```bash
git push origin main
```

## 10. 最短现场流程

```bash
cd /home/team17/auto4508-group-project-2026
rsync -avz --delete --exclude '.git' --exclude 'build' --exclude 'install' --exclude 'log' --exclude '__pycache__' --exclude '*.pyc' ./ team17@<ROBOT_IP>:/home/team17/auto4508-group-project-2026/
ssh team17@<ROBOT_IP>
cd /home/team17/auto4508-group-project-2026
docker rm -f t17-real
./start_robot_docker.sh --mode build --name t17-build -- --packages-select p3at_bringup auto4508_project
./start_robot_docker.sh --mode real
docker exec -it t17-real bash
```
