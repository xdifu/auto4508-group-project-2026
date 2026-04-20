# AUTO4508 Group Project - Team 17 🤖

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Pioneer](https://img.shields.io/badge/Robot-Pioneer_3AT-green)

*(English version is located below the Chinese version / 英文版本在中文指南下方)*

## 🇨🇳 中文真机部署指南 (Chinese Guide)

### 环境配置与底层编译 (Environment Setup)
1. **获取最新代码与专属容器**  
   你需要通过 SSH 连接到机载电脑（如 `pioneer7`），然后在原本的终端中执行以下命令，即可一键化获取包含完整操作权限的无菌容器环境。
   ```bash
   cd ~/auto4508-group-project-2026
   git pull origin main
   # 这一步会瞬间拉起一个带有特权的脚本，如果你的容器环境缺失导航库，它会自动触发构建。
   ./start_robot_docker.sh
   ```
   *（如果你看到控制台瞬间变成了 `root@xxxx:/workspace#` ，说明你已经拥有了硬件最高权限，进入了开发无菌室！）*

2. **核心编译**  
   由于本架构解耦了硬件驱动、视觉探测与运动预测，你必须强制重新编译全部节点使其在新环境中映射生效：
   ```bash
   colcon build
   source install/setup.bash
   ```

### 运行小车与底层启动命令 (Run the Robot)

为了最大化安全性，我们采用**双开窗口监控**的模式，以便物理底层状态监测和高阶自动驾驶逻辑完全分开。

**窗口 A（硬件驱动与建图底层）:**
在刚刚执行完编译的无菌容器黑框里（`/workspace#`）：
```bash
# 第 1 步：手动拉起 Pioneer 的轮子电机硬件节点！
ros2 run ariaNode ariaNode -rp /dev/ttyUSB0 &

# 第 2 步：拉起蓝牙手柄的信号转换（这需要你们物理连接好手柄并确认电量充足）
ros2 run joy joy_node &

# 第 3 步：拉起雷达和建图导航框架！
ros2 launch p3at_bringup real.launch.py &
```

**窗口 B（真正的杰作：重构后的高纬度控制集群）:**
这个时候你需要新开一个全新的电脑终端（使用SSH重新登入这台车）。然后**钻进刚才已经在后台跑着驱动的心血容器 `t17-core` 中**：
```bash
docker exec -it t17-core bash
source install/setup.bash

# 启动超级安全多路复用控制器 (并喂给它防止各种硬编码错误的热更全局配置)
ros2 run auto4508_project safety_node --ros-args --params-file global_topics.yaml
```

### 实车测试验证
当你听到控制台成功输出 `Modular Safety Node Started` 后，恭喜你打通了最后一公里：
* 按下手柄的 `O` 键切换到 **MANUAL（手动） 模式**。此时你可以直接推动左右摇杆操控这台野兽，不需要按动其它按键。
* 按下手柄的 `X` 键切换为 **AUTO（自动） 模式**。此模式下代码仲裁器接管底盘，此时**必须死死按住手柄后背的 L1 (死人开关/Dead-man)**，如果有任何算法自动指令下发，车轮才会动；中途一旦手滑松开按键，瞬间触发全局最高优先级的急刹车指令切断动力，保障车和人的绝对安全！

---

## 🇬🇧 English Hardware Deployment Guide

### Environment Configuration
1. **Pull the Source Code & Bootstrap the Sandbox**  
   First, SSH into the onboard Linux computer (e.g., `pioneer7`), drop into your project directory, and let the robust script handle the environment variables and privileged hardware allocations:
   ```bash
   cd ~/auto4508-group-project-2026
   git pull origin main
   # Run the custom auto-detecting privileged docker launcher
   ./start_robot_docker.sh
   ```
   *(If your terminal smoothly changes to `root@xxxx:/workspace#`, congratulations, you have successfully assumed top-level root privileges to the USB hardware!)*

2. **Clean Compile the Workspace**  
   Every time we shift a robot machine or update a branch, a clean compile is vital.
   ```bash
   colcon build
   source install/setup.bash
   ```

### Starting the Operations

We enforce a Multi-Terminal workflow to safely observe hardware nodes individually from advanced algorithm behaviors.

**Window A (Hardware & Nav2 Backend):**
In your first running `/workspace#` terminal session:
```bash
# Step 1: Launch the Pioneer 3-AT serial/USB hardware connection node
ros2 run ariaNode ariaNode -rp /dev/ttyUSB0 &

# Step 2: Establish connection to the linux controller node (Ensure gamepad is active)
ros2 run joy joy_node &

# Step 3: Launch the LiDar logic and physical mapping framework
ros2 launch p3at_bringup real.launch.py &
```

**Window B (Logical Nodes Sandbox):**
Open a brand-new SSH session to the robot host, attach to our existing container, and execute our ultimate safety supervisor node:
```bash
docker exec -it t17-core bash
source install/setup.bash

# Fire up the multiplexer, dynamically loading parameters to avoid string hardcoding!
ros2 run auto4508_project safety_node --ros-args --params-file global_topics.yaml
```

### Verification & Hand-over Tests:
Once the multiplexer has launched perfectly, it's testing time: 
* Press the `O` key shortcut to invoke **MANUAL MODE**. Pushing the analog sticks around will instantly control the robot logic safely.
* Press the `X` key shortcut to invoke **AUTO MODE**. Inside this environment, the robot will **ONLY evaluate Nav2 inputs if you physically compress and hold the L1 "Dead-man switch" trigger button**. Releasing the L1 trigger will instantly override all `/cmd_vel` transmissions, injecting `0.0` linear force to stop the robot dead in its tracks. Safety first!
