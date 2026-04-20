#!/bin/bash
# 一键启动带真机硬件控制权限的 Docker 容器 (自动适配各种先锋机器人!)

echo "🚀 起飞！启动 Team 17 专属最高权限太空舱..."

# 如果用户在执行时手动传入了名字，比如 ./start_robot_docker.sh my_image，则优先采用手动名字
if [ -n "$1" ]; then
    IMAGE_NAME="$1"
    echo "✅ 使用您手动指定的底层引擎: $IMAGE_NAME"
else
    # 自适应架构：自动嗅探这台物理机里装了哪款老底子镜像
    if docker image inspect docker-t5_paneer:latest >/dev/null 2>&1; then
        IMAGE_NAME="docker-t5_paneer:latest"
    elif docker image inspect mobrob-ros2-ros2:latest >/dev/null 2>&1; then
        IMAGE_NAME="mobrob-ros2-ros2:latest"
    else
        IMAGE_NAME="mobrob-ros2-ros2:latest"
        echo "⚠️ 未能自动侦测到已知的课程镜像，盲猜使用: $IMAGE_NAME"
    fi
    echo "✅ 自动探测并选中底层引擎: $IMAGE_NAME"
fi

echo "正在将你们所有的代码装载进入 $IMAGE_NAME 的无菌舱..."

docker run -it --rm \
    --name t17-core \
    --privileged \
    --network host \
    -v /dev:/dev \
    -v ~/auto4508-group-project-2026:/workspace \
    -w /workspace \
    $IMAGE_NAME bash
