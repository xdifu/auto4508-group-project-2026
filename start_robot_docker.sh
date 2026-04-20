#!/bin/bash
# 一键启动带真机硬件控制权限的 Docker 容器

echo "🚀 起飞！启动 Team 17 专属最高权限太空舱..."

docker run -it --rm \
    --name t17-core \
    --privileged \
    --network host \
    -v /dev:/dev \
    -v ~/auto4508-group-project-2026:/workspace \
    -w /workspace \
    mobrob-ros2-ros2:latest bash
