#!/bin/bash
# 一键起飞：自动构建带 Nav2 和硬件特权的 Team 17 专属环境

echo "🚀 起飞！启动 Team 17 专属最高权限太空舱..."

# 1. 自适应架构：自动嗅探基础镜像
if [ -n "$1" ]; then
    BASE_IMAGE="$1"
    echo "✅ 使用您手动指定的基础引擎: $BASE_IMAGE"
else
    if docker image inspect docker-t5_paneer:latest >/dev/null 2>&1; then
        BASE_IMAGE="docker-t5_paneer:latest"
    elif docker image inspect team9_pioneer_v2:latest >/dev/null 2>&1; then
        BASE_IMAGE="team9_pioneer_v2:latest"
    elif docker image inspect mobrob-ros2-ros2:latest >/dev/null 2>&1; then
        BASE_IMAGE="mobrob-ros2-ros2:latest"
    else
        BASE_IMAGE="mobrob-ros2-ros2:latest"
        echo "⚠️ 未能自动侦测到已知的课程镜像，盲猜使用: $BASE_IMAGE"
    fi
    echo "✅ 自动探测并选中基础引擎: $BASE_IMAGE"
fi

# 2. 只有在第一次启动或依赖改变时，这步才会消耗时间（之后全部秒进缓存）
echo "🛠️ 正在基于 ${BASE_IMAGE} 为 Team 17 烤制永久的导航库..."
docker build --build-arg BASE_IMAGE=${BASE_IMAGE} -t auto4508_team17_env -f Dockerfile.team17 .

echo "正在将你们所有的代码装载进入 Team 17 的无菌舱..."

# 3. 运行我们的专属长效镜像
docker run -it --rm \
    --name t17-core \
    --privileged \
    --network host \
    -v /dev:/dev \
    -v ~/auto4508-group-project-2026:/workspace \
    -w /workspace \
    auto4508_team17_env bash
