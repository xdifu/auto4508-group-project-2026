#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ARIACODA_SRC_DIR="${ARIACODA_SRC_DIR:-/tmp/AriaCoda}"
ARIACODA_PREFIX="/usr/local"

sudo apt install -y build-essential git
if ! sudo apt install -y ros2-testing-apt-source; then
  echo "ros2-testing-apt-source unavailable; continuing with existing apt sources"
fi
sudo apt update
sudo apt remove -y \
  "ros-${ROS_DISTRO}-depthai" \
  "ros-${ROS_DISTRO}-depthai-bridge" \
  "ros-${ROS_DISTRO}-depthai-descriptions" \
  "ros-${ROS_DISTRO}-depthai-examples" \
  "ros-${ROS_DISTRO}-depthai-filters" \
  "ros-${ROS_DISTRO}-depthai-ros" \
  "ros-${ROS_DISTRO}-depthai-ros-driver" \
  "ros-${ROS_DISTRO}-depthai-ros-msgs" || true
sudo apt install -y \
  "ros-${ROS_DISTRO}-depthai-ros-v3" \
  "ros-${ROS_DISTRO}-cv-bridge" \
  "ros-${ROS_DISTRO}-image-transport" \
  "ros-${ROS_DISTRO}-camera-info-manager" \
  "ros-${ROS_DISTRO}-joy" \
  "ros-${ROS_DISTRO}-teleop-twist-joy" \
  "ros-${ROS_DISTRO}-robot-localization" \
  "ros-${ROS_DISTRO}-navigation2" \
  "ros-${ROS_DISTRO}-nav2-bringup" \
  "ros-${ROS_DISTRO}-nav2-collision-monitor" \
  "ros-${ROS_DISTRO}-nav2-velocity-smoother" \
  "ros-${ROS_DISTRO}-slam-toolbox" \
  "ros-${ROS_DISTRO}-nav2-map-server" \
  "ros-${ROS_DISTRO}-nmea-navsat-driver" \
  "ros-${ROS_DISTRO}-phidgets-spatial" \
  "ros-${ROS_DISTRO}-sick-scan-xd" \
  python3-opencv

if [ ! -d "${ARIACODA_SRC_DIR}/.git" ]; then
  rm -rf "${ARIACODA_SRC_DIR}"
  git clone --depth 1 https://github.com/reedhedges/AriaCoda "${ARIACODA_SRC_DIR}"
fi

make -C "${ARIACODA_SRC_DIR}" -j"$(nproc)"
sudo make -C "${ARIACODA_SRC_DIR}" install-default install-utils
echo "${ARIACODA_PREFIX}/lib" | sudo tee /etc/ld.so.conf.d/ariacoda.conf >/dev/null
sudo ldconfig

grep -qxF "export ARIACODA_PREFIX=${ARIACODA_PREFIX}" "${HOME}/.bashrc" || echo "export ARIACODA_PREFIX=${ARIACODA_PREFIX}" >> "${HOME}/.bashrc"
grep -qxF 'export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}' "${HOME}/.bashrc" || echo 'export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}' >> "${HOME}/.bashrc"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
ros2 pkg prefix depthai_ros_driver_v3
ros2 pkg prefix teleop_twist_joy
ros2 pkg prefix nav2_collision_monitor
ros2 pkg prefix nmea_navsat_driver
ros2 pkg prefix phidgets_spatial
ros2 pkg prefix sick_scan_xd
test -f "${ARIACODA_PREFIX}/include/Aria/Aria.h"
ldconfig -p | grep -q "libAria"
