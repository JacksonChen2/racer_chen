#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  cat >&2 <<'EOF'
未检测到 ROS 2 Humble。
请先按照 ROS 官方说明在 Ubuntu 22.04 安装 ros-humble-desktop，
然后重新执行本脚本。Isaac Sim 5.1 也需要单独安装，压缩包不包含它。
EOF
  exit 2
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git \
  python3-colcon-common-extensions python3-rosdep python3-pip \
  python3-numpy python3-pytest \
  libeigen3-dev libjsoncpp-dev zlib1g-dev

if [[ ! -r /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

set +u
source /opt/ros/humble/setup.bash
set -u
for workspace in ros2_3d_C_ws ros2_3d_py_ws ros2_3d_sionna_ws; do
  rosdep install \
    --from-paths "${ROOT_DIR}/${workspace}/src" \
    --ignore-src -r -y
done

cat <<'EOF'
ROS 2 算法依赖安装完成。
接下来执行：
  ./racer doctor
  ./racer build all
通信版还需执行：
  ./racer setup comm
EOF
