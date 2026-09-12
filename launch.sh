#!/bin/bash
# 启动 Gazebo 完整仿真（cod 导航栈 + 仿真世界）
# 用法：./launch.sh
# 说明：仿真已自足（fzsd 仿真包已 vendored），无需外部 fzsd2025；FZSD_DIR 存在则叠加。

# 清理残留的 Gazebo/导航进程（上次 Ctrl+C 后可能没死干净，旧世界会占位）
pkill -9 -f "ign gazebo" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ros_gz" 2>/dev/null || true
pkill -9 -f "component_container" 2>/dev/null || true
pkill -9 -f "gt_odom" 2>/dev/null || true
pkill -9 -f "static_transform" 2>/dev/null || true
sleep 2

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FZSD_DIR="${FZSD_DIR:-$HOME/fzsd2025}"

source /opt/ros/humble/setup.bash
if [ -d "$FZSD_DIR/install" ]; then
  source "$FZSD_DIR/install/setup.bash"
fi
source "$SCRIPT_DIR/install/setup.bash"

ros2 launch cod_sim fzsd_sim_launch.py
