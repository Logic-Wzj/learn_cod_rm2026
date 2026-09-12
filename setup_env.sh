#!/bin/bash
# 一键配好本项目（含仿真，自足、不依赖外部 fzsd 工作区）
# 目标环境：Ubuntu 22.04 + ROS 2 Humble
#
# 用法：
#   cd <本仓库>
#   ./setup_env.sh              # 装依赖 + 编译
#   ./setup_env.sh --deps-only  # 只装依赖不编译
#
# 前置：已装好 ROS 2 Humble（apt 方式）。本脚本会 source 它。
set -e

# --- 0. 确认 ROS 2 Humble ---
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "!! 没找到 /opt/ros/humble —— 请先安装 ROS 2 Humble"; exit 1
fi
echo "ROS_DISTRO=$ROS_DISTRO"

# --- 1. 系统依赖 ---
echo "== 安装系统依赖 =="
APT_PKGS="git build-essential cmake curl \
    python3-pip python3-yaml python3-rosdep python3-colcon-common-extensions \
    libeigen3-dev libpcl-dev \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces \
    ros-humble-robot-state-publisher ros-humble-xacro ros-humble-joint-state-publisher \
    ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
    ros-humble-slam-toolbox ros-humble-rviz2 ros-humble-tf2-tools \
    ros-humble-pluginlib ros-humble-behaviortree-cpp-v3 ros-humble-message-filters \
    ros-humble-laser-geometry ros-humble-pcl-conversions ros-humble-pcl-ros \
    ros-humble-interactive-markers ros-humble-visualization-msgs ros-humble-std-srvs \
    ros-humble-nav-msgs ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs ros-humble-launch ros-humble-launch-ros \
    ros-humble-ament-index-python ros-humble-ament-cmake-auto ros-humble-ament-cmake-clang-tidy"

# Gazebo Fortress 已存在就跳过（不同版本是不同 apt 包，这里只判断 Fortress=ign gazebo）
if command -v ign >/dev/null 2>&1 && ign gazebo --version >/dev/null 2>&1; then
  echo "   检测到 ignition gazebo(Fortress) 已安装，跳过 ignition-fortress"
else
  APT_PKGS="$APT_PKGS ignition-fortress"
fi

sudo apt update
# shellcheck disable=SC2086
sudo apt install -y $APT_PKGS

# --- 2. pip 依赖（仿真 launch 需要 xmacro）---
echo "== 安装 python 依赖 =="
pip3 install --no-cache-dir xmacro==1.2.1

# --- 3. rosdep 补齐剩余依赖 ---
echo "== rosdep 安装剩余依赖 =="
if ! command -v rosdep >/dev/null 2>&1; then
  sudo apt install -y python3-rosdep
  sudo rosdep init 2>/dev/null || true
fi
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro humble || \
  echo "!! rosdep 有未解析项（多为可选），继续；编不过再单独处理"

# --- 4. 编译 ---
if [ "$1" = "--deps-only" ]; then
  echo "== 仅装依赖，跳过编译 =="; exit 0
fi
echo "== 编译 =="
colcon build --symlink-install

echo ""
echo "== 完成 =="
echo "  仿真:   source install/setup.bash && ros2 launch cod_sim rm_decision_launch.py"
echo "  决策层: source install/setup.bash && ros2 launch cod_decision_bt decision_tree.launch.py"
