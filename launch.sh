#!/bin/zsh
# 启动 fzsd Gazebo 完整仿真（cod 导航栈 + fzsd 仿真世界）
# 用法：./launch.sh   （首次需 chmod +x launch.sh）

# 清理残留的 Gazebo/导航进程：
# 上次 Ctrl+C 后 ign gazebo 子进程可能没死干净，旧世界（含旧机器人位姿）
# 会一直活着，导致这次 spawn 跑进旧世界、位置停留在上次。
# 注意：pkill 无匹配时返回非 0，需 || true 吞掉，否则 set -e 会中断脚本
pkill -9 -f "ign gazebo" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ros_gz" 2>/dev/null || true
pkill -9 -f "component_container" 2>/dev/null || true
pkill -9 -f "gt_odom" 2>/dev/null || true
pkill -9 -f "static_transform" 2>/dev/null || true
sleep 2

set -e  # 环境 source 失败立即退出

source /opt/ros/humble/setup.zsh
source ~/fzsd2025/install/setup.zsh
source ~/cod_-rm2026_-navigation/install/setup.zsh

ros2 launch cod_sim fzsd_sim_launch.py
