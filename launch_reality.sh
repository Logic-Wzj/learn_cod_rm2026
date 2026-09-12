#!/bin/bash
# 实车"上层决策"一键启动：决策树 + 假裁判 + 巡航点收集器
#
# 依赖说明：
#   - 决策层(本脚本起的节点)只需要本仓库 + ROS nav2，不需要 fzsd。
#   - 导航栈(fzsd bringup，提供 /<ns>/navigate_to_pose 与 cmd_vel)必须在某台机器上跑，
#     且与本机 ROS 图互通(同一 ROS_DOMAIN_ID / 网段)。导航那台机器才需要 fzsd。
#
# 用法：
#   ./launch_reality.sh                                 # 默认
#   ./launch_reality.sh start_rviz:=true               # 一起开 RViz（可点巡航点）
#   ./launch_reality.sh start_keyboard:=false          # 无显示环境：键盘另开终端跑
#   FZSD_DIR=/path/to/fzsd2025 ./launch_reality.sh     # 本机有 fzsd 想叠加时指定
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FZSD_DIR="${FZSD_DIR:-$HOME/fzsd2025}"

source /opt/ros/humble/setup.bash
if [ -d "$FZSD_DIR/install" ]; then
  source "$FZSD_DIR/install/setup.bash"
else
  echo "[launch_reality] 未找到 fzsd 工作区: $FZSD_DIR —— 跳过（跑决策层不需要它）。"
fi
source "$SCRIPT_DIR/install/setup.bash"

ros2 launch cod_decision_bt decision_tree_reality.launch.py "$@"
