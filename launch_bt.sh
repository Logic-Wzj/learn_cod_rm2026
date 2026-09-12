#!/bin/bash
# 仿真：gazebo + 裁判系统 + 决策树
# 用法：./launch_bt.sh
# 说明：仿真已自足（fzsd 仿真包已 vendored 到 src/fzsd_vendor），无需外部 fzsd2025。
#       FZSD_DIR 存在时仍会叠加（实车/兼容用），不存在则跳过。
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FZSD_DIR="${FZSD_DIR:-$HOME/fzsd2025}"

source /opt/ros/humble/setup.bash
[ -d "$FZSD_DIR/install" ] && source "$FZSD_DIR/install/setup.bash"
source "$SCRIPT_DIR/install/setup.bash"

ros2 launch cod_sim rm_decision_launch.py
