#!/bin/bash
# PRIEST 仿真测试
# 用法：./launch_priest.sh
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FZSD_DIR="${FZSD_DIR:-$HOME/fzsd2025}"

source /opt/ros/humble/setup.bash
[ -d "$FZSD_DIR/install" ] && source "$FZSD_DIR/install/setup.bash"
source "$SCRIPT_DIR/install/setup.bash"

ros2 launch cod_sim priest_launch.py
