#!/bin/zsh
# 仿真：fzsd_sim 完整链（gazebo + 裁判系统 + 决策树）
# 用法：./launch_bt.sh
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FZSD_DIR="${FZSD_DIR:-$HOME/fzsd2025}"

source "$FZSD_DIR/install/setup.zsh"
source "$SCRIPT_DIR/install/setup.zsh"

ros2 launch cod_sim rm_decision_launch.py
