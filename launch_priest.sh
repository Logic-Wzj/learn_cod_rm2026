#!/bin/zsh
# PRIEST 仿真测试
# 用法：./launch_priest.sh
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FZSD_DIR="${FZSD_DIR:-$HOME/fzsd2025}"

source "$FZSD_DIR/install/setup.zsh"
source "$SCRIPT_DIR/install/setup.zsh"

ros2 launch cod_sim priest_launch.py
