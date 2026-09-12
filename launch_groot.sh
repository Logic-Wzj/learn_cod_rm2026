#!/bin/bash
# 打开 Groot 连接决策树实时监控（ZMQ 端口 1666）
# 用法：./launch_groot.sh   （需先起决策树）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FZSD_DIR="${FZSD_DIR:-$HOME/fzsd2025}"

source /opt/ros/humble/setup.bash
[ -d "$FZSD_DIR/install" ] && source "$FZSD_DIR/install/setup.bash"
source "$SCRIPT_DIR/install/setup.bash"

groot
