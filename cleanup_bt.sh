#!/bin/bash
# 清理 COD 仿真进程，防止残留导致 TF/DDS 冲突
for pat in \
  "ros2 launch" \
  "ign gazebo" \
  "gzserver" \
  "rmua19_robot_base" \
  "decision_tree_node" \
  "referee_simulator_node" \
  "referee_keyboard" \
  "bt_navigator" \
  "controller_server" \
  "planner_server" \
  "velocity_smoother" \
  "map_server" \
  "rviz2" \
  "gt_odom" \
  "fake_vel_transform" \
  "robot_state_publisher" \
  "static_transform_publisher" \
  "tf_forward" \
  "urdf_global" \
  "pcd_publisher" \
  "priest_bridge" ; do
  pkill -f "$pat" 2>/dev/null
done
sleep 3
rm -f /dev/shm/sem.fastrtps_* /dev/shm/fastrtps_* 2>/dev/null
echo "清理完成"
