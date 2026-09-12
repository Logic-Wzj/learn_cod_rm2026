cmake_minimum_required(VERSION 3.8)
project(pb2025_nav_bringup)

# 纯资源包：只装地图/配置等，不依赖 nav2/fzsd，编它只需 ament_cmake_auto。
# （去掉了原先的 BUILD_TESTING + ament_lint_auto 块，避免拉一堆 lint 依赖。）
find_package(ament_cmake_auto REQUIRED)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  map
  config
  behavior_trees
  rviz
)
