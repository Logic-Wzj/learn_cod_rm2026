cmake_minimum_required(VERSION 3.8)
project(cod_decision_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# 自定义 BT 插件库（由 decision_tree_node 通过 registerFromPlugin 加载）
ament_auto_add_library(cod_decision_bt_nodes SHARED
  src/conditions/is_game_running_condition.cpp
  src/conditions/is_hp_low_condition.cpp
  src/actions/set_retreat_goal_action.cpp
  src/actions/stop_robot_action.cpp
  src/actions/patrol_next_waypoint_action.cpp
  src/register_nodes.cpp
)
target_compile_definitions(cod_decision_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

# 决策树运行节点
ament_auto_add_executable(decision_tree_node
  src/decision_tree_node.cpp
)

install(DIRECTORY behavior_trees config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
