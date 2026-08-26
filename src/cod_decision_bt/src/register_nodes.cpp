// 注册自定义 BT 节点到插件库（decision_tree_node 加载后生效）
#include <behaviortree_cpp_v3/bt_factory.h>

#include "cod_decision_bt/conditions/is_game_running_condition.hpp"
#include "cod_decision_bt/conditions/is_hp_low_condition.hpp"
#include "cod_decision_bt/actions/set_retreat_goal_action.hpp"
#include "cod_decision_bt/actions/stop_robot_action.hpp"
#include "cod_decision_bt/actions/patrol_next_waypoint_action.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<cod_decision_bt::IsGameRunningCondition>("IsGameRunning");
  factory.registerNodeType<cod_decision_bt::IsHpLowCondition>("IsHpLow");
  factory.registerNodeType<cod_decision_bt::SetRetreatGoalAction>("SetRetreatGoal");
  factory.registerNodeType<cod_decision_bt::StopRobotAction>("StopRobot");
  factory.registerNodeType<cod_decision_bt::PatrolNextWaypointAction>("PatrolNextWaypoint");
}
