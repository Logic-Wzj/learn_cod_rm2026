#include "cod_decision_bt/actions/set_retreat_goal_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace cod_decision_bt
{

SetRetreatGoalAction::SetRetreatGoalAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus SetRetreatGoalAction::tick()
{
  double x = 0.0;
  double y = 0.0;
  std::string frame_id = "map";
  getInput("x", x);
  getInput("y", y);
  getInput("frame_id", frame_id);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = frame_id;
  goal.header.stamp = node_->now();
  goal.pose.orientation.w = 1.0;

  // 优先用启动时缓存的出生点（黑板上 spawn_pose）
  geometry_msgs::msg::PoseStamped spawn;
  if (config().blackboard->get<geometry_msgs::msg::PoseStamped>("spawn_pose", spawn)) {
    goal = spawn;
  } else {
    goal.pose.position.x = x;
    goal.pose.position.y = y;
  }

  config().blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal);

  RCLCPP_INFO(
    node_->get_logger(), "[SetRetreatGoal] 回补给点 (%.2f, %.2f)",
    goal.pose.position.x, goal.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace cod_decision_bt
