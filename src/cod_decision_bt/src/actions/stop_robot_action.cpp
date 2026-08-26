#include "cod_decision_bt/actions/stop_robot_action.hpp"

namespace cod_decision_bt
{

StopRobotAction::StopRobotAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  cmd_vel_topic_ = "/cmd_vel";
  getInput("cmd_vel_topic", cmd_vel_topic_);

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);
}

BT::NodeStatus StopRobotAction::tick()
{
  geometry_msgs::msg::Twist stop;
  cmd_vel_pub_->publish(stop);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace cod_decision_bt
