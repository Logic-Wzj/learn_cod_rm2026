#include "cod_decision_bt/conditions/is_hp_low_condition.hpp"

namespace cod_decision_bt
{

IsHpLowCondition::IsHpLowCondition(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string topic = "/referee/robot_status";
  getInput("robot_status_topic", topic);

  robot_status_sub_ = node_->create_subscription<cod_referee_interfaces::msg::RobotStatus>(
    topic, 10,
    std::bind(&IsHpLowCondition::robot_status_callback, this, std::placeholders::_1));
}

void IsHpLowCondition::robot_status_callback(
  const cod_referee_interfaces::msg::RobotStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->max_hp > 0) {
    hp_ratio_ = static_cast<double>(msg->remain_hp) / msg->max_hp;
  }
  data_received_ = true;
}

BT::NodeStatus IsHpLowCondition::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!data_received_) {
    return BT::NodeStatus::FAILURE;
  }

  double threshold = 0.3;
  getInput("threshold", threshold);

  return hp_ratio_ < threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace cod_decision_bt
