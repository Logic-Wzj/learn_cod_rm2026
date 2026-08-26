#include "cod_decision_bt/conditions/is_game_running_condition.hpp"

namespace cod_decision_bt
{

IsGameRunningCondition::IsGameRunningCondition(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string topic = "/referee/game_status";
  getInput("game_status_topic", topic);

  game_status_sub_ = node_->create_subscription<cod_referee_interfaces::msg::GameStatus>(
    topic, 10,
    std::bind(&IsGameRunningCondition::game_status_callback, this, std::placeholders::_1));
}

void IsGameRunningCondition::game_status_callback(
  const cod_referee_interfaces::msg::GameStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  stage_ = msg->stage;
  data_received_ = true;
}

BT::NodeStatus IsGameRunningCondition::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!data_received_) {
    return BT::NodeStatus::FAILURE;
  }

  return stage_ == cod_referee_interfaces::msg::GameStatus::STAGE_RUNNING
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace cod_decision_bt
