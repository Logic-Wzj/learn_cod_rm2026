// 条件节点：游戏是否处于 RUNNING 阶段
#ifndef COD_DECISION_BT__CONDITIONS__IS_GAME_RUNNING_CONDITION_HPP_
#define COD_DECISION_BT__CONDITIONS__IS_GAME_RUNNING_CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cod_referee_interfaces/msg/game_status.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace cod_decision_bt
{

class IsGameRunningCondition : public BT::ConditionNode
{
public:
  IsGameRunningCondition(const std::string & name, const BT::NodeConfiguration & conf);
  IsGameRunningCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "game_status_topic", "/referee/game_status", "裁判系统游戏状态话题"),
    };
  }

private:
  void game_status_callback(const cod_referee_interfaces::msg::GameStatus::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<cod_referee_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;

  std::mutex mutex_;
  uint8_t stage_ = 0;
  bool data_received_ = false;
};

}  // namespace cod_decision_bt

#endif  // COD_DECISION_BT__CONDITIONS__IS_GAME_RUNNING_CONDITION_HPP_
