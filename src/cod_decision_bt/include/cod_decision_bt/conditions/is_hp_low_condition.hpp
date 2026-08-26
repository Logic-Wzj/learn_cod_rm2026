// 条件节点：血量比例（remain_hp / max_hp）低于阈值
#ifndef COD_DECISION_BT__CONDITIONS__IS_HP_LOW_CONDITION_HPP_
#define COD_DECISION_BT__CONDITIONS__IS_HP_LOW_CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cod_referee_interfaces/msg/robot_status.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace cod_decision_bt
{

class IsHpLowCondition : public BT::ConditionNode
{
public:
  IsHpLowCondition(const std::string & name, const BT::NodeConfiguration & conf);
  IsHpLowCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("threshold", 0.3, "血量比例阈值，低于此值返回 SUCCESS"),
      BT::InputPort<std::string>(
        "robot_status_topic", "/referee/robot_status", "裁判系统机器人状态话题"),
    };
  }

private:
  void robot_status_callback(const cod_referee_interfaces::msg::RobotStatus::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<cod_referee_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;

  std::mutex mutex_;
  double hp_ratio_ = 1.0;
  bool data_received_ = false;
};

}  // namespace cod_decision_bt

#endif  // COD_DECISION_BT__CONDITIONS__IS_HP_LOW_CONDITION_HPP_
