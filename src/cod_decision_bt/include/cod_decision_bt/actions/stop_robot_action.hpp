// 动作节点：向底盘速度话题发布 0 速度，原地停车
#ifndef COD_DECISION_BT__ACTIONS__STOP_ROBOT_ACTION_HPP_
#define COD_DECISION_BT__ACTIONS__STOP_ROBOT_ACTION_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace cod_decision_bt
{

class StopRobotAction : public BT::SyncActionNode
{
public:
  StopRobotAction(const std::string & name, const BT::NodeConfiguration & conf);
  StopRobotAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "cmd_vel_topic", "/cmd_vel", "速度指令话题（发 0 速度停车）"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::string cmd_vel_topic_;
};

}  // namespace cod_decision_bt

#endif  // COD_DECISION_BT__ACTIONS__STOP_ROBOT_ACTION_HPP_
