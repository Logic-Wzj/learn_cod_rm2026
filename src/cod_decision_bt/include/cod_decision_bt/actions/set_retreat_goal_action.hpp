// 动作节点：把出生点写到黑板 goal 上，后续交给 Nav2 导航
// 出生点在树启动时由 decision_tree_node 缓存到黑板 spawn_pose
#ifndef COD_DECISION_BT__ACTIONS__SET_RETREAT_GOAL_ACTION_HPP_
#define COD_DECISION_BT__ACTIONS__SET_RETREAT_GOAL_ACTION_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace cod_decision_bt
{

class SetRetreatGoalAction : public BT::SyncActionNode
{
public:
  SetRetreatGoalAction(const std::string & name, const BT::NodeConfiguration & conf);
  SetRetreatGoalAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x", 0.0, "备用补给点 x 坐标（读不到出生点时用）"),
      BT::InputPort<double>("y", 0.0, "备用补给点 y 坐标（读不到出生点时用）"),
      BT::InputPort<std::string>("frame_id", "map", "目标坐标系"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace cod_decision_bt

#endif  // COD_DECISION_BT__ACTIONS__SET_RETREAT_GOAL_ACTION_HPP_
