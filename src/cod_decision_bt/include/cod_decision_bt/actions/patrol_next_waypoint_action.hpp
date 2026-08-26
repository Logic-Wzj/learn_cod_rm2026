// 动作节点：选下一个巡航点写到黑板 goal，后续交给 Nav2 导航
// 巡航点默认相对出生点（黑板 spawn_pose + 偏移），use_spawn_pose=false 时用绝对坐标
#ifndef COD_DECISION_BT__ACTIONS__PATROL_NEXT_WAYPOINT_ACTION_HPP_
#define COD_DECISION_BT__ACTIONS__PATROL_NEXT_WAYPOINT_ACTION_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace cod_decision_bt
{

class PatrolNextWaypointAction : public BT::SyncActionNode
{
public:
  PatrolNextWaypointAction(const std::string & name, const BT::NodeConfiguration & conf);
  PatrolNextWaypointAction() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "waypoints", "1,0; 3,0; 3,-1; 1,-1",
        "巡航点：分号分隔的 x,y（默认相对出生点偏移；use_spawn_pose=false 时是绝对坐标）"),
      BT::InputPort<bool>("use_spawn_pose", true, "巡航点相对出生点（黑板 spawn_pose）"),
      BT::InputPort<std::string>("map_frame", "map", "取出生点用的目标坐标系"),
      BT::InputPort<std::string>("spawn_frame", "base_footprint", "机器人根坐标系"),
      BT::InputPort<std::string>("frame_id", "map", "目标坐标系"),
    };
  }

private:
  void parse_waypoints(const std::string & s);
  bool lookup_spawn();

  rclcpp::Node::SharedPtr node_;

  std::vector<std::pair<double, double>> waypoints_;
  size_t waypoint_index_ = 0;

  bool spawn_cached_ = false;
  double spawn_x_ = 0.0;
  double spawn_y_ = 0.0;
};

}  // namespace cod_decision_bt

#endif  // COD_DECISION_BT__ACTIONS__PATROL_NEXT_WAYPOINT_ACTION_HPP_
