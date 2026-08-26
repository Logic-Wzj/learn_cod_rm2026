#include "cod_decision_bt/actions/patrol_next_waypoint_action.hpp"

#include <cstdlib>
#include <sstream>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace cod_decision_bt
{

PatrolNextWaypointAction::PatrolNextWaypointAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

void PatrolNextWaypointAction::parse_waypoints(const std::string & s)
{
  waypoints_.clear();

  std::istringstream iss(s);
  std::string seg;
  while (std::getline(iss, seg, ';')) {
    size_t a = seg.find_first_not_of(" \t");
    size_t b = seg.find_last_not_of(" \t");
    if (a == std::string::npos) {
      continue;
    }
    seg = seg.substr(a, b - a + 1);

    size_t comma = seg.find(',');
    if (comma == std::string::npos) {
      continue;
    }
    double x = std::atof(seg.substr(0, comma).c_str());
    double y = std::atof(seg.substr(comma + 1).c_str());
    waypoints_.emplace_back(x, y);
  }
}

bool PatrolNextWaypointAction::lookup_spawn()
{
  if (spawn_cached_) {
    return true;
  }

  // 出生点在树启动时由 decision_tree_node 缓存到黑板 spawn_pose
  geometry_msgs::msg::PoseStamped spawn;
  if (config().blackboard->get<geometry_msgs::msg::PoseStamped>("spawn_pose", spawn)) {
    spawn_x_ = spawn.pose.position.x;
    spawn_y_ = spawn.pose.position.y;
    spawn_cached_ = true;
    return true;
  }

  RCLCPP_WARN(node_->get_logger(), "[PatrolNextWaypoint] 出生点未缓存");
  return false;
}

BT::NodeStatus PatrolNextWaypointAction::tick()
{
  bool use_spawn_pose = true;
  std::string waypoints_str = "1,0; 3,0; 3,-1; 1,-1";
  std::string frame_id = "map";
  getInput("use_spawn_pose", use_spawn_pose);
  getInput("waypoints", waypoints_str);
  getInput("frame_id", frame_id);

  if (waypoints_.empty()) {
    parse_waypoints(waypoints_str);
  }
  if (waypoints_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[PatrolNextWaypoint] 没有有效的巡航点");
    return BT::NodeStatus::FAILURE;
  }

  auto [wx, wy] = waypoints_[waypoint_index_];
  waypoint_index_ = (waypoint_index_ + 1) % waypoints_.size();

  double gx = wx;
  double gy = wy;

  if (use_spawn_pose) {
    if (!lookup_spawn()) {
      return BT::NodeStatus::FAILURE;
    }
    gx = spawn_x_ + wx;
    gy = spawn_y_ + wy;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = frame_id;
  goal.header.stamp = node_->now();
  goal.pose.position.x = gx;
  goal.pose.position.y = gy;
  goal.pose.orientation.w = 1.0;

  config().blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal);

  RCLCPP_INFO(
    node_->get_logger(), "[PatrolNextWaypoint] 巡航到 (%.2f, %.2f)", gx, gy);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace cod_decision_bt
