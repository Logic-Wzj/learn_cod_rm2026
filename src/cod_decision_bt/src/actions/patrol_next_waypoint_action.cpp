#include "cod_decision_bt/actions/patrol_next_waypoint_action.hpp"

#include <cstdlib>
#include <sstream>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace cod_decision_bt
{

PatrolNextWaypointAction::PatrolNextWaypointAction(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // 订阅 Publish Point 收集器的实时巡航点（transient_local，晚到也能拿到已收集的点）
  std::string live_topic = "/patrol_waypoints";
  getInput("patrol_waypoints_topic", live_topic);
  auto latched_qos = rclcpp::QoS(10).transient_local();
  live_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    live_topic, latched_qos,
    std::bind(&PatrolNextWaypointAction::live_waypoints_cb, this, std::placeholders::_1));
}

void PatrolNextWaypointAction::live_waypoints_cb(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  // 收到过列表（哪怕空）就切到 live 模式：用现场点的点巡航，忽略 XML 里的静态点
  live_mode_ = true;
  live_points_.clear();
  for (const auto & p : msg->poses) {
    live_points_.emplace_back(p.position.x, p.position.y);
  }
  warned_empty_ = false;
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

  double gx = 0.0;
  double gy = 0.0;

  if (live_mode_) {
    // 现场 Publish Point 收集的点（map 绝对系）：优先巡航它们
    if (live_points_.empty()) {
      if (!warned_empty_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[PatrolNextWaypoint] 收集器在线但还没有点：在 RViz 里用 Publish Point 点巡航点");
        warned_empty_ = true;
      }
      return BT::NodeStatus::FAILURE;
    }
    auto [wx, wy] = live_points_[live_index_ % live_points_.size()];
    live_index_ = (live_index_ + 1) % live_points_.size();
    gx = wx;
    gy = wy;
  } else {
    // 静态 XML 巡航点（sim/无收集器时回退）
    if (waypoints_.empty()) {
      parse_waypoints(waypoints_str);
    }
    if (waypoints_.empty()) {
      RCLCPP_WARN(node_->get_logger(), "[PatrolNextWaypoint] 没有有效的巡航点");
      return BT::NodeStatus::FAILURE;
    }

    auto [wx, wy] = waypoints_[waypoint_index_];
    waypoint_index_ = (waypoint_index_ + 1) % waypoints_.size();

    gx = wx;
    gy = wy;

    if (use_spawn_pose) {
      if (!lookup_spawn()) {
        return BT::NodeStatus::FAILURE;
      }
      gx = spawn_x_ + wx;
      gy = spawn_y_ + wy;
    }
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
