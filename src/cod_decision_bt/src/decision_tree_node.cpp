// 高层决策行为树运行节点：加载决策树 XML，按周期 tick
// 树创建失败自动重试（等 bt_navigator 就绪）；启动时缓存出生点到黑板
#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "behaviortree_cpp_v3/blackboard.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/basic_types.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "nav2_behavior_tree/behavior_tree_engine.hpp"

class DecisionTreeNode : public rclcpp::Node
{
public:
  DecisionTreeNode() : Node("decision_tree_node")
  {
    declare_parameter<std::string>("bt_xml", "");
    declare_parameter<std::vector<std::string>>(
      "plugin_libraries", std::vector<std::string>{});
    declare_parameter<int>("bt_loop_duration_ms", 100);
    declare_parameter<int>("server_timeout_ms", 1000);
    declare_parameter<int>("wait_for_service_timeout_ms", 1000);

    bt_xml_ = get_parameter("bt_xml").as_string();
    plugin_libraries_ = get_parameter("plugin_libraries").as_string_array();
    bt_loop_duration_ = std::chrono::milliseconds(get_parameter("bt_loop_duration_ms").as_int());
    server_timeout_ = std::chrono::milliseconds(get_parameter("server_timeout_ms").as_int());
    wait_for_service_timeout_ = std::chrono::milliseconds(
      get_parameter("wait_for_service_timeout_ms").as_int());

    if (bt_xml_.empty()) {
      RCLCPP_ERROR(get_logger(), "未配置 bt_xml 参数");
      return;
    }

    init_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&DecisionTreeNode::try_create_tree, this));
  }

private:
  void try_create_tree()
  {
    if (tree_created_) {
      return;
    }

    try {
      bt_engine_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_libraries_);

      // 初始化黑板（与 bt_navigator 相同的键）
      blackboard_ = BT::Blackboard::create();
      blackboard_->set<rclcpp::Node::SharedPtr>("node", shared_from_this());
      blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
      blackboard_->set<std::chrono::milliseconds>("server_timeout", server_timeout_);
      blackboard_->set<std::chrono::milliseconds>(
        "wait_for_service_timeout", wait_for_service_timeout_);
      blackboard_->set<geometry_msgs::msg::PoseStamped>(
        "goal", geometry_msgs::msg::PoseStamped());

      tree_ = std::make_unique<BT::Tree>(bt_engine_->createTreeFromFile(bt_xml_, blackboard_));
      tree_created_ = true;

      // Groot Monitor 实时监控（ZMQ 端口 1666）
      zmq_pub_ = std::make_unique<BT::PublisherZMQ>(*tree_);
      RCLCPP_INFO(get_logger(), "Groot 实时监控已开启（端口 1666）");

      // 启动时缓存出生点（机器人还没动），供 SetRetreatGoal/PatrolNextWaypoint 使用
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      spawn_cache_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&DecisionTreeNode::cache_spawn_pose, this));

      tick_timer_ = create_wall_timer(
        bt_loop_duration_,
        std::bind(&DecisionTreeNode::tick_tree, this));

      RCLCPP_INFO(get_logger(), "决策树创建成功: %s", bt_xml_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "决策树创建失败，0.5s 后重试: %s", e.what());
    }
  }

  void tick_tree()
  {
    if (!tree_created_) {
      return;
    }

    auto status = tree_->tickRoot();

    if (status != last_status_) {
      last_status_ = status;
      RCLCPP_INFO(get_logger(), "决策树状态: %s", BT::toStr(status).c_str());
    }

    // 树跑完一个决策周期后复位，下一 tick 重新决策
    if (BT::StatusCompleted(status)) {
      bt_engine_->haltAllActions(tree_->rootNode());
    }
  }

  // 树启动时把机器人出生点（map 系）缓存到黑板，TF 未就绪则重试
  void cache_spawn_pose()
  {
    if (spawn_cached_ || !tree_created_) {
      return;
    }

    try {
      auto tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped spawn;
      spawn.header.frame_id = "map";
      spawn.header.stamp = now();
      spawn.pose.position.x = tf.transform.translation.x;
      spawn.pose.position.y = tf.transform.translation.y;
      spawn.pose.orientation.w = 1.0;

      blackboard_->set<geometry_msgs::msg::PoseStamped>("spawn_pose", spawn);
      spawn_cached_ = true;
      RCLCPP_INFO(
        get_logger(), "出生点已缓存到黑板: (%.2f, %.2f)",
        spawn.pose.position.x, spawn.pose.position.y);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "出生点 TF 未就绪，0.5s 后重试: %s", e.what());
    }
  }

  std::string bt_xml_;
  std::vector<std::string> plugin_libraries_;
  std::chrono::milliseconds bt_loop_duration_{100};
  std::chrono::milliseconds server_timeout_{1000};
  std::chrono::milliseconds wait_for_service_timeout_{1000};
  bool tree_created_ = false;
  BT::NodeStatus last_status_ = BT::NodeStatus::IDLE;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr spawn_cache_timer_;
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_engine_;
  std::unique_ptr<BT::Tree> tree_;
  std::unique_ptr<BT::PublisherZMQ> zmq_pub_;

  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool spawn_cached_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DecisionTreeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
