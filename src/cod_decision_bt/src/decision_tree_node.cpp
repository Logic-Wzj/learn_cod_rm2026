// 高层决策行为树运行节点：加载决策树 XML，按周期 tick
// 树创建失败自动重试（等 bt_navigator 就绪）
#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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

    // ===== 全部可调旋钮（集中在一份 params 文件里改，XML 通过 ${key} 引用）=====
    declare_parameter<double>("hp_threshold", 0.3);
    declare_parameter<double>("retreat_x", 0.2);
    declare_parameter<double>("retreat_y", 0.3);
    declare_parameter<bool>("patrol_use_spawn_pose", false);
    declare_parameter<std::string>(
      "patrol_waypoints",
      "5.2,0.0; 4.3,-6.3; 9.8,-6.5; 9.9,-0.5; -0.2,-6.1; 0.2,0.3");
    declare_parameter<std::string>("goal_frame", "map");
    declare_parameter<std::string>("stop_cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("navigate_server_name", "/navigate_to_pose");
    declare_parameter<std::string>("game_status_topic", "/referee/game_status");
    declare_parameter<std::string>("robot_status_topic", "/referee/robot_status");

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

      // 旋钮参数注入黑板：XML 里用 ${hp_threshold} 这类写法引用
      blackboard_->set<double>("hp_threshold", get_parameter("hp_threshold").as_double());
      blackboard_->set<double>("retreat_x", get_parameter("retreat_x").as_double());
      blackboard_->set<double>("retreat_y", get_parameter("retreat_y").as_double());
      blackboard_->set<bool>(
        "patrol_use_spawn_pose", get_parameter("patrol_use_spawn_pose").as_bool());
      blackboard_->set<std::string>(
        "patrol_waypoints", get_parameter("patrol_waypoints").as_string());
      blackboard_->set<std::string>("goal_frame", get_parameter("goal_frame").as_string());
      blackboard_->set<std::string>(
        "stop_cmd_vel_topic", get_parameter("stop_cmd_vel_topic").as_string());
      blackboard_->set<std::string>(
        "navigate_server_name", get_parameter("navigate_server_name").as_string());
      blackboard_->set<std::string>(
        "game_status_topic", get_parameter("game_status_topic").as_string());
      blackboard_->set<std::string>(
        "robot_status_topic", get_parameter("robot_status_topic").as_string());

      tree_ = std::make_unique<BT::Tree>(bt_engine_->createTreeFromFile(bt_xml_, blackboard_));
      tree_created_ = true;

      // Groot Monitor 实时监控（ZMQ 端口 1666）
      zmq_pub_ = std::make_unique<BT::PublisherZMQ>(*tree_);
      RCLCPP_INFO(get_logger(), "Groot 实时监控已开启（端口 1666）");

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

  std::string bt_xml_;
  std::vector<std::string> plugin_libraries_;
  std::chrono::milliseconds bt_loop_duration_{100};
  std::chrono::milliseconds server_timeout_{1000};
  std::chrono::milliseconds wait_for_service_timeout_{1000};
  bool tree_created_ = false;
  BT::NodeStatus last_status_ = BT::NodeStatus::IDLE;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_engine_;
  std::unique_ptr<BT::Tree> tree_;
  std::unique_ptr<BT::PublisherZMQ> zmq_pub_;

  BT::Blackboard::Ptr blackboard_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DecisionTreeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
