// patrol_collector_node.cpp — RViz Publish Point 收集巡航点，喂给决策树 Patrol
//
// 场地未知时不想手填坐标：在 RViz 里用 "Publish Point" 依次点巡航点，
// 本节点订阅 /clicked_point(PointStamped)，转成 map 系绝对坐标收集起来，
// 通过 /patrol_waypoints(PoseArray, transient_local) 发布给决策树；
// PatrolNextWaypoint 收到后就在这些点之间循环巡航。
// 同时用 Marker 在 RViz 里可视化已点出来的点。
//
// 用法：
//   ros2 run cod_decision_bt patrol_collector_node        # 先起
//   RViz 工具栏点 "Publish Point"，在地图上依次点巡航点
//   ros2 service call /patrol_collector/clear std_srvs/srv/Trigger    # 清空重来
//   ros2 service call /patrol_collector/remove_last std_srvs/srv/Trigger  # 删最后一个
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace cod_decision_bt
{

class PatrolCollectorNode : public rclcpp::Node
{
public:
  PatrolCollectorNode()
  : Node("patrol_collector_node")
  {
    declare_parameter<std::string>("clicked_topic", "/clicked_point");
    declare_parameter<std::string>("waypoints_topic", "/patrol_waypoints");
    declare_parameter<std::string>("marker_topic", "/patrol_waypoints_markers");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("service_prefix", "/patrol_collector");

    const std::string clicked_topic = get_parameter("clicked_topic").as_string();
    const std::string waypoints_topic = get_parameter("waypoints_topic").as_string();
    const std::string marker_topic = get_parameter("marker_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    const std::string svc = get_parameter("service_prefix").as_string();

    // transient_local：晚订阅的决策树也能立刻拿到已收集的点
    auto latched_qos = rclcpp::QoS(1).transient_local();

    waypoints_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      waypoints_topic, latched_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic, latched_qos);

    clicked_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_topic, 10,
      std::bind(&PatrolCollectorNode::on_clicked_point, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    clear_srv_ = create_service<std_srvs::srv::Trigger>(
      svc + "/clear", std::bind(
        &PatrolCollectorNode::on_clear, this, std::placeholders::_1, std::placeholders::_2));
    remove_last_srv_ = create_service<std_srvs::srv::Trigger>(
      svc + "/remove_last", std::bind(
        &PatrolCollectorNode::on_remove_last, this, std::placeholders::_1,
        std::placeholders::_2));

    // 启动即发布一次空列表：告诉决策树进入"实时点"模式（清空后没有点就先等着）
    publish_all();
    RCLCPP_INFO(
      get_logger(),
      "[PatrolCollector] 就绪：在 RViz 用 Publish Point 点巡航点，%s 发布 %d 点",
      waypoints_topic.c_str(), static_cast<int>(points_.size()));
  }

private:
  void on_clicked_point(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    double x = msg->point.x;
    double y = msg->point.y;

    // 统一转到 map 系（RViz 固定坐标系通常就是 map，这里做一层保险）
    if (msg->header.frame_id != map_frame_) {
      try {
        geometry_msgs::msg::PointStamped in = *msg;
        geometry_msgs::msg::PointStamped out;
        tf_buffer_->transform(in, out, map_frame_, tf2::durationFromSec(0.3));
        x = out.point.x;
        y = out.point.y;
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN(
          get_logger(), "[PatrolCollector] 忽略点击：转 %s->%s 失败 (%s)",
          msg->header.frame_id.c_str(), map_frame_.c_str(), e.what());
        return;
      }
    }

    points_.emplace_back(x, y);
    publish_all();
    RCLCPP_INFO(
      get_logger(), "[PatrolCollector] 点 %zu: (%.2f, %.2f)（累计 %zu 个）",
      points_.size(), x, y, points_.size());
  }

  bool on_clear(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    points_.clear();
    publish_all();
    resp->success = true;
    resp->message = "cleared";
    RCLCPP_INFO(get_logger(), "[PatrolCollector] 清空巡航点");
    return true;
  }

  bool on_remove_last(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    if (!points_.empty()) {
      points_.pop_back();
      publish_all();
    }
    resp->success = true;
    resp->message = points_.empty() ? "empty" : "ok";
    RCLCPP_INFO(
      get_logger(), "[PatrolCollector] 删除最后一个，剩 %zu 个", points_.size());
    return true;
  }

  void publish_all()
  {
    geometry_msgs::msg::PoseArray pa;
    pa.header.frame_id = map_frame_;
    pa.header.stamp = now();
    for (auto & [x, y] : points_) {
      geometry_msgs::msg::Pose p;
      p.position.x = x;
      p.position.y = y;
      p.position.z = 0.0;
      p.orientation.w = 1.0;
      pa.poses.push_back(p);
    }
    waypoints_pub_->publish(pa);

    // Marker 可视化：球 + 序号
    visualization_msgs::msg::MarkerArray ma;
    for (size_t i = 0; i < points_.size(); ++i) {
      auto [x, y] = points_[i];

      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = map_frame_;
      sphere.header.stamp = now();
      sphere.ns = "waypoint";
      sphere.id = static_cast<int>(i);
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = x;
      sphere.pose.position.y = y;
      sphere.pose.position.z = 0.15;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = 0.3;
      sphere.scale.y = 0.3;
      sphere.scale.z = 0.3;
      sphere.color.r = 0.0f;
      sphere.color.g = 1.0f;
      sphere.color.b = 0.0f;
      sphere.color.a = 0.9f;
      ma.markers.push_back(sphere);

      visualization_msgs::msg::Marker label;
      label.header.frame_id = map_frame_;
      label.header.stamp = now();
      label.ns = "label";
      label.id = static_cast<int>(i);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = x;
      label.pose.position.y = y;
      label.pose.position.z = 0.6;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.35;
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 1.0f;
      label.text = std::to_string(i + 1);
      ma.markers.push_back(label);
    }
    marker_pub_->publish(ma);
  }

  std::string map_frame_;
  std::vector<std::pair<double, double>> points_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_last_srv_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace cod_decision_bt

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cod_decision_bt::PatrolCollectorNode>());
  rclcpp::shutdown();
  return 0;
}
