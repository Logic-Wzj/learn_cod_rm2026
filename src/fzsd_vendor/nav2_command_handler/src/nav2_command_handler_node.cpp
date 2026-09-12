#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class NavCommandMapper : public rclcpp::Node
{
public:
  NavCommandMapper() : Node("nav_command_mapper")
  {
    // 订阅 /nav2_command 话题，使用Int8类型
    nav_command_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/nav2_command", rclcpp::SensorDataQoS(),
      std::bind(&NavCommandMapper::nav_command_callback, this, std::placeholders::_1));

    goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/red_standard_robot1/goal_pose", 10);

    RCLCPP_INFO(this->get_logger(), "NavCommandMapper 节点已启动");
  }

private:
  // 修改回调函数参数类型为Int8
  void nav_command_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "收到命令: %u", msg->data);

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->get_clock()->now();
    goal_pose.header.frame_id = "map";

    // 使用整数比较代替字符串比较
     if (msg->data == 0) {
      goal_pose.pose.position.x = 4.598969459533691;
      goal_pose.pose.position.y = -3.047443151473999;
      goal_pose.pose.position.z = 0.003662109375;

    } else if (msg->data == 1) {
      goal_pose.pose.position.x = -0.30235517024993896;
      goal_pose.pose.position.y = 0.01861780881881714;
      goal_pose.pose.position.z = 0.0014929771423339844;

    } else if (msg->data == 2) {
      goal_pose.pose.position.x = 4.604015350341797;
      goal_pose.pose.position.y = -3.9705653190612793;
      goal_pose.pose.position.z = -0.00054168701171875;

    
    } else if (msg->data == 3) {
      goal_pose.pose.position.x = 5.506138801574707;
      goal_pose.pose.position.y = -1.6566252708435059;
      goal_pose.pose.position.z = 0.0003814697265625;
    
    } else if (msg->data == 4) {
      goal_pose.pose.position.x = 2.8689160346984863;
      goal_pose.pose.position.y = -4.21600341796875;
      goal_pose.pose.position.z = -0.002277374267578125;
    
    } else if (msg->data == 5) {
      goal_pose.pose.position.x = 4.744440078735352;
      goal_pose.pose.position.y = -0.5410155057907104;
      goal_pose.pose.position.z = -0.000370025634765625;

    } else {
      RCLCPP_WARN(this->get_logger(), "收到未知命令: %d", msg->data);
      return;
    }

    // 统一设置方向参数
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "发布目标坐标：position(%.6f, %.6f, %.6f)",
                goal_pose.pose.position.x, 
                goal_pose.pose.position.y, 
                goal_pose.pose.position.z);

    goal_pose_publisher_->publish(goal_pose);
  }

  // 使用正确的Int8类型定义订阅者
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_command_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavCommandMapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
