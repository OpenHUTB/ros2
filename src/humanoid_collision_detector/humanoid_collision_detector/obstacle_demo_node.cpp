/**
 * @file obstacle_demo_node.cpp
 * @brief 演示用障碍物位置发布节点（正弦轨迹模拟移动障碍物）
 *
 * 用法: ros2 run humanoid_collision_detector obstacle_demo_node
 * 话题: 发布 /obstacle_position (geometry_msgs/PointStamped)
 */
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;

class ObstacleDemoNode : public rclcpp::Node
{
public:
  ObstacleDemoNode()
  : Node("obstacle_demo_node"), t_(0.0)
  {
    this->declare_parameter<double>("origin_x", 0.85);
    this->declare_parameter<double>("origin_y", 0.0);
    this->declare_parameter<double>("origin_z", 1.15);
    this->declare_parameter<double>("amplitude", 0.55);
    this->declare_parameter<double>("frequency", 1.0);
    this->declare_parameter<double>("publish_hz", 20.0);
    this->declare_parameter<bool>("enabled", true);

    // 使用默认 QoS (reliable), 与 collision_detector_node 的订阅端保持一致
    // 注意: best_effort 发布者与 reliable 订阅者在 ROS2 中 QoS 不兼容, 会导致收不到数据
    pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/obstacle_position", 10);

    double hz = this->get_parameter("publish_hz").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / hz),
      std::bind(&ObstacleDemoNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "障碍物演示节点已启动 - 原点(%.1f,%.1f,%.1f), 振幅%.2f, 频率%.2fHz",
      this->get_parameter("origin_x").as_double(),
      this->get_parameter("origin_y").as_double(),
      this->get_parameter("origin_z").as_double(),
      this->get_parameter("amplitude").as_double(),
      this->get_parameter("frequency").as_double());
  }

private:
  void timerCallback()
  {
    bool enabled = this->get_parameter("enabled").as_bool();
    if (!enabled) return;

    t_ += 0.05;
    double amp = this->get_parameter("amplitude").as_double();
    double freq = this->get_parameter("frequency").as_double();
    double ox = this->get_parameter("origin_x").as_double();
    double oy = this->get_parameter("origin_y").as_double();
    double oz = this->get_parameter("origin_z").as_double();

    auto msg = geometry_msgs::msg::PointStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";
    msg.point.x = ox + std::sin(t_ * freq) * amp;
    msg.point.y = oy + std::cos(t_ * freq * 0.7) * (amp * 0.4);
    msg.point.z = oz + std::sin(t_ * freq * 0.5) * 0.2;

    pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDemoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
