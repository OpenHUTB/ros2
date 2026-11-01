#include "humanoid_collision_detector/collision_detector.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace humanoid_collision_detector
{

/**
 * @brief 人形机器人碰撞体边界检测ROS2节点
 *
 * 功能:
 *   1. 加载 12 个静态简化碰撞体
 *   2. 接收障碍物位置 (geometry_msgs/PointStamped)
 *   3. 计算最小边界距离并发布检测结果
 *   4. 生成RViz2可视化Marker
 *
 * 话题:
 *   订阅:
 *     /obstacle_position   (geometry_msgs/PointStamped)
 *   发布:
 *     /collision_boundary   (visualization_msgs/MarkerArray)
 *     /distance_to_obstacle (std_msgs/Float64)
 *     /collision_detected   (std_msgs/Bool)
 */
class CollisionDetectorNode : public rclcpp::Node
{
public:
  CollisionDetectorNode()
  : Node("humanoid_collision_detector")
  {
    // ========== 参数声明 ==========
    this->declare_parameter<double>("safety_boundary", 0.15);
    this->declare_parameter<double>("obstacle_radius", 0.2);
    this->declare_parameter<double>("check_frequency", 100.0);

    // ========== 初始化检测器 ==========
    loadDefaultCollisionBodies();

    // ========== 创建发布者和订阅者 ==========
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/collision_boundary", 10);

    distance_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/distance_to_obstacle", 10);

    collision_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/collision_detected", 10);

    obstacle_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/obstacle_position", 10,
      std::bind(&CollisionDetectorNode::obstacleCallback, this, std::placeholders::_1));

    // 障碍物默认位置（可通过话题覆盖）
    default_obstacle_ = Eigen::Vector3d(0.85, 0.0, 1.15);

    // ========== 定时检测循环 ==========
    double freq = this->get_parameter("check_frequency").as_double();
    auto period = std::chrono::duration<double>(1.0 / freq);
    check_timer_ = this->create_wall_timer(
      period,
      std::bind(&CollisionDetectorNode::checkLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "人形机器人碰撞体边界检测节点已启动 - %zu 个静态碰撞体, %.1f Hz",
      detector_.getCollisionBodies().size(), freq);

    RCLCPP_INFO(this->get_logger(),
      "安全边界: %.3f m, 障碍物半径: %.3f m",
      this->get_parameter("safety_boundary").as_double(),
      this->get_parameter("obstacle_radius").as_double());
  }

private:
  void loadDefaultCollisionBodies()
  {
    // 默认12个碰撞体（简化人形，世界坐标系 z 为高度方向）
    CollisionBody head, torso;
    head.name = "head"; head.type = CollisionType::BOX;
    head.size = Eigen::Vector3d(0.22, 0.22, 0.22);
    head.position = Eigen::Vector3d(0, 0, 1.60);
    head.rotation = Eigen::Quaterniond::Identity();

    torso.name = "torso"; torso.type = CollisionType::BOX;
    torso.size = Eigen::Vector3d(0.45, 0.25, 0.60);
    torso.position = Eigen::Vector3d(0, 0, 1.15);
    torso.rotation = Eigen::Quaterniond::Identity();

    detector_.addCollisionBody(head);
    detector_.addCollisionBody(torso);

    // 左右臂
    for (const std::string & side : {"L", "R"}) {
      double sign = (side == "L") ? -1.0 : 1.0;
      CollisionBody ua, fa;
      ua.name = side + "_upper_arm"; ua.type = CollisionType::CYLINDER;
      ua.size = Eigen::Vector3d(0.05, 0.35, 0);
      ua.position = Eigen::Vector3d(sign * 0.32, 0, 1.30);
      ua.rotation = Eigen::Quaterniond::Identity();

      fa.name = side + "_forearm"; fa.type = CollisionType::CYLINDER;
      fa.size = Eigen::Vector3d(0.04, 0.3, 0);
      fa.position = Eigen::Vector3d(sign * 0.45, 0, 0.95);
      fa.rotation = Eigen::Quaterniond::Identity();

      detector_.addCollisionBody(ua);
      detector_.addCollisionBody(fa);
    }

    // 左右腿
    for (const std::string & side : {"L", "R"}) {
      double sign = (side == "L") ? -1.0 : 1.0;
      CollisionBody th, sk, ft;
      th.name = side + "_thigh"; th.type = CollisionType::CYLINDER;
      th.size = Eigen::Vector3d(0.06, 0.45, 0);
      th.position = Eigen::Vector3d(sign * 0.13, 0, 0.72);
      th.rotation = Eigen::Quaterniond::Identity();

      sk.name = side + "_shank"; sk.type = CollisionType::CYLINDER;
      sk.size = Eigen::Vector3d(0.05, 0.35, 0);
      sk.position = Eigen::Vector3d(sign * 0.13, 0, 0.35);
      sk.rotation = Eigen::Quaterniond::Identity();

      ft.name = side + "_foot"; ft.type = CollisionType::BOX;
      ft.size = Eigen::Vector3d(0.14, 0.22, 0.06);
      ft.position = Eigen::Vector3d(sign * 0.13, 0.02, 0.05);
      ft.rotation = Eigen::Quaterniond::Identity();

      detector_.addCollisionBody(th);
      detector_.addCollisionBody(sk);
      detector_.addCollisionBody(ft);
    }
  }

  void obstacleCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    default_obstacle_.x() = msg->point.x;
    default_obstacle_.y() = msg->point.y;
    default_obstacle_.z() = msg->point.z;
  }

  void checkLoop()
  {
    double obstacle_radius = this->get_parameter("obstacle_radius").as_double();
    double safety_boundary = this->get_parameter("safety_boundary").as_double();

    DetectionResult result = detector_.detect(default_obstacle_, obstacle_radius, safety_boundary);

    // 发布距离
    std_msgs::msg::Float64 dist_msg;
    dist_msg.data = result.min_distance;
    distance_pub_->publish(dist_msg);

    // 发布碰撞状态
    std_msgs::msg::Bool col_msg;
    col_msg.data = result.collision_detected;
    collision_pub_->publish(col_msg);

    // 发布可视化Marker
    auto markers = detector_.generateBoundaryMarkers(safety_boundary, result);
    marker_pub_->publish(markers);

    // 定期日志输出
    static int log_counter = 0;
    if (++log_counter % 20 == 0) {
      const char * risk_text[] = {"SAFE", "WARNING", "DANGER", "COLLIDED"};
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[%s] min_dist=%.3fm closest='%s' obs=(%.2f,%.2f,%.2f) collision=%s",
        risk_text[result.risk_level],
        result.min_distance,
        result.closest_body.c_str(),
        default_obstacle_.x(), default_obstacle_.y(), default_obstacle_.z(),
        result.collision_detected ? "true" : "false");
    }
  }

  CollisionDetector detector_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  Eigen::Vector3d default_obstacle_;
};

}  // namespace humanoid_collision_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<humanoid_collision_detector::CollisionDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
