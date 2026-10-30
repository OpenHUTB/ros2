#include "humanoid_collision_detector/collision_detector.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"

using namespace std::chrono_literals;

namespace humanoid_collision_detector
{

/**
 * @brief 人形机器人碰撞体边界检测ROS2节点
 *
 * 功能:
 *   1. 从robot_description参数或URDF文件加载碰撞体描述
 *   2. 通过TF实时获取各碰撞体的世界变换
 *   3. 接收障碍物位置 (geometry_msgs/PointStamped)
 *   4. 计算最小边界距离并发布检测结果
 *   5. 生成RViz2可视化Marker
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
  : Node("humanoid_collision_detector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ========== 参数声明 ==========
    this->declare_parameter<std::string>("urdf_file", "");
    this->declare_parameter<std::string>("base_frame", "world");
    this->declare_parameter<double>("safety_boundary", 0.15);
    this->declare_parameter<double>("obstacle_radius", 0.2);
    this->declare_parameter<double>("check_frequency", 100.0);
    this->declare_parameter<bool>("use_robot_description", true);

    // ========== 初始化检测器 ==========
    // 优先尝试 URDF 加载（如果有 urdf 库），否则回退到默认 12 个碰撞体
    bool loaded = false;

    // 暂时跳过 URDF 加载（需 ros-humble-urdf 包），直接用默认碰撞体
    // 后续在有完整 ROS2 环境时可以打开下面的条件分支
    RCLCPP_WARN(this->get_logger(),
      "URDF 自动加载暂未启用，使用默认 12 个碰撞体作为演示");
    loadDefaultCollisionBodies();
    loaded = true;

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

    // 障碍物默认位置（可通过参数设置或话题覆盖）
    default_obstacle_.x() = 1.2;
    default_obstacle_.y() = 0.0;
    default_obstacle_.z() = 1.0;

    // ========== 定时检测循环 ==========
    double freq = this->get_parameter("check_frequency").as_double();
    auto period = std::chrono::duration<double>(1.0 / freq);
    check_timer_ = this->create_wall_timer(
      period,
      std::bind(&CollisionDetectorNode::checkLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "人形机器人碰撞体边界检测节点已启动 - %zu 个碰撞体, %.1f Hz",
      detector_.getCollisionBodies().size(), freq);

    RCLCPP_INFO(this->get_logger(),
      "安全边界: %.3f m, 障碍物半径: %.3f m",
      this->get_parameter("safety_boundary").as_double(),
      this->get_parameter("obstacle_radius").as_double());
  }

private:
  void loadDefaultCollisionBodies()
  {
    // 默认12个碰撞体（简化人形）
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
    // 尝试从TF更新各碰撞体的世界位置
    // 注意: TF 未就绪时只探测一次, 避免 100Hz 循环反复打印 tf2 warning 刷爆日志
    std::string base_frame = this->get_parameter("base_frame").as_string();
    std::unordered_map<std::string, Eigen::Isometry3d> transforms;

    if (!tf_probed_) {
      tf_probe_counter_++;
      // 给 TF 前 3 秒的发布时间去建立, 3 秒后做一次最终判断
      if (tf_probe_counter_ > static_cast<int>(3.0 * this->get_parameter("check_frequency").as_double())) {
        tf_available_ = tf_buffer_.canTransform(base_frame, "torso", tf2::TimePointZero);
        tf_probed_ = true;
        RCLCPP_INFO(this->get_logger(), "TF 探测结果: %s (base_frame='%s')",
          tf_available_ ? "可用, 使用TF位姿" : "不可用, 使用默认碰撞体位姿", base_frame.c_str());
      }
    }

    for (const auto & body : detector_.getCollisionBodies()) {
      // 默认单位变换: 机器人基座位于世界原点时, 碰撞体世界坐标 = 本地偏移
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      if (tf_available_) {
        try {
          geometry_msgs::msg::TransformStamped ts =
            tf_buffer_.lookupTransform(base_frame, body.name, tf2::TimePointZero);
          tf = tf2::transformToEigen(ts);
        } catch (const tf2::TransformException &) {
          // 单个 link 缺失时保留单位变换
        }
      }
      transforms[body.name] = tf;
    }

    detector_.updateTransforms(transforms);

    // 执行检测
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

    // 定期日志输出 (每~20次循环输出一次)
    static int log_counter = 0;
    if (++log_counter % 20 == 0) {
      const char * risk_text[] = {"SAFE", "WARNING", "DANGER", "COLLIDED"};
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[%s] min_dist=%.3fm closest='%s' obs=(%.2f,%.2f,%.2f)",
        risk_text[result.risk_level],
        result.min_distance,
        result.closest_body.c_str(),
        default_obstacle_.x(), default_obstacle_.y(), default_obstacle_.z());
    }
  }

  CollisionDetector detector_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool tf_probed_{false};
  bool tf_available_{false};
  int tf_probe_counter_{0};

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
