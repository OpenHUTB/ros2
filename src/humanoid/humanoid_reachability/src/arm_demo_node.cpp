#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "humanoid_reachability/kinematics.hpp"

namespace humanoid_reachability
{
using std_msgs::msg::Float64MultiArray;

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

/// 演示节点：在关节限位内生成连续正弦关节轨迹，供可达域节点做在线正运动学
class ArmDemoNode : public rclcpp::Node
{
public:
  ArmDemoNode()
  : Node("arm_demo_node")
  {
    publish_hz_ = declare_parameter<double>("publish_hz", 20.0);
    period_ = declare_parameter<double>("period", 8.0);
    amplitude_scale_ = declare_parameter<double>("amplitude_scale", 0.35);

    if (publish_hz_ <= 0.0 || period_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "publish_hz / period 必须为正数，节点退出");
      throw std::invalid_argument("invalid demo parameters");
    }

    arms_ = make_default_arms();
    publisher_ = create_publisher<Float64MultiArray>("/demo_joint_angles", 10);
    start_time_ = now();

    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ArmDemoNode::tick, this));

    RCLCPP_INFO(
      get_logger(), "双臂关节演示已启动：%.1f Hz，周期 %.1f s，振幅系数 %.2f",
      publish_hz_, period_, amplitude_scale_);
  }

private:
  void tick()
  {
    const double elapsed = (now() - start_time_).seconds();
    const double omega = 2.0 * kPi / period_;

    // 每个关节的频率倍率与相位，避免所有关节同相位摆动
    static constexpr std::array<double, kDof> kFrequency{{1.0, 1.5, 0.8, 1.2, 2.0, 2.5, 1.8}};
    static constexpr std::array<double, kDof> kPhase{{0.0, 0.9, 1.8, 0.4, 2.2, 1.1, 2.8}};

    Float64MultiArray msg;
    msg.data.resize(static_cast<size_t>(2 * kDof));

    for (int side = 0; side < 2; ++side) {
      const double side_phase = side == 0 ? 0.0 : 1.3;
      for (int i = 0; i < kDof; ++i) {
        const JointSpec & joint = arms_[side].joints[static_cast<size_t>(i)];
        const double mid = 0.5 * (joint.lower + joint.upper);
        const double amplitude = amplitude_scale_ * 0.5 * (joint.upper - joint.lower);
        double q = mid + amplitude * std::sin(
          omega * kFrequency[static_cast<size_t>(i)] * elapsed +
          kPhase[static_cast<size_t>(i)] + side_phase);
        q = std::max(joint.lower, std::min(joint.upper, q));
        msg.data[static_cast<size_t>(side * kDof + i)] = q;
      }
    }

    publisher_->publish(msg);
  }

  std::array<ArmModel, 2> arms_;
  double publish_hz_{20.0};
  double period_{8.0};
  double amplitude_scale_{0.35};
  rclcpp::Time start_time_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace humanoid_reachability

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_reachability::ArmDemoNode>());
  rclcpp::shutdown();
  return 0;
}
