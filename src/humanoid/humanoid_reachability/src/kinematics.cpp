#include "humanoid_reachability/kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace humanoid_reachability
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double deg2rad(double deg)
{
  return deg * kPi / 180.0;
}

JointSpec make_joint(
  const std::string & name, const Eigen::Vector3d & axis,
  const Eigen::Vector3d & offset, double lower_deg, double upper_deg)
{
  JointSpec joint;
  joint.name = name;
  joint.axis = axis.normalized();
  joint.offset = offset;
  joint.lower = deg2rad(lower_deg);
  joint.upper = deg2rad(upper_deg);
  return joint;
}
}  // namespace

std::array<ArmModel, 2> make_default_arms()
{
  // 局部坐标系：x 前、y 左、z 上；上臂 0.30 m、前臂 0.28 m、末端参考点距腕 0.10 m
  std::array<JointSpec, kDof> joints{{
    make_joint("shoulder_pitch", Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 0), -120.0, 120.0),
    make_joint("shoulder_roll", Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 0), -20.0, 170.0),
    make_joint("shoulder_yaw", Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0), -85.0, 85.0),
    make_joint("elbow_flex", Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0.30, 0, 0), 0.0, 145.0),
    make_joint("wrist_roll", Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0.28, 0, 0), -90.0, 90.0),
    make_joint("wrist_pitch", Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 0), -70.0, 70.0),
    make_joint("wrist_yaw", Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0), -45.0, 45.0),
  }};

  ArmModel left;
  left.name = "left_arm";
  left.side = 1;
  left.shoulder_y = 0.20;
  left.shoulder_z = 1.42;
  left.joints = joints;
  left.tool_offset = 0.10;

  // 右臂与左臂几何镜像：采样后统一把 y 取反
  ArmModel right = left;
  right.name = "right_arm";
  right.side = -1;

  return {left, right};
}

ChainPoints forward_kinematics(const ArmModel & arm, const JointAngles & q)
{
  ChainPoints points;
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translate(Eigen::Vector3d(0.0, arm.shoulder_y, arm.shoulder_z));
  points[0] = transform.translation();

  for (int i = 0; i < kDof; ++i) {
    transform.translate(arm.joints[i].offset);
    transform.rotate(Eigen::AngleAxisd(q[i], arm.joints[i].axis));
    points[i + 1] = transform.translation();
  }

  Eigen::Vector3d tool = transform * Eigen::Vector3d(arm.tool_offset, 0.0, 0.0);
  points[kDof + 1] = tool;

  if (arm.side < 0) {
    for (auto & point : points) {
      point.y() = -point.y();
    }
  }
  return points;
}

Eigen::Vector3d end_effector_position(const ArmModel & arm, const JointAngles & q)
{
  return forward_kinematics(arm, q).back();
}

JointAngles sample_joint_angles(const ArmModel & arm, std::mt19937 & rng)
{
  JointAngles q{};
  for (int i = 0; i < kDof; ++i) {
    std::uniform_real_distribution<double> dist(arm.joints[i].lower, arm.joints[i].upper);
    q[i] = dist(rng);
  }
  return q;
}

bool within_limits(const ArmModel & arm, const JointAngles & q, double tolerance)
{
  for (int i = 0; i < kDof; ++i) {
    if (q[i] < arm.joints[i].lower - tolerance || q[i] > arm.joints[i].upper + tolerance) {
      return false;
    }
  }
  return true;
}

}  // namespace humanoid_reachability
