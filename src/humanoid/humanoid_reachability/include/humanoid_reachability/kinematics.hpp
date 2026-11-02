#ifndef HUMANOID_REACHABILITY__KINEMATICS_HPP_
#define HUMANOID_REACHABILITY__KINEMATICS_HPP_

#include <array>
#include <random>
#include <string>

#include <Eigen/Geometry>

namespace humanoid_reachability
{

/// 单臂自由度数目（肩 3 + 肘 1 + 腕 3）
constexpr int kDof = 7;

/// 手臂运动链上的采样点数量：肩点 + 7 个关节 + 末端参考点
constexpr int kChainPoints = kDof + 2;

using JointAngles = std::array<double, kDof>;
using ChainPoints = std::array<Eigen::Vector3d, kChainPoints>;

struct JointSpec
{
  std::string name;
  Eigen::Vector3d axis;
  Eigen::Vector3d offset;  ///< 相对父关节坐标系的平移
  double lower;            ///< 下限（rad）
  double upper;            ///< 上限（rad）
};

struct ArmModel
{
  std::string name;
  int side;                ///< +1 左臂（y > 0），-1 右臂（镜像到 y < 0）
  double shoulder_y;       ///< 左肩基准位置（m）
  double shoulder_z;       ///< 肩点高度（m）
  std::array<JointSpec, kDof> joints;
  double tool_offset;      ///< 腕点到末端参考点的偏移（沿局部 x 轴，m）
};

/// 生成默认双臂模型：几何对称的简化 7-DoF 链，无 URDF 依赖
std::array<ArmModel, 2> make_default_arms();

/// 正向运动学：返回肩点、各关节原点与末端参考点的世界坐标
ChainPoints forward_kinematics(const ArmModel & arm, const JointAngles & q);

/// 末端参考点世界坐标
Eigen::Vector3d end_effector_position(const ArmModel & arm, const JointAngles & q);

/// 关节限位内均匀随机采样
JointAngles sample_joint_angles(const ArmModel & arm, std::mt19937 & rng);

/// 关节角是否在限位内
bool within_limits(const ArmModel & arm, const JointAngles & q, double tolerance = 1e-6);

}  // namespace humanoid_reachability

#endif  // HUMANOID_REACHABILITY__KINEMATICS_HPP_
