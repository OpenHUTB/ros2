#ifndef HUMANOID_COLLISION_DETECTOR__COLLISION_DETECTOR_HPP_
#define HUMANOID_COLLISION_DETECTOR__COLLISION_DETECTOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include <Eigen/Dense>

namespace humanoid_collision_detector
{

/**
 * @brief 碰撞体基元类型
 */
enum class CollisionType {
  BOX,
  SPHERE,
  CYLINDER
};

/**
 * @brief 单个碰撞体描述
 */
struct CollisionBody {
  std::string name;           // 连杆名称
  CollisionType type;
  Eigen::Vector3d size;       // 尺寸参数 (box: xyz, sphere: x=radius, cylinder: x=radius, y=height)
  Eigen::Vector3d position;   // 当前世界坐标
  Eigen::Quaterniond rotation;
  Eigen::Vector3d local_position;  // 建模时的本地偏移 (相对 link 原点), 用于 TF 变换
};

/**
 * @brief 检测结果
 */
struct DetectionResult {
  bool collision_detected;
  double min_distance;
  std::string closest_body;
  Eigen::Vector3d closest_point_on_body;
  Eigen::Vector3d obstacle_position;
  uint8_t risk_level;  // 0=safe, 1=warning, 2=danger, 3=collided
};

/**
 * @brief 碰撞体边界检测器核心类
 */
class CollisionDetector
{
public:
  CollisionDetector();
  ~CollisionDetector();

  /**
   * @brief 手动添加一个碰撞体（节点层负责从URDF解析然后调用这里）
   */
  void addCollisionBody(const CollisionBody & body);

  /**
   * @brief 更新所有碰撞体的世界位置（根据TF）
   * @param transforms TF变换表 (link_name -> world transform)
   */
  void updateTransforms(const std::unordered_map<std::string, Eigen::Isometry3d> & transforms);

  /**
   * @brief 检测障碍物与所有碰撞体的边界距离
   * @param obstacle_pos 障碍物世界坐标
   * @param obstacle_radius 障碍物近似半径
   * @param safety_boundary 安全边界膨胀系数
   */
  DetectionResult detect(
    const Eigen::Vector3d & obstacle_pos,
    double obstacle_radius,
    double safety_boundary);

  /**
   * @brief 生成RViz可视化MarkerArray
   */
  visualization_msgs::msg::MarkerArray generateBoundaryMarkers(
    double safety_boundary,
    const DetectionResult & result);

  /**
   * @brief 获取已注册的碰撞体列表
   */
  const std::vector<CollisionBody> & getCollisionBodies() const { return collision_bodies_; }

  /**
   * @brief 清除所有碰撞体
   */
  void clear() { collision_bodies_.clear(); }

private:
  /**
   * @brief 计算点到box的最近距离
   */
  double pointToBoxDistance(
    const Eigen::Vector3d & point,
    const Eigen::Vector3d & box_center,
    const Eigen::Vector3d & box_size) const;

  /**
   * @brief 计算点到球体的最近距离
   */
  double pointToSphereDistance(
    const Eigen::Vector3d & point,
    const Eigen::Vector3d & sphere_center,
    double radius) const;

  /**
   * @brief 计算点到圆柱体的最近距离（外接盒近似）
   */
  double pointToCylinderDistance(
    const Eigen::Vector3d & point,
    const Eigen::Vector3d & cyl_center,
    double radius, double height) const;

  std::vector<CollisionBody> collision_bodies_;
};

}  // namespace humanoid_collision_detector

#endif  // HUMANOID_COLLISION_DETECTOR__COLLISION_DETECTOR_HPP_
