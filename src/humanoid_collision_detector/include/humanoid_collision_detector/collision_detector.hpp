#ifndef HUMANOID_COLLISION_DETECTOR__COLLISION_DETECTOR_HPP_
#define HUMANOID_COLLISION_DETECTOR__COLLISION_DETECTOR_HPP_

#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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
  Eigen::Vector3d position;   // 世界坐标
  Eigen::Quaterniond rotation;
};

/**
 * @brief 检测结果
 */
struct DetectionResult {
  bool collision_detected;    // 障碍物是否已进入安全边界
  double min_distance;
  std::string closest_body;
  Eigen::Vector3d closest_point_on_body;
  Eigen::Vector3d obstacle_position;
  uint8_t risk_level;  // 0=safe, 1=warning, 2=danger, 3=collided
};

/**
 * @brief 碰撞体边界检测器核心类
 *
 * 当前仅支持静态简化碰撞体：碰撞体位置在注册时确定，运行期不再更新。
 */
class CollisionDetector
{
public:
  CollisionDetector();
  ~CollisionDetector();

  /**
   * @brief 手动添加一个碰撞体
   */
  void addCollisionBody(const CollisionBody & body);

  /**
   * @brief 检测障碍物与所有碰撞体的边界距离
   * @param obstacle_pos 障碍物世界坐标
   * @param obstacle_radius 障碍物近似半径
   * @param safety_boundary 安全边界阈值
   */
  DetectionResult detect(
    const Eigen::Vector3d & obstacle_pos,
    double obstacle_radius,
    double safety_boundary);

  /**
   * @brief 生成RViz可视化MarkerArray
   * @param safety_boundary 安全边界阈值
   * @param obstacle_radius 障碍物半径（用于绘制与检测尺寸一致的障碍物球）
   * @param result 当前检测结果
   */
  visualization_msgs::msg::MarkerArray generateBoundaryMarkers(
    double safety_boundary,
    double obstacle_radius,
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
   * @brief 计算点到box的最近点（世界坐标，位于box表面或其内部）
   */
  Eigen::Vector3d closestPointOnBox(
    const Eigen::Vector3d & point,
    const Eigen::Vector3d & box_center,
    const Eigen::Vector3d & box_size) const;

  std::vector<CollisionBody> collision_bodies_;
};

}  // namespace humanoid_collision_detector

#endif  // HUMANOID_COLLISION_DETECTOR__COLLISION_DETECTOR_HPP_
