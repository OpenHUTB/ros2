#include "humanoid_collision_detector/collision_detector.hpp"

#include <algorithm>
#include <limits>

namespace humanoid_collision_detector
{

CollisionDetector::CollisionDetector() = default;
CollisionDetector::~CollisionDetector() = default;

void CollisionDetector::addCollisionBody(const CollisionBody & body)
{
  CollisionBody b = body;
  // 记录建模时的本地偏移, 作为 TF 变换的基准, 避免逐帧累积漂移
  b.local_position = body.position;
  collision_bodies_.push_back(b);
}

void CollisionDetector::updateTransforms(
  const std::unordered_map<std::string, Eigen::Isometry3d> & transforms)
{
  for (auto & body : collision_bodies_) {
    auto it = transforms.find(body.name);
    if (it != transforms.end()) {
      const Eigen::Isometry3d & world_tf = it->second;
      // 始终基于 local_position 重新计算, 不可在 body.position 上累加
      body.position = world_tf.translation() + world_tf.linear() * body.local_position;
      body.rotation = Eigen::Quaterniond(world_tf.linear());
    }
  }
}

double CollisionDetector::pointToBoxDistance(
  const Eigen::Vector3d & point,
  const Eigen::Vector3d & box_center,
  const Eigen::Vector3d & box_size) const
{
  const Eigen::Vector3d half = box_size * 0.5;
  const Eigen::Vector3d diff = point - box_center;
  Eigen::Vector3d clamped;
  clamped.x() = std::clamp(diff.x(), -half.x(), half.x());
  clamped.y() = std::clamp(diff.y(), -half.y(), half.y());
  clamped.z() = std::clamp(diff.z(), -half.z(), half.z());
  return (diff - clamped).norm();
}

double CollisionDetector::pointToSphereDistance(
  const Eigen::Vector3d & point,
  const Eigen::Vector3d & sphere_center,
  double radius) const
{
  return (point - sphere_center).norm() - radius;
}

double CollisionDetector::pointToCylinderDistance(
  const Eigen::Vector3d & point,
  const Eigen::Vector3d & cyl_center,
  double radius, double height) const
{
  // 使用外接box近似：适用于大多数机器人碰撞体检测场景
  // 注意: 世界坐标系以 z 轴为高度方向, 因此圆柱高度对应 box 的 z 分量
  Eigen::Vector3d box_size(radius * 2.0, radius * 2.0, height);
  return pointToBoxDistance(point, cyl_center, box_size);
}

DetectionResult CollisionDetector::detect(
  const Eigen::Vector3d & obstacle_pos,
  double obstacle_radius,
  double safety_boundary)
{
  DetectionResult result;
  result.min_distance = std::numeric_limits<double>::max();
  result.collision_detected = false;
  result.risk_level = 0;
  result.obstacle_position = obstacle_pos;

  for (const auto & body : collision_bodies_) {
    double dist = 0.0;
    switch (body.type) {
      case CollisionType::BOX:
        dist = pointToBoxDistance(obstacle_pos, body.position, body.size) - obstacle_radius;
        break;
      case CollisionType::SPHERE:
        dist = pointToSphereDistance(obstacle_pos, body.position, body.size.x()) - obstacle_radius;
        break;
      case CollisionType::CYLINDER:
        dist = pointToCylinderDistance(obstacle_pos, body.position, body.size.x(), body.size.y()) - obstacle_radius;
        break;
    }

    if (dist < result.min_distance) {
      result.min_distance = dist;
      result.closest_body = body.name;
      result.closest_point_on_body = body.position;
    }
  }

  // 判定风险等级
  constexpr double kCollisionThreshold = 0.0;
  constexpr double kDangerThreshold = 0.15;
  constexpr double kWarningThreshold = 0.30;

  if (result.min_distance < kCollisionThreshold) {
    result.collision_detected = true;
    result.risk_level = 3;
  } else if (result.min_distance < safety_boundary) {
    result.risk_level = 2;  // danger zone
  } else if (result.min_distance < kWarningThreshold) {
    result.risk_level = 1;  // warning zone
  } else {
    result.risk_level = 0;  // safe
  }

  return result;
}

visualization_msgs::msg::MarkerArray CollisionDetector::generateBoundaryMarkers(
  double safety_boundary,
  const DetectionResult & result)
{
  visualization_msgs::msg::MarkerArray markers;

  // 颜色配置
  auto getColor = [&](double dist) -> std_msgs::msg::ColorRGBA {
    std_msgs::msg::ColorRGBA c;
    c.a = 0.6;
    if (dist < 0.0) {
      c.r = 1.0; c.g = 0.2; c.b = 0.2;
    } else if (dist < safety_boundary) {
      c.r = 1.0; c.g = 0.4; c.b = 0.2;
    } else if (dist < safety_boundary + 0.15) {
      c.r = 1.0; c.g = 0.8; c.b = 0.2;
    } else {
      c.r = 0.3; c.g = 0.8; c.b = 0.8;
    }
    return c;
  };

  // 为每个碰撞体创建一个MARKER_CUBE / MARKER_SPHERE
  int32_t id = 0;
  for (const auto & body : collision_bodies_) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = rclcpp::Time(0);
    m.ns = "collision_bodies";
    m.id = id++;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = body.position.x();
    m.pose.position.y = body.position.y();
    m.pose.position.z = body.position.z();

    Eigen::Quaterniond q = body.rotation;
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();

    // 给每个body一个基础颜色（closest body高亮）
    bool is_closest = (body.name == result.closest_body);
    m.color = is_closest ? getColor(result.min_distance) : getColor(1.0);

    if (body.type == CollisionType::BOX) {
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.scale.x = body.size.x();
      m.scale.y = body.size.y();
      m.scale.z = body.size.z();
    } else if (body.type == CollisionType::SPHERE) {
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.scale.x = m.scale.y = m.scale.z = body.size.x() * 2.0;
    } else if (body.type == CollisionType::CYLINDER) {
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.scale.x = m.scale.z = body.size.x() * 2.0;
      m.scale.y = body.size.y();
    }

    markers.markers.push_back(m);

    // 边界线框 (LINE_LIST)
    if (body.type == CollisionType::BOX) {
      visualization_msgs::msg::Marker edges;
      edges.header = m.header;
      edges.ns = "collision_boundaries";
      edges.id = id++;
      edges.type = visualization_msgs::msg::Marker::LINE_LIST;
      edges.action = visualization_msgs::msg::Marker::ADD;
      edges.pose = m.pose;
      edges.scale.x = 0.005;
      edges.color = m.color;
      edges.color.a = 0.9;

      const double hx = body.size.x() / 2.0;
      const double hy = body.size.y() / 2.0;
      const double hz = body.size.z() / 2.0;
      // 8个顶点
      std::vector<Eigen::Vector3d> corners = {
        { -hx, -hy, -hz }, {  hx, -hy, -hz },
        {  hx,  hy, -hz }, { -hx,  hy, -hz },
        { -hx, -hy,  hz }, {  hx, -hy,  hz },
        {  hx,  hy,  hz }, { -hx,  hy,  hz },
      };
      // 12条边
      std::vector<std::pair<int, int>> edge_idx = {
        {0,1},{1,2},{2,3},{3,0},
        {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7}
      };
      for (const auto & [a, b] : edge_idx) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = corners[a].x(); p1.y = corners[a].y(); p1.z = corners[a].z();
        p2.x = corners[b].x(); p2.y = corners[b].y(); p2.z = corners[b].z();
        edges.points.push_back(p1);
        edges.points.push_back(p2);
      }
      markers.markers.push_back(edges);
    }
  }

  // 障碍物Marker
  visualization_msgs::msg::Marker obs;
  obs.header.frame_id = "world";
  obs.header.stamp = rclcpp::Time(0);
  obs.ns = "obstacle";
  obs.id = 9999;
  obs.type = visualization_msgs::msg::Marker::SPHERE;
  obs.action = visualization_msgs::msg::Marker::ADD;
  obs.pose.position.x = result.obstacle_position.x();
  obs.pose.position.y = result.obstacle_position.y();
  obs.pose.position.z = result.obstacle_position.z();
  obs.scale.x = obs.scale.y = obs.scale.z = 0.4;
  obs.color.r = 1.0;
  obs.color.g = 0.4;
  obs.color.b = 0.4;
  obs.color.a = 0.9;
  markers.markers.push_back(obs);

  // 距离指示线
  visualization_msgs::msg::Marker dist_line;
  dist_line.header.frame_id = "world";
  dist_line.header.stamp = rclcpp::Time(0);
  dist_line.ns = "distance_indicator";
  dist_line.id = 9998;
  dist_line.type = visualization_msgs::msg::Marker::LINE_LIST;
  dist_line.action = visualization_msgs::msg::Marker::ADD;
  dist_line.scale.x = 0.008;
  dist_line.color = getColor(result.min_distance);
  dist_line.color.a = 1.0;

  geometry_msgs::msg::Point p_obs, p_body;
  p_obs.x = result.obstacle_position.x();
  p_obs.y = result.obstacle_position.y();
  p_obs.z = result.obstacle_position.z();
  p_body.x = result.closest_point_on_body.x();
  p_body.y = result.closest_point_on_body.y();
  p_body.z = result.closest_point_on_body.z();
  dist_line.points.push_back(p_obs);
  dist_line.points.push_back(p_body);
  markers.markers.push_back(dist_line);

  // 文字标注
  visualization_msgs::msg::Marker text;
  text.header.frame_id = "world";
  text.header.stamp = rclcpp::Time(0);
  text.ns = "distance_text";
  text.id = 9997;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.pose.position = p_body;
  text.pose.position.z += 0.1;
  text.scale.z = 0.08;
  text.color = getColor(result.min_distance);
  text.text = result.closest_body + ": " + std::to_string(result.min_distance).substr(0, 5) + " m";
  markers.markers.push_back(text);

  return markers;
}

}  // namespace humanoid_collision_detector
