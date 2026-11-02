// 八叉树 A* 全局寻路 —— 基础类型
//
// 与 ROS 无关，只依赖 octomap 与 STL，便于离线单元测试。

#ifndef OCTREE_UAV_3D_PATHFINDING_ASTAR_TYPES_H
#define OCTREE_UAV_3D_PATHFINDING_ASTAR_TYPES_H

#include <cstddef>
#include <string>

#include <octomap/OcTreeKey.h>

namespace octree_uav_3d_pathfinding {

/// 八叉树叶节点的唯一标识。
///
/// 为什么不直接用 OcTreeKey：key 只在**同一深度**内唯一 —— 一个粗节点和
/// 它内部的细节点会落在不同的 key 网格上，因此 key 单独不足以标识一个节点。
/// 必须把深度一起带上。
struct NodeId {
  octomap::OcTreeKey key;
  unsigned int depth;

  NodeId() : depth(0) {}
  NodeId(const octomap::OcTreeKey& k, unsigned int d) : key(k), depth(d) {}

  bool operator==(const NodeId& o) const {
    return depth == o.depth && key == o.key;
  }
  bool operator!=(const NodeId& o) const { return !(*this == o); }
};

struct NodeIdHash {
  std::size_t operator()(const NodeId& n) const {
    // KeyHash 是 OcTreeKey 的嵌套结构体，不在 octomap 命名空间下。
    return octomap::OcTreeKey::KeyHash()(n.key) ^
           (static_cast<std::size_t>(n.depth) * 2654435761u);
  }
};

/// 一次规划的统计与诊断信息。
struct PlanStats {
  bool success = false;
  std::string reason;          ///< 失败原因；成功时为空
  std::size_t expanded = 0;    ///< A* 实际展开（出堆且非过期）的节点数
  std::size_t discovered = 0;  ///< 曾进入 open 表的节点数
  std::size_t waypoints = 0;   ///< 路径航点数
  double path_length = 0.0;    ///< 路径总长（m）
  double elapsed_ms = 0.0;     ///< 规划耗时（ms）
};

}  // namespace octree_uav_3d_pathfinding

#endif  // OCTREE_UAV_3D_PATHFINDING_ASTAR_TYPES_H
