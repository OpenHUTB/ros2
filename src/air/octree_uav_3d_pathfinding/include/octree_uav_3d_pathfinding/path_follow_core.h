#ifndef OCTREE_UAV_3D_PATHFINDING_PATH_FOLLOW_CORE_H
#define OCTREE_UAV_3D_PATHFINDING_PATH_FOLLOW_CORE_H

// =============================================================================
// 路径跟踪的状态机（纯逻辑，不依赖 ROS、不依赖 octomap）
//
// 为什么单独抽出来：状态机是这一阶段唯一容易出错的地方（前瞻切换、终点判定、
// 超时跳过），而它一旦掺进 ROS 的回调里就只能靠起整套节点来验证。
// 抽成纯函数后，`tests/test_path_follower.cpp` 可以在内存里喂任意路径与位置序列，
// 跑几百个组合，不需要 ROS 运行时、不需要模拟器。
//
// ROS 外壳在 `src/path_follower.cpp`：它只负责把消息转成这里的结构、把结果发出去。
//
// 语义约定（与设计文档 §3.3 一致）：
//   · 距当前航点 < lookahead        → 切到下一个航点（**不停稳**，这是连续飞行的关键）
//   · 最后一个航点用 finish_tolerance 判定到达，不参与 lookahead 提前切换
//   · 单点超时（per_point_timeout）→ 跳过该点；但终点超时不跳过，直接结束，
//     否则"永远到不了终点"会让状态机永远停在 Following
// =============================================================================

#include <cmath>
#include <cstddef>
#include <vector>

namespace octree_uav_3d_pathfinding {

/// 三维点。这里刻意不用 octomap::point3d 或 ROS 的 Point —— 保持核心零依赖。
struct FollowVec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

static inline FollowVec3 followVec(double x, double y, double z) {
  FollowVec3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

static inline double followDistance(const FollowVec3& a, const FollowVec3& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// 点到**线段** ab 的最短距离。
///
/// 为什么不用"到航点的距离"来算偏离：前瞻跳点会让实际轨迹切内角，
/// 而航点是稀疏的，到航点的距离会把"正常切内角"误判成大幅偏离。
/// 到线段才是对"离规划路径有多远"的正确度量。
static inline double followPointSegmentDistance(const FollowVec3& p,
                                                const FollowVec3& a,
                                                const FollowVec3& b) {
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double vz = b.z - a.z;
  const double wx = p.x - a.x;
  const double wy = p.y - a.y;
  const double wz = p.z - a.z;

  const double len2 = vx * vx + vy * vy + vz * vz;
  if (len2 <= 0.0) return followDistance(p, a);   // 退化线段（重复点）

  double t = (wx * vx + wy * vy + wz * vz) / len2;
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;

  const FollowVec3 q = followVec(a.x + t * vx, a.y + t * vy, a.z + t * vz);
  return followDistance(p, q);
}

/// 点到整条折线的最小距离。
static inline double followDistanceToPolyline(const std::vector<FollowVec3>& path,
                                              const FollowVec3& p) {
  if (path.empty()) return 0.0;
  if (path.size() == 1) return followDistance(p, path[0]);

  double best = followDistance(p, path[0]);
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double d = followPointSegmentDistance(p, path[i], path[i + 1]);
    if (d < best) best = d;
  }
  return best;
}

/// 折线总长（m）。
static inline double followPolylineLength(const std::vector<FollowVec3>& path) {
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    total += followDistance(path[i], path[i + 1]);
  }
  return total;
}

/// 跟踪参数。默认值与 `config/params.yaml` 的 `path_follower` 组一致。
struct FollowOptions {
  /// 距当前航点该距离内就切换到下一个航点（m）。
  /// **必须明显大于 finish_tolerance**，否则退化成"每点停稳"。
  double lookahead = 1.5;

  /// 判定"到达终点"的距离（m）。约 2 倍地图分辨率。
  double finish_tolerance = 0.5;

  /// 单个航点的超时（s）。超时跳过该点；终点超时则直接结束。
  double per_point_timeout = 15.0;
};

enum class FollowPhase {
  Idle,       ///< 还没开始（等合格的路径）
  Following,  ///< 正在跟踪
  Done        ///< 已到达终点（或终点超时结束）
};

enum class FollowAction {
  None,      ///< 本轮无需动作
  Send,      ///< 请把 path[index] 发给控制器
  Finished   ///< 已到达终点，停止发送
};

/// 状态机的全部可变状态。ROS 外壳持有一个实例，每帧位姿调用一次 followStep。
struct FollowState {
  FollowPhase phase = FollowPhase::Idle;
  std::size_t index = 0;              ///< 当前要追的航点下标
  std::size_t skipped = 0;            ///< 因超时被跳过的航点数
  double last_progress_s = 0.0;       ///< 上次前进的时刻（用于超时判定）
  double max_deviation = 0.0;         ///< 到折线的历史最大偏离（m），**只统计贴上路径之后**
};

/// 一次推进的结论。
struct FollowStep {
  FollowAction action = FollowAction::None;
  std::size_t index = 0;            ///< action == Send 时要发送的航点下标
  double distance_to_index = 0.0;   ///< 当前位姿到该航点的距离（m），供日志
  bool timed_out = false;           ///< 本轮是否触发了超时
};

/// 复位状态机（收到新路径时调用）。
///
/// 为什么收到新路径必须复位：`astar_planner` 每规划一次就发一条新路径（latched），
/// 旧路径可能已经因为地图更新而失效，继续追旧的下标没有意义。
static inline void followReset(FollowState& st) { st = FollowState(); }

/// 推进状态机一次。
///
/// @param st     状态（就地修改）
/// @param path   要跟踪的路径（至少两个点才有效）
/// @param pos    当前位姿
/// @param now_s  当前时刻（s，单调递增即可；用 ros::Time 的秒数）
/// @param opt    参数
static inline FollowStep followStep(FollowState& st,
                                    const std::vector<FollowVec3>& path,
                                    const FollowVec3& pos, double now_s,
                                    const FollowOptions& opt) {
  FollowStep out;

  // 路径不合格：回到 Idle 等一条新的
  if (path.size() < 2) {
    st.phase = FollowPhase::Idle;
    st.index = 0;
    return out;
  }
  if (st.phase == FollowPhase::Done) return out;

  const std::size_t last = path.size() - 1;

  // 每帧都更新最大偏离：这是"前瞻跳点切内角程度"的量化指标。
  //
  // **只在 index > 0 时统计**：index == 0 表示载具还在飞向路径起点（接近段），
  // 那一段的轨迹本来就不在路径上。若不排除，一次"从上一趟终点飞回新路径起点"的
  // 22.5 m 直线会把 max_dev 拉到 5 m 以上，指标就失去意义（2026-10-21 实测踩到）。
  const double dev = followDistanceToPolyline(path, pos);
  if (st.index > 0 && dev > st.max_deviation) st.max_deviation = dev;

  // ---- 第一次进入：从 0 号点开始 ----
  if (st.phase == FollowPhase::Idle) {
    st.phase = FollowPhase::Following;
    st.index = 0;
    st.last_progress_s = now_s;
    // 起点就在脚下时不必"飞到起点"，直接追下一个点
    if (last >= 1 && followDistance(path[0], pos) < opt.finish_tolerance) {
      st.index = 1;
    }
    out.action = FollowAction::Send;
    out.index = st.index;
    out.distance_to_index = followDistance(path[st.index], pos);
    return out;
  }

  // ---- 跟踪中 ----
  const double d = followDistance(path[st.index], pos);
  out.distance_to_index = d;

  // 超时：跳过该点。终点不跳 —— 否则永远到不了终点
  if (now_s - st.last_progress_s > opt.per_point_timeout) {
    out.timed_out = true;
    if (st.index >= last) {
      st.phase = FollowPhase::Done;
      out.action = FollowAction::Finished;
      out.index = last;
      return out;
    }
    ++st.skipped;
    ++st.index;
    st.last_progress_s = now_s;
    out.action = FollowAction::Send;
    out.index = st.index;
    out.distance_to_index = followDistance(path[st.index], pos);
    return out;
  }

  // 已经是最后一个点：只看到达容差，不做前瞻切换
  if (st.index >= last) {
    if (d < opt.finish_tolerance) {
      st.phase = FollowPhase::Done;
      out.action = FollowAction::Finished;
      out.index = last;
    }
    return out;
  }

  // 中间点：进入前瞻半径就切下一个（这是"连续飞行"的关键，不要求停稳）
  if (d < opt.lookahead) {
    ++st.index;
    st.last_progress_s = now_s;
    out.action = FollowAction::Send;
    out.index = st.index;
    out.distance_to_index = followDistance(path[st.index], pos);
  }
  return out;
}

}  // namespace octree_uav_3d_pathfinding

#endif  // OCTREE_UAV_3D_PATHFINDING_PATH_FOLLOW_CORE_H
