// =============================================================================
// P6 路径跟踪状态机的离线单元测试
//
// 只依赖 STL：不需要 ROS 运行时、不需要模拟器、不需要 octomap。
// 被测对象是 include/octree_uav_3d_pathfinding/path_follow_core.h 里的纯函数。
//
// 运行方式：~/uav_ws/devel/lib/octree_uav_3d_pathfinding/test_path_follower
// 退出码 0 = 全部通过
//
// 覆盖的重点（都是"停稳走"与"连续飞"之间的判断差异）：
//   1. 前瞻切换：进 lookahead 就切，不要求停稳
//   2. 终点判定：最后一个点只认 finish_tolerance，不吃 lookahead 的提前量
//   3. 超时：中间点跳过并计数；终点超时直接结束（否则永远到不了终点）
//   4. 新路径复位
//   5. 退化输入（空路径 / 单点路径）
//   6. 偏离度量是"到折线"而不是"到航点"
// =============================================================================

#include <cmath>
#include <cstdio>
#include <vector>

#include "octree_uav_3d_pathfinding/path_follow_core.h"

using octree_uav_3d_pathfinding::FollowAction;
using octree_uav_3d_pathfinding::FollowOptions;
using octree_uav_3d_pathfinding::FollowPhase;
using octree_uav_3d_pathfinding::FollowState;
using octree_uav_3d_pathfinding::FollowStep;
using octree_uav_3d_pathfinding::FollowVec3;
using octree_uav_3d_pathfinding::followDistanceToPolyline;
using octree_uav_3d_pathfinding::followPointSegmentDistance;
using octree_uav_3d_pathfinding::followPolylineLength;
using octree_uav_3d_pathfinding::followReset;
using octree_uav_3d_pathfinding::followStep;
using octree_uav_3d_pathfinding::followVec;

namespace {

int g_checks = 0;
int g_failed = 0;

void check(bool cond, const char* what) {
  ++g_checks;
  if (cond) {
    std::printf("  [ ok ] %s\n", what);
  } else {
    ++g_failed;
    std::printf("  [FAIL] %s\n", what);
  }
}

void checkNear(double got, double want, double tol, const char* what) {
  ++g_checks;
  const bool ok = std::fabs(got - want) <= tol;
  if (ok) {
    std::printf("  [ ok ] %s (got %f, want %f)\n", what, got, want);
  } else {
    ++g_failed;
    std::printf("  [FAIL] %s (got %f, want %f)\n", what, got, want);
  }
}

// 一条沿 +x 的三点路径：0 → 10 → 20
std::vector<FollowVec3> straightPath() {
  std::vector<FollowVec3> p;
  p.push_back(followVec(0, 0, 0));
  p.push_back(followVec(10, 0, 0));
  p.push_back(followVec(20, 0, 0));
  return p;
}

// ---------------------------------------------------------------------------
// 用例 1：前瞻切换 —— 进 lookahead 就切，不等停稳
// ---------------------------------------------------------------------------
void testLookaheadSwitching() {
  std::printf("\n[case 1] lookahead switching\n");

  const std::vector<FollowVec3> path = straightPath();
  const FollowOptions opt;   // lookahead 1.5 / finish 0.5 / timeout 15
  FollowState st;

  // 起点在脚下 → 直接从 0 号点跳到 1 号点，不"飞回起点"
  FollowStep s = followStep(st, path, followVec(0, 0, 0), 0.0, opt);
  check(s.action == FollowAction::Send, "first step asks to send a goal");
  check(s.index == 1, "start at path[0] advances straight to index 1");
  check(st.phase == FollowPhase::Following, "phase becomes Following");

  // 距 1 号点 8.5 m（> lookahead）→ 不动
  s = followStep(st, path, followVec(1.5, 0, 0), 1.0, opt);
  check(s.action == FollowAction::None, "far from waypoint: no action");
  check(st.index == 1, "index unchanged while farther than lookahead");

  // 距 1 号点 1.4 m（< lookahead 1.5）→ 切到 2 号点
  s = followStep(st, path, followVec(8.6, 0, 0), 2.0, opt);
  check(s.action == FollowAction::Send, "inside lookahead: send next goal");
  check(s.index == 2, "index advanced by one");
  checkNear(s.distance_to_index, 11.4, 1e-9,
            "reported distance is to the newly selected waypoint");

  // 边界：恰好等于 lookahead 时**不**切换（用 < 判定）
  FollowState st2;
  followStep(st2, path, followVec(0, 0, 0), 0.0, opt);
  s = followStep(st2, path, followVec(8.5, 0, 0), 1.0, opt);
  check(s.action == FollowAction::None,
        "distance exactly equal to lookahead does not switch");
  check(st2.index == 1, "index stays at the boundary");
}

// ---------------------------------------------------------------------------
// 用例 2：终点判定只认 finish_tolerance
// ---------------------------------------------------------------------------
void testFinishTolerance() {
  std::printf("\n[case 2] finish tolerance applies to the last waypoint only\n");

  const std::vector<FollowVec3> path = straightPath();
  FollowOptions opt;
  // 这一组刻意用旧的 0.5：它验证的是"只认容差、不吃 lookahead 提前量"这条规则，
  // 与容差取多大无关。AirSim 上用的默认值 1.0 的用例见 case 7。
  opt.finish_tolerance = 0.5;
  FollowState st;

  followStep(st, path, followVec(0, 0, 0), 0.0, opt);   // → index 1
  followStep(st, path, followVec(8.6, 0, 0), 1.0, opt); // → index 2（最后一个）
  check(st.index == 2, "reached the last waypoint index");

  // 距终点 1.0 m：小于 lookahead(1.5) 但大于 finish_tolerance(0.5) → 不算到达
  FollowStep s = followStep(st, path, followVec(19.0, 0, 0), 2.0, opt);
  check(s.action == FollowAction::None,
        "inside lookahead but outside finish_tolerance is NOT arrival");
  check(st.phase == FollowPhase::Following, "still following");

  // 距终点 0.4 m → 到达
  s = followStep(st, path, followVec(19.6, 0, 0), 3.0, opt);
  check(s.action == FollowAction::Finished, "inside finish_tolerance finishes");
  check(st.phase == FollowPhase::Done, "phase becomes Done");

  // Done 之后再喂任何位姿都不再动作
  s = followStep(st, path, followVec(0, 0, 0), 4.0, opt);
  check(s.action == FollowAction::None, "Done state never sends again");
}

// ---------------------------------------------------------------------------
// 用例 3：超时 —— 中间点跳过；终点不跳过而是结束
// ---------------------------------------------------------------------------
void testTimeouts() {
  std::printf("\n[case 3] per-waypoint timeout\n");

  const std::vector<FollowVec3> path = straightPath();
  const FollowOptions opt;   // per_point_timeout 15 s
  FollowState st;

  followStep(st, path, followVec(0, 0, 0), 0.0, opt);   // 启动 → index 1

  // 原地不动，时间越过 15 s → 跳过 1 号点
  FollowStep s = followStep(st, path, followVec(0, 0, 0), 16.0, opt);
  check(s.timed_out, "timeout is reported");
  check(s.action == FollowAction::Send, "after skipping we send the next goal");
  check(st.skipped == 1, "skipped counter incremented");
  check(!st.final_timed_out, "intermediate timeout does not set final_timed_out");
  check(st.index == 2, "index advanced past the timed-out waypoint");

  // 终点也超时 → 直接结束，**不**跳过一个不存在的点
  s = followStep(st, path, followVec(0, 0, 0), 32.0, opt);
  check(s.timed_out, "timeout on the last waypoint is reported");
  check(s.action == FollowAction::Finished, "timeout on the last waypoint finishes");
  check(st.phase == FollowPhase::Done, "phase becomes Done on final timeout");
  check(st.skipped == 1, "the final waypoint is not counted as skipped");
  check(st.final_timed_out, "final timeout is flagged separately (final_timed_out)");

  // 超时按"距上次前进"计，前进一次就会重新计时
  FollowState st2;
  followStep(st2, path, followVec(0, 0, 0), 0.0, opt);    // index 1, progress @ 0
  followStep(st2, path, followVec(8.6, 0, 0), 2.0, opt);  // 前进 → index 2, progress @ 2
  FollowStep s2 = followStep(st2, path, followVec(8.6, 0, 0), 16.0, opt);
  check(!s2.timed_out, "timer restarts after progress (14 s < 15 s)");
}

// ---------------------------------------------------------------------------
// 用例 4：新路径复位
// ---------------------------------------------------------------------------
void testResetOnNewPath() {
  std::printf("\n[case 4] reset when a new path arrives\n");

  const std::vector<FollowVec3> path = straightPath();
  const FollowOptions opt;
  FollowState st;

  followStep(st, path, followVec(0, 0, 0), 0.0, opt);
  followStep(st, path, followVec(8.6, 0, 0), 1.0, opt);
  check(st.index == 2, "precondition: midway through the old path");

  followReset(st);
  check(st.phase == FollowPhase::Idle, "reset returns to Idle");
  check(st.index == 0, "reset clears the index");
  check(st.skipped == 0, "reset clears the skipped counter");
  check(!st.final_timed_out, "reset clears final_timed_out");
  checkNear(st.max_deviation, 0.0, 1e-12, "reset clears max deviation");

  // 复位后从新路径的起点重新开始
  std::vector<FollowVec3> other;
  other.push_back(followVec(100, 0, 0));
  other.push_back(followVec(110, 0, 0));
  FollowStep s = followStep(st, other, followVec(100, 0, 0), 5.0, opt);
  check(s.action == FollowAction::Send, "restarts on the new path");
  check(st.phase == FollowPhase::Following, "phase is Following again");
}

// ---------------------------------------------------------------------------
// 用例 5：退化输入
// ---------------------------------------------------------------------------
void testDegenerateInput() {
  std::printf("\n[case 5] degenerate input\n");

  const FollowOptions opt;
  std::vector<FollowVec3> empty;
  std::vector<FollowVec3> single;
  single.push_back(followVec(0, 0, 0));

  FollowState st;
  FollowStep s = followStep(st, empty, followVec(1, 2, 3), 0.0, opt);
  check(s.action == FollowAction::None, "empty path: no action");
  check(st.phase == FollowPhase::Idle, "empty path keeps Idle");

  s = followStep(st, single, followVec(1, 2, 3), 1.0, opt);
  check(s.action == FollowAction::None, "single-point path: no action");
  check(st.phase == FollowPhase::Idle, "single-point path keeps Idle");

  // 重复点（退化线段）时距离不该产生 NaN
  const double d = followPointSegmentDistance(
      followVec(1, 1, 0), followVec(5, 5, 0), followVec(5, 5, 0));
  check(std::isfinite(d), "duplicate-point segment gives a finite distance");
  checkNear(d, std::sqrt(32.0), 1e-9, "distance to a degenerate segment is point-to-point");

  // 空折线的长度与偏离都应为 0，且不崩溃
  checkNear(followPolylineLength(empty), 0.0, 1e-12, "empty polyline has zero length");
  checkNear(followDistanceToPolyline(empty, followVec(0, 0, 0)), 0.0, 1e-12,
            "distance to an empty polyline is 0");
}

// ---------------------------------------------------------------------------
// 用例 6：偏离是"到折线"，不是"到航点"
// ---------------------------------------------------------------------------
void testMaxDeviationUsesSegments() {
  std::printf("\n[case 6] max deviation is measured to the polyline\n");

  const FollowOptions opt;

  // 直线上方 2 m 的点：到线段距离 2，而到最近航点(0,0,0)是 sqrt(29)≈5.39
  std::vector<FollowVec3> line;
  line.push_back(followVec(0, 0, 0));
  line.push_back(followVec(10, 0, 0));
  FollowState st;
  // 先站到路径起点，让状态机进入跟踪阶段（index 变成 1）——
  // 偏离只在 index > 0 时统计，index == 0 属于"接近段"
  followStep(st, line, followVec(0, 0, 0), 0.0, opt);
  check(st.index == 1, "precondition: standing on the path start advances to index 1");

  followStep(st, line, followVec(5, 2, 0), 1.0, opt);
  checkNear(st.max_deviation, 2.0, 1e-9,
            "deviation to the segment (not to the sparse waypoint)");

  // 接近段不计入：index 仍为 0 时（还在飞向路径起点），偏离不统计
  FollowState approach;
  followStep(approach, line, followVec(5, 2, 0), 0.0, opt);
  check(approach.index == 0, "precondition: still approaching the path start");
  checkNear(approach.max_deviation, 0.0, 1e-12,
            "deviation during the approach leg is not counted");

  // 折返路径（0→10→0）：中间那点仍应算 1 m
  std::vector<FollowVec3> back;
  back.push_back(followVec(0, 0, 0));
  back.push_back(followVec(10, 0, 0));
  back.push_back(followVec(0, 0, 0));
  FollowState st2;
  followStep(st2, back, followVec(0, 0, 0), 0.0, opt);      // 进入跟踪阶段
  followStep(st2, back, followVec(5, 1, 0), 1.0, opt);
  checkNear(st2.max_deviation, 1.0, 1e-9, "doubled-back path also measures 1 m");

  // 历史最大值只增不减
  followStep(st2, back, followVec(5, 3, 0), 1.0, opt);
  checkNear(st2.max_deviation, 3.0, 1e-9, "max deviation keeps the running maximum");
  followStep(st2, back, followVec(5, 0.5, 0), 2.0, opt);
  checkNear(st2.max_deviation, 3.0, 1e-9, "max deviation never decreases");

  // 折线长度
  checkNear(followPolylineLength(straightPath()), 20.0, 1e-9,
            "polyline length of the straight path is 20 m");
  checkNear(followPolylineLength(back), 20.0, 1e-9,
            "doubled-back polyline length is 20 m");
}

// ---------------------------------------------------------------------------
// 用例 7：finish_tolerance = 1.0（AirSim 上用的默认值）也必须判得到
// ---------------------------------------------------------------------------
void testFinishToleranceOneMetre() {
  std::printf("\n[case 7] arrival with finish_tolerance = 1.0 (the real default)\n");

  const std::vector<FollowVec3> path = straightPath();
  FollowOptions opt;
  opt.finish_tolerance = 1.0;
  FollowState st;

  followStep(st, path, followVec(0, 0, 0), 0.0, opt);   // → index 1
  followStep(st, path, followVec(8.6, 0, 0), 1.0, opt); // → index 2（最后一个）
  check(st.index == 2, "precondition: standing on the last waypoint");

  // 距终点 1.2 m > 1.0 → 不算到达
  FollowStep s = followStep(st, path, followVec(18.8, 0, 0), 2.0, opt);
  check(s.action == FollowAction::None,
        "1.2 m away is NOT arrival with tolerance 1.0");

  // 距终点 0.9 m < 1.0 → 到达
  s = followStep(st, path, followVec(19.1, 0, 0), 3.0, opt);
  check(s.action == FollowAction::Finished,
        "0.9 m away finishes with tolerance 1.0");
  check(!st.final_timed_out,
        "arrival within tolerance is not flagged as a timeout wrap-up");
}

// ---------------------------------------------------------------------------
// 用例 8：已进容差却恰好落在超时那一拍 —— 必须先判到达
// ---------------------------------------------------------------------------
void testArrivalOnTimeoutTick() {
  std::printf("\n[case 8] inside tolerance on the very tick of the timeout\n");

  const std::vector<FollowVec3> path = straightPath();
  const FollowOptions opt;   // finish_tolerance 1.0、per_point_timeout 15 s
  FollowState st;

  followStep(st, path, followVec(0, 0, 0), 0.0, opt);   // → index 1, progress @ 0
  followStep(st, path, followVec(8.6, 0, 0), 1.0, opt); // → index 2, progress @ 1

  // 距终点 0.5 m（在 1.0 容差内），而时间已越过 15 s 超时线：
  // 顺序若写反，这里会被记成"超时收尾、没有到达"（PR #596 评审指出的用例）
  FollowStep s = followStep(st, path, followVec(19.5, 0, 0), 17.0, opt);
  check(s.action == FollowAction::Finished, "finishes");
  check(!st.final_timed_out,
        "already inside tolerance is NOT a timeout wrap-up");
  check(!s.timed_out, "no timeout is reported for an in-tolerance arrival");
}

}  // namespace

int main() {
  std::printf("=== P6 path-follower state machine unit tests ===\n");

  testLookaheadSwitching();
  testFinishTolerance();
  testTimeouts();
  testResetOnNewPath();
  testDegenerateInput();
  testMaxDeviationUsesSegments();
  testFinishToleranceOneMetre();
  testArrivalOnTimeoutTick();

  std::printf("\n=== %d checks, %d failed ===\n", g_checks, g_failed);
  return (g_failed == 0) ? 0 : 1;
}
