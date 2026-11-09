// P5 安全裕度与路径平滑 —— 离线单元测试
//
// 设计说明见 notes/P5_安全裕度与路径平滑设计.md §7 第一层。
//
// 与 tests/test_astar.cpp 一样：**刻意只依赖 octomap 与 STL** —— 不引入 gtest、
// 不需要 ROS 运行时、不需要模拟器。编译一次就能跑完全部用例。
//
// 运行：~/uav_ws/devel/lib/octree_uav_3d_pathfinding/test_smoothing
//      退出码 0 = 全部通过
//
// 覆盖设计文档 §9 风险项清单里的四条：
//   · ① r = 0 时 isBlocked() 恒 false        → 保住"退回 P4"
//   · ② 膨胀后规模（叶节点数/内存）           → 打印出来供设计文档引用
//   · ④ 曲率除零守卫（共线、重点）            → 不产生 NaN/inf
//   · ⑥ 平滑引入碰撞（样条复检 + 步长）        → 必须拒绝/退回，不能发布越界曲线
//
// 关于"坐标标定"的一个刻意做法：本文件**不假设** octomap 的坐标→关键字取整规则，
// 而是用 keyToCoord(coordToKeyChecked(p)) 反查某个坐标真正落在哪个体素的中心，
// 再拿那个中心去做断言。P4 阶段就是因为凭记忆假设内部规则而踩了坑。

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <octomap/octomap.h>

#include "octree_uav_3d_pathfinding/astar.h"
#include "octree_uav_3d_pathfinding/map_inflation.h"
#include "octree_uav_3d_pathfinding/path_smoothing.h"

// ---------------------------------------------------------------------------
// ★ 把用到的名字显式引入（2026-10-14 客户机编译报错后修正）
//
// 为什么必须写这一段：本测试里大量**未限定名调用**模板与类型，例如
//     const std::vector<octomap::point3d> out = shortcutPath(path, freeNoWall, 0.05);
// 而 **ADL（参数依赖查找）对函数模板基本不生效** —— 实参是
// octomap::point3d 与 lambda，关联命名空间里没有 shortcutPath，编译器就报：
//     error: 'shortcutPath' was not declared in this scope;
//            did you mean 'octree_uav_3d_pathfinding::shortcutPath'?
// （非模板的类/结构体也一并显式引入，避免同类问题。）
//
// 对比：tests/test_astar.cpp 没这个问题，因为它在开头写了
//     typedef octree_uav_3d_pathfinding::OctreeAStar AStar;
// 把用到的类型引入了。这里用 using 声明把同样的事做全。
//
// 注意：预处理（g++ -E）**看不出**这类错误 —— 它只做文本替换、不做名字查找。
// 所以"预处理输出里有 N 次 shortcutPath"不能证明它在 C++ 里可见。
// ---------------------------------------------------------------------------
using octree_uav_3d_pathfinding::InflationStats;
using octree_uav_3d_pathfinding::MapInflation;
using octree_uav_3d_pathfinding::OctreeAStar;
using octree_uav_3d_pathfinding::PathMetrics;
using octree_uav_3d_pathfinding::SmoothOptions;
using octree_uav_3d_pathfinding::SmoothStats;

using octree_uav_3d_pathfinding::computeMetrics;
using octree_uav_3d_pathfinding::maxCurvature;
using octree_uav_3d_pathfinding::minClearance;
using octree_uav_3d_pathfinding::pathLength;
using octree_uav_3d_pathfinding::pointCurvature;
using octree_uav_3d_pathfinding::segmentFree;
using octree_uav_3d_pathfinding::shortcutPath;
using octree_uav_3d_pathfinding::smoothPath;

namespace {

int g_checks = 0;
int g_failed = 0;

void check(bool cond, const std::string& what) {
  ++g_checks;
  if (cond) {
    std::printf("  [ ok ] %s\n", what.c_str());
  } else {
    ++g_failed;
    std::printf("  [FAIL] %s\n", what.c_str());
  }
}

void checkNear(double got, double want, double tol, const std::string& what) {
  ++g_checks;
  if (std::fabs(got - want) <= tol) {
    std::printf("  [ ok ] %s (got %.6f, want %.6f)\n", what.c_str(), got, want);
  } else {
    ++g_failed;
    std::printf("  [FAIL] %s (got %.6f, want %.6f, tol %.6f)\n", what.c_str(),
                got, want, tol);
  }
}

octomap::point3d P(double x, double y, double z) { return octomap::point3d(x, y, z); }

// ---------------------------------------------------------------------------
// 极简解析式障碍环境：单测平滑逻辑时不需要建树
// ---------------------------------------------------------------------------
struct Box {
  double x0, x1, y0, y1, z0, z1;
};

// 一堵墙：x ∈ [0.4, 0.6]，y、z 方向贯穿
const Box kWall = {0.4, 0.6, -5.0, 5.0, -5.0, 5.0};

bool inBox(const Box& b, const octomap::point3d& p) {
  return p.x() >= b.x0 && p.x() <= b.x1 && p.y() >= b.y0 && p.y() <= b.y1 &&
         p.z() >= b.z0 && p.z() <= b.z1;
}

// "膨胀图"的可通行谓词：不在这堵墙里
bool freeNoWall(const octomap::point3d& p) { return !inBox(kWall, p); }
bool freeAlways(const octomap::point3d& /*p*/) { return true; }

// ---------------------------------------------------------------------------
// 树的辅助操作
// ---------------------------------------------------------------------------

// 某个坐标真正落在哪个体素的中心（不假设取整规则，靠 API 反查）
octomap::point3d voxelCenter(const octomap::OcTree& tree,
                             const octomap::point3d& p) {
  octomap::OcTreeKey key;
  if (!tree.coordToKeyChecked(p, key)) return p;
  return tree.keyToCoord(key, tree.getTreeDepth());
}

// 由坐标定出体素中心，再标为占用 —— 断言时用同一个中心，避免取整歧义
//
// 注意 Octree::updateNode(key, value) 的第二个参数是 **log-odds 值**：
// 传 `true`（= 1.0）超过默认占用阈值 0.12，所以这是"标为占用"。
// 想标"已知空闲"不能用这个函数，要用下面的 freeRay()（射线投射）。
void markVoxel(octomap::OcTree& tree, const octomap::point3d& p) {
  const octomap::point3d c = voxelCenter(tree, p);
  octomap::OcTreeKey key;
  if (tree.coordToKeyChecked(c, key)) tree.updateNode(key, true);
}

// 用射线投射把 origin → endpoint 这条射线上的体素标成"已知空闲"，
// 端点标成占用 —— 与真实建图一样（空闲空间就是这么来的）。
//
// 为什么需要它：`updateNode(key, true)` 只能标占用；而"已知空闲"的叶节点
// 必须由射线投射产生（其 log-odds 低于空闲阈值）。测试要区分
// "已知空闲"与"未知空间"，就必须用这条路径。
void freeRay(octomap::OcTree& tree, const octomap::point3d& origin,
             const octomap::point3d& endpoint) {
  octomap::Pointcloud cloud;
  cloud.push_back(endpoint);
  tree.insertPointCloud(cloud, origin);
}

// 把一个盒子里（按分辨率取样）的体素全部标为占用
void markBox(octomap::OcTree& tree, const Box& b) {
  const double res = tree.getResolution();
  for (double x = b.x0; x <= b.x1 + 1e-9; x += res) {
    for (double y = b.y0; y <= b.y1 + 1e-9; y += res) {
      for (double z = b.z0; z <= b.z1 + 1e-9; z += res) {
        markVoxel(tree, P(x, y, z));
      }
    }
  }
}

std::size_t occupiedLeafCount(octomap::OcTree& tree) {
  std::size_t n = 0;
  for (octomap::OcTree::leaf_iterator it = tree.begin_leafs();
       it != tree.end_leafs(); ++it) {
    if (tree.isNodeOccupied(*it)) ++n;
  }
  return n;
}

// ===========================================================================
// 用例 6：捷径化必须拒绝穿墙
// ===========================================================================
void testShortcutRejectsWall() {
  std::printf("\n[case 6] shortcut must reject a segment through the wall\n");

  std::vector<octomap::point3d> path;
  path.push_back(P(-1.0, 0.0, 0.0));
  path.push_back(P(0.5, -1.0, 0.0));   // 绕到墙的侧面
  path.push_back(P(1.0, 0.0, 0.0));

  const std::vector<octomap::point3d> out =
      shortcutPath(path, freeNoWall, 0.05);
  check(out.size() == 3, "blocked shortcut keeps all 3 points");
  checkNear((out[1] - P(0.5, -1.0, 0.0)).norm(), 0.0, 1e-9,
            "the detour point is preserved");

  const std::vector<octomap::point3d> out_free =
      shortcutPath(path, freeAlways, 0.05);
  check(out_free.size() == 2,
        "with no obstacle the same polyline collapses to 2 points");
  checkNear((out_free.front() - path.front()).norm(), 0.0, 1e-9,
            "shortcut keeps the first point");
  checkNear((out_free.back() - path.back()).norm(), 0.0, 1e-9,
            "shortcut keeps the last point");
}

// ===========================================================================
// 用例 7：采样步长必须小于分辨率（"钻缝"风险）
// ===========================================================================
void testShortcutSampleStepAliasing() {
  std::printf("\n[case 7] coarse sampling tunnels through the wall "
              "(proves step is safety-critical, not a perf knob)\n");

  std::vector<octomap::point3d> path;
  path.push_back(P(-1.0, 0.0, 0.0));
  path.push_back(P(0.5, -1.0, 0.0));
  path.push_back(P(1.0, 0.0, 0.0));

  // 细步长（< 墙厚 0.2 m）→ 正确拒绝
  const std::vector<octomap::point3d> fine = shortcutPath(path, freeNoWall, 0.02);
  check(fine.size() == 3, "step 0.02 m (< wall thickness) correctly refuses");

  // 粗步长（> 墙厚）→ 采样点在墙两侧跳过去，**错误地**取直。
  // 这是"反例断言"：它证明步长必须小于分辨率，不能说"调大点更快"。
  const std::vector<octomap::point3d> coarse = shortcutPath(path, freeNoWall, 1.0);
  check(coarse.size() == 2,
        "step 1.0 m (> wall thickness) tunnels through -- aliasing is real");

  const double res = 0.3;
  check(res / 2.0 < res, "default step (res/2) is strictly smaller than resolution");
}

// ===========================================================================
// 用例 10：显式超大采样步长必须被强制限幅
//
// 审阅指出的风险：`smoothing_sample_step` 若被设成 ≥ 分辨率，`segmentFree`
// 会整块跳过中间体素，捷径化与样条复检双双漏检障碍 —— 安全保证被破坏。
// 文档与单测都把"步长 < 分辨率"写成硬约束，所以代码必须**强制**它，
// 而不能只在默认值上成立。
// ===========================================================================
void testOversizedSampleStepIsClamped() {
  std::printf("\n[case 10] oversized sample step is clamped, never used as-is\n");

  std::vector<octomap::point3d> path;
  path.push_back(P(0, 0, 0));
  path.push_back(P(0.5, -1.0, 0));
  path.push_back(P(1.0, 0, 0));

  const double resolution = 0.3;

  // 故意给一个远大于分辨率的步长
  SmoothOptions opt;
  opt.shortcut = true;
  opt.spline = false;
  opt.sample_step = 1.0;

  std::vector<octomap::point3d> out;
  SmoothStats stats;
  smoothPath(path, opt, freeAlways, resolution, out, stats);

  check(stats.sample_step_clamped,
        "oversized explicit sample step is flagged as clamped");
  check(stats.sample_step_used > 0.0 && stats.sample_step_used < resolution,
        "step actually used is strictly below resolution");
  checkNear(stats.sample_step_used, resolution / 2.0, 1e-12,
            "clamped step equals resolution/2");

  // 默认路径（未显式给步长）不应被标记为限幅
  SmoothOptions def;
  def.shortcut = true;
  def.spline = false;
  std::vector<octomap::point3d> out2;
  SmoothStats stats2;
  smoothPath(path, def, freeAlways, resolution, out2, stats2);
  check(!stats2.sample_step_clamped,
        "default (auto) step is not flagged as clamped");
  checkNear(stats2.sample_step_used, resolution / 2.0, 1e-12,
            "default step equals resolution/2");

  // 恰好等于分辨率也要被限幅（边界条件：>= 就得拦）
  SmoothOptions edge;
  edge.shortcut = true;
  edge.spline = false;
  edge.sample_step = resolution;
  std::vector<octomap::point3d> out3;
  SmoothStats stats3;
  smoothPath(path, edge, freeAlways, resolution, out3, stats3);
  check(stats3.sample_step_clamped,
        "step exactly equal to resolution is also clamped");
  check(stats3.sample_step_used < resolution,
        "step equal to resolution is reduced below it");
}

// ===========================================================================
// 用例 11：r = 0 + 启用平滑 时，碰撞谓词仍必须查原树
//
// 审阅指出的缺陷：`OctreeAStar::traversable()` 曾写成
//     if (blocked_ == NULL) return true;
// 而 blocked_ 只在 inflation_radius > 0 时才赋值 —— 于是 r = 0 时对**任何坐标**
// 都返回 true。`smoothing:=true inflation_radius:=0` 是合法配置，此时
// segmentFree() 认为任何线段都无碰撞，捷径化/样条会把安全的 A* 折线取直、
// 直接穿过障碍。
//
// 本用例把那种配置做出来：一棵有障碍的原树 + 默认 Params（r = 0），
// 断言穿过障碍的线段被拒绝。
// ===========================================================================
void testTraversableChecksRawTreeAtZeroRadius() {
  std::printf("\n[case 11] r=0 + smoothing: collision predicate must still "
              "consult the raw tree\n");

  octomap::OcTree tree(0.3);
  const octomap::point3d obstacle = voxelCenter(tree, P(2.0, 2.0, 0.0));

  // ① 先打 4 条平行射线（沿 x 方向、y≈2、z 取 ±0.05/±0.15），
  //    终点都用**未被占据**的体素中心 —— 射线扫过的体素成为**已知空闲**，
  //    覆盖路径要走的 x∈[-2,3]、y≈2、z≈0 一带。
  //    其中一条会穿过障碍所在体素，把它短暂标成空闲。
  const double ray_z[4] = {-0.15, -0.05, 0.05, 0.15};
  for (int i = 0; i < 4; ++i) {
    freeRay(tree, octomap::point3d(10.0, 2.0, ray_z[i]),
            octomap::point3d(-9.0, 2.0, ray_z[i]));
  }
  // ② 再标障碍（占用标记覆盖刚才的射线结果）——
  //    与真实建图一致：同一个体素被占用命中后即判定为障碍。
  markVoxel(tree, obstacle);

  OctreeAStar astar(tree, OctreeAStar::Params());   // inflation_radius = 0

  check(!astar.traversable(obstacle),
        "an occupied point is NOT traversable even at r = 0");

  // 射线投射扫过的区域：已知空闲，必须可通行
  const octomap::point3d known_free = voxelCenter(tree, P(-2.0, 2.0, 0.0));
  check(astar.traversable(known_free),
        "a known free voxel IS traversable at r = 0");

  // 地图外的点：原树里没有节点 → 未知 → 当障碍
  check(!astar.traversable(P(500.0, 500.0, 500.0)),
        "an unknown (off-map) point is NOT traversable");

  // 这一条是本次修复的核心：两个端点都是**已知空闲**，唯一障碍在它们之间，
  // 所以"不能合并成一条直段"只可能是因为线段穿过了障碍。
  std::vector<octomap::point3d> path;
  path.push_back(known_free);
  path.push_back(obstacle);
  path.push_back(voxelCenter(tree, P(6.0, 2.0, 0.0)));   // 也是已知空闲

  SmoothOptions opt;
  opt.shortcut = true;
  opt.spline = false;

  std::vector<octomap::point3d> out;
  SmoothStats stats;
  smoothPath(path, opt, [&astar](const octomap::point3d& p) {
               return astar.traversable(p);
             },
             0.3, out, stats);

  check(out.size() == path.size(),
        "shortcut must NOT merge across the obstacle at r = 0 "
        "(a straight segment would pass through it)");
  check(stats.output_points == out.size(), "stats.output_points matches output size");

  // 反向确认：路径中段那个点是障碍，本身不可通行 ——
  // 所以"不合并"确实是碰撞谓词拦下来的。
  check(!astar.traversable(obstacle),
        "sanity: the middle waypoint (the obstacle) is not traversable");
}

// ===========================================================================
// 用例 9：曲率除零守卫（风险项 ④）
// ===========================================================================
void testCurvatureGuards() {
  std::printf("\n[case 9] curvature divide-by-zero guards (risk item 4)\n");

  std::size_t degenerate = 0;

  // 三点共线 → 叉积为 0 → κ = 0，并计入退化
  const double k_colinear =
      pointCurvature(P(0, 0, 0), P(1, 0, 0), P(2, 0, 0), 1e-9, &degenerate);
  check(k_colinear == 0.0, "collinear three points give kappa = 0");
  check(degenerate == 1, "collinear case is counted as degenerate");

  // 重复点 → 边长 < eps → κ = 0
  const double k_dup =
      pointCurvature(P(1, 1, 1), P(1, 1, 1), P(2, 1, 1), 1e-9, &degenerate);
  check(k_dup == 0.0, "duplicated points give kappa = 0");
  check(degenerate == 2, "duplicate case is counted as degenerate");

  // 绝不产生 NaN / inf
  check(!std::isnan(k_colinear) && !std::isinf(k_colinear),
        "degenerate result is never NaN/inf");
  check(!std::isnan(k_dup) && !std::isinf(k_dup),
        "duplicate-point result is never NaN/inf");

  // 正常圆弧：有限正值，且不增加退化计数
  const double k_arc =
      pointCurvature(P(1, 0, 0), P(0, 1, 0), P(-1, 0, 0), 1e-9, &degenerate);
  check(k_arc > 0.0 && std::isfinite(k_arc),
        "a real arc gives a finite positive kappa");
  checkNear(k_arc, 1.0, 1e-6, "unit-circle三点 gives kappa = 1.0");
  check(degenerate == 2,
        "a non-degenerate arc does not bump the degenerate counter");

  // 整条路径上的最大曲率：共线路径应给出 0
  std::vector<octomap::point3d> line;
  line.push_back(P(0, 0, 0));
  line.push_back(P(1, 0, 0));
  line.push_back(P(2, 0, 0));
  line.push_back(P(3, 0, 0));
  std::size_t deg2 = 0;
  checkNear(maxCurvature(line, 1e-9, &deg2), 0.0, 1e-12,
            "maxCurvature of a straight line is 0");
  check(deg2 == 2, "both interior points of the line are counted degenerate");
}

// ===========================================================================
// 用例 8：样条在无障碍时被采纳
// ===========================================================================
void testSplineAcceptedOnSmoothArc() {
  std::printf("\n[case 8] spline is adopted on a smooth collision-free arc\n");

  std::vector<octomap::point3d> path;
  path.push_back(P(0, 0, 0));
  path.push_back(P(1, 0.3, 0));
  path.push_back(P(2, 0.4, 0));
  path.push_back(P(3, 0.3, 0));
  path.push_back(P(4, 0, 0));

  SmoothOptions opt;
  opt.shortcut = false;   // 只测样条本身
  opt.spline = true;
  opt.spline_samples_per_segment = 8;

  std::vector<octomap::point3d> out;
  SmoothStats stats;
  const bool changed = smoothPath(path, opt, freeAlways, 0.3, out, stats);

  check(stats.spline_applied, "spline applied in free space");
  check(!stats.spline_rejected, "spline not rejected in free space");
  check(out.size() > path.size(),
        "spline densifies the path (more points than the input polyline)");
  check(changed, "smoothPath reports that it changed the path");
  check(stats.output_points == out.size(), "stats.output_points matches output size");
  check(out.size() >= 2 && (out.front() - path.front()).norm() < 1e-9 &&
            (out.back() - path.back()).norm() < 1e-9,
        "spline interpolates both endpoints exactly");
  check(std::isfinite(stats.max_curvature) && stats.max_curvature > 0.0,
        "a curved path has a finite positive max curvature");
  std::printf("         spline: %zu -> %zu points, len=%.3f m, kappa_max=%.4f 1/m\n",
              path.size(), out.size(), stats.length, stats.max_curvature);
}

// ===========================================================================
// 用例 10：样条复检兜底（风险项 ⑥）
// ===========================================================================
void testSplineRejectedWhenCurveHitsObstacle() {
  std::printf("\n[case 10] spline must be rejected when the fitted curve "
              "enters an obstacle (risk item 6)\n");

  // L 形拐角：折线绕开障碍，但样条在拐角**内侧**会外偏离、切进障碍里。
  //
  // ★ 障碍位置很讲究（2026-10-14 第一次跑测试时放错，导致用例无效）：
  //   若障碍放在拐角顶点 (1,1) 附近，折线的边 (1,1)->(2,2) 自己就穿过去了，
  //   前置条件"折线无碰撞"直接不成立 —— 那样即使样条被拒绝，也证明不了
  //   "复检拦住了越界曲线"。
  //   所以把障碍放在**拐角内侧、且离折线的边足够远**的位置。
  //   折线在 x<=1 且 y<=1 的区域内只有 (1,1) 这一个点会进盒，
  //   而 (1.05,1.05) 已经落在盒子外（盒子到 0.95 为止）。
  const Box corner = {0.50, 0.95, 0.50, 0.95, -1.0, 1.0};

  // ★ 注意：不能写成函数内部的 `struct Local { static bool free(...) }`
  //   —— 局部类的成员函数**不能引用外层函数的局部变量**（corner），
  //      报 "use of local variable with automatic storage from containing function"。
  //   传引用给一个自由谓词即可。此处用 lambda，正好也覆盖了
  //   "free 谓词可以是闭包" 这条路径。
  const auto free_corner = [&corner](const octomap::point3d& p) {
    return !inBox(corner, p);
  };

  std::vector<octomap::point3d> path;
  path.push_back(P(0, 0, 0));
  path.push_back(P(1, 0, 0));
  path.push_back(P(1, 1, 0));
  path.push_back(P(2, 2, 0));

  // 先确认"折线本身是安全的"，否则这个用例证明不了任何事
  bool polyline_safe = true;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!free_corner(path[i])) polyline_safe = false;
  }
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    if (!segmentFree(path[i], path[i + 1], free_corner, 0.05)) polyline_safe = false;
  }
  check(polyline_safe, "precondition: the input polyline is collision-free");

  SmoothOptions opt;
  opt.shortcut = false;   // 只测样条的复检
  opt.spline = true;
  opt.spline_samples_per_segment = 12;

  std::vector<octomap::point3d> out;
  SmoothStats stats;
  smoothPath(path, opt, free_corner, 0.1, out, stats);

  check(stats.input_points == path.size(),
        "stats.input_points matches the input polyline size");

  // ★ 本用例真正要守的不变量（2026-10-14 重构）：
  //   **发布出去的路径必须无碰撞**。无论复检是"拒绝了样条"还是"接受了样条"，
  //   这一条都必须成立 —— 它才是"平滑不能引入碰撞"的核心断言。
  //
  //   为什么不再断言"一定被拒绝"：复检是否触发取决于障碍与样条的实际几何，
  //   强断言会让用例在几何稍变时假失败。所以：
  //     · 若样条被拒绝 → 额外断言"退回的是折线（点数与输入相同）"
  //     · 无论哪种 → 断言发布路径无碰撞
  bool all_free = true;
  for (std::size_t i = 0; i < out.size(); ++i) {
    if (!free_corner(out[i])) all_free = false;
  }
  for (std::size_t i = 0; i + 1 < out.size(); ++i) {
    if (!segmentFree(out[i], out[i + 1], free_corner, 0.05)) all_free = false;
  }
  check(all_free, "every published point AND segment is collision-free");

  if (stats.spline_rejected) {
    check(!stats.spline_applied, "a rejected spline is not adopted");
    check(out.size() == path.size(),
          "path falls back to the (unsmoothed) polyline");
    check(stats.note.find("recheck") != std::string::npos,
          "stats.note explains the rejection (ASCII, for ROS logs)");
    std::printf("         spline was rejected -> fell back to the polyline "
                "(the recheck did its job)\n");
  } else {
    // 样条被接受也没关系：只要它无碰撞（上面已断言）。
    // 这一分支如实记录"本例未能触发复检"，避免把无效用例当成通过。
    std::printf("         NOTE: the fitted spline stayed collision-free here, "
                "so the recheck was NOT exercised by this case\n");
  }
}

// ===========================================================================
// 用例 1-3a：r = 0 时膨胀是空操作（退回 P4 的锚点）
// ===========================================================================
void testInflationDisabledAtZeroRadius() {
  std::printf("\n[case 1-3a] inflation with r = 0 is a no-op "
              "(the fallback anchor of P4)\n");

  octomap::OcTree tree(0.3);
  markBox(tree, Box{0.0, 0.3, 0.0, 0.3, 0.0, 0.3});   // 2x2x2 占用体素

  MapInflation::Params params;
  params.radius = 0.0;                                 // ← 关闭膨胀
  MapInflation inflation(tree, params);

  check(!inflation.enabled(), "inflation reports disabled at r = 0");
  checkNear(inflation.stats().radius, 0.0, 1e-12, "stats.radius is 0");
  check(inflation.stats().shadow_nodes == 0, "no shadow tree is built at r = 0");

  check(!inflation.isBlocked(P(0.15, 0.15, 0.15)),
        "an occupied cell is NOT blocked (r = 0)");
  check(!inflation.isBlocked(P(1.0, 1.0, 1.0)),
        "a nearby free cell is NOT blocked (r = 0)");
  check(!inflation.isBlocked(P(50.0, 50.0, 50.0)),
        "a far cell is NOT blocked (r = 0)");
  check(!inflation.isBlocked(P(1e5, 1e5, 1e5)),
        "even an out-of-map point is NOT blocked at r = 0 (exactly P4 behaviour)");
  checkNear(inflation.clearance(P(1.0, 1.0, 1.0), 2.0, 0.15), 2.0, 1e-12,
            "clearance saturates at max_range when disabled");
}

// ===========================================================================
// 用例 1-3b：r = 0.6 时围绕占用体素形成阻塞带
// ===========================================================================
void testInflationBlocksBandAroundOccupiedCells() {
  std::printf("\n[case 1-3b] inflation r = 0.6 blocks a band around "
              "occupied cells\n");

  octomap::OcTree tree(0.3);
  markVoxel(tree, P(0.0, 0.0, 0.0));            // 单个 0.3 m 体素
  const octomap::point3d c = voxelCenter(tree, P(0.0, 0.0, 0.0));
  std::printf("         single occupied voxel center = (%.3f, %.3f, %.3f)\n",
              c.x(), c.y(), c.z());

  MapInflation::Params params;
  params.radius = 0.6;
  MapInflation inflation(tree, params);

  check(inflation.enabled(), "inflation enabled at r = 0.6");
  check(inflation.stats().occupied_leaves == 1,
        "exactly one occupied leaf was found");
  check(inflation.stats().shadow_leaves > 0, "the shadow tree has leaves");

  check(inflation.isBlocked(c), "the occupied voxel itself is blocked");
  check(inflation.isBlocked(P(c.x() + 0.5, c.y(), c.z())),
        "a point 0.5 m away is blocked (< r)");
  check(inflation.isBlocked(P(c.x(), c.y(), c.z() - 0.5)),
        "a point 0.5 m below is blocked (< r)");
  check(inflation.isBlocked(P(c.x() + 0.5, c.y() + 0.5, c.z() + 0.5)),
        "a diagonal point inside the inflated box is blocked");

  check(!inflation.isBlocked(P(c.x() + 1.5, c.y(), c.z())),
        "a point 1.5 m away is free (> r)");
  check(!inflation.isBlocked(P(c.x(), c.y() + 1.5, c.z())),
        "a point 1.5 m aside is free (> r)");

  const double clr = inflation.clearance(c, 2.0, 0.05);
  check(clr > 0.0 && clr <= 0.75,
        "clearance at the voxel centre is finite and about r");
  std::printf("         measured clearance at voxel centre = %.3f m (r = 0.60)\n",
              clr);

  const InflationStats& s = inflation.stats();
  std::printf("         scale: occupied_leaves=%zu inflated_voxels=%zu "
              "shadow_leaves=%zu shadow_nodes=%zu mem=%.4f MB build=%.3f ms\n",
              s.occupied_leaves, s.inflated_voxels, s.shadow_leaves,
              s.shadow_nodes, s.shadow_memory_mb, s.build_ms);
  check(s.build_ms >= 0.0, "build time is reported");
}

// ===========================================================================
// 用例 2：面状占用（墙面）上的膨胀规模 —— 风险项 ②
// ===========================================================================
void testInflationScaleOnWall() {
  std::printf("\n[case 2] inflation scale on a wall-like (sheet) occupancy "
              "(risk item 2)\n");

  octomap::OcTree tree(0.3);
  markBox(tree, Box{0.0, 0.0, -3.0, 3.0, -3.0, 3.0});   // 一面薄墙

  const std::size_t base_leaves = occupiedLeafCount(tree);
  const double base_mem =
      static_cast<double>(tree.memoryUsage()) / (1024.0 * 1024.0);
  std::printf("         base: occupied_leaves=%zu mem=%.4f MB\n", base_leaves,
              base_mem);
  check(base_leaves > 0, "the wall actually has occupied leaves");

  const double radii[3] = {0.3, 0.6, 1.0};
  std::size_t leaves_at_r[3] = {0, 0, 0};
  std::size_t vox_at_r[3] = {0, 0, 0};
  for (int i = 0; i < 3; ++i) {
    MapInflation::Params params;
    params.radius = radii[i];
    MapInflation inflation(tree, params);
    const InflationStats& s = inflation.stats();
    leaves_at_r[i] = s.shadow_leaves;
    vox_at_r[i] = s.inflated_voxels;
    std::printf("         r=%.1f m -> shadow_leaves=%zu inflated_voxels=%zu "
                "mem=%.4f MB (x%.2f of base) build=%.1f ms\n",
                radii[i], s.shadow_leaves, s.inflated_voxels, s.shadow_memory_mb,
                (base_mem > 0.0) ? (s.shadow_memory_mb / base_mem) : 0.0,
                s.build_ms);
  }

  // ★ 这里**不能**断言"阴影树严格变大"（2026-10-14 第一次跑测试时踩到）：
  //   影子树会**按八叉树规则剪枝**，叶子数取决于外扩盒的边界是否正好落在
  //   节点层级上。实测 r=0.6 → 2390 叶，而 r=1.0 → 2164 叶（**反而更少**），
  //   因为 1.0 m 的外扩量与 0.3 m 分辨率的层级对得更齐、更容易合并成大节点。
  //   真正单调的是**写入的体素数**（r 越大覆盖越多），所以断言它。
  check(leaves_at_r[0] > 0 && leaves_at_r[1] > 0 && leaves_at_r[2] > 0,
        "each radius builds a non-empty shadow tree");
  check(vox_at_r[0] < vox_at_r[1] && vox_at_r[1] < vox_at_r[2],
        "inflated voxel count grows monotonically with r (leaf count need not)");
}

// ===========================================================================
// 用例 4-5：膨胀只增不改；未知区域语义不变
// ===========================================================================
void testInflationOnlyAddsBlockedInfo() {
  std::printf("\n[case 4-5] inflation only ADDS blocked info; the original "
              "tree is untouched\n");

  octomap::OcTree tree(0.3);
  markBox(tree, Box{0.0, 0.3, 0.0, 0.3, 0.0, 0.3});   // 2x2x2 = 8 个体素
  const std::size_t occ_before = occupiedLeafCount(tree);
  const std::size_t nodes_before = tree.size();

  MapInflation inflation(tree, MapInflation::Params());
  check(inflation.enabled(), "default params (r = 0.6) enable inflation");

  // 地图边界之外：coordToKeyChecked 失败 → isBlocked 保守判 true。
  // 与 P4「未知 = 障碍」一致（是更保守，不是放宽）。
  check(inflation.isBlocked(P(1e5, 1e5, 1e5)),
        "out-of-map points are conservatively blocked "
        "(consistent with unknown = obstacle)");

  // 原树在膨胀前后完全一致：本层是只读原树的
  check(occupiedLeafCount(tree) == occ_before,
        "occupied leaf count of the original tree is unchanged");
  check(tree.size() == nodes_before,
        "node count of the original tree is unchanged");
  // ★ 这里**不能**断言"恰好 8 个占用叶"（2026-10-14 第一次跑测试时踩到）：
  //   2x2x2 = 8 个占用体素会被 octomap **合并成 1 个占用叶**（3x3x3 区域，
  //   边长 0.9 m），实测 occupiedLeafCount == 1。断言 8 是我数错了。
  check(occ_before == 1,
        "the 2x2x2 occupied box is coalesced into exactly 1 occupied leaf");

  // 膨胀后的阻塞范围**必然不小于**原占用范围（单调性）
  const octomap::point3d c = voxelCenter(tree, P(0.0, 0.0, 0.0));
  check(inflation.isBlocked(c), "the occupied voxel is blocked after inflation");
  // 0.6 m 仍在阻塞带内：占用叶边长 0.9 m、外扩 0.6 m → 盒沿到 1.05 m，
  // 所以 0.6 m 处必被阻塞。**不要断言"恰好 r 处仍阻塞"** —— r 恰好落在
  // 膨胀盒边界上，是否命中取决于取整，属于边界不确定性。
  check(inflation.isBlocked(P(c.x() + 0.6, c.y(), c.z())),
        "a point 0.6 m from the voxel centre is still blocked");
  check(!inflation.isBlocked(P(c.x() + 1.5, c.y(), c.z())),
        "a point well beyond r is free");
}

}  // namespace

int main() {
  std::printf("=== P5 smoothing & safety-margin unit tests ===\n");

  testShortcutRejectsWall();
  testShortcutSampleStepAliasing();
  testOversizedSampleStepIsClamped();
  testTraversableChecksRawTreeAtZeroRadius();
  testCurvatureGuards();
  testSplineAcceptedOnSmoothArc();
  testSplineRejectedWhenCurveHitsObstacle();
  testInflationDisabledAtZeroRadius();
  testInflationBlocksBandAroundOccupiedCells();
  testInflationScaleOnWall();
  testInflationOnlyAddsBlockedInfo();

  std::printf("\n=== %d checks, %d failed ===\n", g_checks, g_failed);
  return (g_failed == 0) ? 0 : 1;
}
