// A* 核心算法的离线单元测试
//
// 刻意**不依赖 ROS 运行时、不依赖模拟器** —— 只 include 头文件、只用 octomap
// 在内存里搭几棵小树。这样编译一次就能跑几百个用例，是调试 A* 的主战场。
//
// 运行方式（catkin_make 之后）：
//   ~/uav_ws/devel/lib/octree_uav_3d_pathfinding/test_astar
// 退出码 0 表示全部通过，非 0 表示有失败用例。

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include <octomap/OcTree.h>

#include "octree_uav_3d_pathfinding/astar.h"

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool ok, const char* what, int line) {
  ++g_checks;
  if (!ok) {
    ++g_failures;
    std::printf("    [FAIL] %s   (line %d)\n", what, line);
  }
}

#define CHECK(cond) check((cond), #cond, __LINE__)

typedef octree_uav_3d_pathfinding::OctreeAStar AStar;
typedef octree_uav_3d_pathfinding::PlanStats PlanStats;

/// 把 [lo, hi) 的立方体区域整块标为「空闲」或「占用」。
void fillBox(octomap::OcTree& tree, const octomap::point3d& lo,
             const octomap::point3d& hi, bool occupied) {
  const double res = tree.getResolution();
  for (double x = lo.x() + res * 0.5; x < hi.x(); x += res) {
    for (double y = lo.y() + res * 0.5; y < hi.y(); y += res) {
      for (double z = lo.z() + res * 0.5; z < hi.z(); z += res) {
        tree.updateNode(octomap::point3d(x, y, z), occupied);
      }
    }
  }
}

/// 收尾：把占用状态传播到内节点，再剪枝 ——
/// 剪枝会把连片的空闲区域塌缩成大节点，这正是我们要测的跨分辨率场景。
void finish(octomap::OcTree& tree) {
  tree.updateInnerOccupancy();
  tree.prune();
}

/// 树里最大的自由叶节点边长（用来确认测试场景真的有多分辨率结构）。
double maxFreeLeafSize(octomap::OcTree& tree) {
  double m = 0.0;
  for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(); it != tree.end(); ++it) {
    if (!tree.isNodeOccupied(*it) && it.getSize() > m) m = it.getSize();
  }
  return m;
}

/// 路径中相邻航点的最大间距。
/// 若它明显大于分辨率，说明 A* 确实跨过了大节点 —— 这是跨分辨率的直接证据。
double maxGap(const std::vector<octomap::point3d>& path) {
  double g = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double d = (path[i] - path[i - 1]).norm();
    if (d > g) g = d;
  }
  return g;
}

/// 打印某点所在叶子的概况（诊断用）。
/// 寻路失败时，先看起点/终点的叶子长什么样、有几个可通行邻居。
void dumpLeaf(AStar& astar, const char* label, const octomap::point3d& p) {
  std::string d;
  astar.probeInfo(p, d);
  std::printf("    %s: %s\n", label, d.c_str());
}

// ---------------------------------------------------------------------------

/// 用例 1：空旷区域应当能规划，且路径长度接近直线距离。
void caseOpenSpace() {
  std::printf("\n[1] 空旷区域\n");
  octomap::OcTree tree(1.0);
  fillBox(tree, octomap::point3d(0, 0, 0), octomap::point3d(20, 20, 20), false);
  finish(tree);

  const octomap::point3d start(2, 2, 2);
  const octomap::point3d goal(18, 18, 18);
  AStar astar(tree, AStar::Params());
  dumpLeaf(astar, "起点", start);
  dumpLeaf(astar, "终点", goal);
  std::vector<octomap::point3d> path;
  PlanStats st;
  const bool ok = astar.plan(start, goal, path, st);

  CHECK(ok);
  CHECK(path.size() >= 2);
  // 注意：航点是**节点中心**，不是用户请求的 start / goal 本身。
  // 起点 (2,2,2) 落在边长 16 m 的粗节点里，该节点中心是 (8,8,8)，
  // 两者相距 10 m —— 路径是从节点中心开始算的。这是八叉树 A* 的固有特性
  // （路径由节点序列构成），不是误差。
  // 所以这里比较"路径是否接近直线"时，用路径自身的首末航点来算。
  const double span = (path.back() - path.front()).norm();
  CHECK(st.path_length >= span * 0.99);   // 不可能比首末航点连线还短
  CHECK(st.path_length <= span * 1.2);    // 也不该绕
  std::printf("    展开 %zu 节点 | 航点 %zu | 长度 %.2f m"
              "（首末航点相距 %.2f m；请求起终点相距 %.2f m）\n",
              st.expanded, st.waypoints, st.path_length, span,
              (goal - start).norm());
}

/// 用例 2：一堵墙中间有个洞 —— 必须能穿洞过去。
void caseWallWithHole() {
  std::printf("\n[2] 墙上有洞\n");
  octomap::OcTree tree(1.0);
  fillBox(tree, octomap::point3d(0, 0, 0), octomap::point3d(20, 20, 20), false);
  // 墙：x ∈ [10, 11] 整面
  fillBox(tree, octomap::point3d(10, 0, 0), octomap::point3d(11, 20, 20), true);
  // 洞：把中间 y,z ∈ [9, 11] 的位置重新挖空
  fillBox(tree, octomap::point3d(10, 9, 9), octomap::point3d(11, 11, 11), false);
  finish(tree);

  const octomap::point3d start(2, 10, 10);
  const octomap::point3d goal(18, 10, 10);
  AStar astar(tree, AStar::Params());
  std::vector<octomap::point3d> path;
  PlanStats st;
  const bool ok = astar.plan(start, goal, path, st);

  CHECK(ok);
  // 穿墙点必须落在洞口附近：取路径中 x 最接近 10.5 的航点，检查它的 y、z
  // 是否落在洞口范围（洞开在 y, z ∈ [9, 11]）。
  //
  // 注意不能用"航点 x ∈ (10, 11)"来判定 —— 一个横跨 x=10 的粗节点，
  // 中心可能正好是 10.0，会被严格不等号漏掉。
  std::size_t nearest = 0;
  double best = 1e9;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double d = std::fabs(path[i].x() - 10.5);
    if (d < best) { best = d; nearest = i; }
  }
  const octomap::point3d cross = path[nearest];
  std::printf("    展开 %zu 节点 | 航点 %zu | 长度 %.2f m | 穿越点 (%.2f, %.2f, %.2f)\n",
              st.expanded, st.waypoints, st.path_length,
              cross.x(), cross.y(), cross.z());
  CHECK(cross.y() > 8.5 && cross.y() < 11.5);
  CHECK(cross.z() > 8.5 && cross.z() < 11.5);
}

/// 用例 3：完全封闭的空腔 —— 必须返回「无解」，而不是死循环或穿墙。
void caseSealed() {
  std::printf("\n[3] 完全封闭（应当无解）\n");
  octomap::OcTree tree(1.0);
  fillBox(tree, octomap::point3d(0, 0, 0), octomap::point3d(20, 20, 20), false);
  // 实心块 [6,14]^3，内部挖出 [9,11]^3 的空腔 —— 壁厚 3 m。
  //
  // 壁厚必须**大于 snapToFree 的搜索半径**（默认 3 m）：否则起点会被
  // 螺旋搜索"穿墙"吸附到外面的自由空间上 —— 吸附只看距离、不看连通性，
  // 这个封闭用例就失去意义了。（吸附半径是给"起点陷在薄障碍里"用的，
  // 无人机场景不会出现"被封死在房间里"的起点。）
  fillBox(tree, octomap::point3d(6, 6, 6), octomap::point3d(14, 14, 14), true);
  fillBox(tree, octomap::point3d(9, 9, 9), octomap::point3d(11, 11, 11), false);
  finish(tree);

  AStar astar(tree, AStar::Params());
  std::vector<octomap::point3d> path;
  PlanStats st;
  const bool ok = astar.plan(octomap::point3d(10, 10, 10),
                             octomap::point3d(18, 18, 18), path, st);

  CHECK(!ok);
  CHECK(!st.reason.empty());
  std::printf("    如期失败：%s（展开 %zu 节点）\n", st.reason.c_str(), st.expanded);
}

/// 用例 4（关键）：跨分辨率 —— 确认树里有粗节点，且 A* 真的跨过了它。
void caseMultiResolution() {
  std::printf("\n[4] 跨分辨率：粗节点邻居\n");
  octomap::OcTree tree(0.5);
  fillBox(tree, octomap::point3d(0, 0, 0), octomap::point3d(20, 20, 20), false);
  finish(tree);   // 剪枝应当把整片空闲塌缩成远大于 0.5 m 的大节点

  const double biggest = maxFreeLeafSize(tree);
  std::printf("    树里最大的自由叶节点边长 = %.2f m（分辨率 %.2f m）\n",
              biggest, tree.getResolution());
  CHECK(biggest > tree.getResolution() * 4.0);   // 场景本身确实是多分辨率的

  AStar astar(tree, AStar::Params());
  dumpLeaf(astar, "起点", octomap::point3d(1, 1, 1));
  dumpLeaf(astar, "终点", octomap::point3d(19, 19, 19));
  std::vector<octomap::point3d> path;
  PlanStats st;
  const bool ok = astar.plan(octomap::point3d(1, 1, 1),
                             octomap::point3d(19, 19, 19), path, st);

  CHECK(ok);
  const double gap = maxGap(path);
  std::printf("    相邻航点最大间距 = %.2f m（%.1f 倍分辨率）\n",
              gap, gap / tree.getResolution());
  // ★ 核心断言：间距远大于分辨率，说明 A* 确实一步跨过了粗节点，
  //   而不是在最细分辨率上一点点挪 —— 这就是跨分辨率邻接生效的直接证据。
  CHECK(gap > tree.getResolution() * 2.0);
}

/// 用例 5：起点落在障碍里 —— 应当吸附到附近最近的空闲节点，仍能规划。
void caseStartInObstacle() {
  std::printf("\n[5] 起点在障碍内\n");
  octomap::OcTree tree(1.0);
  fillBox(tree, octomap::point3d(0, 0, 0), octomap::point3d(20, 20, 20), false);
  fillBox(tree, octomap::point3d(2, 2, 2), octomap::point3d(5, 5, 5), true);
  finish(tree);

  AStar astar(tree, AStar::Params());
  dumpLeaf(astar, "起点", octomap::point3d(3, 3, 3));
  dumpLeaf(astar, "终点", octomap::point3d(18, 18, 18));
  std::vector<octomap::point3d> path;
  PlanStats st;
  const bool ok = astar.plan(octomap::point3d(3, 3, 3),   // 正好在实心块里
                             octomap::point3d(18, 18, 18), path, st);

  CHECK(ok);
  CHECK(st.waypoints >= 1);
  std::printf("    吸附后成功：航点 %zu | 长度 %.2f m\n", st.waypoints, st.path_length);
}

}  // namespace

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  std::printf("==== 八叉树 A* 单元测试 ====\n");

  caseOpenSpace();
  caseWallWithHole();
  caseSealed();
  caseMultiResolution();
  caseStartInObstacle();

  std::printf("\n==== 结果：%d 项检查，%d 项失败 ====\n", g_checks, g_failures);
  if (g_failures == 0) {
    std::printf("全部通过。\n");
    return 0;
  }
  std::printf("有失败项，见上方 [FAIL]。\n");
  return 1;
}
