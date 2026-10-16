// 八叉树上的 A* 全局寻路 —— 核心算法
//
// 设计要点（详见 P4 设计文档 notes/P4_A星寻路设计.md）：
//
//  * 搜索图的节点就是八叉树的**自由叶节点**，不做栅格化 —— 空旷处的大节点
//    一步就能跨过去，这正是八叉树相对均匀栅格的价值所在。
//
//  * 邻接用「探测点法」求 26 邻域：从自身中心沿方向 o 挪 s/2 + resolution/2
//    得到一个探针点，它必定落在紧贴的那个邻居体内（因为任何邻居的边长
//    都 >= resolution）。邻居比自己更细时，探针只命中了接触面上的一个子节点，
//    此时按接触类型（面 k×k、棱 k×1、角 1×1）展开采样。
//
//  * 代价 = 两节点中心的欧氏距离。**不能数步数** —— 跨一个大节点和跨一个
//    小节点的实际位移可能差两个数量级。
//
//  * 未知区域（octomap 里表现为"该分支没有分配节点"）一律当障碍。
//
// 本文件只依赖 octomap 与 STL，不依赖 ROS，因此可以离线单元测试
// （见 tests/test_astar.cpp）—— 编译一次就能跑几百个用例，不用起 ROS，
// 也不用拉模拟器。

#ifndef OCTREE_UAV_3D_PATHFINDING_ASTAR_H
#define OCTREE_UAV_3D_PATHFINDING_ASTAR_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <octomap/OcTree.h>

// 说明：octomap::point3d 是 octomap/math/Vector3.h 里的 typedef，
// 由 OcTree.h 间接引入 —— octomap 并没有 Point3d.h 这个头文件
// （容易和 Pointcloud.h 混淆）。

#include "octree_uav_3d_pathfinding/astar_types.h"

namespace octree_uav_3d_pathfinding {

/// 供可视化使用的「已展开节点」：中心 + 边长。
///
/// 这一路输出是给文档用的 —— 把 A* 实际展开过的节点按尺寸着色画出来，
/// 一眼就能看出「算法在空旷区一步跨过大节点、在障碍附近才细碎展开」，
/// 也就是跨分辨率邻接生效的直观证据。
struct ExpandedCell {
  octomap::point3d center;
  double size;

  ExpandedCell() : size(0.0) {}
  ExpandedCell(const octomap::point3d& c, double s) : center(c), size(s) {}
};

class OctreeAStar {
 public:
  struct Params {
    /// 细侧邻居展开的上限。k = 自身边长 / 邻居边长；k > k_max 时**放弃该方向**。
    ///
    /// 为什么需要这个上限：空旷处一个被剪枝的空闲节点边长可达几十米，
    /// 而它旁边紧挨的细碎结构只有 0.3 m，于是 k 可以大到 128 —— 面接触要采
    /// 16384 个点，不可接受。k 大意味着"紧挨着障碍物或传感器噪声区域"，
    /// 放弃它只会让路径绕远，**不会产生穿墙路径**，是保守的近似。
    double k_max = 4.0;

    /// A* 最大展开次数，防止在大图上跑飞。
    std::size_t max_iterations = 200000;

    /// 启发权重。= 1.0 为最优 A*；> 1.0 为加权 A*，更快但不再保证最优。
    double heuristic_weight = 1.0;

    /// 起终点落在障碍/未知区域时，在该半径（m）内螺旋搜索最近的可行节点。
    double search_radius = 3.0;

    /// 单次规划超时（s）。<= 0 表示不限时。
    double plan_timeout = 10.0;
  };

  /// 注意：这里收的是**非 const 引用**。
  ///
  /// octomap 的 const 是"浅 const" —— const 树上的 getChild()、begin_leafs()
  /// 等接口在部分版本里不可用，收非 const 引用可以避免一连串 const 相关的
  /// 编译错误。A* 本身并不修改地图，这个不变量由调用方保证。
  OctreeAStar(octomap::OcTree& tree, const Params& params)
      : tree_(tree), params_(params) {}

  /// 规划一条从 start 到 goal 的路径（世界系坐标，单位 m）。
  ///
  /// 成功时把节点中心序列写入 path_out（含起点与终点）并返回 true；
  /// 失败时返回 false，具体原因写在 stats.reason 里。
  ///
  /// expanded_out 可选：传入时填入本次实际展开（出堆且非过期）的节点，
  /// 供 RViz 可视化「算法探索过哪些区域、这些节点多大」。
  bool plan(const octomap::point3d& start,
            const octomap::point3d& goal,
            std::vector<octomap::point3d>& path_out,
            PlanStats& stats,
            std::vector<ExpandedCell>* expanded_out = NULL) const {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    stats = PlanStats();
    if (expanded_out != NULL) {
      expanded_out->clear();
    }

    // ---- 1. 吸附起终点 ----
    // 用户给的位置可能落在障碍里或未探索区域，先在附近找最近的可行节点。
    Node start_node;
    if (!snapToFree(start, params_.search_radius, start_node)) {
      stats.reason = "no traversable cell near start";
      stats.elapsed_ms = elapsedMs(t0);
      return false;
    }
    Node goal_node;
    if (!snapToFree(goal, params_.search_radius, goal_node)) {
      stats.reason = "no traversable cell near goal";
      stats.elapsed_ms = elapsedMs(t0);
      return false;
    }

    // 起终点吸附到同一个节点 —— 平凡解
    if (start_node.id == goal_node.id) {
      path_out.assign(1, start_node.center);
      stats.success = true;
      stats.waypoints = 1;
      stats.elapsed_ms = elapsedMs(t0);
      return true;
    }

    // ---- 2. A* 主循环 ----
    std::priority_queue<OpenItem, std::vector<OpenItem>, OpenItemGreater> open;
    std::unordered_map<NodeId, double, NodeIdHash> g_score;
    std::unordered_map<NodeId, NodeId, NodeIdHash> came_from;
    std::unordered_map<NodeId, Node, NodeIdHash> node_of;
    std::unordered_set<NodeId, NodeIdHash> closed;

    g_score[start_node.id] = 0.0;
    node_of[start_node.id] = start_node;
    open.push(OpenItem{weightedHeuristic(start_node, goal_node), start_node});

    bool found = false;
    double elapsed = 0.0;
    std::vector<Node> neighbors;

    while (!open.empty()) {
      if (stats.expanded >= params_.max_iterations) {
        stats.reason = "exceeded max_iterations";
        break;
      }
      if (params_.plan_timeout > 0.0) {
        elapsed = elapsedMs(t0);
        if (elapsed > params_.plan_timeout * 1000.0) {
          stats.reason = "plan timeout";
          break;
        }
      }

      const OpenItem current = open.top();
      open.pop();

      // 惰性删除：同一个节点可能被以更差的 g 入堆多次，出堆时发现已确定最优就丢弃。
      if (closed.find(current.node.id) != closed.end()) continue;
      closed.insert(current.node.id);
      ++stats.expanded;

      if (expanded_out != NULL) {
        expanded_out->push_back(
            ExpandedCell(current.node.center, current.node.size));
      }

      if (current.node.id == goal_node.id) {
        found = true;
        break;
      }

      const double g_current = g_score[current.node.id];

      collectNeighbors(current.node, neighbors);
      for (std::size_t i = 0; i < neighbors.size(); ++i) {
        const Node& nb = neighbors[i];
        if (closed.find(nb.id) != closed.end()) continue;

        const double tentative_g = g_current + cost(current.node, nb);
        const std::unordered_map<NodeId, double, NodeIdHash>::iterator it =
            g_score.find(nb.id);
        if (it != g_score.end() && tentative_g >= it->second) continue;

        g_score[nb.id] = tentative_g;
        came_from[nb.id] = current.node.id;
        node_of[nb.id] = nb;
        open.push(OpenItem{tentative_g + weightedHeuristic(nb, goal_node), nb});
        ++stats.discovered;
      }
    }

    // ---- 3. 回溯 ----
    if (!found) {
      if (stats.reason.empty()) stats.reason = "open set exhausted, goal unreachable";
      stats.elapsed_ms = elapsedMs(t0);
      return false;
    }

    std::vector<NodeId> chain;
    NodeId cursor = goal_node.id;
    while (true) {
      chain.push_back(cursor);
      if (cursor == start_node.id) break;
      const std::unordered_map<NodeId, NodeId, NodeIdHash>::const_iterator it =
          came_from.find(cursor);
      if (it == came_from.end()) {
        stats.reason = "parent chain broken (internal error)";
        stats.elapsed_ms = elapsedMs(t0);
        return false;
      }
      cursor = it->second;
    }
    std::reverse(chain.begin(), chain.end());

    path_out.clear();
    path_out.reserve(chain.size());
    double length = 0.0;
    for (std::size_t i = 0; i < chain.size(); ++i) {
      const Node& n = node_of[chain[i]];
      if (i > 0) length += (n.center - node_of[chain[i - 1]].center).norm();
      path_out.push_back(n.center);
    }

    stats.success = true;
    stats.waypoints = path_out.size();
    stats.path_length = length;
    stats.elapsed_ms = elapsedMs(t0);
    return true;
  }

  /// 诊断用：描述某点所在叶节点，以及它 26 邻域里能走通的邻居数量。
  ///
  /// 寻路失败时第一个要问的就是"起点/终点的叶子长什么样、它有几个邻居"。
  /// 返回 false 表示该点落在未知空间。
  ///
  /// 输出一律 ASCII —— 这个字符串会经由 ROS_WARN 打印，而 roscpp 会把中文
  /// 输出成 '?'。
  bool probeInfo(const octomap::point3d& p, std::string& desc) const {
    Node n;
    if (!findLeaf(p, n)) {
      desc = "unknown space (no node at this point)";
      return false;
    }
    std::vector<Node> nb;
    collectNeighbors(n, nb);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "leaf center=(%.2f, %.2f, %.2f) size=%.2f m depth=%u "
                  "occupied=%s | traversable neighbours=%zu",
                  n.center.x(), n.center.y(), n.center.z(), n.size, n.id.depth,
                  n.occupied ? "yes" : "no", nb.size());
    desc = buf;
    return true;
  }

  /// 自检：按指定的**子节点编号位序**遍历自由叶节点，对每个节点取它的中心
  /// 再查一遍，统计查不回来的个数。
  ///
  /// order 的含义（三个 key 分量谁占高位）：
  ///   0: (x<<2)|(y<<1)|z      —— 目前 findLeaf 用的
  ///   1: (z<<2)|(y<<1)|x      —— x/z 互换
  ///   2: (x<<2)|(z<<1)|y
  ///   3: (y<<2)|(x<<1)|z
  ///
  /// 为什么要试 4 种：起点 (0,0,0) 的 key 是 (32768,32768,32768)，
  /// **三个分量完全相同** —— 任何位序都给出同样的索引，所以它永远查得到，
  /// 把位序错误藏了起来。只有三个分量各不相同的点（房间角落）才会暴露。
  std::size_t selfCheckOrder(unsigned int order, std::size_t limit,
                             std::string& first_detail) const {
    std::size_t checked = 0;
    std::size_t bad = 0;
    first_detail.clear();
    for (octomap::OcTree::leaf_iterator it = tree_.begin_leafs();
         it != tree_.end_leafs() && checked < limit; ++it) {
      if (tree_.isNodeOccupied(*it)) {
        continue;
      }
      ++checked;
      const octomap::point3d c = tree_.keyToCoord(it.getKey(), it.getDepth());

      octomap::OcTreeKey k;
      if (!tree_.coordToKeyChecked(c, k)) {
        ++bad;
        continue;
      }
      octomap::OcTreeNode* cur = tree_.getRoot();
      if (cur == NULL) {
        ++bad;
        continue;
      }
      unsigned int lv = tree_.getTreeDepth();
      for (; lv > 0; --lv) {
        if (!tree_.nodeHasChildren(cur)) break;
        const unsigned int idx = childIndexOrder(k, lv, order);
        if (!tree_.nodeChildExists(cur, idx)) break;
        cur = tree_.getNodeChild(cur, idx);
      }
      // 下潜结束：理想情况下应停在 (depth == it.getDepth())
      const unsigned int reached_depth = tree_.getTreeDepth() - lv;
      if (reached_depth != it.getDepth()) {
        ++bad;
        if (first_detail.empty()) {
          char buf[256];
          std::snprintf(buf, sizeof(buf),
                        "center (%.3f, %.3f, %.3f) expect depth=%u, reached %u",
                        c.x(), c.y(), c.z(), it.getDepth(), reached_depth);
          first_detail = buf;
        }
      }
    }
    return bad;
  }

  /// 自检（当前位序）：遍历自由叶节点，对每个节点取它的**中心**再 findLeaf() 一次，
  /// 看能不能找回**同一个节点**。
  ///
  /// 这是分辨「findLeaf 有 bug」还是「地图里自由空间本来就稀疏」的决定性测试：
  ///   · 大量失配 → findLeaf 的读取逻辑有问题；
  ///   · 全部命中 → findLeaf 没问题，是地图里的自由体素确实稀疏。
  ///
  /// 返回不匹配的个数；first_detail 填第一条不匹配的详情（ASCII）。
  std::size_t selfCheckLookup(std::size_t limit, std::string& first_detail) const {
    std::size_t checked = 0;
    std::size_t bad = 0;
    first_detail.clear();
    for (octomap::OcTree::leaf_iterator it = tree_.begin_leafs();
         it != tree_.end_leafs() && checked < limit; ++it) {
      if (tree_.isNodeOccupied(*it)) {
        continue;
      }
      ++checked;
      const octomap::point3d c = tree_.keyToCoord(it.getKey(), it.getDepth());
      Node n;
      char buf[512];
      if (!findLeaf(c, n)) {
        ++bad;
        if (first_detail.empty()) {
          // 手动重放一次下潜，记录「断在哪一层」——这是定位 bug 的关键。
          octomap::OcTreeKey k2;
          tree_.coordToKeyChecked(c, k2);
          octomap::OcTreeNode* cur = tree_.getRoot();
          unsigned int lv = tree_.getTreeDepth();
          unsigned int broke_idx = 0;
          bool broke_missing_child = false;
          bool broke_no_children = false;
          for (; lv > 0; --lv) {
            if (!tree_.nodeHasChildren(cur)) {
              broke_no_children = true;
              break;
            }
            const unsigned int idx = childIndex(k2, lv);
            if (!tree_.nodeChildExists(cur, idx)) {
              broke_idx = idx;
              broke_missing_child = true;
              break;
            }
            cur = tree_.getNodeChild(cur, idx);
          }
          std::snprintf(buf, sizeof(buf),
                        "leaf center (%.3f, %.3f, %.3f) depth=%u"
                        " | tree_depth=%u | descent broke at level %u"
                        " (%s, childIdx=%u) | key=(%u, %u, %u)",
                        c.x(), c.y(), c.z(), it.getDepth(),
                        tree_.getTreeDepth(), lv,
                        broke_no_children ? "node has NO children array"
                                          : (broke_missing_child ? "child slot is NULL"
                                                                 : "reached max depth"),
                        broke_idx, static_cast<unsigned int>(k2[0]),
                        static_cast<unsigned int>(k2[1]),
                        static_cast<unsigned int>(k2[2]));
          first_detail = buf;
        }
      } else if (n.id.depth != it.getDepth() || !(n.id.key == it.getKey())) {
        ++bad;
        if (first_detail.empty()) {
          std::snprintf(buf, sizeof(buf),
                        "leaf center (%.2f, %.2f, %.2f)"
                        " expected depth=%u, findLeaf got depth=%u",
                        c.x(), c.y(), c.z(), it.getDepth(), n.id.depth);
          first_detail = buf;
        }
      }
    }
    return bad;
  }

  /// 地图里自由叶节点的总数（诊断用；遍历整棵树，慎在大图上频繁调用）。
  std::size_t countFreeLeaves() const {
    std::size_t n = 0;
    for (octomap::OcTree::leaf_iterator it = tree_.begin_leafs(); it != tree_.end(); ++it) {
      if (!tree_.isNodeOccupied(*it)) ++n;
    }
    return n;
  }

 private:
  /// 搜索图的一个节点：叶标识 + 几何。自带 occupied 标记，避免到处传节点指针。
  struct Node {
    NodeId id;
    octomap::point3d center;
    double size;
    bool occupied;

    Node() : size(0.0), occupied(true) {}
    Node(const NodeId& i, const octomap::point3d& c, double s, bool occ)
        : id(i), center(c), size(s), occupied(occ) {}
  };

  struct OpenItem {
    double f;
    Node node;
  };
  struct OpenItemGreater {
    bool operator()(const OpenItem& a, const OpenItem& b) const { return a.f > b.f; }
  };

  static double elapsedMs(const std::chrono::steady_clock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0)
        .count();
  }

  /// 按给定的**位序**计算子节点索引。
  ///
  /// order 决定三个 key 分量谁占高位：
  ///   0: (x<<2)|(y<<1)|z   1: (z<<2)|(y<<1)|x
  ///   2: (x<<2)|(z<<1)|y   3: (y<<2)|(x<<1)|z
  ///
  /// 之所以要做成可切换：octomap 的键是三个 16 位分量，"第 level 层取第几个子节点"
  /// 依赖分量的位序约定，而这份约定在文档里没说清。用 selfCheckOrder() 试一遍
  /// 就能确定哪个是对的 —— 比反复猜测靠谱。
  static unsigned int childIndexOrder(const octomap::OcTreeKey& key,
                                      unsigned int level, unsigned int order) {
    const unsigned int shift = level - 1;
    const unsigned int a = (static_cast<unsigned int>(key[0]) >> shift) & 1u;
    const unsigned int b = (static_cast<unsigned int>(key[1]) >> shift) & 1u;
    const unsigned int c = (static_cast<unsigned int>(key[2]) >> shift) & 1u;
    switch (order) {
      case 1:  return (c << 2) | (b << 1) | a;
      case 2:  return (a << 2) | (c << 1) | b;
      case 3:  return (b << 2) | (a << 1) | c;
      default: return (a << 2) | (b << 1) | c;
    }
  }

  /// octomap 的子节点索引。
  ///
  /// ★ 位序是 (z<<2)|(y<<1)|x —— **x 在最低位**。
  ///
  /// 这一点 octomap 的文档里没有明确说明，而它极其关键：用错位序会让
  /// **绝大多数叶子查不回来**（实测 2000 个自由叶节点里错 1883 个）。
  /// 之所以一直没暴露，是因为起点 (0,0,0) 的 key 是 (32768,32768,32768)，
  /// **三个分量完全相同** —— 任何位序都算出同一个索引。这个巧合把 bug
  /// 藏了很久，直到遍历叶子做自检（selfCheckOrder）才被抓出来。
  ///
  /// 现在直接采用经过自检验证的 order 1。
  static unsigned int childIndex(const octomap::OcTreeKey& key, unsigned int level) {
    return childIndexOrder(key, level, 1);
  }

  /// 由「探针点的最大深度键 + 叶子深度 + 节点指针」构造图节点。
  ///
  /// ★ 关键：NodeId 里的键必须折算到**叶子自己所在的那一层**，不能直接用
  ///   探针点的最大深度键 —— 否则同一个叶子经由不同的探针点找到时，会得到
  ///   不同的 id，导致两个后果：
  ///     1. closed / g_score 的去重完全失效，同一节点被反复展开；
  ///     2. 终点判定永远匹配不上（A* 走到终点叶子时的 id 来自某个探针点，
  ///        而 goal_node.id 来自用户给的终点位置）。
  ///
  ///   只有"叶子边长恰好等于分辨率"时两者才巧合地一致 —— 这也解释了为什么
  ///   细碎场景能跑通、而有大片剪枝粗节点的场景全崩。
  ///
  /// 另外：octomap 的 OcTreeNode 自身不带键与深度，所以这两样都得从
  /// findLeaf() 的下潜过程中带出来。
  Node makeNode(const octomap::OcTreeKey& maxKey, unsigned int depth,
                const octomap::OcTreeNode* n) const {
    const unsigned int shift = tree_.getTreeDepth() - depth;
    const octomap::OcTreeKey leafKey(
        static_cast<uint16_t>(maxKey[0] >> shift),
        static_cast<uint16_t>(maxKey[1] >> shift),
        static_cast<uint16_t>(maxKey[2] >> shift));
    return Node(NodeId(leafKey, depth),
                tree_.keyToCoord(maxKey, depth),   // 中心：用最大深度键折算到该层
                tree_.getNodeSize(depth),
                tree_.isNodeOccupied(n));
  }

  /// 求**包含点 p 的叶节点**。
  ///
  /// 这里刻意不用 OcTree::search()：search() 会一路下潜到指定深度，中途缺少
  /// 子节点就返回 NULL；而被剪枝后的粗节点恰恰就是"没有子节点"，于是会把这个
  /// **有效的空闲粗节点**误判成未知 —— 而 A* 的路径正是要走这些大空闲节点。
  ///
  /// 所以自己下潜：**没有子节点就把它当叶子**；只有"子节点槽位存在但为空"
  /// 才判定为未知（返回 false）。下潜过程中同步记录叶子的深度。
  bool findLeaf(const octomap::point3d& p, Node& out) const {
    octomap::OcTreeKey key;
    if (!tree_.coordToKeyChecked(p, key)) return false;

    octomap::OcTreeNode* n = tree_.getRoot();
    if (n == NULL) return false;

    unsigned int depth = 0;                     // 根节点深度为 0
    for (unsigned int level = tree_.getTreeDepth(); level > 0; --level) {
      // 子节点访问必须走**树上的接口**，不能用节点自己的成员：
      //   octomap 1.9 把 node->hasChildren() 标记为 OCTOMAP_DEPRECATED，
      //   而 node->getChild() 在这个版本里根本不存在。
      //
      // 另外 getNodeChild() 内部是 **assert(node->children[idx] != NULL)** ——
      // 子节点不存在时会直接断言崩溃。而"子节点不存在"恰恰就是我们判定
      // 未知空间的依据，所以必须先用 nodeChildExists() 检查。
      if (!tree_.nodeHasChildren(n)) break;     // 子节点数组未分配 → n 是叶子
      const unsigned int idx = childIndex(key, level);
      if (!tree_.nodeChildExists(n, idx)) return false;   // 槽位存在但为空 → 未知空间
      n = tree_.getNodeChild(n, idx);
      ++depth;
    }
    out = makeNode(key, depth, n);
    return true;
  }

  /// 枚举一个节点的全部可通行邻居（26 邻域，含跨分辨率处理）。
  void collectNeighbors(const Node& n, std::vector<Node>& out) const {
    out.clear();
    for (int ox = -1; ox <= 1; ++ox) {
      for (int oy = -1; oy <= 1; ++oy) {
        for (int oz = -1; oz <= 1; ++oz) {
          if (ox == 0 && oy == 0 && oz == 0) continue;
          probeDirection(n, ox, oy, oz, out);
        }
      }
    }
  }

  /// 单一方向的探测，必要时展开更细的邻居。
  void probeDirection(const Node& n, int ox, int oy, int oz,
                      std::vector<Node>& out) const {
    const double half = n.size / 2.0;
    // 探针偏移：出自身表面再往里半个最小体素。因为任何合法邻居的边长都
    // >= resolution，这个点必定落在紧贴的那个邻居体内，不会跨过它。
    const double delta = tree_.getResolution() / 2.0;

    const octomap::point3d base(n.center.x() + ox * (half + delta),
                                n.center.y() + oy * (half + delta),
                                n.center.z() + oz * (half + delta));

    Node nb;
    if (!findLeaf(base, nb)) return;      // 未知空间
    if (nb.occupied) return;              // 障碍
    if (nb.id == n.id) return;            // 兜底（正常不会发生）

    // 情况一：同层或更粗 —— 一个邻居
    if (nb.size >= n.size) {
      out.push_back(nb);
      return;
    }

    // 情况二：邻居更细 —— 探针只命中了接触面上的一个子节点，需要展开。
    const double k = n.size / nb.size;
    if (k > params_.k_max) return;        // 保守近似：放弃该方向（见 Params::k_max 注释）
    const int ki = static_cast<int>(k + 0.5);

    // 接触类型决定采样点数：o 分量为 0 的轴各取 ki 个点（面 k×k），
    // o 分量非 0 的轴只有 1 个点（棱 k×1、角 1×1）。
    const int nx = (ox == 0) ? ki : 1;
    const int ny = (oy == 0) ? ki : 1;
    const int nz = (oz == 0) ? ki : 1;

    std::unordered_set<NodeId, NodeIdHash> seen;
    for (int i = 0; i < nx; ++i) {
      const double ax = (ox == 0) ? (-half + nb.size / 2.0 + i * nb.size) : 0.0;
      for (int j = 0; j < ny; ++j) {
        const double ay = (oy == 0) ? (-half + nb.size / 2.0 + j * nb.size) : 0.0;
        for (int m = 0; m < nz; ++m) {
          const double az = (oz == 0) ? (-half + nb.size / 2.0 + m * nb.size) : 0.0;
          const octomap::point3d q(base.x() + ax, base.y() + ay, base.z() + az);

          Node c;
          if (!findLeaf(q, c)) continue;
          if (c.occupied) continue;
          if (c.id == n.id) continue;
          if (seen.insert(c.id).second) out.push_back(c);
        }
      }
    }
  }

  /// 把落在障碍/未知里的点吸附到附近最近的可行节点。
  /// 先原地试一次，失败则以 resolution 为步长逐层向外扫壳层。
  bool snapToFree(const octomap::point3d& p, double radius, Node& out) const {
    Node direct;
    if (findLeaf(p, direct) && !direct.occupied) {
      out = direct;
      return true;
    }

    const double step = tree_.getResolution();
    const int layers = static_cast<int>(radius / step + 0.5);
    for (int r = 1; r <= layers; ++r) {
      for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
          for (int dz = -r; dz <= r; ++dz) {
            const int chebyshev = std::max(std::max(std::abs(dx), std::abs(dy)),
                                           std::abs(dz));
            if (chebyshev != r) continue;   // 只扫当前壳层
            const octomap::point3d q(p.x() + dx * step, p.y() + dy * step,
                                     p.z() + dz * step);
            Node cand;
            if (findLeaf(q, cand) && !cand.occupied) {
              out = cand;
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  /// 代价：两节点中心的欧氏距离。
  /// 对粗邻居这个值偏高（高估），会让 A* 偏向细分路径 —— 属于保守方向。
  static double cost(const Node& a, const Node& b) {
    return (b.center - a.center).norm();
  }

  /// 欧氏启发，可采纳（不会高估真实剩余代价）。
  static double heuristic(const Node& a, const Node& b) {
    return (b.center - a.center).norm();
  }

  double weightedHeuristic(const Node& a, const Node& b) const {
    return params_.heuristic_weight * heuristic(a, b);
  }

  octomap::OcTree& tree_;
  Params params_;
};

}  // namespace octree_uav_3d_pathfinding

#endif  // OCTREE_UAV_3D_PATHFINDING_ASTAR_H
