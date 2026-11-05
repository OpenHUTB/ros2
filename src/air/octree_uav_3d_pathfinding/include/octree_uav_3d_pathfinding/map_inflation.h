// 八叉树 A* 全局寻路 —— 安全裕度：地图膨胀层
//
// 对应文章第 5 节。设计说明见 notes/P5_安全裕度与路径平滑设计.md §3.2。
//
// 为什么需要这一层
// ----------------
// P4 的路径航点落在**自由叶节点的中心**上，代价是"两节点中心的欧氏距离"，
// 因此路径中心可以任意贴近障碍 —— 只是"贴着走"，并没有任何硬保证。
//
// 本层把原树里的每个**占用叶节点**沿轴向外扩 r（安全裕度），把外扩盒内的体素
// 全部标进一棵**影子树**，于是"是否可通行"多了一道硬判定：
//
//     isBlocked(p) == true  →  p 落在膨胀占用里，不可通行
//
// 这样路径中心到**真障碍**的距离就有了下界 r。
//
// 三个关键的工程决定（都是刻意的）
// ------------------------------
//  1. **不修改原树**。影子树只增不改，原树仍原样拿去发 /octomap_full 与 RViz
//     可视化，"占用体素长什么样"这件事在 P5 前后完全一致。
//
//  2. **不使用 begin_leafs_bbx() 之类的包围盒迭代器**。octomap 的 API 面在不同
//     发行版之间变过（这个项目在 P4 阶段已经吃过"凭记忆写 API"的亏），而且 bbx
//     迭代器按**叶子中心**筛选包围盒 —— 而膨胀要的是"与包围盒**相交**"的叶子，
//     语义本来就不对（一个 16 m 的大空闲叶，中心在盒外、边界却压着盒）。
//
//     所以本层只用 P4 已经实测验证过的那批 API：
//       begin_leafs/end_leafs、keyToCoord、getNodeSize、isNodeOccupied、
//       nodeHasChildren/nodeChildExists/getNodeChild、coordToKeyChecked、
//       updateNode。**一个新接口都没有** —— 风险面为零。
//
//  3. **r = 0 时影子树为空、isBlocked() 恒为 false**，于是整套 P5 代码在
//     r = 0 时行为**逐位等于 P4**。这是回归测试的锚点（见 tests/test_astar.cpp）。
//
// 本文件只依赖 octomap 与 STL，不依赖 ROS，可以离线单元测试。

#ifndef OCTREE_UAV_3D_PATHFINDING_MAP_INFLATION_H
#define OCTREE_UAV_3D_PATHFINDING_MAP_INFLATION_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>

#include <octomap/OcTree.h>

namespace octree_uav_3d_pathfinding {

/// 膨胀层的统计信息（用于日志与文档里的量化对照）。
struct InflationStats {
  bool enabled = false;          ///< r > 0 且建树成功
  double radius = 0.0;           ///< 实际使用的膨胀半径（m）
  double resolution = 0.0;       ///< 影子树分辨率（m）

  std::size_t occupied_leaves = 0;  ///< 原树里的占用叶节点数
  std::size_t source_voxels = 0;    ///< 原占用叶折算到影子分辨率的体素总数
  std::size_t inflated_voxels = 0;  ///< 实际写入影子树的体素个数（去重后）
  std::size_t shadow_leaves = 0;    ///< 影子树的叶节点数
  std::size_t shadow_nodes = 0;     ///< 影子树的节点总数（tree.size()）
  double shadow_memory_mb = 0.0;    ///< 影子树内存（tree.memoryUsage()，字节换算）

  double build_ms = 0.0;         ///< 建树耗时（ms）
};

/// 地图膨胀：由原树生成一棵"膨胀占用"影子树，并提供阻塞判定。
///
/// 用法：
///     MapInflation inflation(tree, params);
///     if (inflation.isBlocked(p)) { ... }     // p 落在安全裕度内 → 不可通行
///
/// 注意：本类**持有** octomap::OcTree 的引用，不拥有它；调用方保证原树的生命周期。
class MapInflation {
 public:
  struct Params {
    /// 安全裕度半径（m）。**设 0 表示关闭膨胀**（整套行为退回 P4）。
    double radius = 0.6;

    /// 影子树分辨率（m）。<= 0 表示与原树一致。
    ///
    /// 通常应等于原树分辨率：与 isBlocked() 的逐格判定配套。
    /// 调大可以省内存，但膨胀边界会变粗糙（比 r 更保守或更激进），需要实测确认。
    double resolution = 0.0;
  };

  MapInflation(octomap::OcTree& tree, const Params& params) : tree_(tree) {
    stats_.radius = params.radius;

    if (params.radius <= 0.0 || tree_.getResolution() <= 0.0) {
      // r = 0（或树没有合法分辨率）→ 不建影子树，isBlocked() 恒 false。
      // 这条分支就是"退回 P4"的保证。
      return;
    }

    const double res =
        (params.resolution > 0.0) ? params.resolution : tree_.getResolution();
    stats_.resolution = res;
    stats_.enabled = true;

    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    shadow_.reset(new octomap::OcTree(res));

    // ---- 遍历原树的占用叶，把外扩盒逐格写进影子树 ----
    //
    // 为什么按"逐格插入"而不是把占用叶整个长大：逐格插入的结果与
    // isBlocked() 的逐格判定完全配套，边界不会有歧义。
    for (octomap::OcTree::leaf_iterator it = tree_.begin_leafs(),
                                        end = tree_.end_leafs();
         it != end; ++it) {
      if (!tree_.isNodeOccupied(*it)) continue;

      ++stats_.occupied_leaves;

      // 节点中心与边长。注意 OcTreeNode 自身不带 key/depth/size，
      // 这三样都只能从遍历器拿（P4 踩坑表第 3 条）。
      const octomap::point3d center =
          tree_.keyToCoord(it.getKey(), it.getDepth());
      const double half = tree_.getNodeSize(it.getDepth()) / 2.0;

      insertInflatedBox(center, half, params.radius);
    }

    stats_.shadow_leaves = countLeaves(*shadow_);
    stats_.shadow_nodes = shadow_->size();
    stats_.shadow_memory_mb =
        static_cast<double>(shadow_->memoryUsage()) / (1024.0 * 1024.0);
    stats_.build_ms = elapsedMs(t0);
  }

  /// 是否落在膨胀占用里（即"到真障碍的距离 < r"）。
  ///
  /// 语义与 P4 的判定并列：调用方应写成
  ///     if (nb.occupied) return;        // 原树占用（不变）
  ///     if (inflation.isBlocked(p)) return;   // ★ 新增：安全裕度
  ///
  /// ★★ 改用 octomap 自己的 search()（2026-10-14 客户机实测后修正）
  ///
  /// 这里原来是自己写的一套下潜，判据是
  ///     if (!nodeHasChildren(n)) return false;      // 当作"未被标占用"
  ///     if (!nodeChildExists(n, idx)) return false; // 当作"该分支未分配"
  /// 实测**是错的**：同一棵树、同样的逐层子节点索引（已验证与 octomap 的
  /// computeChildIdx 逐层一致），我们的下潜判 free、而 octomap 的 search()
  /// 判 occupied —— 4 个连续体素上都不一致（见 .tools/p5_descent_cmp.cpp 的对照）。
  ///
  /// 根因：octomap 在合并/剪枝时会把**连片的空闲兄弟删掉**，把占用状态留在
  /// 父节点上。因此"子槽位为空"**不能推出"该处未被标占用"**。P4 当年为了
  /// 绕开 search()（怕它遇到粗自由节点返回 NULL）才手写了 findLeaf，而那个
  /// 绕开动作正是这一连串 bug 的源头（客户机上 `findLeaf self-check: 172/2000
  /// 查不回来` 也是同一个根因）。
  ///
  /// 为什么在**影子树**里 search() 是可靠的：
  ///   · 影子树是用 updateNode(key, true) 逐个**底层键**插进去的，
  ///     里面根本没有"粗自由节点"，不存在 search() 返回 NULL 的那种情形；
  ///   · search() 是 octomap 自己的、已被充分使用的遍历，判据不会出错。
  ///
  /// 语义：search() 返回 NULL 表示该点未被标占用（影子树里即"不在膨胀占用内"）；
  /// 但**地图边界之外**我们仍保守判为阻塞 —— 与 P4「未知 = 障碍」一致。
  bool isBlocked(const octomap::point3d& p) const {
    if (!stats_.enabled || shadow_ == NULL) return false;

    octomap::OcTreeKey key;
    if (!shadow_->coordToKeyChecked(p, key)) {
      // 出界：保守判为阻塞。注意这只影响"地图边界之外"的点，
      // 而 P4 本来就把未知区域当障碍，所以语义上是自洽的。
      return true;
    }

    octomap::OcTreeNode* n = shadow_->search(p);
    if (n == NULL) return false;              // 未被标占用 → 不在膨胀占用内
    return shadow_->isNodeOccupied(n);
  }

  /// 到最近膨胀边界的近似间隙（m），已按 max_range 截断。
  ///
  /// 用途：代价惩罚（软偏好）需要"这个节点离障碍多远"。
  /// 做法：从 p 沿 6 个轴向往外按 step 步进，数到第一次 isBlocked() 为真。
  ///
  /// **这是一个近似**（只沿 6 个轴、步长有限），代价 O(6 · max_range / step)
  /// 但非常便宜，且对"惩罚"这种软偏好足够。文档里要如实这么写。
  ///
  /// 返回 max_range 表示"在 max_range 之内没有碰到膨胀边界"。
  double clearance(const octomap::point3d& p, double max_range = 2.0,
                   double step = 0.0) const {
    if (!stats_.enabled || shadow_ == NULL) return max_range;

    const double h = (step > 0.0) ? step : stats_.resolution / 2.0;
    const int steps = static_cast<int>(max_range / h + 0.5);
    static const int kAxis[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                                    {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};
    double best = max_range;
    for (int a = 0; a < 6; ++a) {
      for (int i = 1; i <= steps; ++i) {
        const double d = i * h;
        if (d >= best) break;  // 已经不可能更优
        const octomap::point3d q(p.x() + kAxis[a][0] * d,
                                 p.y() + kAxis[a][1] * d,
                                 p.z() + kAxis[a][2] * d);
        if (isBlocked(q)) {
          best = d;
          break;
        }
      }
    }
    return best;
  }

  const InflationStats& stats() const { return stats_; }
  bool enabled() const { return stats_.enabled; }

 private:
  /// 把「中心 center、半边长 half」的占用叶外扩 r 后的盒内体素写进影子树。
  ///
  /// ★★ 必须**按「键」枚举**，不能按 `lo + (i+0.5)*res` 生成坐标
  ///    （2026-10-14 单元测试抓到的真 bug）：
  ///
  ///    octomap 的体素中心落在 `(k + 0.5) * res` 这组格点上。如果自己按
  ///    "盒下界 + 半个体素" 生成中心，得到的格点会与键格**错开半个体素**，
  ///    于是"写入的体素"与"判定时命中的体素"不是同一批 —— 症状是
  ///    **连被膨胀的体素中心自己都判不出阻塞**（实测 clearance 只有 0.45 m
  ///    而非预期的 0.6 m，因为放进影子树的格点是 −0.3/0.0/0.3/0.6/0.9，
  ///    而判定 (0.15,0.15,0.15) 命中的是 0.15 那一格）。
  ///
  ///    所以这里：先把盒的坐标范围用 coordToKeyChecked 落到键上，再**按整数键
  ///    枚举**，最后用 keyToCoord 取回体素中心并做一次回代自检 ——
  ///    保证"插入的位置"与"判定时命中的位置"逐格一致。
  void insertInflatedBox(const octomap::point3d& center, double half, double r) {
    const octomap::point3d lo(center.x() - half - r, center.y() - half - r,
                              center.z() - half - r);
    const octomap::point3d hi(center.x() + half + r, center.y() + half + r,
                              center.z() + half + r);

    octomap::OcTreeKey lo_key;
    octomap::OcTreeKey hi_key;
    if (!shadow_->coordToKeyChecked(lo, lo_key)) return;
    if (!shadow_->coordToKeyChecked(hi, hi_key)) return;

    // 键分量是无符号 16 位、以 tree_max_val 为原点，因此负坐标天然被覆盖。
    // 防御：任何一轴步数超过上限就放弃（说明调用方给的半径异常大）。
    const long long kMaxSteps = 4096;
    const long long k0[3] = {static_cast<long long>(lo_key[0]),
                             static_cast<long long>(lo_key[1]),
                             static_cast<long long>(lo_key[2])};
    const long long k1[3] = {static_cast<long long>(hi_key[0]),
                             static_cast<long long>(hi_key[1]),
                             static_cast<long long>(hi_key[2])};

    for (long long i = k0[0]; i <= k1[0]; ++i) {
      if (i - k0[0] > kMaxSteps) break;
      for (long long j = k0[1]; j <= k1[1]; ++j) {
        if (j - k0[1] > kMaxSteps) break;
        for (long long k = k0[2]; k <= k1[2]; ++k) {
          if (k - k0[2] > kMaxSteps) break;

          const octomap::OcTreeKey key(static_cast<uint16_t>(i),
                                       static_cast<uint16_t>(j),
                                       static_cast<uint16_t>(k));

          // 自检：该键的体素中心折算回键必须还是它自己。
          const octomap::point3d q = shadow_->keyToCoord(key);
          octomap::OcTreeKey back;
          if (!shadow_->coordToKeyChecked(q, back)) continue;
          if (!(back == key)) continue;

          shadow_->updateNode(key, true);
          ++stats_.inflated_voxels;
        }
      }
    }
  }

  /// 【已删除】原来这里有一个手写的子节点索引导出函数 inflChildIndex。
  ///
  /// 它存在的唯一理由是配合"自己写的下潜"。改用 octomap 的 search() 之后
  /// （见 isBlocked 的说明），下潜不存在了，它也就成了死代码 —— 故删除。
  ///
  /// 留这段注释是为了记住两条教训：
  ///   1. 它最初叫 `childIndex`，与 astar.h 里那个**同名同签名**，作为头文件里的
  ///      自由函数构成 **ODR 违规**：链接器只保留一份、静默丢弃另一份。
  ///      后来改名为 inflChildIndex 并加 `static`（内部链接）才从结构上消除。
  ///   2. 位序确实是 **x 在最低位**（`(z<<2)|(y<<1)|x`）—— 这一点已在客户机上
  ///      用 octomap 自带的 computeChildIdx 逐层对照验证过，**两处完全一致**。
  ///      也就是说：这条路径上从来没有位序错误，之前几轮围绕"位序"的怀疑都不成立。

  static std::size_t countLeaves(const octomap::OcTree& tree) {
    std::size_t n = 0;
    for (octomap::OcTree::leaf_iterator it = tree.begin_leafs();
         it != tree.end_leafs(); ++it) {
      ++n;
    }
    return n;
  }

  static double elapsedMs(const std::chrono::steady_clock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - t0)
        .count();
  }
  octomap::OcTree& tree_;                    ///< 原树（不拥有）
  std::unique_ptr<octomap::OcTree> shadow_;  ///< 膨胀占用影子树（r > 0 时才建）
  InflationStats stats_;
};

}  // namespace octree_uav_3d_pathfinding

#endif  // OCTREE_UAV_3D_PATHFINDING_MAP_INFLATION_H
