// 八叉树 A* 全局寻路 —— 路径平滑层
//
// 对应文章第 5 节。设计说明见 notes/P5_安全裕度与路径平滑设计.md §3.4。
//
// 为什么需要这一层
// ----------------
// A* 输出的路径是**八叉树叶节点中心的序列**，所以它天然是折线，而且航点间距
// 很不均匀（空旷处一步跨过十几米，障碍附近一步 0.3 m）。直接把这种折线喂给
// 控制器有两个问题：
//
//   1. 折线拐角处的曲率在数学上是**无穷大** —— 控制器无法跟踪；
//   2. 航点太密（P4 实测 30 个点里很多共线），冗余。
//
// 本层做两步（顺序不能反）：
//
//   第 1 步 捷径化（shortcut）：贪心地把折线取直，丢掉中间多余航点。
//   第 2 步 样条：在取直后的折线上做三次样条（**曲线过航点**），得到曲率连续的光滑路径。
//
// 三条安全上的硬要求（都是踩过坑才定下的）
// ------------------------------------
//  1. **碰撞检测必须在"膨胀图"上做**（调用方传进来的 free 谓词就是膨胀后的判定）。
//     这样平滑本身也受安全裕度约束，**不会把路径拉回到障碍旁边**。
//
//  2. **采样步长必须小于分辨率**（默认 resolution/2）。否则线段会从两个占用体素的
//     对角缝里"穿"过去 —— 这是最容易在上机验收时被抓到的错误。
//
//  3. **样条必须复检，不过就退回折线**。拟合的曲线在拐角**内侧**会向外偏离折线，
//     于是可能侵入障碍。平滑是"增强"，绝不能变成"引入碰撞"：
//     宁可输出一条没平滑的折线，也不输出一条会撞的曲线。
//
// 关于"拟合 vs 逼近"（要写进文档的取舍）
// ------------------------------------
//   · 拟合（曲线**过**航点）：贴合原折线，但拐角内侧会外偏离 → 所以必须复检
//   · 逼近（航点当**控制点**）：曲线在控制点凸包内、更平滑，但**不过航点**，会系统性切角
//
// 本实现选**拟合 + 复检 + 不过就退回**，因为第一优先级是"不引入碰撞"。
// min-snap（最小化 jerk/snap 的多项式轨迹）只讲解、不实现 —— 它是拟合之外的
// 另一条路线（需要给定时间参数化），留作进一步工作。
//
// 本文件只依赖 STL 与 octomap 的 point3d，不依赖 ROS，可离线单元测试。
// 碰撞检测通过**模板谓词**注入，因此本层不直接依赖 MapInflation。

#ifndef OCTREE_UAV_3D_PATHFINDING_PATH_SMOOTHING_H
#define OCTREE_UAV_3D_PATHFINDING_PATH_SMOOTHING_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <octomap/octomap.h>

namespace octree_uav_3d_pathfinding {

/// 平滑的参数与统计。
struct SmoothOptions {
  /// 捷径化：贪心取直
  bool shortcut = true;
  /// 三次样条
  bool spline = true;
  /// 碰撞检测采样步长（m）。**必须 < 分辨率**；<= 0 表示用 resolution/2。
  double sample_step = 0.0;
  /// 样条每段的采样点数（越大曲线越细）
  int spline_samples_per_segment = 8;
  /// 曲率计算的退化守卫阈值
  double curvature_eps = 1e-9;
};

struct SmoothStats {
  bool ok = false;               ///< 平滑流程是否成功走完（不代表结果被采纳）
  std::string note;              ///< ASCII 说明（退化、退回折线等），供日志

  std::size_t input_points = 0;
  std::size_t shortcut_points = 0;   ///< 捷径化后的点数
  std::size_t output_points = 0;     ///< 最终采纳的点数

  bool shortcut_applied = false;
  bool spline_applied = false;
  bool spline_rejected = false;      ///< 复检没过 → 退回折线（**这是正常的失败模式**）

  double shortcut_ms = 0.0;
  double spline_ms = 0.0;
  double recheck_ms = 0.0;

  /// 曲率统计（1/m）
  double max_curvature = 0.0;
  std::size_t degenerate_curvatures = 0;  ///< 命中除零守卫的次数（**风险项 ④** 的直接证据）

  double length = 0.0;               ///< 平滑后路径长度（m）

  /// 实际用于碰撞检测的采样步长（m）。调用方可据此核对"我设的值有没有被改"。
  double sample_step_used = 0.0;

  /// 采样步长是否被限幅过。
  ///
  /// 为什么必须有这个字段：步长 ≥ 分辨率时 `segmentFree` 会跳过中间体素，
  /// 捷径化与样条复检都会漏检障碍 —— 安全保证被破坏。所以 `smoothPath`
  /// 会把它压到 resolution/2 之下；**压过就要留痕**，不能静默改掉用户参数。
  bool sample_step_clamped = false;
};

// ---------------------------------------------------------------------------
// 基础几何
// ---------------------------------------------------------------------------

// 说明：下面所有自由函数都声明为 `static`（内部链接）。
//
// 这不是随手加的 —— 2026-10-13 在客户机上真的踩过一次 ODR 违规：
// map_inflation.h 与 astar.h 里各有一个同名同签名的 childIndex()，
// 链接器只保留一份、静默丢弃另一份，而两份的位序写法不同，
// 结果就是 findLeaf 下潜走错分支、单元测试段错误。
// 头文件里的自由函数一律给内部链接，从结构上消除这类撞名。

/// 折线总长。
static double pathLength(const std::vector<octomap::point3d>& path) {
  double len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    len += (path[i] - path[i - 1]).norm();
  }
  return len;
}

/// 线段是否无碰撞：沿线段按 step 采样，逐点问 free 谓词。
///
/// free 应当返回 true 表示"该点可通行"。传入的通常就是
/// "不在膨胀占用里" —— 于是平滑与规划共享同一套安全语义。
///
/// step 必须 < 分辨率，否则会从体素对角缝里穿过去（见文件头注释 2）。
template <typename FreeFn>
inline bool segmentFree(const octomap::point3d& a, const octomap::point3d& b,
                        FreeFn free, double step) {
  if (step <= 0.0) return false;  // 调用方必须给合法步长：保守拒绝
  const double len = (b - a).norm();
  if (len <= 0.0) return free(a);

  const int n = static_cast<int>(std::ceil(len / step));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    const octomap::point3d p(a.x() + (b.x() - a.x()) * t,
                             a.y() + (b.y() - a.y()) * t,
                             a.z() + (b.z() - a.z()) * t);
    if (!free(p)) return false;
  }
  return true;
}

/// 取 point3d 的第 dim 个分量（0=x, 1=y, 2=z）。
static double component(const octomap::point3d& p, int dim) {
  return (dim == 0) ? p.x() : ((dim == 1) ? p.y() : p.z());
}

/// 单调时钟取毫秒。定义在本文件靠前处，供下面各段计时用。
static double elapsedMs(const std::chrono::steady_clock::time_point& t0) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - t0)
      .count();
}

/// 三点外接圆曲率 κ = 2·|(p1−p0)×(p2−p1)| / (|a|·|b|·|c|)。
///
/// **风险项 ④（曲率除零）**：分母是三条边长的乘积，共线、重点、零长时都会
/// 逼近 0。守卫策略：
///   · 任一邻边长度 < eps  → κ = 0（视为直线）
///   · 叉积模长 < eps      → κ = 0（三点共线）
/// 命中守卫时通过 degenerate 计数上报，**绝不返回 NaN/inf**。
static double pointCurvature(const octomap::point3d& p0,
                             const octomap::point3d& p1,
                             const octomap::point3d& p2,
                             double eps = 1e-9,
                             std::size_t* degenerate = NULL) {
  const octomap::point3d a = p1 - p0;
  const octomap::point3d b = p2 - p1;
  const octomap::point3d c = p2 - p0;

  const double la = a.norm();
  const double lb = b.norm();
  const double lc = c.norm();
  if (la < eps || lb < eps || lc < eps) {
    if (degenerate != NULL) ++(*degenerate);
    return 0.0;
  }

  // octomap::point3d 没有 cross()，自己算
  const double cx = a.y() * b.z() - a.z() * b.y();
  const double cy = a.z() * b.x() - a.x() * b.z();
  const double cz = a.x() * b.y() - a.y() * b.x();
  const double cross = std::sqrt(cx * cx + cy * cy + cz * cz);
  if (cross < eps) {
    if (degenerate != NULL) ++(*degenerate);
    return 0.0;
  }
  return 2.0 * cross / (la * lb * lc);
}

/// 路径上的最大离散曲率（用相邻三点）。
static double maxCurvature(const std::vector<octomap::point3d>& path,
                           double eps = 1e-9,
                           std::size_t* degenerate = NULL) {
  double best = 0.0;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const double k = pointCurvature(path[i - 1], path[i], path[i + 1], eps, degenerate);
    if (k > best) best = k;
  }
  return best;
}

/// 一条路径的对比指标（设计文档 §5）。P4 原路径与 P5 平滑路径各算一份，
/// 打印在同一段日志里，供文章第 5 节直接引用。
struct PathMetrics {
  std::size_t points = 0;
  double length = 0.0;          ///< 折线总长（m）
  double max_curvature = 0.0;   ///< 最大离散曲率（1/m）
  double min_clearance = 0.0;   ///< 到障碍的最小间隙（m，近似值）
  std::size_t degenerate = 0;   ///< 曲率退化（共线/重点）命中次数
};

/// 路径上各点到障碍的最小间隙（m，近似）。
///
/// 为什么是近似：clearance 只沿 6 个轴向步进探测（见 MapInflation::clearance）。
/// 对"安全性指标"来说这个近似是**偏乐观**的（真实最近点可能在对角方向），
/// 所以文档里要如实写"下界性质不保证"，不要当成严格的碰撞证明。
/// 精确做法（对每个点做 26 邻域的欧氏距离变换）代价高得多，留作进一步工作。
template <typename ClearanceFn>
inline double minClearance(const std::vector<octomap::point3d>& path,
                           ClearanceFn clearance, double probe_range) {
  if (path.empty()) return 0.0;
  double best = probe_range;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double c = clearance(path[i]);
    if (c < best) best = c;
  }
  return best;
}

/// 一次性算齐一条路径的对比指标。
template <typename ClearanceFn>
inline PathMetrics computeMetrics(const std::vector<octomap::point3d>& path,
                                  ClearanceFn clearance, double probe_range,
                                  double curvature_eps = 1e-9) {
  PathMetrics m;
  m.points = path.size();
  m.length = pathLength(path);
  m.max_curvature = maxCurvature(path, curvature_eps, &m.degenerate);
  m.min_clearance = minClearance(path, clearance, probe_range);
  return m;
}

// ---------------------------------------------------------------------------
// 第 1 步：捷径化
// ---------------------------------------------------------------------------

/// 贪心捷径化：对每个 i，从最远的 j 往回试，第一条无碰撞的弦就采纳。
///
/// 为什么从最远的 j 试起：取直的收益最大（丢掉的航点最多），而且只要一次成功
/// 就可以跳过去，整体是 O(n²) 条线段、每条 O(len/step) 次检测 —— 对几十个航点
/// 完全够用。
///
/// 首尾两个点**永远保留**：起终点是任务给定或吸附得到的，不能被平滑挪走。
template <typename FreeFn>
inline std::vector<octomap::point3d> shortcutPath(
    const std::vector<octomap::point3d>& path, FreeFn free, double step) {
  if (path.size() <= 2) return path;

  std::vector<octomap::point3d> out;
  out.push_back(path.front());

  std::size_t i = 0;
  while (i + 1 < path.size()) {
    bool advanced = false;
    for (std::size_t j = path.size() - 1; j >= i + 2; --j) {
      if (segmentFree(path[i], path[j], free, step)) {
        out.push_back(path[j]);
        i = j;
        advanced = true;
        break;
      }
      if (j == i + 2) break;  // 防止 size_t 下溢
    }
    if (!advanced) {
      out.push_back(path[i + 1]);
      ++i;
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// 第 2 步：三次样条（自然边界条件，曲线**过**所有航点）
// ---------------------------------------------------------------------------

/// 单段三次样条求值（标准 Hermite 形式）。
///
/// 定义在 cubicSplinePath 之前 —— 被它逐段调用。
static double splineEval(double y0, double y1, double m0, double m1, double h,
                         double u) {
  const double a = 1.0 - u;
  return a * y0 + u * y1 +
         ((a * a * a - a) * m0 + (u * u * u - u) * m1) * (h * h) / 6.0;
}

/// 自然三次样条插值：返回按弦长参数化、逐段采样的平滑路径。
///
/// 实现：按累积弦长参数化 t，对三个坐标分量分别解一次**三对角方程组**
/// （Thomas 算法），自然边界条件 M₀ = M_{n-1} = 0。这是标准做法，
/// 不需要引入 Eigen / 第三方样条库。
///
/// 空输入或少于 3 个点时直接原样返回（无法定义样条）。
inline std::vector<octomap::point3d> cubicSplinePath(
    const std::vector<octomap::point3d>& path, int samples_per_segment) {
  const std::size_t n = path.size();
  if (n < 3) return path;
  if (samples_per_segment < 1) samples_per_segment = 1;

  // ---- 弦长参数化 ----
  std::vector<double> t(n, 0.0);
  for (std::size_t i = 1; i < n; ++i) {
    t[i] = t[i - 1] + (path[i] - path[i - 1]).norm();
  }
  if (t[n - 1] <= 0.0) return path;  // 所有点重合，退化

  std::vector<double> h(n - 1);
  for (std::size_t i = 0; i + 1 < n; ++i) h[i] = t[i + 1] - t[i];

  // ---- 对每个坐标分量解三对角方程（自然边界）----
  std::vector<std::vector<double> > m(3, std::vector<double>(n, 0.0));
  for (int dim = 0; dim < 3; ++dim) {
    std::vector<double> rhs(n, 0.0);
    for (std::size_t i = 1; i + 1 < n; ++i) {
      const double pi0 = component(path[i - 1], dim);
      const double pi1 = component(path[i], dim);
      const double pi2 = component(path[i + 1], dim);
      rhs[i] = 6.0 * ((pi2 - pi1) / h[i] - (pi1 - pi0) / h[i - 1]);
    }

    // Thomas 算法：m[i] = 二阶导
    std::vector<double> a(n, 0.0), b(n, 0.0), c(n, 0.0), d(n, 0.0);
    b[0] = 1.0;  // 自然边界 M0 = 0
    d[0] = 0.0;
    for (std::size_t i = 1; i + 1 < n; ++i) {
      a[i] = h[i - 1];
      b[i] = 2.0 * (h[i - 1] + h[i]);
      c[i] = h[i];
      d[i] = rhs[i];
    }
    b[n - 1] = 1.0;  // 自然边界 M_{n-1} = 0
    d[n - 1] = 0.0;

    for (std::size_t i = 1; i < n; ++i) {
      const double w = (b[i - 1] != 0.0) ? a[i] / b[i - 1] : 0.0;
      b[i] -= w * c[i - 1];
      d[i] -= w * d[i - 1];
    }
    m[dim][n - 1] = (b[n - 1] != 0.0) ? d[n - 1] / b[n - 1] : 0.0;
    for (std::size_t i = n - 1; i-- > 0;) {
      m[dim][i] = (b[i] != 0.0) ? (d[i] - c[i] * m[dim][i + 1]) / b[i] : 0.0;
    }
  }

  // ---- 逐段采样 ----
  std::vector<octomap::point3d> out;
  out.push_back(path.front());
  for (std::size_t i = 0; i + 1 < n; ++i) {
    for (int s = 1; s <= samples_per_segment; ++s) {
      const double u = static_cast<double>(s) / static_cast<double>(samples_per_segment);
      const double x = splineEval(component(path[i], 0), component(path[i + 1], 0),
                                  m[0][i], m[0][i + 1], h[i], u);
      const double y = splineEval(component(path[i], 1), component(path[i + 1], 1),
                                  m[1][i], m[1][i + 1], h[i], u);
      const double z = splineEval(component(path[i], 2), component(path[i + 1], 2),
                                  m[2][i], m[2][i + 1], h[i], u);
      out.push_back(octomap::point3d(x, y, z));
    }
  }
  return out;
}

// ---------------------------------------------------------------------------
// 总入口
// ---------------------------------------------------------------------------

/// 平滑主流程：捷径化 → 样条 → **在膨胀图上复检** → 不过就退回折线。
///
/// free 谓词由调用方注入，通常是"不在膨胀占用里"。
///
/// 返回是否产生了**被采纳**的平滑结果。以下情况都算"正常退回"（不是错误）：
///   · 点数 <= 2
///   · 样条复检没过（spline_rejected = true，stat.note 里有说明）
///   · free 谓词给出非法步长
///
/// 注意：**任何一步失败都退回上一步的结果**，最终至少能返回原始路径的副本。
template <typename FreeFn>
inline bool smoothPath(const std::vector<octomap::point3d>& input,
                       const SmoothOptions& opt, FreeFn free, double resolution,
                       std::vector<octomap::point3d>& out, SmoothStats& stats) {
  const std::chrono::steady_clock::time_point t_all = std::chrono::steady_clock::now();
  stats = SmoothStats();
  stats.input_points = input.size();
  out = input;

  if (input.size() <= 2) {
    stats.note = "too few points to smooth";
    stats.output_points = out.size();
    stats.length = pathLength(out);
    stats.ok = false;
    return false;
  }

  // 采样步长：**必须 < 分辨率**，否则 segmentFree 只在线段两端与少量中间点问
  // free 谓词，会整块跳过中间体素 —— 又细又薄的障碍会被漏检，捷径化与样条复检
  // 双双失效，本模块的安全保证就被破坏。单元测试里有一条反例断言证明这一点
  // （步长 1.0 m 会钻过 0.2 m 厚的墙）。
  //
  // 所以这里**强制** 0 < step < resolution：
  //   · sample_step <= 0        → 取 resolution/2（默认行为）
  //   · sample_step >= resolution → 限幅到 resolution/2，并在 stats 里留痕
  double step = (opt.sample_step > 0.0) ? opt.sample_step : (resolution / 2.0);
  if (resolution > 0.0 && step >= resolution) {
    // 不能静默改用户参数 —— 记下来，调用方要把它打到日志里
    stats.sample_step_clamped = true;
    step = resolution / 2.0;
  }
  if (step <= 0.0) {
    stats.note = "invalid sample step (need resolution > 0 or explicit step)";
    stats.output_points = out.size();
    stats.length = pathLength(out);
    return false;
  }
  stats.sample_step_used = step;

  // ---- 第 1 步：捷径化 ----
  std::vector<octomap::point3d> sh = input;
  if (opt.shortcut) {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    sh = shortcutPath(input, free, step);
    stats.shortcut_ms = elapsedMs(t0);
    stats.shortcut_applied = (sh.size() < input.size());
  }
  stats.shortcut_points = sh.size();

  // ---- 第 2 步：样条 + 复检 ----
  std::vector<octomap::point3d> candidate = sh;
  if (opt.spline && sh.size() >= 3) {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    std::vector<octomap::point3d> curve =
        cubicSplinePath(sh, opt.spline_samples_per_segment);
    stats.spline_ms = elapsedMs(t0);

    // ★ 复检：拟合曲线在拐角内侧会外偏离折线，可能侵入障碍。
    //   逐点问 free；只要有一点越界就**整条丢掉**，退回捷径化折线。
    const std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    bool safe = curve.size() >= 2;
    for (std::size_t i = 0; safe && i < curve.size(); ++i) {
      if (!free(curve[i])) safe = false;
    }
    // 只检查"点"还不够：采样点之间也可能跨过障碍，所以再逐段做一次线段检测
    for (std::size_t i = 0; safe && i + 1 < curve.size(); ++i) {
      if (!segmentFree(curve[i], curve[i + 1], free, step)) safe = false;
    }
    stats.recheck_ms = elapsedMs(t1);

    if (safe) {
      candidate = curve;
      stats.spline_applied = true;
    } else {
      stats.spline_rejected = true;
      stats.note = "spline rejected by collision recheck; fell back to shortcut polyline";
    }
  } else if (opt.spline) {
    stats.note = "spline skipped (fewer than 3 points after shortcut)";
  }

  out = candidate;
  stats.output_points = out.size();
  stats.length = pathLength(out);
  stats.max_curvature = maxCurvature(out, opt.curvature_eps,
                                     &stats.degenerate_curvatures);
  stats.ok = true;
  (void)t_all;
  return stats.spline_applied || stats.shortcut_applied;
}

}  // namespace octree_uav_3d_pathfinding

#endif  // OCTREE_UAV_3D_PATHFINDING_PATH_SMOOTHING_H
