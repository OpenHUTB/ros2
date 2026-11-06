#include "water_rov_path_planning/path_planner.hpp"
#include <cmath>
#include <limits>
#include <queue>
#include <algorithm>

namespace water_rov_path_planning
{

PathPlanner::PathPlanner(const Params & p)
: params_(p), last_cost_(0.0)
{
}

void PathPlanner::setEnvironment(std::shared_ptr<Heatmap> heatmap,
                                  std::shared_ptr<OceanCurrent> current)
{
  heatmap_ = heatmap;
  current_ = current;
}

double PathPlanner::heuristic(int gx, int gy, int goal_gx, int goal_gy) const
{
  // 欧氏距离
  double dx = gx - goal_gx;
  double dy = gy - goal_gy;
  return std::sqrt(dx * dx + dy * dy);
}

double PathPlanner::stepCost(int from_gx, int from_gy, int to_gx, int to_gy) const
{
  if (!heatmap_) return std::numeric_limits<double>::infinity();

  // 移动距离（格）与对应的实际距离（米）
  double dx = to_gx - from_gx;
  double dy = to_gy - from_gy;
  double dist_cells = std::sqrt(dx * dx + dy * dy);
  double dist_m = dist_cells * heatmap_->resolution();

  // 热力图风险代价
  double risk = heatmap_->getRisk(to_gx, to_gy);
  if (risk > params_.max_risk_threshold) {
    return std::numeric_limits<double>::infinity();  // 不可通行
  }

  // 长度项与风险项都按“单位距离代价 × 本步米数”累加
  double cost = params_.length_weight * dist_m;
  cost += params_.risk_weight * risk * dist_m;

  if (current_ && dist_cells > 0.0) {
    double wx, wy;
    heatmap_->gridToWorld(to_gx, to_gy, wx, wy);
    // 规划时刻参与洋流计算，潮汐模式下才能反映当前相位
    Eigen::Vector2d vel = current_->getVelocity(wx, wy, time_);
    double current_mag = vel.norm();

    // 移动方向单位向量
    Eigen::Vector2d move_dir(dx, dy);
    move_dir.normalize();

    // 洋流与移动方向的点积：顺流为正，逆流为负
    double dot = move_dir.dot(vel);

    // 洋流项同样乘以本步米数，使三项都是“每米代价 × 距离”；
    // |v| 先按参考流速归一化，current_weight 才是无量纲权重
    double ref_speed = std::max(params_.current_ref_speed, 1e-9);
    double current_factor = params_.current_weight * (current_mag / ref_speed) * dist_m;
    if (dot > 0) {
      // 顺流：降低代价
      cost -= params_.current_compensation * current_factor * dot;
    } else {
      // 逆流：增加代价
      cost += current_factor * (-dot);
    }
  }

  return std::max(cost, 0.0);
}

bool PathPlanner::plan(const Point & start, const Point & goal, std::vector<Point> & path)
{
  path.clear();
  if (!heatmap_) return false;

  int start_gx, start_gy, goal_gx, goal_gy;
  if (!heatmap_->worldToGrid(start.x, start.y, start_gx, start_gy)) return false;
  if (!heatmap_->worldToGrid(goal.x, goal.y, goal_gx, goal_gy)) return false;

  int n = heatmap_->rows();

  // A* 开放列表：pair<f, index>
  std::priority_queue<std::pair<double, int>,
    std::vector<std::pair<double, int>>,
    std::greater<std::pair<double, int>>> open;

  // 节点信息
  struct NodeInfo {
    double g = std::numeric_limits<double>::infinity();
    int parent_x = -1, parent_y = -1;
    bool closed = false;
  };
  std::vector<NodeInfo> nodes(n * n);

  int start_idx = start_gy * n + start_gx;
  nodes[start_idx].g = 0.0;
  open.push({heuristic(start_gx, start_gy, goal_gx, goal_gy), start_idx});

  int iterations = 0;
  bool found = false;

  // 8 邻域
  const int dx8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  const int dy8[] = {-1, -1, -1, 0, 0, 1, 1, 1};

  while (!open.empty() && iterations < params_.max_iterations) {
    iterations++;
    int idx = open.top().second;
    open.pop();

    int gx = idx % n;
    int gy = idx / n;

    if (nodes[idx].closed) continue;
    nodes[idx].closed = true;

    if (gx == goal_gx && gy == goal_gy) {
      found = true;
      break;
    }

    for (int i = 0; i < 8; ++i) {
      int nx = gx + dx8[i];
      int ny = gy + dy8[i];
      if (nx < 0 || nx >= n || ny < 0 || ny >= n) continue;

      int nidx = ny * n + nx;
      if (nodes[nidx].closed) continue;

      // 对角移动时禁止贴着封禁格切角：两个正交邻居都必须可通行
      if (dx8[i] != 0 && dy8[i] != 0) {
        if (heatmap_->getRisk(nx, gy) > params_.max_risk_threshold ||
            heatmap_->getRisk(gx, ny) > params_.max_risk_threshold) {
          continue;
        }
      }

      double step = stepCost(gx, gy, nx, ny);
      if (std::isinf(step)) continue;

      double new_g = nodes[idx].g + step;
      if (new_g < nodes[nidx].g) {
        nodes[nidx].g = new_g;
        nodes[nidx].parent_x = gx;
        nodes[nidx].parent_y = gy;
        double h = heuristic(nx, ny, goal_gx, goal_gy);
        open.push({new_g + h, nidx});
      }
    }
  }

  if (!found) return false;

  // 回溯路径
  int cx = goal_gx, cy = goal_gy;
  while (cx != -1 && cy != -1) {
    double wx, wy;
    heatmap_->gridToWorld(cx, cy, wx, wy);
    path.push_back({wx, wy});
    int idx = cy * n + cx;
    int px = nodes[idx].parent_x;
    int py = nodes[idx].parent_y;
    cx = px;
    cy = py;
  }
  std::reverse(path.begin(), path.end());

  last_cost_ = nodes[goal_gy * n + goal_gx].g;

  // 路径平滑：Jacobi 加权平均；若平滑后侵入风险区，smoothPath 内部回退到上面的 A* 原始路径
  smoothPath(path);
  return true;
}

double PathPlanner::pathLength(const std::vector<Point> & path) const
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    double dx = path[i].x - path[i - 1].x;
    double dy = path[i].y - path[i - 1].y;
    length += std::sqrt(dx * dx + dy * dy);
  }
  return length;
}

bool PathPlanner::pointSafe(const Point & p) const
{
  int gx, gy;
  if (!heatmap_->worldToGrid(p.x, p.y, gx, gy)) return false;
  return heatmap_->getRisk(gx, gy) <= params_.max_risk_threshold;
}

bool PathPlanner::smoothPath(std::vector<Point> & path, int iterations)
{
  if (path.size() < 3 || iterations <= 0 || !heatmap_) return true;

  const std::vector<Point> original = path;
  for (int iter = 0; iter < iterations; ++iter) {
    // Jacobi 更新：本轮全部用上一轮的点计算，避免第 i 点依赖已更新的第 i-1 点
    std::vector<Point> next = path;
    for (size_t i = 1; i < path.size() - 1; ++i) {
      next[i].x = 0.5 * path[i].x + 0.25 * (path[i - 1].x + path[i + 1].x);
      next[i].y = 0.5 * path[i].y + 0.25 * (path[i - 1].y + path[i + 1].y);
    }

    // 平滑后逐点、逐段中点以及段内按半格采样检查风险，任一点超阈值则整条回退到原始 A* 路径
    const double sample_step = 0.5 * heatmap_->resolution();
    for (size_t i = 0; i < next.size(); ++i) {
      if (!pointSafe(next[i])) {
        path = original;
        return false;
      }
      if (i + 1 >= next.size()) continue;

      // 长段可能在中间跨过风险区，按半个格子采样逐段校验
      double dx = next[i + 1].x - next[i].x;
      double dy = next[i + 1].y - next[i].y;
      double seg_len = std::sqrt(dx * dx + dy * dy);
      int samples = static_cast<int>(std::ceil(seg_len / sample_step));
      for (int s = 1; s < samples; ++s) {
        double r = static_cast<double>(s) / samples;
        Point p{next[i].x + dx * r, next[i].y + dy * r};
        if (!pointSafe(p)) {
          path = original;
          return false;
        }
      }
    }

    path.swap(next);
  }
  return true;
}

}  // namespace water_rov_path_planning
