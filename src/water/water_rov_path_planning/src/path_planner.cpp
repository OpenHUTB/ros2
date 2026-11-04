#include "water_rov_path_planning/path_planner.hpp"
#include <cmath>
#include <queue>
#include <unordered_map>
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
  // 移动距离
  double dx = to_gx - from_gx;
  double dy = to_gy - from_gy;
  double dist = std::sqrt(dx * dx + dy * dy);

  // 热力图风险代价
  double risk = heatmap_->getRisk(to_gx, to_gy);
  if (risk > params_.max_risk_threshold) {
    return std::numeric_limits<double>::infinity();  // 不可通行
  }

  // 洋流代价
  double cost = params_.length_weight * dist;
  cost += params_.risk_weight * risk * dist;

  if (current_) {
    double wx, wy;
    heatmap_->gridToWorld(to_gx, to_gy, wx, wy);
    Eigen::Vector2d vel = current_->getVelocity(wx, wy);
    double current_mag = vel.norm();

    // 移动方向向量
    Eigen::Vector2d move_dir(dx, dy);
    move_dir.normalize();

    // 洋流与移动方向的点积：顺流为正，逆流为负
    double dot = move_dir.dot(vel);

    // 顺流奖励，逆流惩罚
    double current_factor = params_.current_weight * current_mag;
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
    auto [f, idx] = open.top();
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

  // 路径平滑
  smoothPath(path);
  return true;
}

double PathPlanner::computePathCost(const std::vector<Point> & path) const
{
  double cost = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    double dx = path[i].x - path[i - 1].x;
    double dy = path[i].y - path[i - 1].y;
    cost += std::sqrt(dx * dx + dy * dy);
  }
  return cost;
}

void PathPlanner::smoothPath(std::vector<Point> & path, int iterations)
{
  if (path.size() < 3) return;
  for (int iter = 0; iter < iterations; ++iter) {
    for (size_t i = 1; i < path.size() - 1; ++i) {
      path[i].x = 0.5 * path[i].x + 0.25 * (path[i - 1].x + path[i + 1].x);
      path[i].y = 0.5 * path[i].y + 0.25 * (path[i - 1].y + path[i + 1].y);
    }
  }
}

}  // namespace water_rov_path_planning
