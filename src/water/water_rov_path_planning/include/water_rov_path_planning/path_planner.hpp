#ifndef WATER_ROV_PATH_PLANNING__PATH_PLANNER_HPP_
#define WATER_ROV_PATH_PLANNING__PATH_PLANNER_HPP_

#include <vector>
#include <Eigen/Dense>
#include "water_rov_path_planning/heatmap.hpp"
#include "water_rov_path_planning/ocean_current.hpp"

namespace water_rov_path_planning
{

/**
 * @brief 自适应路径规划器
 *
 * 基于 A* 算法，代价函数综合考虑：
 * 1. 热力图风险（障碍物距离、地形崎岖度）
 * 2. 洋流推力（逆流代价高，顺流代价低）
 * 3. 路径长度（启发式）
 *
 * 规划出的路径会自动避开高风险区域和强洋流区域，
 * 并在洋流方向上选择顺流段以节省能耗。
 */
class PathPlanner
{
public:
  struct Params
  {
    double risk_weight = 5.0;       // 热力图风险权重
    double current_weight = 3.0;    // 洋流权重
    double length_weight = 1.0;     // 路径长度权重
    double current_compensation = 0.5; // 顺流奖励系数（0~1）
    double max_risk_threshold = 0.7;   // 风险阈值，超过则不可通行
    int max_iterations = 100000;       // A* 最大迭代次数
  };

  struct Point
  {
    double x;
    double y;
  };

  explicit PathPlanner(const Params & p);

  /// 绑定热力图和洋流场
  void setEnvironment(std::shared_ptr<Heatmap> heatmap,
                      std::shared_ptr<OceanCurrent> current);

  /// 规划路径（起点、终点为世界坐标），返回路径点列表
  bool plan(const Point & start, const Point & goal, std::vector<Point> & path);

  /// 计算某段路径的总代价（用于评估）
  double computePathCost(const std::vector<Point> & path) const;

  /// 获取规划的路径代价
  double getLastCost() const { return last_cost_; }

  /// 路径平滑（B 样条近似）
  static void smoothPath(std::vector<Point> & path, int iterations = 3);

private:
  Params params_;
  std::shared_ptr<Heatmap> heatmap_;
  std::shared_ptr<OceanCurrent> current_;
  double last_cost_ = 0.0;

  // A* 节点
  struct Node
  {
    int gx, gy;
    double g, f;
    int parent_x, parent_y;
    bool in_open;
  };

  double heuristic(int gx, int gy, int goal_gx, int goal_gy) const;
  double stepCost(int from_gx, int from_gy, int to_gx, int to_gy) const;
};

}  // namespace water_rov_path_planning

#endif  // WATER_ROV_PATH_PLANNING__PATH_PLANNER_HPP_
