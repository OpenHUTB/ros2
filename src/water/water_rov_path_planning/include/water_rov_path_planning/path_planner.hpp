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
 * 规划出的路径会自动避开高风险区域，
 * 并在洋流方向上选择顺流段、规避强逆流段以节省能耗。
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

  /// 设置规划时刻（秒），用于潮汐模式；规划前由调用方更新为当前时间
  void setTime(double t) { time_ = t; }

  /// 规划路径（起点、终点为世界坐标），返回路径点列表
  bool plan(const Point & start, const Point & goal, std::vector<Point> & path);

  /// 计算路径长度（单位：米），仅用于评估；A* 的加权累计代价见 getLastCost()
  double pathLength(const std::vector<Point> & path) const;

  /// 获取规划的加权累计代价
  double getLastCost() const { return last_cost_; }

  /// 路径平滑：Jacobi 加权平均（用上一轮结果整体更新，避免单向偏移）；
  /// 平滑后逐点与逐段检查风险，超阈值则回退到原始 A* 路径
  /// @return true 表示平滑结果安全可用，false 表示已回退到原始路径
  bool smoothPath(std::vector<Point> & path, int iterations = 3);

  /// 单步加权代价（米制口径：长度、风险、洋流三项都乘以本步米数），供测试与调试
  double stepCost(int from_gx, int from_gy, int to_gx, int to_gy) const;

private:
  Params params_;
  std::shared_ptr<Heatmap> heatmap_;
  std::shared_ptr<OceanCurrent> current_;
  double last_cost_ = 0.0;
  double time_ = 0.0;

  double heuristic(int gx, int gy, int goal_gx, int goal_gy) const;

  /// 判断世界坐标点所在格子的风险是否不超过阈值（出界视为不安全）
  bool pointSafe(const Point & p) const;
};

}  // namespace water_rov_path_planning

#endif  // WATER_ROV_PATH_PLANNING__PATH_PLANNER_HPP_
