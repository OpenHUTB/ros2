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
    double current_weight = 3.0;    // 洋流权重（作用于无量纲的 (û·v)/current_ref_speed）
    double length_weight = 1.0;     // 路径长度权重
    double current_compensation = 0.5; // 顺流奖励系数（0~1，且 k·w_c·magnitude/v_ref < w_l）
    double current_ref_speed = 1.0;    // 参考流速（m/s），把 (û·v) 归一化到无量纲
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

  /// 计算路径长度（单位：米），仅用于评估；与 getLastCost() 描述同一条平滑后路径
  double pathLength(const std::vector<Point> & path) const;

  /// 对给定路径按与 stepCost 相同的代价函数逐段重算加权累计代价。
  /// 风险按半格步长采样整段取最大值（与 smoothPath 的安全校验一致），
  /// 而 A* 搜索按目标格取风险，两者采样方式不同，发布代价可能略大于搜索累计的 g 值
  double pathCost(const std::vector<Point> & path) const;

  /// 获取规划的加权累计代价（plan() 在平滑完成后基于最终路径逐段重算）
  double getLastCost() const { return last_cost_; }

  /// 路径平滑：Jacobi 加权平均（用上一轮结果整体更新，避免单向偏移）；
  /// 平滑后逐点、逐段中点以及段内按半格采样检查风险，任一点超阈值则回退到原始 A* 路径
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

  /// 启发函数：欧氏距离换算成米并乘长度权重，与 g 值的长度项同量纲。
  /// 但顺流边每米代价可远低于长度项，h 仍可能高估实际剩余代价，
  /// 因此不满足可采纳性，A* 返回的是近优解而非全局最优
  double heuristic(int gx, int gy, int goal_gx, int goal_gy) const;

  /// 一段世界坐标线段的加权代价（risk 由调用方按代表点取好）
  double segmentCost(double wx0, double wy0, double wx1, double wy1, double risk) const;

  /// 判断世界坐标点所在格子的风险是否不超过阈值（出界视为不安全）
  bool pointSafe(const Point & p) const;
};

}  // namespace water_rov_path_planning

#endif  // WATER_ROV_PATH_PLANNING__PATH_PLANNER_HPP_
