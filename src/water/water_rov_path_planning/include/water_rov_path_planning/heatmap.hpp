#ifndef WATER_ROV_PATH_PLANNING__HEATMAP_HPP_
#define WATER_ROV_PATH_PLANNING__HEATMAP_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace water_rov_path_planning
{

/**
 * @brief 海底环境热力图
 *
 * 将海底环境离散化为二维网格，每个格子存储一个风险值（0~1）。
 * 风险来源：障碍物距离、地形崎岖度、洋流强度。
 * 热力图用 OpenCV Mat 存储，可转换为图像发布用于可视化。
 */
class Heatmap
{
public:
  struct Params
  {
    int grid_size = 100;          // 网格边长（格子数）
    double resolution = 0.5;      // 每个格子对应实际尺寸（米）
    double obstacle_radius = 2.0; // 障碍物影响半径（米）
    double risk_decay = 0.15;     // 风险随距离衰减系数
  };

  explicit Heatmap(const Params & p);

  /// 清空热力图，所有格子风险归零
  void clear();

  /// 添加一个障碍物（世界坐标 x, y，单位米），自动扩散风险
  void addObstacle(double world_x, double world_y);

  /// 设置洋流强度贡献到风险（洋流越强，该格子风险越高）
  void addCurrentRisk(double world_x, double world_y, double current_magnitude);

  /// 获取某格子的风险值（0~1）
  double getRisk(int gx, int gy) const;

  /// 世界坐标转网格坐标
  bool worldToGrid(double wx, double wy, int & gx, int & gy) const;

  /// 网格坐标转世界坐标（格子中心）
  void gridToWorld(int gx, int gy, double & wx, double & wy) const;

  /// 获取热力图尺寸
  int rows() const { return grid_size_; }
  int cols() const { return grid_size_; }
  double resolution() const { return resolution_; }

  /// 生成彩色可视化图像（蓝=低风险，红=高风险）
  cv::Mat toColorImage() const;

  /// 获取原始风险矩阵（CV_64F）
  const cv::Mat & riskMatrix() const { return risk_; }

private:
  int grid_size_;
  double resolution_;
  double obstacle_radius_;
  double risk_decay_;
  cv::Mat risk_;  // CV_64FC1, 0~1

  void spreadRisk(int cx, int cy, double max_val, double radius_m);
};

}  // namespace water_rov_path_planning

#endif  // WATER_ROV_PATH_PLANNING__HEATMAP_HPP_
