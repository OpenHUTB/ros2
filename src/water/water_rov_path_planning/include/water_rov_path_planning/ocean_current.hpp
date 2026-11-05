#ifndef WATER_ROV_PATH_PLANNING__OCEAN_CURRENT_HPP_
#define WATER_ROV_PATH_PLANNING__OCEAN_CURRENT_HPP_

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace water_rov_path_planning
{

/**
 * @brief 洋流场模型
 *
 * 用二维向量场表示海底洋流。每个位置有一个流速向量（vx, vy），单位 m/s。
 * 支持多种洋流模式：均匀流、涡流、潮汐振荡、剪切流。
 * 用于路径规划时估算洋流对 ROV 的推力干扰。
 */
class OceanCurrent
{
public:
  enum class Pattern
  {
    UNIFORM,   // 均匀流
    VORTEX,    // 涡流（顺时针/逆时针旋转）
    TIDAL,     // 潮汐振荡（随时间周期变化）
    SHEAR      // 剪切流（流速随位置变化）
  };

  struct Params
  {
    Pattern pattern = Pattern::VORTEX;
    double magnitude = 0.4;     // 基础流速（m/s）
    double direction = 0.0;     // 均匀流方向（弧度，0=+x）
    double vortex_x = 25.0;     // 涡流中心 x（米）
    double vortex_y = 25.0;     // 涡流中心 y（米）
    double vortex_radius = 15.0; // 涡流影响半径（米）
    double tidal_period = 60.0;  // 潮汐周期（秒）
    double world_size = 50.0;    // 世界尺寸（米）
  };

  explicit OceanCurrent(const Params & p);

  /// 获取某位置的洋流速度向量（vx, vy），单位 m/s
  Eigen::Vector2d getVelocity(double world_x, double world_y, double t = 0.0) const;

  /// 计算洋流对 ROV 运动的代价（流速越大代价越高）
  double getCost(double world_x, double world_y, double t = 0.0) const;

  /// 生成洋流场可视化图像（箭头图）
  cv::Mat toVectorImage(int img_size = 400) const;

  /// 设置当前时间（用于潮汐模式）
  void setTime(double t) { time_ = t; }

private:
  Params params_;
  double time_ = 0.0;
};

}  // namespace water_rov_path_planning

#endif  // WATER_ROV_PATH_PLANNING__OCEAN_CURRENT_HPP_
