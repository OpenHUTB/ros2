#include "water_rov_path_planning/heatmap.hpp"
#include <cmath>
#include <algorithm>

namespace water_rov_path_planning
{

Heatmap::Heatmap(const Params & p)
: grid_size_(p.grid_size),
  resolution_(p.resolution),
  obstacle_radius_(p.obstacle_radius),
  risk_decay_(p.risk_decay)
{
  risk_ = cv::Mat::zeros(grid_size_, grid_size_, CV_64FC1);
}

void Heatmap::clear()
{
  risk_ = cv::Mat::zeros(grid_size_, grid_size_, CV_64FC1);
}

void Heatmap::addObstacle(double world_x, double world_y)
{
  int gx, gy;
  if (!worldToGrid(world_x, world_y, gx, gy)) return;
  spreadRisk(gx, gy, 1.0, obstacle_radius_);
}

void Heatmap::addCurrentRisk(double world_x, double world_y, double current_magnitude)
{
  int gx, gy;
  if (!worldToGrid(world_x, world_y, gx, gy)) return;
  // 洋流强度 > 0.5 m/s 时开始增加风险
  double risk = std::clamp((current_magnitude - 0.5) / 1.0, 0.0, 0.6);
  if (risk > 0.0) {
    spreadRisk(gx, gy, risk, 3.0);  // 洋流风险影响半径 3 米
  }
}

void Heatmap::spreadRisk(int cx, int cy, double max_val, double radius_m)
{
  int radius_cells = static_cast<int>(radius_m / resolution_);
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      int x = cx + dx;
      int y = cy + dy;
      if (x < 0 || x >= grid_size_ || y < 0 || y >= grid_size_) continue;
      double dist = std::sqrt(dx * dx + dy * dy) * resolution_;
      if (dist > radius_m) continue;
      // 高斯衰减
      double decay = std::exp(-risk_decay_ * dist * dist);
      double val = max_val * decay;
      double & cur = risk_.at<double>(y, x);
      cur = std::max(cur, val);
      cur = std::min(cur, 1.0);
    }
  }
}

double Heatmap::getRisk(int gx, int gy) const
{
  if (gx < 0 || gx >= grid_size_ || gy < 0 || gy >= grid_size_) return 1.0;
  return risk_.at<double>(gy, gx);
}

bool Heatmap::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
  gx = static_cast<int>(wx / resolution_);
  gy = static_cast<int>(wy / resolution_);
  return (gx >= 0 && gx < grid_size_ && gy >= 0 && gy < grid_size_);
}

void Heatmap::gridToWorld(int gx, int gy, double & wx, double & wy) const
{
  wx = (gx + 0.5) * resolution_;
  wy = (gy + 0.5) * resolution_;
}

cv::Mat Heatmap::toColorImage() const
{
  // 风险 0 -> 蓝色，0.5 -> 黄色，1.0 -> 红色
  cv::Mat color(grid_size_, grid_size_, CV_8UC3);
  for (int y = 0; y < grid_size_; ++y) {
    for (int x = 0; x < grid_size_; ++x) {
      double r = risk_.at<double>(y, x);
      // HSV: H=240(蓝) -> H=0(红), S=255, V=255
      double h = 240.0 * (1.0 - r);
      cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h, 255, 255));
      cv::Mat bgr;
      cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
      color.at<cv::Vec3b>(y, x) = bgr.at<cv::Vec3b>(0, 0);
    }
  }
  // 放大方便查看
  cv::Mat big;
  cv::resize(color, big, cv::Size(), 4, 4, cv::INTER_NEAREST);
  return big;
}

}  // namespace water_rov_path_planning
