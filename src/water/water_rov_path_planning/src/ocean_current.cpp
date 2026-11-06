#include "water_rov_path_planning/ocean_current.hpp"
#include <algorithm>
#include <cmath>

namespace water_rov_path_planning
{

OceanCurrent::OceanCurrent(const Params & p)
: params_(p), time_(0.0)
{
}

Eigen::Vector2d OceanCurrent::getVelocity(double world_x, double world_y, double t) const
{
  Eigen::Vector2d vel(0.0, 0.0);
  double m = params_.magnitude;

  switch (params_.pattern) {
    case Pattern::UNIFORM:
      vel.x() = m * std::cos(params_.direction);
      vel.y() = m * std::sin(params_.direction);
      break;

    case Pattern::VORTEX: {
      double dx = world_x - params_.vortex_x;
      double dy = world_y - params_.vortex_y;
      double r = std::sqrt(dx * dx + dy * dy);
      if (r < 0.01 || r > params_.vortex_radius) break;
      // 切向速度（逆时针旋转：(vx, vy) ∝ (-dy, dx)）
      double tangential = m * (1.0 - r / params_.vortex_radius);
      vel.x() = -dy / r * tangential;
      vel.y() = dx / r * tangential;
      break;
    }

    case Pattern::TIDAL: {
      // 潮汐：流速随时间正弦变化
      double factor = std::sin(2.0 * M_PI * t / params_.tidal_period);
      vel.x() = m * factor * std::cos(params_.direction);
      vel.y() = m * factor * std::sin(params_.direction);
      break;
    }

    case Pattern::SHEAR: {
      // 剪切流：流速随 y 坐标线性变化
      double shear = m * (world_y / params_.world_size - 0.5) * 2.0;
      vel.x() = shear * std::cos(params_.direction);
      vel.y() = shear * std::sin(params_.direction) * 0.3;
      break;
    }
  }

  return vel;
}

cv::Mat OceanCurrent::toVectorImage(int img_size) const
{
  cv::Mat img = cv::Mat::zeros(img_size, img_size, CV_8UC3);
  img.setTo(cv::Scalar(20, 20, 20));  // 深色背景

  // img_size < 20 时整数除法会得到 0，导致下面的循环步长无效（死循环）
  int step = std::max(1, img_size / 20);
  for (int py = step; py < img_size; py += step) {
    for (int px = step; px < img_size; px += step) {
      double wx = (static_cast<double>(px) / img_size) * params_.world_size;
      double wy = (static_cast<double>(py) / img_size) * params_.world_size;
      Eigen::Vector2d v = getVelocity(wx, wy, time_);
      double mag = v.norm();
      if (mag < 0.01) continue;

      // 箭头长度与流速成正比
      double arrow_len = std::min(mag * 30.0, step * 0.8);
      double angle = std::atan2(v.y(), v.x());
      int ex = px + static_cast<int>(arrow_len * std::cos(angle));
      int ey = py + static_cast<int>(arrow_len * std::sin(angle));

      // 颜色：流速越大越亮（magnitude 为 0 时该点会因 mag<0.01 被跳过，这里仍做保护）
      double mag_scale = std::max(params_.magnitude, 1e-9);
      int intensity = static_cast<int>(std::min(mag / mag_scale, 1.0) * 255);
      cv::arrowedLine(img, cv::Point(px, py), cv::Point(ex, ey),
        cv::Scalar(intensity, 255 - intensity / 2, 255 - intensity), 1, cv::LINE_AA, 0, 0.3);
    }
  }
  return img;
}

}  // namespace water_rov_path_planning
