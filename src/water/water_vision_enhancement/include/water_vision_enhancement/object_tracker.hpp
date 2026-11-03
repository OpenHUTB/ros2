#ifndef WATER_VISION_ENHANCEMENT__OBJECT_TRACKER_HPP_
#define WATER_VISION_ENHANCEMENT__OBJECT_TRACKER_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <string>

// OpenCV 4.5.1 起 KCF / CSRT / MOSSE 等经典追踪器移至 cv::legacy 命名空间，
// 之前的版本仍在 cv 命名空间下，这里按版本统一别名，保证两种环境都能编译。
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && \
    (CV_VERSION_MINOR > 5 || (CV_VERSION_MINOR == 5 && CV_VERSION_REVISION >= 1)))
namespace tracker_ns = cv::legacy;
#else
namespace tracker_ns = cv;
#endif

namespace water_vision_enhancement
{

/**
 * @brief 目标追踪器
 *
 * 在增强后的图像上运行 OpenCV 追踪算法（KCF / CSRT / MOSSE），
 * 输出目标边界框与中心点。首次调用 init 选定初始框后即可连续追踪。
 */
class ObjectTracker
{
public:
  enum class TrackerType
  {
    KCF,     // 速度快，适合纹理丰富目标
    CSRT,    // 精度高，抗遮挡，速度较慢
    MOSSE    // 极快，适合简单目标
  };

  struct Params
  {
    TrackerType type{TrackerType::KCF};
    double min_confidence{0.3};   // 低于此置信度判定为丢失
    bool   auto_reinit{true};     // 丢失后自动重新检测
    int    detect_interval{30};   // 自动重检间隔帧
  };

  ObjectTracker();
  explicit ObjectTracker(const Params & params);

  /**
   * @brief 用初始边界框初始化追踪器
   * @param frame  当前帧
   * @param bbox   初始目标边界框
   * @return true 初始化成功
   */
  bool init(const cv::Mat & frame, const cv::Rect2d & bbox);

  /**
   * @brief 更新追踪
   * @param frame  当前帧
   * @param bbox   输出：当前帧目标边界框
   * @return true 追踪成功，false 目标丢失
   */
  bool update(const cv::Mat & frame, cv::Rect2d & bbox);

  /** 是否已初始化 */
  bool isInitialized() const { return initialized_; }

  /** 获取当前目标中心点（图像坐标） */
  cv::Point2d getCenter() const { return center_; }

  /** 获取当前置信度（0~1） */
  double getConfidence() const { return confidence_; }

  /** 重置追踪器 */
  void reset();

  void setParams(const Params & params) { params_ = params; }
  const Params & getParams() const { return params_; }

private:
  cv::Ptr<tracker_ns::Tracker> createTracker();

  Params params_;
  cv::Ptr<tracker_ns::Tracker> tracker_;
  bool initialized_{false};
  cv::Rect2d current_bbox_;
  cv::Point2d center_;
  double confidence_{0.0};
  int frame_count_{0};
  int lost_count_{0};
};

}  // namespace water_vision_enhancement

#endif  // WATER_VISION_ENHANCEMENT__OBJECT_TRACKER_HPP_
