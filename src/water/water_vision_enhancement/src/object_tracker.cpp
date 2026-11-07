#include "water_vision_enhancement/object_tracker.hpp"

#include <algorithm>
#include <stdexcept>

// MOSSE 自 OpenCV 4.5.1 起移入 cv::legacy 命名空间；
// KCF / CSRT 始终留在 cv 命名空间，无需按版本切换。
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && \
    (CV_VERSION_MINOR > 5 || (CV_VERSION_MINOR == 5 && CV_VERSION_REVISION >= 1)))
namespace mosse_ns = cv::legacy;
#else
namespace mosse_ns = cv;
#endif

namespace water_vision_enhancement
{

ObjectTracker::ObjectTracker()
: params_{}
{
}

ObjectTracker::ObjectTracker(const Params & params)
: params_(params)
{
}

cv::Ptr<cv::Tracker> ObjectTracker::createTracker()
{
  switch (params_.type) {
    case TrackerType::KCF:
      return cv::TrackerKCF::create();
    case TrackerType::CSRT:
      return cv::TrackerCSRT::create();
    case TrackerType::MOSSE:
      return mosse_ns::TrackerMOSSE::create();
    default:
      return cv::TrackerKCF::create();
  }
}

bool ObjectTracker::init(const cv::Mat & frame, const cv::Rect2d & bbox)
{
  if (frame.empty() || bbox.width <= 0 || bbox.height <= 0) {
    return false;
  }

  tracker_ = createTracker();
  if (!tracker_) {
    return false;
  }

  bool ok = tracker_->init(frame, bbox);
  if (ok) {
    initialized_ = true;
    current_bbox_ = bbox;
    center_ = cv::Point2d(bbox.x + bbox.width / 2.0, bbox.y + bbox.height / 2.0);
    confidence_ = 1.0;
    frame_count_ = 0;
    lost_count_ = 0;

    // 保存初始框灰度模板，作为后续置信度计算的基准
    cv::Rect roi = cv::Rect(
      static_cast<int>(bbox.x), static_cast<int>(bbox.y),
      static_cast<int>(bbox.width), static_cast<int>(bbox.height)) &
      cv::Rect(0, 0, frame.cols, frame.rows);
    if (roi.width >= 5 && roi.height >= 5) {
      cv::cvtColor(frame(roi), template_, cv::COLOR_BGR2GRAY);
    } else {
      template_.release();
    }
  }
  return ok;
}

bool ObjectTracker::update(const cv::Mat & frame, cv::Rect2d & bbox)
{
  if (!initialized_ || !tracker_ || frame.empty()) {
    return false;
  }

  frame_count_++;
  cv::Rect2d tracked_bbox;
  bool ok = tracker_->update(frame, tracked_bbox);

  if (ok) {
    current_bbox_ = tracked_bbox;
    center_ = cv::Point2d(
      tracked_bbox.x + tracked_bbox.width / 2.0,
      tracked_bbox.y + tracked_bbox.height / 2.0);

    // 真实置信度：当前追踪区域与初始模板的归一化互相关系数
    confidence_ = computeConfidence(frame, tracked_bbox);

    if (confidence_ >= params_.min_confidence) {
      bbox = tracked_bbox;
      lost_count_ = 0;
      return true;
    }
  }

  // 追踪失败或置信度低
  lost_count_++;
  if (!ok) {
    // tracker 本身失败时无有效测量，置信度归零；
    // 置信度低于阈值时保留实测值，保证指标输出为真实测量
    confidence_ = 0.0;
  }

  if (params_.auto_reinit && lost_count_ >= params_.detect_interval) {
    // 自动重置，等待下一帧重新初始化
    reset();
  }

  return false;
}

double ObjectTracker::computeConfidence(const cv::Mat & frame, const cv::Rect2d & bbox)
{
  if (template_.empty()) {
    return 0.0;
  }

  // 追踪框可能漂移出图像，先裁剪到图像范围内
  cv::Rect roi = cv::Rect(
    static_cast<int>(bbox.x), static_cast<int>(bbox.y),
    static_cast<int>(bbox.width), static_cast<int>(bbox.height)) &
    cv::Rect(0, 0, frame.cols, frame.rows);
  if (roi.width < 5 || roi.height < 5) {
    return 0.0;
  }

  cv::Mat patch_gray;
  cv::cvtColor(frame(roi), patch_gray, cv::COLOR_BGR2GRAY);
  if (patch_gray.size() != template_.size()) {
    cv::resize(patch_gray, patch_gray, template_.size());
  }

  cv::Mat result;
  cv::matchTemplate(patch_gray, template_, result, cv::TM_CCOEFF_NORMED);
  double score = static_cast<double>(result.at<float>(0, 0));  // -1 ~ 1
  return std::clamp(score, 0.0, 1.0);
}

void ObjectTracker::reset()
{
  if (tracker_) {
    tracker_.release();
  }
  template_.release();
  initialized_ = false;
  current_bbox_ = cv::Rect2d();
  center_ = cv::Point2d();
  confidence_ = 0.0;
  frame_count_ = 0;
  lost_count_ = 0;
}

}  // namespace water_vision_enhancement
