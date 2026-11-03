#include "water_vision_enhancement/object_tracker.hpp"

#include <stdexcept>

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

cv::Ptr<tracker_ns::Tracker> ObjectTracker::createTracker()
{
  switch (params_.type) {
    case TrackerType::KCF:
      return tracker_ns::TrackerKCF::create();
    case TrackerType::CSRT:
      return tracker_ns::TrackerCSRT::create();
    case TrackerType::MOSSE:
      return tracker_ns::TrackerMOSSE::create();
    default:
      return tracker_ns::TrackerKCF::create();
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

    // 简单置信度估计：边界框在图像内且尺寸合理
    double cx = center_.x;
    double cy = center_.y;
    bool inside = (cx >= 0 && cx < frame.cols && cy >= 0 && cy < frame.rows);
    bool reasonable_size = (tracked_bbox.width > 5 && tracked_bbox.height > 5);
    confidence_ = (inside && reasonable_size) ? 0.8 : 0.2;

    if (confidence_ >= params_.min_confidence) {
      bbox = tracked_bbox;
      lost_count_ = 0;
      return true;
    }
  }

  // 追踪失败或置信度低
  lost_count_++;
  confidence_ = 0.0;

  if (params_.auto_reinit && lost_count_ >= params_.detect_interval) {
    // 自动重置，等待下一帧重新初始化
    reset();
  }

  return false;
}

void ObjectTracker::reset()
{
  if (tracker_) {
    tracker_.release();
  }
  initialized_ = false;
  current_bbox_ = cv::Rect2d();
  center_ = cv::Point2d();
  confidence_ = 0.0;
  frame_count_ = 0;
  lost_count_ = 0;
}

}  // namespace water_vision_enhancement
