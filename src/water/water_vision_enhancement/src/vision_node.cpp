#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include "water_vision_enhancement/image_enhancer.hpp"
#include "water_vision_enhancement/object_tracker.hpp"

#include <opencv2/opencv.hpp>
#include <string>
#include <mutex>

using namespace water_vision_enhancement;

class VisionNode : public rclcpp::Node
{
public:
  VisionNode()
  : Node("water_vision_node")
  {
    // ---- 参数声明 ----
    // 图像增强参数
    declare_parameter("red_gain", 1.35);
    declare_parameter("green_gain", 1.0);
    declare_parameter("blue_gain", 0.85);
    declare_parameter("use_clahe", true);
    declare_parameter("clahe_clip_limit", 2.0);
    declare_parameter("clahe_tile_grid", 8);
    declare_parameter("use_dehaze", true);
    declare_parameter("dehaze_strength", 0.6);
    declare_parameter("atmosphere_light", 0.95);

    // 追踪参数
    declare_parameter("tracker_type", "KCF");
    declare_parameter("min_confidence", 0.3);
    declare_parameter("auto_reinit", true);
    declare_parameter("detect_interval", 30);

    // 初始化框（演示用，可被外部设置）
    declare_parameter("init_bbox_x", 290);
    declare_parameter("init_bbox_y", 220);
    declare_parameter("init_bbox_w", 60);
    declare_parameter("init_bbox_h", 40);

    // 话题名
    declare_parameter("input_topic", "/camera/image_raw");
    declare_parameter("enhanced_topic", "/enhanced_image");
    declare_parameter("tracking_visual_topic", "/tracking_result");
    declare_parameter("object_position_topic", "/tracked_object");
    declare_parameter("metrics_topic", "/enhancement_metrics");

    // ---- 读取参数并初始化增强器 ----
    ImageEnhancer::Params ep;
    ep.red_gain = get_parameter("red_gain").as_double();
    ep.green_gain = get_parameter("green_gain").as_double();
    ep.blue_gain = get_parameter("blue_gain").as_double();
    ep.use_clahe = get_parameter("use_clahe").as_bool();
    ep.clahe_clip_limit = get_parameter("clahe_clip_limit").as_double();
    ep.clahe_tile_grid = get_parameter("clahe_tile_grid").as_int();
    ep.use_dehaze = get_parameter("use_dehaze").as_bool();
    ep.dehaze_strength = get_parameter("dehaze_strength").as_double();
    ep.atmosphere_light = get_parameter("atmosphere_light").as_double();
    enhancer_ = std::make_shared<ImageEnhancer>(ep);

    // ---- 初始化追踪器 ----
    ObjectTracker::Params tp;
    std::string ttype = get_parameter("tracker_type").as_string();
    if (ttype == "CSRT") tp.type = ObjectTracker::TrackerType::CSRT;
    else if (ttype == "MOSSE") tp.type = ObjectTracker::TrackerType::MOSSE;
    else tp.type = ObjectTracker::TrackerType::KCF;
    tp.min_confidence = get_parameter("min_confidence").as_double();
    tp.auto_reinit = get_parameter("auto_reinit").as_bool();
    tp.detect_interval = get_parameter("detect_interval").as_int();
    tracker_ = std::make_shared<ObjectTracker>(tp);

    init_bbox_ = cv::Rect2d(
      get_parameter("init_bbox_x").as_int(),
      get_parameter("init_bbox_y").as_int(),
      get_parameter("init_bbox_w").as_int(),
      get_parameter("init_bbox_h").as_int());

    // ---- 发布者 ----
    enhanced_pub_ = image_transport::create_publisher(this,
      get_parameter("enhanced_topic").as_string());
    tracking_vis_pub_ = image_transport::create_publisher(this,
      get_parameter("tracking_visual_topic").as_string());
    object_pos_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      get_parameter("object_position_topic").as_string(), 10);
    metrics_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      get_parameter("metrics_topic").as_string(), 10);

    // ---- 订阅者 ----
    input_sub_ = image_transport::create_subscription(this,
      get_parameter("input_topic").as_string(),
      std::bind(&VisionNode::imageCallback, this, std::placeholders::_1),
      "raw");

    RCLCPP_INFO(get_logger(),
      "water_vision_node 启动，输入话题: %s，追踪器: %s",
      get_parameter("input_topic").as_string().c_str(), ttype.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // ROS Image -> cv::Mat
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败: %s", e.what());
      return;
    }

    if (frame.empty()) return;

    // 1. 图像增强
    cv::Mat enhanced;
    enhancer_->enhance(frame, enhanced);

    // 2. 质量指标
    double contrast = 0.0, colorfulness = 0.0, psnr = 0.0;
    enhancer_->computeMetrics(frame, enhanced, contrast, colorfulness, psnr);

    // 3. 目标追踪
    cv::Mat tracking_vis = enhanced.clone();
    bool tracking_ok = false;
    cv::Rect2d bbox;

    if (!tracker_->isInitialized()) {
      // 首次：用初始框初始化
      if (init_bbox_.x + init_bbox_.width <= frame.cols &&
          init_bbox_.y + init_bbox_.height <= frame.rows)
      {
        if (tracker_->init(enhanced, init_bbox_)) {
          RCLCPP_INFO(get_logger(), "追踪器初始化，初始框: [%.0f, %.0f, %.0f, %.0f]",
            init_bbox_.x, init_bbox_.y, init_bbox_.width, init_bbox_.height);
        }
      }
    }

    tracking_ok = tracker_->update(enhanced, bbox);

    if (tracking_ok) {
      cv::rectangle(tracking_vis, bbox, cv::Scalar(0, 255, 0), 2);
      cv::circle(tracking_vis, tracker_->getCenter(), 4, cv::Scalar(0, 0, 255), -1);

      // 发布目标位置（归一化坐标 0~1）
      auto pos_msg = geometry_msgs::msg::PointStamped();
      pos_msg.header.stamp = msg->header.stamp;
      pos_msg.header.frame_id = "camera_frame";
      pos_msg.point.x = tracker_->getCenter().x / frame.cols;
      pos_msg.point.y = tracker_->getCenter().y / frame.rows;
      pos_msg.point.z = tracker_->getConfidence();
      object_pos_pub_->publish(pos_msg);
    } else {
      cv::putText(tracking_vis, "Tracking Lost", cv::Point(20, 40),
        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
    }

    // 4. 发布增强图像
    auto enhanced_msg = cv_bridge::CvImage(msg->header, "bgr8", enhanced).toImageMsg();
    enhanced_pub_.publish(*enhanced_msg);

    // 5. 发布追踪可视化图像
    auto vis_msg = cv_bridge::CvImage(msg->header, "bgr8", tracking_vis).toImageMsg();
    tracking_vis_pub_.publish(*vis_msg);

    // 6. 发布质量指标 [contrast, colorfulness, psnr, tracking_confidence]
    auto metrics_msg = std_msgs::msg::Float32MultiArray();
    metrics_msg.data = {
      static_cast<float>(contrast),
      static_cast<float>(colorfulness),
      static_cast<float>(psnr),
      static_cast<float>(tracker_->getConfidence())
    };
    metrics_pub_->publish(metrics_msg);

    frame_count_++;
    if (frame_count_ % 30 == 0) {
      RCLCPP_INFO(get_logger(),
        "帧 %d | 对比度=%.1f 色彩度=%.1f PSNR=%.1f dB | 追踪=%s 置信度=%.2f",
        frame_count_, contrast, colorfulness, psnr,
        tracking_ok ? "OK" : "LOST", tracker_->getConfidence());
    }
  }

  std::shared_ptr<ImageEnhancer> enhancer_;
  std::shared_ptr<ObjectTracker> tracker_;
  cv::Rect2d init_bbox_;
  int frame_count_{0};
  std::mutex mutex_;

  image_transport::Publisher enhanced_pub_;
  image_transport::Publisher tracking_vis_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr object_pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr metrics_pub_;
  image_transport::Subscriber input_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
