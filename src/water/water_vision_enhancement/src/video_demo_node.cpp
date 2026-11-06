#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>

#include <opencv2/opencv.hpp>
#include <cmath>
#include <random>

/**
 * @brief 模拟浑浊水体视频输入节点
 *
 * 合成水下场景：蓝绿色背景 + 悬浮颗粒噪声 + 一个移动的矩形目标（模拟鱼/物体），
 * 然后对整张图施加雾化和低对比度处理，模拟浑浊水体成像效果。
 * 输出到 /camera/image_raw，供 vision_node 消费。
 */
class VideoDemoNode : public rclcpp::Node
{
public:
  VideoDemoNode()
  : Node("water_video_demo_node")
  {
    declare_parameter("width", 640);
    declare_parameter("height", 480);
    declare_parameter("fps", 20);
    declare_parameter("turbidity", 0.5);          // 浑浊度 0~1
    declare_parameter("target_speed", 80.0);      // 目标移动速度 px/s
    declare_parameter("output_topic", "/camera/image_raw");

    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    fps_ = get_parameter("fps").as_int();
    turbidity_ = get_parameter("turbidity").as_double();
    target_speed_ = get_parameter("target_speed").as_double();
    output_topic_ = get_parameter("output_topic").as_string();

    pub_ = image_transport::create_publisher(this, output_topic_);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / fps_),
      std::bind(&VideoDemoNode::tick, this));

    // 目标初始位置和方向
    target_x_ = width_ / 2.0;
    target_y_ = height_ / 2.0;
    target_angle_ = 0.3;

    RCLCPP_INFO(get_logger(),
      "water_video_demo_node 启动，输出: %s，尺寸 %dx%d，浑浊度 %.2f",
      output_topic_.c_str(), width_, height_, turbidity_);
  }

private:
  void tick()
  {
    // 1. 生成原始清晰水下场景
    cv::Mat frame = cv::Mat::zeros(height_, width_, CV_8UC3);

    // 背景：蓝绿色渐变（模拟水下光线）
    for (int y = 0; y < height_; y++) {
      for (int x = 0; x < width_; x++) {
        double depth_ratio = static_cast<double>(y) / height_;
        frame.at<cv::Vec3b>(y, x) = cv::Vec3b(
          static_cast<uchar>(80 + 40 * depth_ratio),    // B
          static_cast<uchar>(140 + 30 * depth_ratio),   // G
          static_cast<uchar>(30 + 10 * depth_ratio));   // R（水下红光弱）
      }
    }

    // 2. 绘制移动目标（橙色矩形，模拟鱼/物体）
    target_angle_ += 0.02;
    double vx = std::cos(target_angle_) * target_speed_ / fps_;
    double vy = std::sin(target_angle_ * 0.7) * target_speed_ / fps_;
    target_x_ += vx;
    target_y_ += vy;
    // 边界反弹
    if (target_x_ < 40 || target_x_ > width_ - 40) target_angle_ = M_PI - target_angle_;
    if (target_y_ < 40 || target_y_ > height_ - 40) target_angle_ = -target_angle_;
    target_x_ = std::clamp(target_x_, 40.0, width_ - 40.0);
    target_y_ = std::clamp(target_y_, 40.0, height_ - 40.0);

    cv::Rect target_rect(
      static_cast<int>(target_x_ - 30), static_cast<int>(target_y_ - 20), 60, 40);
    cv::rectangle(frame, target_rect, cv::Scalar(30, 120, 220), -1);  // 橙色
    // 目标上的纹理条纹
    for (int i = 0; i < 3; i++) {
      cv::line(frame,
        cv::Point(target_rect.x, target_rect.y + 10 + i * 10),
        cv::Point(target_rect.x + target_rect.width, target_rect.y + 10 + i * 10),
        cv::Scalar(20, 80, 160), 2);
    }

    // 3. 叠加悬浮颗粒噪声
    cv::Mat noise = cv::Mat::zeros(height_, width_, CV_8UC1);
    cv::randu(noise, 0, 255);
    std::vector<cv::Mat> channels(3);
    cv::split(frame, channels);
    for (auto & ch : channels) {
      cv::Mat noise_float;
      noise.convertTo(noise_float, CV_32F, turbidity_ * 0.3, -turbidity_ * 40);
      cv::Mat ch_f;
      ch.convertTo(ch_f, CV_32F);
      ch_f += noise_float;
      ch_f.convertTo(ch, CV_8U);
    }
    cv::merge(channels, frame);

    // 4. 施加雾化（叠加一层蓝白色雾）
    cv::Mat haze = cv::Mat(height_, width_, CV_8UC3,
      cv::Scalar(150, 170, 160));
    cv::addWeighted(frame, 1.0 - turbidity_ * 0.5, haze, turbidity_ * 0.5, 0, frame);

    // 5. 降低对比度
    cv::Mat lab;
    cv::cvtColor(frame, lab, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> lab_ch;
    cv::split(lab, lab_ch);
    // 压缩 L 通道范围
    lab_ch[0].convertTo(lab_ch[0], CV_32F);
    lab_ch[0] = (lab_ch[0] - 128) * (1.0 - turbidity_ * 0.4) + 128;
    lab_ch[0].convertTo(lab_ch[0], CV_8U);
    cv::merge(lab_ch, lab);
    cv::cvtColor(lab, frame, cv::COLOR_Lab2BGR);

    // 6. 轻微模糊模拟散射
    cv::GaussianBlur(frame, frame, cv::Size(3, 3), 0);

    // 7. 发布
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_frame";
    pub_.publish(*msg);
  }

  int width_;
  int height_;
  int fps_;
  double turbidity_;
  double target_speed_;
  std::string output_topic_;

  double target_x_;
  double target_y_;
  double target_angle_;

  image_transport::Publisher pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VideoDemoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
