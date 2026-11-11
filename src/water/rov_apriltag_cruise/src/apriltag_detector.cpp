// AprilTag（ArUco 6x6 字典）视觉检测节点
// 订阅 ROV 前置相机图像，输出带识别框的标注图像、标签位姿 TF 与检测结果
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

class ApriltagDetector : public rclcpp::Node
{
public:
  ApriltagDetector() : Node("apriltag_detector")
  {
    marker_size_ = declare_parameter("marker_size", 0.6);
    show_gui_ = declare_parameter("show_gui", true);
    const double fx = declare_parameter("fx", 459.0);
    const double fy = declare_parameter("fy", 459.0);
    camera_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0.0, 320.0, 0.0, fy, 240.0, 0.0, 0.0, 1.0);
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->adaptiveThreshWinSizeMin = 3;
    detector_params_->adaptiveThreshWinSizeMax = 23;
    detector_params_->adaptiveThreshWinSizeStep = 2;
    detector_params_->minMarkerPerimeterRate = 0.01;
    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    // 标牌为「白色底板 + 黑色标记」，开启反转检测才能稳定识别完整标牌
    detector_params_->detectInvertedMarker = true;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/rov/camera/image_detected", 5);
    det_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/rov/tag_detections", 10);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/rov/camera/image_raw", 5,
      std::bind(&ApriltagDetector::onImage, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/rov/camera/camera_info", 5,
      [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (msg->k[0] > 0.0 && msg->k[4] > 0.0) {
          camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            msg->k[0], 0.0, msg->k[2], 0.0, msg->k[4], msg->k[5], 0.0, 0.0, 1.0);
          if (msg->d.size() >= 5) {
            dist_coeffs_ = cv::Mat(5, 1, CV_64F);
            for (int i = 0; i < 5; ++i) { dist_coeffs_.at<double>(i) = msg->d[i]; }
          }
        }
      });

    RCLCPP_INFO(get_logger(), "AprilTag 检测节点已启动，marker_size=%.2f m", marker_size_);
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "图像转换失败: %s", e.what());
      return;
    }

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_);

    std_msgs::msg::Float32MultiArray det;
    det.data.clear();

    if (!ids.empty()) {
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(
        corners, static_cast<float>(marker_size_), camera_matrix_, dist_coeffs_, rvecs, tvecs);

      for (size_t i = 0; i < ids.size(); ++i) {
        cv::Vec3d rvec = rvecs[i];
        cv::Vec3d tvec = tvecs[i];
        cv::aruco::drawAxis(frame, camera_matrix_, dist_coeffs_, rvec, tvec, 0.25f);

        const double dist = cv::norm(tvec);
        cv::putText(frame, "ID " + std::to_string(ids[i]),
                    corners[i][0], cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 255, 0), 2);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%.2f m", dist);
        cv::putText(frame, buf, corners[i][0] + cv::Point2f(0.0f, 30.0f),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

        // 相机坐标系（x 前, y 左, z 上）与 OpenCV 光心坐标系（x 右, y 下, z 前）的旋转
        cv::Mat R_opt = (cv::Mat_<double>(3, 3) << 0, 0, 1, -1, 0, 0, 0, -1, 0);
        cv::Mat R_marker, R_cam;
        cv::Rodrigues(rvec, R_marker);
        R_cam = R_opt * R_marker;
        cv::Mat t_cam = R_opt * cv::Mat(tvec);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = msg->header.stamp;
        tf.header.frame_id = "camera_link";
        tf.child_frame_id = "tag_" + std::to_string(ids[i]);
        tf.transform.translation.x = t_cam.at<double>(0);
        tf.transform.translation.y = t_cam.at<double>(1);
        tf.transform.translation.z = t_cam.at<double>(2);
        tf2::Matrix3x3 rot(
          R_cam.at<double>(0, 0), R_cam.at<double>(0, 1), R_cam.at<double>(0, 2),
          R_cam.at<double>(1, 0), R_cam.at<double>(1, 1), R_cam.at<double>(1, 2),
          R_cam.at<double>(2, 0), R_cam.at<double>(2, 1), R_cam.at<double>(2, 2));
        tf2::Quaternion q;
        rot.getRotation(q);
        q.normalize();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(tf);

        det.data.push_back(static_cast<float>(ids[i]));
        det.data.push_back(static_cast<float>(t_cam.at<double>(0)));
        det.data.push_back(static_cast<float>(t_cam.at<double>(1)));
        det.data.push_back(static_cast<float>(t_cam.at<double>(2)));
        det.data.push_back(static_cast<float>(dist));
      }

      cv::aruco::drawDetectedMarkers(frame, corners, ids);
    }

    // 画面左上角状态栏
    cv::rectangle(frame, cv::Rect(0, 0, 400, 34), cv::Scalar(0, 0, 0), cv::FILLED);
    char bar[96];
    std::snprintf(bar, sizeof(bar), "ROV CAMERA  |  AprilTag detected: %zu", ids.size());
    cv::putText(frame, bar, cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.62,
                ids.empty() ? cv::Scalar(0, 200, 255) : cv::Scalar(0, 255, 0), 2);

    sensor_msgs::msg::Image out;
    cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg(out);
    image_pub_->publish(out);
    det_pub_->publish(det);

    if (show_gui_) {
      if (!window_ready_) {
        cv::namedWindow("ROV Camera - AprilTag", cv::WINDOW_NORMAL);
        cv::resizeWindow("ROV Camera - AprilTag", 720, 540);
        // 不指定窗口位置：由窗口管理器决定，避免在小分辨率屏幕上跑出屏幕外
        window_ready_ = true;
      }
      cv::imshow("ROV Camera - AprilTag", frame);
      cv::waitKey(1);
    }
  }

  double marker_size_{0.6};
  bool show_gui_{true};
  bool window_ready_{false};
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr det_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApriltagDetector>());
  rclcpp::shutdown();
  return 0;
}
