#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "humanoid_reachability/kinematics.hpp"
#include "humanoid_reachability/workspace_analyzer.hpp"

namespace humanoid_reachability
{
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using std_msgs::msg::Float64MultiArray;
using std_msgs::msg::String;
using geometry_msgs::msg::PointStamped;

class ReachabilityNode : public rclcpp::Node
{
public:
  ReachabilityNode()
  : Node("humanoid_reachability")
  {
    samples_per_arm_ = declare_parameter<int>("samples_per_arm", 200000);
    voxel_resolution_ = declare_parameter<double>("voxel_resolution", 0.03);
    slice_height_ = declare_parameter<double>("slice_height", 0.05);
    random_seed_ = declare_parameter<int>("random_seed", 2026);
    max_display_voxels_ = declare_parameter<int>("max_display_voxels", 2500);
    publish_rate_ = declare_parameter<double>("publish_rate", 2.0);

    if (voxel_resolution_ <= 0.0 || slice_height_ <= 0.0 || publish_rate_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "voxel_resolution / slice_height / publish_rate 必须为正数，节点退出");
      throw std::invalid_argument("invalid workspace parameters");
    }
    if (samples_per_arm_ <= 0) {
      RCLCPP_ERROR(get_logger(), "samples_per_arm 必须为正整数，节点退出");
      throw std::invalid_argument("invalid samples_per_arm");
    }

    arms_ = make_default_arms();
    for (int side = 0; side < 2; ++side) {
      demo_q_[side].fill(0.0);
      for (int i = 0; i < kDof; ++i) {
        demo_q_[side][i] = 0.5 * (arms_[side].joints[i].lower + arms_[side].joints[i].upper);
      }
    }

    marker_pub_ = create_publisher<MarkerArray>("/reachable_workspace", 1);
    volume_pub_ = create_publisher<Float64MultiArray>("/reachable_volume", 10);
    bounds_pub_ = create_publisher<Float64MultiArray>("/reachable_bounds", 10);
    stats_pub_ = create_publisher<String>("/reachability_stats", 10);
    ee_left_pub_ = create_publisher<PointStamped>("/demo_end_effector_left", 10);
    ee_right_pub_ = create_publisher<PointStamped>("/demo_end_effector_right", 10);

    demo_sub_ = create_subscription<Float64MultiArray>(
      "/demo_joint_angles", 10,
      std::bind(&ReachabilityNode::on_demo_joint_angles, this, std::placeholders::_1));

    compute_workspace();

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ReachabilityNode::publish_all, this));

    log_summary();
  }

private:
  Eigen::Vector3d shoulder_point(int side) const
  {
    return Eigen::Vector3d(0.0, arms_[side].side * arms_[side].shoulder_y, arms_[side].shoulder_z);
  }

  void compute_workspace()
  {
    const auto start = std::chrono::steady_clock::now();
    const unsigned seed = static_cast<unsigned>(random_seed_);

    for (int side = 0; side < 2; ++side) {
      std::mt19937 rng(seed + 7919u * static_cast<unsigned>(side));
      grids_[side] = std::make_unique<VoxelGrid>(
        voxel_resolution_, slice_height_, shoulder_point(side));

      for (int n = 0; n < samples_per_arm_; ++n) {
        const JointAngles q = sample_joint_angles(arms_[side], rng);
        grids_[side]->insert(end_effector_position(arms_[side], q));
      }
      grids_[side]->analyze();
    }

    // 双臂体素并集体积（重叠区域只计一次）
    size_t overlap = 0;
    for (const int64_t key : grids_[0]->keys()) {
      if (grids_[1]->keys().count(key) != 0u) {
        ++overlap;
      }
    }
    overlap_voxels_ = overlap;
    const double cell = voxel_resolution_ * voxel_resolution_ * voxel_resolution_;
    union_volume_ = static_cast<double>(
      grids_[0]->stats().occupied + grids_[1]->stats().occupied - overlap) * cell;

    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
    RCLCPP_INFO(
      get_logger(), "双臂采样完成：每臂 %d 点，耗时 %.2f s（体素 %.3f m）",
      samples_per_arm_, elapsed, voxel_resolution_);
  }

  void log_summary()
  {
    for (int side = 0; side < 2; ++side) {
      const auto & stats = grids_[side]->stats();
      RCLCPP_INFO(
        get_logger(),
        "%s 可达域：体素 %zu（表面 %zu），体积 %.3f m³，包围盒 x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]，最大半径 %.3f m",
        arms_[side].name.c_str(), stats.occupied, stats.surface, stats.volume,
        stats.min_bound.x(), stats.max_bound.x(),
        stats.min_bound.y(), stats.max_bound.y(),
        stats.min_bound.z(), stats.max_bound.z(), stats.max_radius);
    }
    RCLCPP_INFO(get_logger(), "双臂并集体积 %.3f m³（重叠体素 %zu）", union_volume_, overlap_voxels_);
  }

  void on_demo_joint_angles(const Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < static_cast<size_t>(2 * kDof)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "/demo_joint_angles 需要 %d 个元素，收到 %zu 个", 2 * kDof, msg->data.size());
      return;
    }

    std::array<JointAngles, 2> angles;
    for (int side = 0; side < 2; ++side) {
      for (int i = 0; i < kDof; ++i) {
        angles[side][i] = msg->data[static_cast<size_t>(side * kDof + i)];
      }
    }

    for (int side = 0; side < 2; ++side) {
      if (!within_limits(arms_[side], angles[side])) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "/demo_joint_angles 超出 %s 关节限位，已忽略该帧", arms_[side].name.c_str());
        return;
      }
    }

    Eigen::Vector3d ends[2];
    for (int side = 0; side < 2; ++side) {
      ends[side] = end_effector_position(arms_[side], angles[side]);
    }

    {
      std::lock_guard<std::mutex> lock(demo_mutex_);
      demo_q_ = angles;
      for (int side = 0; side < 2; ++side) {
        auto & trace = traces_[side];
        trace.push_back(ends[side]);
        while (trace.size() > kTraceLength) {
          trace.pop_front();
        }
      }
    }

    publish_end_effector(ee_left_pub_, ends[0]);
    publish_end_effector(ee_right_pub_, ends[1]);
  }

  void publish_end_effector(
    const rclcpp::Publisher<PointStamped>::SharedPtr & publisher, const Eigen::Vector3d & point)
  {
    PointStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.point.x = point.x();
    msg.point.y = point.y();
    msg.point.z = point.z();
    publisher->publish(msg);
  }

  void publish_all()
  {
    marker_pub_->publish(build_markers());
    volume_pub_->publish(build_volume_msg());
    bounds_pub_->publish(build_bounds_msg());
    stats_pub_->publish(build_stats_msg());
  }

  Marker make_marker(const std::string & ns, int id, int32_t type)
  {
    Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  static std_msgs::msg::ColorRGBA make_color(float r, float g, float b, float a)
  {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

  MarkerArray build_markers()
  {
    MarkerArray array;

    if (!reset_published_) {
      Marker reset;
      reset.action = Marker::DELETEALL;
      array.markers.push_back(reset);
      reset_published_ = true;
    }

    for (int side = 0; side < 2; ++side) {
      array.markers.push_back(build_voxel_marker(side, 10 + side * 100));
      array.markers.push_back(build_slice_marker(side, 11 + side * 100));
      array.markers.push_back(build_bounds_marker(side, 12 + side * 100));
      array.markers.push_back(build_summary_text(side, 13 + side * 100));
    }

    std::array<JointAngles, 2> demo_q;
    std::array<std::vector<Eigen::Vector3d>, 2> traces;
    {
      std::lock_guard<std::mutex> lock(demo_mutex_);
      demo_q = demo_q_;
      for (int side = 0; side < 2; ++side) {
        traces[side].assign(traces_[side].begin(), traces_[side].end());
      }
    }

    std::array<ChainPoints, 2> chains;
    for (int side = 0; side < 2; ++side) {
      chains[side] = forward_kinematics(arms_[side], demo_q[side]);
    }

    array.markers.push_back(build_skeleton_marker(chains));
    array.markers.push_back(build_joint_marker(chains));

    for (int side = 0; side < 2; ++side) {
      array.markers.push_back(build_trace_marker(side, traces[side]));
    }
    array.markers.push_back(build_end_effector_marker(chains));
    array.markers.push_back(build_body_skeleton_marker());
    array.markers.push_back(build_body_head_marker());

    return array;
  }

  /// 静态参考骨架：躯干、骨盆与下肢，仅用于空间参照，不参与可达域计算
  Marker build_body_skeleton_marker()
  {
    Marker marker = make_marker("body_skeleton", 40, Marker::LINE_LIST);
    marker.scale.x = 0.015;
    marker.color = make_color(0.60f, 0.65f, 0.70f, 0.90f);

    const Eigen::Vector3d hips(0.0, 0.0, 0.92);
    const Eigen::Vector3d chest(0.0, 0.0, 1.30);
    const Eigen::Vector3d neck(0.0, 0.0, 1.42);
    add_segment(marker, hips, chest);
    add_segment(marker, chest, neck);

    for (int side = 0; side < 2; ++side) {
      const double sy = side == 0 ? 1.0 : -1.0;
      const Eigen::Vector3d shoulder(0.0, sy * 0.20, 1.42);
      const Eigen::Vector3d hip(sy * 0.11, 0.0, 0.92);
      add_segment(marker, chest, shoulder);
      add_segment(marker, shoulder, hip);

      const Eigen::Vector3d knee(sy * 0.13, 0.0, 0.52);
      const Eigen::Vector3d ankle(sy * 0.13, 0.0, 0.12);
      const Eigen::Vector3d toe(sy * 0.13, 0.14, 0.05);
      add_segment(marker, hip, knee);
      add_segment(marker, knee, ankle);
      add_segment(marker, ankle, toe);
    }
    add_segment(marker, Eigen::Vector3d(0.0, 0.11, 0.92), Eigen::Vector3d(0.0, -0.11, 0.92));
    return marker;
  }

  Marker build_body_head_marker()
  {
    Marker marker = make_marker("body_head", 41, Marker::SPHERE);
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 1.56;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.22;
    marker.color = make_color(0.72f, 0.77f, 0.82f, 0.60f);
    return marker;
  }

  static void add_segment(
    Marker & marker, const Eigen::Vector3d & a, const Eigen::Vector3d & b)
  {
    for (const Eigen::Vector3d * v : {&a, &b}) {
      geometry_msgs::msg::Point point;
      point.x = v->x();
      point.y = v->y();
      point.z = v->z();
      marker.points.push_back(point);
    }
  }

  Marker build_voxel_marker(int side, int id)
  {
    const auto & stats = grids_[side]->stats();
    const auto & surface = grids_[side]->surface_voxels();

    Marker marker = make_marker(
      side == 0 ? "voxels_left" : "voxels_right", id, Marker::CUBE_LIST);
    marker.scale.x = voxel_resolution_ * 0.9;
    marker.scale.y = voxel_resolution_ * 0.9;
    marker.scale.z = voxel_resolution_ * 0.9;

    size_t stride = 1;
    if (max_display_voxels_ > 0 && surface.size() > static_cast<size_t>(max_display_voxels_)) {
      stride = (surface.size() + static_cast<size_t>(max_display_voxels_) - 1) /
               static_cast<size_t>(max_display_voxels_);
    }

    const double z_span = std::max(1e-3, stats.max_bound.z() - stats.min_bound.z());
    for (size_t i = 0; i < surface.size(); i += stride) {
      const Eigen::Vector3d center = grids_[side]->voxel_center(
        surface[i].x(), surface[i].y(), surface[i].z());
      geometry_msgs::msg::Point point;
      point.x = center.x();
      point.y = center.y();
      point.z = center.z();
      marker.points.push_back(point);

      const double t = std::max(0.0, std::min(1.0, (center.z() - stats.min_bound.z()) / z_span));
      marker.colors.push_back(side == 0 ? left_color(t) : right_color(t));
    }
    return marker;
  }

  static std_msgs::msg::ColorRGBA left_color(double t)
  {
    return make_color(
      0.10f,
      static_cast<float>(0.55 + 0.40 * t),
      static_cast<float>(0.95 - 0.35 * t),
      0.85f);
  }

  static std_msgs::msg::ColorRGBA right_color(double t)
  {
    return make_color(
      static_cast<float>(0.95),
      static_cast<float>(0.60 - 0.38 * t),
      static_cast<float>(0.15 + 0.15 * t),
      0.85f);
  }

  Marker build_slice_marker(int side, int id)
  {
    const auto & stats = grids_[side]->stats();
    Marker marker = make_marker(
      side == 0 ? "slices_left" : "slices_right", id, Marker::LINE_LIST);
    marker.scale.x = 0.005;
    marker.color = side == 0 ?
      make_color(0.20f, 0.85f, 1.00f, 0.55f) : make_color(1.00f, 0.72f, 0.25f, 0.55f);

    for (const auto & slice : stats.slices) {
      if (slice.count < 5u) {
        continue;
      }
      const double z = 0.5 * (slice.z_min + slice.z_max);
      const double x0 = slice.xy_min.x();
      const double y0 = slice.xy_min.y();
      const double x1 = slice.xy_max.x();
      const double y1 = slice.xy_max.y();

      const std::array<std::array<double, 2>, 5> corners{{
        {{x0, y0}}, {{x1, y0}}, {{x1, y1}}, {{x0, y1}}, {{x0, y0}}}};
      for (const auto & corner : corners) {
        geometry_msgs::msg::Point point;
        point.x = corner[0];
        point.y = corner[1];
        point.z = z;
        marker.points.push_back(point);
      }
    }
    return marker;
  }

  Marker build_bounds_marker(int side, int id)
  {
    const auto & stats = grids_[side]->stats();
    Marker marker = make_marker(
      side == 0 ? "workspace_bounds_left" : "workspace_bounds_right", id, Marker::LINE_LIST);
    marker.scale.x = 0.01;
    marker.color = side == 0 ?
      make_color(0.15f, 0.75f, 1.00f, 0.45f) : make_color(1.00f, 0.55f, 0.20f, 0.45f);

    const Eigen::Vector3d lo = stats.min_bound;
    const Eigen::Vector3d hi = stats.max_bound;
    const std::array<Eigen::Vector3d, 8> corners{{
      {lo.x(), lo.y(), lo.z()}, {hi.x(), lo.y(), lo.z()},
      {hi.x(), hi.y(), lo.z()}, {lo.x(), hi.y(), lo.z()},
      {lo.x(), lo.y(), hi.z()}, {hi.x(), lo.y(), hi.z()},
      {hi.x(), hi.y(), hi.z()}, {lo.x(), hi.y(), hi.z()}}};

    const std::array<std::array<int, 2>, 12> edges{{
      {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
      {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
      {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}}}};

    for (const auto & edge : edges) {
      for (int k = 0; k < 2; ++k) {
        const Eigen::Vector3d & corner = corners[static_cast<size_t>(edge[k])];
        geometry_msgs::msg::Point point;
        point.x = corner.x();
        point.y = corner.y();
        point.z = corner.z();
        marker.points.push_back(point);
      }
    }
    return marker;
  }

  Marker build_summary_text(int side, int id)
  {
    const auto & stats = grids_[side]->stats();
    Marker marker = make_marker(
      side == 0 ? "summary_left" : "summary_right", id, Marker::TEXT_VIEW_FACING);
    marker.pose.position.x = 0.0;
    marker.pose.position.y = side == 0 ? 0.95 : -0.95;
    marker.pose.position.z = stats.max_bound.z() + 0.10;
    marker.scale.z = 0.06;
    marker.color = make_color(1.0f, 1.0f, 1.0f, 1.0f);

    std::ostringstream text;
    text << (side == 0 ? "左臂" : "右臂") << "末端可达域\n"
         << "体积 " << std::fixed << std::setprecision(3) << stats.volume << " m³\n"
         << "最大伸展 " << std::setprecision(3) << stats.max_radius << " m";
    marker.text = text.str();
    return marker;
  }

  Marker build_skeleton_marker(const std::array<ChainPoints, 2> & chains)
  {
    Marker marker = make_marker("demo_skeleton", 20, Marker::LINE_LIST);
    marker.scale.x = 0.02;
    marker.color = make_color(0.95f, 0.95f, 0.95f, 0.95f);

    for (int side = 0; side < 2; ++side) {
      const ChainPoints & p = chains[side];
      // 肩点 -> 肘、肘 -> 腕、腕 -> 末端参考点
      const std::array<std::array<int, 2>, 3> segments{{
        {{0, 4}}, {{4, 5}}, {{5, kDof + 1}}}};
      for (const auto & segment : segments) {
        for (int k = 0; k < 2; ++k) {
          geometry_msgs::msg::Point point;
          const Eigen::Vector3d & v = p[static_cast<size_t>(segment[k])];
          point.x = v.x();
          point.y = v.y();
          point.z = v.z();
          marker.points.push_back(point);
        }
      }
    }
    return marker;
  }

  Marker build_joint_marker(const std::array<ChainPoints, 2> & chains)
  {
    Marker marker = make_marker("demo_joints", 21, Marker::SPHERE_LIST);
    marker.scale.x = marker.scale.y = marker.scale.z = 0.045;
    marker.color = make_color(0.95f, 0.90f, 0.45f, 1.0f);

    for (int side = 0; side < 2; ++side) {
      const ChainPoints & p = chains[side];
      const std::array<int, 4> joints{{0, 4, 5, kDof + 1}};
      for (const int index : joints) {
        geometry_msgs::msg::Point point;
        const Eigen::Vector3d & v = p[static_cast<size_t>(index)];
        point.x = v.x();
        point.y = v.y();
        point.z = v.z();
        marker.points.push_back(point);
      }
    }
    return marker;
  }

  Marker build_trace_marker(int side, const std::vector<Eigen::Vector3d> & trace)
  {
    Marker marker = make_marker(
      side == 0 ? "demo_trace_left" : "demo_trace_right", 30 + side, Marker::LINE_STRIP);
    marker.scale.x = 0.008;
    marker.color = side == 0 ?
      make_color(0.35f, 1.00f, 0.85f, 1.0f) : make_color(1.00f, 0.55f, 0.35f, 1.0f);

    for (const Eigen::Vector3d & point3d : trace) {
      geometry_msgs::msg::Point point;
      point.x = point3d.x();
      point.y = point3d.y();
      point.z = point3d.z();
      marker.points.push_back(point);
    }
    return marker;
  }

  Marker build_end_effector_marker(const std::array<ChainPoints, 2> & chains)
  {
    Marker marker = make_marker("demo_end_effector", 32, Marker::SPHERE_LIST);
    marker.scale.x = marker.scale.y = marker.scale.z = 0.05;

    for (int side = 0; side < 2; ++side) {
      const Eigen::Vector3d & v = chains[side].back();
      geometry_msgs::msg::Point point;
      point.x = v.x();
      point.y = v.y();
      point.z = v.z();
      marker.points.push_back(point);
      marker.colors.push_back(
        side == 0 ? make_color(0.35f, 1.00f, 0.85f, 1.0f) : make_color(1.00f, 0.55f, 0.35f, 1.0f));
    }
    return marker;
  }

  Float64MultiArray build_volume_msg() const
  {
    Float64MultiArray msg;
    msg.data.resize(3);
    msg.data[0] = grids_[0]->stats().volume;
    msg.data[1] = grids_[1]->stats().volume;
    msg.data[2] = union_volume_;
    return msg;
  }

  Float64MultiArray build_bounds_msg() const
  {
    Float64MultiArray msg;
    msg.data.resize(12);
    for (int side = 0; side < 2; ++side) {
      const auto & bound = grids_[side]->stats();
      const size_t offset = static_cast<size_t>(side) * 6;
      msg.data[offset + 0] = bound.min_bound.x();
      msg.data[offset + 1] = bound.max_bound.x();
      msg.data[offset + 2] = bound.min_bound.y();
      msg.data[offset + 3] = bound.max_bound.y();
      msg.data[offset + 4] = bound.min_bound.z();
      msg.data[offset + 5] = bound.max_bound.z();
    }
    return msg;
  }

  String build_stats_msg() const
  {
    String msg;
    std::ostringstream text;
    text << std::fixed << std::setprecision(3);
    text << "双臂末端可达域 | 体素 " << voxel_resolution_ << " m | 切片 " << slice_height_ << " m\n";

    for (int side = 0; side < 2; ++side) {
      const auto & stats = grids_[side]->stats();
      text << (side == 0 ? "左臂" : "右臂")
           << " | 采样 " << stats.samples
           << " | 体素 " << stats.occupied
           << " | 表面 " << stats.surface
           << " | 体积 " << stats.volume << " m³"
           << " | 最大半径 " << stats.max_radius << " m"
           << " | 包围盒 x[" << stats.min_bound.x() << ", " << stats.max_bound.x()
           << "] y[" << stats.min_bound.y() << ", " << stats.max_bound.y()
           << "] z[" << stats.min_bound.z() << ", " << stats.max_bound.z() << "]\n";
    }
    text << "并集体积 " << union_volume_ << " m³ | 重叠体素 " << overlap_voxels_;
    msg.data = text.str();
    return msg;
  }

  static constexpr size_t kTraceLength = 300;

  std::array<ArmModel, 2> arms_;
  std::array<std::unique_ptr<VoxelGrid>, 2> grids_;

  int samples_per_arm_{200000};
  double voxel_resolution_{0.03};
  double slice_height_{0.05};
  int random_seed_{2026};
  int max_display_voxels_{2500};
  double publish_rate_{2.0};

  double union_volume_{0.0};
  size_t overlap_voxels_{0};

  std::mutex demo_mutex_;
  std::array<JointAngles, 2> demo_q_{};
  std::array<std::deque<Eigen::Vector3d>, 2> traces_;
  bool reset_published_{false};

  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr volume_pub_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr bounds_pub_;
  rclcpp::Publisher<String>::SharedPtr stats_pub_;
  rclcpp::Publisher<PointStamped>::SharedPtr ee_left_pub_;
  rclcpp::Publisher<PointStamped>::SharedPtr ee_right_pub_;
  rclcpp::Subscription<Float64MultiArray>::SharedPtr demo_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace humanoid_reachability

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_reachability::ReachabilityNode>());
  rclcpp::shutdown();
  return 0;
}
