#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include "water_rov_path_planning/heatmap.hpp"
#include "water_rov_path_planning/ocean_current.hpp"
#include "water_rov_path_planning/path_planner.hpp"

#include <memory>
#include <mutex>
#include <exception>

using namespace water_rov_path_planning;

class PlanningNode : public rclcpp::Node
{
public:
  PlanningNode()
  : Node("rov_planning_node")
  {
    // 参数（全部只在 params.yaml 定义一处，launch 不做硬编码覆盖）
    declare_parameter("grid_size", 100);
    declare_parameter("resolution", 0.5);
    declare_parameter("obstacle_radius", 2.0);
    declare_parameter("risk_decay", 0.15);
    declare_parameter("risk_weight", 5.0);
    declare_parameter("current_weight", 3.0);
    declare_parameter("current_compensation", 0.5);
    declare_parameter("max_risk_threshold", 0.7);
    declare_parameter("current_pattern", "vortex");
    declare_parameter("current_magnitude", 0.4);
    declare_parameter("current_direction", 0.0);
    declare_parameter("vortex_x", 25.0);
    declare_parameter("vortex_y", 25.0);
    declare_parameter("vortex_radius", 15.0);
    declare_parameter("tidal_period", 60.0);

    // 统一参数校验：非法值直接报错退出，避免 worldToGrid 除零、图像归一化除零等未定义行为
    const int grid_size = get_parameter("grid_size").as_int();
    const double resolution = get_parameter("resolution").as_double();
    const double obstacle_radius = get_parameter("obstacle_radius").as_double();
    const double risk_decay = get_parameter("risk_decay").as_double();
    const double current_magnitude = get_parameter("current_magnitude").as_double();
    const double vortex_radius = get_parameter("vortex_radius").as_double();
    const double tidal_period = get_parameter("tidal_period").as_double();
    const double max_risk_threshold = get_parameter("max_risk_threshold").as_double();

    if (grid_size <= 1 || resolution <= 0.0 || obstacle_radius < 0.0 ||
      risk_decay < 0.0 || current_magnitude < 0.0 || vortex_radius <= 0.0 ||
      tidal_period <= 0.0 || max_risk_threshold <= 0.0 || max_risk_threshold >= 1.0)
    {
      RCLCPP_ERROR(
        get_logger(),
        "参数校验失败：需满足 grid_size>1、resolution>0、obstacle_radius>=0、risk_decay>=0、"
        "current_magnitude>=0、vortex_radius>0、tidal_period>0、0<max_risk_threshold<1，节点退出");
      params_valid_ = false;
      rclcpp::shutdown();
      return;
    }

    // 初始化环境
    Heatmap::Params hp;
    hp.grid_size = grid_size;
    hp.resolution = resolution;
    hp.obstacle_radius = obstacle_radius;
    hp.risk_decay = risk_decay;
    heatmap_ = std::make_shared<Heatmap>(hp);

    OceanCurrent::Params ocp;
    std::string pattern = get_parameter("current_pattern").as_string();
    if (pattern == "vortex") ocp.pattern = OceanCurrent::Pattern::VORTEX;
    else if (pattern == "tidal") ocp.pattern = OceanCurrent::Pattern::TIDAL;
    else if (pattern == "shear") ocp.pattern = OceanCurrent::Pattern::SHEAR;
    else ocp.pattern = OceanCurrent::Pattern::UNIFORM;
    ocp.magnitude = current_magnitude;
    ocp.direction = get_parameter("current_direction").as_double();
    ocp.vortex_x = get_parameter("vortex_x").as_double();
    ocp.vortex_y = get_parameter("vortex_y").as_double();
    ocp.vortex_radius = vortex_radius;
    ocp.tidal_period = tidal_period;
    ocp.world_size = hp.grid_size * hp.resolution;
    current_ = std::make_shared<OceanCurrent>(ocp);

    PathPlanner::Params pp;
    pp.risk_weight = get_parameter("risk_weight").as_double();
    pp.current_weight = get_parameter("current_weight").as_double();
    pp.current_compensation = get_parameter("current_compensation").as_double();
    pp.max_risk_threshold = max_risk_threshold;
    planner_ = std::make_shared<PathPlanner>(pp);
    planner_->setEnvironment(heatmap_, current_);

    // 发布者（相对名，默认解析到根命名空间；加 __ns 后可多实例并行）
    path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    heatmap_pub_ = image_transport::create_publisher(this, "heatmap");
    current_pub_ = image_transport::create_publisher(this, "ocean_current");
    cost_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("path_cost", 10);

    // 订阅者（障碍物队列加深到 50，容纳演示节点一次性发布的一批障碍物）
    obstacle_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "obstacle", rclcpp::QoS(50),
      std::bind(&PlanningNode::obstacleCallback, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "goal", 10,
      std::bind(&PlanningNode::goalCallback, this, std::placeholders::_1));
    start_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "start", 10,
      std::bind(&PlanningNode::startCallback, this, std::placeholders::_1));

    // 定时发布热力图和洋流场
    timer_ = create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&PlanningNode::publishEnvironment, this));

    RCLCPP_INFO(get_logger(),
      "rov_planning_node 启动，网格 %dx%d，分辨率 %.2f m，洋流模式 %s",
      hp.grid_size, hp.grid_size, hp.resolution, pattern.c_str());
  }

  /// 参数校验是否通过；main 据此返回非零退出码，便于 launch 层识别失败
  bool parameters_valid() const { return params_valid_; }

private:
  void obstacleCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    heatmap_->addObstacle(msg->point.x, msg->point.y);
    RCLCPP_INFO(get_logger(), "添加障碍物 (%.1f, %.1f)", msg->point.x, msg->point.y);
  }

  void startCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    start_ = {msg->point.x, msg->point.y};
    RCLCPP_INFO(get_logger(), "设置起点 (%.1f, %.1f)", start_.x, start_.y);
  }

  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_ = {msg->point.x, msg->point.y};
    RCLCPP_INFO(get_logger(), "设置终点 (%.1f, %.1f)，开始规划", goal_.x, goal_.y);
    replan();
  }

  void replan()
  {
    if (start_.x < 0 || goal_.x < 0) return;

    // 把当前时间传给规划器，潮汐模式才能按当前相位计算洋流代价
    planner_->setTime(now().seconds());

    std::vector<PathPlanner::Point> path;
    bool ok = planner_->plan(start_, goal_, path);

    if (ok && !path.empty()) {
      nav_msgs::msg::Path path_msg;
      path_msg.header.stamp = now();
      path_msg.header.frame_id = "world";
      for (auto & p : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.position.z = 0.0;
        path_msg.poses.push_back(pose);
      }
      path_pub_->publish(path_msg);

      double length = planner_->pathLength(path);
      auto cost_msg = std_msgs::msg::Float32MultiArray();
      cost_msg.data = {
        static_cast<float>(planner_->getLastCost()),
        static_cast<float>(length),
        static_cast<float>(path.size())
      };
      cost_pub_->publish(cost_msg);

      RCLCPP_INFO(get_logger(),
        "路径规划成功，点数 %zu，路径长度 %.2f m，加权累计代价 %.2f（无量纲）",
        path.size(), length, planner_->getLastCost());
    } else {
      RCLCPP_WARN(get_logger(), "路径规划失败，可能终点在高风险区或无路可达");
    }
  }

  void publishEnvironment()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_->setTime(now().seconds());

    // 发布热力图
    cv::Mat heat_img = heatmap_->toColorImage();
    auto heat_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", heat_img).toImageMsg();
    heat_msg->header.stamp = now();
    heatmap_pub_.publish(*heat_msg);

    // 发布洋流场
    cv::Mat cur_img = current_->toVectorImage();
    auto cur_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cur_img).toImageMsg();
    cur_msg->header.stamp = now();
    current_pub_.publish(*cur_msg);
  }

  std::shared_ptr<Heatmap> heatmap_;
  std::shared_ptr<OceanCurrent> current_;
  std::shared_ptr<PathPlanner> planner_;
  PathPlanner::Point start_{-1, -1};
  PathPlanner::Point goal_{-1, -1};
  std::mutex mutex_;
  bool params_valid_ = true;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  image_transport::Publisher heatmap_pub_;
  image_transport::Publisher current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cost_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return node->parameters_valid() ? 0 : 1;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("planning_node"), "节点异常退出: %s", e.what());
    return 1;
  }
}
