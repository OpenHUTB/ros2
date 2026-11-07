#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <chrono>
#include <cmath>
#include <exception>
#include <random>
#include <utility>
#include <vector>

/**
 * @brief 演示节点：模拟海底环境
 *
 * 发布若干障碍物、起点、终点，触发路径规划节点工作。
 * 模拟 50m x 50m 的海底区域，随机散布障碍物，
 * 设置起点和终点后由 planning_node 完成自适应路径规划。
 */
class DemoNode : public rclcpp::Node
{
public:
  DemoNode()
  : Node("rov_demo_node")
  {
    declare_parameter("world_size", 50.0);
    declare_parameter("num_obstacles", 12);
    declare_parameter("start_x", 5.0);
    declare_parameter("start_y", 25.0);
    declare_parameter("goal_x", 45.0);
    declare_parameter("goal_y", 25.0);
    declare_parameter("obstacle_seed", 42);
    // 与规划节点一致的网格参数，仅用于启动时交叉校验 world_size
    declare_parameter("grid_size", 100);
    declare_parameter("resolution", 0.5);

    world_size_ = get_parameter("world_size").as_double();
    num_obstacles_ = get_parameter("num_obstacles").as_int();
    start_x_ = get_parameter("start_x").as_double();
    start_y_ = get_parameter("start_y").as_double();
    goal_x_ = get_parameter("goal_x").as_double();
    goal_y_ = get_parameter("goal_y").as_double();
    obstacle_seed_ = get_parameter("obstacle_seed").as_int();
    const int grid_size = get_parameter("grid_size").as_int();
    const double resolution = get_parameter("resolution").as_double();

    // 参数校验：避免负数量、零尺寸或起终点越界。
    // 起终点必须严格小于 world_size：x == world_size 会映射到 grid_size 号格被判越界。
    // world_size 还必须等于规划侧的 grid_size*resolution，否则改一个不改另一个
    // 就会出现一半地图不可达
    if (world_size_ <= 4.0 || num_obstacles_ < 0 ||
      start_x_ < 0.0 || start_x_ >= world_size_ || start_y_ < 0.0 || start_y_ >= world_size_ ||
      goal_x_ < 0.0 || goal_x_ >= world_size_ || goal_y_ < 0.0 || goal_y_ >= world_size_ ||
      grid_size <= 1 || resolution <= 0.0 ||
      std::abs(world_size_ - grid_size * resolution) > 1e-6)
    {
      RCLCPP_ERROR(
        get_logger(),
        "参数校验失败：需满足 world_size>4、num_obstacles>=0、起点/终点落在 [0, world_size) 内、"
        "且 world_size 与规划节点的 grid_size*resolution（当前 %d x %.3f = %.3f m）一致，节点退出",
        grid_size, resolution, grid_size * resolution);
      params_valid_ = false;
      rclcpp::shutdown();
      return;
    }

    // 相对话题名，默认解析到根命名空间；障碍物队列加深到 50，容纳一次性发布的一批障碍物
    obstacle_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("obstacle", rclcpp::QoS(50));
    start_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("start", 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("goal", 10);

    // 用热力图作为“障碍物已被规划节点处理”的确认信号：
    // 障碍物发布后，等到时间戳晚于发布时刻的热力图到达，再发终点触发规划，
    // 避免 obstacle / goal 走不同话题时终点先到、按旧热力图规划
    heatmap_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "heatmap", rclcpp::QoS(1),
      std::bind(&DemoNode::heatmapCallback, this, std::placeholders::_1));

    // 轮询等待 planning_node 订阅就绪，代替固定时长的 sleep
    ready_timer_ = create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&DemoNode::startDemo, this));

    // 周期性重规划：独立定时器成员，构造时创建；演示开始前不发消息，
    // 避免在自身回调里替换正在执行的定时器对象
    replan_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      if (!demo_active_) return;
      publishPoint(goal_pub_, goal_x_, goal_y_);
      RCLCPP_INFO(get_logger(), "触发重规划...");
    });
  }

  /// 参数校验是否通过；main 据此返回非零退出码，便于 launch 层识别失败
  bool parameters_valid() const { return params_valid_; }

private:
  void publishPoint(
    const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr & publisher,
    double x, double y)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.point.x = x;
    msg.point.y = y;
    msg.point.z = 0.0;
    publisher->publish(msg);
  }

  void startDemo()
  {
    // 三个发布者都有订阅者后再发数据，避免发现阶段丢消息
    if (obstacle_pub_->get_subscription_count() == 0 ||
      start_pub_->get_subscription_count() == 0 ||
      goal_pub_->get_subscription_count() == 0)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "等待 rov_planning_node 订阅就绪...");
      return;
    }

    ready_timer_->cancel();

    std::mt19937 rng(obstacle_seed_);
    std::uniform_real_distribution<double> dist(2.0, world_size_ - 2.0);

    RCLCPP_INFO(get_logger(), "=== ROV 路径规划演示开始 ===");
    RCLCPP_INFO(get_logger(), "海域尺寸: %.0f x %.0f m", world_size_, world_size_);
    RCLCPP_INFO(get_logger(), "起点: (%.1f, %.1f)  终点: (%.1f, %.1f)",
      start_x_, start_y_, goal_x_, goal_y_);

    // 生成障碍物（避开起点和终点附近 5m）
    RCLCPP_INFO(get_logger(), "正在生成 %d 个障碍物...", num_obstacles_);
    std::vector<std::pair<double, double>> obstacles;
    for (int i = 0; i < num_obstacles_; ++i) {
      double ox, oy;
      int tries = 0;
      do {
        ox = dist(rng);
        oy = dist(rng);
        tries++;
      } while (tries < 50 && (
        (std::hypot(ox - start_x_, oy - start_y_) < 5.0) ||
        (std::hypot(ox - goal_x_, oy - goal_y_) < 5.0)));
      obstacles.emplace_back(ox, oy);
      RCLCPP_INFO(get_logger(), "  障碍物 %d: (%.1f, %.1f)", i + 1, ox, oy);
    }

    for (const auto & obstacle : obstacles) {
      publishPoint(obstacle_pub_, obstacle.first, obstacle.second);
    }
    publishPoint(start_pub_, start_x_, start_y_);

    // 终点不立即发布：等一张时间戳晚于此刻的热力图，确认障碍物已被规划节点处理
    confirm_after_ = now();
    waiting_confirm_ = true;
    RCLCPP_INFO(get_logger(), "已发布障碍物与起点，等待热力图刷新后发送终点...");
  }

  void heatmapCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!waiting_confirm_) return;
    // 留 0.1 s 余量，确保这张热力图是在障碍物回调处理完之后才发布的
    if (rclcpp::Time(msg->header.stamp) <
      confirm_after_ + rclcpp::Duration::from_seconds(0.1)) return;

    waiting_confirm_ = false;
    publishPoint(goal_pub_, goal_x_, goal_y_);
    demo_active_ = true;
    RCLCPP_INFO(get_logger(), "热力图已刷新，发布终点，开始规划");
  }

  double world_size_ = 50.0;
  int num_obstacles_ = 12;
  double start_x_ = 5.0;
  double start_y_ = 25.0;
  double goal_x_ = 45.0;
  double goal_y_ = 25.0;
  int obstacle_seed_ = 42;
  bool params_valid_ = true;
  bool demo_active_ = false;
  bool waiting_confirm_ = false;
  rclcpp::Time confirm_after_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr start_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr heatmap_sub_;
  rclcpp::TimerBase::SharedPtr ready_timer_;
  rclcpp::TimerBase::SharedPtr replan_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DemoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return node->parameters_valid() ? 0 : 1;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("demo_node"), "节点异常退出: %s", e.what());
    return 1;
  }
}
