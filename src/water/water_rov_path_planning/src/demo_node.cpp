#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>
#include <vector>
#include <random>

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

    world_size_ = get_parameter("world_size").as_double();
    num_obstacles_ = get_parameter("num_obstacles").as_int();

    obstacle_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rov/obstacle", 10);
    start_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rov/start", 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/rov/goal", 10);

    // 延迟启动，等 planning_node 就绪
    timer_ = create_wall_timer(std::chrono::milliseconds(2000),
      std::bind(&DemoNode::startDemo, this));
  }

private:
  void startDemo()
  {
    timer_->cancel();

    std::mt19937 rng(get_parameter("obstacle_seed").as_int());
    std::uniform_real_distribution<double> dist(2.0, world_size_ - 2.0);

    double start_x = get_parameter("start_x").as_double();
    double start_y = get_parameter("start_y").as_double();
    double goal_x = get_parameter("goal_x").as_double();
    double goal_y = get_parameter("goal_y").as_double();

    RCLCPP_INFO(get_logger(), "=== ROV 路径规划演示开始 ===");
    RCLCPP_INFO(get_logger(), "海域尺寸: %.0f x %.0f m", world_size_, world_size_);
    RCLCPP_INFO(get_logger(), "起点: (%.1f, %.1f)  终点: (%.1f, %.1f)",
      start_x, start_y, goal_x, goal_y);

    // 发布障碍物（避开起点和终点附近 5m）
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
        (std::hypot(ox - start_x, oy - start_y) < 5.0) ||
        (std::hypot(ox - goal_x, oy - goal_y) < 5.0)));
      obstacles.emplace_back(ox, oy);
      RCLCPP_INFO(get_logger(), "  障碍物 %d: (%.1f, %.1f)", i + 1, ox, oy);
    }

    // 重发 3 次，避免订阅端尚未完成发现时丢消息
    for (int round = 0; round < 3; ++round) {
      for (const auto & obstacle : obstacles) {
        auto msg = geometry_msgs::msg::PointStamped();
        msg.header.stamp = now();
        msg.header.frame_id = "world";
        msg.point.x = obstacle.first;
        msg.point.y = obstacle.second;
        msg.point.z = 0.0;
        obstacle_pub_->publish(msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // 等待 1 秒让热力图更新
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 发布起点
    auto start_msg = geometry_msgs::msg::PointStamped();
    start_msg.header.stamp = now();
    start_msg.header.frame_id = "world";
    start_msg.point.x = start_x;
    start_msg.point.y = start_y;
    start_pub_->publish(start_msg);

    // 发布终点（触发规划）
    auto goal_msg = geometry_msgs::msg::PointStamped();
    goal_msg.header.stamp = now();
    goal_msg.header.frame_id = "world";
    goal_msg.point.x = goal_x;
    goal_msg.point.y = goal_y;
    goal_pub_->publish(goal_msg);

    RCLCPP_INFO(get_logger(), "已发布起点和终点，等待规划结果...");

    // 5 秒后重新发布终点触发重规划（演示周期性重规划）
    timer_ = create_wall_timer(std::chrono::seconds(5),
      [this, goal_x, goal_y]() {
        auto goal_msg = geometry_msgs::msg::PointStamped();
        goal_msg.header.stamp = now();
        goal_msg.header.frame_id = "world";
        goal_msg.point.x = goal_x;
        goal_msg.point.y = goal_y;
        goal_pub_->publish(goal_msg);
        RCLCPP_INFO(get_logger(), "触发重规划...");
      });
  }

  double world_size_;
  int num_obstacles_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr start_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
