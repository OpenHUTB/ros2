/*
 * 八叉树 A* 全局寻路 —— ROS 节点
 *
 * 作用：
 *   订阅建图节点发布的八叉树地图；每收到一个目标点就做一次 A* 全局寻路，
 *   发布一条从当前位置到目标点的无碰撞路径。
 *
 *   核心算法在 include/octree_uav_3d_pathfinding/astar.h 里，与 ROS 完全解耦，
 *   可以离线单元测试（tests/test_astar.cpp）。本文件只是它的一层 ROS 外壳：
 *   负责收发话题、查 TF、把结果转成 ROS 消息。
 *
 * 订阅：
 *   /octomap_full   octomap_msgs/Octomap      八叉树占用地图（latched，一连上就有）
 *   /uav/goal       geometry_msgs/Point       目标点（世界系 ENU）
 * 发布：
 *   /uav/path               nav_msgs/Path                规划出的路径
 *   /uav/explored_markers   visualization_msgs/MarkerArray 已展开节点（按尺寸着色）
 *
 * 运行：
 *   roslaunch octree_uav_3d_pathfinding main.launch
 *
 * 说明：
 *   · 本节点**只做规划，不控制飞行** —— 把路径喂给控制器是后续阶段的事
 *   · 航点是**八叉树叶节点的中心**，所以第一个航点不一定是飞行器当前位置，
 *     而是它所在的那个叶节点的中心（粗节点可能差出十几米）
 *   · 未知区域一律当作障碍：地图没扫到的地方不会规划出路径
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "octree_uav_3d_pathfinding/astar.h"

// 核心算法里的类型都在这个命名空间下。这里只引入用到的三个，
// 而不是 using namespace —— 避免把无关的名字也带进来。
using octree_uav_3d_pathfinding::ExpandedCell;
using octree_uav_3d_pathfinding::OctreeAStar;
using octree_uav_3d_pathfinding::PlanStats;

class AstarPlanner
{
public:
  AstarPlanner()
    : nh_(),
      pnh_("~"),
      tf_buffer_(ros::Duration(30.0)),
      tf_listener_(tf_buffer_),
      has_map_(false),
      free_leaves_(0),
      plan_count_(0)
  {
    // ---------------- 参数 ----------------
    map_topic_ = pnh_.param<std::string>("map_topic", "/octomap_full");
    goal_topic_ = pnh_.param<std::string>("goal_topic", "/uav/goal");
    path_topic_ = pnh_.param<std::string>("path_topic", "/uav/path");
    explored_topic_ =
        pnh_.param<std::string>("explored_topic", "/uav/explored_markers");
    world_frame_ = pnh_.param<std::string>("world_frame", "world");
    body_frame_ = pnh_.param<std::string>("body_frame", "base_link");

    params_.k_max = pnh_.param("k_max", 4.0);
    const int max_iterations = pnh_.param("max_iterations", 200000);
    params_.max_iterations =
        static_cast<std::size_t>(max_iterations < 1 ? 1 : max_iterations);
    params_.heuristic_weight = pnh_.param("heuristic_weight", 1.0);
    params_.search_radius = pnh_.param("search_radius", 3.0);
    params_.plan_timeout = pnh_.param("plan_timeout", 10.0);

    max_expanded_markers_ = pnh_.param("max_expanded_markers", 20000);

    // ---------------- ROS 接口 ----------------
    map_sub_ = nh_.subscribe(map_topic_, 1, &AstarPlanner::mapCallback, this);
    goal_sub_ = nh_.subscribe(goal_topic_, 1, &AstarPlanner::goalCallback, this);

    // 路径用 latched 发布：规划不是周期性的，晚连上来的 RViz 也该立刻看到上一条
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
    // 「已展开节点」同样 latched：在 RViz 里勾选/取消显示项会让它重新订阅，
    // 而非 latched 的话题要等下一次规划才有消息 —— 那样一勾选就是一片空白。
    // 这一路的数组可能比较大（上万个方块），但都是本地话题，代价可以接受。
    explored_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        explored_topic_, 1, true);

    // 日志一律用 ASCII：roscpp 的 rosconsole 在部分环境下会把中文输出成 '?'，
    // 而同样内容用 rospy 打印就正常。为保证终端输出与文档截图干净，这里统一用英文。
    ROS_INFO("astar_planner started");
    ROS_INFO("  map topic     : %s", map_topic_.c_str());
    ROS_INFO("  goal topic    : %s", goal_topic_.c_str());
    ROS_INFO("  path topic    : %s", path_topic_.c_str());
    ROS_INFO("  frames        : %s <- %s", world_frame_.c_str(), body_frame_.c_str());
    ROS_INFO("  k_max         : %.1f", params_.k_max);
    ROS_INFO("  max iterations: %zu", params_.max_iterations);
    ROS_INFO("  heuristic w   : %.2f", params_.heuristic_weight);
  }

private:
  // ------------------------------------------------------------------
  // 地图回调：反序列化成 OcTree
  // ------------------------------------------------------------------
  void mapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
  {
    octomap::AbstractOcTree* abstract = octomap_msgs::msgToMap(*msg);
    if (abstract == NULL) {
      ROS_WARN_THROTTLE(5.0, "failed to convert octomap_msgs/Octomap to an octree");
      return;
    }

    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(abstract);
    if (tree == NULL) {
      ROS_WARN_THROTTLE(5.0, "received map is not a standard OcTree");
      delete abstract;
      return;
    }

    tree_.reset(tree);
    map_stamp_ = msg->header.stamp;

    // 诊断：整棵树里有多少自由叶节点。
    // 寻路失败时这个数字能立刻把问题一分为二：
    //   数字很小 → 建图根本没把自由空间标出来；
    //   数字很大 → 自由空间是有的，问题在 findLeaf / 邻接。
    free_leaves_ = countFreeLeaves();

    if (!has_map_) {
      has_map_ = true;
      ROS_INFO("first map received: %zu free leaves, resolution %.2f m",
               free_leaves_, tree_->getResolution());
    }
  }

  // ------------------------------------------------------------------
  // 目标回调：取起点 → A* → 发布路径
  // ------------------------------------------------------------------
  void goalCallback(const geometry_msgs::Point::ConstPtr& msg)
  {
    if (!has_map_ || tree_ == NULL) {
      ROS_WARN("goal received but no map yet -- is the mapping node running?");
      return;
    }

    // 起点：由 TF 查 world <- base_link，取当前位姿
    geometry_msgs::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(world_frame_, body_frame_,
                                          ros::Time(0), ros::Duration(0.5));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN("cannot look up TF %s -> %s: %s", world_frame_.c_str(),
               body_frame_.c_str(), ex.what());
      return;
    }
    const geometry_msgs::Vector3& t = tf_msg.transform.translation;
    const octomap::point3d start(t.x, t.y, t.z);
    const octomap::point3d goal(msg->x, msg->y, msg->z);

    // 规划
    OctreeAStar planner(*tree_, params_);
    std::vector<octomap::point3d> path;
    std::vector<ExpandedCell> expanded;
    PlanStats stats;
    const bool ok = planner.plan(start, goal, path, stats, &expanded);

    plan_count_ += 1;
    ROS_INFO("plan #%d: %s | expanded=%zu discovered=%zu waypoints=%zu "
             "length=%.2f m in %.1f ms",
             plan_count_, ok ? "OK" : "FAILED", stats.expanded,
             stats.discovered, stats.waypoints, stats.path_length,
             stats.elapsed_ms);

    if (!ok) {
      ROS_WARN("plan failed: %s", stats.reason.c_str());
      ROS_WARN("start=(%.2f, %.2f, %.2f) goal=(%.2f, %.2f, %.2f)",
               start.x(), start.y(), start.z(), goal.x(), goal.y(), goal.z());
      // 诊断：把起点/终点所在叶子的真实情况打出来 ——
      // "找不到可行节点"时，首先要知道那个位置到底是未知、还是被占用。
      std::string desc;
      planner.probeInfo(start, desc);
      ROS_WARN("start cell: %s", desc.c_str());
      planner.probeInfo(goal, desc);
      ROS_WARN("goal  cell: %s", desc.c_str());
      ROS_WARN("map has %zu free leaves in total", free_leaves_);
      // 自检：遍历自由叶节点，用它们的中心反过来查一遍。
      // 这一手在开发时抓到过一个很隐蔽的 bug（子节点编号位序写错，
      // 2000 个叶子里 1883 个查不回来），所以保留成为常驻诊断 ——
      // 出现"明明有地图却规划不了"时，先看这个数字。
      {
        std::string detail;
        const std::size_t bad = planner.selfCheckLookup(2000, detail);
        ROS_WARN("findLeaf self-check: %zu unreachable among 2000 free leaves%s%s",
                 bad, detail.empty() ? "" : " | ", detail.c_str());
      }
      // 发一条空路径，把 RViz 里上一次的线清掉
      publishPath(std::vector<octomap::point3d>());
      return;
    }

    ROS_INFO("  start cell center=(%.2f, %.2f, %.2f)",
             path.front().x(), path.front().y(), path.front().z());
    ROS_INFO("  goal  cell center=(%.2f, %.2f, %.2f)",
             path.back().x(), path.back().y(), path.back().z());

    publishPath(path);
    publishExplored(expanded);
  }

  // ------------------------------------------------------------------
  // 发布路径
  // ------------------------------------------------------------------
  void publishPath(const std::vector<octomap::point3d>& path)
  {
    nav_msgs::Path msg;
    msg.header.frame_id = world_frame_;
    msg.header.stamp = ros::Time::now();
    msg.poses.reserve(path.size());

    for (std::size_t i = 0; i < path.size(); ++i) {
      geometry_msgs::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = path[i].x();
      pose.pose.position.y = path[i].y();
      pose.pose.position.z = path[i].z();
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    path_pub_.publish(msg);

    if (path.empty()) {
      ROS_INFO("published an empty path (clears the previous line in RViz)");
    }
  }

  // ------------------------------------------------------------------
  // 发布「已展开节点」—— 按节点尺寸着色
  //
  // 这是理解 A* 在八叉树上行为最直观的一路输出：
  //   · 偏蓝 = 小节点（障碍物附近，需要细碎地走）
  //   · 偏红 = 大节点（空旷区域，一步跨过去）
  // 一张图就能看出跨分辨率邻接是否真的生效。
  // ------------------------------------------------------------------
  void publishExplored(const std::vector<ExpandedCell>& cells)
  {
    // 先**单独**发一条 DELETEALL 清掉上一次的。
    //
    // 为什么不清空和新方块放在同一个 MarkerArray 里：有些 RViz 版本会把整个
    // 数组当作一批处理 —— 那样 DELETEALL 会把同一批里的新方块也一起清掉，
    // 结果就是"发了很多方块却什么都看不到"。
    visualization_msgs::MarkerArray clear_array;
    visualization_msgs::Marker clear;
    clear.header.frame_id = world_frame_;
    clear.header.stamp = ros::Time::now();
    clear.action = visualization_msgs::Marker::DELETEALL;
    clear_array.markers.push_back(clear);
    explored_pub_.publish(clear_array);

    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = clear.header.stamp;
    marker.ns = "astar_expanded";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.0);  // 常驻，直到下一次规划覆盖

    int id = 0;
    for (std::size_t i = 0; i < cells.size(); ++i) {
      if (id >= max_expanded_markers_) {
        ROS_WARN_THROTTLE(10.0,
                          "expanded cells exceed max_expanded_markers = %d; "
                          "showing only the first %d",
                          max_expanded_markers_, max_expanded_markers_);
        break;
      }
      marker.id = id++;
      marker.pose.position.x = cells[i].center.x();
      marker.pose.position.y = cells[i].center.y();
      marker.pose.position.z = cells[i].center.z();
      // 略微缩小，避免相邻方块的边线互相穿插
      marker.scale.x = cells[i].size * 0.98;
      marker.scale.y = cells[i].size * 0.98;
      marker.scale.z = cells[i].size * 0.98;
      sizeColor(cells[i].size, marker.color);
      array.markers.push_back(marker);
    }

    explored_pub_.publish(array);
    // 打印一行，便于确认到底发出去了多少个节点 ——
    // 如果规划是平凡解（起终点落在同一个叶节点里），这里会是 0，
    // RViz 里也就什么都看不到，那是正常的。
    ROS_INFO("published %zu explored cells", cells.size());
  }

  // 按节点尺寸着色：以 2 为底做对数归一化，跨 128 倍封顶。
  // 分辨率处的叶节点 → 蓝；跨了 128 倍的粗节点 → 红。
  void sizeColor(double size, std_msgs::ColorRGBA& color) const
  {
    double ratio = 0.0;
    if (tree_ != NULL && size > tree_->getResolution()) {
      ratio = std::log(size / tree_->getResolution()) / std::log(128.0);
    }
    ratio = std::min(std::max(ratio, 0.0), 1.0);

    color.r = ratio;
    color.g = 0.2;
    color.b = 1.0 - ratio;
    // 0.6 而不是更低：看"展开节点按尺寸着色"那张图时其它显示项都关掉了，
    // 方块要能在深色背景上单独看清
    color.a = 0.6;
  }

  // ------------------------------------------------------------------
  // 诊断
  // ------------------------------------------------------------------
  std::size_t countFreeLeaves() const
  {
    if (tree_ == NULL) {
      return 0;
    }
    std::size_t n = 0;
    for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs(),
                                        end = tree_->end_leafs();
         it != end; ++it) {
      if (!tree_->isNodeOccupied(*it)) {
        n += 1;
      }
    }
    return n;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<octomap::OcTree> tree_;
  bool has_map_;
  std::size_t free_leaves_;
  ros::Time map_stamp_;

  ros::Subscriber map_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;
  ros::Publisher explored_pub_;

  std::string map_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string explored_topic_;
  std::string world_frame_;
  std::string body_frame_;

  OctreeAStar::Params params_;
  int max_expanded_markers_;
  int plan_count_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "astar_planner");
  AstarPlanner node;
  ros::spin();
  return 0;
}
