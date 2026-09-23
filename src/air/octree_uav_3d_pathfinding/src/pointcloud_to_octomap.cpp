/*
 * 点云 → 八叉树占用地图
 *
 * 作用：
 *   把 3D 激光雷达发布的点云（sensor_msgs/PointCloud2）插入 octomap 的八叉树，
 *   生成三维占用地图，供后续在八叉树上做 A* 全局寻路使用。
 *
 * 每收到一帧点云的处理流程：
 *   1) 用 TF 把点云从传感器坐标系变换到世界坐标系（world）
 *   2) 以传感器位置为射线原点，调用 OcTree::insertPointCloud 做射线投射：
 *      射线终点体素记为「占用」，射线途经体素记为「空闲」，
 *      没有被任何射线穿过的区域保持「未知」
 *   3) 按高度给占用体素着色，转成 MarkerArray 供 RViz 直接显示
 *
 * 订阅：
 *   /cloud_in                  sensor_msgs/PointCloud2      3D 激光雷达点云（AirSim 桥接节点发布）
 * 发布：
 *   /octomap_full              octomap_msgs/Octomap          完整八叉树（latched）
 *   /occupied_cells_vis_array  visualization_msgs/MarkerArray 占用体素可视化
 *
 * 运行：
 *   roslaunch octree_uav_3d_pathfinding main.launch
 *
 * 说明：
 *   · 点云由 scripts/airsim_bridge.py 从 AirSim 取回后发布，坐标系是 lidar_link
 *     （ROS 机体系 FLU）；AirSim 的 NED/FRD 换算已经在桥接节点里做过
 *   · 射线原点取 TF 中传感器坐标系的原点，即雷达位置；相对 0.2 m 的分辨率，
 *     雷达与机体之间的安装偏移可以忽略
 */

#include <algorithm>
#include <cmath>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class PointCloudToOctomap
{
public:
  PointCloudToOctomap()
    : nh_(),
      pnh_("~"),
      tf_buffer_(ros::Duration(30.0)),
      tf_listener_(tf_buffer_),
      // OcTree 的构造函数必须给定分辨率，这里先用一个占位值，
      // 读到 ~resolution 参数后再调用 setResolution 生效（此时树还是空的）
      tree_(0.1),
      cloud_count_(0),
      point_count_(0),
      received_cloud_(false)
  {
    // ---------------- 参数 ----------------
    cloud_topic_ = pnh_.param<std::string>("cloud_topic", "/cloud_in");
    world_frame_ = pnh_.param<std::string>("world_frame", "world");
    sensor_frame_ = pnh_.param<std::string>("sensor_frame", "lidar_link");

    resolution_ = pnh_.param("resolution", 0.2);
    max_range_ = pnh_.param("max_range", 20.0);
    min_range_ = pnh_.param("min_range", 0.3);

    const double prob_hit = pnh_.param("prob_hit", 0.7);
    const double prob_miss = pnh_.param("prob_miss", 0.4);
    const double clamp_min = pnh_.param("clamp_min", 0.12);
    const double clamp_max = pnh_.param("clamp_max", 0.97);

    publish_rate_ = pnh_.param("publish_rate", 1.0);
    color_min_z_ = pnh_.param("color_min_z", 0.0);
    color_max_z_ = pnh_.param("color_max_z", 6.0);
    max_markers_ = pnh_.param("max_markers", 20000);
    point_stride_ = pnh_.param("point_stride", 1);
    if (point_stride_ < 1) {
      point_stride_ = 1;
    }
    status_period_ = pnh_.param("status_period", 5.0);
    cloud_timeout_ = pnh_.param("cloud_timeout", 10.0);

    // ---------------- 八叉树概率模型 ----------------
    tree_.setResolution(resolution_);
    tree_.setProbHit(prob_hit);      // 命中：log-odds 增加
    tree_.setProbMiss(prob_miss);    // 未命中（射线穿过）：log-odds 减少
    tree_.setClampingThresMin(clamp_min);
    tree_.setClampingThresMax(clamp_max);

    // ---------------- ROS 接口 ----------------
    // 八叉树地图变化很慢，用 latched 发布：新订阅者（例如后续的 A* 规划节点）
    // 一连上就能拿到当前完整地图，不必等下一次更新
    octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/octomap_full", 1, true);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/occupied_cells_vis_array", 1);

    // queue_size 取 1：建图一般跟不上传感器帧率时，宁可丢旧帧也不要堆积延迟
    cloud_sub_ = nh_.subscribe(cloud_topic_, 1,
                               &PointCloudToOctomap::cloudCallback, this);

    const double rate = (publish_rate_ > 0.0) ? publish_rate_ : 1.0;
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / rate),
                                     &PointCloudToOctomap::publishCallback, this);
    status_timer_ = nh_.createTimer(ros::Duration(1.0),
                                    &PointCloudToOctomap::statusCallback, this);

    start_time_ = ros::Time::now();
    last_status_ = start_time_;

    // 日志一律用 ASCII：roscpp 的 rosconsole 在部分环境下会把中文输出成 '?'，
    // 而同样内容用 rospy 打印就正常。为保证终端输出与文档截图干净，这里统一用英文。
    ROS_INFO("pointcloud_to_octomap started");
    ROS_INFO("  cloud topic   : %s", cloud_topic_.c_str());
    ROS_INFO("  world frame   : %s", world_frame_.c_str());
    ROS_INFO("  resolution    : %.2f m", resolution_);
    ROS_INFO("  range         : %.2f ~ %.2f m", min_range_, max_range_);
    ROS_INFO("  publishing    : /octomap_full, /occupied_cells_vis_array (%.1f Hz)",
             rate);
  }

private:
  // ------------------------------------------------------------------
  // 点云回调：变换 → 射线投射 → 更新八叉树
  // ------------------------------------------------------------------
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    if (msg->width * msg->height == 0) {
      return;
    }

    std::string cloud_frame = msg->header.frame_id;
    if (cloud_frame.empty()) {
      cloud_frame = sensor_frame_;
    }

    // 查询 world <- cloud_frame 的变换。
    // 先用点云自带的时间戳（更准确），若 TF 还没更新到该时刻则退化为取最新变换。
    geometry_msgs::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(world_frame_, cloud_frame,
                                          msg->header.stamp, ros::Duration(0.2));
    } catch (const tf2::TransformException& ex) {
      try {
        tf_msg = tf_buffer_.lookupTransform(world_frame_, cloud_frame,
                                            ros::Time(0), ros::Duration(0.0));
        ROS_WARN_THROTTLE(10.0,
                          "TF %s -> %s not available at cloud stamp, using latest: %s",
                          world_frame_.c_str(), cloud_frame.c_str(), ex.what());
      } catch (const tf2::TransformException& ex2) {
        ROS_WARN_THROTTLE(5.0, "cannot look up TF %s -> %s: %s",
                          world_frame_.c_str(), cloud_frame.c_str(), ex2.what());
        return;
      }
    }

    const geometry_msgs::Vector3& t = tf_msg.transform.translation;
    const geometry_msgs::Quaternion& q = tf_msg.transform.rotation;
    const tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    const tf2::Matrix3x3 rot_matrix(rotation);
    const tf2::Vector3 origin(t.x, t.y, t.z);  // 射线原点＝雷达在世界系中的位置

    octomap::Pointcloud cloud;
    cloud.reserve(static_cast<size_t>(msg->width) * msg->height);

    const double min_range_sq = min_range_ * min_range_;

    // PointCloud2 是扁平缓冲，用迭代器按 x/y/z 三个字段顺序读取，
    // 这样不必引入 PCL，也能顺带处理 points 不是紧密排列的情况
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    size_t total = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const size_t index = total++;
      // 抽稀：AirSim 一帧近万个点，逐点投射在虚拟机上跟不上，
      // 隔 point_stride 个取一个，地图质量几乎不变
      if (point_stride_ > 1 && (index % static_cast<size_t>(point_stride_)) != 0) {
        continue;
      }
      const float px = *iter_x;
      const float py = *iter_y;
      const float pz = *iter_z;

      // 没有打到任何物体的射线在点云里是 NaN/inf，直接跳过
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
        continue;
      }

      // 太近的点来自机体自身或传感器噪声，丢弃
      const double range_sq = static_cast<double>(px) * px +
                              static_cast<double>(py) * py +
                              static_cast<double>(pz) * pz;
      if (range_sq < min_range_sq) {
        continue;
      }

      // 传感器坐标系 → 世界坐标系
      const tf2::Vector3 p_world =
          rot_matrix * tf2::Vector3(px, py, pz) + origin;
      cloud.push_back(static_cast<float>(p_world.x()),
                      static_cast<float>(p_world.y()),
                      static_cast<float>(p_world.z()));
    }

    if (cloud.size() == 0) {
      return;
    }

    // 射线投射：从 origin 向每个点发射一条射线，
    // 终点之前的体素记为未命中（空闲），终点体素记为命中（占用）。
    // lazy_eval = true：先只更新叶节点，最后统一 updateInnerOccupancy()，
    // 避免每次插入都向上回溯，速度快很多。
    tree_.insertPointCloud(cloud,
                           octomap::point3d(origin.x(), origin.y(), origin.z()),
                           max_range_, true, false);
    tree_.updateInnerOccupancy();

    if (!received_cloud_) {
      received_cloud_ = true;
      ROS_INFO("first cloud: frame_id=%s, raw points=%zu, usable points=%zu",
               cloud_frame.c_str(), total, cloud.size());
    }

    cloud_count_ += 1;
    point_count_ += cloud.size();
    last_stamp_ = msg->header.stamp;
  }

  // ------------------------------------------------------------------
  // 周期发布：八叉树消息 + 占用体素可视化
  // ------------------------------------------------------------------
  void publishCallback(const ros::TimerEvent&)
  {
    if (!received_cloud_) {
      return;  // 还没有任何点云，不必发布空地图
    }
    publishOctomap();
    publishMarkers();
  }

  void publishOctomap()
  {
    octomap_msgs::Octomap msg;
    msg.header.frame_id = world_frame_;
    msg.header.stamp = last_stamp_.isZero() ? ros::Time::now() : last_stamp_;

    // fullMapToMsg 保留每个节点的概率值；后续 A* 规划节点可以直接由它还原八叉树
    if (!octomap_msgs::fullMapToMsg(tree_, msg)) {
      ROS_WARN_THROTTLE(5.0, "failed to convert octree to octomap_msgs/Octomap");
      return;
    }
    octomap_pub_.publish(msg);
  }

  void publishMarkers()
  {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = world_frame_;
    marker.header.stamp = last_stamp_.isZero() ? ros::Time::now() : last_stamp_;
    marker.ns = "octomap_occupied";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    // 立方体边长＝八叉树分辨率，即一个叶节点
    marker.scale.x = resolution_;
    marker.scale.y = resolution_;
    marker.scale.z = resolution_;
    marker.color.a = 1.0;
    // 给有限存活时间：地图基本只增不减，万一有体素被清掉，
    // 旧 Marker 会自行过期，不必额外发 DELETE
    marker.lifetime = ros::Duration(2.0 / std::max(publish_rate_, 0.1));

    int id = 0;
    for (octomap::OcTree::leaf_iterator it = tree_.begin_leafs(),
                                        end = tree_.end_leafs();
         it != end; ++it) {
      if (!tree_.isNodeOccupied(*it)) {
        continue;  // 空闲节点不画
      }
      if (id >= max_markers_) {
        ROS_WARN_THROTTLE(10.0, "occupied voxels exceed max_markers = %d, "
                          "showing only the first %d", max_markers_, max_markers_);
        break;
      }
      marker.id = id++;
      marker.pose.position.x = it.getX();
      marker.pose.position.y = it.getY();
      marker.pose.position.z = it.getZ();
      heightColor(it.getZ(), marker.color);
      array.markers.push_back(marker);
    }

    markers_pub_.publish(array);
  }

  // 按高度着色：低处偏蓝、中间偏绿、高处偏红，便于在 RViz 中分辨高度层次
  void heightColor(double z, std_msgs::ColorRGBA& color) const
  {
    double ratio = 0.0;
    if (color_max_z_ > color_min_z_) {
      ratio = (z - color_min_z_) / (color_max_z_ - color_min_z_);
    }
    ratio = std::min(std::max(ratio, 0.0), 1.0);

    color.r = ratio;
    color.g = 1.0 - std::fabs(2.0 * ratio - 1.0);
    color.b = 1.0 - ratio;
    color.a = 1.0;
  }

  // ------------------------------------------------------------------
  // 状态输出：既是运行观察，也是「没收到点云」时的排查提示
  // ------------------------------------------------------------------
  void statusCallback(const ros::TimerEvent&)
  {
    const ros::Time now = ros::Time::now();

    if (!received_cloud_) {
      if ((now - start_time_).toSec() > cloud_timeout_) {
        ROS_WARN_THROTTLE(10.0,
                          "no point cloud received after %.0f s (topic %s). "
                          "Check that the simulator is running and the bridge node "
                          "is publishing; use 'rostopic info %s' to verify the type "
                          "is sensor_msgs/PointCloud2",
                          (now - start_time_).toSec(), cloud_topic_.c_str(),
                          cloud_topic_.c_str());
      }
      return;
    }

    if ((now - last_status_).toSec() < status_period_) {
      return;
    }
    last_status_ = now;

    size_t occupied = 0;
    for (octomap::OcTree::leaf_iterator it = tree_.begin_leafs(),
                                        end = tree_.end_leafs();
         it != end; ++it) {
      if (tree_.isNodeOccupied(*it)) {
        occupied += 1;
      }
    }

    ROS_INFO("frames=%d, points=%zu, occupied voxels=%zu "
             "(resolution %.2f m, octree memory %.1f MB)",
             cloud_count_, point_count_, occupied, resolution_,
             tree_.memoryUsage() / 1048576.0);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  octomap::OcTree tree_;

  ros::Subscriber cloud_sub_;
  ros::Publisher octomap_pub_;
  ros::Publisher markers_pub_;
  ros::Timer publish_timer_;
  ros::Timer status_timer_;

  // 参数
  std::string cloud_topic_;
  std::string world_frame_;
  std::string sensor_frame_;
  double resolution_;
  double max_range_;
  double min_range_;
  double publish_rate_;
  double color_min_z_;
  double color_max_z_;
  int max_markers_;
  int point_stride_;
  double status_period_;
  double cloud_timeout_;

  // 运行状态
  int cloud_count_;
  size_t point_count_;
  bool received_cloud_;
  ros::Time start_time_;
  ros::Time last_status_;
  ros::Time last_stamp_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_to_octomap");
  PointCloudToOctomap node;
  ros::spin();
  return 0;
}
