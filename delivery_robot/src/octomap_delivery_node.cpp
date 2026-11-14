#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

octomap::OcTree* octree = nullptr;
ros::Publisher vel_pub;
geometry_msgs::Point target_point;

// 八叉树地图回调函数
void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg)
{
  if(octree != nullptr)
  {
    delete octree;
    octree = nullptr;
  }
  octomap::AbstractOcTree* tree_abs = octomap_msgs::binaryMsgToMap(*msg);
  octree = dynamic_cast<octomap::OcTree*>(tree_abs);
  if(!octree)
  {
    ROS_ERROR("八叉树地图转换失败！");
    return;
  }
  ROS_INFO("八叉树地图更新成功");
}

// 里程计回调，控制小车向快递目标点(4,0)前进，简易避障
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  if(octree == nullptr) return;

  double robot_x = odom_msg->pose.pose.position.x;
  double robot_y = odom_msg->pose.pose.position.y;
  double robot_yaw = tf::getYaw(odom_msg->pose.pose.orientation);

  geometry_msgs::Twist vel_cmd;

  // 计算到目标点距离
  double dx = target_point.x - robot_x;
  double dy = target_point.y - robot_y;
  double dist = sqrt(dx*dx + dy*dy);

  // 到达快递终点，停车
  if(dist < 0.3)
  {
    vel_cmd.linear.x = 0;
    vel_cmd.angular.z = 0;
    vel_pub.publish(vel_cmd);
    ROS_INFO("✅ 小车到达快递投递点，配送完成！");
    return;
  }

  // 判断前方有没有障碍物（沿小车正前方0.8m检测）
  octomap::point3d check_point(robot_x + 0.8*cos(robot_yaw), robot_y + 0.8*sin(robot_yaw), 0.2);
  octomap::OcTreeNode* node = octree->search(check_point);
  bool has_obstacle = false;
  if(node != nullptr)
  {
    has_obstacle = octree->isNodeOccupied(node);
  }

  if(has_obstacle)
  {
    // 检测到障碍物，原地右转避障
    ROS_WARN("🚧 前方检测障碍物，正在绕行！");
    vel_cmd.linear.x = 0;
    vel_cmd.angular.z = 0.4;
  }
  else
  {
    // 无障碍物，朝向目标前进
    double target_angle = atan2(dy, dx);
    double angle_diff = target_angle - robot_yaw;
    while(angle_diff > M_PI) angle_diff -= 2*M_PI;
    while(angle_diff < -M_PI) angle_diff += 2*M_PI;

    vel_cmd.angular.z = 0.6 * angle_diff;
    vel_cmd.linear.x = 0.2;
  }
  vel_pub.publish(vel_cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_delivery_node");
  ros::NodeHandle nh;

  // 快递目标点：红色方块 (4.0, 0.0)
  target_point.x = 4.0;
  target_point.y = 0.0;
  target_point.z = 0;

  ros::Subscriber octomap_sub = nh.subscribe("/octomap_binary",1,octomapCallback);
  ros::Subscriber odom_sub = nh.subscribe("/odom",1,odomCallback);
  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

  ROS_INFO("🚚 快递配送小车八叉树寻路节点启动！目标点(4,0)");
  ros::spin();

  if(octree) delete octree;
  return 0;
}

