# ROS 虚拟机器人圆形轨迹 RViz 可视化入门

## 一、项目概述

本项目基于 ROS Noetic，编写了一个简单的 ROS 节点，持续发布里程计（Odometry）话题。该节点模拟了一个虚拟机器人沿着半径 1 米的圆形轨迹匀速绕圈，并通过 RViz 进行实时可视化展示。

**项目亮点：**
- 轻量级：无需安装 Gazebo 等大型仿真软件，基础 ROS 环境即可运行。
- 直观：一键启动后，RViz 自动加载预配置视图，直观看到小机器人绕圈。
- 知识点串联：将“发布消息”与“RViz 可视化”结合，适合初学者练手。

## 二、部署与运行流程

### 2.1 创建并编译工作区
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
###2.2 一键启动运行
roslaunch ros_topic_demo rviz_demo.launch
启动后，RViz 窗口将自动弹出，无需手动配置。节点会同时广播 odom -> base_link 的 TF，确保不会出现 No TF data 报错。
## 三、常见问题与解决记录
1. Fixed Frame 选错导致看不到点
◦ 解决：将 Fixed Frame 选为 odom 即可显示。
2. Rviz 画出一堆密密麻麻的箭头
◦ 解决：修正 rospy.Rate 参数（如 30Hz），使轨迹平滑。
3. 报错 No TF data
◦ 解决：在代码中使用 tf2_ros.TransformBroadcaster 广播 odom -> base_link。
## 四、运行效果
启动后，RViz 窗口中的 Odometry 箭头沿半径 1m 的正圆匀速绕圈，Global Status 为 Ok。
