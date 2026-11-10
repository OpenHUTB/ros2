# ugv_slam_nav

地面无人车 SLAM 建图 + 导航（ROS2 Humble ament_python），turtlebot3 + slam_toolbox + nav2。

文档见 [docs/ground/ugv_slam_nav.md](../../docs/ground/ugv_slam_nav.md)。

## 快速跑

```bash
# 先起 turtlebot3 gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# 再起本包
ros2 launch ugv_slam_nav main.launch.py
```

## 测试

```bash
python3 test/test_ugv_slam_nav_local.py
```
