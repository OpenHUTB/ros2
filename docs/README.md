# ackermann_sim — ROS1 Noetic 阿克曼小车仿真

最小可跑的阿克曼转向(Ackermann steering)ROS 工程。不用 Carla, 不用 Gazebo,
纯 Python 运动学模型 + matplotlib 可视化, 适合上课/作业/调控制算法。

## 工程结构

```
ackermann_ws/
└── src/ackermann_sim/
    ├── package.xml
    ├── CMakeLists.txt
    ├── scripts/
    │   ├── bike_model.py        # 阿克曼自行车运动学 (纯 Python, 无 ROS 依赖)
    │   ├── sim_node.py           # ROS 仿真节点: 订阅 /cmd_vel, 发布 /odom + tf
    │   ├── keyboard_teleop.py    # 键盘遥控: WASD/方向键, 发 /cmd_vel
    │   └── plot_viewer.py        # matplotlib 画图: 订阅 /odom 画小车和轨迹
    └── launch/
        └── sim.launch            # 一键启动仿真 + 可视化
```

## 运动学模型

自行车模型 (bicycle model):

```
x_dot     = v * cos(theta)
y_dot     = v * sin(theta)
theta_dot = v / L * tan(delta)
v_dot     = a
```

- L = 2.5 m (轴距)
- max_delta = 0.5 rad (~28.6°)
- max_v = 5 m/s

## 话题约定

| 话题     | 类型                      | 方向    | 含义 |
|----------|---------------------------|---------|------|
| /cmd_vel | geometry_msgs/Twist       | 订阅    | linear.x = 目标速度 v (m/s), angular.z = 前轮转角 delta (rad) |
| /odom    | nav_msgs/Odometry         | 发布    | 小车位姿 (x, y, theta) 和速度 |
| tf       | odom → base_link          | 发布    | 供 rviz / 导航栈使用 |

## 编译运行

```bash
# 1. 进到 catkin workspace 根目录
cd ~/ackermann_ws          # 或者你放这个工程的位置
catkin_make                # 或 catkin build
source devel/setup.bash

# 2. 启动 roscore (新开一个终端)
roscore

# 3. 启动仿真 + 可视化 (新开一个终端)
roslaunch ackermann_sim sim.launch

# 4. 键盘遥控 (再开一个终端)
source devel/setup.bash
rosrun ackermann_sim keyboard_teleop.py
# 然后在键盘终端里按:
#   w/↑ 前进   s/↓ 刹车/倒车
#   a/← 左转   d/→ 右转
#   空格 急停   q 退出
```

## 只用 rviz 看 (不用 matplotlib)

把 `launch/sim.launch` 里的 `plot_viewer` 节点注释掉, 然后:

```bash
rosrun rviz rviz
# Fixed Frame 选 odom
# 加一个 TF 显示就能看到 base_link 在动
# 加一个 Odometry, topic 选 /odom, 就能看到轨迹线
```

## 自己写控制节点

只要往 `/cmd_vel` 发 `geometry_msgs/Twist` 就行:

```python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node("my_controller")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
r = rospy.Rate(20)

while not rospy.is_shutdown():
    msg = Twist()
    msg.linear.x = 1.0     # 1 m/s 前进
    msg.angular.z = 0.3    # 前轮右转 0.3 rad
    pub.publish(msg)
    r.sleep()
```

## 调参

在 launch 文件里改:
- `wheelbase` 轴距
- `max_steer` 最大转角
- `max_speed` 最大速度
- `rate` 仿真频率
