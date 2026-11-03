# mrobot_gazebo

在 Gazebo Classic 11 中加载一台带 **RPLIDAR 二维激光雷达**的 **mrobot** 移动机器人，并把它放进自建的房间世界 `my_room`，为后续 SLAM 建图与自主导航做环境准备。

## 运行环境

- Ubuntu 20.04（focal）
- ROS 1 Noetic（`rosdistro=noetic`）
- Gazebo Classic 11
- 依赖：`gazebo_ros`、`gazebo_plugins`、`gazebo_ros_control`、`xacro`、`joint_state_publisher`、`robot_state_publisher`

## 目录结构

```text
mrobot_gazebo/
├── launch/
│   └── mrobot_laser_nav_gazebo.launch   # 启动 Gazebo、载入 my_room、生成 mrobot
├── urdf/
│   ├── mrobot_with_rplidar.urdf.xacro   # 顶层：车身 + 激光雷达
│   ├── mrobot_body.urdf.xacro           # 底盘、轮子、万向轮、两层板、差速插件
│   └── rplidar.xacro                    # RPLIDAR 模型与激光插件
├── worlds/
│   └── my_room.world                    # 自建房间世界（Building Editor 绘制）
└── models/
    └── my_room/                         # 房间模型（model.config + model.sdf）
```

## 运行步骤

```bash
# 1. 安装依赖
sudo apt update
sudo apt install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
                    ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher ros-noetic-xacro

# 2. 编译并 source 工作空间
cd <catkin_ws>
catkin_make
source devel/setup.bash

# 3. 启动
roslaunch mrobot_gazebo mrobot_laser_nav_gazebo.launch
```

## 预期效果

- Gazebo 打开自建房间 `my_room`（含交通锥）；
- 终端无报错，`rostopic echo -n1 /gazebo/model_states` 的 name 中包含 `mrobot`；
- 房间原点出现一台 mrobot：圆形底盘、两个驱动轮、前部万向轮、两层顶板、顶部激光雷达。

## Noetic 适配说明

1. 宏调用必须带 `xacro:` 前缀（如 `<xacro:mrobot_body/>`、`<xacro:wheel/>`），否则新版 xacro 不会展开，导致 `Could not find link 'plate_2_link'`、`parseURDF` 失败、小车无法生成。
2. xacro 默认按文档顺序处理，`--inorder` 选项已删除。
3. 若启动后出现两个 `/joint_states` 发布者导致轮子抖动，在 launch 中注释掉 `joint_state_publisher` 节点。
4. 启动时 Gazebo 顶部"end-of-life"黄条、激光/差速插件的 deprecation 警告均为正常提示，不影响功能。
