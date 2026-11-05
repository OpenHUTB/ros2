# mrobot_navigation

mrobot 激光 SLAM 建图示例。本包只提供 launch、rviz 配置和保存好的地图，自身不含可执行节点；真正的 SLAM 节点由外部功能包 `gmapping` 提供，地图保存由 `map_server` 提供。

## 目录结构

```text
mrobot_navigation/
├── launch/
│   ├── gmapping.launch          # 配置并启动 slam_gmapping 节点（参数 + 话题）
│   └── gmapping_demo.launch     # 包含 gmapping.launch，并启动 rviz
├── rviz/
│   └── gmapping.rviz            # Fixed Frame=map，显示 Map(/map)、LaserScan(/scan)、RobotModel、TF
└── maps/
    ├── map.pgm                  # 建出的占据栅格地图（图像）
    └── map.yaml                 # 地图配置（分辨率、原点、阈值）
```

## 运行（需配合 mrobot_gazebo 与 mrobot_teleop）

```bash
# 终端 1：Gazebo 房间 + 小车
roslaunch mrobot_gazebo mrobot_laser_nav_gazebo.launch
# 终端 2：gmapping + rviz
roslaunch mrobot_navigation gmapping_demo.launch
# 终端 3：键盘遥控
roslaunch mrobot_teleop mrobot_teleop.launch
```

## 保存地图

```bash
cd <要保存的目录>
rosrun map_server map_saver
```

生成 `map.pgm` 与 `map.yaml`。本包 `maps/` 下的地图即为一次实际建图结果（576×544，0.05 m/pix）。

## 建图建议

- 线速度保持 0.2 m/s 左右，转弯点动，不要误触 `q/w/e` 加速键；
- 全覆盖每个房间，并尽量走闭环以修正累计误差；
- 速度过快会导致激光与定位跟不上，地图出现放射状噪点和重影。
