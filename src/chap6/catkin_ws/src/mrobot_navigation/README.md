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
├── maps/
│   ├── gmapping_map.pgm         # 随包提供的一次建图结果（512×544，0.05 m/pix）
│   ├── gmapping_map.yaml        # 上面这张地图的配置
│   └── map.yaml                 # 自己建图时 map_saver 生成的配置模板
├── scripts/
│   ├── random_navigation.py     # move_base 随机目标点自动导航（需 ROS）
│   └── map_planner.py           # 离线地图检查 + 全局路径规划（不依赖 ROS）
└── test/
    └── test_map_planner.py      # map_planner 的 33 个单元测试
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

生成 `map.pgm` 与 `map.yaml`。本包 `maps/gmapping_map.pgm` 即为一次实际建图结果（512×544，0.05 m/pix）。

## 离线检查地图与规划路径（不需要 ROS）

`map_planner.py` 把「地图体检 + 全局路径规划」搬到离线：读完 `map_saver` 生成的
`.pgm` + `.yaml`，统计自由/占用/未知比例与连通性，按机器人半径做障碍膨胀，
再用 A* 规划路径并做视线法平滑，最后输出一张叠加图。只用 Python 标准库，
Windows / macOS / Ubuntu 都能直接跑，不必启动 Gazebo 和 move_base。

```bash
cd src/chap6/catkin_ws/src/mrobot_navigation

# 只体检
python3 scripts/map_planner.py --map maps/gmapping_map.yaml --stats

# 规划一条全局路径（坐标与 rviz 的 2D Nav Goal 一致，单位米）
python3 scripts/map_planner.py --map maps/gmapping_map.yaml \
    --start 1.15 5.46 --goal 9.77 5.17 --radius 0.18 --out plan.png
```

输出示例（本章地图）：

```text
自由 / 占用 / 未知 : 11.5% / 1.2% / 87.4%
最大连通自由区    : 97.7%（占自由空间）
可通行面积        : 7.1%（半径 0.18 m 膨胀后，膨胀前 11.5%）
A* 路径长度       : 10.59 m
平滑后点数 / 长度  : 6 / 9.96 m
直线距离          : 8.62 m（绕行系数 1.15）
```

测试：

```bash
cd src/chap6/catkin_ws/src/mrobot_navigation
python3 -m unittest discover -s test
```

应显示 `Ran 33 tests ... OK`。

## 建图建议

- 线速度保持 0.2 m/s 左右，转弯点动，不要误触 `q/w/e` 加速键；
- 全覆盖每个房间，并尽量走闭环以修正累计误差；
- 速度过快会导致激光与定位跟不上，地图出现放射状噪点和重影。
