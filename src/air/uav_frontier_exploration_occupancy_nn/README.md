# uav_frontier_exploration_occupancy_nn

占据栅格前沿探索仿真：射线探测维护未知/自由/占据三态栅格，提取并聚类前沿候选，两层 numpy MLP 按距离、方位、未知邻域与自由密度打分选择目标前沿，控制器朝目标绕行探索。

## 概览

| 项目 | 内容 |
| --- | --- |
| 话题类别 | 任务 3：虚拟环境建图与自主探索（占据栅格 + 神经网络前沿选点） |
| 平台 | Ubuntu 20.04 ROS Noetic / Ubuntu 22.04 ROS 2 Humble；离线仿真可在 Windows Python 3.13 运行 |
| 语言 | Python 3 |
| 依赖 | rclpy、nav_msgs、std_msgs（ROS 节点）；离线仅依赖 numpy、pygame |
| 运行方式 | `ros2 launch uav_frontier_exploration_occupancy_nn uav_frontier_exploration_occupancy_nn.launch.py`；离线 `python main.py --shots` |
| 仿真器 | 内置 numpy 平面运动模型 + pygame 离屏/窗口渲染 |

## 原理

- 18 条射线在 240° 视场内探测，沿途标记自由格，命中障碍在端点标记占据格，维护未知/自由/占据三态占据栅格。
- 自由格且邻接未知格即为前沿格，八邻域连通聚类为候选前沿点，提取距离、相对方位、未知邻域数与局部自由密度 4 维特征。
- 两层 MLP（输入 4，隐藏层 6）对候选打分，选评分最高的前沿作为导航目标；控制器航向 PD 跟踪目标，前方受阻时朝自由侧转向，位姿安全护盾禁止驶入占据格。

## 快速运行

### ROS 2 Humble

```bash
colcon build --packages-select uav_frontier_exploration_occupancy_nn --symlink-install
source install/setup.bash
ros2 launch uav_frontier_exploration_occupancy_nn uav_frontier_exploration_occupancy_nn.launch.py
```

### 离线仿真（无需 ROS）

```bash
python main.py --shots    # 离屏运行并保存全过程截图到 docs/img/
python main.py --demo     # 打开观察窗口，查看前沿探索过程
```

## 测试

```bash
python -m pytest test/ -q
```

## 文件结构

```
uav_frontier_exploration_occupancy_nn/
├── launch/uav_frontier_exploration_occupancy_nn.launch.py
├── uav_frontier_exploration_occupancy_nn/
│   ├── occupancy_map.py  # 三态占据栅格、射线探测与前沿格判定
│   ├── frontier.py       # 前沿聚类与候选特征提取
│   ├── frontier_policy.py# 两层 MLP 前沿打分与目标选择
│   ├── controller.py     # 航向跟踪、受阻转向与安全护盾
│   ├── explorer.py       # 建图-选点-导航闭环
│   └── explorer_node.py  # ROS 2 封装节点
├── test/                  # 地图、前沿、策略、控制器单元测试
├── main.py                # 离线一键实验入口
├── package.xml
└── setup.py
```

## 许可

MIT
