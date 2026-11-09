# ackermann_dubins_path — 满足最小转弯半径的最短路径

阿克曼小车不能原地转向，也走不出比最小转弯半径 `R` 更急的弯。于是「两点之间最短」
不再是一条直线，而是「圆弧 - 直线 - 圆弧」或「弧 - 弧 - 弧」的组合。Dubins 证明
最优解一定属于六种情形：`LSL / LSR / RSL / RSR / RLR / LRL`。

本模块给出几何构造 + 端点闭环校验的实现，并用配置空间搜索交叉验证最短性。

## 目录

```
ackermann_dubins_path/
├── scripts/
│   ├── dubins.py         # 六情形几何构造、端点校验、路径采样
│   ├── simulate.py       # 工况表、朝向扫描、半径扫描
│   ├── main.py           # 打印统计 + 生成 PNG
│   ├── ros_node.py       # 发布 nav_msgs/Path
│   └── simple_plot.py    # 手写 zlib PNG（无需 matplotlib）
├── tests/test_dubins.py  # 19 项单元测试
└── launch/ackermann_dubins_path.launch.py
```

## 快速开始

```bash
python3 scripts/main.py      # 打印工况表并生成 3 张 PNG
python3 tests/test_dubins.py # 19 项单元测试
```

只依赖 Python 标准库，不需要 ROS、numpy 或 matplotlib。

## 结果（R = 1 m）

| 工况 | 最优情形 | 长度 | 次优解 | 相比省下 |
|---|---|---|---|---|
| (0,0,0) → (10,0,0) 同向 | LSL | 10.0000 m | 10.0000 m | 0% |
| (0,0,0) → (10,0,90°) | RSL | 10.6266 m | 15.7580 m | 32.6% |
| (0,0,0) → (10,0,180°) | LSR | 13.3423 m | 13.3423 m | 0%（与 RSL 对称） |
| (0,0,0) → (6,4,-90°) | LSR | 8.8291 m | 12.3282 m | 28.4% |

端点闭环校验：把三段弧长积分回去，位置误差 < 1e-15 m、朝向误差 0。

## 在 ROS 2 中运行

```bash
colcon build --packages-select ackermann_dubins_path
source install/setup.bash
ros2 launch ackermann_dubins_path ackermann_dubins_path.launch.py
# 另开终端
ros2 topic echo /dubins/path --once
rviz2      # 添加 Path 显示，Topic 选 /dubins/path
```

参数：`start_x / start_y / start_yaw / goal_x / goal_y / goal_yaw / radius`。

## 关于验证范围

算法与全部数值都在本地实跑验证（`tests/test_dubins.py` 19 项全过）。
`colcon build` 与 `ros2 launch` 未在安装了 ROS 2 发行版的机器上实测，
节点代码按 rclpy 标准接口编写。
