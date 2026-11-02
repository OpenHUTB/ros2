# ros2_pure_pursuit

差速移动机器人的**纯追踪路径跟踪 + 激光反应式避障**实现。

- 文档页：[地面载具 ROS2 差速机器人纯追踪路径跟踪与反应式避障](https://openhutb.github.io/ros2/ground/ros2_pure_pursuit/)
- 作者：李诚轩（学号 2535030014）

核心算法与闭环仿真只依赖 NumPy，**不需要 ROS、不需要仿真器**即可完整复现；
在有 ROS 2 (Humble) 的环境下，`src/ros2_pure_pursuit/node.py` 提供开箱即用的节点。

## 目录结构

```
ros2_pure_pursuit/
├── src/ros2_pure_pursuit/
│   ├── robot.py          # 差速运动学、加速度限幅、轮速反解
│   ├── paths.py          # 参考路径生成与几何工具
│   ├── pure_pursuit.py   # 纯追踪控制器
│   ├── obstacle.py       # 激光扇区危险度与反应式避障
│   ├── arbitrate.py      # 速度仲裁（终点/避障/急停/限幅的优先级）
│   ├── safety.py         # 话题超时跟踪与参数合法性校验
│   └── node.py           # ROS2 节点（订阅 /odom /scan，发布 /cmd_vel）
├── sim/
│   ├── run_sim.py        # 单场景离线仿真与出图
│   └── benchmark.py      # 4 路径 × 有无避障 基准矩阵
├── launch/nav.launch.py  # ROS2 启动文件
├── tests/test_core.py    # 单元测试（27 项）
├── ci/run_ci.sh          # 一键检查（语法/依赖/测试/仿真/基准）
├── package.xml           # ROS2 包描述
└── setup.py              # Python 包配置
```

## 快速开始

```bash
# 1. 安装依赖
pip3 install -r requirements.txt        # numpy, matplotlib

# 2. 离线仿真（无需 ROS）
python3 sim/run_sim.py --path eight     # 可选 circle / eight / s_curve / straight
python3 sim/run_sim.py --path eight --no-avoid   # 对照组：关闭避障
python3 sim/benchmark.py                # 生成基准矩阵 results/benchmark.md

# 3. 一键检查
bash ci/run_ci.sh

# 4. ROS2 中部署
colcon build --packages-select ros2_pure_pursuit
source install/setup.bash
ros2 launch ros2_pure_pursuit nav.launch.py
```

## 核心公式

前视距离随速度自适应：

$$L_d = \mathrm{clip}(0.45|v| + 0.6,\ 0.4,\ 2.5)$$

由纯追踪几何关系解出角速度：

$$\omega = \frac{2v\sin\alpha}{L_d}$$

前向扇区危险度驱动的减速与转向偏置：

$$v \leftarrow v \cdot (1 - 0.85 r),\qquad \omega_{\text{bias}} = \sigma \cdot 1.6 \cdot r$$

## 控制仲裁与安全兜底

每个控制周期最终下发哪一组 $(v, \omega)$ 由 `arbitrate()` 统一裁决，优先级从高到低：

| 优先级 | 条件 | 结果 | `reason` |
| :---: | --- | --- | --- |
| 0 | `/odom` 超过 `odom_timeout`(0.5 s) 未更新 | 停车 $(0,0)$ | `stale_odom` |
| 1 | 到达路径终点 | 停车 $(0,0)$ | `finished` |
| 2 | 正常跟踪 | 采纳纯追踪输出 | `pursuit` |
| 3 | 前向有障碍 | 叠加转向偏置与减速 | `avoidance` |
| 4 | 前向测距 $\le d_{\text{danger}}$(0.45 m) | 急停 $v=0$，保留转向脱离 | `emergency_stop` |

出口处统一把角速度截断到 $\pm$`max_angular`(1.8 rad/s)。
`arbitrate()` 与 `safety.py` 都不依赖 rclpy，因此上述规则全部有单元测试覆盖。

## 实验结果摘要

四条路径（圆形 / 8 字形 / S 形 / 直线）× 有无避障的对照实验：

| 避障 | 碰撞 | RMS 横向误差 | 前向最小测距 |
| --- | :---: | ---: | ---: |
| 开 | 否（4/4 安全通过） | 0.070 ~ 0.116 m | 0.711 ~ 0.742 m |
| 关 | **是（4/4 全部碰撞）** | — | — |

单元测试 27 项全部通过（`python3 tests/test_core.py`），覆盖运动学、纯追踪、避障、
急停、话题超时与参数校验；一键 `bash ci/run_ci.sh` 五阶段全绿。
