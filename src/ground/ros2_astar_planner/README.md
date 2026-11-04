# ROS2 差速机器人栅格 A* 全局路径规划

面向差速移动机器人的**全局路径规划**模块：给定占用栅格地图与目标位置，输出一条
**安全、平滑、带时间参数**的路径，可直接交给下层控制器（如
[`ros2_pure_pursuit`](https://github.com/OpenHUTB/ros2/tree/master/src/ground/ros2_pure_pursuit)）跟踪。

核心算法只依赖 NumPy，**无需 ROS / 仿真器**即可跑通仿真与单元测试。

> 定位：本模块属于**规划层**。与仓库中已合并的
> `ros2_pure_pursuit`（路径跟踪控制层）、`ros2_ekf_localization`（状态估计层）互补，
> 三者构成 "定位 → 规划 → 控制" 的完整链路。

## 1. 算法总览

```
/map (OccupancyGrid) ─┐
                      ├─► 膨胀(避障裕度) ─► A* 搜索 ─► 视线捷径 ─► 梯度平滑 ─► 梯形速度剖面 ─► /plan (Path)
/goal (PoseStamped) ──┤        ▲                                                        ▲
/robot_pose ──────────┘        └──────── 连通性预检 / 端点修复 / 超时保护 ─────────────┘
```

### 1.1 占用栅格与膨胀

栅格数组 `grid[i, j]` 中 `i` 为行（y 方向）、`j` 为列（x 方向），非零即占用。

规划前按机器人半径做**方形结构元膨胀**（切比雪夫距离），实现上用两次可分离的
最小值滤波完成，复杂度 `O(HW)` 且与半径无关：

$$
d_{\text{clear}} = \left\lceil \frac{r_{\text{robot}} + r_{\text{extra}}}{\text{resolution}} \right\rceil \text{ 格}
$$

膨胀把机器人当作一个圆处理，避免规划出的路径"贴着墙走"。

### 1.2 A* 搜索

状态为栅格坐标，代价函数用 8 邻接步长（直行 1、对角 $\sqrt{2}$）：

$$
f(n) = g(n) + w \cdot h(n), \qquad g(n) = g(\text{parent}) + c(n, \text{parent})
$$

启发式按邻接方式自动选择，保证**可采纳性**（$h$ 不超过真实剩余代价）：

| 邻接 | 启发式 | 公式 |
| --- | --- | --- |
| 8 邻接 | octile | $\sqrt{2}\min(\Delta i,\Delta j) + \max(\Delta i,\Delta j) - \min(\Delta i,\Delta j)$ |
| 4 邻接 | manhattan | $\lvert \Delta i\rvert + \lvert \Delta j\rvert$ |

工程细节：

- **二叉堆 + 延迟删除**：不维护 decrease-key 索引，节点重复弹出时跳过；
- **平局打破**：`f` 相同时优先 `h` 小者，开阔地形上减少无谓扩展；
- **严格穿角**：对角步要求两侧格**都**是自由格。只禁止"两侧都占用"虽然栅格层面合法，
  但连接两个格心的直线仍可能擦过占用格一角，与后处理的连续空间碰撞检查语义不一致；
- **有界搜索**：`max_expansions` 超限立即终止，地图异常时不会打满 CPU。

### 1.3 路径后处理

1. **视线捷径（string pulling）**：若两点间直线不穿越障碍，用直线段替换中间折点。
   碰撞检查用**超采样**（采样间距为分辨率的 1/4），比 Bresenham 更保守；
2. **梯度平滑**：目标函数

   $$
   E_i = w_s \left\lVert p_i - \tfrac{1}{2}(p_{i-1}+p_{i+1}) \right\rVert^2
       + w_o \max(0, d_{\text{clear}} - d(p_i))^2
   $$

   第一项把折线拉直，第二项只在点离障碍过近时产生斥力，把路径推离墙面。
   梯度解析可求，无需自动微分。

平滑后**逐段复核碰撞**，若任一段被判碰撞则回退到捷径结果——保证输出永远安全。

### 1.4 时间参数化

几何路径转成可执行轨迹：

1. 三点外接圆估计曲率 $\kappa$；
2. 曲率限速 $v \le \sqrt{a_{\text{lat}} / \lvert\kappa\rvert}$；
3. 沿弧长做**前向 + 后向两遍扫描**，用 $v^2 - v_0^2 = 2a\Delta s$ 保证加减速不超限
   （后向扫描负责提前刹车）；
4. 逐段积分得到时间戳。

## 2. 目录结构

```
ros2_astar_planner/
├── src/ros2_astar_planner/
│   ├── gridmap.py       # 占用栅格：坐标换算、膨胀、连通域、地图生成
│   ├── astar.py         # A* 搜索（二叉堆、平局打破、严格穿角、扩展上限）
│   ├── smoothing.py     # 视线捷径 + 梯度平滑 + 连续空间碰撞检查
│   ├── trajectory.py    # 曲率估计、梯形速度剖面、时间参数化
│   ├── safety.py        # 参数校验、端点修复、连通性预检、耗时保护
│   └── node.py          # ROS2 节点（/map /goal /robot_pose → /plan）
├── sim/
│   ├── run_sim.py       # 单场景离线仿真与出图（4 联图）
│   └── benchmark.py     # 基准矩阵（启发式 / 邻接 / 权重 / 通道宽度）
├── launch/plan.launch.py
├── tests/test_core.py   # 单元测试（50 项）
├── ci/run_ci.sh         # 一键自检（语法/依赖/测试/仿真/基准 五阶段）
└── results/             # 仿真图与基准矩阵
```

## 3. 快速开始

```bash
# 一键自检（不需要 ROS）
bash ci/run_ci.sh

# 单元测试
python3 tests/test_core.py

# 单场景仿真并出图
python3 sim/run_sim.py --map corridor --out results/corridor.png
python3 sim/run_sim.py --map maze --out results/maze.png
python3 sim/run_sim.py --map random --out results/random.png

# 基准矩阵
python3 sim/benchmark.py      # 输出 results/benchmark.md
```

### 在 ROS2 中运行

```bash
colcon build --packages-select ros2_astar_planner
source install/setup.bash
ros2 launch ros2_astar_planner plan.launch.py robot_radius:=0.20
```

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/map` | `nav_msgs/OccupancyGrid` | 订阅 | 栅格地图（未知区域按占用处理） |
| `/goal` | `geometry_msgs/PoseStamped` | 订阅 | 目标点 |
| `/robot_pose` | `geometry_msgs/PoseStamped` | 订阅 | 机器人当前位姿（起点） |
| `/plan` | `nav_msgs/Path` | 发布 | 平滑并时间参数化后的路径 |
| `/plan_info` | `std_msgs/String` | 发布 | JSON 诊断信息 |

常用参数：`robot_radius`、`inflation_extra`、`allow_diagonal`、`heuristic`、
`weight`、`v_max`、`a_max`、`plan_timeout`、`map_timeout`、`use_smoothing`。

## 4. 实验结果摘要

### 4.1 规划质量（三种地图 × 五种搜索配置）

| 指标 | 结果 |
| --- | --- |
| 最优性 | A*(octile) 与 Dijkstra 代价**完全一致**（相对差 0.00%），证明启发式可采纳 |
| 搜索效率 | A* 相比 Dijkstra 扩展节点减少 **2.8% ~ 67.2%**（迷宫因通道狭窄，启发式信息量低） |
| 加权 A* | `w=1.5` 扩展节点最多减少 **80.5%**，代价最多劣化 **3.2%** |
| 对角收益 | 8 邻接比 4 邻接代价低 **11% ~ 18%** |

### 4.2 平滑效果

| 场景 | 原始长度 (m) | 平滑后 (m) | 缩短 | 转角数 |
| --- | --- | --- | --- | --- |
| corridor | 6.35 | 6.05 | 4.8% | 12 → 4 |
| maze | 44.67 | 44.61 | 0.1% | 29 → 30 |
| random | 7.69 | 7.28 | 5.3% | 21 → 12 |

迷宫场景几乎无法缩短——通道狭窄时两点之间本就没有可直达的视线。
这说明**后处理不会为了缩短路径而牺牲避障安全**。

### 4.3 通道宽度与机器人半径

用"一墙一门"地图（端点在房间中央，连通性只由门宽决定）：

| 门洞净宽 | 膨胀 1 格 (0.10 m) | 膨胀 2 格 (0.20 m) | 膨胀 3 格 (0.30 m) |
| --- | --- | --- | --- |
| 0.10 m | 不可通行 | 不可通行 | 不可通行 |
| 0.20 m | 可通行 | 不可通行 | 不可通行 |
| 0.30 m | 可通行 | 可通行 | 不可通行 |
| 0.40 m 以上 | 可通行 | 可通行 | 可通行 |

> **通道净宽必须大于 2×膨胀半径**。膨胀层把门洞堵死时，规划器会如实返回
> "起点与终点不连通"，而**不会给出一条穿墙的路径**。

## 5. 安全设计

| 情况 | 处理 |
| --- | --- |
| 地图过期（`/map` 超时） | 拒绝规划并告警，避免在过期地图上给出穿墙路径 |
| 起点缺失 | 拒绝规划（不拿目标当起点） |
| 目标落在障碍内 | BFS 找到最近自由格，并在日志中说明 |
| 起终点不连通 | 连通性预检直接失败，并指出"分属不同自由区域" |
| 不可采纳启发式 | 允许配置，但基准实验会显示其不再保证最优 |
| 参数非法（NaN / 非正 / 过大） | 启动时校验，非法值回退默认并告警 |
| 地图异常（搜索不收敛） | `max_expansions` 兜底，避免打满 CPU |
| 平滑后路径可能碰撞 | 逐段复核，异常时回退到未经平滑的路径 |
| 地图未知区域 | 按占用处理（保守），不当作可通行区域 |

## 6. 局限与展望

- 8 邻接 A* 在大地图（>200×200）上扩展节点仍偏多，可用 **JPS / 双向 A\*** 改进；
- 膨胀是各向同性的方形结构元，未考虑机器人非圆形外形（如长方体车体的朝向约束）；
- 后处理是几何层面的，没有做**动力学可行性**检查（最小转弯半径）；
- 仅规划单段路径，未实现多段路网与全局重规划；
- 离线仿真使用合成地图，尚未在 Gazebo / CARLA 中接入真实 `/map` 话题验证。

## 7. 许可与署名

MIT License。

作者：**李诚轩（学号 2535030014）**
