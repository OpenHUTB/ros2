# ROS 2 A* 全局路径平滑（Catmull-Rom，带安全校验）

A\* 路径是栅格折线，直接给差速底盘跟踪会有角速度跳变。本模块做后处理：
视线简化 + Catmull-Rom，并在**逐段碰撞检查**、**精确间隙**约束下选择最大曲率
最低的安全候选。节点依赖 `/map`，校验不过则退回输入路径。

## 1. 方法

### 1.1 A\* 与逐段视线简化

* 8 邻域 A\*，禁止斜穿障碍角点；
* `segment_free()` 以 0.25 格步长采样整条线段，并额外检查斜向移动的两个
  正交邻格，避免简化直线斜穿对角障碍；
* 从当前点连到最远可见点，压掉栅格锯齿。

### 1.2 Catmull-Rom 与候选选择

控制点按 0.5 m **弧长**重采样后插值：

$$
q(t)=\tfrac12\left(2p_1+(-p_0+p_2)t+(2p_0-5p_1+4p_2-p_3)t^2+(-p_0+3p_1-3p_2+p_3)t^3\right)
$$

候选包括简化路径和 α=1/0.75/0.5/0.25 的“平滑—原路径”混合：

* 原路径按**弧长**重采样到与平滑路径同样的点数，避免按索引错位；
* 每个候选做 `path_free()`（逐段）和 `clearance()`；
* 优先选满足 `min_clearance` 的候选，再取最大曲率最小者。

### 1.3 精确间隙

`clearance()` 计算点到**障碍格矩形**的距离：

$$
d=\sqrt{\max(|p_x-c_x|-\Delta/2,0)^2+\max(|p_y-c_y|-\Delta/2,0)^2}
$$

比格心距离保守，不会把 0.10 m 报成实际 0.05 m。

### 1.4 回退

没有候选满足间隙时，返回仍通过 `path_free()` 的简化/原始路径并置
`safe=False`；节点收到 `safe=False` 时不发布平滑结果，原样转发输入。

## 文件结构

```text
src/ground/ros2_path_smoothing/
├── ros2_path_smoothing/  # planner.py / sim.py / main_smooth.py
├── config/smoothing_params.yaml
├── launch/main_launch.py
├── results/benchmark.json
├── test/                 # 7 项单元测试
└── README.md
```

## 2. 运行

### 运行环境

| 项目 | 版本 |
| --- | --- |
| ROS 2 | Ubuntu 20.04.6 + Foxy 0.9.2 |
| Python | 3.8.10 |
| 离线基准 | Windows 11 + Python 3.12 + NumPy 2.x |

```bash
cd ~/hutb_ws
colcon build --packages-select ros2_path_smoothing
source install/setup.bash
ros2 launch ros2_path_smoothing main_launch.py
```

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/plan` | `nav_msgs/Path` |
| 订阅 | `/map` | `nav_msgs/OccupancyGrid`（transient local） |
| 发布 | `/plan_smoothed` | `nav_msgs/Path` |

节点先等 `/map`；没有地图时原样转发输入并告警。参数在
`config/smoothing_params.yaml`：`map_topic`、`min_clearance`。

## 参数说明

| 参数 | 含义 | 默认 |
| --- | --- | --- |
| `input_topic` / `output_topic` | 输入/输出路径话题 | `/plan` / `/plan_smoothed` |
| `map_topic` | 碰撞/间隙检查用地图 | `/map` |
| `min_clearance` | 最小安全间隙 | 0.10 m |

## 3. 离线复现

```bash
cd src/ground/ros2_path_smoothing
python3 -m unittest discover -s test -v   # 7 项
python3 -m ros2_path_smoothing.sim
```

## 4. 结果

| 地图 | 原始长度 | 平滑长度 | 原始最大曲率 | 平滑最大曲率 | 原始间隙 | 平滑间隙 | 安全 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| corridor | 19.28 m | 18.76 m | 5.685 1/m | **0.016 1/m** | 0.05 m | 0.15 m | 是 |
| doorway | 21.19 m | 19.64 m | 5.685 1/m | **0 1/m** | 0.07 m | 0.45 m | 是 |
| clutter | 22.01 m | 20.51 m | 5.685 1/m | **0.014 1/m** | 0.05 m | 0.45 m | 是 |

doorway 的起终点连线正好穿过门洞，简化后就是直线；corridor/clutter 的简化
直线段也是低曲率候选，最终按“满足间隙 + 最小曲率”选中。

> 指标在 Windows 11 + Python 3.12 + NumPy 2.x 上离线复现；ROS 包在
> Ubuntu 20.04 + Foxy 下构建。`results/benchmark.json` 的 `environment`
> 记录的是离线环境，不是 ROS 构建环境。

## 运行效果

corridor 最大曲率 5.685 → **0.016 1/m**、间隙 0.15 m；clutter 5.685 → **0.014 1/m**、
间隙 0.45 m；doorway 直线通过、曲率 0：

![A* 与平滑路径对比](../img/ground/ros2_path_smoothing/path_compare.png)

## 5. 与控制器/规划器的关系

* 输入任意 `nav_msgs/Path`，可接 #1077 的 A\* 或 Nav2；
* 输出仍是 `nav_msgs/Path`，下游可用 Pure Pursuit/Stanley 跟踪；
* 节点需要 `/map` 才能做碰撞/间隙校验；无地图时只原样转发。

## 6. 局限

* 只处理几何路径，不重新规划；
* 极端狭窄场景可能退回简化路径，曲率仍低于原始折线，但平滑收益变小；
* 节点依赖 `/map`，无地图时不做平滑；
* Catmull-Rom 在急弯处会切角，靠逐段检查和精确间隙兜底；
* 2D 路径，不处理高度/坡度。

## 大模型使用声明

代码与文档在编写过程中使用了 AI 大模型辅助。作者已实际运行 7 项单元测试和
3 张离线地图，并在 Ubuntu 20.04 + Foxy 下完成 `colcon build`；对提交内容的
正确性与完整性负全部责任。
