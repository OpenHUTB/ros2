# multi_uav_airspace_monitor

面向多无人机低空任务的**多边形禁飞区 + 机间冲突**监测模块。它补上了现有
`no_fly_zone_check` 只检查单个经纬度点、只支持圆形禁飞区的不足，把输入扩展为
多架无人机的时间序列轨迹，并同时输出电子围栏告警和两两最近接近距离（CPA）。

## 与已有模块的关系

| 模块 | 关注点 | 本模块的区别 |
| --- | --- | --- |
| `no_fly_zone_check` | 单个点是否落在圆形禁飞区 | 支持多边形、高度层、缓冲区，以及多机轨迹 |
| `carlair_ros_bridge` | AirSim/CarlaAir 与 ROS 的坐标和话题桥接 | 不连接仿真器，只消费已经统一到 ENU 的轨迹 |
| `ap_cpp_overfomo` | 覆盖路径规划 | 不规划航迹，只对已有航迹做安全评估 |

这个模块可以独立运行，也可以作为规划器/控制器的安全监视层。核心代码不依赖
ROS 2、AirSim 或 GPU；ROS 2 节点只是可选外壳。

## 功能

- GeoJSON/JSON 多边形禁飞区，支持 `floor_z`、`ceiling_z` 和水平缓冲区。
- 多架无人机的 ENU 轨迹输入，逐点检查电子围栏。
- 两两无人机最近接近距离、冲突时刻、告警等级（时间窗内取最小距离）。
- 确定性冲突解脱建议（爬升/减速），便于后续接入规划器。
- 输出 JSON 摘要和 PNG 俯视图，适合直接放入 PR 的运行效果。

## 运行环境

- Python 3.10+
- `numpy`
- `Pillow`

离线演示不需要 ROS 2、AirSim、Unreal 或 GPU。

```bash
python -m pip install -r requirements.txt
```

## 快速开始

在模块目录执行：

```bash
cd src/air/multi_uav_airspace_monitor
python main.py demo
```

本机 Windows 11 + Python 3.12 的实际输出：

```text
==> multi_uav_airspace_monitor
    status=critical
    uavs=4 zones=1
    minimum_separation_m=0.0
    alerts=3 conflicts=1
    conflict uav_alpha vs uav_beta: 0.00 m at 20.00 s
    screenshot: results/demo_screenshot.png
```

运行效果：

![multi-UAV airspace monitor](results/demo_screenshot.png)

图中两条对角航迹在中心交叉，圆形标记为机间冲突；红色多边形为禁飞区，
`uav_gamma` 进入禁飞区，`uav_alpha` 与 `uav_beta` 在 20 s 时最近距离为 0 m。

## 测试

```bash
python -m unittest discover -s tests -v
```

当前 8 个测试覆盖点在多边形内/外/边界、点到边距离、经纬度往返、禁飞区高度层、
迎头冲突、平行安全航迹和演示场景。

## ROS 2 节点

节点订阅 JSON 状态消息，而不是绑定某一种仿真器：

```text
/airspace/uav_states  std_msgs/String
{
  "stamp": 12.5,
  "uavs": [
    {"id": "uav_alpha", "x": 1.0, "y": 2.0, "z": 30.0}
  ]
}
```

输出 `/airspace/alerts`，内容与 `summary.json` 相同，额外带 `stamp`。

```bash
colcon build --packages-select multi_uav_airspace_monitor
source install/setup.bash
ros2 launch multi_uav_airspace_monitor main.launch.py
```

`launch/main.launch.py` 默认不加载禁飞区；实际使用时把 `zones_json` 参数替换为
场景中的 zone 列表。ROS 2 节点只做消息适配，核心算法仍由 `monitor.py` 提供，
因此可以在没有 ROS 的机器上完整测试。

## 输入格式

`main.py check --config scenario.json` 接受如下结构：

```json
{
  "horizon_s": 20.0,
  "warning_distance_m": 30.0,
  "critical_distance_m": 10.0,
  "zones": [
    {
      "name": "city_center_restricted",
      "polygon": [[20, -30], [90, -30], [90, 60], [20, 60]],
      "floor_z": 0,
      "ceiling_z": 120,
      "buffer_m": 20
    }
  ],
  "uavs": [
    {
      "id": "uav_alpha",
      "times": [0, 1, 2],
      "positions": [[0, 0, 60], [10, 0, 60], [20, 0, 60]]
    }
  ]
}
```

坐标统一为局部 ENU：`x` 东、`y` 北、`z` 上，单位米。

## 已知限制

- 轨迹在采样点之间采用线性插值；高速强机动场景应提高采样率。
- 冲突检测是离线/短时域 CPA：先截取公共起点起的 `horizon_s` 秒，再取窗口内最小距离，
  所以窗口外更近的接近不会掩盖窗口内的告警；整体仍不是完整的 UTM 航路调度器。
- 解脱建议是确定性规则，尚未接入速度障碍、MPC 或强化学习策略。
- 禁飞区是垂直棱柱，不支持任意三维网格；复杂地形可先做分层 zone。
- 当前尚未在 ROS 2 Humble 的 RViz 中做可视化，节点只发布 JSON 告警。

## 大模型使用声明

本模块的代码、测试和文档在 OpenAI Codex 辅助下完成；运行结果与截图来自本机
Windows 11 + Python 3.12 的实际执行。提交者需复核代码并对提交内容负责。
