# survey_mission_planner

面向无人机倾斜摄影/正射航测的**飞行前任务规划器**。输入相机参数、目标 GSD、
航向/旁向重叠度和测区多边形，输出航高、航带间距、拍照间距、航点、预计航程/时间
以及 CSV、KML、DJI WPML 文件。

## 与已有模块的关系

| 模块 | 关注点 | 本模块的区别 |
| --- | --- | --- |
| `ap_cpp_overfomo` | 主动感知覆盖路径规划 | 本模块是飞行前航测参数与航带规划，不做信息增益重规划 |
| `uav_waypoint_tracking` | PID 跟踪已有航点 | 本模块只生成航点，不控制无人机 |
| `no_fly_zone_check` | 单点禁飞区检查 | 本模块不替代空域安全校验，规划后仍应做禁飞区检查 |

它适合“给定测区和相机，明天要飞一次航测”的场景，也适合把真实无人机采集的
航迹/照片与仿真中的数字孪生资产对应起来。

## 功能

- 由目标 GSD 反算航高，输出实际 GSD 和地面覆盖范围。
- 按航向/旁向重叠度计算航带间距和拍照间距。
- 主方向对齐后的往复式（boustrophedon）航带，支持凹多边形分段。
- 采样覆盖率检查、航程/时间估计和 120 m 高度/续航警告。
- 导出 CSV、KML 和最小 DJI WPML 示例。
- 输出 PNG 规划图和 JSON 指标。

## 运行环境

- Python 3.10+
- `numpy`
- `Pillow`

离线规划不需要 ROS 2、AirSim、Unreal 或 GPU。

```bash
python -m pip install -r requirements.txt
```

## 快速开始

```bash
cd src/air/survey_mission_planner
python main.py demo
```

本机 Windows 11 + Python 3.12 的实际输出：

```text
==> survey_mission_planner
    altitude_m=119.3891
    gsd_m=0.012
    footprint_width_m=65.664
    footprint_height_m=43.776
    line_spacing_m=19.6992
    shot_spacing_m=8.7552
    line_count=10
    image_count=378
    flight_distance_m=3456.358
    flight_time_s=450.045
    coverage_ratio=1.0
    waypoint_count=20
```

运行效果：

![survey mission plan](results/demo_screenshot.png)

示例使用 1 英寸传感器、24 mm 镜头、5472×3648 图像，目标 GSD 为 1.2 cm/px，
对应航高约 119.4 m；旁向重叠 70%、航向重叠 80% 时，航带间距约 19.7 m，
拍照间距约 8.8 m。测区面积约 6.5 万 m²，规划 10 条航带、20 个航点、378 张照片。

## 输出文件

`python main.py demo` 会在 `results/` 下生成：

| 文件 | 内容 |
| --- | --- |
| `demo_screenshot.png` | 测区、航带、航点和指标图 |
| `plan.json` | 航高、GSD、间距、航程、时间、覆盖率、警告 |
| `survey_waypoints.csv` | 序号、ENU 坐标、经纬度、航高 |
| `survey_mission.kml` | 航迹线，可在 Google Earth/QGIS 查看 |
| `survey_mission.wpml` | 最小 DJI WPML 示例，需按具体机型/Pilot 版本校验 |

## 参数与公式

目标 GSD 反算航高：

$$
H = \frac{\mathrm{GSD} \cdot f \cdot W_{px}}{W_{sensor}}
$$

地面覆盖宽高：

$$
W_{ground} = \frac{H \cdot W_{sensor}}{f}, \quad
H_{ground} = \frac{H \cdot H_{sensor}}{f}
$$

航带间距 = $W_{ground}(1 - \text{side overlap})$；拍照间距 =
$H_{ground}(1 - \text{forward overlap})$。

参数在 `config/demo_mission.json` 中，换成自己的测区多边形和相机参数即可。
坐标使用局部 ENU（东、北，米），`origin_lat/origin_lon` 用于导出经纬度。

## 测试

```bash
python -m unittest discover -s tests -v
```

8 个测试覆盖 GSD/航高互算、覆盖范围、航带/拍照间距、覆盖率、航点位于测区内、
确定性，以及 CSV/KML/WPML 可解析。

## ROS 2 节点

节点订阅 `/survey/plan_request`（`std_msgs/String`，JSON，字段与
`config/demo_mission.json` 相同），发布：

- `/survey/path`：`nav_msgs/Path`，ENU 航点；
- `/survey/summary`：`std_msgs/String`，规划指标 JSON。

```bash
colcon build --packages-select survey_mission_planner
source install/setup.bash
ros2 launch survey_mission_planner main.launch.py
```

ROS 2 节点在当前 Windows 开发机上未执行；离线规划、8 个单元测试和三种导出格式
均已实际运行。ROS 2 Humble 的启动验证需要在 Ubuntu 虚拟机中完成。

## 已知限制

- 测区按平面处理，不做地形跟随；山地应使用 DEM/DSM 生成分层航高；
- 不考虑风、电池电压曲线和真实爬升/转弯动力学；
- 凹多边形会生成多个航段，但不会做全局最优旅行商优化；
- WPML 是最小示例，DJI Pilot 版本/机型字段可能需要按官方 schema 调整；
- 尚未用真实无人机执行 WPML 文件；正式飞行前必须在地面站中检查航高、
  返航高度、禁飞区和电池余量。

## 大模型使用声明

代码、测试和文档使用 OpenAI Codex 辅助完成；运行指标与截图来自本机实际执行。
提交者需复核代码并对提交内容负责。
