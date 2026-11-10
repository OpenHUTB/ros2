# uav_weather_risk

低空多源气象扰动下的小型飞行器影响预测与风险分级演示。模块把三段链路放在同一个 ROS 2 图中：

```text
NWP + 地面风塔 + 机载风估计
        ↓ 逆方差融合
平均风 + 阵风/湍流随机过程
        ↓ 降阶飞行器响应
10 s 蒙特卡洛轨迹与姿态响应
        ↓
NOMINAL / CAUTION / WARNING / CRITICAL
```

完整说明、模型公式、实测结果与限制见站点文档：
`docs/air/uav_weather_risk.md`。

## 快速开始

环境：Ubuntu 22.04、ROS 2 Humble、Python 3.10。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/air/uav_weather_risk --ignore-src -r -y
colcon build --symlink-install --packages-select uav_weather_risk
source install/setup.bash
ros2 launch uav_weather_risk main.launch.py
```

查看风险与可视化标记：

```bash
ros2 topic echo /uav_weather_risk/assessment
ros2 topic echo /uav_weather_risk/markers
```

RViz2 中添加 `MarkerArray`，话题选择 `/uav_weather_risk/markers`，固定坐标系设为 `map`。节点只输出风险建议，不发送任何飞控指令。

## 离线复现

不安装 ROS 2 也可以运行同一套核心模型并生成 CSV、PNG 和 GIF：

```bash
python3 -m pip install matplotlib
python3 main.py --duration 60 --seed 42 --output-dir artifacts/demo
```

## 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/uav_weather_risk/odom` | `nav_msgs/Odometry` | 输入 | ENU 位置、速度 |
| `/uav_weather_risk/weather` | `std_msgs/String` | 输入 | 融合风场 JSON |
| `/uav_weather_risk/target` | `geometry_msgs/PointStamped` | 输入 | 当前航点 |
| `/uav_weather_risk/assessment` | `std_msgs/String` | 输出 | 四级风险、概率、原因 |
| `/uav_weather_risk/markers` | `visualization_msgs/MarkerArray` | 输出 | 预测轨迹、风险球、风矢量 |

## 边界

本模块是课程/研究用降阶代理模型，不是气象预报产品或适航认证工具。风险分数综合当前机动需求与气象扰动，不能单独解释为天气因果归因。实机运行前必须由持证人员独立评估。
