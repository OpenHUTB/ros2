# uwb_range_odom_fusion

UWB 测距与轮式里程计的平面融合定位，ROS 2（Humble）包。模块包含四个可对照的估计器：

| 估计器 | 输入 | 说明 |
|---|---|---|
| `odometry_only` | 轮式里程计 | 漂移基线 |
| `trilateration` | 4 路基站测距 + 里程计外推 | 逐帧加权最小二乘（Huber 降权），无滤波、无门限；解算失败时沿用里程计结果，是混合估计器 |
| `ekf_ungated` | 里程计 + 测距 | EKF，接受全部测距 |
| `ekf_gated` | 里程计 + 测距 | EKF + 卡方新息门限，剔除 NLOS |

四个估计器共用同一份确定性仿真数据（同 seed 完全一致），因此表格里的差值是算法差异，不是随机波动。

## 环境

- 实测（离线部分）：Windows 11、Python 3.12.14、NumPy 2.3.5、Matplotlib 3.11.2（只用于画图）
- ROS 2 部分：Ubuntu 22.04 + ROS 2 Humble（`python3-numpy`）
- 核心算法只依赖 NumPy；不装 ROS 也能跑完整实验

## 构建与运行（ROS 2）

```bash
source /opt/ros/humble/setup.bash
cd ~/hutb_ws
colcon build --symlink-install --packages-select uwb_range_odom_fusion
source install/setup.bash
ros2 launch uwb_range_odom_fusion main.launch.py duration:=60
```

`duration:=0` 表示循环播放；加 `rviz:=true` 会启动预置视图。

运行话题：

```bash
ros2 topic list | grep -E "uwb|odom"
ros2 topic echo --once /uwb/pose
ros2 topic echo /uwb/status
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/ground_truth/pose` | `geometry_msgs/PoseStamped` | 输出（仿真） | 真值位姿 |
| `/odom` | `nav_msgs/Odometry` | 输出（仿真） | 带标定误差的轮式里程计 |
| `/uwb/anchor_<i>/range` | `sensor_msgs/Range` | 输出（仿真） | 第 i 个基站的距离，10 Hz |
| `/uwb/pose` | `geometry_msgs/PoseWithCovarianceStamped` | 输出（融合） | 融合位姿与 3×3 协方差 |
| `/uwb/status` | `std_msgs/String` | 输出（融合） | 位置、σ、接受/剔除计数（JSON） |
| `/uwb/markers` | `visualization_msgs/MarkerArray` | 输出（融合） | RViz 中的基站、估计与计数 |

参数见 `config/demo.yaml`：`anchor_x` / `anchor_y`（基站坐标，等长数组）、`range_sigma`、`gate_chi2`、`use_gating`、`process_sigma_v`、`process_sigma_omega`、`output_csv`。

## 离线实验（不需要 ROS）

```bash
cd src/ground/uwb_range_odom_fusion
python3 -m pip install -r requirements.txt      # numpy，可选 matplotlib
python3 main.py --seeds 10 --duration 70 --out results
```

输出 `results/summary.json`（聚合表 + 每种子的明细 + 几何实验）与 `results/timeseries.csv`。
复现文档中的图需要加 `--figure-dir`，不想装 matplotlib 时用 `--no-plots`。

## 测试

```bash
python3 -m unittest discover -s tests -v
```

33 项测试只依赖 NumPy：几何、里程计标定、EKF 预测/门限、场景确定性、指标与端到端对比。

## 说明

- 数值结果由 `results/summary.json` 生成，文档表格与之一一对应；重新运行会覆盖 `generated_utc`，其余数值不变。
- 本模块不控制真实机器人，只输出位姿估计与协方差；基站坐标需要事先标定。
- 详见 `docs/ground/uwb_range_odom_fusion.md`（方法、公式、实测与限制）。
