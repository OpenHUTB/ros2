# 无人机 EKF 状态估计：IMU、GPS 与气压计融合

## 为什么需要这个模块

[基于 GRU 的状态预测](../air/state_prediction.md) 解决的是“未来状态预测”，
本模块解决的是“当前状态估计”。两者输入都包含飞行状态，但目标不同：

- 状态估计：融合 IMU、GPS、气压计，给出当前位置和速度；
- 状态预测：根据历史状态预测 0.25/0.5/1.0 s 后的位置和速度。

现有 `state_prediction` 已经提供了完整的 GRU 训练和 ROS 2 在线误差分析；本模块
补上更基础的导航滤波环节，便于没有 GPU、只想验证 IMU/GPS 融合的读者直接运行。

## 快速运行

```bash
cd src/air/uav_ekf_state_estimation
python -m pip install -r requirements.txt
python main.py demo
```

本机实测：

```text
ekf_rmse_m=1.7452
dead_reckoning_rmse_m=96.4764
gps_rmse_m=2.6594
ekf_final_error_m=2.0726
complementary_yaw_rmse_deg=1.0004
```

![EKF 状态估计](uav_ekf_state_estimation/demo_screenshot.png)

合成轨迹包含 24–34 s 的水平阵风。EKF 的三维 RMSE 低于纯 GPS，航位推算随时间
明显漂移；这说明 GPS/气压计更新确实在约束 IMU 积分。

## 滤波器结构

状态为 NED 位置和速度共 6 维。IMU 比力通过姿态旋转到 NED，加入重力后积分；
GPS 更新水平位置，气压计更新高度。姿态被视为飞控或 VIO 的输入，包内另外提供了
一个互补滤波器用于演示陀螺仪、加速度计和磁力计融合。

### 坐标系与符号约定

`/uav/imu` 遵循 REP-103：机体系为 FLU（x 前、y 左、z 上），`linear_acceleration`
**含重力**，因此水平静止时读数约为 `(0, 0, +g)`。互补滤波器的倾角估计
`roll = atan2(ay, az)` 正是按这个约定写的。

EKF 采用航空常用的 FRD（x 前、y 右、z 下）：静止时比力应为 `(0, 0, -g)`，
这样 `R·f + g_ned` 才能相互抵消。两种约定只在 `frames.py` 一处交汇：

- `specific_force_flu_to_frd()` 把比力从 FLU 换到 FRD（y、z 取反）；
- `tilt_flu_to_frd()` 把 FLU 倾角换到 FRD（俯仰角取反，水平姿态仍是水平）；
- `step_imu()` 是 `on_imu` 唯一的处理链，姿态滤波拿原始 REP-103 数据，EKF 拿
  转换后的 FRD 量。

这样水平静止输入穿过回调后速度保持为零，不会出现“同一加速度按两套重力符号各算
一遍”的漂移。

这里刻意没有把加速度计零偏放进状态。只有 GPS 位置和气压计高度时，垂直零偏与
重力/姿态误差不可观，强行估计会让零偏吸收误差并导致发散。需要零偏估计时，应
升级到包含姿态、陀螺零偏和加速度计零偏的 15 状态滤波器。

## ROS 2 接口

节点订阅 `/uav/imu`、`/uav/gps`、`/uav/baro`，发布 `/uav/ekf/odom` 和
`/uav/ekf/diagnostics`。GPS 经纬度在节点内投影到局部 ENU；正式使用时建议替换
为地图的 `GeoProjection`，避免大范围飞行时的投影误差。

```bash
colcon build --packages-select uav_ekf_state_estimation
source install/setup.bash
ros2 launch uav_ekf_state_estimation main.launch.py
```

ROS 2 节点在当前 Windows 开发机上未执行；离线核心、12 个单元测试和 PNG 截图
均已实际运行。ROS 2 Humble 的实时验证需要在 Ubuntu 虚拟机中完成。

## 验证

```bash
python -m unittest discover -s tests -v
```

12 个测试全部通过，覆盖 GPS 协方差收敛、气压计更新、悬停预测、姿态滤波、
指标有限性和随机种子可重复性，以及 REP-103(FLU)→FRD 的符号转换与水平静止输入
贯穿 `on_imu` 处理链（`step_imu`）的回归。

## 已知限制

- 合成数据，不是真实飞控日志；
- 姿态作为输入，不估计姿态/陀螺零偏；
- 不模拟 GPS 丢星、磁干扰和气压计漂移；
- 阵风是确定性模型，不是连续湍流；
- ROS 2 节点尚未在 RViz2 中做实时曲线显示。

## 大模型使用声明

代码、测试和文档使用 OpenAI Codex 辅助完成；运行指标与截图来自本机实际执行。
提交者需对内容负责。
