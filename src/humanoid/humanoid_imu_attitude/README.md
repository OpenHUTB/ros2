# humanoid_imu_attitude

人形机器人 IMU 姿态解算对比：同一段带噪声的 IMU 数据分别送入互补滤波与 Madgwick
滤波器，按时间对齐真值后统计 roll/pitch 误差，并给出 yaw 漂移与单步耗时。

详细的原理、实测数据与结论见文档
[docs/humanoid/humanoid_imu_attitude.md](../../../docs/humanoid/humanoid_imu_attitude.md)。

## 依赖

- Ubuntu 20.04 + ROS 2 Foxy
- Python 3.8，NumPy、Matplotlib（ROS 2 基础环境已包含，无需额外 pip 安装）

## 构建与运行

```
# 可选：按 package.xml 安装依赖
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select humanoid_imu_attitude
source install/setup.bash
cd src/humanoid/humanoid_imu_attitude

# 三个场景各 20 s，最后打印指标表并生成 results/effect.png
DURATION=20 ./run_eval.sh
```

单场景：

```
ros2 launch humanoid_imu_attitude main.launch.py \
  profile:=fast duration:=20 out_dir:=results
```

只重新统计已有 CSV：

```
ros2 run humanoid_imu_attitude attitude_eval --results results --figure results/effect.png
```

## 节点与接口

| 节点 / 可执行文件 | 作用 |
|-------------------|------|
| `imu_sim` | 按轨迹合成 IMU 数据，发布 `/imu/data` 与 `/imu/ground_truth`，写真值 CSV |
| `attitude_filter` | 解算姿态，发布 `/imu/attitude/<method>`，写估计 CSV |
| `attitude_eval` | 统计误差、输出 `metrics.csv`、绘制对比图、测单步耗时 |

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `profile` | `slow` | `static` / `slow` / `fast` |
| `duration` | `20.0` | 采集时长（秒），滤波器会多等 0.5 s 再退出 |
| `rate_hz` | `200.0` | IMU 发布频率 |
| `out_dir` | `results` | CSV 与图片输出目录 |
| `alpha` | `0.98` | 互补滤波的陀螺仪权重（与 dt 无关，改 `rate_hz` 后需重新整定） |
| `beta` | `0.05` | Madgwick 梯度下降增益 |

## 结果

三个场景各 20 s、200 Hz（本机实测）：

| 场景 | 方法 | roll RMSE (°) | pitch RMSE (°) | 最大误差 (°) | p95 (°) | yaw 末段漂移 (°) |
|------|------|---------------|----------------|--------------|---------|------------------|
| static | complementary | 0.18 | 0.27 | 1.60 | 0.41 | 5.1 |
| static | madgwick | 0.18 | 0.28 | 1.60 | 0.38 | 5.1 |
| slow | complementary | 0.18 | 0.27 | 1.71 | 0.42 | 4.9 |
| slow | madgwick | 0.18 | 0.29 | 1.71 | 0.39 | 4.9 |
| fast | complementary | 0.28 | 0.31 | 1.92 | 0.53 | 3.2 |
| fast | madgwick | 0.34 | 0.35 | 1.94 | 0.61 | 3.2 |

单步耗时（纯算法，5 万次平均，虚拟机负载不同时抖动较大）：互补滤波约 50 µs，Madgwick 约 16 µs。


## 限制

- 6 轴 IMU 没有磁力计，yaw 不可观测，表中 yaw 一列只是陀螺零偏造成的漂移。
- 数据来自合成 IMU（常值零偏 + 高斯白噪声），不代表真实硬件的温漂与振动。
- 所有节点在同一台机器上运行，未评估真实网络的时钟偏差。

## 测试

```
PYTHONPATH=src/humanoid/humanoid_imu_attitude PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python3 -m pytest src/humanoid/humanoid_imu_attitude/test -q
```

共 14 个用例（18 条断言）：四元数恒等式与欧拉角往返、轨迹解析角速度一致性、两种滤波器在静态倾斜下的收敛性、动态输入下的误差上界。
