# encoder_quantization_velocity

编码器量化对速度估计的影响：1 kHz 差分量化位置会把量化误差放大 1000 倍，
本模块对比原始差分、滑动平均、α-β 跟踪器与两状态卡尔曼在 1024/4096/16384 CPR 下的
速度误差与延迟。

## 环境

- 离线：Windows 11、Python 3.12.14（最低 3.8，未使用 3.10+ 语法）、NumPy 2.3.5、Matplotlib 3.11.2（仅绘图）
- ROS 2：Ubuntu 22.04 + ROS 2 Humble

## 构建与运行（ROS 2）

```bash
source /opt/ros/humble/setup.bash
cd ~/hutb_ws
colcon build --symlink-install --packages-select encoder_quantization_velocity
source install/setup.bash
ros2 launch encoder_quantization_velocity main.launch.py cpr:=4096 duration_s:=120
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/joint/encoder` | `sensor_msgs/JointState` | 输出（仿真） | 量化后的关节位置 |
| `/joint/velocity_raw` | `std_msgs/Float64` | 输出（估计） | 原始差分速度 |
| `/joint/velocity` | `std_msgs/Float64` | 输出（估计） | 卡尔曼速度 |
| `/joint/velocity_status` | `std_msgs/String` | 输出（估计） | 三种估计与样本数（JSON） |

## 离线实验

```bash
cd src/ground/encoder_quantization_velocity
python3 main.py --out results --figure-dir ../../../docs/img/ground/encoder_quantization_velocity
python3 -m unittest discover -s tests -v
```

## 主要结果（梯形速度曲线，1 kHz）

| CPR | 原始差分 | 滑动平均 | α-β | 卡尔曼 |
|---|---|---|---|---|
| 1024 | 1.371 | 0.339 | **0.198** | 0.242 |
| 4096 | 0.404 | 0.114 | **0.080** | 0.110 |
| 16384 | 0.133 | **0.047** | 0.068 | **0.047** |

（单位 rad/s；延迟分别为 0 / 2 / 3 / 2–12 ms）

表中 CPR 指**四倍频（quadrature ×4）解码后的计数/转**，不是编码器线数：若按线数选型，1024 线对应 4096 CPR、4096 线对应 16384 CPR。量化步长 Δ = 2π/CPR，与 `EncoderSpec.step_rad` 一致。

详见 `docs/ground/encoder_quantization_velocity.md`。
