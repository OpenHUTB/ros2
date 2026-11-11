# uav_airsim_wind_rejection — AirSim 无人机阵风扰动抑制

在 AirSim Blocks 场景里给无人机注入强侧风，用神经网络根据 IMU 与位置误差实时输出补偿速度，把无人机按在原地。

对应课程任务 ①「物理仿真控制」：外部扰动下的控制与补偿。

## 演示

`docs/img/uav/wind_reject/demo.gif`（真实 AirSim 画面，150 帧）

## 效果

同一股 15 m/s 侧风下：

| 策略 | 平均偏移 | 最大偏移 |
|---|---|---|
| 不补偿（AirSim 自身速度环） | 14.68 m | 22.80 m |
| 神经网络补偿 | **3.42 m** | **5.00 m** |

平均偏移降低约 77%。

## 结构

| 文件 | 作用 |
|---|---|
| `env.py` | AirSim 连接、IMU 采样与 5 帧滑窗、`simSetWind` 注入、世界系速度指令 |
| `policy.py` | PD 位置保持 + 风速前馈的专家策略 |
| `mlp.py` | 纯 NumPy 两层 MLP + Adam |
| `dataset.py` | 6 回合 × 100 步样本采集，按通道标准化 |
| `train.py` | 训练、验证集 R²、L2 正则 |
| `demo.py` | 有风实飞演示、无补偿对照、真实画面 gif |
| `node.py` | ROS 2 节点，订阅 `/uav/imu` 与 `/uav/pose`，发布 `/uav/wind_cmd` |
| `launch/main.launch.py` | launch 启动 |

## 用法

```bash
python3 main.py --collect --episodes 6 --steps 100
python3 main.py --train --epochs 120
python3 main.py --demo --steps 150 --gif demo.gif --wind_x 15 --wind_y -5
python3 main.py --baseline --steps 120 --wind_x 15 --wind_y -5
```

ROS 2：

```bash
colcon build --packages-select uav_airsim_wind_rejection
source install/setup.bash
ros2 launch uav_airsim_wind_rejection main.launch.py
```

## 前置条件

- AirSim 1.8.1 Blocks 场景，`SimMode: Multirotor`，机型 `SimpleFlight`
- `settings.json` 中 `CameraDefaults` 配 640×480 Scene 相机
- RPC 端口 41451 处于 `LISTEN` 状态

## 已知局限

1. SimpleFlight 内置速度环本身抗风很强，`simSetWind` 的设定值与实际扰动量差一个数量级：20 m/s 风只产生约 1.6 m/s 等效漂移。因此前馈系数按实测标定为 0.08，而非理想模型的 1.0。
2. 网络输入含位置误差，所以它主要学到的是反馈映射（R² 0.997），风的隐式辨识只占输出的一小部分。
3. 演示末段仍残留约 4 m 偏移，补偿强度略小于扰动。
