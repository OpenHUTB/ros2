# uav_airsim_ttc_predictor — AirSim 无人机碰撞时间预测

从单帧前视深度图预测「还有多少秒会撞上最近的障碍」（Time-To-Collision）。网络不看速度、不看位姿，只看一张 64×64 的深度图。

对应课程任务 ②「感知」中的安全预警环节。

## 演示

`docs/img/uav/ttc/demo.gif`（真实 AirSim 画面，102 帧）

无人机以 1.4 m/s 从 14 m 外向方块墙飞去，网络预测的 TTC 从 5.5 s 一路降到 1.6 s，触发安全停。

## 效果

| 指标 | 数值 |
|---|---|
| 样本量 | 612（10 回合 × 70 步） |
| 输入 | 前视深度 64×64（单通道，按 30 m 归一化） |
| 输出 | TTC（秒，0~20 归一化） |
| 验证 MSE | 0.00442 |
| 验证 R² | **0.606** |
| 实飞预测 MAE | **1.36 s** |

## 结构

| 文件 | 作用 |
|---|---|
| `env.py` | AirSim 连接、深度图取用与 64×64 双线性缩放、世界系前进指令 |
| `cnn.py` | 手写卷积网络：Conv(1→8, 5×5, stride2) → ReLU → MaxPool(2) → FC(1800→64) → FC(64→1) |
| `dataset.py` | 采集深度图与 TTC 真值（最近距离 / 前进速度） |
| `train.py` | 逐样本 SGD + Adam，按验证 MSE 留最优 |
| `demo.py` | 飞向墙的实飞演示、安全停逻辑、真实画面 gif |
| `node.py` | ROS 2 节点，订阅 `/uav/depth`，发布 `/uav/ttc` |
| `launch/main.launch.py` | launch 启动 |

## 用法

```bash
python3 main.py --collect --episodes 10 --steps 70
python3 main.py --train --epochs 40
python3 main.py --demo --steps 110 --gif demo.gif --speed 1.4
```

ROS 2：

```bash
colcon build --packages-select uav_airsim_ttc_predictor
source install/setup.bash
ros2 launch uav_airsim_ttc_predictor main.launch.py
```

## 前置条件

- AirSim 1.8.1 Blocks 场景，`SimMode: Multirotor`，机型 `SimpleFlight`
- `CameraDefaults` 配 640×480 的 DepthPlanar
- RPC 端口 41451 处于 `LISTEN`

## 已知局限

1. TTC 真值依赖前进速度，而 AirSim 的估计速度在指令结束后会归零，位置差分噪声又大，因此真值用「指令速度与差分速度各占一半」的折中，标签本身带有约 0.5 m/s 的不确定度。
2. 网络只看单帧深度，无法区分「自己正在飞」还是「静止而墙在动」，慢速时有系统性高估。
3. 实飞 MAE 1.36 s，在 TTC 尺度（约 4 s）上误差明显，只适合做粗粒度预警，不适用于精确制动。
4. 安全停会在预测偏保守时提前触发（本次演示在距离 4.2 m 就停了）。
