# mujoco_nn_latency_robustness —— 神经网络跟踪策略的执行延迟鲁棒性

MuJoCo 赛道上的对比实验：**同一份权重、同一条赛道，只改「指令从算出来到落到车上有多少毫秒」**，
看端到端策略怎么退化，以及三种延迟处理能把损失挽回多少。

三件事：

1. **延迟扫描**：0 / 200 / 400 / 600 / 800 / 1000 ms 执行延迟，三种**可上车**的补偿方式
   外加一个 oracle 上界（用仿真器直接取预测位姿的扫描）；
2. **频率扫描**：10 / 20 / 50 Hz 控制频率（延迟固定 600 ms）；
3. **三条赛道**：半径 3 m 的圆、肾形、花生形，全部用代码生成场景，不依赖外部资源。

## 为什么单独做这个模块

仓库里已有不少「同一算法不同实现」的对比（`lateral_control_benchmark`、`uav_rl_benchmark`），
但**没有人把「时延」当成自变量**。而端到端系统里最常被忽视的失效模式恰恰是它：
模型精度、算力都达标，上车以后因为推理 + 通信 + 执行器排队多了 300 ms，就贴墙了。

本模块给出的结论是：

- 20 Hz、600 ms 延迟下，不加处理策略横着撞满整圈，800 ms 直接跑不完；
- 匀速外推（把当前扫描按位姿差平移）能把 600 ms 的误差压到三分之一，但 1000 ms 时同样失效；
- **模型前向（Smith 预测 + 扫描变换）**在 600～800 ms 仍能完成，1000 ms 时也开始擦墙；
- 只有 **oracle**（允许直接向"地图"要预测位姿处的完整扫描）能在 1000 ms 下保持精度——
  这条虚线标出了"有图 vs 无图"的差距，也是本模块对真实系统的诚实边界。

## 环境

| 组件 | 版本 |
| --- | --- |
| 操作系统 | Windows 11 / Ubuntu 20.04+ |
| Python | 3.10 |
| MuJoCo | 3.17.0（课程虚拟机里 3.1.6 同样通过） |
| numpy / matplotlib / pillow | 2.2 / 3.10 / 11.x |
| ROS 2 | Humble（`colcon build` + `ros2 launch` 已验证） |

## 快速开始

```bash
cd src/ground/mujoco_nn_latency_robustness
pip install -r requirements.txt
python main.py --demo      # 训练 + 延迟矩阵 + 频率矩阵 + 出图 + 出 GIF（纯 CPU，约 6 min）
python main.py --test      # 38 条单测
python main.py --sweep     # 用已有权重重跑实验矩阵（会自己建 results/）
python main.py --plots     # 只重画图与 GIF，不动训练
```

产物在 `results/`：`delay_sweep.png`、`rate_sweep.png`、`traces.png`、
`track_none.gif`、`track_model.gif`、`metrics.json`；训练好的权重在 `models/policy.npz`。

ROS 2（Humble）：

```bash
colcon build --packages-select mujoco_nn_latency_robustness
source install/setup.bash
ros2 launch mujoco_nn_latency_robustness main.launch.py
ros2 launch mujoco_nn_latency_robustness main.launch.py delay_ms:=800.0 comp:=none
ros2 topic echo /car/status
```

话题：`/car/scan`（LaserScan，24 线）、`/car/odom`、`/car/status`、`/tf`（`map → base_link`）。

## 目录

```
mujoco_nn_latency_robustness/
├── lat_nav/
│   ├── track.py     三条赛道的中心线、边界与 MJCF 生成
│   ├── env.py       MuJoCo 环境：速度伺服小车 + 24 线雷达（mj_ray）
│   ├── net.py       26→64→32→2 MLP，numpy 手写前反向 + Adam
│   ├── nav.py       纯追踪专家、延迟线、三种补偿、闭环统计
│   ├── train.py     专家采样 → 模仿 → DAgger
│   ├── sweep.py     实验矩阵与绘图
│   └── node.py      ROS 2 节点
├── launch/main.launch.py, config/params.yaml
├── models/policy.npz
├── main.py|.sh|.bat, test/, README.md
```

## 结果

见文档 [`docs/ground/mujoco_nn_latency_robustness.md`](../../../docs/ground/mujoco_nn_latency_robustness.md)。
