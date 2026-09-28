# carla_perception_control —— CARLA 传感器感知与给定轨迹跟踪（神经网络版）

> 机器人操作系统及应用 · 课程作业二
>
> 用 CARLA 0.9.16 的**三路传感器**（RGB 相机 / 深度相机 / 激光雷达）做感知，
> 并沿**给定轨迹**对车辆做运动控制。**感知与控制算法均为神经网络**
> （纯 numpy 实现，含前向传播与反向传播，无需 TensorFlow/PyTorch）。

## 1. 功能

| 环节 | 输入 | 神经网络 | 输出 |
|---|---|---|---|
| 感知 | 三路传感器 → 4 维归一化特征 | MLP 分类器 `[4, 12, 3]` | 障碍方向：无目标 / 偏左 / 偏右 |
| 控制 | 状态 (航向差, 前视距离) | MLP 策略网络 `[2, 32, 1]` | 转向角 `steer ∈ [-1,1]` |

## 2. 目录结构

```
src/ground/carla_perception_control/
├── main.py                                   # 主入口（train / run / headless 三种模式）
├── main.sh / main.bat                        # 课程约定的 main.* 一键运行脚本
├── carla_perception_control/
│   ├── __init__.py
│   ├── nn_models.py                          # 纯 numpy 神经网络库
│   ├── carla_common.py                       # CARLA Python API 封装
│   └── perception_control_node.py            # ROS 2 节点
├── launch/
│   ├── main.launch.py                        # ROS 2 Humble
│   └── main.launch                           # ROS 1 Noetic
├── config/sim_params.yaml                    # 仿真参数
├── resource/carla_perception_control
├── setup.py / setup.cfg / package.xml
├── requirements.txt
└── test/test_perception_logic.py             # 单元测试（9 项，无需 CARLA）
```

## 3. 快速开始

```bash
# ① 离线训练两个神经网络（不需要 CARLA）
python3 main.py --mode train --epochs 300 --out models/nn_percept.json
#    预期：感知 NN 准确率 ≈ 0.995，控制 NN MSE ≈ 0.003

# ② 离线取证（不需要 CARLA，也不需要图形界面）
python3 main.py --headless --demo --epochs 300 --sim_time 60 --save_dir ~/shots
#    导出 4 张曲线图：感知损失 / 控制损失 / 横向误差 / 转向指令

# ③ 在线感知 + 沿给定轨迹跟踪（需要 CARLA 服务端）
python3 main.py --mode run --host <宿主机IP> --model models/nn_percept.json \
        --waypoints "40,-8 40,12 25,20" --sim_time 30

# ④ ROS 2 / ROS 1
ros2 launch carla_perception_control main.launch.py host:=<宿主机IP>
roslaunch carla_perception_control main.launch host:=<宿主机IP>
```

一键脚本：`bash main.sh --host <宿主机IP>` 或 Windows `main.bat --mode train`。

## 4. 测试

```bash
python3 test/test_perception_logic.py     # 9 项全部通过，无需 CARLA
```

## 5. 实测指标（离线取证模式）

| 指标 | 数值 |
|---|---|
| 感知 NN 训练集准确率 | 0.995 |
| 控制 NN 训练集 MSE | 0.00303 |
| 横向误差 RMSE | 0.231 m |
| 平均速度 | 6.96 m/s |

## 6. 与已有示例的关系

本模块是 [`set_up_and_connect_to_carla`](https://openhutb.github.io/ros2/set_up_and_connect_to_carla/)
与「路径点发布器」示例的**拓展**：已有示例依赖 `carla_ros_bridge`，本模块改为
CARLA Python API 直连，并把感知与控制都换成神经网络。
**CARLA 服务端启动、宿主机 IP 与端口查看等通用配置步骤不在此重复**，
详见上述已有示例与
[模块文档](https://openhutb.github.io/ros2/ground/carla_perception_control/)。
