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
├── check_connection.py                       # CARLA 连通性分步诊断（连不上时先跑它）
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
└── test/test_perception_logic.py             # 单元测试（20 项，无需 CARLA 即可跑）
```

## 3. 快速开始

```bash
# ① 离线训练两个神经网络（不需要 CARLA）
python3 main.py --mode train --epochs 300 --out models/nn_percept.json
#    预期：感知 NN 准确率 ≈ 0.995，控制 NN MSE ≈ 0.003

# ② 离线取证（不需要 CARLA，也不需要图形界面）
python3 main.py --headless --demo --epochs 300 --sim_time 90 --save_dir ~/shots
#    导出 4 张曲线图：感知损失 / 控制损失 / 横向误差 / 转向指令

# ③ 在线感知 + 沿给定轨迹跟踪（需要 CARLA 服务端）
#    省略 --waypoints 即用内置 DEMO_ROUTE（26 个路点，已逐点校验在车道上）
python3 main.py --mode run --host <宿主机IP> --sim_time 90 --save_dir ~/shots
#    加 --follow 让 CARLA 大窗口以第三人称跟随自车
python3 main.py --mode run --host <宿主机IP> --sim_time 90 --follow

# ④ 连不上时先做连通性诊断（分步打印耗时，指出卡在哪一环）
python3 check_connection.py <宿主机IP> 2000 Town05

# ⑤ ROS 2 / ROS 1
ros2 launch carla_perception_control main.launch.py host:=<宿主机IP>
roslaunch carla_perception_control main.launch host:=<宿主机IP>
```

一键脚本：`bash main.sh --host <宿主机IP>` 或 Windows `main.bat --mode train`。

> **⚠ 给定轨迹的两个约束**
>
> 1. **航点必须落在可行驶车道上**：CARLA 中航点之间是直线连接，航点若在路面外，
>    车辆会沿直线开出路面撞上障碍物后卡死。
> 2. **弯道处航点要加密**：前视距离仅 6 m，转弯处航点间距过大会让车来不及转弯。
>
> 内置 `DEMO_ROUTE` 已满足这两点（26 航点、最大偏离车道 0.13 m、
> 弯道间距 8~9 m、含两个约 90° 弯）。自定义轨迹请参考模块文档 5.7 节。

## 4. 测试

```bash
python3 test/test_perception_logic.py     # 20 项全部通过，无需 CARLA
```

## 5. 实测指标

| 指标 | 离线取证 | 在线运行（Town05） |
|---|---|---|
| 感知 NN 训练集准确率 | 0.995 | 0.995 |
| 控制 NN 训练集 MSE | 0.00303 | 0.00303 |
| 横向误差 RMSE | **0.249 m** | **1.01 m** |
| 是否到达终点 | 是（t=54.8 s） | 是（t=28.9 s） |
| 传感器帧数 | — | 相机 577 帧 / 雷达约 500 点·帧 |

离线回放使用理想自行车模型，无碰撞与轮胎滑移；在线受路面碰撞、转向执行延迟
与同步步长影响，误差略高属正常差异。

## 6. 与已有示例的关系

本模块是 [`set_up_and_connect_to_carla`](https://openhutb.github.io/ros2/set_up_and_connect_to_carla/)
与「路径点发布器」示例的**拓展**：已有示例依赖 `carla_ros_bridge`，本模块改为
CARLA Python API 直连，并把感知与控制都换成神经网络。
**CARLA 服务端启动、宿主机 IP 与端口查看等通用配置步骤不在此重复**，
详见上述已有示例与
[模块文档](https://openhutb.github.io/ros2/ground/carla_perception_control/)。
