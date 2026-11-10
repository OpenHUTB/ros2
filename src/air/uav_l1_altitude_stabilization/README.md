# uav_l1_altitude_stabilization

基于 **纯 NumPy** 离线仿真的无人机 L1 自适应高度控制模块：一维点质量高度通道在
5 m 阶跃目标与阵风扰动下，由 PD 反馈 + L1 自适应估计项补偿集总风扰，输出垂向推力，
并给出超调、稳态误差、调节时间与扰动偏差等指标。

## 组成

| 文件 | 说明 |
|---|---|
| `main.sh` | 一键入口：定位模块目录并转调 `run_demo.sh` |
| `run_demo.sh` | 进入模块目录，运行核心脚本 `scripts/l1_altitude_node.py` |
| `scripts/l1_altitude_node.py` | 被控对象 `UAVAltitudePlant`、L1 控制器 `L1AltitudeController`、阵风模型 `gust(t)`、仿真与指标评测、三子图出图 |
| `README.md` | 模块说明、环境、运行方式与实测结果 |

脚本内约定的高度通道话题（未对接真实 ROS 发布 / 订阅）：
`/uav/altitude_setpoint`、`/uav/altitude_state`、`/uav/vertical_thrust`、`/uav/gust_disturbance`。

## 环境

- Ubuntu 20.04 / 22.04（本地 Windows 亦可离线运行）
- Python 3.8+（实测 Python 3.13）、NumPy 2.5.4、Matplotlib（Agg 后端）
- 纯 NumPy 点质量离线仿真，无需 ROS、Gazebo 或 AirSim

## 安装与运行

```bash
cd src/air/uav_l1_altitude_stabilization
bash main.sh
# 或直接运行核心脚本
python3 scripts/l1_altitude_node.py
```

> 纯离线数值仿真，不需要图形会话；响应曲线由 Matplotlib Agg 后端直接保存为
> `altitude_response.png`。

## 流程

1. **阶跃目标**：高度目标设为 5 m，无人机从 0 m 爬升；
2. **闭环控制**：PD 反馈（`kp=12.0`、`kd=6.0`）+ 自适应律（`gamma=20.0`）估计并
   补偿集总扰动，输出垂向推力；
3. **阵风扰动**：t=5s 叠加 2.0 m/s² 阶跃阵风，t=8s 起叠加振幅 1.5 m/s²、0.5 Hz
   正弦阵风；
4. **指标评测**：统计末高度、最大超调、稳态误差、2% 调节时间与阵风后最大偏差，
   绘制高度 / 推力 / 阵风三子图。

## 实测结果（Python 3.13 + NumPy 2.5.4，纯离线）

| 指标 | 数值 |
|---|---|
| 仿真时长 / 步长 | 15 s / 0.01 s |
| 目标高度 | 5.0 m |
| 末高度 | **5.191 m** |
| 最大超调 | 0.454 m（约 9.1%） |
| 稳态误差 | 0.191 m |
| 调节时间（±2% 误差带） | 0.73 s |
| 阵风后最大偏差 | 0.197 m |

## 已知限制

- 被控对象为简化一维点质量模型，未包含姿态内环与横向位置通道；
- 阵风模型为阶跃 + 正弦叠加，不等同于真实湍流风场；
- 观测为理想状态反馈，未引入传感器噪声、测量延迟与状态滤波；
- 未封装 ROS 2 节点，话题仅为脚本内约定，真实系统需接入状态估计与姿态内环后再验证。

## 大模型使用声明

本模块代码、参数调试与文档由 AI 助手辅助完成；所有仿真、曲线图、GIF 与数据均在
本地 Python 3.13 + NumPy 2.5.4 环境中实际运行生成，作者已核对并对内容负责。
