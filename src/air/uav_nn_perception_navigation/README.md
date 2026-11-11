# uav_nn_perception_navigation

基于神经网络的无人机感知、导航与端到端控制模块，运行于 **AirSim 1.8.1 + ROS Noetic**。

## 功能一览

| 编号 | 功能 | 入口 | 说明 |
| --- | --- | --- | --- |
| 1 | 键盘遥控 | `main.py --mode teleop` | 终端按键控制无人机六个自由度 |
| 2 | 激光雷达感知 + 神经网络控制 | `main.py --mode perception` | 点云障碍检测，神经网络输出速度指令 |
| 3 | SLAM 建图 + 导航 | `main.py --mode slam` / `--mode nav` | 边飞边建图，基于地图规划路径 |
| 4 | 端到端视觉控制 | `main.py --mode end2end` | 图像直接输入网络，输出控制指令 |

## 运行环境

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 20.04（或 WSL2 中的 Ubuntu 20.04） |
| ROS | Noetic |
| 仿真器 | AirSim 1.8.1（Windows 或 Linux 均可） |
| Python | 3.8 |

## 依赖安装

```bash
pip3 install -r requirements.txt

# AirSim 官方客户端（PyPI 上的 airsim 包在部分 Python 版本下无法构建，
# 推荐直接使用 microsoft/AirSim 仓库中的 PythonClient 源码）
git clone --depth 1 https://github.com/microsoft/AirSim.git
export AIRSIM_PYTHON_CLIENT=$PWD/AirSim/PythonClient
```

把 `config/settings.json` 复制到 AirSim 的配置位置：

- **Windows**：放在 `Blocks.exe` **同目录**。
  注意：如果系统用户名是中文，AirSim 无法正确解析 `C:\Users\用户名\Documents\AirSim\`，
  必须使用可执行文件同目录的方案。
- **Linux**：`~/Documents/AirSim/settings.json`

## 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

如果 AirSim 运行在 Windows 主机上，虚拟机里的 ROS 需要通过主机 IP 访问：

```bash
# WSL2 中的 Windows 主机地址
python3 -c "import struct; \
[print('.'.join(str(b) for b in bytes.fromhex(l.split()[2])[::-1])) \
 for l in open('/proc/net/route').readlines()[1:] if l.split()[1]=='00000000']"
```

## 运行

```bash
# 键盘遥控（先启动 AirSim 仿真器）
roslaunch uav_nn_perception_navigation teleop.launch airsim_ip:=<主机IP>
```

| 按键 | 动作 |
| --- | --- |
| w / s | 前进 / 后退 |
| a / d | 左移 / 右移 |
| q / e | 左转 / 右转 |
| r / f | 上升 / 下降 |
| 空格 | 急停 |

## 话题与服务

| 名称 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/uav/cmd_vel` | `geometry_msgs/Twist` | 订阅 | 速度指令，m/s 与 rad/s |
| `/uav/odom` | `nav_msgs/Odometry` | 发布 | 位姿与速度（已由 NED 转到 ENU） |
| `/uav/takeoff` | `std_srvs/Trigger` | 服务 | 起飞 |
| `/uav/land` | `std_srvs/Trigger` | 服务 | 降落 |

## 目录结构

```
uav_nn_perception_navigation/
├── launch/          # 各功能的启动文件
├── scripts/         # main.py 统一入口与各节点
├── config/          # AirSim 配置与参数
├── models/          # 训练好的网络权重
└── docs/            # 说明文档（对应仓库 docs/air/ 下的页面）
```

## 常见问题

**连接失败 `Retry connection over the limit`**
仿真器没启动，或者 IP 不对。先确认 Blocks 窗口已加载完成，再确认 `airsim_ip`。

**改了 settings.json 不生效**
Windows 下中文用户名会导致 `Documents\AirSim\settings.json` 被覆盖回默认值，
把配置放到可执行文件同目录即可。

**键盘没反应**
节点必须运行在真实终端里；通过管道或后台运行时读不到标准输入。

## 安全兜底

`trajectory_control.py` 的网络输出会先经过 `safety_shield.py` 再下发，
网络仍然是控制量的主要来源，安全层只在下列情况限制或接管：

| 情况 | 判定 | 处理 |
| --- | --- | --- |
| 动作非法 | 出现 NaN / Inf | 水平速度与偏航归零，保持定高 |
| 输入过期 | 雷达或里程计 > 0.5 s 未更新 | 悬停等待 |
| 输出越界 | 水平速度 > 3 m/s、偏航 > 1 rad/s | 按训练量程限幅 |
| 正前方障碍 | 最近距离 < 1.2 m | 禁止继续前进 |
| 正前方极近 | 最近距离 < 0.5 m | 直接悬停 |
| 单侧障碍 | 该侧最近距离 < 0.9 m | 禁止朝该侧平移 |
| 偏航抖动 | 最近 8 帧中符号翻转 ≥ 5 次 | 偏航指令减半 |

安全层可以关闭，用于和"裸网络"做对照实验：

```bash
roslaunch uav_nn_perception_navigation perception.launch safety_enabled:=false
```

## 自主飞行实测

启动方式：

```bash
roslaunch uav_nn_perception_navigation perception.launch \
    airsim_ip:=<仿真器IP> goal_x:=0 goal_y:=20 goal_z:=15
```

| 指标 | 数值 |
| --- | --- |
| 起点 | (-0.35, 2.73, 15.17) |
| 目标 | (0.00, 20.00, 15.00) |
| 终点 | (-0.08, 20.76, 14.99) |
| 到达误差 | 0.76 m |
| 耗时 | 8.5 s |
| 高度波动 | ±0.1 m |

## 多场景评估

单次飞行的结果不能说明泛化能力，因此提供 `eval_multi_target.py`：固定 10 组
不同方位、距离和障碍布局的起终点对，逐组闭环飞行并统计。

```bash
python3 eval_multi_target.py --pairs 10 --altitude 4 \
    --out eval/multi_target.csv --trace-dir eval/trace
python3 eval_multi_target.py --pairs 10 --no-shield      # 对照组
```

脚本直接走 AirSim RPC，不依赖 ROS；特征提取、控制律和安全层与部署节点共用同一套
代码。每组输出到达误差、耗时、碰撞次数、安全层介入次数，以及闭环过程中网络输出
与势场法专家动作的偏差（`expert_dev_mean` / `expert_dev_max`）。

实测结果（AirSim 1.8.1 Blocks，4 m 高度，平面误差 < 1.0 m 记为成功）：

| 组 | 直线距离 | 到达误差 | 耗时 | 安全层介入 |
| --- | --- | --- | --- | --- |
| 1 | 25.6 m | 0.99 m | 18.2 s | 21 步 |
| 2 | 34.0 m | 0.98 m | 14.3 s | 45 步 |
| 3 | 42.2 m | 0.86 m | 15.4 s | 94 步 |
| 4 | 48.2 m | 0.99 m | 39.0 s | 31 步 |
| 5 | 39.5 m | 0.83 m | 16.9 s | 18 步 |
| 6 | 41.6 m | 0.93 m | 18.4 s | 13 步 |
| 7 | 40.1 m | 0.99 m | 40.6 s | 24 步 |
| 8 | 38.8 m | 0.80 m | 15.8 s | 35 步 |
| 9 | 29.7 m | 0.92 m | 15.3 s | 39 步 |
| 10 | 38.8 m | 0.97 m | 18.7 s | 2 步 |

| 配置 | 成功 | 误差（均值 ± 标准差） | 耗时均值 | 碰撞 |
| --- | --- | --- | --- | --- |
| 安全层开启 | 10/10 | 0.93 ± 0.07 m | 21.3 s | 0 |
| 安全层关闭 | 10/10 | 0.95 ± 0.06 m | 20.9 s | 0 |

在开阔场景里网络本身已经能稳定到达，安全层实际只做限幅（322 步，约占全部控制步
的 16%）；障碍屏蔽和异常接管通过故障注入验证：第 40 步把网络输出换成 NaN 后，
安全层立即接管并正常到达（0.98 m / 18.2 s，`shield_reasons=non_finite:1`）；
关闭安全层时同样注入，NaN 会直接下发给仿真器，本次 AirSim 恰好忽略了该帧，
但控制耗时从 18.2 s 增加到 39.6 s。

原始数据见 `eval/`：`multi_target_shield.csv`、`multi_target_noshield.csv`、
`fault_injection_*.csv`，以及对应的 `*_summary.json`。

## 网络与专家策略的偏差

验证损失小不等于闭环稳定，因此单独统计网络输出与势场法专家的偏差。开环用新采集
的 360 个状态（3 回合 × 120 步），再叠加扰动构造分布外状态；闭环用上表 10 组飞行
的逐步偏差。结果在 `models/train_metrics.json`，重新评估用：

```bash
python3 train_control.py --report-model models/controller.pt --episodes 3 --steps 120
```

| 指标 | 同分布 | 扰动（±10% 扇区 / ±5° 方位） |
| --- | --- | --- |
| 动作 MAE（三轴平均） | 0.392 | 0.400 |
| vx / vy / ω 的 MAE | 0.76 / 0.34 / 0.08 | 0.76 / 0.36 / 0.08 |
| 动作范数误差 均值 | 0.92 | 0.92 |
| 动作范数误差 P95 | 4.31 | 4.25 |
| 速度偏差 | 0.45 m/s | 0.45 m/s |
| 航向偏差 | 4.3° | 4.8° |
| 网络速度超 3 m/s 的比例 | 50% | 38% |

两点结论：

1. 小幅输入扰动几乎不放大偏差（MAE 0.392 → 0.400），说明网络对传感器噪声不敏感；
2. 偏差集中在 vx 和少数近障状态（P95 4.3，最大 5.2），也就是专家在大斥力、大方位角
   时的动作网络没有完全复现——这类状态在闭环里正是安全层负责兜底的部分。

闭环过程中每步的平均偏差为 0.78（安全层开启）/ 0.74（关闭），逐组最大 1.34~4.13，
比开环均值小：实际飞行轨迹很少进入训练分布边缘的高偏差状态。
