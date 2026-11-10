# 无人机遥测异常检测与 ROS2 在线诊断

本页记录一个可复现的**遥测数据可靠性**实验：从 AirSim 记录的飞行状态里，判断当前观测是否"自相矛盾"——位置差分与速度不符、速度差分与加速度不符，或时间戳异常。输出异常分数、阈值、报警与诊断消息，**不下发任何飞控指令**。

![实际 ROS2 看板：异常分数越过阈值后报警](../img/air/telemetry_diagnostics/demo.gif)

上面的动图不是摆拍：`telemetry_diagnostics` 节点订阅回放话题，逐条计算分数并发布 `/uav/anomaly_score`，看板窗口把这 361 条消息画成曲线。第 6.5 秒左右分数从 −4 跳到 +8，越过阈值，报警位从 0 变 1。

---

## 1. 任务定义与边界

**输入**：9 个通道的飞行状态——位置 `px,py,pz`、速度 `vx,vy,vz`、加速度 `ax,ay,az`，以及源时间戳与到达时间。

**输出**：每条观测一个异常分数（logit）、阈值、`alarm` 布尔、原因码、数据有效性与模型就绪状态。

**边界**：

| 不做什么 | 原因 |
|---|---|
| 不输出控制指令 | 只做诊断，不参与飞控，没有失效保护 |
| 不识别"真实传感器硬件故障" | 位置漂移、速度偏置、冻结、丢包都是**软件观测层注入** |
| 不重复导航/避障/轨迹预测 | 那是 PPO、YOLO、状态预测几个项目的事 |

三个设计取舍：

| 取舍 | 选择 | 理由 |
|---|---|---|
| 学什么 | **跨通道一致性残差**，不学具体航迹 | 航迹千变万化，而"位置导数应等于速度"永远成立 |
| 谁负责规则 | 时间戳冻结/缺包交给规则，网络不背这个锅 | 规则零成本且 100% 可解释，混进网络只会让归因变模糊 |
| 阈值怎么定 | 正常验证分数的 99.5% 分位数 | 不用测试集调参，保证最终测试是"一次性的" |

**与相邻项目的界限**：上游的[飞行状态预测（PR #121）](https://github.com/OpenHUTB/ros2/pull/121)预测未来轨迹；本项目判断当前数据是否自洽。两者可以串联，但本项目不依赖它，`prepare` 之后即可独立运行。

---

## 2. 运行环境与快速开始

### 2.1 环境

| 项 | 配置 |
|---|---|
| 系统 | Ubuntu 20.04.6（VMware），6 核 / 8.5 GB |
| ROS | ROS2 **Humble**（`/opt/ros/humble`，Python 3.8） |
| 深度学习 | PyTorch **2.4.1+cpu** |
| 可视化 | Tk + matplotlib（不用 RViz2，本机缺 Ogre） |

### 2.2 安装与自检

```bash
cd ~/ros2/src/air/telemetry_diagnostics
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages ~/uav_prediction_env
source ~/uav_prediction_env/bin/activate
python -m pip install -r requirements.txt
bash main.sh doctor
bash main.sh build
```

`doctor` 检查 13 个模块（含 `launch_ros` 依赖的 `lark`），打印实际解释器并写 `artifacts/environment.json`。

常见故障：`launch_ros: No module named 'lark'` → 在本模块目录、已激活的环境中重跑 `pip install -r requirements.txt`。不要跳过检查硬上。

### 2.3 最快演示

```bash
python main.py prepare     # 解压语料并生成 70 个开发场景
python main.py build
bash main.sh demo          # 回放，约 28 秒
```

另一个终端（同一环境、`DISPLAY` 可用）：

```bash
python main.py dashboard
```

也可以一次启动：

```bash
source install/setup.bash
ros2 launch telemetry_diagnostics main.launch.py \
  model:="$PWD/models/consistency_42" csv:="$PWD/sample/observations.csv" plot:=true
```

演示约 18 秒仿真时间加启动等待，脚本自动退出并清理子进程。`sample/evaluation_only.csv` 不会送给诊断节点——避免"演示数据泄漏进评估"。

---
