# MuJoCo 肌肉驱动夹爪：真实接触抓取

两指夹爪由 **Hill 肌肉**驱动合拢，与方块产生**真实接触摩擦**，抬升时方块完全靠摩擦力跟手。
八组实验回答：肌肉激活多大才夹得住、摩擦系数如何影响、抓持力如何闭环、松开后多久会掉、以及装到手臂上能否抓取。

与"运动学跟随"的演示不同，这里**不对方块施加任何位置或焊接约束**，抓不住就是抓不住。

## 运行环境

| 依赖 | 版本 |
|---|---|
| Ubuntu / ROS 2 | 22.04 / Humble，Docker 中构建和 launch 验收通过 |
| Python | 3.10 |
| MuJoCo | 3.3.7 |
| numpy | 1.26.4 |
| Pillow / matplotlib | 11.3.0 / 3.9.4 |

可通过主脚本独立运行，也可构建为 ROS 2 包并使用 `main.launch.py` 启动。

## 运行

```bash
python3 main.py check                 # 模型自检
python3 main.py run --scene g2        # 单个实验
python3 main.py bench --out out       # 八组实验，导出 out/metrics.json
python3 main.py gif --out gif         # 抓取、打滑与手臂联合抓取三段动图
python3 main.py manual               # 终端键盘控制
python3 main.py manual --keys wwwwwwwwwwrrrrq # 重放相同按键并输出真实接触力
python3 main.py neural --out out/neural # 采样、训练、独立测试和闭环评估
python3 main.py slam --out out/slam    # 移动底座神经网络建图与导航同步仿真
python3 tools/plot_metrics.py out/metrics.json out
python3 -m unittest discover -s tests # 基础与建图导航共 27 项测试
```

### Docker 与 ROS 2

在本模块目录执行：

```bash
docker build -t muscle-gripper:humble .
docker run --rm muscle-gripper:humble
docker run --rm --entrypoint bash muscle-gripper:humble -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch mujoco_muscle_gripper main.launch.py mode:=neural out:=/tmp/neural'
```

`mode` 支持 `check`、`run`、`bench`、`manual`、`neural`、`gif`、`slam`；
`scene` 选择 G1–G8，`out` 指定结果目录，`keys` 指定键盘回放序列。
保存结果时将主机目录绑定到容器 `/results`。Docker 中已完成 18 项测试，
并通过键盘、八场景与神经网络三种模式的真实 launch 验收。

python3 main.py gif --out gif         # 两段动图（抓住 / 打滑）
python3 main.py manual               # 终端键盘控制
python3 main.py manual --keys wwwwwwwwwwrrrrq # 重放相同按键并输出真实接触力
python3 main.py neural --out out/neural # 采样、训练、独立测试和闭环评估
python3 tools/plot_metrics.py out/metrics.json out
python3 -m unittest discover -s tests # 24 项测试
python3 tools/plot_metrics.py out/metrics.json out
python3 -m unittest discover -s tests # 21 项测试
```

### 键盘控制

`W/S` 每次增减 0.05 肌肉激活，`R/F` 每次升降 10 mm，空格松开，`Q` 退出。
肌肉激活限制为 0–1，抬升目标限制为 0–150 mm。每次操作推进 0.1 s 物理仿真，
输出仿真时间、接触点数和双指实际接触力。交互模式需要终端；自动复现使用 `--keys`。

### 神经网络力控制与图像策略

力策略使用真实接触反馈训练单隐层网络，跟踪 2、4、8 N 目标；图像策略使用
相机渲染的 16×16 灰度像素预测手指肌肉激活。手掌抬升按给定时间轨迹执行。
两类数据均按完整仿真回合划分训练、验证和测试集，导出损失曲线、闭环 CSV、权重、
相机截图和 GIF。默认固定随机种子 21，训练 200 轮。该命令包含独立 PI 基准对照。

Ubuntu 22.04 / MuJoCo 3.3.7 实测：力策略测试 MSE 为 `1.377e-8`，
图像闭环抬升 `98.11 mm`。输出目录内 `metrics.json` 保存完整参数及测量值。

### 移动底座建图与导航

夹爪移动底座在二维虚拟场景中同步完成定位、占据栅格建图和目标导航。
传感器提供激光距离和轮式里程计，神经网络参与运动估计、规划与控制。
三种障碍布局分别记录地图、运动轨迹、定位误差和目标到达结果。

在本模块目录运行：

```bash
python3 -m pip install -r requirements-slam.txt
python3 main.py slam --out out/slam --seed 21
```

本机 Windows / Python 3.11.8 / PyTorch 2.5.1 CPU 实测：三个场景全部到达目标，
碰撞次数均为 0；SLAM 位置 RMSE 为 0.097、0.098、0.109 m，
地图观测覆盖率为 93.75%、90.72%、94.63%。完整训练与三场景运行用时 7.36 s。

ROS 2 环境通过同一实现启动：

```bash
ros2 launch mujoco_muscle_gripper main.launch.py mode:=slam out:=/tmp/gripper-slam
```

输出包括 `metrics.json`、轨迹 CSV、地图与路径图、定位误差曲线、网络训练曲线和连续导航 GIF。
完整建模公式、运行结果和源码对应见中文教程第 12 节。

## 八组实验

### G1 肌肉激活 → 夹持力

| 激活 | 肌力 | 弹簧回位力 | 接触法向力（双指合计） | 接触点数 |
|---|---|---|---|---|
| 0.05 | 1.17 N | 1.17 N | 0.00 N | 0 |
| 0.10 | 2.29 N | 2.29 N | 0.00 N | 0 |
| 0.20 | 4.32 N | 4.32 N | 0.00 N | 0 |
| 0.40 | 7.67 N | 7.42 N | **0.50 N** | 8 |
| 0.80 | 14.13 N | 9.20 N | **9.86 N** | 8 |

夹持力取**手指与方块之间的真实接触法向力**（`mj_contactForce`），而不是肌力：
肌力要先扣掉 800 N/m 的回位弹簧力才作用到方块上。前三档手指行程只有 1.5–5.4 mm，
还没碰到方块（接触点数 0），因此接触力为 0——**不是"夹了 4.3 N 但滑了"，而是根本没夹上**。

### G2 抬升阈值：夹得多紧才抬得起来

抬升目标 100 mm，方块 30 g：

| 激活 | 抬升高度 | 结果 |
|---|---|---|
| 0.05 / 0.10 / 0.20 | ~0 mm | 打滑 |
| **0.40** | **97.8 mm** | 夹住 |
| 0.80 | 98.2 mm | 夹住 |

**阈值在 0.2 与 0.4 之间**：0.2 时手指还未接触方块（接触力 0），0.4 时每指真实法向力约 0.25 N 起就能夹住。

### G3 摩擦系数的影响

| 摩擦 μ | 激活 0.4 时抬升 |
|---|---|
| 0.3 | 3.7 mm（滑脱） |
| 0.8 | 97.8 mm |
| 1.5 | 97.8 mm |

摩擦不足时，**力够了也夹不住**——限制来自接触面而非肌肉。

### G4 对照组：松开肌肉会掉

夹住抬到 95 mm，然后把激活降到 0：

| 指标 | 数值 |
|---|---|
| 抬升高度 | 95.3 mm |
| **松开后掉落时间** | **0.16 s** |

### G5 抓持力闭环

PI 调节激活，反馈取 `mj_contactForce` 测得的双指接触法向力合计，并采用抗积分饱和。Ubuntu 22.04 / ROS 2 Humble / MuJoCo 3.3.7 实测：

| 目标 | 稳态 | 上升时间 | 末态激活 |
|---|---|---|---|
| 2 N | 2.00 N | 0.576 s | 0.474 |
| 4 N | 4.00 N | 0.445 s | 0.554 |
| 8 N | 8.00 N | 0.390 s | 0.713 |

### G6 滑移反射

40 g 方块、初始激活 0.33（刚好在打滑边缘）：

| 配置 | 抬升高度 | 加力步数 |
|---|---|---|
| 无反射 | **0 mm（抓不住）** | 0 |
| 有反射 | **86.0 mm** | 6 步（每步 +0.01，合计 +0.06 激活）|

判据：手掌已抬起但方块没跟上（相对位移 > 2 mm）即判为滑移，立即提高激活。

### G7 负载扫描

| 方块质量 | 最小可抓激活 |
|---|---|
| 10 g | 0.38 |
| 30 g | 0.38 |
| 60 g | 0.40 |

### G8 手臂-夹爪联合抓取

在夹爪上方加两连杆手臂（肩/肘 PD 跟踪逆解目标、腕关节伺服保持手掌竖直），
手指参数与独立实验完全一致：

| 指标 | 数值 |
|---|---|
| 方块起始高度 | −0.345 m |
| 方块结束高度 | −0.257 m |
| **抬升** | **87.6 mm** |

手臂位置由逆解给出，抓取仍然只靠手指与方块之间的摩擦——没有任何把方块设到手上的语句。

## 文件结构

```
mujoco_muscle_gripper/
├── main.py
├── package.xml / setup.py / setup.cfg
├── launch/main.launch.py
├── resource/
├── mujoco_muscle_gripper/
│   ├── core.py       # Hill 肌肉、MJCF、手指几何
│   ├── runs.py       # G1-G4 与录制
│   ├── extra.py      # G5-G8 与真实接触反馈
│   ├── manual.py     # 键盘控制
│   ├── neural.py     # 神经网络采样、训练与闭环评价
│   ├── slam_navigation.py # 移动底座建图、神经规划与控制
│   └── cli.py
├── tests/            # 24 项测试
├── Dockerfile        # Humble 固定运行环境
├── tests/            # 27 项测试（建图导航依赖见 requirements-slam.txt）
├── tests/            # 21 项测试
└── tools/plot_metrics.py
```

## 已知限制

1. **单指单肌肉**：每根手指只有一条屈肌 + 一个回位弹簧，没有拮抗肌对与肌梭反馈。
2. **平面两指**：没有指尖形状（近似为长方体），未建模滚动接触。
3. **方块为刚体，30 g**，不对应具体物体。
4. **仿真步长 0.5 ms**：接触刚度较高，需要小步长才能稳定。
5. 未接入 ROS 话题，未做 sim2real。

## 参考

- A. V. Hill, *The heat of shortening and the dynamic constants of muscle*, 1938
- R. M. Murray et al., *A Mathematical Introduction to Robotic Manipulation*, 1994（抓取力学）
