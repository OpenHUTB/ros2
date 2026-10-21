# uav_neural_safety_filter — 神经安全过滤器

对应作业任务（2）传感器感知与任务（3）导航控制的安全层。

订阅 360° 激光点云与标称速度指令，输出修正后的速度指令。几何 CBF 层作为
监督信号与安全上界；神经网络用行为克隆学习它，单帧推理 0.14 ms。

依赖 [uav_mujoco_sim](../uav_mujoco_sim) 提供 `/lidar/points`、`/uav/odom` 与
`/uav/cmd_vel`。

---

## 1. 与课程已有模块的区别

| 已有模块 | 区别 |
|---|---|
| `uav_ppo_nav` | 端到端 RL 策略，不含独立安全层，碰撞率随训练波动 |
| `uav_depth_gate_racing_e2e` | 策略直接输出指令，没有指令级安全约束 |
| `octree_uav_3d_pathfinding` | 几何规划，默认位姿已知，不处理底层指令安全 |

课程内没有“在标称控制器之后加安全层”的模块。

## 2. 方法

观测：3×36 距离栅格（36 方位 × 3 俯仰，8 m 量程）+ 机头系速度 + 目标方向 +
距离/高度，共 116 维。

几何层：对一组候选速度，按环境的一阶速度低通（`tau=0.12 s`）与航向跟随
外推 0.6 s，取路径上的最小间距 `q`。约束 `q >= 0.8 m`，可行者取离标称指令
最近的一个；无可行候选时刹停。

神经层：输入 116 维观测 + 3 维标称指令，输出 3 维修正指令。
MLP 119→128→128→3，32259 个参数。

## 3. 安装

运行环境：ROS2 Humble（Ubuntu 20.04，Python 3.8）+ MuJoCo 3.12.0（仿真侧 Python ≥3.10）。

```bash
cd ~/uav_ws/ros2_ws/src
cp -r <本目录> .

uv venv ~/uav_ws/.venv --python 3.12
uv pip install --python ~/uav_ws/.venv/bin/python 'mujoco==3.12.0' numpy

cd ~/uav_ws/ros2_ws
colcon build --packages-select uav_mujoco_sim uav_neural_safety_filter
source install/setup.bash
```

## 4. 运行步骤

### 4.1 自检

不需要 ROS、不需要显示器：

```bash
cd ~/uav_ws/ros2_ws/src/uav_neural_safety_filter
python main.py --selftest
```

### 4.2 采集与训练

```bash
# 评价器数据（h/q 监督）
python main.py --collect           --out data/safety_dataset.npz
python main.py --train             --data data/safety_dataset.npz --out data/safety_net.npz

# 行为克隆数据：几何安全层闭环飞行
python main.py --collect-imitation --out data/safety_imitation.npz
python main.py --train-policy      --data data/safety_imitation.npz \
                                   --out data/safety_policy.npz --epochs 120
```

### 4.3 闭环评测

```bash
python main.py --evaluate --model data/safety_policy.npz \
               --episodes 12 --max-time 20 --eval-seed 100 --out results.json
```

### 4.4 出图

```bash
python main.py --figures --model data/safety_policy.npz --results results.json \
               --outdir docs/img/air/uav_neural_safety_filter
```

### 4.5 ROS2 在线运行

```bash
# 终端 1：仿真 + 桥接 + 安全层 + 演示名义控制器
ros2 launch uav_neural_safety_filter safety.launch.py with_sim:=true

# 只起安全层（假定 /lidar/points、/uav/odom、/uav/cmd_vel_nominal 已在别处运行）
ros2 launch uav_neural_safety_filter safety.launch.py nominal:=false
```

标称控制器可以换成键盘或规划器，只要把指令发到 `/uav/cmd_vel_nominal`。
键盘节点这样接：

```bash
ros2 run uav_mujoco_sim keyboard --ros-args -r /uav/cmd_vel:=/uav/cmd_vel_nominal
```

## 5. 接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/lidar/points` | sensor_msgs/PointCloud2 | 相对机体中心、世界轴向 |
| 订阅 | `/uav/odom` | nav_msgs/Odometry | 位姿与世界系速度 |
| 订阅 | `/uav/cmd_vel_nominal` | geometry_msgs/Twist | 机头系标称指令 |
| 订阅 | `/uav/goal` | geometry_msgs/Point | 目标点（构特征用） |
| 发布 | `/uav/cmd_vel` | geometry_msgs/Twist | 机体系，已抵消姿态 |
| 发布 | `/safety/status` | std_msgs/String | 推理次数、延迟、指令 |

## 6. 实测结果

12 个未见场景（seed 100–111，20 障碍，2.5 m/s，20 s）：

| 过滤器 | 碰撞率 | 到达率 | 推进率 | 最小间距 | 延迟 |
|---|---|---|---|---|---|
| 无过滤 | 42% | 58% | 74% | 0.63 m | 0.01 ms |
| 刹车 | 17% | 50% | 72% | 0.62 m | 0.01 ms |
| 几何 CBF | 8% | 75% | 83% | 0.84 m | 19.0 ms |
| **神经安全层** | **8%** | **75%** | **79%** | **0.77 m** | **0.14 ms** |

测距噪声 0.08 m + 15% 丢点：碰撞 17%、到达 75%、推进 81%。

（图见 `docs/air/uav_neural_safety_filter.md`，由 `--figures` 直接生成。）

## 7. 文件结构

```
uav_neural_safety_filter/
├── main.py                       入口
├── uav_neural_safety_filter/
│   ├── cli.py                    采集 / 训练 / 评测 / 出图 / 自检
│   ├── selftest.py               31 项自检
│   ├── plotting.py               六张评测图
│   ├── core/
│   │   ├── nn.py                 Linear / Adam
│   │   ├── geometry.py           盒/柱/球 SDF、扫掠与 rollout 间距
│   │   ├── ranges.py             3×36 距离栅格与特征
│   │   ├── env.py                SafetyEnv、场景与 episode
│   │   ├── dataset.py            评价器数据与行为克隆数据
│   │   ├── network.py            SafetyNet（h/q）与 PolicyNet（指令）
│   │   ├── training.py           两类训练与离线指标
│   │   ├── filter.py             候选集、几何层、神经层
│   │   └── experiment.py         闭环评测
│   └── ros/                      safety_node / nominal_node
├── config/safety.yaml
├── launch/safety.launch.py
├── data/                         数据集与权重
└── test/                         pytest 26 项
```

## 8. 已知限制

- 低矮障碍（<1.2 m）在 −15°/0°/+15° 三层雷达之间有盲区；评测场景的障碍高度
  已限制在 1.2 m 以上。
- 8% 的碰撞来自与教师层相同的近距极限状态；神经层没有超过教师层。
- 测距噪声与丢点让碰撞率翻倍（8%→17%）。
- 过滤器是短视的，只保证 0.6 s 视界内的间距；被完全封死的通道会停下而不是绕路。
- 策略在 16 个训练场景上拟合，换场景类型（走廊、城市）需要重新采集。
