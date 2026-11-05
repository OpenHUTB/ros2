# ros2_ekf_localization

差速移动机器人的 **EKF 多传感器融合定位**：轮式里程计 + IMU 航向 + 绝对位置观测。

- 文档页：[地面载具 ROS2 差速机器人 EKF 多传感器融合定位](https://openhutb.github.io/ros2/ground/ros2_ekf_localization/)
- 作者：李诚轩（学号 2535030014）

核心算法与闭环仿真只依赖 NumPy，**不需要 ROS、不需要仿真器**即可完整复现；
在有 ROS 2 (Humble) 的环境下，`src/ros2_ekf_localization/node.py` 提供开箱即用的节点。

## 目录结构

```
ros2_ekf_localization/
├── src/ros2_ekf_localization/
│   ├── ekf.py            # EKF 预测/更新、角度残差归一化、NEES
│   ├── sensors.py        # 里程计/IMU/位置观测的噪声模型与航位推算基线
│   ├── trajectories.py   # 等弧长真值轨迹与里程计输入反推
│   ├── safety.py         # 话题超时跟踪、噪声参数校验、协方差数值修复
│   └── node.py           # ROS2 节点（订阅 /odom /imu /pose_measurement）
├── sim/
│   ├── run_sim.py        # 单场景离线仿真与出图
│   └── benchmark.py      # 轨迹 × 模式 × 观测频率 基准矩阵
├── launch/localize.launch.py
├── tests/test_core.py    # 单元测试（28 项）
├── ci/run_ci.sh          # 一键检查（语法/依赖/测试/仿真/基准）
├── package.xml           # ROS2 包描述
└── setup.py              # Python 包配置
```

## 快速开始

```bash
# 1. 安装依赖
pip3 install -r requirements.txt        # numpy, matplotlib

# 2. 离线仿真（无需 ROS）
python3 sim/run_sim.py --traj eight                  # 可选 circle / eight / s_curve
python3 sim/run_sim.py --traj eight --pos-rate 0     # 对照组：不使用位置观测
python3 sim/benchmark.py                             # 生成基准矩阵 results/benchmark.md

# 3. 一键检查
bash ci/run_ci.sh

# 4. ROS2 中部署
colcon build --packages-select ros2_ekf_localization
source install/setup.bash
ros2 launch ros2_ekf_localization localize.launch.py
```

## 核心公式

状态 $x = [p_x, p_y, \theta]^T$，控制量取里程计 $(v, \omega)$：

$$p_x' = p_x + v\cos\theta\,\Delta t,\quad p_y' = p_y + v\sin\theta\,\Delta t,\quad \theta' = \theta + \omega\Delta t$$

$$\hat P = F P F^T + Q,\quad F = \frac{\partial f}{\partial x},\quad Q = G N G^T,\; G = \frac{\partial f}{\partial u}$$

两类观测：

$$\text{IMU：} H = [0\ 0\ 1]\qquad \text{位置：} H = \begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}$$

更新采用 **Joseph 形式** $P \leftarrow (I-KH)P(I-KH)^T + KRK^T$，数值上比 $(I-KH)P$ 更稳定；
航向残差一律折算到 $(-\pi, \pi]$。

## 安全与数值保护

| 环节 | 处理 |
| --- | --- |
| `/odom` 超时或从未收到 | 跳过预测，只发布上一次估计（不在不可信控制量上递推） |
| `/imu`、`/pose_measurement` 超时 | 停止对应观测更新，诊断信息中给出 `*_stale` 与 `*_age` |
| 噪声参数 $\sigma \le 0$ | 回退默认值并告警（$\sigma=0$ 会让滤波器把预测当真理） |
| 观测含 NaN / 四元数非法 | 拒绝该帧，计入 `rejected_updates` |
| 协方差失去对称性或出现非有限值 | 对称化、对角夹紧、必要时重置，计入 `cov_repairs` |
| 时钟异常或调度延迟 | `dt` 退回标称周期，避免超大步长把状态推飞 |

`ekf.py` 与 `safety.py` 都不依赖 rclpy，上述规则全部有单元测试覆盖。

## 实验结果摘要

三条轨迹（圆形 / 8 字形 / S 形）× 三种模式，位置观测 1 Hz：

| 模式 | RMS 位置误差 | 末端误差 | NEES 均值 |
| --- | ---: | ---: | ---: |
| 纯航位推算 | 0.820 ~ 2.520 m | 1.278 ~ 3.591 m | — |
| EKF（里程计 + IMU 航向） | 0.199 ~ 0.388 m | 0.059 ~ 0.628 m | 3.70 ~ 12.19 |
| EKF（+ 绝对位置观测） | **0.083 ~ 0.110 m** | **0.064 ~ 0.074 m** | 3.03 ~ 6.04 |

单元测试 28 项全部通过（`python3 tests/test_core.py`）；
`bash ci/run_ci.sh` 五阶段全绿。
