# 任务 2：多传感器感知与神经网络 3D 轨迹跟踪

## 1. 运行环境与入口

```bash
./main.sh --track              # 3D 视窗：感知 + 神经网络轨迹跟踪
./main.sh --eval-trajectory    # 两工况自动化评测
./main.sh --test-nn            # 训练有效性与零权值消融
./main.sh --launch-track       # ros2 launch
```

## 2. 传感器

| 话题 | 类型 | 内容 |
|---|---|---|
| `/rov/sonar/scan` | `sensor_msgs/LaserScan` | 72 束前视声呐，FOV 120°，量程 15 m |
| `/rov/camera/image_raw` | `sensor_msgs/Image` | 前视相机真实离屏渲染 |
| `/rov/imu` | `sensor_msgs/Imu` | 姿态与角速度 |
| `/rov/depth` | `std_msgs/Float64` | 深度 |
| `/rov/desired_path` | `nav_msgs/Path` | 期望轨迹 |

声呐模型见 `rov_mujoco/sonar_sensor.py`，图像见 `rov_mujoco/camera_sensor.py`。

## 3. 控制网络

### 3.1 输入特征

世界系误差先旋转到机体坐标系：

$$
\mathbf e_p^b = R(\psi)^\top(\mathbf p_d-\mathbf p),\qquad
\mathbf e_v^b = R(\psi)^\top(\mathbf v_d-\mathbf v),\qquad
\mathbf v_d^b = R(\psi)^\top\mathbf v_d .
$$

10 维状态：

$$
\mathbf s = \big[e_{px},e_{py},e_{pz},e_{vx},e_{vy},e_{vz},
\sin e_\psi,\cos e_\psi,v_{dx}^b,v_{dy}^b\big]\odot \mathbf k,
$$

归一化系数 $\mathbf k=(1/1.5,1/1.5,1/1.5,1/0.8,1/0.8,1/0.8,1,1,1/0.6,1/0.6)$。

### 3.2 网络

$$
\mathbf h_1=\tanh(W_1\mathbf s+\mathbf b_1),\quad
\mathbf h_2=\tanh(W_2\mathbf h_1+\mathbf b_2),\quad
\mathbf a=\tanh(W_3\mathbf h_2+\mathbf b_3),
$$

尺寸 $10\to64\to64\to4$，参数量 5124。输出解码为

$$
f_x=350a_0,\quad f_y=350a_1,\quad f_z=500a_2,\quad \tau_z=80a_3 .
$$

权重文件 `rov_mujoco/nn_weights.npz`，由 `train/train_trajectory_policy.py` 训练。
`NNTrajectoryController` 在构造时加载 `.npz`；`zero_weights=True` 用于消融。

## 4. 专家与训练

LOS + PID 专家：

$$
f_x=\operatorname{clip}(280e_x+85e_{vx}+18v_{xd},\pm350),
$$

$f_y$ 同式，$f_z$ 用 $480e_z+140e_{vz}$，$\tau_z=120e_\psi+30e_{\omega_z}$（截断）。

数据：60 000 个随机状态（70% 均匀采样 + 30% 近零误差），80 epochs，Adam。
实测验证集：

```text
val MSE = 0.000428
物理 RMSE: fx 8.70 N, fy 8.91 N, fz 10.11 N, tau 0.48 N·m
动作相关系数: 0.9991
```

## 5. 闭环结果

两工况，NN 与 LOS+PID 基准同样跑 3D 螺旋轨迹（`pr05_task2_tracking.log`）：

| 工况 | 控制 | 3D RMSE | 水平 RMSE | 垂向 RMSE | 最大偏差 |
|---|---|---|---|---|---|
| 静水 | LOS+PID | 0.216 m | 0.214 m | 0.034 m | 0.322 m |
| 静水 | NN | 0.221 m | 0.219 m | 0.032 m | 0.331 m |
| 强剪切洋流 | LOS+PID | 0.223 m | 0.220 m | 0.034 m | 0.332 m |
| 强剪切洋流 | NN | 0.227 m | 0.225 m | 0.031 m | 0.341 m |

NN 与专家基准相差 0.005–0.007 m，垂向略优；控制指令完全来自网络输出。

零权值消融（定点悬停，初始偏差 1.05 m，2500 步）：

```text
训练网络 : RMSE 0.504 m, 终点误差 0.047 m
零权值   : RMSE 1.098 m, 终点误差 1.106 m  (2.2×)
```

## 6. 源码解析

| 文件:行号 | 内容 |
|---|---|
| `rov_mujoco/nn_trajectory_controller.py:25` | `STATE_SCALE` 归一化系数 |
| `rov_mujoco/nn_trajectory_controller.py:52` | `NNTrajectoryController` 权值加载 |
| `rov_mujoco/nn_trajectory_controller.py:154` | 机体坐标变换、10 维特征与推理 |
| `rov_mujoco/nn_trajectory_controller.py:224` | LOS+PID 基准 |
| `train/train_trajectory_policy.py:35` | 专家数据生成 |
| `train/train_trajectory_policy.py:85` | 训练与保存 |
| `test/test_nn_training.py:34` | 专家一致性与零权值闭环测试 |

## 7. 已知限制

- 纯 NumPy 训练；训练样本 60k，约数分钟。
- 网络是对 LOS/PID 的蒸馏，性能不会显著超过专家；在强洋流下略差 0.004–0.007 m。
- 感知（声呐/相机）到控制是分模块串联，不是单一端到端网络（端到端见任务 4）。
!!! warning "Python 环境说明"
    - **Ubuntu 22.04**：默认使用系统 Python 3.10 环境即可直接运行。
    - **Ubuntu 20.04**：需使用 Python 3.8 执行指令，以保证与 ROS 2 C-extension (`_rclpy_pybind11.cpython-38`) 的 ABI 兼容性。

### 7.3 无 ROS 的离线回归评测（新增）

上面的测试与仿真都要求 ROS 2 和 MuJoCo，在没有 ROS 的机器（Windows、纯 Python 环境）上一步都跑不了。新增的离线评测把现有的 NN / LOS 控制器、轨迹生成器接到一个**一阶速度响应代理**上，只依赖 `numpy` 和 `matplotlib`：

```bash
# 在模块目录下
python3 eval_offline.py --duration 60
python3 eval_offline.py --controllers nn --trajectories 3d_helix --duration 30
```

代理模型为 `m·dv/dt = R(yaw)·[fx, fy, 0] + [0, 0, fz] − c·v`、`I·dω/dt = τz − c_yaw·ω`，**不是 MuJoCo 的 6-DOF 水动力**，因此结果只用于回归和相对比较，不能替代仿真结论。评测从偏离参考点约 1 m 的位置出发，考察收敛过程与稳态误差。

| 航迹 / 控制器 | RMSE (m) | 最大误差 (m) | 末段误差 (m) | 偏航 RMSE (rad) | 平均控制量 |
|---|---:|---:|---:|---:|---:|
| 3D 螺旋 / NN | 0.133 | 1.370 | 0.083 | 0.110 | 11 |
| 3D 螺旋 / LOS+PID | 0.112 | 1.370 | 0.027 | 0.110 | 11 |
| 割草机 / NN | 0.122 | 1.370 | 0.077 | 0.242 | 23 |
| 割草机 / LOS+PID | 0.106 | 1.371 | 0.022 | 0.246 | 20 |

两个控制器都能把 1.4 m 的初始偏差在约 4 秒内收敛到 0.1 m 以内，差别主要在稳态残差：LOS+PID 为 0.027 / 0.022 m，NN 约 8 cm。这更像是训练环境的差异——NN 权值在 MuJoCo 中训练，而代理模型没有推进器布置、附加水质量等效应，网络没有补偿掉这部分稳态偏差；不能据此认为 NN 在 MuJoCo 中更差，两者在真实仿真里的对比仍以第 4 节为准。

评测可以作为**回归门禁**使用：任一组合超过给定 RMSE 即返回非零退出码。

```bash
python3 eval_offline.py --duration 60 --max-rmse 0.2   # 当前全部通过
```

![离线闭环误差曲线](img/water/rov_trajectory_tracking_offline.png)

指标写入 `results/offline_eval.json`，曲线写入 `results/offline_tracking.png`。对应单元测试不依赖 ROS：`python3 -m unittest discover -s test -p "test_offline_eval.py"`。

### 7.4 异机复测记录

本模块的离线评测在另一台机器上独立复跑过一次，用于确认「不依赖 ROS / 本机环境」这一点成立：

| 项目 | 复测环境 |
|---|---|
| 系统 | Ubuntu 22.04.5（WSL2） |
| Python | 3.10.12 |
| NumPy | 1.21.5 |

复测结论（与本文表格一致）：

- `python3 -m unittest discover -s test -p "test_offline_eval.py" -v` → `Ran 6 tests in 0.529s`，**6/6 通过**
- `python3 eval_offline.py --duration 60 --max-rmse 0.2` → `GATE PASS`，退出码 0
- 四组 RMSE 复现一致：3D 螺旋 NN 0.133 / LOS 0.112，割草机 NN 0.122 / LOS 0.106（m）

复测环境与开发环境（Windows 10 + Python 3.11 + NumPy 1.24）不同，说明这套评测只依赖 `numpy` 与 `matplotlib`，不挑平台；但代理模型仍是运动学近似，结论范围见上一节的说明。
