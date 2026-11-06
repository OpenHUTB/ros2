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
