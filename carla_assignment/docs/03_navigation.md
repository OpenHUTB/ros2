# 作业三 · CARLA 建图 + 导航（神经网络版，导航与 SLAM 同步仿真）

> 对应老师任务 (3)：在虚拟环境中实现对机器人的建图和导航，导航与 SLAM 的同步仿真。
> 同时满足老师"**规划算法需要为神经网络**"的要求：导航规划器为神经网络
> （`nn_models.MLPPolicy`），由「目标方位 + 前方障碍分布 → 前进/转向指令」。

## 1. 任务目标

- **建图**：车辆边行驶，边用 LiDAR 命中点投影到 2D **占用栅格地图**，体现"边动边建图"
  SLAM 理念。
- **规划**：将栅格感知 + 实时障碍分布喂给**规划神经网络**，驱动车辆向目标导航并避障。

## 2. 仿真环境

| 传感器 | 用途 |
|---|---|
| `sensor.lidar.ray_cast` (32 通道) | 环境扫描 → 占用栅格 + 障碍分布特征 |

## 3. 计算原理

### 3.1 占用栅格建图

雷达点云从传感器系转到世界系（yaw 旋转 + 平移），再投影到栅格：

$$
\begin{bmatrix} p_x^w \\ p_y^w \end{bmatrix}
= \mathbf R(\psi) \begin{bmatrix} p_x^s \\ p_y^s \end{bmatrix}
+\begin{bmatrix} x_v\\ y_v\end{bmatrix},\quad
r = \Big\lfloor \tfrac{p_y^w-o_y}{m}+\tfrac S2\Big\rfloor,\;
c = \Big\lfloor \tfrac{p_x^w-o_x}{m}+\tfrac S2\Big\rfloor
$$

多帧累加即得占用图。栅格概率可用贝叶斯更新。

### 3.2 规划神经网络

状态向量（5 维）：
$$
\mathbf s = \Big[\, \underbrace{\tfrac{e_\psi}{\pi}}_{\text{目标方位差}},\,
\underbrace{\tfrac N{50}}_{\text{左障碍密度}},\,
\underbrace{\tfrac N{50}}_{\text{右障碍密度}},\,
\underbrace{\tfrac{D_{\min}}8}_{\text{最近距离}},\,
D_{\min}\,\Big]^\top
$$

规划器输出 2 维控制（前进量、转向）：

$$
\mathbf h=\mathrm{ReLU}(W_1\mathbf s+b_1),\quad
[\tau,\ \delta] = W_2\mathbf h + b_2,\quad
\delta = \mathrm{clip}(\delta,-1,1)
$$

损失为 MSE。网络学到"左侧障碍多则右转、近障碍则减速"等行为。

## 4. 算法流程

```
train: 合成状态→期望(前进,转向) → MLP 回归训练 → 保存 json
run  : 连 CARLA → 挂雷达
       1) 建图阶段：直行 + 每 tick 把雷达命中写入占用栅格
       2) 导航阶段：提取障碍分布特征 → 规划 NN 输出(前进,转向) → 避障驶向目标
       3) 打印最终到目标最近距离 + occupied 格数
```

## 5. 源码解析（`03_navigation/main.py`）

- `_lidar_to_world()` / `build_occ_from_lidar()`：点云转世界系并写栅格。
- `obs_features()`：雷达 → 5 维归一化状态向量。
- `train_planning()`：合成数据集训练 `MLPPolicy([5,16,2])`。
- `run_carla()`：建图阶段 + 规划 NN 导航阶段。

## 6. 运行

```bash
# 1) 离线训练规划神经网络（仅 numpy）
python 03_navigation/main.py --mode train --epochs 300 --out models/nn_plan.json
#    输出：规划 NN MSE≈0.006

# 2) 在线建图 + NN 导航（需 CARLA）
python 03_navigation/main.py --mode run --model models/nn_plan.json \
       --goal "20,8" --sim_time 30
```

Windows:
```bat
main.bat navigation --mode train
main.bat navigation --mode run --goal 20,8
```
ROS launch：
```bash
# ROS2 Humble
ros2 launch carla_assignment 03_navigation_launch.py mode:=run goal:="20,8"
# ROS1 Noetic
roslaunch carla_assignment 03_navigation.launch mode:=run goal:="20,8"
```

## 7. 录屏剧本

1. 观察建图阶段 occupied 格数增长（地图扩展）。
2. 导航阶段：规划 NN 输出控制，车辆向目标行驶并绕开障碍，录制 10-15 秒。

!!! note "运行动图"
    录屏后替换此占位图为真实动图：`![](assets/placeholder_navigation.png)` → `![](assets/03_navigation.gif)`

## 8. 性能评价

| 指标 | 数值（示例） |
|---|---|
| 规划 NN MSE | ≈0.006 |
| 导航到达最近距离 (m) | 运行后填写 |
| 建图 occupied 格数 | 运行后填写 |

- **导航到达误差**：越小越好。
- **建图覆盖**：探测占用格数越多建图越充分。
- **避障效果**：规划 NN 输出在障碍附近转向平滑、不发生碰撞。
