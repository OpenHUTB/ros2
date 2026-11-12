# 任务 3：声呐 SLAM 建图与神经网络导航

## 1. 运行环境与入口

```bash
./main.sh --task3            # 3D 视窗：SLAM + Neural A* + 局部避障
./main.sh --test3            # 两场景导航闭环评测
./main.sh --test-planning    # Neural A* 与局部策略训练有效性
./main.sh --launch3          # ros2 launch
```

## 2. 声呐占据栅格 SLAM

地图 20 m × 20 m，分辨率 0.05 m（400×400 栅格）。每个栅格维护对数几率

$$
L_t(\mathbf m)=L_{t-1}(\mathbf m)+
\begin{cases}
l_{\text{occ}}=1.2, & \text{命中},\\
-l_{\text{free}}=-0.35, & \text{波束穿过},
\end{cases}
\qquad L_t\in[-4,4].
$$

占据概率

$$
p(\mathbf m)=\sigma(L)=\frac{1}{1+e^{-L}},
$$

发布 `/map` 时 $\text{grid}=100p$。射线沿线用 Bresenham 标记空闲，端点标占用
（`sonar_slam_node.py:72`）。

## 3. Neural A* 全局规划

### 3.1 代价

$$
f(n)=g(n)+h(n)\big[1+\lambda(1-\Phi(n))\big],\qquad \lambda=0.35 .
$$

$\Phi\in[0,1]$ 是引导场：靠近起点—目标椭圆走廊时接近 1，远离时衰减。
解析先验（训练标签）：

$$
\Phi_{\text{ref}}=\exp\!\left(-\frac{(d_s+d_g)-d_{sg}}{2}\right),\qquad
\Phi_{\text{ref}}[\text{obstacle}]=0 .
$$

### 3.2 引导网络

逐栅格输入 13 维：

1. $(x-g_x)/\text{diag}$，$(y-g_y)/\text{diag}$
2. 走廊坐标 $(d_s+d_g-d_{sg})/8$
3. $d_g/\text{diag}$
4. 3×3 邻域占据（地图外视为障碍，共 9 维）

网络 $13\to64\to32\to1$（sigmoid），由 `train/train_neural_astar.py` 训练。
实测：验证 MSE $3\times10^{-6}$，与解析场相关系数 0.99998；走廊内预测 0.6319，
专家 0.6325。

### 3.3 规划结果

| 地图 | 经典 A* | Neural A* | 零权值 | 路径长度 |
|---|---|---|---|---|
| 200×200 测试图 | 346 节点 / 9.2 ms | 146 节点 / 306 ms（−57.8%） | 141 节点 / 215 ms | 保持 |
| 任务 3 全场 400×400 | 3307 节点 / 97 ms | 1864 节点 / 372 ms（−43.6%） | — | 12.07 → 12.08 m |

说明：纯 NumPy 推理增加了单次规划耗时（97→372 ms），节点削减与路径长度是主要收益；
这一点在评价中如实列出。

## 4. 局部避障策略网络

输入 76 维：

- 72 束声呐距离 / 15 m；
- 目标距离 / 10 m；
- $\sin$、$\cos$（目标相对方位）；
- 当前前向速度 / max_speed。

网络 $76\to128\to64\to3$（tanh），输出解码：

$$
v_x=\tfrac{1}{2}v_{\max}(a_0+1),\qquad
v_y=0.20\,a_1,\qquad
w_z=1.2\,a_2\ (\text{截断}\pm0.9).
$$

训练标签来自声呐势场专家：目标吸引 + 障碍排斥 + 近障减速 + 左右开阔度打破对称。
`train/train_local_policy.py` 训练 35k 样本、60 epochs，验证 MSE 0.0839，
vx/vy/wz 相关系数 0.88/0.71/0.91。

行为抽查（目标正前方 3 m）：

| 场景 | vx (m/s) | wz (rad/s) |
|---|---|---|
| 无遮挡 | 0.527 | +0.037 |
| 正前方 0.5 m | 0.399 | +0.055 |
| 左前方 0.5 m | — | −0.227 |
| 右前方 0.5 m | — | +0.512 |
| 零权值（正前方） | 0.275 | 0 |

安全层：0.5 m 内叠加几何反射转向并减速，0.3 m 内停止前进；该层仅兜底，
常规避障转向由网络输出。

## 5. 导航闭环结果

两场景各 4 个航点（`pr05_task3.log`）：

| 场景 | 到达 | 终点误差 | 里程 | 仿真时间 | 避障事件 |
|---|---|---|---|---|---|
| 静水巡检 | 4/4 | 0.100 m | 6.25 m | 20.0 s | 0 |
| 强剪切洋流 | 4/4 | 0.100 m | 6.36 m | 20.3 s | 0 |

避障事件为 0 是因为测试走廊无遮挡；局部策略的转向能力由 `--test-planning` 单独验证。

## 6. 源码解析

| 文件:行号 | 内容 |
|---|---|
| `rov_mujoco/sonar_slam_node.py:26` | 占据栅格核心 |
| `rov_mujoco/sonar_slam_node.py:72` | 声呐射线更新 |
| `rov_mujoco/sonar_slam_node.py:213` | SLAM ROS2 节点与 `/map` 发布 |
| `rov_mujoco/neural_path_planner.py:35` | 解析引导场 |
| `rov_mujoco/neural_path_planner.py:48` | 13 维引导特征 |
| `rov_mujoco/neural_path_planner.py:269` | `NeuralAStarPlanner` |
| `rov_mujoco/neural_path_planner.py:440` | `NeuralLocalAvoidancePolicy` |
| `rov_mujoco/neural_path_planner.py:520` | 76 维状态与动作解码 |
| `train/train_neural_astar.py` | 引导网络训练 |
| `train/train_local_policy.py:45` | 势场专家标签 |
| `test/test_nn_planning.py` | 规划/局部策略训练有效性 |

## 7. 已知限制

- Neural A* 的收益是节点数，不是墙钟时间；纯 NumPy 推理比经典 A* 慢（97→372 ms）。
- 局部策略在完全对称的正前方障碍上转向较弱（约 0.05 rad/s），0.5 m 内依靠安全层。
- 闭环测试场景无障碍，避障性能由独立单元测试覆盖。
