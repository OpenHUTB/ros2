# 阿克曼小车的 MPC 跟踪：为什么要「提前几步」打方向

上一节用 Dubins 算出了一条车能走的路。但「能走」和「走得住」是两件事：车有初始
偏差、有转向速率限制、路面不平，跟踪器必须不断修正。斯坦利（Stanley）只看**当前**
的横向误差，MPC 则往前看若干步。

这篇用同一条 S 弯路径、同样的限幅、同样的速度，把两者摆在一起比。

## 1. 被控对象与误差模型

运动学自行车模型（后轴中心为参考点）：

$$
\dot x = v\cos\psi,\qquad \dot y = v\sin\psi,\qquad
\dot\psi = \frac{v\tan\delta}{L}
$$

$v$ 是纵向速度，$\delta$ 是前轮转角，$L$ 是轴距。跟踪问题不关心绝对位置，只关心
相对参考路径的两项误差：横向误差 $e_y$（沿路径左法向为正）和航向误差
$e_\psi = \psi - \psi_{ref}$。在参考转角 $\delta_{ref} = \arctan(L\kappa_{ref})$ 处线性化：

$$
\dot e_y = v\,e_\psi,\qquad
\dot e_\psi = \frac{v}{L}\,(\delta - \delta_{ref})
$$

写成矩阵形式 $\dot e = Ae + Bu$，其中

$$
A=\begin{bmatrix}0 & v\\ 0 & 0\end{bmatrix},\qquad
B=\begin{bmatrix}0\\ v/L\end{bmatrix},\qquad
u = \delta - \delta_{ref}
$$

模型是线性的、二维的，所以后面每一轮优化都只是一个 15 维的二次规划——纯 Python
手写就能在毫秒级解掉。

## 2. MPC 与 Stanley 的真正差别：预瞄

Stanley 的控制律是

$$
\delta = -e_\psi - \arctan\!\left(\frac{k\,e_y}{v}\right) + \arctan(L\kappa_{ref})
$$

三项分别是航向修正、横向修正、曲率前馈。它**没有预瞄**：误差出现了才修。
在曲率突变的地方（比如直线进圆弧），它必然滞后半拍。

MPC 在长度为 $N$ 的时域上最小化

$$
J=\sum_{k=1}^{N} e_k^\top Q e_k
  + \sum_{k=0}^{N-1}\left(R\,\delta_k^2 + R_{rate}\,(\delta_k-\delta_{k-1})^2\right)
$$

约束是前轮转角幅值 $|\delta_k|\le \delta_{max}$ 和转向速率
$|\delta_k-\delta_{k-1}|\le \dot\delta_{max}\Delta t$。它每一步都在问：
「如果接下来 1.5 秒我都这么打方向，误差会变成什么样？」——于是它在**还没进弯**
的时候就开始打方向了。

这个差别在下面的转角曲线上看得很清楚：

![前轮转角对比：MPC 提前进入，Stanley 滞后并饱和](../img/chap7/ackermann_mpc_tracking/mpc_steer.png)

蓝线（MPC）在弧长 15 m 左右就开始向负方向打，峰值 0.483 rad，全程平滑；红线
（Stanley）晚一步，而且直接顶到 0.60 rad 的限幅上——**饱和意味着它已经没有余量
再修正了**，这直接导致后面的大幅过冲。

## 3. 怎么解这个 QP

决策变量取**转角增量** $\Delta\delta_k$，于是：

$$
\delta_k = \delta_{prev} + \sum_{i\le k}\Delta\delta_i
$$

这样速率约束 $|\Delta\delta_k|\le \dot\delta_{max}\Delta t$ 就是标准的箱式约束，
投影投影回可行域只要一次 `clip`。把增量代入误差预测后，代价函数仍是二次型，
于是问题变成

$$
\min_{\Delta\delta}\ \tfrac12 \Delta\delta^\top H\Delta\delta + g^\top\Delta\delta
\quad\text{s.t.}\quad |\Delta\delta_k|\le \Delta_{max}
$$

本模块用**坐标下降 + 箱式投影**求解：逐个变量求一维二次极小再投影，几十轮就收敛
（实测 28 轮，残差 $3\times10^{-13}$）。规模只有 15 维，比引入 cvxpy 更省事，也让
读者能看清每一步在算什么。

初版有过一个反面教训：原来把「转角偏差」当决策变量、只用软惩罚管速率，结果
QP 认为转向可以瞬间到位，而真实执行器要爬 0.4 s，两者一对不上就振荡发散
（RMSE 从 0.22 恶化到 2.19）。**约束写在惩罚里而不是可行域里，模型和现实就会打架**
——这是 MPC 工程化里最常见的坑之一。

## 4. 对比结果

场景：直线 6 m → 右转 R=6 m 90° → 左转 R=6 m 90° → 直线 8 m，总长 32.85 m。
轴距 2.5 m，速度 5 m/s（18 km/h），初始横向偏差 0.8 m，转角上限 0.6 rad，
转向速率上限 1.6 rad/s，两个控制器完全相同。

| 控制器 | RMSE | 最大误差 | 末值误差 | 峰值转角 | 转向速率 RMS |
|---|---|---|---|---|---|
| MPC | **0.2182 m** | 0.8000 m | **0.0009 m** | 0.483 rad | 0.708 rad/s |
| Stanley（k=1.5） | 0.5502 m | 1.2638 m | 0.1654 m | 0.600 rad（饱和） | 0.730 rad/s |

两个数字值得单独说：

- **最大误差**：MPC 的 0.8000 m 就是初始偏差，一步没超调；Stanley 冲到 1.2638 m，
  比初始偏差还大 58%——先冲过头再拉回来。
- **末值误差**：MPC 收到 0.9 mm，Stanley 还挂着 16.5 cm。

横向误差随弧长的完整曲线：

![横向误差对比：MPC 快速收敛，Stanley 在 S 弯出口冲到 1.26 m](../img/chap7/ackermann_mpc_tracking/mpc_error.png)

轨迹俯视图：

![参考路径与两条跟踪轨迹](../img/chap7/ackermann_mpc_tracking/mpc_tracking_path.png)

## 5. 参数敏感性：这才是 MPC 最实用的地方

Stanley 的 `k` 是那种「调好了今天好用、换个速度就不好用」的参数：

| `k` | 0.5 | 1.0 | 1.5 | 2.5 | 4.0 | 6.0 | 10.0 |
|---|---|---|---|---|---|---|---|
| RMSE (m) | 0.640 | 0.632 | 0.550 | 0.564 | **0.379** | 1.197 | 2.106 |

最优 `k = 4.0` 给出 0.379 m，仍高于 MPC 的 0.218 m；而 `k` 取到 6 以上就直接崩掉。
换句话说，**即使把 Stanley 调到它自己的最优点，也追不上 MPC 的默认参数**。

MPC 这边：

| `q_y` | 2 | 5 | 10 | 30 | 60 |
|---|---|---|---|---|---|
| RMSE (m) | 0.231 | 0.224 | **0.218** | 0.220 | 0.228 |

权重在 30 倍范围内变动，RMSE 只在 0.218~0.231 之间晃（±3%），而且最大误差
始终等于初始偏差——不超调这个性质是**权重不敏感**的。

原因不神秘：MPC 的权重有明确的物理含义（误差 vs 转向代价），而 Stanley 的 `k`
和速度耦合在一起，换个车速就得重调。

初始偏差扫描（速度不变）：

| 初始 `e_y` | 0.3 m | 0.6 m | 0.9 m | 1.2 m |
|---|---|---|---|---|
| MPC RMSE | 0.076 | 0.158 | 0.249 | 0.346 |
| Stanley RMSE | 0.501 | 0.527 | 0.564 | 0.608 |
| 倍数 | **6.6×** | 3.3× | 2.3× | 1.8× |

偏差小时差距最大（6.6 倍），因为 Stanley 在高速小偏差下会持续小幅振荡，
而 MPC 直接把它压到零。

## 6. 代码与运行

```bash
cd src/chap7/ackermann_mpc_tracking
python3 tests/test_mpc.py   # 24 项单元测试
python3 scripts/main.py     # 打印全部对比表并重新生成 3 张图
```

`scripts/mpc.py` 的关键接口：

| 接口 | 说明 |
|---|---|
| `KinematicBicycle.step(state, v, delta, dt)` | RK4 积分一步 |
| `PathTracker.update(x, y)` | 局部搜索最近点，给出 `e_y / e_psi / kappa_ref` |
| `LinearMPC.solve(e0, delta_ref, delta_prev)` | 返回转角增量序列 |
| `LinearMPC.delta_sequence(d_seq, delta_prev)` | 增量还原成绝对转角（带幅值兜底） |

### 在 ROS 2 中跑

```bash
colcon build --packages-select ackermann_mpc_tracking
source install/setup.bash
ros2 launch ackermann_mpc_tracking ackermann_mpc_tracking.launch.py
ros2 topic echo /cmd_vel
ros2 topic echo /mpc/debug    # [e_y, e_psi, kappa_ref, delta, s]
```

节点订阅 `/odom`、发布 `/cmd_vel`；阿克曼底盘一般吃等效角速度
（`angular.z = v·tan(δ)/L`），所以节点直接算好角速度发出去，底层不用再转换。

## 7. 局限与延伸

- **线性化只在参考点附近成立**。误差大到 1 m 以上时模型精度下降，本模块靠
  滚动重规划补偿，但没有做非线性 MPC（NMPC）。
- **预测时域固定 1.5 s**。速度提高时应同步加长时域，否则「看不够远」。
- **没有把障碍物写进约束**。工程上通常把避障表达成状态约束，QP 就从箱式变成一般
  线性约束，求解器也要换成 active-set 或内点法（那时就该用 `osqp` / `cvxpy` 了）。
- **前轮转角直接当控制量**。真车还有转向执行器的一阶滞后，严格做法是把滞后也
  建进模型（状态扩一维）。

## 8. 与已有内容的关系

- 与「Dubins 最短路径」前后衔接：那篇给出一条满足曲率约束的路径，这篇负责把它
  跟住——路径规划与轨迹跟踪本来就是一条链上的两步。
- 与「Stanley 前视路径跟踪」是同题对比：Stanley 轻、好调、够用（低速时几乎无差别），
  MPC 贵、但带约束、抗参数扰动。**低速选 Stanley，高速或要明确约束选 MPC**，
  这是工程上常见的选择。
- 与「梯形与 S 曲线速度规划」互补：那篇管纵向怎么加速，这篇管横向怎么打方向。

> 本文档由 AI 辅助撰写，模型、QP 求解、闭环结果与全部数值均在本机实跑验证
> （`tests/test_mpc.py` 24 项全过）。`colcon build` 与 `ros2 launch` 未在安装了
> ROS 2 发行版的机器上实测。
