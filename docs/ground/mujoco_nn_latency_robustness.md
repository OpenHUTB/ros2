# 神经网络跟踪策略的执行延迟鲁棒性（MuJoCo）

!!! abstract "一句话"
    同一份权重、同一条赛道，只把「指令算出来 → 落到车上」的时间从 0 ms 拉到 1000 ms，
    端到端策略会从 0.035 m 的跟踪误差退化到贴着墙跑、800 ms 之后连一圈都跑不完。

    在策略前面加一层**模型前向（Smith 预测 + 扫描变换）**这种不依赖地图的补偿，
    可以把 600 ms 的误差从 0.418 m 压到 0.092 m、800 ms 从 0.440 m 压到 0.154 m；
    但到 1000 ms（预测位移 1 m）它自己也不行了。

    再往上还有一档 oracle：允许直接向"完美地图"要预测位姿处的扫描，1000 ms 下仍是 0.054 m。
    这条虚线标出的正是**无地图补偿的上限**，也是本模块对真实系统最诚实的结论。

| 全景（延迟 600 ms，无补偿） | 全景（延迟 600 ms，模型前向补偿） |
| --- | --- |
| ![](../../img/ground/lat_nav_none.gif) | ![](../../img/ground/lat_nav_model.gif) |

## 1. 为什么要单独研究"时延"

端到端（感知 → 网络 → 控制）在仿真里跑得再好，上车以后常栽在一件很朴素的事上：
**策略看到的永远是过去的场景**。相机曝光、推理、ROS 通信、执行器排队，每一环都加几十毫秒；
当车以 1 m/s 前进、控制频率 20 Hz 时，300 ms 就是 30 cm 的盲区——已经等于本模块里车道半宽
（0.75 m）的 40%。

本模块把这个变量单独拎出来做实验：

- **自变量**：执行延迟 \(d\)（0～1000 ms）、控制频率 \(f\)（10/20/50 Hz）、补偿方式（3 种）；
- **因变量**：横向误差 RMS、完成率、碰撞帧数、最小净空；
- **控制变量**：同一份网络权重、同一条赛道、同样的随机种子。

与"换个控制器再比一比"不同，这里只动一个旋钮，因此结论可以直接迁移到别处的端到端系统。

## 2. 场景与机器人

### 2.1 三条不自交的赛道

场景完全由代码生成（`track.build_xml()` → `mj_model.from_xml_string`），
不依赖任何外部资源文件——这一点是有意的，见 §7.2 的踩坑说明。

| 赛道 | 参数方程 | 长度 | 最小曲率半径 |
| --- | --- | --- | --- |
| `circle` | \((3\cos t,\ 3\sin t)\) | 18.8 m | 3.00 m |
| `kidney` | \(r=3+0.9\sin t\) 的极坐标曲线 | 19.3 m | 2.86 m |
| `peanut` | \(r=1.6(3+1.2\cos 2t)\) | 34.7 m | 1.73 m |

车道半宽 \(w=0.75\) m。墙体沿中心线法向两侧各铺一圈 14 cm 的方块：

$$
\mathbf{b}_\pm(s) = \mathbf{c}(s) \pm w\,\mathbf{n}(s), \qquad
\mathbf{n}(s) = \big(-t_y(s),\ t_x(s)\big)
$$

其中 \(\mathbf{c}(s)\) 是弧长参数化的中心线，\(\mathbf{t}\) 是单位切向。

!!! note "为什么必须检查曲率半径"
    内侧墙体自己会互相压到一起：若最小曲率半径 \(R_{\min} < w\)，弯心处的墙块挤成一团，
    车根本开不过去。第一版用了 Gerono 双纽线（8 字），它在交叉点处两条支路共用一段走廊，
    墙直接把路封死；花生形在 k=1.0 时 \(R_{\min}=1.08\ \mathrm{m}\)，同样过不去。
    `test_curvature_allows_lane_width` 把 \(R_{\min} > w + 0.5\) 钉成了回归测试。

### 2.2 机器人：理想速度伺服

车体是一个 0.32 × 0.20 m 的盒子，用三个关节（\(x\) 滑移、\(y\) 滑移、\(z\) 转动）+
三个 velocity 执行器驱动：

$$
\dot{x} = v\cos\theta,\qquad \dot{y}=v\sin\theta,\qquad \dot{\theta}=\omega
$$

执行器力 \(\tau = k_v(u - \dot{q})\)，本实验取 \(k_v=200\)，物理步长 5 ms（RK4），
每个控制周期跑 10 个物理步。实测从 0 到 1 m/s 的建立时间约 2 个控制周期（100 ms），
**这个上升时间算在延迟之外**，不参与后面的补偿对比。

这么设计是为了让"延迟"成为链路里**唯一**的时变误差源：没有轮胎侧偏、没有扭矩饱和、
没有打滑，剩下的差异全部来自"指令晚到了"。

### 2.3 感知：24 线雷达

每条射线用 `mj_ray` 打到墙上：

$$
r_i = \min\Big(d_{\max},\ \big\|\mathbf{p} + r(\varphi_i)\,\mathbf{e}(\varphi_i) - \mathbf{p}_{hit}\big\|\Big)
+ \epsilon_i,\qquad \epsilon_i \sim \mathcal{N}(0, 0.01^2)
$$

\(\varphi_i = \theta + 2\pi i/24\)（\(i=0..23\)），\(d_{\max}=6\) m。
射线起点在车身内部，因此必须用 `bodyexclude` 把 `robot` 这个 body 排除，
否则每束都会读回自己的车壳（实测起步 24 束全是 0.1 m，见 §7.3）。

## 3. 策略网络

输入 26 维：24 束归一化距离 + 当前速度 + 上一帧角速度。

$$
\mathbf{z}_1 = \mathbf{W}_1\mathbf{x}+\mathbf{b}_1,\quad \mathbf{h}_1=\tanh(\mathbf{z}_1),
\quad \mathbf{z}_2 = \mathbf{W}_2\mathbf{h}_1+\mathbf{b}_2,\quad \mathbf{h}_2=\tanh(\mathbf{z}_2)
$$

输出层两路激活不同，因为物理量纲不同：

$$
v = \frac{v_{\max}}{1+e^{-z_3^{(1)}}}\in[0,1.4],\qquad
\omega = \omega_{\max}\tanh\big(z_3^{(2)}\big)\in[-1.6,1.6]
$$

参数总量 26×64 + 64 + 64×32 + 32 + 32×2 + 2 = **3874**，全部用 numpy 手写前向/反向 + Adam。

损失与梯度（\(N\) 为 batch 大小）：

$$
L=\frac{1}{N}\sum_{n=1}^{N}\big\|\hat{\mathbf{y}}_n-\mathbf{y}_n\big\|^2,
\qquad
\frac{\partial L}{\partial z^{(1)}} = \frac{2(\hat v - v)}{N}\,\hat v\Big(1-\frac{\hat v}{v_{\max}}\Big)
$$

第二路因为 \(\omega=\omega_{\max}\tanh z\)，多乘一个 \(\omega_{\max}\)：

$$
\frac{\partial L}{\partial z^{(2)}} = \frac{2(\hat\omega-\omega)}{N}\,\omega_{\max}\Big(1-\big(\tfrac{\hat\omega}{\omega_{\max}}\big)^2\Big)
$$

!!! warning "少乘这个 \(\omega_{\max}\) 会怎样"
    第一版漏了它，训练照样收敛（只是角速度学得慢），单测的"数值梯度对比"却用
    `np.mean` 当损失、容差写成 \(2\times10^{-2}\max(1,|g|)\)，而该元素梯度量级才 \(10^{-3}\)——
    于是**梯度错了 2 倍也照样通过**。现在测试与 `backward` 同口径（\(1/N\sum\sum\)）、
    查三个元素、判据是相对误差 < 5e-2。

训练数据来自真值纯追踪专家（**专家看位姿，网络只看雷达**）：

$$
\omega_{ref} = \frac{2 v_{ref}\sin\alpha}{L_d},\qquad \alpha = \operatorname{wrap}\big(\psi_{目标}-\theta\big)
$$

预瞄距离 \(L_d=1.0\) m 按弧长取。采 3 条赛道 × 3 回合（3600 条）→ 模仿 100 轮 →
再补一轮 DAgger（600 条，在 300 ms 延迟下让网络自己跑、专家贴标签）→ 40 轮。

## 4. 延迟与四种处理方式

三种是**真车做得到**的（`none` / `extrap` / `model`），第四种 `oracle` 故意留了一个"作弊"的，
用来标出「有地图 vs 没地图」的差距——上一版评审指出：直接在预测位姿上问仿真器要扫描，
真车做不到，所以它只能当上界，不能当结论。

### 4.1 延迟模型

第 \(t\) 帧算出的指令要排 \(d\) 帧的队（\(d = \operatorname{round}(d_{ms} f/1000)\)）：

$$
\mathbf{u}_{applied}(t) = \mathbf{u}_{cmd}(t-d)
$$

实现就是一个固定长度 FIFO：入队新指令、取出最老的一条。初始状态填零指令，
所以起步阶段车会"愣" \(d\) 帧——这部分也计入指标，不做特殊处理。

### 4.2 无补偿（`none`）

策略拿当前帧的雷达、按当前位姿做决策。这是绝大多数实现的实际状态。

### 4.3 匀速外推（`extrap`）

假设速度与角速度在 \(d\) 帧内不变，把**位姿**往前推，再把**当前扫描**按位姿差刚性搬过去：

$$
\hat{\mathbf{x}} = \mathbf{x} \oplus \big(v,\ \omega,\ d\Delta t\big),
\qquad
\hat r(\varphi) = r\big(\varphi + \Delta\theta\big) - \Delta s(\varphi)
$$

代价是一次积分 + 一次极坐标平移，对模型误差不敏感，也不假设任何地图。

### 4.4 模型前向 + 扫描变换（`model`，可上车）

Smith 预测器的标准做法：**用不含延迟的内部模型，把排队中的指令依次施加，算出指令真正生效时车在哪**。
时刻 \(t\) 算出的指令将在 \(t+d\) 生效，所以

$$
\hat{\mathbf{x}}(t+d) = f^{d}\Big(\mathbf{x}(t);\ \mathbf{u}(t-d+1),\dots,\mathbf{u}(t)\Big)
$$

\(f\) 就是 §2.2 的单轨模型。接着把手上**唯一**的观测（当前帧 24 束扫描）搬到 \(\hat{\mathbf{x}}\) 上：
先把每束还原成世界系点

$$
\mathbf{p}_j = \mathbf{p} + r_j\,\mathbf{e}\big(\theta+\varphi_j\big),\qquad j=1..24
$$

再用 \(\hat{\mathbf{x}}\) 的位姿写成极坐标 \((\rho_j,\beta_j) = \operatorname{polar}\big(\mathbf{R}(-\hat\theta)(\mathbf{p}_j - \hat{\mathbf{p}})\big)\)，
最后按极角线性插值重采样回 24 束：

$$
\hat r(\varphi_i) = \operatorname{interp}\Big(\varphi_i;\ \{(\beta_j,\rho_j)\}_{j=1}^{24}\Big)
$$

这一步是**真车做得到的**：不需要地图，也不需要仿真器，只要有里程计和当前帧扫描。

它有两个已知误差源，都写进了 §9：插值会把墙角"磨圆"，
而且**看不到新露出来的几何**（平移后本该出现的新遮挡关系被忽略，方向偏乐观）。

### 4.5 oracle 上界（`oracle`，需要地图）

允许直接向仿真器/精确地图索要 \(\hat{\mathbf{x}}\) 那一处的完整扫描：

$$
\hat r(\varphi_i) = \operatorname{raycast}\big(\hat{\mathbf{x}},\ \varphi_i\big)
$$

这是"假如我有一张完美地图"的结果，作为上界参考，**不是**一个可部署的方案。

### 4.6 四种方式只在两处不同

```python
q = query_pose(...)                     # 只决定"预测位姿"
if comp == "oracle":
    scan_q = scan_at(env, q)            # 需要地图：直接问仿真器
elif comp == "none":
    scan_q = scan_now                   # 不预测
else:                                   # extrap / model
    scan_q = warp_scan(scan_now, pose, q, env.beam_angles)   # 扫描刚性变换
```

## 5. ROS 2 封装

| 项目 | 内容 |
| --- | --- |
| 节点 | `latency_nav_node`（`lat_nav/node.py`） |
| 参数 | `shape`、`delay_ms`、`comp`（`none`/`extrap`/`model`/`oracle`）、`rate_hz`、`seed`（`config/params.yaml`） |
| 发布 | `/car/scan`（`sensor_msgs/LaserScan`）、`/car/odom`（`nav_msgs/Odometry`）、`/car/status`（JSON 字符串）、`/tf`（`map → base_link`） |
| 启动 | `ros2 launch mujoco_nn_latency_robustness main.launch.py` |

`/car/status` 每帧回一行 JSON，把这一帧的内部量都摊开，方便直接对上论文里的数字：

```json
{"step": 187, "shape": "kidney", "delay_ms": 600.0, "comp": "model",
 "rate_hz": 20.0, "e": 0.043, "psi": -0.01, "v_cmd": 0.98, "w_cmd": 0.06, "progress": 0.41}
```

消息头按仓库规范填：`LaserScan` 的 `angle_max = angle_min + (N-1)·increment`（不是 \(2\pi\)）、
`scan_time = 1/f`、`time_increment` 一并给出；`Odometry` 的 `frame_id=map`、`child_frame_id=base_link`。

## 6. 实验

环境：Windows 11 + Python 3.10 + `mujoco.__version__ == 3.17.0`（课程虚拟机 3.1.6 同样跑通）、
numpy 2.2。时间：`python main.py --demo` 全程约 3 min（CPU）。

### 6.1 上界参考：真值纯追踪

| 赛道 | RMS | 完成 |
| --- | --- | --- |
| circle | 0.041 m | 是 |
| kidney | 0.051 m | 是 |
| peanut | 0.030 m | 是 |

网络在 0 延迟下是 **0.035 m**（三赛道平均），已经和"看真值"的专家持平——这说明后面测到的
性能下降不是"网络不行"，而是延迟造成的。

!!! note "跨版本与跨机器"
    下表产自 Windows 11 + mujoco 3.17.0。同一份权重在课程虚拟机（mujoco 3.1.6）上重跑，
    18 个格子里 12 个逐位一致、5 个差在 0.003 m 以内、只有 1000 ms 外推那格差 0.010 m
    （0.416 / 0.426），完成率 50% / 67%——复现时看到这个量级的差属于正常。

### 6.2 延迟扫描（20 Hz，3 赛道 × 2 种子）

| 延迟 | 无补偿 | 匀速外推 | 模型前向（扫描变换，可上车） | oracle（需地图，只作上界） |
| --- | --- | --- | --- | --- |
| 0 ms | 0.035 m / 100% | 0.035 m / 100% | 0.035 m / 100% | 0.035 m / 100% |
| 200 ms | 0.037 m / 100% | 0.045 m / 100% | 0.046 m / 100% | 0.039 m / 100% |
| 400 ms | 0.087 m / 100% | 0.051 m / 100% | 0.066 m / 100% | 0.044 m / 100% |
| **600 ms** | **0.418 m / 67%** | 0.151 m / 100% | **0.092 m / 100%** | 0.048 m / 100% |
| **800 ms** | **0.440 m / 0%** | 0.269 m / 83% | **0.154 m / 100%** | 0.051 m / 100% |
| **1000 ms** | **0.457 m / 0%** | 0.420 m / 67% | **0.450 m / 0%** | 0.054 m / 100% |

每个格子是 RMS / 完成率，3 条赛道 × 2 个种子的平均。

![延迟对精度与完成率的影响](../../img/ground/lat_nav_delay_sweep.png)

读法：

1. **400 ms 是分水岭**。0～400 ms 三档的误差都在 0.1 m 以内（0.035～0.087 m）；
   200 ms 处补偿反而更差（0.045 / 0.046 对 0.037），因为扫描变换本身有插值误差，
   延迟太小时"补的"不如"错的"多——这是个诚实的负结果；到 400 ms 已经明显更好
   （0.051 / 0.066 对 0.087），但离 oracle 的 0.044 m 还有一点距离；
2. **600 ms 起无补偿直接崩**：误差跳到 0.418 m（车道半宽 0.75 m 的一半以上），
   碰撞帧 3184——不是撞一下，是**贴着墙蹭完整圈**；此时"完成率"仍可能报 100%，所以单看完成率会漏判；
3. **模型前向（扫描变换）在这一档最划算**：600 ms 0.418 → 0.092 m、800 ms 0.440 → 0.154 m，
   两次都是 100% 完成、0 帧碰撞；
4. **但它有天花板**：1000 ms 时预测位移 1 m，刚性变换 + 极角插值误差已经超过车道宽度，
   0.450 m / 0%——和"什么都不补"一样跑不完；
5. **oracle 全程平线**（0.035 → 0.054 m），说明"精确地图"这一条信息本身就值 0.4 m 的误差；
   它不是可部署方案，只是标出上限的参照物。

![同一 600 ms 延迟下的三条轨迹](../../img/ground/lat_nav_traces.png)

俯视图把差异摊得很清楚：红色（无补偿）在每个弯口都冲出去再拐回来，锯齿状；橙色（外推）
只在内侧切一点；蓝色（模型前向）几乎压在中心线上。

### 6.3 逐赛道明细（600 ms）

| 赛道 | 无补偿 | 匀速外推 | 模型前向（扫描变换） | oracle |
| --- | --- | --- | --- | --- |
| circle | 0.414 m / 100% / 892 帧碰撞 | 0.130 m / 100% / 0 | 0.096 m / 100% / 0 | 0.045 m |
| kidney | 0.426 m / 100% / 999 | 0.138 m / 100% / 0 | 0.101 m / 100% / 0 | 0.062 m |
| peanut | 0.413 m / **0%** / 1293 | 0.184 m / 100% / 26 | 0.080 m / 100% / 0 | 0.035 m |

碰撞帧是 2 个种子之和；"帧"指 20 Hz 控制周期，蹭墙时每帧都记一次。

花生形的曲率变化最剧烈（\(R_{\min}=1.73\) m），是唯一让无补偿策略**跑不完**的赛道；
三条赛道上模型前向（扫描变换）都能完成，误差 0.080～0.101 m，oracle 是 0.035～0.062 m
——剩下这 0.04 m 左右就是"没有地图"的代价。

### 6.4 控制频率扫描（延迟固定 600 ms）

| 频率 | 无补偿 | 匀速外推 | 模型前向（扫描变换） | oracle |
| --- | --- | --- | --- | --- |
| 10 Hz | 0.422 m | 0.167 m | 0.082 m | 0.053 m |
| 20 Hz | 0.418 m | 0.151 m | 0.092 m | 0.048 m |
| 50 Hz | 0.414 m | 0.141 m | 0.096 m | 0.045 m |

![控制频率的影响](../../img/ground/lat_nav_rate_sweep.png)

结论有点反直觉：**在延迟固定（毫秒计）的情况下，提高控制频率几乎没有帮助**（10 → 50 Hz
只把误差从 0.422 降到 0.414 m）。原因是延迟造成的相位滞后与采样率无关，
多算几帧只是把同样的"过期指令"更频繁地发出去。

有意思的是模型前向补偿反而随频率**略微变差**（0.082 → 0.096 m）：
频率越高，同样的 600 ms 对应更多帧，链式预测要连推的步数也更多，误差自然累积得更多。

!!! note "碰撞帧数不能横着比"
    50 Hz 下碰撞帧数最高（7573 vs 1760），但这**不代表更危险**：同一段时间里采样点数多了 5 倍。
    跨频率比较应当只看 RMS 与完成率；碰撞帧数只在同一频率内比较才有意义。

### 6.5 目标环境验证（Ubuntu 20.04 + ROS 2 Humble + mujoco 3.1.6）

课程虚拟机里（无外网，依赖走离线 manylinux wheel）实测：

```text
$ colcon build --packages-select mujoco_nn_latency_robustness --symlink-install
Finished <<< mujoco_nn_latency_robustness [2.75s]
Summary: 1 package finished [3.32s]

$ python3 -m pytest test -q
38 passed in 10.32 seconds

$ ros2 launch mujoco_nn_latency_robustness main.launch.py
[latency_nav_node-1] [INFO] 赛道=kidney 延迟=600 ms 补偿=model 频率=20 Hz

$ ros2 topic echo /car/status --once
data: '{"step": 331, "shape": "kidney", "delay_ms": 600.0, "comp": "model",
        "rate_hz": 20.0, "e": -0.006, "psi": 0.006, "v_cmd": 0.999, ...}'

$ ros2 topic hz /car/scan
average rate: 19.991

$ ros2 topic list | grep car
/car/odom  /car/scan  /car/status

$ ros2 run tf2_ros tf2_echo map base_link
- Translation: [-2.010, 3.212, 0.000]
- Rotation: in RPY (radian) [0.000, 0.000, -2.439]

Ctrl+C → [INFO] process has finished cleanly    （Traceback 0 次）
```

另外在干净目录（删掉 `results/`）里跑过 `python main.py --sweep`，exit 0 并生成两张图。

虚拟机里 `MUJOCO_GL` 用 `glfw` + `DISPLAY=:0`；本模块只用 `mj_ray` 与动力学，不需要离屏渲染，
因此不受"VMware 软件 OpenGL 缺 `ARB_clip_control`、深度图静默返回 inf"那个老问题影响（该问题见
`uav_mujoco_depth_avoidance` 的处理）。

## 7. 源码解析

### 7.1 模块分工

| 文件 | 职责 |
| --- | --- |
| `lat_nav/track.py` | 三条中心线、法向、等距边界点、MJCF 字符串生成 |
| `lat_nav/env.py` | MuJoCo 环境：位姿/速度、`mj_ray` 雷达、误差、控制周期推进 |
| `lat_nav/net.py` | 26→64→32→2 MLP，前向/反向/Adam/save/load |
| `lat_nav/nav.py` | 纯追踪专家、`DelayLine`、`warp_scan`、四种处理方式、`run_episode` 指标 |
| `lat_nav/train.py` | 专家采样、DAgger、评测汇总 |
| `lat_nav/sweep.py` | 实验矩阵与三张图 |
| `lat_nav/node.py` | ROS 2 节点（话题 + TF + 参数） |

### 7.2 场景不用资源文件

`track.build_xml()` 直接返回 MJCF 字符串，`mujoco.MjModel.from_xml_string()` 编译。
这样装进 ROS 包以后不存在"`models/` 在 `share/<pkg>/models` 下、`Path(__file__).parents[1]`
找不到"的问题——上一版模块在这里踩过坑，`ros2 launch` 直接 `ValueError`。
权重仍要放进包里，所以 `node._asset()` 保留三级查找（环境变量 → `share/<pkg>/models` → 源码目录）。

### 7.3 雷达排除自身

```python
self._bodyexclude = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "robot")
d = mujoco.mj_ray(model, data, pnt, vec, geomgroup, 1, self._bodyexclude, geomid)
```

不加 `bodyexclude` 时，射线起点在车壳内部，24 束全部读回 0.1 m 左右的自车距离，
`test_lidar_ignores_own_chassis` 就是为这个现象写的（断言 `scan().min() > 0.3`）。

### 7.4 延迟线

```python
class DelayLine:
    def push(self, cmd):
        if self.n <= 0:
            return cmd
        self.buf.append(cmd)
        return self.buf.pop(0)
```

`pending()` 把队列摊开给补偿器用——这正是 Smith 预测需要的"还在飞行中的指令"。

### 7.5 闭环主循环（`nav.run_episode`）

```
每个控制周期：
 1. 读位姿 x_t、速度 (v, ω)，并在当前位姿打一圈雷达（这一帧的"真实观测"）
 2. 按 comp 选择预测位姿 x̂：
      none   → x_t
      extrap → x_t ⊕ (v, ω, d·Δt)
      model  → f^d(x_t; 排队中的 d 条指令)
      oracle → 同 model
 3. 取观测：
      none          → 用当前帧扫描
      extrap/model  → warp_scan(当前帧扫描, x_t → x̂)   ← 真车做得到
      oracle        → 直接 raycast(x̂, φ_i)             ← 需要地图，只作上界
 4. 26 维特征 → 网络 → u_t
 5. u_t 入延迟线，取出 u_{t-d} 施加到环境
 6. 统计横向误差、最小净空、碰撞帧、进度
```

第 2～3 步是四种方式的唯一区别（`warp_scan` 见 §4.4），第 5 步是延迟的唯一入口——
所以对比是干净的。

## 8. 与已有模块的差异

| 仓库已有模块 | 与本模块的差别 |
| --- | --- |
| `lateral_control_benchmark` | 比的是纯追踪 / Stanley / LQR 三种**算法**，没有延迟这个自变量 |
| `pix2cmd_end2end_driving` | 比的是端到端**模型结构**（MLP/CNN 宽度），控制链路假设无延迟 |
| `mujoco_mppi_neural_tracker` | 比的是 MPPI 与蒸馏网络的**精度/算力**，关注噪声而非时延 |
| `mujoco_slam_nav_sync` | 关注建图与定位，不是控制回路的时延 |
| `uav_rl_benchmark` | 主成分是 RL 算法与网络宽度，且不含执行延迟 |

本模块的贡献是**把"时延"当成唯一自变量**，并给出一份可复现的补偿对照。

## 9. 局限

1. 执行器用理想速度伺服 + `bodyexclude` 雷达，没有轮胎侧偏、打滑、扭矩饱和；
   真实车辆上延迟与动力学耦合更强，结论只能作为**下界**参考。
2. **`model` 用的是扫描刚性变换**：把当前帧点云搬到预测位姿、再按极角插值重采样。
   它不需要地图，但有两个固有误差——插值会把墙角磨圆；**看不到新露出来的几何**
   （平移后本该出现的新遮挡被忽略，方向上偏乐观）。1000 ms 那档的崩溃（0.450 m / 0%）
   就是这两个误差累积的结果，说明**无地图补偿的有效范围大约到 800 ms 为止**。
3. **`oracle` 需要精确地图**（直接向仿真器要预测位姿处的扫描），只作为上界，
   不代表可部署方案。它和 `model` 之间的差值（约 0.04 m）就是"有图 vs 没图"的代价。
4. 只研究了**执行延迟**，没有研究**感知延迟**（图像老化）与**抖动**（jitter，非固定延迟）。
   固定延迟与抖动的补偿方式不同：后者需要预测分布而不是单点。
5. Smith 预测依赖内部模型准确。本实验里内部模型与真实环境完全一致（都是单轨模型），
   因此这一部分仍是**乐观**结果；模型失配时的鲁棒性没有测。
6. 网络在三条赛道上一起训（3600 条专家 + 600 条 DAgger），换赛道仍需重新采样。
7. 只做了 2 个随机种子 × 3 条赛道；论文级结论需要更多种子与更多延迟取值。

## 10. 大模型使用声明

本模块的代码骨架、实验设计、数值与文档由作者与大模型（Hermes Agent / DeepSeek）协作完成：
模型负责起草代码与文档、跑实验、记录数字；实验口径（延迟定义、补偿方式、指标）与最终结论
由作者确认。仓库内所有数字均由 `python main.py --demo` 复现产出（脚本输出与 `results/metrics.json`）。
