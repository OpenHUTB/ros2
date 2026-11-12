# 控制：模仿学习速度控制器

`ca_drone/models/control.py` · `ControlNet`
教师：`ca_drone/control/expert.py`

## 1. 任务

给定规划路径、局部障碍分布与自身运动状态，输出机体系速度指令

\[
\mathbf{u}_k=(v_x,v_y)=\pi_\theta\!\left(\mathbf{o}_k,\,
\mathbf{g}^{\text{body}}_k,\,\mathbf{v}^{\text{body}}_k,\,d_k\right),
\qquad \|\mathbf{u}_k\|_\infty\le1.5\ \text{m/s}
\]

其中 \(\mathbf{o}_k\) 是 9×9 局部占用栅格，\(\mathbf{g}^{\text{body}}\) 是
lookahead 目标在机体系下的坐标，\(\mathbf{v}^{\text{body}}\) 是当前速度，
\(d_k\) 是到最近障碍的间隙。

## 2. 教师：几何专家

控制网络通过模仿一个几何控制器学习。教师由两部分组成：

### 2.1 纯追踪（pure pursuit）

在规划折线上找当前投影点，再沿折线前进 \(L=2.2\ \text{m}\) 得到预瞄点
\(\mathbf{p}^{\text{la}}\)，期望速度：

\[
\mathbf{v}^{\text{des}}=\frac{\mathbf{p}^{\text{la}}-\mathbf{p}}{\|\mathbf{p}^{\text{la}}-\mathbf{p}\|}\cdot v_{\text{cruise}},
\qquad v_{\text{cruise}}=1.35\ \text{m/s}
\]

```python
def lookahead_point(path, pos, lookahead=2.0):
    for j in range(len(path) - 1):          # 对每段求投影
        t = clip((p - a) @ ab / |ab|^2, 0, 1)
        ...
    return 从投影点沿折线前进 lookahead 米
```

早期实现“取最近路点再往前数几个点”，在 90° 门洞处会直接切角撞墙；
改为**折线投影**后，预瞄点始终落在路径上，这是走廊场景能通过的关键。

### 2.2 切向斥力

对每个距离 \(d<\rho=2.8\ \text{m}\) 的障碍，产生切向避让速度：

\[
\mathbf{v}^{\text{rep}}_j =
\kappa\left(\frac{1}{d_j}-\frac{1}{\rho}\right)
\left(0.85\,\hat{\mathbf{t}}_j+0.35\,\hat{\mathbf{n}}_j\right),
\qquad \kappa=1.5
\]

其中 \(\hat{\mathbf{n}}_j\) 是从障碍指向无人机的单位向量，\(\hat{\mathbf{t}}_j\)
是与期望方向同侧的切向量（保证“绕过去”而不是“顶上去”）。
总指令 \(\mathbf{v}^{\text{des}}+\sum_j\mathbf{v}^{\text{rep}}_j\) 限幅到 1.45 m/s。

## 3. 为什么需要 DAgger 式增广

第一版数据只沿专家轨迹采样，标签几乎恒为 \((1.35,0)\)：

| 统计量 | 旧数据 | 增广后 |
| --- | ---: | ---: |
| 样本数 | 5,760 | 15,305 |
| \(v_y\) 范围 | \([-0.33,0.26]\) | \([-1.45,1.45]\) |
| \(v_y\) 标准差 | 0.071 | 0.65 |
| 目标侧向偏移 \(y_b\) 范围 | \([-0.21,0.25]\) | \([-0.84,0.88]\)（归一化） |

网络因此退化成“恒定前进”，在走廊里对大侧向误差毫无反应：

```text
目标 body=(-1.8,-3.3) → 网络输出 (1.1, 0.0)   # 完全不转向
```

修复方法是 DAgger 式**目标增广**：每一步除了原 lookahead 点，再随机采样
2 个目标（\(\pm2.8\ \text{m}\)，要求在世界内且间隙 >0.6 m），
用教师重新标注。这样同一帧局部栅格会对应“向前/向左/向后”多种标签，
网络被迫学会根据目标方向产生侧向速度。

## 4. 网络

| 层 | 输入 | 输出 | 激活 |
| --- | ---: | ---: | --- |
| fc1 | 86 | 128 | ReLU |
| fc2 | 128 | 128 | ReLU |
| out | 128 | 2 | \(\tanh\times1.5\) |

输入向量拼接顺序（`control_features`）：

\[
\left[\frac{x_b^g}{5},\frac{y_b^g}{5},
\frac{v_x^b}{1.5},\frac{v_y^b}{1.5},
\text{vec}(\mathbf{o})_{81},
\frac{\min(d,3)}{3}\right]
\]

参数 **27,906**；输出层权重乘 0.1、偏置置零，使初始策略接近悬停，
避免训练初期“满速乱飞”。

## 5. 训练

| 项目 | 设置 |
| --- | --- |
| 样本 | 15,305（36 回合 + 每步 2 个增广目标） |
| 损失 | MSE |
| 优化器 | Adam，lr \(10^{-3}\)，batch 128 |
| epoch | 60 |
| 验证 | 15%：MAE **0.0356 m/s**，角度误差 **1.89°** |

## 6. 闭环结果（真值地图 + A\*，隔离控制误差）

| 场景 | 成功 | 步数 | 最小间隙 (m) | 路径 (m) | 推理 (ms) |
| --- | --- | ---: | ---: | ---: | ---: |
| sparse | ✅ | 445 | 2.05 | 53.2 | 0.10 |
| dense | ✅ | 432 | 2.07 | 51.6 | 0.05 |
| dynamic | ✅ | 451 | 1.40 | 53.9 | 0.03 |
| corridor | ✅ | 431 | 2.05 | 51.4 | 0.05 |
| unseen（留出） | ✅ | 486 | 2.05 | 58.0 | 0.04 |

命令：`python main.py --module control --scene <scene>`。
这一组实验故意使用**真值地图**，把感知与规划误差排除在外，
因此 5/5 成功说明控制网络本身能够跟踪路径；全链路（含感知）的结果
见 [性能评价](results.md)。

## 7. 源码解析

```python
class ControlNet(Module):
    def forward(self, x):
        h = self.fc1(x).relu()
        h = self.fc2(x).relu()
        return self.out(h).tanh() * MAX_SPEED
```

- 两层 128 宽 MLP 足以拟合“局部栅格 → 速度”的映射；更深的网络在
  15k 样本上开始过拟合；
- `tanh` 保证输出有界，避免网络在分布外状态给出爆炸指令；
- 推理只做一次矩阵乘，CPU 上 0.03–0.1 ms，比规划快两个数量级。

闭环里还有两条工程保护（`pipeline.py`）：

```python
n = float(np.linalg.norm(cmd))
if n > MAX_CMD:                 # 1.2 m/s
    cmd = cmd * (MAX_CMD / n)
if clear < 0.6:                 # 离障碍太近时减速
    cmd = cmd * 0.75
```

它们不改变网络输出，只做安全限幅；消融时可关闭。

## 8. 入口

```bash
python main.py --module control --scene corridor
python main.py --module control --scene sparse --gif
```
