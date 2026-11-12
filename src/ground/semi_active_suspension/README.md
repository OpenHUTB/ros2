# semi_active_suspension

半主动悬架车身姿态控制仿真：7 自由度整车垂向模型（车身垂向 / 俯仰 / 侧倾 + 四个
非簧载质量独立垂向），四个减振器按天棚 + 地棚混合律在线调节阻尼，对比「被动
减振器 / 半主动」在过减速带、急刹点头、过弯侧倾与 ISO 8608 C 级随机路面四个工况
下的差别。

除 ROS 2 节点外只用 Python 标准库（含 PNG 编码），没有 numpy 也能跑。

## 快速开始

```bash
cd src/ground/semi_active_suspension
python3 -m semi_active_suspension.main --output results
python3 -m pytest tests -q      # 38 项
```

跑完四个工况，生成三张对比曲线 + 一张指标汇总图，同时打印指标表。
只想看一个工况就加 `--scenario speed_bump`。

| 工况 | 指标 | 被动 | 半主动 | 变化 |
|---|---|---|---|---|
| 过减速带（20 km/h，50 mm） | 车身加速度峰值 | 5.95 m/s² | **5.40 m/s²** | −9.3% |
| | 车身加速度 RMS | 1.246 m/s² | **1.070 m/s²** | −14.2% |
| 急刹（100 km/h，0.6 g） | 俯仰角峰值 | 2.08° | **1.74°** | −16.3% |
| | 车身加速度 RMS | 0.0184 m/s² | **0.0144 m/s²** | −21.7% |
| 过弯（80 km/h，0.4 g） | 侧倾角峰值 | 2.69° | **2.44°** | −9.2% |
| | 最小轮荷 | 2718 N | **2758 N** | +1.5% |
| ISO 8608 C 级（80 km/h） | 车身加速度 RMS | 1.170 m/s² | **1.074 m/s²** | −8.2% |
| | 最小轮荷 | 1221 N | **1292 N** | +5.8% |

前三个工况改善的其实都是**超调量**，不是稳态值。制动点头的稳态俯仰角由弹簧刚度
和质心高度决定，两种减振器下都收敛到 1.62°（表格里的 2.08° / 1.74° 是峰值）。
半主动能做的只是在车身压下去的过程中多给一点阻尼、把过冲削掉一点，这在
`docs/ground/semi_active_suspension.md` 里有单独一段说明。

## 模型

7 个自由度：车身垂向 $z_s$、俯仰 $\theta$、侧倾 $\varphi$，以及四个非簧载质量的
垂向位移 $z_{u,i}$。车轮编号 FL / FR / RL / RR，$a_i$ 为纵向位置（前正），$b_i$ 为
横向位置（左正）。

单个角的悬架力（压缩为正）：

$$
F_i = k_i\left(z_{u,i} - z_{s,i}\right) + c_i\left(\dot z_{u,i} - \dot z_{s,i}\right),
\qquad z_{s,i} = z_s - a_i\theta + b_i\varphi
$$

三个车身自由度：

$$
\begin{aligned}
m_s\ddot z_s &= \sum_i F_i \\
I_y\ddot\theta &= \sum_i \left(-a_i F_i\right) + m_s\left(-a_x\right)h \\
I_x\ddot\varphi &= \sum_i \left(b_i F_i\right) + m_s a_y\left(h - h_{rc}\right)
\end{aligned}
$$

轮胎只保留垂向刚度与很小的阻尼，并且**只能受压**：

$$
F_{t,i} = \max\left(0,\ F_{z0} + k_t\left(z_{r,i} - z_{u,i}\right) + c_t\left(\dot z_{r,i} - \dot z_{u,i}\right)\right) - F_{z0}
$$

先夹总量再减静态轮荷，这样车轮离地时动载停在 $-F_{z0}$，不会出现"地面把车轮往下拽"
的负刚度假象。整车参数取 1.5 t 紧凑型轿车：簧上 1450 kg、非簧载 42 kg、轴距 2.5 m、
前/后弹簧 32 / 29 kN·m⁻¹、轮胎 210 kN·m⁻¹。

路面写成空间函数 $z_r(x)$，四个轮子按各自纵向位置取同一张路面，前后轮的时间差
$L/v$ 自然产生，不需要额外加延时。随机路面按 ISO 8608 用 200 项正弦叠加合成，
相位由固定 seed 生成，所以每次跑出来的曲线完全一样。

积分用定步长 RK4，`DT = 0.5 ms`；步长减半后峰值加速度变化小于 1%（有测试守着）。

## 控制律

半主动减振器只能耗能、不能出力，所以只能在"当前相对速度方向允许"的范围内改阻尼。
每个角分别算两个期望值再取大：

$$
c_i = \mathrm{clamp}\left(\max\left(c_{\text{sky}},\ c_{\text{gh}}\right),\ c_{\min},\ c_{\max}\right)
$$

$$
c_{\text{sky}} =
\begin{cases}
K_{\text{sky}}\dfrac{|\dot z_{s,i}|}{|\dot z_{u,i} - \dot z_{s,i}|}, & \dot z_{s,i}\left(\dot z_{u,i} - \dot z_{s,i}\right) < 0 \\[4pt]
0, & \text{其它}
\end{cases}
\qquad
c_{\text{gh}} =
\begin{cases}
K_{\text{gh}}\dfrac{|\dot z_{u,i}|}{|\dot z_{u,i} - \dot z_{s,i}|}, & \dot z_{u,i}\left(\dot z_{u,i} - \dot z_{s,i}\right) > 0 \\[4pt]
0, & \text{其它}
\end{cases}
$$

条件的意思是：减振器出力方向恰好顶住车身时走天棚（压舒适性），恰好压住车轮时走
地棚（压轮胎动载）；两个条件都不满足就退到最小阻尼 $c_{\min} = 500$ N·s/m，
因为这时候想要的力根本产生不出来，硬给阻尼只会把冲击传到车身上。

阻尼阀不是瞬间到位的，加了一阶跟随：

$$
c_i \leftarrow c_i + \left(c_i^{\text{target}} - c_i\right)\frac{\Delta t}{\tau},
\qquad \tau = 20\ \text{ms}
$$

没有这一层的话控制器会在 $c_{\min}$ 和 $c_{\max}$ 之间每个积分步硬切，曲线全是锯齿。

$K_{\text{sky}} = 4200$、$K_{\text{gh}} = 1200$ N·s/m 是标定出来的折中。把 $K_{\text{gh}}$
调到 0，舒适性还能再好一截（减速带峰值 4.99、随机路面 RMS 0.94），代价是随机路面上
最小轮荷从 1292 N 掉到 681 N —— 接地性换舒适性，这是半主动悬架绕不开的一条权衡线，
标定表在站点文档里。

## ROS 2 节点

```bash
# 只起节点，不开 RViz2（默认）
ros2 launch semi_active_suspension main.launch.py scenario:=speed_bump

# 要画面时再加 rviz:=true
ros2 launch semi_active_suspension main.launch.py scenario:=speed_bump rviz:=true
```

`rviz` 默认是 `false`：无显示环境（CI、容器、WSL 没配 X）里 RViz2 找不到
`libOgreMain.so` 会直接 exit 127，那不是本节点的问题，但一起启动会让人以为节点挂了。
先不加 `rviz` 确认 `/suspension/markers` 有输出，再开界面更省事。

RViz2 里同时回放两排：上排被动、下排半主动，各画车身参考面、四个减振器、四个车轮
和脚下正在向后滚动的路面；最前面一排竖条是四个减振器此刻的阻尼系数，颜色从蓝（最软）
到橙（最硬）。车身固定在世界原点，路面按行驶里程滚动，看的不是"车跑到哪"而是
"车压过什么"。

参数：`scenario`、`playback_rate`、`frame_step`、`height_gain`（垂向位移放大倍数，
默认 4，悬架行程只有几十毫米，不放大在 RViz 里看不出来）、`row_gap`。

话题 `/suspension/markers`（MarkerArray）、`/suspension/status`（String），服务
`/suspension/replay`（Trigger）。

## 文件

```
semi_active_suspension/
├── params.py      整车参数、状态量、阻尼配置
├── suspension.py  7 自由度动力学、天棚/地棚控制律、轮胎力
├── road.py        平路、单个减速带、ISO 8608 随机路面
├── scenarios.py   四个工况
├── sim.py         RK4 主循环与指标
├── render.py      索引色画布 + 5x7 点阵字 + PNG 编码（纯标准库）
├── scene.py       四张图
├── ros_node.py    RViz2 回放节点
└── main.py        命令行入口
```

## 已知局限

- 轮胎是单点接地模型，没有包络效应。压过 50 mm 减速带时前轴轮荷会冲到 20 kN
  （静态 8 kN），真实轮胎因为胎面把短波"抹平"了，峰值没这么夸张。
- 只有垂向动力学，没有侧向力、纵向力与垂向的耦合，所以过弯侧倾是准静态的侧向
  加速度激励出来的，不涉及载荷转移对侧偏刚度的影响。
- 减振器特性画成理想的可调阻尼，没有考虑压缩/复原行程的不对称和温度衰减。
