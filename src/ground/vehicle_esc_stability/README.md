# vehicle_esc_stability

车辆 ESC（电子稳定控制）仿真：双轨车辆动力学 + 简化魔术公式轮胎，用横摆角速度跟踪
与侧偏角约束做单侧差动制动，对比四个工况下「有 ESC / 无 ESC」的差别。

除 ROS 2 节点外只用 Python 标准库（含 PNG / GIF 编码），没有 numpy 也能跑。

## 快速开始

```bash
cd src/ground/vehicle_esc_stability
python3 -m vehicle_esc_stability.main --output results
python3 -m pytest tests -q      # 41 项
```

会跑完四个工况，输出轨迹对比、状态曲线、对开路面制动、指标汇总四张图和一段动画，
同时打印一张指标表：

| 工况 | 最大侧偏角 | 最大路径偏离 | 甩尾 |
|---|---|---|---|
| 双移线（96 km/h，μ=0.9） | 89.2° → **3.8°** | 17.02 m → **0.59 m** | 是 → 否 |
| 正弦停滞（100 km/h，0.5 Hz） | 4.7° → 4.3° | — （横摆误差 9.20 → **3.02** °/s） | 否 → 否 |
| 对开路面制动（μ 0.95 / 0.30） | 89.0° → **0.4°** | — （停车距离 57.6 → 169.5 m） | 是 → 否 |
| 湿滑双移线（65 km/h，μ=0.5） | 36.0° → **3.3°** | 10.08 m → **0.38 m** | 是 → 否 |

「→」左边是关掉 ESC，右边是打开。

对开路面那一行要单独看一眼：侧偏角从 89.0° 压到 0.4°，停车距离从 57.6 m 变成
169.5 m。这不是调参失败，是 select-low 的必然结果 —— ESC 优先保住方向稳定性，
宁可把高压侧的制动力压到和低压侧一样，也不让车被拽偏。两个数不能直接相减当损失：
没 ESC 那次是横着滑过去被侧向阻力拽停的（车转了 89°，占三条车道），ESC 那次是
直线停住。真实车上驾驶员会配合打方向，距离增幅没这么夸张（见下文）。

双移线那一行也不是"ESC 顺便把车刹慢了"：默认配置跑完 95.62 km/h，只留横摆差动制动
同样是 95.62 km/h，稳定性来自横摆力矩而非减速。拆开两半的消融表在站点文档里。

## 车辆模型

平面三自由度（纵向、侧向、横摆）+ 四个车轮各自的侧偏角与法向力，所以左右附着不同
时能自然产生横摆力矩：

$$
\begin{aligned}
\dot u &= \frac{\sum F_x}{m} + v r \\
\dot v &= \frac{\sum F_y}{m} - u r \\
\dot r &= \frac{\sum M_z}{I_z}
\end{aligned}
$$

轮胎侧向力用简化魔术公式，$B=9.5$、$C=1.35$，峰值侧偏角
$\arctan(\tan(\pi/2C)/B) \approx 13.7°$。实车轮胎一般在 6°～8°，这里偏大是因为 $B$
取得小，轮胎被写得「更宽容」，绝对数值别直接对标实车（结论不受影响，见站点文档里的说明）。

$$
F_y = -\mu F_z \sin\left(C \arctan(B\alpha)\right)
$$

法向力按准静态载荷转移算：纵向按 $m a_x h / L$ 在前后轴之间转移，侧向按侧倾刚度比
（前 55%）分配到左右轮。附着椭圆限制纵向与侧向力的合成不超过 $\mu F_z$。

制动力按**理想 ABS**处理：制动时纵向附着只用掉峰值的 $k_{\mathrm{ABS}}$ 倍（0.90，
代码里的 `TireSpec.abs_ratio`），剩下的留给侧向：

$$
F_x = -b \cdot \min\left(F_{b,\max},\ k_{\mathrm{ABS}}\,\mu F_z\right),\qquad b \in [0,1]
$$

$F_{b,\max}$ 是制动器硬件的力上限，$k_{\mathrm{ABS}}\mu F_z$ 是轮胎这一端能承受的纵向
附着上限，取两者较小的那个；$b$ 是踏板指令，实际输出在上限之内按比例给。

不做这一条，全力制动时纵向力会把附着吃干，侧向力等于零，对开路面上 ESC 怎么打方向
都没用 —— 早期版本就是这样，加 ESC 反而更容易甩。

积分用四阶 Runge–Kutta，步长 2 ms；ESC 与驾驶员 50 Hz 决策，20 Hz 记录一帧。

## ESC 控制律

参考横摆角速度取稳态运动学值，并用附着能提供的侧向加速度封顶：

$$
r_{\text{ref}} = \operatorname{clamp}\left(\frac{u \tan\delta}{L(1+Ku^2)},\ \pm\frac{k_{\text{ref}}\,\mu g}{u}\right)
$$

$k_{\text{ref}}=0.9$ 是参考值的安全系数（`EscConfig.reference_margin`），和上面的
$k_{\mathrm{ABS}}$ 不是一回事。

误差 $e = r_{\text{ref}} - r$ 与侧偏角超限一起折算成需要的修正横摆力矩：

$$
d = k_p e + k_i \!\int\! e\,dt + k_d \dot e + k_\beta \beta_{\text{excess}} - k_{\mu}\,\delta_b (\mu_L - \mu_R)
$$

最后一项是左右附着差的前馈：对开路面上驾驶员一踩刹车，车就被高附着侧拽着偏，
先用前馈抵掉大部分，积分项只负责收尾。没有这一项时，积分要在 1 s 内把误差从
10.7 °/s 压到 0，饱和之后反而失控。

力矩方向决定压哪一侧：需要左转力矩就压左侧两轮、松右侧，反之亦然。驾驶员已经在
全力制动（> 0.4）时只减压不加压 —— 低附着侧再加压只会让它先锁死。侧偏角超过 5.7°
时四轮一起补制动、同时请求降扭，把车速拉回附着能力以内。

## 为什么对开路面的制动距离变长

ESC 在对开路面上做的是「select-low」：为了让左右纵向力相等、不产生横摆力矩，
高压侧只能压到和低压侧一样的制动力。100 km/h 在 μ=0.95/0.30 上刹停，制动力被
0.30 那一侧拖住，直线停住要走 169.5 m；不带 ESC 那一组因为甩尾横滑、被车身侧面的
阻力拽停，反而只用了 57.6 m，但车是横着停在路中间的。

停车距离用**合速度**判定（$\sqrt{u^2+v^2} < 0.5$ m/s），不是纵向速度 —— 横滑时
纵向速度会先过零，按它判会把「还在 61 km/h 侧滑」的车记成已停车。

真实车上也是这个取舍，只是驾驶员会配合打方向，ESC 不必把横摆力矩压到零，所以距离
增加得没有这么夸张。这里没有给对开路面工况配转向驾驶员（`steer_fn` 恒为 0），
ESC 只能靠制动把车扶正，距离代价就被放大了。**这正是这个工况想说明的事：**
稳定性是买来的，代价是制动距离。

## ROS 2

```bash
ros2 launch vehicle_esc_stability main.launch.py
ros2 launch vehicle_esc_stability main.launch.py scenario:=mu_split
```

| 话题 | 类型 | 说明 |
|---|---|---|
| `/esc_stability/markers` | `visualization_msgs/MarkerArray` | 车道、参考线、轨迹、车辆、文字 |
| `/esc_stability/status` | `std_msgs/String` | `t=..s ESC off beta ../on beta ..` |
| `/esc_stability/replay` | `std_srvs/Trigger` | 从头重放 |

RViz2 里上下两条车道并排跑同一条路，上车道 ESC 关、下车道 ESC 开，两辆车同时出发。
一眼能看出无 ESC 的车在哪一刻甩出去、有 ESC 的车跟参考线贴得有多紧。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `scenario` | `lane_change` | `lane_change` / `sine_dwell` / `mu_split` / `wet_lane_change` |
| `playback_rate` | `1.0` | 回放倍速 |
| `frame_step` | `4` | 每个周期跨过的仿真帧数（单帧 0.05 s） |
| `lane_gap` | `18.0` | 两条车道的间距，m |

车辆与 ESC 参数在 `vehicle_esc_stability/params.py` 里，都是 dataclass，改动不需要
碰控制代码。默认值是一辆 1.5 t 前驱紧凑型轿车：轴距 2.5 m、质心高 0.55 m、
$I_z = 2500\ \text{kg·m}^2$、轮距 1.55 m。

## 代码检查与许可

风格检查用 `ament_flake8`（99 列），测试放在 `test/test_flake8.py` —— 是 `test/` 不是
`tests/`，所以 `python3 -m pytest tests -q` 不会收它，只有 `colcon test` 才会跑。
这么放是为了不把 lint 依赖带进纯离线的单元测试：只装 Python 标准库 + pytest 也能跑完
41 项结果测试。

没接 pep257 和 copyright 检查：本包的 docstring 是中文，pydocstyle 的 D400 要求首行以
英文句号结尾，中文的「。」直接就不合规，要么改一整套 docstring、要么关掉检查，两条都不划算，
所以没有声明。

许可证是 MIT，见同目录 `LICENSE`。
