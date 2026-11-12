# line_follower_car

纯 NumPy + Matplotlib 的 **巡线小车** 二维仿真：黑色引导线构成一条带内凹弯的
闭合赛道，小车车头一排虚拟光传感器读取压线情况，控制器沿路径取预瞄目标点
（pure pursuit 思想），按朝向误差 PD 控制转向角速度沿中线巡线。不依赖
ROS / Gazebo / CARLA / MuJoCo。

对应文档：[`docs/ground/line_follower_car.md`](../../../docs/ground/line_follower_car.md)

## 文件

```
main.py                              赛道采样、小车模型、PD 跟踪、渲染与 CLI
requirements.txt                     numpy、matplotlib
tests/test_main.py                   路径采样、贴线跟踪、脱线判定、确定性回归测试
```

## 环境

- Python 3.8+

```bash
pip install -r requirements.txt
```

## 运行

在仓库根目录执行：

```bash
python src/ground/line_follower_car/main.py --demo --outdir demo_out
```

`--demo` 跑完一圈并保存 `line_follower_car_track.png` 到 `demo_out/`。

无窗口评测：

```bash
python src/ground/line_follower_car/main.py --headless --duration 90 --outdir demo_out
```

## 控制原理

1. 在采样路径上沿前进方向取第 `LOOKAHEAD` 个点作为预瞄目标；
2. 目标方位角 `α = atan2(ty - y, tx - x)`，朝向误差 `err = normalize(α - θ)`；
3. 角速度 `ω = Kp·err + Kd·derr/dt`；
4. 独轮运动学积分 `x += v·cosθ·dt`，`y += v·sinθ·dt`，`θ += ω·dt`。

车顶五个传感器横向分布在 `-0.9 … 0.9 m`，压在线上显示绿色，否则红色。

## 指标

`metrics.json` 字段：

| 字段 | 含义 |
|---|---|
| `steps` | 仿真步数 |
| `path_length` | 累计行驶里程（米） |
| `mean_abs_offset` | 平均绝对横向偏差（米） |
| `max_abs_offset` | 最大绝对横向偏差（米） |
| `offline` | 是否持续脱线（到路径距离超阈值 15 步） |
| `finished` | 是否跑完一圈 |

## 测试

```bash
python -m unittest discover -s src/ground/line_follower_car/tests -v
```

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `SPEED` | 4.0 m/s | 巡线速度 |
| `KP` / `KD` | 1.8 / 0.25 | 朝向误差比例 / 微分增益 |
| `LOOKAHEAD` | 8 点 | 沿路径向前看的采样点数（约 4 m） |
| `OFFLINE_E` | 1.5 m | 脱线距离阈值 |
| `DT` | 1/30 s | 固定仿真步长 |

## 限制

- 赛道为预设折线，未接入真实摄像头图像；传感器为几何距离判定。
- 直角弯处横向偏差峰值较大，靠预瞄点提前转向，不做车速自适应减速。
- 独轮运动学不阿克曼转向半径约束。
