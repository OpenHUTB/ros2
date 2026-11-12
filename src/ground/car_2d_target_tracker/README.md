# car_2d_target_tracker

纯 Pygame 的 2D 小车目标追踪仿真：小车自动追一个随机游走的目标，支持空格切换
手动 WASD 驾驶、鼠标设置目标、静态障碍物碰撞，以及无窗口批量评测。

对应文档：[`docs/ground/car_2d_target_tracker.md`](../../../docs/ground/car_2d_target_tracker.md)

## 文件

```
main.py                 仿真、绘制、键盘/鼠标交互、CLI
tests/test_main.py      角度归一化、追踪、碰撞、绕障、目标反弹、headless 确定性
requirements.txt        pygame
```

## 环境

- Python 3.8+
- pygame >= 2.5.2（本机用 Python 3.13 + pygame 2.6.1 验证；2.5.2 的 API 兼容）

```bash
pip install -r requirements.txt
```

## 运行

在仓库根目录执行。命令里的 `python` 在 Linux/macOS 上通常写成 `python3`，
Windows 也可以用 `py -3`：

```bash
python src/ground/car_2d_target_tracker/main.py
```

Windows 终端里中文显示乱码时，先执行 `chcp 65001`。

演示模式（自动跑三个场景并保存截图，默认写到 `demo_out/`）：

```bash
python src/ground/car_2d_target_tracker/main.py --demo
```

无窗口评测（CI / 服务器 / 无显示器机器可用，不打开窗口）：

```bash
python src/ground/car_2d_target_tracker/main.py --headless --duration 20 --seed 42 --outdir demo_out
```

输出 `demo_out/metrics.json`，并打印一行汇总：

```
steps=1200 time=20.00s arrived_once=True t_target=14.70s
mean_dist=33.8px final_dist=20.2px path=1422.1px hits=0
```

加 `--save-frame` 会在结束时保存 `demo_out/final_frame.png`。

## 操作

| 按键 | 功能 |
|---|---|
| 空格 | 自动模式 / 手动模式切换 |
| 鼠标左键 | 自动模式下，点击位置设为新目标 |
| W / ↑ | 手动模式前进 |
| S / ↓ | 手动模式倒车 |
| A / D 或 ← / → | 手动模式转向 |
| R | 小车重置到窗口中央 |
| ESC | 退出 |

## 指标

`--headless` 写入 `metrics.json`：

| 字段 | 含义 |
|---|---|
| `steps` | 仿真步数（固定 60 Hz） |
| `arrived` | 最后一帧是否在 18 px 到达半径内（目标移动后会变） |
| `arrived_once` | 是否至少进入过一次 18 px 到达半径 |
| `time_to_target` | 第一次到达的时间（秒），没到达为 `null` |
| `mean_distance` | 全程与目标距离的平均值 |
| `final_distance` | 结束时与目标的距离 |
| `path_length` | 小车实际走过的路径长度 |
| `collisions` | 碰撞事件次数（一次持续接触只记一次） |

固定 `--seed` 后，`metrics.json` 可完全复现。

## 测试

```bash
python -m unittest discover -s src/ground/car_2d_target_tracker/tests -v
```

应显示 `Ran 17 tests ... OK`。测试使用 `SDL_VIDEODRIVER=dummy`，不会弹窗。

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `MAX_SPEED` | 180 px/s | 自动模式最大速度 |
| `MAX_STEER` | 3.5 rad/s | 自动模式最大转向角速度 |
| `KP_TURN` | 2.5 | 朝向误差的比例增益 |
| `SLOWDOWN_DISTANCE` | 60 px | 距目标多近开始线性减速 |
| `ARRIVE_RADIUS` | 18 px | 到达判定半径 |
| `MANUAL_SPEED` | 160 px/s | 手动模式速度 |
| `FIXED_DT` | 1/60 s | headless/demo 固定步长 |
| `ESCAPE_SPEED_RATIO` | 0.85 | 被障碍挡住时沿切向绕行的速度比例（0 = 旧行为：停下原地打转） |
| `ESCAPE_HOLD_S` | 0.5 s | 选定绕行方向后至少保持多久，避免两侧来回抖 |
| `STUCK_TIME_S` | 1.0 s | 对准切向仍走不动多久就翻到另一侧绕 |
| `COLLISION_HYSTERESIS_PX` | 4 px | 离开障碍表面超过这么多像素才算解除接触（碰撞计数用） |

## 限制

- 自动模式是贪心追踪，没有全局路径规划。障碍正挡在目标方向上时会沿障碍表面
  切向绕行（见下），能绕开凸形障碍，但**凹形障碍群**可能来回试探。
- 目标不避障，可能穿过障碍物。
- 窗口固定 900×600，没有缩放。
- 碰撞检测只检查小车新位置，不检查车身扫过的区域。

## 撞到障碍后的绕行

旧实现撞上后把速度清零、只按最大角速度原地转。障碍正压在"车 → 目标"连线上时
会一直贴墙转圈：实测（车 (120,300)、目标 (800,300)、半径 90 px 的圆正挡中间、30 s）
**95.6% 的时间速度低于 5 px/s，末距始终 440 px，永远到不了**。

现在改为沿障碍表面切向绕行：取径向 ±90° 的两条切向，选与目标方位夹角更小的一侧，
以 `0.85 × MAX_SPEED` 前进；贴得太紧切向仍会撞时先沿径向外退一点再转车头
（否则车会被"正好贴在碰撞边界"钉死）；对准切向仍走不动超过 1 s 就翻到另一侧。
同一场景改后 **5.45 s 绕过并到达**，卡死占比 14.7%，全程未穿入障碍。
