# narrow_giveway_passing

纯 NumPy + Matplotlib 的 **窄道会车让行** 二维仿真：一条双向共用的单车道窄路，
路侧设有两处避让龛，两辆对向小车按“谁先靠近自己一侧的避让龛，谁进龛等待”
的规则会车——让行车驶入避让龛停稳，对向车直行通过，交汇完成后让行车再驶出
避让龛继续通行。不依赖 ROS / Gazebo / CARLA / MuJoCo。

对应文档：[`docs/ground/narrow_giveway_passing.md`](../../../docs/ground/narrow_giveway_passing.md)

## 文件

```
main.py                              场景、状态机、运动学、渲染与 CLI
requirements.txt                     numpy、matplotlib
tests/test_main.py                   让行决策、无碰撞完成、等待阶段、确定性回归测试
```

## 环境

- Python 3.8+
- 依赖见 `requirements.txt`：

```bash
pip install -r requirements.txt
```

## 运行

在仓库根目录执行。Linux/macOS 上 `python` 通常写成 `python3`：

```bash
python src/ground/narrow_giveway_passing/main.py --demo --outdir demo_out
```

`--demo` 自动跑完会车过程并保存两张过程截图到 `demo_out/`：

- `narrow_giveway_passing_yield.png`：让行车在避让龛内等待，对向车正在通过；
- `narrow_giveway_passing_passed.png`：对向车通过后，让行车驶出避让龛。

无窗口评测（CI / 无显示器机器可用，不打开窗口）：

```bash
python src/ground/narrow_giveway_passing/main.py --headless --duration 40 --outdir demo_out
```

输出 `demo_out/metrics.json`，字段见下文“指标”。

## 状态机

让行车经历五个阶段：

| 状态 | 含义 |
|---|---|
| `to_niche` | 驶向自己一侧的避让龛入口 |
| `enter` | 驶入避让龛并停稳 |
| `waiting` | 龛内等待，直到对向车完全越过本车龛位区间 |
| `exit` | 对向车清空后驶回行车道 |
| `cruise` | 沿车道直行到终点 |

对向车全程处于 `cruise` 直行通过。

## 指标

`metrics.json` 字段：

| 字段 | 含义 |
|---|---|
| `yield_role` | 让行车角色（A 左→右 / B 右→左） |
| `pass_role` | 直行通过车角色 |
| `collisions` | 碰撞事件次数（一次持续接触只记一次） |
| `min_headway` | 两车全程最小间距（米） |
| `wait_time` | 让行车在龛内等待时长（秒） |
| `total_time` | 场景总时长（秒） |
| `passed` | 是否无碰撞完成交汇 |

固定 `--seed` 后指标可完全复现。

## 测试

```bash
python -m unittest discover -s src/ground/narrow_giveway_passing/tests -v
```

覆盖角度归一化、让行决策、无碰撞完成、等待阶段经历与同种子确定性。

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `MAX_SPEED` | 6.0 m/s | 小车最大速度 |
| `ARRIVE_R` | 1.2 m | 到达目标点判定半径 |
| `DT` | 1/30 s | 固定仿真步长 |
| `NICHES` | 两处 | 避让龛 x 区间与停靠中心 |

## 限制

- 让行规则按“到己方避让龛距离更近”判定，未引入实时车速、车型尺寸协商。
- 几何为二维独轮运动学，不考虑车身扫掠、转向半径与悬挂。
- 避让龛数量与位置写死在场景里，没有动态编辑器。
