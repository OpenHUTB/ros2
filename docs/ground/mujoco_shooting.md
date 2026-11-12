# MuJoCo小车 打靶仿真

## 实验简介
本实验使用MuJoCo搭建带旋转炮塔的四轮小车，固定在场地上自动打靶：前方扇形布置 6 个彩色靶（3 近 3 远），炮塔像雷达一样自动识别还立着的靶子，绿色激光瞄准并锁定（靶子上方亮起白色准星），根据距离自动控制发射力度，逐个击倒；偶尔打偏会自动加力补枪，全部击倒自动结束，无需人工干预。

## 运行环境
Ubuntu20.04 / Windows + Python3 + mujoco（本机实测 mujoco 3.17）

## 运行步骤

!!! warning
    下面命令需要在仓库根目录运行，不然会找不到文件。

```bash
# 1. 安装依赖
pip install mujoco

# 2. 打开可视化窗口（默认）
python3 src/ground/mujoco_shooting/main.py

# 3. 无头跑一局：不开窗口，跑完打印结果，返回码 0 = 全部击倒
python3 src/ground/mujoco_shooting/main.py --headless --max-steps 12000

# 4. 无头 + 侧风 + 成绩单：风加速度 (ax, ay)，逐发记录写 CSV
python3 src/ground/mujoco_shooting/main.py --headless --wind 0.8 0.3 --report out.csv

# 5. Linux 无显示服务（WSL2 / 服务器）时，给无头模式指定软件渲染
MUJOCO_GL=osmesa python3 src/ground/mujoco_shooting/main.py --headless
```

安装后可以用这个命令检查是否装好：
```bash
python3 -c "import mujoco; print(mujoco.__version__)"
```

可视化模式下：绿色激光开场扫描，随后炮塔自动锁定靶子发射小球，靶子逐个倒下，全部击倒后自动停止。

![打靶仿真动图](../img/ground/mujoco_shooting.gif)

## 代码结构

| 文件 | 作用 |
|---|---|
| `src/ground/mujoco_shooting/scene.py` | MJCF 场景：四轮小车 + 旋转炮塔 + 6 个可击倒靶子 + 12 发球 |
| `src/ground/mujoco_shooting/controller.py` | 纯控制逻辑：选靶、P-D 瞄准、弹道初速、击倒判定（不依赖 mujoco，可单测） |
| `src/ground/mujoco_shooting/ballistics.py` | 弹道反解（固定飞行时间 + 恒定风加速度）与成绩统计（纯标准库） |
| `src/ground/mujoco_shooting/main.py` | 状态机与两种跑法（viewer / headless），含 `--wind / --report` |
| `tests/test_mujoco_shooting_local.py` | 12 项离线测试：纯逻辑 + 无头物理冒烟 |
| `tests/test_mujoco_shooting_ballistics.py` | 18 项离线测试：弹道解算 + 成绩统计（不需要 mujoco） |

## 控制流程

1. **扫描**：开场 0.8 s 炮塔左右扫场（绿色激光示意）；
2. **选靶**：在还立着的靶子里，选与当前炮塔朝向夹角最小的一个；
3. **瞄准**：P-D 控制炮塔速度，角度与角速度都进入死区才开火；
4. **发射**：固定飞行时间 0.45 s，水平速度 = 距离 / 飞行时间（含风偏补偿），垂直速度按重力补偿；打偏过的靶子下一发力度 ×1.25；
5. **判定**：靶子铰链角超过 0.25 rad 记为击倒；全部击倒 / 12 发用尽 / 超时则结束。

## 弹道解算、风偏补偿与成绩报告

### 弹道模型

发射时不再把抛物线写死在循环里，而是解一个"固定飞行时间"的反问题（`ballistics.py`）：

```text
vx = (x − x0)/T − ½·ax·T
vy = (y − y0)/T − ½·ay·T
vz = (z − z0 + ½·g·T²)/T
```

`T` 固定 0.45 s，距离越远初速度越大；风 `(ax, ay)` 作为**恒定水平加速度**参与解算，
仿真里每步给飞行中的小球加 `m·a` 的外力，因此有风时也能落回靶心。

### 为什么原来等效重力只有 8.2 m/s²

`scene.py` 的 `<default>` 给所有关节设了 `damping="0.05" armature="0.02"`，小球原来用的
`<freejoint>` 会一并继承：

* `armature` 把转动/平动惯量从 0.1 kg 抬到 0.12 kg，重力加速度变成 `0.1·g / 0.12 ≈ 8.2 m/s²`；
* `damping` 让小球边飞边减速，水平速度每 0.1 s 掉约 0.05 m/s。

结果是解析弹道和仿真对不上（打远处会系统性偏低）。现在 12 个球的自由关节显式写成
`<joint type="free" damping="0" armature="0"/>`，实心球本来就该没有关节阻尼，弹道与解析式一致。

### 成绩统计与逐发记录

无头模式现在会输出成绩单（`score_shots`）：

```text
==============================================
打靶成绩
==============================================
  结束原因    : 全部击倒
  发射发数    : 9
  击倒靶数    : 6 / 6
  命中率      : 66.7%
  一发一个靶  : 3 个
  平均用弹    : 1.50 发/靶
  耗时        : 11.94 s
  风加速度    : (0.80, 0.30) m/s²
==============================================
```

`--report out.csv` 会把逐发记录写成 CSV（UTF-8 BOM，Excel 直接打开）：
`shot / sim_time_s / target / distance_m / speed_m_s / hit`，末尾附一行汇总。

### 实测

| 场景 | 发射 | 击倒 | 命中率 | 平均用弹 |
|---|---|---|---|---|
| 无风 | 9 | 6 / 6 | 66.7% | 1.50 发/靶 |
| 风 (0.8, 0.3) m/s² | 9 | 6 / 6 | 66.7% | 1.50 发/靶 |

自由空间单发验证（风 3.0 m/s²、飞行 0.45 s）：不做补偿落点偏 **0.32 m**，补偿后只偏 **0.045 m**。

## 测试

```bash
python3 tests/test_mujoco_shooting_local.py        # 12 项：纯逻辑 + 无头物理
python3 tests/test_mujoco_shooting_ballistics.py   # 18 项：弹道解算 + 成绩统计（不需要 mujoco）
```

没装 mujoco 时只跑纯逻辑部分（选靶、瞄准、弹道补偿）；装了则额外跑一局无头物理，断言 12 发内 6 个靶全部倒下。

本机实测（Windows + Python 3.13 + mujoco 3.17）：

```text
结果: {'shots': 9, 'downed': 6, 'hit_rate': 0.667, 'first_shot_hits': 3,
       'avg_shots_per_down': 1.5, 'duration': 11.948, 'finished': True,
       'reason': '全部击倒', 'steps': 5974, 'targets': 6, 'all_down': True,
       'time_s': 11.95, 'wind': (0.0, 0.0), 'down': 6}
```

## 说明
本作业的代码与文档在编写过程中使用了 AI 大模型辅助，本人已核对并确认提交内容，对提交内容负责。
