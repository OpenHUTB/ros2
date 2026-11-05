# MuJoCo小车 打靶仿真

## 实验简介
本实验使用MuJoCo搭建带旋转炮塔的四轮小车，固定在场地上自动打靶：前方扇形布置 6 个彩色靶（3 近 3 远），炮塔像雷达一样自动识别还立着的靶子，绿色激光瞄准并锁定（靶子上方亮起白色准星），根据距离自动控制发射力度，逐个击倒；偶尔打偏会自动加力补枪，全部击倒自动结束，无需人工干预。

## 运行环境
Ubuntu20.04 / Windows + Python3 + mujoco（本机实测 mujoco 3.17）

## 运行步骤
```bash
# 1. 安装依赖
pip install mujoco

# 2. 打开可视化窗口（默认）
python3 src/ground/mujoco_shooting/main.py

# 3. 无头跑一局：不开窗口，跑完打印结果，返回码 0 = 全部击倒
python3 src/ground/mujoco_shooting/main.py --headless --max-steps 12000

# 4. Linux 无显示服务（WSL2 / 服务器）时，给无头模式指定软件渲染
MUJOCO_GL=osmesa python3 src/ground/mujoco_shooting/main.py --headless
```

可视化模式下：绿色激光开场扫描，随后炮塔自动锁定靶子发射小球，靶子逐个倒下，全部击倒后自动停止。

![打靶仿真动图](../img/ground/mujoco_shooting.gif)

## 代码结构

| 文件 | 作用 |
|---|---|
| `src/ground/mujoco_shooting/scene.py` | MJCF 场景：四轮小车 + 旋转炮塔 + 6 个可击倒靶子 + 12 发球 |
| `src/ground/mujoco_shooting/controller.py` | 纯控制逻辑：选靶、P-D 瞄准、弹道初速、击倒判定（不依赖 mujoco，可单测） |
| `src/ground/mujoco_shooting/main.py` | 状态机与两种跑法（viewer / headless） |
| `tests/test_mujoco_shooting_local.py` | 12 项离线测试：纯逻辑 + 无头物理冒烟 |

## 控制流程

1. **扫描**：开场 0.8 s 炮塔左右扫场（绿色激光示意）；
2. **选靶**：在还立着的靶子里，选与当前炮塔朝向夹角最小的一个；
3. **瞄准**：P-D 控制炮塔速度，角度与角速度都进入死区才开火；
4. **发射**：固定飞行时间 0.45 s，水平速度 = 距离 / 飞行时间，垂直速度按重力补偿；打偏过的靶子下一发力度 ×1.25；
5. **判定**：靶子铰链角超过 0.25 rad 记为击倒；全部击倒 / 12 发用尽 / 超时则结束。

## 测试

```bash
python3 tests/test_mujoco_shooting_local.py
```

没装 mujoco 时只跑纯逻辑部分（选靶、瞄准、弹道补偿）；装了则额外跑一局无头物理，断言 12 发内 6 个靶全部倒下。

本机实测（Windows + Python 3.13 + mujoco 3.17）：

```text
结果: {'steps': 5974, 'shots': 9, 'down': 6, 'targets': 6, 'all_down': True, 'time_s': 11.95}
```

## 说明
本作业的代码与文档在编写过程中使用了 AI 大模型辅助，本人已核对并确认提交内容，对提交内容负责。
