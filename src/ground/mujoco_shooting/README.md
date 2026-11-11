# mujoco_shooting

MuJoCo 小车自动打靶：车固定在场地上，炮塔扫描识别还立着的靶子，瞄准后按距离控制力度发射，
逐个击倒；打偏会加力补枪，全部击倒自动结束。

## 运行

```bash
pip install mujoco

# 可视化窗口（默认）
python3 src/ground/mujoco_shooting/main.py

# 无头跑一局：跑完打印结果，返回码 0 = 全部击倒（可进 CI）
python3 src/ground/mujoco_shooting/main.py --headless --max-steps 12000

# Linux 上没有显示服务（WSL2 / 服务器）时，给无头模式指定软件渲染
MUJOCO_GL=osmesa python3 src/ground/mujoco_shooting/main.py --headless
```

无头模式输出示例（本机实测）：

```text
锁定靶4 距离2.77m
第6发 发射 力度6.2
...
结果: {'steps': 5974, 'shots': 9, 'down': 6, 'targets': 6, 'all_down': True, 'time_s': 11.95}
```

## 代码结构

| 文件 | 作用 |
|---|---|
| `scene.py` | MJCF 场景：四轮小车 + 旋转炮塔 + 6 个可击倒靶子 + 12 发球 |
| `controller.py` | 纯控制逻辑：选靶、P-D 瞄准、弹道初速、击倒判定（不依赖 mujoco，可单测） |
| `main.py` | 状态机与两种跑法（viewer / headless） |
| `tests/test_mujoco_shooting_local.py`（仓库根） | 纯逻辑单测 + 无头物理冒烟 |

## 控制流程

1. **扫描**：开场 0.8 s 炮塔左右扫场（绿色激光示意）；
2. **选靶**：在还立着的靶子里，选与当前炮塔朝向夹角最小的（`select_target`）；
3. **瞄准**：P-D 控制炮塔速度，角度与角速度都进入死区才开火；
4. **发射**：固定飞行时间 0.45 s，水平速度 = 距离 / 飞行时间，垂直速度按重力补偿；
   打偏过的靶子下一发力度 ×1.25；
5. **判定**：靶子铰链角超过 0.25 rad 记为击倒；全部击倒 / 12 发用尽 / 超时则结束。

## 测试

```bash
python3 tests/test_mujoco_shooting_local.py
```

没装 mujoco 时只跑纯逻辑部分（选靶、瞄准、弹道），装了则额外跑一局无头物理并断言 6 个靶全倒。
