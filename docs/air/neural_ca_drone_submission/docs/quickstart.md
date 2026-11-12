# 运行入口：`main.py`

所有模块都由仓库根目录的 `main.py` 驱动，`--module` 选择一个模块，
`--scene` 选择场景。每次运行在 `runs/<时间戳>_<模块>/` 下生成
PNG/GIF 与 `metrics.json`。

## 1. 命令速查

```bash
# 只渲染一帧，确认仿真正常
python main.py --module sim --scene sparse

# 感知模块：深度图 → 占用概率，输出三联图与 IoU
python main.py --module perception --scene dense

# 规划模块：经典 A* vs 神经启发式 A*
python main.py --module planning --scene corridor

# 控制模块：真值地图 + A* + 神经控制器
python main.py --module control --scene sparse

# 端到端：深度 + 目标 → 速度
python main.py --module e2e --scene dynamic --gif

# 分层全链路 + GIF
python main.py --module full --scene sparse --gif

# 五场景巡检，各生成一个 GIF
python main.py --module demo
```

## 2. 参数说明

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--module` | `full` | `sim / perception / planning / control / e2e / full / demo` |
| `--scene` | `sparse` | `sparse / dense / dynamic / corridor` |
| `--weights` | `weights` | 权重目录 |
| `--max-steps` | `650` | 单回合最大步数（10 Hz，65 s） |
| `--render-every` | `4` | 每隔几步渲染一帧（越小 GIF 越流畅、越大文件越小） |
| `--seed` | `0` | 随机种子（动态障碍相位、感知噪声） |
| `--gif` | 关 | 输出 GIF 而不是最后一帧 PNG |
| `--gif-ms` | `90` | GIF 每帧时长（毫秒） |

## 3. 输出示例

```bash
$ python main.py --module full --scene sparse --gif
saved runs/20261110_120102_full/full_sparse.gif (8.93 MB, 109 frames)
{
  "scene": "sparse",
  "mode": "full",
  "success": true,
  "collision": false,
  "steps": 435,
  "duration_s": 43.5,
  "path_length_m": 51.91,
  "min_clearance_m": 1.294,
  "replans": 44,
  "plan_ms": 250.9,
  "infer_ms": 0.83,
  ...
}
```

`metrics.json` 的字段与 [性能评价](results.md) 中的表格一一对应。

## 4. 模块与入口的对应关系

| 课程要求 | 入口 | ROS launch |
| --- | --- | --- |
| 感知（神经网络） | `main.py --module perception` | `perception.launch` |
| 规划（神经网络） | `main.py --module planning` | `planning.launch` |
| 控制（神经网络） | `main.py --module control` | `control.launch` |
| 端到端（神经网络） | `main.py --module e2e` | `e2e.launch` |
| 全链路 | `main.py --module full` | `full.launch` / `main.launch` |

!!! tip "入口命名规范"
    仓库根入口为 `main.py`，ROS1 节点脚本为 `scripts/main.py`，
    ROS2 为 `scripts/main.py`；每个 launch 文件都只调用 `main.py`，
    满足“入口以 main. 开头”的要求。

## 5. 一次完整复现（约 15 分钟）

```bash
# 1) 生成数据（约 30 s）
python tools/make_data.py --out data --e2e-episodes 60 --e2e-limit 14000

# 2) 训练四个网络（CPU，约 10 min）
python tools/train.py --module all --data data --weights weights

# 3) 批量评测（5 场景 × 4 模式 × 3 种子 = 60 回合）
python tools/evaluate.py --scenes all --modes expert,control,full,e2e --seeds 3

# 4) 生成图表与 GIF
python tools/make_figs.py
python main.py --module demo --render-every 4
```

各步骤的预期输出与耗时见 [复现实验](reproduce.md)。
