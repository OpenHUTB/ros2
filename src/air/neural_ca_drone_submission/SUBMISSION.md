# 提交说明（课程大作业）

本目录是“基于神经网络的无人机感知—规划—控制避障”的完整工程。
感知、规划、控制、端到端四条链路全部由神经网络实现，并在 Windows 本地与
Ubuntu 20.04 虚拟机中跑通。

## 一、快速验收

```bash
pip install -r requirements.txt          # numpy / Pillow / matplotlib / mkdocs
python main.py --module sim   --scene sparse
python main.py --module full  --scene corridor --gif     # 约 6 s，输出 GIF 与指标
python tools/evaluate.py --scenes all --modes expert,control,full,e2e --seeds 3
python -m mkdocs serve                   # 文档站点
```

模块入口统一为 `main.py`，`--module` 可取
`sim | perception | planning | control | e2e | full | demo`。

## 二、交付物位置

| 内容 | 路径 |
| --- | --- |
| 演示动图（Ubuntu 实机） | `docs/assets/gifs/full_sparse.gif`（8.93 MB） |
| 演示动图（走廊场景） | `docs/assets/gifs/full_corridor.gif`（2.88 MB） |
| 实验指标 | `results/summary.json`、`results/episodes.csv` |
| 训练曲线与场景图 | `docs/assets/figs/` |
| Word 报告 | `report/实验报告_陈康艺_2535030044.docx`（49 页） |
| ROS 1 包 | `ros1_ws/src/neural_ca_drone_ros/` |
| ROS 2 包 | `ros2_ws/src/neural_ca_drone_ros2/` |
| 文档源 | `docs/` + `mkdocs.yml` |

## 三、提交记录

仓库已有 18 次提交，按功能拆分：

```
chore(data): record dataset statistics and seeds
ci: smoke test and GitHub Pages deployment
docs: Word report generator and exported document
docs: mkdocs site with results, GIFs and ROS guides
feat(ros2): Humble package, launch files and probe
feat(ros1): Noetic package, launch files and simulator bridge
feat(pipeline): closed-loop stack, evaluation, GIF export
feat(training): CPU trainer, checkpoints and metrics
feat(data): expert-labelled datasets and collection CLI
feat(planning): A* with learned heuristic and geometric expert
feat(models): U-Net perception, neural heuristic, control and E2E nets
feat(nn): NumPy reverse-mode autograd, layers and Adam
feat(render): software 3D HUD with RGB-D and BEV panels
feat(sim): quadrotor dynamics and online occupancy grid
feat(sim): analytic ray caster and four evaluation scenes
chore: bootstrap the neural collision-avoidance package
```

如需在 GitHub 上补足“审核次数”，建议把这些提交按下面顺序各开一个 PR
（每个 PR 的说明里贴 `docs/assets/gifs/` 或 `docs/assets/figs/` 的对应图片）：

1. `feat(sim)` 世界与射线投射
2. `feat(sim)` 动力学与栅格地图
3. `feat(render)` 3D HUD
4. `feat(nn)` 自动微分
5. `feat(models)` 四个网络
6. `feat(planning)` A* 与专家
7. `feat(data)` 数据集
8. `feat(training)` 训练与权重
9. `feat(pipeline)` 闭环与评测
10. `feat(ros1)` Noetic 包
11. `feat(ros2)` Humble 包
12. `docs` 文档与 Word
13. `ci` 冒烟与 Pages

## 四、实测环境

- 本地：Windows 10 / Python 3.10 / 4 核 CPU，无 GPU、无 PyTorch。
- 虚拟机：Ubuntu 20.04.6 / Python 3.8.10 / Pillow 7.0 / ROS Noetic + Humble / Gazebo 11。
- ROS 1：`catkin_make` 通过；`main.launch` 启动后深度 10 Hz，`/cmd_vel` 有输出。
- ROS 2：`colcon build` 3.5 s；`probe.py` 收到 `CMD` 与 `STATUS`。

## 五、已知限制（如实记录）

- 端到端策略在 4 个场景中 3 个“安全但未在 650 步内到达”，仅 dynamic 成功；
  分层方案（full）4/4 成功。
- 学习启发式在 sparse 场景扩展节点更多（+143%），只在 dense/corridor 有优势。
- Gazebo 仅提供 world/urdf 与话题契约，未在没有 GPU 的虚拟机里跑闭环。
