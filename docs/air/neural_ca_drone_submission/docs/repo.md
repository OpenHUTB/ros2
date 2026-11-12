# 工程组织与协作流程

## 1. 目录约定

```
neural_ca_drone/
├── main.py                  # 唯一 CLI 入口（main. 前缀）
├── ca_drone/                # 算法包
│   ├── sim/                 # CA-Sim：世界、动力学、射线投射
│   ├── render/              # Pillow 3D 渲染与 HUD
│   ├── nn/                  # NumPy 自动微分、层、优化器
│   ├── models/              # 四个网络
│   ├── mapping.py           # 在线栅格
│   ├── planning/            # A* 与路径简化
│   ├── control/             # 几何专家（数据教师）
│   ├── data/                # 数据集生成
│   ├── training/            # 训练循环
│   └── pipeline.py          # 闭环编排
├── tools/                   # 数据、训练、评测、图表、打包
├── weights/                 # 训练好的 npz（随仓库提交）
├── ros1_ws/  ros2_ws/       # ROS 适配层
├── docs/                    # mkdocs 文档
```

**入口规则**：所有可执行入口以 `main.` 开头（`main.py`、`main.launch`、
`main.launch.py`、`scripts/main.py`），保证“名字即入口”。

## 2. 分支与提交

| 项 | 约定 |
| --- | --- |
| 主分支 | `main`，始终保持可运行 |
| 功能分支 | `feat/<模块>-<短描述>` |
| 提交信息 | `<type>(<scope>): <摘要>`，正文写“为什么” |
| 类型 | feat / fix / docs / test / chore / perf |
| 单次提交 | 只做一件事，能独立回滚 |

示例：

```
feat(perception): add U-Net depth-to-occupancy model
fix(mapping): remove double 1/f factor in depth back-projection
docs(results): add 48-episode benchmark table
```

## 3. 每次 PR 的硬性检查

1. `python main.py --module full --scene sparse` 能直接跑通；
2. `python tools/evaluate.py --scenes all --modes full --seeds 1` 不报错；
3. 新增/修改的图表已重新生成（`tools/make_figs.py`）；
4. `mkdocs build --strict` 通过；
5. PR 描述中附一张运行截图或 GIF；
6. README 的运行步骤仍然有效。

本仓库提供 `.github/PULL_REQUEST_TEMPLATE.md` 与
`.github/workflows/ci.yml`（在 Ubuntu 上跑冒烟测试）。

## 4. 提交记录

仓库按功能演进拆成 18 次提交，下表按功能归并展示 12 个主要节点（完整列表见 `git log --oneline`）：

| # | 提交 | 内容 |
| --- | --- | --- |
| 1 | chore: bootstrap package | 目录、版本、依赖 |
| 2 | feat(sim): analytic ray caster | 世界、射线、四场景 |
| 3 | feat(render): software 3D HUD | 追尾视图、RGB-D、BEV |
| 4 | feat(nn): numpy autograd | Tensor、conv、Adam |
| 5 | feat(models): four networks | 感知/规划/控制/E2E |
| 6 | feat(data): expert labels | 数据集与教师 |
| 7 | feat(training): CPU trainer | 训练循环与权重 |
| 8 | feat(pipeline): closed loop | 在线建图、重规划 |
| 9 | fix(mapping): depth projection | 修 1/f 二次除法 |
| 10 | feat(ros1): launch + bridge | Noetic 包 |
| 11 | feat(ros2): launch + probe | Humble 包 |
| 12 | docs: mkdocs + results | 文档、图表 |

真实哈希见 `git log --oneline`。

## 5. 评审清单（自查）

- [ ] 新增代码有 docstring，公式与实现一致；
- [ ] 数值稳定性：除零、`nan`、空数组都有保护；
- [ ] 训练/评测使用固定种子，结果可复现；
- [ ] 没有把 `data/*.npz`、`runs/` 提交进仓库；
- [ ] 文档中的数字来自 `results/`，不是手写估计；
- [ ] 负结果如实记录，不粉饰。
