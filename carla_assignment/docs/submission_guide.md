# CARLA 无人车 · PR / 提交流程指南

老师设计要求：*"提交次数和审核次数分别不少于 10 次"*，且*"每次 PR 都能通过 main 脚本直接
运行整个模块，提交信息提供运行效果图，README.md 提供运行环境与步骤；合并前需至少一名
其他人在另一台机器测试并同意"*。

本文件给出满足该要求的**分阶段提交流程**与建议拆分的多次 PR。仓库使用 `main`(主分支) +
短命特性分支模式，每次 PR 一个具体、不宽泛的标题。

## 1. 分支与提交流程

```bash
# 在远程建好仓库后
git remote add origin <your-repo-url>
git push -u origin main

# 每个改动：新建特性分支
git checkout -b feat/02-nn-perception-control
# ... 改代码/文档 ...
git add -A
git commit -m "carla 0.9.16 作业二：感知/控制改为神经网络(纯numpy MLP)，离线可训练+在线运行"
git push -u origin feat/02-nn-perception-control

# 在 GitHub 上对 main 发起 Pull Request（用 .github/pull_request_template.md）
```

也可以用自带脚本一键生成 ≥10 个具体命名的 PR 分支骨架：

```bash
bash scripts/gen_pr_branches.sh --dry     # 先看会建哪些分支
bash scripts/gen_pr_branches.sh           # 实际生成 10 个分支 + 起点 commit
```

## 2. PR 模板要点（见 pull_request_template.md）

每个 PR 必须：
1. **标题具体**：说明改了什么、为什么（避免泛泛如 "update code"）。
2. **main 可直接运行**：给出 `python main.py ...` 或 `main.bat ...` 的完整命令。
3. **提交信息带运行效果图**：GIF（<10MB）或截图，放 `docs/assets/`。
4. **说明测试**：哪台机器、ROS 版本、是否已由另一位同学在另一台机器测试并同意。
5. 声明是否使用大模型辅助。

## 3. 建议拆分 ≥10 次的 PR 清单（按此顺序提交可满足次数）

| # | PR 标题（具体） | 主要改动 |
|---|---|---|
| 1 | carla 0.9.16：初始化仓库骨架 + README + 环境脚本 | 目录、`main.py`、`setup_env.sh`、`requirements.txt` |
| 2 | carla 0.9.16 作业一：键盘运动控制 + 前视相机 | `01_control` main + launch |
| 3 | carla：新增纯 numpy 神经网络库 `nn_models.py` | 感知/控制/规划共用 MLP（含自测） |
| 4 | carla 0.9.16 作业二：感知/轨迹 NN 化 | `02_perception` 离线训练 + 在线运行 |
| 5 | carla 0.9.16 作业三：建图 + 规划 NN 导航 | `03_navigation` 离线训练 + 在线建图导航 |
| 6 | carla 0.9.16 作业四：端到端 CNN(图像→转向) | `04_end_to_end` collect/train/test |
| 7 | carla 0.9.16 作业五：整合入口 + 性能评价 | `05_reports` + `gen_train_loss.py` |
| 8 | carla：跨平台入口 `main.bat`/`main.sh` + ROS2 ament 打包 | 包名不冲突出入口、`package.xml`/`setup.py` |
| 9 | carla：ROS1 Noetic / ROS2 Humble launch 封装完善 | 各模块 launch + ros-bridge 说明 |
| 10 | carla：文档 NN 化 + 交付说明 + GitHub Pages 部署 | `docs/*.md`、`delivery_notes.md`、workflow |
| 11 | carla：动图/录屏补齐（需 CARLA 机）+ 性能评价表 | `docs/assets/*.gif`、实测指标填写 |

> 说明：当前仓库已按上述顺序把前 10 项的**大部分代码**准备就绪（见 git log），
> 你可将已有提交按功能拆分/推送到远程，或直接在本地按此顺序补充分支与 PR。

## 4. 远程配置与 GitHub Pages

- 仓库首页链接：`docs/index.md`（含跳转到各示例的链接）。
- 推送到 `main` 后，`.github/workflows/deploy_page.yml` 自动 `mkdocs build` 并部署到
  `https://<user>.github.io/<repo>/carla_assignment/`。
- 若访问 GitHub 不稳定，可用 Stem++ / fastgithub 加速，或用 Gitee 镜像 `sw/releases/tag/up`。

## 5. 合并前他人测试约定

- 在你的机器用 `main.*` 跑通改动模块，截图/GIF。
- 请至少一名同学在**另一台机器**克隆并重跑，验证后在其评论/PR 中批准同意。
- 只有获得同意后才 merge 到 `main`。
