#!/usr/bin/env bash
# ============================================================
#  CARLA 无人车四次作业 · 生成 10+ 次 PR 分支骨架
#  满足老师"提交次数和审核次数分别不少于 10 次"的要求。
#
#  用法（在仓库根目录、已有基线 commit 后执行）：
#    bash scripts/gen_pr_branches.sh          # 生成 10 个特性分支 + 空 commit
#    bash scripts/gen_pr_branches.sh --dry    # 只打印计划，不实际建分支
#
#  生成的每个分支名都具体说明"改了什么"，便于作为独立 PR 提交到 GitHub。
#  每个分支都会基于当前 master 建一个【标记性空提交】，之后你可把对应改动放到
#  相应分支上，按 submission_guide.md 一个个发 PR。
# ============================================================
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [ "$1" = "--dry" ]; then DRY=1; else DRY=0; fi

# 分支名 + 建议的 PR 标题（分支名已具体，勿宽泛）
BRANCHES=(
  "feat/01-control-keyboard|carla 0.9.16 作业一：键盘运动控制 + 前视相机"
  "feat/02-perception-nn|carla 0.9.16 作业二：感知+轨迹 改为神经网络"
  "feat/03-navigation-nn|carla 0.9.16 作业三：建图+规划 神经网络导航"
  "feat/04-end2end-cnn|carla 0.9.16 作业四：端到端CNN(图像->转向)"
  "feat/05-reports-bench|carla 0.9.16 作业五：整合入口+性能评价"
  "feat/nn-lib-numpy|carla 0.9.16：纯numpy神经网络库 nn_models.py"
  "feat/windows-launch|carla 0.9.16：Windows原生入口 main.bat/main.sh"
  "feat/ros2-ament-pack|carla 0.9.16：ROS2 ament 打包 + ROS1/ROS2 launch"
  "feat/docs-mkdocs|docs：mkdocs 文档(NN化/占位图/性能评价)"
  "feat/ci-offline-check|carla：CI 离线校验 + NN 回归测试"
)

CUR="$(git rev-parse --abbrev-ref HEAD)"
echo "当前分支: $CUR"
echo "计划生成 ${#BRANCHES[@]} 个特征分支："
for line in "${BRANCHES[@]}"; do
  name="${line%%|*}"; title="${line##*|}"
  echo "  - ${name}   （建议 PR 标题：${title}）"
done

if [ "$DRY" = "1" ]; then
  echo "[--dry] 仅打印，未建分支。"
  exit 0
fi

# 先确认工作区干净
if [ -n "$(git status --porcelain)" ]; then
  echo "[警告] 工作区有未提交改动，请先 commit 或 stash 再运行。"
  exit 1
fi

for line in "${BRANCHES[@]}"; do
  name="${line%%|*}"
  git checkout -q master
  git checkout -q -b "$name"
  # 打一个标记性空提交（--allow-empty），作为该 PR 的起点
  git commit -q --allow-empty -m "chore(${name}): 分支起点 - 待放入本次 PR 的具体改动"
  echo "  已建分支: $name"
done
git checkout -q master

echo
echo "==== 完成：已生成 ${#BRANCHES[@]} 个 PR 分支。 ===="
echo "之后操作："
echo "  1) 逐个 checkout 分支，把对应改动放上去并 commit（建议含运行效果图）。"
echo "  2) 按 PR 模板提交：git push -u origin <branch> 后在 GitHub 发 PR。"
echo "  3) 每分支合并前请至少一位同学在另一台机器测试并同意（见 docs/submission_guide.md）。"
