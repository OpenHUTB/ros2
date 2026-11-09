#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把五个无人机实验页面合并成 docs/air/uav_experiments.md。

用法（仓库根目录）：
    python tools/merge_uav_experiments.py

源页面更新后重新运行即可；生成的页面保留原页面的图、表、公式和代码块，
标题统一降一级并去掉重复编号，开头的总览表与代码位置说明在脚本里维护。
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
AIR = ROOT / "docs" / "air"

PARTS = [
    ("uav_keyboard_control", "1. 无人机键盘控制（遥控链路）",
     "把终端按键转换成机体系速度指令，经 `carlair_ros_bridge` 执行；验证 ROS 指令通道与仿真器联动。"),
    ("uav_ppo_nav", "2. PPO 自主导航与避障",
     "随机目标点 + 静态障碍，训练在轻量质点环境、部署到 CarlaAir/AirSim；观测/动作接口两侧一致。"),
    ("uav_rl_benchmark", "3. 强化学习算法与网络结构对比",
     "在同一环境与接口下比较 PPO / SAC / TD3，以及 MLP 加宽、加深与基准结构的差异。"),
    ("uav_route_cruise", "4. 固定航线巡航",
     "沿闭合航线巡航一整圈，遇到压在航线上的障碍时临时绕行并回到航线；与随机目标导航互补。"),
    ("uav_traffic_nav", "5. 城市动态交通下的避障",
     "把静态圆柱换成沿车道行驶的车辆，用规则专家示范 + 行为克隆 + DAgger 学习穿越车流的时机。"),
]

OVERVIEW = """## 总览

| 实验 | 任务与方法 | 主要证据 |
| --- | --- | --- |
| 键盘控制 | 终端按键 → 机体系速度，ROS 桥接 | 按键映射、运行截图、实测记录 |
| PPO 导航 | 随机目标点 + 静态障碍，PPO（MLP） | 训练曲线、部署步骤、效果动图 |
| 算法对比 | PPO / SAC / TD3 同环境，100k 步 | 成功率、平滑度、成本图 |
| 航线巡航 | 固定闭合航线，航点投影 + 局部避障 | 轨迹对比、CarlaAir 实测轨迹 |
| 动态交通 | 穿越车流，专家示范 + BC + DAgger | 多种子成功率表、对比图 |

代码位置：`src/air/uav_keyboard_control`、`src/air/uav_ppo_nav`、`src/air/uav_rl_benchmark`、
`src/air/uav_route_cruise`、`src/air/uav_traffic_nav`。

> 五个实验的独立页面仍然保留；本页只是把内容合并，便于连续阅读和整体汇报。

"""

HEAD = """# 无人机ppo模块研究

本页把 `uav_keyboard_control`、`uav_ppo_nav`、`uav_rl_benchmark`、`uav_route_cruise`、
`uav_traffic_nav` 五个实验合并到一页。每个实验保留原有的小节结构与图，顺序按
「先打通指令通道 → 单目标导航 → 算法/结构对比 → 固定航线 → 动态交通」组织。

"""


def demote(line):
    """降一级标题并去掉“1.2.3”这类编号，避免与部分的编号冲突。"""
    match = re.match(r"^(#{2,6})\s+(.*)$", line)
    if not match:
        return line
    hashes, title = match.group(1), match.group(2).strip()
    title = re.sub(r"^\d+(?:\.\d+)*[.、]?\s*", "", title) or title
    return "#" * min(6, len(hashes) + 1) + " " + title


def body(path):
    lines = path.read_text(encoding="utf-8").splitlines()
    if lines and lines[0].startswith("# "):
        lines = lines[1:]
    while lines and not lines[0].strip():
        lines.pop(0)
    return [demote(line) for line in lines]


def main():
    chunks = [HEAD, OVERVIEW]
    for name, title, intro in PARTS:
        chunks.append("## " + title + "\n\n")
        chunks.append(intro + "\n\n")
        chunks.append("\n".join(body(AIR / (name + ".md"))) + "\n\n")

    merged = "".join(chunks)
    # 标题去掉编号后，原页面的中文锚点链接会失效，改成纯文本引用。
    merged = merged.replace("[7.1.1](#711-关于-rpc-监听地址)", "「关于 RPC 监听地址」小节")
    merged = merged.replace("[7.6](#76-按键闭环的复现方法)", "「按键闭环的复现方法」小节")

    target = AIR / "uav_experiments.md"
    target.write_text(merged, encoding="utf-8")
    print("已生成:", target)
    print("字符数:", len(merged), " 行数:", merged.count("\n"))


if __name__ == "__main__":
    main()
