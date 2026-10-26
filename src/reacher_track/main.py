#!/usr/bin/env python3
"""一键运行 MuJoCo Reacher 机械臂神经网络目标跟踪全流程。

依次执行：
  1) collect_demo.py   —— IK+PD 专家生成演示数据（demo.npz）
  2) train_mlp.py      —— 监督训练 MLP
  3) dagger_iter.py    —— DAgger 迭代（缓解分布偏移）
  4) deploy_eval.py    —— 闭环评估 30 回合，输出误差曲线 + 性能表

用法（在包源码根目录 src/reacher_track/ 下）：
  python3 main.py
也可通过 launch 一键启动（等价流程）：
  ros2 launch reacher_track main.launch.py
"""
import os
import subprocess
import sys

STEPS = [
    ('reacher_track/collect_demo.py', '数据采集（IK+PD 专家演示）'),
    ('reacher_track/train_mlp.py', '监督训练 MLP'),
    ('reacher_track/dagger_iter.py', 'DAgger 迭代'),
    ('reacher_track/deploy_eval.py', '闭环部署评估'),
]


def main():
    root = os.path.dirname(os.path.abspath(__file__))
    print('=' * 62)
    print('Reacher 目标跟踪全流程（工作目录：%s）' % root)
    print('=' * 62)
    for script, desc in STEPS:
        print('\n---- 步骤：%s ----' % desc)
        r = subprocess.run([sys.executable, script], cwd=root)
        if r.returncode != 0:
            sys.exit('[main] 步骤失败：%s（退出码 %d）' % (script, r.returncode))
    print('\n[main] 全部完成。可再运行 python3 reacher_track/render_gif.py '
          '生成跟踪动图。')


if __name__ == '__main__':
    main()
