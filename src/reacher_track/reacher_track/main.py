#!/usr/bin/env python3
"""一键运行 MuJoCo Reacher 机械臂神经网络目标跟踪全流程。

依次执行：
  1) collect_demo   —— IK+PD 专家生成演示数据（demo.npz）
  2) train_mlp      —— 监督训练 MLP
  3) dagger_iter    —— DAgger 迭代（缓解分布偏移）
  4) deploy_eval    —— 闭环评估，输出误差曲线 + 性能表

用法：
  ros2 run reacher_track main          # colcon install 之后
  python3 -m reacher_track.main        # 源码树内（src/reacher_track 在 sys.path 上）
  ros2 launch reacher_track main.launch.py

工作目录（四步的中间产物都落这里）：默认 ~/.ros/reacher_track，
可用环境变量 REACHER_RUN_DIR 覆盖；快速模式见 README（RT_* 环境变量）。

为什么用 `-m` 而不是 `python3 reacher_track/collect_demo.py`：colcon install
之后包在 site-packages 里，源码不在 share 目录，按脚本路径调用会 "can't open
file"。`-m` 在两种布局下都成立——这里把包所在目录补进 PYTHONPATH，源码树
直接 `python3 -m reacher_track.main` 也能跑。
"""
import os
import subprocess
import sys

STEPS = [
    ('collect_demo', '数据采集（IK+PD 专家演示）'),
    ('train_mlp', '监督训练 MLP'),
    ('dagger_iter', 'DAgger 迭代'),
    ('deploy_eval', '闭环部署评估'),
]


def _run_dir():
    d = os.environ.get('REACHER_RUN_DIR') or os.path.join(
        os.path.expanduser('~'), '.ros', 'reacher_track')
    os.makedirs(d, exist_ok=True)
    return d


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    workdir = _run_dir()
    env = dict(os.environ)
    env['PYTHONPATH'] = root + (os.pathsep + env['PYTHONPATH']
                               if env.get('PYTHONPATH') else '')

    print('=' * 62)
    print('Reacher 目标跟踪全流程（工作目录：%s）' % workdir)
    print('=' * 62)
    for stage, desc in STEPS:
        print('\n---- 步骤：%s ----' % desc)
        r = subprocess.run([sys.executable, '-m', 'reacher_track.' + stage],
                           cwd=workdir, env=env)
        if r.returncode != 0:
            sys.exit('[main] 步骤失败：%s（退出码 %d）' % (stage, r.returncode))
    print('\n[main] 全部完成，产物在 %s。' % workdir)
    print('[main] 动图：python3 -m reacher_track.render_gif（需先有 mlp.npz）')


if __name__ == '__main__':
    main()
