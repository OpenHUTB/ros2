#!/usr/bin/env python3
"""launch 启动 Reacher 机械臂目标跟踪的训练与评估流水线。

用法（`colcon build` 且 `source install/setup.bash` 之后）：
  ros2 launch reacher_track main.launch.py

依次执行: collect_demo → train_mlp → dagger_iter → deploy_eval。

工作目录（demo.npz / mlp.npz / eval_summary.csv 都落在这里）：
  默认 ~/.ros/reacher_track，可用环境变量 REACHER_RUN_DIR 覆盖。

快速模式（CI / 冒烟验证）用环境变量压小规模，不传时保持实验默认值：
  RT_DEMO_EPISODES / RT_DAGGER_EPISODES / RT_EVAL_EPISODES
  RT_MAX_STEPS / RT_EPOCHS / RT_DAGGER_ITER
例如：
  RT_DEMO_EPISODES=8 RT_MAX_STEPS=60 RT_EPOCHS=5 RT_DAGGER_ITER=1 \\
  RT_DAGGER_EPISODES=4 RT_EVAL_EPISODES=3 ros2 launch reacher_track main.launch.py

为什么用 `python3 -m reacher_track.<stage>` 而不是 `python3 reacher_track/<stage>.py`：
colcon install 之后源码不在 share 目录里，老写法 `cd share_parent && python3
reacher_track/collect_demo.py` 只会得到 "can't open file"；改成按模块执行，
install 布局与源码树两种情形都能跑。
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown

STAGES = ('collect_demo', 'train_mlp', 'dagger_iter', 'deploy_eval')
DEFAULT_RUN_DIR = os.path.join(os.path.expanduser('~'), '.ros', 'reacher_track')


def generate_launch_description():
    run_dir = os.environ.get('REACHER_RUN_DIR') or DEFAULT_RUN_DIR
    pipeline = 'mkdir -p "{d}" && cd "{d}" && '.format(d=run_dir)
    pipeline += ' && '.join('python3 -m reacher_track.%s' % s for s in STAGES)
    return LaunchDescription([
        # 流水线是「一次性」的，跑完就退出：不加 Shutdown 的话 ros2 launch 会一直挂着，
        # CI 与手动运行都得自己 kill。
        ExecuteProcess(cmd=['bash', '-c', pipeline], output='screen',
                       on_exit=[Shutdown()]),
    ])
