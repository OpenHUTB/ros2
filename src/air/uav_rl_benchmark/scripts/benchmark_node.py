#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对比实验的 ROS 节点封装（供 `roslaunch uav_rl_benchmark main.launch` 启动）.

参数从 ROS 参数服务器读取（launch 里用 `<param>` 传）：
    steps / obstacles / eval_episodes / out

节点在 `/benchmark/status` 上播报进度（`std_msgs/String`）：
``STARTING`` / ``RUN …`` / ``DONE <报告路径>`` / ``ERROR <原因>``。

注意：训练需要 torch / stable-baselines3。若目标机上没有，节点会**明确报错并退出**，
不会静默失败 —— 对比实验是离线训练任务，本就不该在只装了 ROS 的机器上跑。
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)


def main():
    import rospy                                     # noqa: WPS433
    from std_msgs.msg import String                  # noqa: WPS433

    rospy.init_node("uav_rl_benchmark", anonymous=False)
    pub = rospy.Publisher("/benchmark/status", String, queue_size=10)

    def say(text):
        rospy.loginfo("benchmark: %s", text)
        try:
            pub.publish(String(data=text))
        except Exception:                                   # noqa: BLE001
            pass

    say("STARTING")

    try:
        import stable_baselines3 as _sb3                 # noqa: WPS433,F401
    except Exception as exc:                                # noqa: BLE001
        say("ERROR 缺少 stable-baselines3 / torch（本节点只用于离线训练）：%s" % exc)
        return 1

    steps = int(rospy.get_param("steps", 100000))
    obstacles = int(rospy.get_param("obstacles", 16))
    eval_episodes = int(rospy.get_param("eval_episodes", 100))
    eval_freq = int(rospy.get_param("eval_freq", 20000))
    during = int(rospy.get_param("during_episodes", 20))
    out_dir = rospy.get_param("out", "") or os.path.join(PKG_ROOT, "results")

    say("RUN steps=%d obstacles=%d eval_episodes=%d eval_freq=%d during=%d out=%s"
        % (steps, obstacles, eval_episodes, eval_freq, during, out_dir))
    try:
        import benchmark as bm                           # noqa: WPS433
        bm.run_sweep(algos=bm.ALGOS, archs=((64, 64),), steps=steps,
                     seeds=(0, 1, 2), obstacles=obstacles, out_dir=out_dir,
                     eval_episodes=eval_episodes, eval_freq=eval_freq,
                     eval_episodes_during=during)
        from report import build_report                  # noqa: WPS433
        md = build_report(out_dir)
        say("DONE %s" % md)
        return 0
    except Exception as exc:                                # noqa: BLE001
        say("ERROR %s" % exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
