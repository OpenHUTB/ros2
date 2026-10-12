#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
双海龟追逐实验 —— 一键启动入口(main.py)

功能: 自动检测 roscore 与 turtlesim 仿真器, 依次拉起
     逃亡者节点 runner.py 和追逐者节点 chaser.py;
     Ctrl+C 退出时自动清理全部子进程。

运行(需先 source 工作空间):
    rosrun turtle_chase main.py
或
    python main.py
"""

import os
import signal
import socket
import subprocess
import time


def master_alive():
    """通过 ROS_MASTER_URI 探测 roscore 是否已经在运行"""
    uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    try:
        host = uri.split('//')[-1].split(':')[0].split('/')[0]
        port = int(uri.split(':')[-1].split('/')[0])
        s = socket.socket()
        s.settimeout(1.0)
        try:
            s.connect((host, port))
            return True
        finally:
            s.close()
    except Exception:
        return False


def start(cmd):
    """以独立进程组启动子进程, 便于退出时整组清理"""
    print('启动: ' + cmd)
    return subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)


def main():
    procs = []

    if master_alive():
        print('检测到 roscore 已在运行, 跳过启动')
    else:
        procs.append(start('roscore'))
        time.sleep(3)

    procs.append(start('rosrun turtlesim turtlesim_node'))
    time.sleep(2)                                    # 等仿真窗口弹出
    procs.append(start('rosrun turtle_chase runner.py'))
    time.sleep(1)
    procs.append(start('rosrun turtle_chase chaser.py'))

    print('实验运行中, 按 Ctrl+C 一键退出')
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print('正在退出, 清理全部子进程...')
    finally:
        for p in reversed(procs):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1)
        print('已退出')


if __name__ == '__main__':
    main()
