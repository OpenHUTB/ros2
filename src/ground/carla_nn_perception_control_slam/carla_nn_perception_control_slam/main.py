#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模块统一入口（入口文件名以 main 开头）

用法：
    ros2 run carla_nn_perception_control_slam main --task 1
    ros2 run carla_nn_perception_control_slam main --task 2
    ros2 run carla_nn_perception_control_slam main --task 3
    ros2 run carla_nn_perception_control_slam main --task 4 --mode train --epochs 18
"""
import argparse
import os
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

TASKS = {
    '1': ('manual_control.py', '物理仿真与键盘运动控制'),
    '2': ('perception_control.py', 'YOLOv5 感知与轨迹跟踪控制'),
    '3': ('slam_nav.py', '激光建图与 A* 导航（SLAM 同步）'),
    '4': ('e2e_pilotnet.py', 'PilotNet 端到端驾驶'),
}


def main():
    ap = argparse.ArgumentParser(description='CARLA 神经网络感知/控制/SLAM 模块入口')
    ap.add_argument('--task', required=True, choices=sorted(TASKS))
    ap.add_argument('--mode', default='drive', choices=['collect', 'train', 'drive'],
                    help='仅任务4有效：collect 采集 / train 训练 / drive 推理驾驶')
    ap.add_argument('--frames', default='70', help='采集或推理的帧数')
    ap.add_argument('--epochs', default='18', help='训练轮数')
    args, rest = ap.parse_known_args()

    script, desc = TASKS[args.task]
    path = os.path.join(HERE, script)
    print('[main] 任务 %s：%s' % (args.task, desc), flush=True)

    if args.task == '4':
        n = args.epochs if args.mode == 'train' else args.frames
        sys.argv = [path, args.mode, str(n)] + rest
    else:
        sys.argv = [path] + rest
    runpy.run_path(path, run_name='__main__')


if __name__ == '__main__':
    main()
