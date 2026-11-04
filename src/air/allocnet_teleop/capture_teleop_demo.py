#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""扫航点构型并抓帧，用于采集演示素材。

为什么需要"扫"：AllocNet 按固定 5 段推理时间分配，某些起终点几何会让
某段时长推理为 0，规划器打印 "time and seg does not fit" 后放弃本次规划
（不报错、不出轨迹），实测成功率约 1/5。因此这里遍历候选构型，
取第一组成功的。

前置
====
  1. Xvfb 虚屏（无 GPU 环境下的桌面渲染会静默冻结，不能用 :0）
       Xvfb :99 -screen 0 1280x800x24 -ac +extension GLX +render -noreset &
  2. RViz 用 capture_view.rviz（灰底高对比，否则轨迹看不见）
  3. AllocNet 规划器已启动，且 /teleop/key 有订阅者

按键经 /teleop/key 注入，无需 TTY。

用法
====
    DISPLAY=:99 python3 capture_teleop_demo.py
    DISPLAY=:99 python3 capture_teleop_demo.py --frames ~/shots --max-mb 2000
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

# ⚠ 必须在 import rospy 之前设置 DISPLAY
DISP = os.environ.get("DISPLAY", ":99")
os.environ["DISPLAY"] = DISP

import rospy                                    # noqa: E402
from std_msgs.msg import String                 # noqa: E402
from visualization_msgs.msg import Marker       # noqa: E402

KEY, TRAJ = "/teleop/key", "/visualizer/trajectory"
STEP = 0.5
DEFAULT_FRAMES = "/tmp/shots"
DEFAULT_MAX_MB = 2000
traj = []


def on_traj(msg):
    if len(msg.points):
        traj.append(len(msg.points))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="抓取 AllocNet 演示帧")
    parser.add_argument("--frames", default=DEFAULT_FRAMES,
                        help="帧输出目录，默认 %s" % DEFAULT_FRAMES)
    parser.add_argument("--max-mb", type=int, default=DEFAULT_MAX_MB,
                        help="帧目录超过该大小时停止抓取，默认 %d MB" % DEFAULT_MAX_MB)
    return parser.parse_args(argv)


def frames_size_mb(path):
    return sum(entry.stat().st_size for entry in os.scandir(path)) / (1024.0 * 1024.0)


def main(argv=None):
    args = parse_args(argv)
    frames_dir = os.path.abspath(os.path.expanduser(args.frames))
    os.makedirs(frames_dir, exist_ok=True)
    for f in os.listdir(frames_dir):
        if f.endswith(".png"):
            os.remove(os.path.join(frames_dir, f))

    rospy.init_node("best_shot", anonymous=True)
    pub = rospy.Publisher(KEY, String, queue_size=100)
    rospy.Subscriber(TRAJ, Marker, on_traj, queue_size=20)
    time.sleep(2.0)
    print("订阅者=%d" % pub.get_num_connections())

    n = [0]

    def send(s, d=0.06):
        for c in s:
            pub.publish(String(data=c))
            time.sleep(d)

    def grab():
        subprocess.call(["import", "-display", DISP, "-window", "root",
                         os.path.join(frames_dir, "s_%04d.png" % n[0])],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        n[0] += 1
        if n[0] % 10 == 0:
            size_mb = frames_size_mb(frames_dir)
            if size_mb > args.max_mb:
                raise SystemExit("帧目录 %.0f MB 超过 --max-mb %d MB，停止抓取"
                                 % (size_mb, args.max_mb))

    def move_to(tx, ty):
        send("r"); time.sleep(0.4)
        dx, dy = int(round(tx / STEP)), int(round(ty / STEP))
        if dx:
            send(("w" if dx > 0 else "s") * abs(dx))
        if dy:
            send(("a" if dy > 0 else "d") * abs(dy))
        time.sleep(0.35)

    # 多试几种构型（方向、距离各异）
    cands = [
        ((-3.0, 0.0), (2.0, 2.0)),
        ((-2.0, -1.0), (3.0, 1.0)),
        ((-3.0, 1.0), (1.0, 2.0)),
        ((-2.0, 0.0), (2.0, 1.0)),
        ((0.0, -1.0), (3.0, 2.0)),
        ((-4.0, -2.0), (1.0, 1.0)),
        ((-1.0, 2.0), (3.0, -1.0)),
        ((-3.0, -3.0), (2.0, 2.0)),
        ((0.0, 0.0), (2.0, 2.0)),
        ((-2.0, 2.0), (2.0, -2.0)),
    ]

    for i, (s, g) in enumerate(cands):
        before = len(traj)
        print("=== %d: %s -> %s ===" % (i + 1, s, g))
        move_to(*s)
        send("g"); time.sleep(0.7); grab()      # 起点标记
        grab()
        move_to(*g)
        grab()

        t0 = time.time()
        send("g")
        while time.time() - t0 < 3.5:           # 规划瞬间高频抓
            grab(); time.sleep(0.14)

        if len(traj) > before:
            print("  ✔ 成功，记录飞行 16s")
            t1 = time.time()
            while time.time() - t1 < 16:
                grab(); time.sleep(0.28)
            print("  帧数=%d  START=%s GOAL=%s" % (n[0], s, g))
            return 0
        print("  未成功")

    print("✘ 全失败 帧数=%d" % n[0])
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(2)
