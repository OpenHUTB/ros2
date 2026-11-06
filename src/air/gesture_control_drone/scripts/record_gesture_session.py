#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""录制手势数据集：摄像头 -> 手势识别 -> 42 维特征 + 弱标签 -> ``.npz``。

用法
----
    # 采 60 秒，存到 data/session_01.npz
    python3 record_gesture_session.py --out data/session_01.npz --duration 60

    # 只采 30 秒、降采样到 8 Hz（样本更分散）
    python3 record_gesture_session.py --out data/s2.npz --duration 30 --fps 8

采样节奏
--------
``--fps`` 是**按截止时刻排程**的：每帧的目标时刻是 ``t0 + n/fps``，采完睡到该时刻。
如果单帧耗时比 1/fps 还长，就自然掉到"尽可能快"，并在结束时报出**实际频率**，
而不是让名义值骗人（``sleep(1/fps)`` 那种写法在大模型推理下会掉到名义值的一半）。

中断保护
--------
采集中按 ``Ctrl+C`` 不会丢掉已经采到的数据：``KeyboardInterrupt`` 会被捕获，
照常打印质量小结并落盘（这也是采集脚本最容易吃亏的地方）。

采集规范（直接影响能不能训出模型）
----------------------------------
* 手离镜头 **40~60 cm**，手心正对镜头，手指朝上；
* 每个手势保持 **1~2 秒**，然后手**完全放下**停顿一下再换下一个；
* **手势要交替做**，不要"一种做完再做下一种"——10 Hz 连拍的相邻帧几乎重复，
  一次性做完某一种会让这段时间的样本彼此高度相似，训练/验证无论怎么划都容易
  互相泄漏（换成别的划分方式也一样）。交替做才能让各类样本在时间上分散开。

标签是**弱标签**（规则法的判断），每个会话的丢弃原因都会记进 ``meta``。
"""
from __future__ import annotations

import os
import sys
import time

# 与其它脚本一致：把本文件所在目录放进 sys.path，保证辅助模块能 import
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from gesture_session import SessionCollector, summarize   # noqa: E402

DEFAULT_OUT = os.path.join("data", "session_01.npz")


def _grab_frame(camera):
    """兼容 ``Camera.read_frame()`` 与 ``cv2.VideoCapture.read()``。"""
    if hasattr(camera, "read_frame"):
        return camera.read_frame()
    ok, frame = camera.read()
    return frame if ok else None


def make_default_camera(index=0):
    """默认摄像头（MJPG 640x480，虚拟机 USB 透传下才读得到帧）。"""
    from camera_utils import Camera
    cam = Camera(camera_index=index)
    cam.initialize()
    return cam


def make_default_detector(max_hands=1):
    from gesture_detection import GestureDetector
    return GestureDetector(max_hands=max_hands)


def run(camera=None, detector=None, out_path=DEFAULT_OUT, duration=60.0, fps=10.0,
        collector=None, log=print, clock=None, sleep=None):
    """录制主循环（**可以注入假摄像头/假检测器/假时钟**，便于离线测试）。

    返回 ``{"report", "path", "saved", "frames", "kept", "actual_fps", "interrupted"}``。
    """
    clock = clock or time.monotonic
    sleep = sleep or time.sleep
    if duration <= 0:
        raise ValueError("duration 必须为正，收到 %.1f" % duration)
    if fps <= 0:
        raise ValueError("fps 必须为正，收到 %.1f" % fps)

    collector = collector or SessionCollector()
    camera = camera if camera is not None else make_default_camera()
    detector = detector if detector is not None else make_default_detector()

    interval = 1.0 / float(fps)
    log("开始采集 %.0f 秒（目标 %.0f Hz）……" % (duration, fps))
    log("请依次、交替地做：✋ 张开手掌 / ✊ 握拳 / ☝️ 食指 / ✌️ V / 👍 竖起拇指 / "
        "👎 拇指向下 / 👌 OK（每个保持 1~2 秒，之间手放下）")

    started = clock()
    frames = 0
    kept = 0
    no_hand_streak = 0
    dark_warned = False
    hand_hinted = False
    last_log = started
    interrupted = False

    try:
        while True:
            now = clock()
            elapsed = now - started
            if elapsed >= duration:
                break
            frame = _grab_frame(camera)
            if frame is None:
                sleep(interval)
                continue
            frames += 1

            # 选错摄像头时画面基本是黑的：给一条可操作的提示，而不是让用户干等
            if not dark_warned and frames == 3:
                try:
                    import numpy as _np
                    mean = float(_np.mean(frame))
                    if mean < 25.0:
                        dark_warned = True
                        log("警告：画面很暗（均值 %.0f），可能选错了摄像头，"
                            "试试 --camera 1" % mean)
                except Exception:                      # noqa: BLE001
                    pass

            result = detector.detect(frame)
            landmarks = getattr(result, "landmarks", None)
            if collector.add(landmarks, getattr(result, "gesture", None)):
                kept += 1
                no_hand_streak = 0
            elif landmarks is None:
                no_hand_streak += 1
                if not hand_hinted and no_hand_streak >= int(fps * 5):
                    hand_hinted = True
                    log("提示：连续 5 秒没检测到手。请把手放进画面（离镜头 30~50 cm），"
                        "或换一个摄像头重试（--camera N）")

            now = clock()
            if now - last_log >= 2.0:
                last_log = now
                log("  [%.0fs] 处理 %d 帧，已收录 %d 个样本"
                    % (now - started, frames, collector.total))

            # 按截止时刻排程：目标时刻 = t0 + n/fps
            target = started + frames * interval
            wait = target - clock()
            if wait > 0:
                sleep(wait)
    except KeyboardInterrupt:
        interrupted = True
        log("\n收到 Ctrl+C：停止采集，保留已经采到的样本")

    elapsed = max(1e-6, clock() - started)
    actual_fps = frames / elapsed
    report = collector.quality_report()
    summarize(report, write=log)
    log("实际采样率：%.1f Hz（目标 %.1f Hz，共 %d 帧）"
        % (actual_fps, fps, frames))

    saved_path = None
    if collector.total > 0:
        saved_path = collector.finalize(out_path, meta={
            "duration_sec": float(duration),
            "fps_target": float(fps),
            "fps_actual": round(float(actual_fps), 2),
            "frames_seen": int(frames),
            "interrupted": bool(interrupted),
        })
        log("已保存：%s" % saved_path)
    else:
        log("没有任何有效样本，未写盘")
    if collector.missing_classes():
        log("提醒：这些手势一个样本都没有 -> %s"
            % ", ".join(collector.missing_classes()))
    return {"report": report, "path": saved_path, "saved": saved_path is not None,
            "frames": int(frames), "kept": int(kept),
            "actual_fps": float(actual_fps), "interrupted": bool(interrupted)}


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(
        description="录制手势数据集（摄像头 -> 42 维特征 + 弱标签 -> npz）")
    parser.add_argument("--out", default=DEFAULT_OUT, help="输出 .npz 路径")
    parser.add_argument("--duration", type=float, default=60.0, help="录制秒数")
    parser.add_argument("--fps", type=float, default=10.0,
                        help="目标采样频率（按截止时刻排程，实际频率会报出来）")
    parser.add_argument("--camera", type=int, default=0, help="摄像头索引")
    parser.add_argument("--max-hands", type=int, default=1)
    args = parser.parse_args(argv)

    try:
        stats = run(out_path=args.out, duration=args.duration, fps=args.fps,
                    camera=make_default_camera(args.camera),
                    detector=make_default_detector(args.max_hands))
    except KeyboardInterrupt:
        print("\n已中断")
        return 130
    # 一个样本都没采到用 2 退出，方便脚本判断
    return 0 if stats["saved"] else 2


if __name__ == "__main__":
    import sys
    sys.exit(main())
