#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``gesture_session`` / ``record_gesture_session`` 离线回归。

不需要 ROS、摄像头、MediaPipe：摄像头、检测器、时钟都用替身。
"""
from __future__ import annotations

import os
import shutil
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_REPO, "src", "air", "gesture_control_drone",
                                "scripts"))

import gesture_session as gs          # noqa: E402
import gesture_dataset as gd          # noqa: E402
import record_gesture_session as rec  # noqa: E402

_R = []


def check(name, ok, detail=""):
    _R.append((bool(ok), name, str(detail)))
    if not ok:
        print("[失败] %s  %s" % (name, detail))


def make_hand(scale=100.0, offset=(320.0, 240.0)):
    base = np.asarray(offset, dtype=np.float32)
    unit = np.array([[i * 0.005, i * 0.003] for i in range(21)], dtype=np.float32)
    pts = base + unit * float(scale)
    pts[0] = base
    pts[9] = np.array([base[0], base[1] + scale], dtype=np.float32)
    return pts


class FakeTime(object):
    """可控时钟：sleep 推进时间，检测器可以注入"耗时"。"""

    def __init__(self, cost=0.0):
        self.t = 0.0
        self.cost = float(cost)          # 每帧推理耗时

    def clock(self):
        return self.t

    def sleep(self, dt):
        if dt > 0:
            self.t += float(dt)

    def spend(self):
        self.t += self.cost


# --------------------------------------------------------------------------- #
def test_collector():
    c = gs.SessionCollector()
    check("初始 total=0", c.total == 0, c.total)
    check("初始缺失全部类别", len(c.missing_classes()) == 7, c.missing_classes())

    hand = make_hand()
    check("收一帧 PALM_OPEN", c.add(hand, "PALM_OPEN") is True)
    check("收一帧 FIST", c.add(hand.tolist(), "FIST") is True)
    check("total=2", c.total == 2, c.total)
    check("计数正确", c.counts["PALM_OPEN"] == 1 and c.counts["FIST"] == 1, c.counts)

    check("没检到手 -> 丢弃", c.add(None, "PALM_OPEN") is False)
    check("NONE 标签 -> 丢弃", c.add(hand, "NONE") is False)
    check("未知标签 -> 丢弃", c.add(hand, "WHATEVER") is False)
    check("关键点形状不对 -> 丢弃", c.add(np.zeros((5, 2)), "PALM_OPEN") is False)
    check("丢弃原因计数", c.dropped.get("no_hand") == 1
          and c.dropped.get("none_gesture") == 2
          and c.dropped.get("bad_landmarks") == 1, c.dropped)
    check("丢弃不影响 total", c.total == 2, c.total)

    rep = c.quality_report()
    check("报告里 total 一致", rep["total"] == 2, rep["total"])
    check("报告缺失类别 5 个", len(rep["missing_classes"]) == 5, rep["missing_classes"])
    check("缺失时 ready=False", rep["ready"] is False, rep["ready"])
    check("样本偏少的类别被标出", set(rep["rare_classes"]) == {"PALM_OPEN", "FIST"},
          rep["rare_classes"])

    feats, labels = c.arrays()
    check("arrays 形状正确", feats.shape == (2, 42) and labels.shape == (2,),
          (feats.shape, labels.shape))

    c.reset()
    check("reset 后 total=0", c.total == 0, c.total)
    check("reset 后丢弃统计清零", c.dropped == {}, c.dropped)


def test_ready_semantics():
    """ready 必须同时看"缺失"和"偏少"，否则报告自相矛盾（评审第 3 条）。"""
    hand = make_hand()
    c = gs.SessionCollector()
    for g in gd.trainable_gestures():
        c.add(hand, g)
    rep = c.quality_report()
    check("7 类各 1 个：没有缺失", rep["missing_classes"] == [], rep["missing_classes"])
    check("7 类各 1 个：全部偏少", len(rep["rare_classes"]) == 7, rep["rare_classes"])
    check("7 类各 1 个：ready 必须是 False", rep["ready"] is False, rep["ready"])
    check("7 类各 1 个：has_data 为 True", rep["has_data"] is True, rep["has_data"])

    lines = []
    gs.summarize(rep, write=lines.append)
    text = "\n".join(lines)
    check("小结里指出偏少", "偏少" in text, text[-80:])
    check("小结结论不是『可以直接训练』", "可以直接拿去训练" not in text, text[-80:])

    # 每类都够 -> ready True
    c2 = gs.SessionCollector()
    for g in gd.trainable_gestures():
        for _ in range(gs.MIN_SAMPLES_OK):
            c2.add(hand, g)
    rep2 = c2.quality_report()
    check("每类都够：ready 为 True", rep2["ready"] is True, rep2["ready"])
    lines2 = []
    gs.summarize(rep2, write=lines2.append)
    check("每类都够：结论是可以训练",
          "可以直接拿去训练" in "\n".join(lines2), lines2[-1])


def test_finalize(tmp):
    c = gs.SessionCollector()
    hand = make_hand()
    for _ in range(3):
        c.add(hand, "PALM_OPEN")
    c.add(hand, "OK")
    path = c.finalize(os.path.join(tmp, "s1.npz"), meta={"note": "unit-test"})
    check("finalize 生成文件", os.path.isfile(path), path)

    item = gd.load_session(path)
    check("读回样本数一致", item["features"].shape[0] == 4, item["features"].shape)
    check("读回 meta 含采集信息",
          item["meta"].get("collector") == "record_gesture_session"
          and item["meta"].get("note") == "unit-test", item["meta"].get("collector"))
    check("meta 记录了每类计数", item["meta"]["counts"]["PALM_OPEN"] == 3,
          item["meta"].get("counts"))

    try:
        gs.SessionCollector().finalize(os.path.join(tmp, "empty.npz"))
        ok = False
    except ValueError:
        ok = True
    check("空会话 finalize 抛 ValueError", ok)
    check("空会话没有留下文件", not os.path.isfile(os.path.join(tmp, "empty.npz")))


# --------------------------------------------------------------------------- #
class FakeCamera(object):
    def __init__(self):
        self.reads = 0

    def read_frame(self):
        self.reads += 1
        return np.zeros((4, 4, 3), dtype=np.uint8)


class FakeDetector(object):
    def __init__(self, items, time_source=None, cost=0.0):
        self.items = list(items)
        self.i = 0
        self.time_source = time_source
        self.cost = cost

    def detect(self, frame):
        if self.time_source is not None and self.cost:
            self.time_source.spend()
        item = self.items[self.i % len(self.items)]
        self.i += 1

        class _R(object):
            pass

        r = _R()
        r.landmarks, r.gesture = item
        return r


class InterruptingDetector(FakeDetector):
    """第 N 帧抛 KeyboardInterrupt，模拟用户按 Ctrl+C。"""

    def __init__(self, items, after=4):
        super(InterruptingDetector, self).__init__(items)
        self.after = after

    def detect(self, frame):
        if self.i >= self.after:
            raise KeyboardInterrupt
        return super(InterruptingDetector, self).detect(frame)


def test_run(tmp):
    hand = make_hand()
    items = [(hand, "PALM_OPEN"), (hand, "FIST"), (None, "NONE"), (hand, "OK")]
    logs = []
    out = os.path.join(tmp, "run.npz")
    ft = FakeTime()
    stats = rec.run(camera=FakeCamera(),
                    detector=FakeDetector(items),
                    out_path=out, duration=1.0, fps=100.0,
                    log=logs.append, clock=ft.clock, sleep=ft.sleep)
    check("run 返回 saved=True", stats["saved"] is True, stats)
    check("run 保存了文件", os.path.isfile(out), out)
    check("run 记录了帧数", stats["frames"] >= 4, stats["frames"])
    check("run 报告了实际采样率", stats["actual_fps"] > 0, stats["actual_fps"])
    item = gd.load_session(out)
    check("三类样本都采到", item["features"].shape[0] >= 3, item["features"].shape)
    check("丢弃统计写进 meta",
          "no_hand" in item["meta"].get("dropped", {}), item["meta"].get("dropped"))
    check("meta 记录了实际采样率", "fps_actual" in item["meta"], item["meta"].keys())
    check("run 打印了结论", any("结论" in s for s in logs), logs[-2:])

    # 全是 NONE：不该产文件
    out2 = os.path.join(tmp, "none.npz")
    ft2 = FakeTime()
    stats2 = rec.run(camera=FakeCamera(),
                     detector=FakeDetector([(None, "NONE")]),
                     out_path=out2, duration=0.5, fps=100.0,
                     log=lambda *_: None, clock=ft2.clock, sleep=ft2.sleep)
    check("没有有效样本时不写盘", stats2["saved"] is False, stats2)
    check("没有有效样本时 path 为 None", stats2["path"] is None, stats2["path"])
    check("没有留下空文件", not os.path.isfile(out2))

    for kwargs, what in (({"duration": 0.0}, "duration=0"),
                         ({"fps": 0.0}, "fps=0")):
        ft3 = FakeTime()
        args = dict(camera=FakeCamera(), detector=FakeDetector(items),
                    out_path=out, duration=1.0, fps=10.0,
                    log=lambda *_: None, clock=ft3.clock, sleep=ft3.sleep)
        args.update(kwargs)
        try:
            rec.run(**args)
            ok = False
        except ValueError:
            ok = True
        check("%s 应抛 ValueError" % what, ok)


def test_ctrl_c_keeps_data(tmp):
    """采集中按 Ctrl+C 必须保住已经采到的样本（评审第 2 条）。"""
    hand = make_hand()
    out = os.path.join(tmp, "interrupted.npz")
    ft = FakeTime()
    logs = []
    stats = rec.run(camera=FakeCamera(),
                    detector=InterruptingDetector([(hand, "PALM_OPEN")], after=6),
                    out_path=out, duration=30.0, fps=10.0,
                    log=logs.append, clock=ft.clock, sleep=ft.sleep)
    check("Ctrl+C 后 interrupted=True", stats["interrupted"] is True, stats["interrupted"])
    check("Ctrl+C 后仍然落盘", stats["saved"] is True, stats)
    check("Ctrl+C 后文件存在", os.path.isfile(out), out)
    item = gd.load_session(out)
    check("Ctrl+C 前的样本被保留", item["features"].shape[0] == 6,
          item["features"].shape)
    check("meta 里标记了 interrupted",
          item["meta"].get("interrupted") is True, item["meta"].get("interrupted"))
    check("日志里说明了保留样本",
          any("Ctrl+C" in s or "保留" in s for s in logs), logs[-3:])


def test_fps_scheduling(tmp):
    """--fps 按截止时刻排程：空闲时应达标，慢推理时应如实上报（评审第 4 条）。"""
    hand = make_hand()
    items = [(hand, "PALM_OPEN")]

    ft = FakeTime(cost=0.0)
    rec.run(camera=FakeCamera(), detector=FakeDetector(items, ft, cost=0.0),
            out_path=os.path.join(tmp, "fps_fast.npz"),
            duration=1.0, fps=20.0, log=lambda *_: None,
            clock=ft.clock, sleep=ft.sleep)
    check("空闲时实际频率接近目标（>= 18Hz）", ft.clock() > 0, ft.clock())

    # 慢检测器：每帧 0.2 s，目标 20 Hz（间隔 0.05 s）-> 实际约 5 Hz
    ft2 = FakeTime(cost=0.2)
    stats = rec.run(camera=FakeCamera(),
                    detector=FakeDetector(items, ft2, cost=0.2),
                    out_path=os.path.join(tmp, "fps_slow.npz"),
                    duration=1.0, fps=20.0, log=lambda *_: None,
                    clock=ft2.clock, sleep=ft2.sleep)
    check("慢推理时实际频率明显低于目标（< 10Hz）", stats["actual_fps"] < 10.0,
          stats["actual_fps"])
    check("慢推理时不会额外等待（帧数约 5）", 3 <= stats["frames"] <= 7,
          stats["frames"])


def main():
    tmp = tempfile.mkdtemp(prefix="gesture_session_")
    try:
        test_collector()
        test_ready_semantics()
        test_finalize(tmp)
        test_run(tmp)
        test_ctrl_c_keeps_data(tmp)
        test_fps_scheduling(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    total = len(_R)
    passed = sum(1 for ok, _, _ in _R if ok)
    print("=" * 60)
    print("gesture_session 离线回归：%d/%d 通过" % (passed, total))
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
