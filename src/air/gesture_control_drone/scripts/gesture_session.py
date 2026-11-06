#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""采集会话的**纯逻辑**部分：把逐帧识别结果攒成一个可训练的数据集。

为什么不写在录制脚本里
----------------------
录制脚本要 import cv2 / mediapipe（摄像头、检测器），这些依赖在 CI 或无摄像头的
机器上没有，测试就跑不起来。这里只做"特征 -> 样本"的累积与统计，纯 numpy，
可以离线单测；录制脚本只负责接线。

弱标签
------
标签直接取规则法（``GestureResult.gesture``）的结果。这是**弱标签**：
规则法在边界样本上会错，所以：

* 只接受词表里 7 个可训练手势，``NONE`` 一律丢弃（不当作一个类别）；
* 每个会话都记录被丢弃的原因计数，便于事后评估采集质量；
* :meth:`SessionCollector.quality_report` 会给出"每类样本数 / 是否够训练"的提示。

可用性判定
----------
``ready`` 只在**既没有缺失类别、也没有样本偏少的类别**时为 True —— 这份报告的
唯一用途就是"判断这个会话值不值得拿去训练"，如果每类只有 1 个样本还说"可以直接
训练"，它就自相矛盾了。想放宽时传 ``min_samples``。
"""
from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np

import gesture_dataset as gd

#: 每类"够用"的样本数：低于这个值就提醒用户补采
MIN_SAMPLES_OK = 30


class SessionCollector(object):
    """累积 ``(特征, 弱标签)`` 样本，最后一次性存成 ``.npz``。"""

    def __init__(self, feature_fn=None):
        #: 特征提取函数，默认用 :func:`gesture_dataset.normalize_landmarks`
        self._feature_fn = feature_fn or gd.normalize_landmarks
        self._features: List[np.ndarray] = []
        self._labels: List[int] = []
        self._counts: Dict[str, int] = {g: 0 for g in gd.trainable_gestures()}
        #: 被丢弃的帧计数：``no_hand`` / ``none_gesture`` / ``bad_landmarks``
        self._dropped: Dict[str, int] = {}

    # ------------------------------------------------------------------ 采集
    def add(self, landmarks, gesture: Optional[str]) -> bool:
        """喂入一帧识别结果；返回是否被计入数据集。

        ``landmarks`` 为 ``None`` 表示这一帧没检测到手；
        ``gesture`` 不是 7 个可训练手势之一（含 ``NONE``）则丢弃。
        """
        if landmarks is None:
            self._bump("no_hand")
            return False
        if gesture is None or gesture not in gd.GESTURE_INDEX:
            self._bump("none_gesture")
            return False
        try:
            feature = self._feature_fn(landmarks)
        except (ValueError, TypeError):
            self._bump("bad_landmarks")
            return False
        self._features.append(np.asarray(feature, dtype=np.float32))
        self._labels.append(gd.GESTURE_INDEX[gesture])
        self._counts[gesture] = self._counts.get(gesture, 0) + 1
        return True

    def _bump(self, reason: str) -> None:
        self._dropped[reason] = self._dropped.get(reason, 0) + 1

    # ------------------------------------------------------------------ 查询
    @property
    def total(self) -> int:
        """已计入的样本数。"""
        return len(self._labels)

    @property
    def counts(self) -> Dict[str, int]:
        """每类样本数（保持 :func:`gesture_dataset.trainable_gestures` 的顺序）。"""
        return dict(self._counts)

    @property
    def dropped(self) -> Dict[str, int]:
        """被丢弃帧的计数（按原因）。"""
        return dict(self._dropped)

    def missing_classes(self) -> List[str]:
        """一个样本都没有的类别。"""
        return [g for g, n in self._counts.items() if n == 0]

    def rare_classes(self, min_samples: int = MIN_SAMPLES_OK) -> List[str]:
        """有样本但数量少于 ``min_samples`` 的类别。"""
        return [g for g, n in self._counts.items() if 0 < n < int(min_samples)]

    def quality_report(self, min_samples: int = MIN_SAMPLES_OK) -> Dict:
        """采集质量小结：每类数量、缺失/偏少类别、是否真的可以拿去训练。"""
        missing = self.missing_classes()
        rare = self.rare_classes(min_samples)
        return {
            "total": self.total,
            "counts": dict(self._counts),
            "dropped": dict(self._dropped),
            "missing_classes": missing,
            "rare_classes": rare,
            "min_samples": int(min_samples),
            # 只有"没有缺失、也没有偏少"才算 ready；否则这份报告会自相矛盾
            "ready": self.total > 0 and not missing and not rare,
            "has_data": self.total > 0,
        }

    # ------------------------------------------------------------------ 落盘
    def arrays(self):
        """返回 ``(features (N,42), labels (N,))``；空会话返回 ``(None, None)``。"""
        if not self._labels:
            return None, None
        return (np.stack(self._features).astype(np.float32),
                np.array(self._labels, dtype=np.int64))

    def finalize(self, path: str, meta: Optional[Dict] = None) -> str:
        """存成 ``.npz``；没有样本时抛 ``ValueError``（不要产出一个空文件）。"""
        features, labels = self.arrays()
        if features is None:
            raise ValueError("这一轮没有采到任何有效样本"
                             "（丢弃统计：%s）" % (self.dropped or "{}"))
        info = {"collector": "record_gesture_session",
                "dropped": dict(self._dropped),
                "counts": dict(self._counts)}
        if meta:
            info.update(meta)
        return gd.save_session(path, features, labels, meta=info)

    def reset(self) -> None:
        """清空已采集的样本（丢弃统计也一起清）。"""
        self._features.clear()
        self._labels.clear()
        self._counts = {g: 0 for g in gd.trainable_gestures()}
        self._dropped.clear()


def summarize(report: Dict, write=None) -> None:
    """把 :meth:`SessionCollector.quality_report` 的结果写成文字。

    ``write`` 默认是 ``print``；录制脚本会传入自己的 ``log``，
    这样离线测试可以捕获输出，而不是直接写 stdout。
    """
    w = write or print
    w("本轮采集：%d 个样本" % report["total"])
    for g, n in report["counts"].items():
        flag = ""
        if n == 0:
            flag = "  <- 缺失"
        elif n < report["min_samples"]:
            flag = "  <- 偏少（建议 >= %d）" % report["min_samples"]
        w("  %-12s %4d%s" % (g, n, flag))
    if report["dropped"]:
        w("丢弃的帧：%s" % ", ".join("%s=%d" % (k, v)
                                     for k, v in sorted(report["dropped"].items())))
    if not report.get("has_data", report["total"] > 0):
        w("结论：没有采到样本")
    elif report["missing_classes"]:
        w("结论：还有类别缺失，建议补采 -> %s" % ", ".join(report["missing_classes"]))
    elif report.get("rare_classes"):
        w("结论：类别齐全但样本偏少（< %d），建议继续补采 -> %s"
          % (report["min_samples"], ", ".join(report["rare_classes"])))
    else:
        w("结论：可以直接拿去训练")
