#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把训练好的 MLP 接成"每帧一个手势"的分类器（**纯逻辑，不依赖 cv2/ROS**）。

它与规则法的关系
----------------
* **输入相同**：MediaPipe 的 21 个关键点；
* **输出相同**：`gesture_map` 里 7 个手势 token 之一（或 `NONE`）；
* **判定不同**：规则法靠手指伸展的几何启发式；这里用 `gesture_train.MLP`
  （42 维归一化特征 -> 64 隐层 -> 7 类 softmax）。

因此节点里只要换一个"分类器实现"，下游的 `CommandGate` / 词表 / 执行链路完全不用动。

为什么单独一个文件
------------------
节点（`gesture_ros_node.py`）要 import cv2 / rospy，测试跑不动；这里只依赖
`gesture_dataset`（归一化）和传入的模型对象（duck-typing），因此可以离线单测，
也能在训练脚本里复用。

模型来源
--------
训练产物由 ``gesture_train.py`` 写出（``42 -> 64 -> 7``，npz 里带 ``classes``）。
:meth:`MLPClassifier.from_file` 会校验类别顺序，词表变过就拒绝加载。
"""
from __future__ import annotations

import os
from typing import List, Optional, Tuple

import numpy as np

import gesture_dataset as gd
from gesture_map import GESTURE_NONE

#: 低于这个置信度就当"没识别出"，避免拿不准的时候瞎发指令
DEFAULT_THRESHOLD = 0.6
#: 包名，用于 install 空间的路径兜底
PKG_NAME = "gesture_control_drone"
#: 默认模型文件名
DEFAULT_MODEL_NAME = "gesture_mlp.npz"


def model_candidates(name: str = DEFAULT_MODEL_NAME,
                     script_dir: Optional[str] = None) -> List[str]:
    """按可能的布局列出模型路径候选。

    * 源码树 / devel 启动器：``scripts/../models/x.npz``
    * install 空间：``scripts`` 被复制到 ``install/lib/<pkg>/``，模型在
      ``install/share/<pkg>/models/``
    * 兜底：``rospkg`` 查包目录
    """
    here = script_dir or os.path.dirname(os.path.abspath(__file__))
    cands = [
        os.path.join(here, os.pardir, "models", name),
        os.path.join(here, os.pardir, os.pardir, "share", PKG_NAME, "models", name),
        os.path.join(here, "models", name),
    ]
    try:
        import rospkg
        cands.append(os.path.join(rospkg.RosPack().get_path(PKG_NAME), "models", name))
    except Exception:                                  # noqa: BLE001 - 没有 ros 很正常
        pass
    return cands


def find_model_path(value: Optional[str] = None,
                    script_dir: Optional[str] = None) -> Optional[str]:
    """把 ``mlp/model_path`` 参数解析成真实文件路径；找不到返回 ``None``。

    参数可以写绝对路径、相对包目录的路径，或留空用默认文件名。
    """
    here = script_dir or os.path.dirname(os.path.abspath(__file__))
    if value:
        if os.path.isabs(value):
            return os.path.abspath(value) if os.path.isfile(value) else None
        # 相对路径依次按：包目录、scripts/ 目录、install 的 share/<pkg>/ 解析
        bases = (os.path.join(here, os.pardir), here,
                 os.path.join(here, os.pardir, os.pardir, "share", PKG_NAME))
        for base in bases:
            cand = os.path.normpath(os.path.join(base, value))
            if os.path.isfile(cand):
                return os.path.abspath(cand)
        # 显式配置了路径却找不到：不猜。让调用方回退到规则法并告警，
        # 而不是悄悄换成另一个"碰巧存在"的模型（那才是真正难查的问题）。
        return None
    for cand in model_candidates(script_dir=script_dir):
        if os.path.isfile(cand):
            return os.path.abspath(cand)
    return None


class MLPClassifier(object):
    """每帧分类：关键点 -> (手势 token, 置信度)。"""

    def __init__(self, model, threshold: float = DEFAULT_THRESHOLD):
        self.model = model
        self.threshold = float(threshold)
        classes = list(getattr(model, "classes", gd.trainable_gestures()))
        # 直接构造时也要校验：模型自带 classes 且顺序和词表不一致 -> 花式错标
        if classes != list(gd.trainable_gestures()):
            raise ValueError(
                "模型的类别顺序与当前词表不一致，不能直接使用。\n"
                "  模型: %s\n  词表: %s\n（词表改过之后必须重新训练）"
                % (classes, list(gd.trainable_gestures())))
        self.classes = classes
        self.last_confidence = 0.0
        self.last_gesture = GESTURE_NONE

    # ------------------------------------------------------------------ 构造
    @classmethod
    def from_file(cls, path: str, threshold: float = DEFAULT_THRESHOLD) -> "MLPClassifier":
        """从 ``gesture_train.py`` 产出的 npz 加载；类别顺序不一致会抛 ValueError。"""
        try:
            from gesture_train import MLP      # 延迟 import：只有这一步需要它
        except ImportError as exc:
            raise RuntimeError(
                "缺少 gesture_train.py（训练脚本），无法读取模型：%s" % exc) from exc
        return cls(MLP.load(path), threshold=threshold)

    # ------------------------------------------------------------------ 分类
    def predict(self, landmarks) -> Tuple[str, float]:
        """返回 ``(手势 token, 置信度)``；关键点异常时返回 ``("NONE", 0.0)``。"""
        if landmarks is None:
            self.last_gesture, self.last_confidence = GESTURE_NONE, 0.0
            return GESTURE_NONE, 0.0
        try:
            feature = gd.normalize_landmarks(landmarks)
        except (ValueError, TypeError):
            self.last_gesture, self.last_confidence = GESTURE_NONE, 0.0
            return GESTURE_NONE, 0.0
        probs = np.asarray(self.model.predict_proba(feature.reshape(1, -1))[0],
                           dtype=np.float32)
        idx = int(np.argmax(probs))
        conf = float(probs[idx])
        gesture = self.classes[idx]
        self.last_gesture, self.last_confidence = gesture, conf
        if conf < self.threshold:
            # 拿不准就当没看出来 —— 宁可漏报，也不要瞎发指令（起飞/降落是不可逆动作）
            return GESTURE_NONE, conf
        return gesture, conf
