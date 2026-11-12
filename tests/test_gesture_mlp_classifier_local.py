#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``gesture_mlp_classifier`` 离线回归（不需要 ROS / cv2 / 摄像头）。

    python3 tests/test_gesture_mlp_classifier_local.py
"""
from __future__ import annotations

import os
import shutil
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
_SCRIPTS = os.path.join(_REPO, "src", "air", "gesture_control_drone", "scripts")
sys.path.insert(0, _SCRIPTS)

import gesture_dataset as gd            # noqa: E402
import gesture_map as gm                # noqa: E402
import gesture_mlp_classifier as gmc    # noqa: E402

_R = []


def check(name, ok, detail=""):
    _R.append((bool(ok), name, str(detail)))
    if not ok:
        print("[失败] %s  %s" % (name, detail))


class StubModel(object):
    """固定输出：第 idx 类概率 = top。"""

    def __init__(self, idx=1, top=0.9):
        self.classes = list(gd.trainable_gestures())
        self.idx = idx
        self.top = top

    def predict_proba(self, X):
        n = X.shape[0]
        p = np.full((n, 7), (1.0 - self.top) / 6.0, dtype=np.float32)
        p[:, self.idx] = self.top
        return p


def make_hand():
    """造一份形状合法、能被 normalize_landmarks 接受的关键点。"""
    pts = np.zeros((21, 2), dtype=np.float32)
    for i in range(21):
        pts[i] = (100.0 + (i % 5) * 12.0, 200.0 + (i // 5) * 15.0)
    return pts


# --------------------------------------------------------------------------- #
def test_find_model_path():
    real = gmc.find_model_path()
    check("能自动找到仓库模型", real is not None and os.path.isfile(real), real)
    check("返回绝对路径", real is None or os.path.isabs(real), real)

    rel = gmc.find_model_path("models/gesture_mlp.npz")
    check("相对包目录的路径能解析", rel is not None and os.path.isfile(rel), rel)

    missing = gmc.find_model_path("models/definitely_missing.npz")
    check("显式路径不存在时返回 None（不猜别的模型）", missing is None, missing)

    abs_missing = gmc.find_model_path(os.path.join(_REPO, "no_such_model.npz"))
    check("绝对路径不存在时返回 None", abs_missing is None, abs_missing)


def test_predict():
    hand = make_hand()
    clf = gmc.MLPClassifier(model=StubModel(idx=1, top=0.9), threshold=0.6)
    gesture, conf = clf.predict(hand)
    check("预测为第 2 类（FIST）", gesture == "FIST", gesture)
    check("置信度与模型输出一致", abs(conf - 0.9) < 1e-6, conf)
    check("记录了 last_confidence", abs(clf.last_confidence - 0.9) < 1e-6,
          clf.last_confidence)

    # 低于阈值 -> NONE（起飞/降落不可逆，宁可漏报）
    low = gmc.MLPClassifier(model=StubModel(idx=1, top=0.55), threshold=0.6)
    g2, c2 = low.predict(hand)
    check("低于阈值返回 NONE", g2 == gm.GESTURE_NONE, g2)
    check("低于阈值仍返回真实置信度", abs(c2 - 0.55) < 1e-6, c2)

    # 阈值边界
    edge = gmc.MLPClassifier(model=StubModel(idx=0, top=0.6), threshold=0.6)
    g3, _ = edge.predict(hand)
    check("置信度恰好等于阈值时放行", g3 == "PALM_OPEN", g3)

    # 没有关键点
    g4, c4 = gmc.MLPClassifier(model=StubModel()).predict(None)
    check("landmarks=None 返回 NONE", g4 == gm.GESTURE_NONE and c4 == 0.0, (g4, c4))

    # 关键点形状不对
    g5, c5 = gmc.MLPClassifier(model=StubModel()).predict(np.zeros((5, 2)))
    check("关键点形状错误返回 NONE", g5 == gm.GESTURE_NONE and c5 == 0.0, (g5, c5))


def test_from_file(tmp):
    real = gmc.find_model_path()
    clf = gmc.MLPClassifier.from_file(real, threshold=0.5)
    check("从文件加载成功", clf.model is not None)
    check("类别数 7", len(clf.classes) == 7, clf.classes)

    hand = make_hand()
    gesture, conf = clf.predict(hand)
    check("加载的模型能直接预测",
          gesture in gd.trainable_gestures() or gesture == gm.GESTURE_NONE,
          str((gesture, conf)))

    # 篡改 classes 顺序 -> 拒绝加载
    with np.load(real, allow_pickle=False) as data:
        payload = {k: data[k] for k in data.files}
    payload["classes"] = np.array(list(reversed(gd.trainable_gestures())))
    bad = os.path.join(tmp, "bad.npz")
    np.savez_compressed(bad, **payload)
    try:
        gmc.MLPClassifier.from_file(bad)
        ok = False
    except ValueError:
        ok = True
    check("类别顺序不一致时拒绝加载", ok)

    try:
        gmc.MLPClassifier.from_file(os.path.join(tmp, "nope.npz"))
        ok2 = False
    except FileNotFoundError:
        ok2 = True
    check("模型文件不存在时抛 FileNotFoundError", ok2)


def test_direct_construction_validation():
    """直接构造（不经 from_file）也要校验类别顺序。"""
    class WrongOrder(StubModel):
        def __init__(self):
            super(WrongOrder, self).__init__()
            self.classes = list(reversed(self.classes))

    try:
        gmc.MLPClassifier(model=WrongOrder())
        ok = False
    except ValueError:
        ok = True
    check("直接构造时类别顺序不一致抛 ValueError", ok)

    # 不带 classes 的模型：沿用词表，正常构造
    class NoClasses(object):
        def predict_proba(self, X):
            p = np.zeros((X.shape[0], 7), dtype=np.float32)
            p[:, 0] = 1.0
            return p

    clf = gmc.MLPClassifier(model=NoClasses())
    check("模型不带 classes 时沿用词表", clf.classes == list(gd.trainable_gestures()),
          str(clf.classes))
    gesture, conf = clf.predict(make_hand())
    check("无 classes 的模型也能预测", gesture == "PALM_OPEN" and conf > 0.99,
          str((gesture, conf)))


def main():
    tmp = tempfile.mkdtemp(prefix="gesture_mlp_")
    try:
        test_find_model_path()
        test_predict()
        test_from_file(tmp)
        test_direct_construction_validation()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    total = len(_R)
    passed = sum(1 for ok, _, _ in _R if ok)
    print("=" * 60)
    print("gesture_mlp_classifier 离线回归：%d/%d 通过" % (passed, total))
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
