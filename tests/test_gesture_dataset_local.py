#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``gesture_dataset`` 离线回归（不需要 ROS / MediaPipe / 摄像头）。

    python3 test_gesture_dataset.py
"""
from __future__ import annotations

import os
import shutil
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(_HERE)
sys.path.insert(0, os.path.join(_REPO, "src", "air", "gesture_control_drone", "scripts"))

import gesture_dataset as gd  # noqa: E402

_R = []


def check(name, ok, detail=""):
    _R.append((bool(ok), name, detail))
    if not ok:
        print("[失败] %s  %s" % (name, detail))


def make_hand(scale=100.0, offset=(320.0, 240.0)):
    """构造一只"手"：腕部在 offset，中指根在正下方 scale 像素处。

    其余关键点带一点随机偏移（模拟真实手型），但腕部(0)/中根(9)保持精确，
    这样 ``palm_scale`` 恰好等于 ``scale``，便于断言。
    """
    base = np.asarray(offset, dtype=np.float32)
    # 关键点之间的相对位置按 scale 成比例（真实手放大/缩小时就是这样）
    unit = np.array([[i * 0.005, i * 0.003] for i in range(21)], dtype=np.float32)
    pts = base + unit * float(scale)
    pts[0] = base
    pts[9] = np.array([base[0], base[1] + scale], dtype=np.float32)
    return pts


# ------------------------------------------------------------------ 归一化
def test_normalize():
    hand = make_hand(100.0, (320.0, 240.0))
    check("手掌尺度 = 100", abs(gd.palm_scale(hand) - 100.0) < 1e-3, gd.palm_scale(hand))

    feat = gd.normalize_landmarks(hand)
    check("特征维度 = 42（21 个点 × 2）", feat.shape == (42,), feat.shape)
    check("腕部被移到原点", np.allclose(feat[:2], [0.0, 0.0], atol=1e-6), feat[:2])

    # 平移不变
    moved = make_hand(100.0, (10.0, 20.0))
    check("平移不变", np.allclose(gd.normalize_landmarks(moved), feat, atol=1e-5))

    # 缩放不变
    scaled = make_hand(50.0, (320.0, 240.0))
    check("缩放不变（50 像素手掌）",
          np.allclose(gd.normalize_landmarks(scaled), feat, atol=1e-5))

    # 显示尺寸变化（160x120）也不影响
    check("分辨率无关", np.allclose(gd.normalize_landmarks(hand * 0.5), feat, atol=1e-5))

    # 反归一化往返
    back = gd.unnormalize_landmarks(feat, scale=100.0, origin=(320.0, 240.0))
    check("反归一化能还原关键点",
          np.allclose(back, hand, atol=1e-3), (back[9], hand[9]))

    # 异常
    for bad, what in ((np.zeros((20, 2)), "20 个点"), (np.zeros((21, 3)), "3 维")):
        try:
            gd.normalize_landmarks(bad)
            ok = False
        except ValueError:
            ok = True
        check("非法关键点（%s）应抛 ValueError" % what, ok)
    try:
        bad_nan = make_hand()
        bad_nan[0, 0] = np.nan
        gd.normalize_landmarks(bad_nan)
        ok = False
    except ValueError:
        ok = True
    check("含 NaN 应抛 ValueError", ok)


# ------------------------------------------------------------------ 标签
def test_labels():
    for g in gd.trainable_gestures():
        check("标签往返：%s" % g, gd.index_label(gd.label_index(g)) == g)
    check("类别数 = 7", gd.NUM_CLASSES == 7, gd.NUM_CLASSES)
    check("可训练手势不含 NONE", "NONE" not in gd.trainable_gestures())

    try:
        gd.label_index("NONE")
        ok = False
    except ValueError:
        ok = True
    check("NONE 不是可训练标签", ok)

    for bad in (-1, 7, 99):
        try:
            gd.index_label(bad)
            ok = False
        except ValueError:
            ok = True
        check("越界类别下标 %d 应抛 ValueError" % bad, ok)


# ------------------------------------------------------------------ 读写
def test_io(tmp):
    hand = make_hand()
    feats = np.stack([gd.normalize_landmarks(hand)] * 6)
    labs = np.array([0, 0, 1, 1, 2, 2], dtype=np.int64)

    p = gd.save_session(os.path.join(tmp, "s1.npz"), feats, labs,
                        meta={"note": "self check"})
    check("save_session 返回 .npz 路径", p.endswith(".npz"), p)
    check("文件已生成", os.path.isfile(p))

    item = gd.load_session(p)
    check("读回样本数一致", item["features"].shape == (6, 42), item["features"].shape)
    check("读回标签一致", np.array_equal(item["labels"], labs))
    check("meta 透传", item["meta"].get("note") == "self check", item["meta"].get("note"))
    check("meta 里记录了每类数量",
          item["meta"]["counts"]["PALM_OPEN"] == 2, item["meta"].get("counts"))

    # 空数据集 / 标签越界 / 维度错
    cases = [
        (np.zeros((0, 42), np.float32), np.zeros((0,), np.int64), "空数据集"),
        (feats, np.array([0, 0, 1, 1, 2, 9], np.int64), "标签越界"),
        (np.zeros((3, 10), np.float32), np.zeros((3,), np.int64), "维度错误"),
        (feats, np.zeros((3,), np.int64), "长度不一致"),
    ]
    for f, l, what in cases:
        try:
            gd.save_session(os.path.join(tmp, "bad.npz"), f, l)
            ok = False
        except ValueError:
            ok = True
        check("%s 应抛 ValueError" % what, ok)

    try:
        gd.load_session(os.path.join(tmp, "nope.npz"))
        ok = False
    except FileNotFoundError:
        ok = True
    check("读不存在的文件应抛 FileNotFoundError", ok)


# ------------------------------------------------------------------ 合并
def test_merge(tmp):
    hand = make_hand()
    f1 = np.stack([gd.normalize_landmarks(hand)] * 4)
    f2 = np.stack([gd.normalize_landmarks(hand)] * 6)
    p1 = gd.save_session(os.path.join(tmp, "a.npz"), f1,
                         np.array([0, 0, 1, 1], np.int64))
    p2 = gd.save_session(os.path.join(tmp, "b.npz"), f2,
                         np.array([2, 2, 3, 3, 6, 6], np.int64))
    out = gd.merge_sessions([p1, p2], os.path.join(tmp, "merged.npz"), seed=1)
    item = gd.load_session(out)
    check("合并后样本数 = 10", item["features"].shape[0] == 10, item["features"].shape)
    check("合并后标签计数保留",
          item["meta"]["counts"]["PALM_OPEN"] == 2 and item["meta"]["counts"]["OK"] == 2,
          item["meta"].get("counts"))
    check("meta 记录了来源", len(item["meta"].get("merged_from", [])) == 2)

    try:
        gd.merge_sessions([], os.path.join(tmp, "x.npz"))
        ok = False
    except ValueError:
        ok = True
    check("空列表合并应抛 ValueError", ok)


# ------------------------------------------------------------------ 统计与划分
def test_stats_split():
    hand = make_hand()
    feats = np.stack([gd.normalize_landmarks(hand)] * 70)
    labs = np.repeat(np.arange(7), 10).astype(np.int64)

    st = gd.dataset_stats(feats, labs)
    check("统计样本数 = 70", st["num_samples"] == 70, st["num_samples"])
    check("每类 10 个", all(v == 10 for v in st["counts"].values()), st["counts"])
    check("没有缺失类别", st["missing_classes"] == [], st["missing_classes"])

    st2 = gd.dataset_stats(feats[:10], labs[:10])
    check("缺失类别被识别", "FIST" in st2["missing_classes"], st2["missing_classes"])

    xtr, ytr, xv, yv = gd.split_train_val(feats, labs, val_ratio=0.2, seed=0)
    check("划分后总数不变", len(ytr) + len(yv) == 70, (len(ytr), len(yv)))
    check("验证集约占 20%", abs(len(yv) - 14) <= 2, len(yv))
    check("每类验证集至少 1 个", all((yv == c).sum() >= 1 for c in range(7)),
          str({c: int((yv == c).sum()) for c in range(7)}))

    # 索引级校验：训练/验证的**样本下标**不能有交集（原来的写法只比较标签集合，恒真）
    tr_idx, va_idx, skipped = gd.split_train_val(feats, labs, val_ratio=0.2,
                                                 seed=0, return_index=True)
    check("训练/验证索引无交集", len(set(tr_idx.tolist()) & set(va_idx.tolist())) == 0,
          str(len(set(tr_idx.tolist()) & set(va_idx.tolist()))))
    check("索引并集覆盖全部样本",
          sorted(tr_idx.tolist() + va_idx.tolist()) == list(range(70)),
          (len(tr_idx), len(va_idx)))
    check("本次划分没有跳过任何类别", skipped == [], str(skipped))

    # 第 2 条回归：打乱之后的标签不应该还按类聚在一起
    first_half = ytr[: len(ytr) // 2]
    check("训练集已打乱（前一半不是同一类）",
          len(set(first_half.tolist())) > 1, str(first_half.tolist()[:12]))

    # 第 1 条回归：某类只有 1 个样本时，必须留在训练集，不能整类消失
    tiny_feats = np.concatenate([feats[:1], feats[7:10]])       # 类 0 只有 1 个
    tiny_labs = np.array([0, 1, 1, 1], dtype=np.int64)
    tr_i, va_i, sk = gd.split_train_val(tiny_feats, tiny_labs, val_ratio=0.3,
                                        seed=0, return_index=True)
    check("单样本类别留在训练集", 0 in tiny_labs[tr_i].tolist(), tr_i.tolist())
    check("单样本类别记入 skipped", sk == [0], str(sk))
    check("校验集非空", len(va_i) > 0, len(va_i))

    # 第 3 条回归：脏标签必须报错，而不是被统计悄悄丢掉
    try:
        gd.dataset_stats(feats[:3], np.array([0, 7, 99], dtype=np.int64))
        ok = False
    except ValueError:
        ok = True
    check("dataset_stats 对越界标签抛 ValueError", ok)

    try:
        gd.split_train_val(feats, labs, val_ratio=1.5)
        ok = False
    except ValueError:
        ok = True
    check("非法 val_ratio 应抛 ValueError", ok)


# ------------------------------------------------------------------ 从识别结果提取
def test_gesture_map_contract():
    """_TRAIN_GESTURES 的顺序决定 label id，必须和 gesture_map.py 一致。"""
    import re as _re
    import os as _os

    repo = _os.path.dirname(_HERE)
    gm_path = _os.path.join(repo, "src", "air", "gesture_control_drone",
                            "scripts", "gesture_map.py")
    if not _os.path.isfile(gm_path):
        check("能找到 gesture_map.py", False, gm_path)
        return
    src = open(gm_path, encoding="utf-8").read()

    # 取 GESTURE_TO_COMMAND 字典里的 key（按文件里的书写顺序）
    block = src.split("GESTURE_TO_COMMAND = {", 1)[1].split("}", 1)[0]
    keys = _re.findall(r"(GESTURE_[A-Z_]+)\s*:", block)
    consts = dict(_re.findall(r'^(GESTURE_[A-Z_]+)\s*=\s*"([A-Z_]+)"', src,
                              flags=_re.M))
    map_gestures = [consts[k] for k in keys if k in consts]

    check("gesture_map 里有 7 个手势映射", len(map_gestures) == 7, str(map_gestures))
    check("本模块的可训练手势与 gesture_map 完全一致（顺序也一致）",
          list(gd.trainable_gestures()) == map_gestures,
          "%s vs %s" % (list(gd.trainable_gestures()), map_gestures))


def test_synthetic_hand_contract():
    """合成手必须与规则法一致——否则"合成数据 + 弱标签"这条链是错的。

    历史：早期 _make_synthetic_hand 把手指画成朝下（图像坐标 y 向下，
    指尖 y 应更小才是"伸出"），导致张开手掌被判成握拳、7 种形态全错。
    """
    try:
        import gesture_detection as gdet
    except Exception as exc:                    # noqa: BLE001
        check("跳过合成手复判（缺 cv2：%s）" % str(exc)[:40], True)
        return

    mapping = {"palm": "PALM_OPEN", "fist": "FIST", "point": "POINT",
               "victory": "VICTORY", "thumbs_up": "THUMB_UP",
               "thumbs_down": "THUMB_DOWN", "ok": "OK"}
    for kind, want in mapping.items():
        hand = gd._make_synthetic_hand(kind, seed=0)
        pred = gdet.classify_landmarks([[int(x), int(y)] for x, y in hand])
        check("合成手 %-11s -> %s" % (kind, want), pred == want, str(pred))


def test_extract():
    class _Result(object):
        def __init__(self, landmarks, gesture):
            self.landmarks = landmarks
            self.gesture = gesture

    hand = [tuple(p) for p in make_hand().tolist()]
    feat, label = gd.extract_from_result(_Result(hand, "OK"))
    check("能从识别结果取特征", feat.shape == (42,), feat.shape)
    check("标签取自规则法", label == "OK", label)

    check("NONE 不进数据集",
          gd.extract_from_result(_Result(hand, "NONE")) is None)
    check("没检测到手时返回 None",
          gd.extract_from_result(_Result(None, "OK")) is None)


def main():
    tmp = tempfile.mkdtemp(prefix="gesture_ds_")
    try:
        test_normalize()
        test_labels()
        test_io(tmp)
        test_merge(tmp)
        test_stats_split()
        test_gesture_map_contract()
        test_synthetic_hand_contract()
        test_extract()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    total = len(_R)
    passed = sum(1 for ok, _, _ in _R if ok)
    print("=" * 58)
    print("gesture_dataset 离线回归：%d/%d 通过" % (passed, total))
    print("=" * 58)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
