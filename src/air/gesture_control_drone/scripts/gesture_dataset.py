#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手势关键点数据集工具 —— 为神经网络分类器准备训练数据。

为什么需要"归一化"
------------------
MediaPipe 给的 21 个关键点是**像素坐标**，同一手势在不同距离/分辨率下数值差
几十倍。直接把像素丢进网络，模型学到的会是"手离镜头多远"，而不是"手型"。

这里统一做三步（与 ``gesture_detection.palm_scale`` 的尺度定义一致）：

1. **平移**：全部关键点减去腕部（索引 0）坐标 -> 与手在画面中的位置无关；
2. **缩放**：除以手掌尺度（腕部 -> 中指根的距离）-> 与远近/分辨率无关；
3. **展平**：21 个点 × (x, y) = **42 维**特征向量（21 × 2）。

只取 (x, y)：AirSim 的 2D 摄像头与 MediaPipe 的 ``results.multi_hand_landmarks``
都是图像平面坐标，z 分量噪声大且对"手型"贡献小，先不用。

这个模块**不依赖 ROS / MediaPipe / cv2**，因此可以离线单测，也能在训练脚本里复用。
"""
from __future__ import annotations

import json
import os
import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

#: MediaPipe 手部关键点数量
NUM_LANDMARKS = 21
#: 每个点用 (x, y) 两维 -> 42 维特征
FEATURE_DIM = NUM_LANDMARKS * 2

#: 参与分类的手势（不含 NONE：NONE 表示"没有手/无法判定"，不进训练集）
#: 顺序必须与 ``gesture_map.GESTURE_TO_COMMAND`` 一致（类别 id 由它决定）。
#: 改顺序 = 改 label id，已训练模型的输出会错位，所以有专门的契约测试盯着。
_TRAIN_GESTURES = ("PALM_OPEN", "FIST", "POINT", "VICTORY",
                   "THUMB_DOWN", "THUMB_UP", "OK")
GESTURE_INDEX = {g: i for i, g in enumerate(_TRAIN_GESTURES)}
INDEX_GESTURE = {i: g for g, i in GESTURE_INDEX.items()}
#: 类别数（7）
NUM_CLASSES = len(_TRAIN_GESTURES)

#: 手掌尺度下限（像素）：防止检测抖动导致除零或特征爆炸
MIN_PALM_SCALE = 1e-3


def _as_landmarks(landmarks) -> np.ndarray:
    """把 ``[(x, y), ...]`` 转成 ``(21, 2)`` float 数组并校验形状。"""
    arr = np.asarray(landmarks, dtype=np.float32)
    if arr.shape != (NUM_LANDMARKS, 2):
        raise ValueError("关键点应为 (21, 2)，收到 shape=%s" % (arr.shape,))
    if not np.all(np.isfinite(arr)):
        raise ValueError("关键点含 NaN/Inf")
    return arr


def palm_scale(landmarks: np.ndarray) -> float:
    """手掌尺度：腕部(0) -> 中指根(9) 的距离（像素）。"""
    arr = _as_landmarks(landmarks)
    d = float(np.linalg.norm(arr[9] - arr[0]))
    return max(MIN_PALM_SCALE, d)


def normalize_landmarks(landmarks, scale: Optional[float] = None) -> np.ndarray:
    """关键点 -> 42 维归一化特征（平移 + 缩放）。"""
    arr = _as_landmarks(landmarks)
    s = palm_scale(arr) if scale is None else float(scale)
    if s <= 0.0:
        raise ValueError("scale 必须为正，收到 %.6f" % s)
    centered = (arr - arr[0]) / s          # 腕部为原点，按手掌尺度归一化
    return centered.reshape(-1).astype(np.float32)


def unnormalize_landmarks(features, scale: float = 100.0,
                          origin: Tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    """归一化特征 -> 像素关键点（调试图/回放用；``scale`` 为假定的手掌像素长度）。"""
    feat = np.asarray(features, dtype=np.float32).reshape(-1)
    if feat.size != FEATURE_DIM:
        raise ValueError("特征维度应为 %d，收到 %d" % (FEATURE_DIM, feat.size))
    pts = feat.reshape(NUM_LANDMARKS, 2) * float(scale)
    pts[:, 0] += origin[0]
    pts[:, 1] += origin[1]
    return pts.astype(np.float32)


# --------------------------------------------------------------------------- #
# 标签编码
# --------------------------------------------------------------------------- #
def label_index(gesture: str) -> int:
    """手势 token -> 类别下标；未知手势抛 ValueError（避免训练集里混进脏标签）。"""
    if gesture not in GESTURE_INDEX:
        raise ValueError("不是可训练手势：%r（可选：%s）"
                         % (gesture, ", ".join(_TRAIN_GESTURES)))
    return GESTURE_INDEX[gesture]


def index_label(index: int) -> str:
    """类别下标 -> 手势 token。"""
    index = int(index)
    if index not in INDEX_GESTURE:
        raise ValueError("类别下标越界：%d（0~%d）" % (index, NUM_CLASSES - 1))
    return INDEX_GESTURE[index]


def trainable_gestures() -> Tuple[str, ...]:
    """参与训练的手势（固定顺序，与 :data:`GESTURE_INDEX` 一致）。"""
    return _TRAIN_GESTURES


# --------------------------------------------------------------------------- #
# 数据集读写
# --------------------------------------------------------------------------- #
def save_session(path: str,
                 features: np.ndarray,
                 labels: np.ndarray,
                 meta: Optional[Dict] = None) -> str:
    """把一个采集会话存成 ``.npz``（features / labels / meta 三部分）。

    ``features``: ``(N, 42)`` float32；``labels``: ``(N,)`` int64。
    返回实际写入路径（自动补 ``.npz`` 后缀）。
    """
    feats = np.asarray(features, dtype=np.float32)
    labs = np.asarray(labels, dtype=np.int64)
    if feats.ndim != 2 or feats.shape[1] != FEATURE_DIM:
        raise ValueError("features 应为 (N, %d)，收到 %s" % (FEATURE_DIM, feats.shape))
    if labs.shape != (feats.shape[0],):
        raise ValueError("labels 长度与 features 不一致：%s vs %s"
                         % (labs.shape, feats.shape))
    if feats.shape[0] == 0:
        raise ValueError("空数据集不写盘（先确认采集到了数据）")
    if labs.min() < 0 or labs.max() >= NUM_CLASSES:
        raise ValueError("标签越界：范围应为 0~%d，收到 %d~%d"
                         % (NUM_CLASSES - 1, labs.min(), labs.max()))

    if not path.endswith(".npz"):
        path += ".npz"
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)

    info = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "num_samples": int(feats.shape[0]),
        "feature_dim": FEATURE_DIM,
        "num_classes": NUM_CLASSES,
        "classes": list(_TRAIN_GESTURES),
        "counts": {g: int((labs == i).sum()) for g, i in GESTURE_INDEX.items()},
    }
    if meta:
        info.update(meta)
    np.savez_compressed(path, features=feats, labels=labs,
                        meta=np.array(json.dumps(info, ensure_ascii=False)))
    return path


def load_session(path: str) -> Dict:
    """读回 ``.npz``；返回 ``{"features", "labels", "meta"}``。"""
    if not os.path.isfile(path):
        raise FileNotFoundError("找不到数据集文件：%s" % path)
    with np.load(path, allow_pickle=False) as data:
        if "features" not in data or "labels" not in data:
            raise ValueError("%s 不是本模块的数据集（缺 features/labels）" % path)
        feats = np.asarray(data["features"], dtype=np.float32)
        labs = np.asarray(data["labels"], dtype=np.int64)
        meta_raw = data.get("meta")
        meta = {}
        if meta_raw is not None:
            try:
                meta = json.loads(str(meta_raw))
            except (ValueError, TypeError):
                meta = {}
    if feats.ndim != 2 or feats.shape[1] != FEATURE_DIM:
        raise ValueError("%s 的特征维度不对：%s" % (path, feats.shape))
    return {"features": feats, "labels": labs, "meta": meta}


def merge_sessions(paths: Sequence[str], out_path: str,
                   shuffle: bool = True, seed: int = 0) -> str:
    """把多个会话合并成一个数据集（可按固定种子打乱）。"""
    if not paths:
        raise ValueError("至少需要一个会话文件")
    feats, labs = [], []
    for p in paths:
        item = load_session(p)
        feats.append(item["features"])
        labs.append(item["labels"])
    merged_f = np.concatenate(feats, axis=0)
    merged_l = np.concatenate(labs, axis=0)
    if shuffle:
        rng = np.random.RandomState(int(seed))
        order = rng.permutation(len(merged_l))
        merged_f, merged_l = merged_f[order], merged_l[order]
    return save_session(out_path, merged_f, merged_l,
                        meta={"merged_from": [os.path.basename(p) for p in paths]})


# --------------------------------------------------------------------------- #
# 统计与划分
# --------------------------------------------------------------------------- #
def dataset_stats(features, labels) -> Dict:
    """数据集统计：每类样本数、类别是否齐全、特征均值/标准差。"""
    feats = np.asarray(features, dtype=np.float32)
    labs = np.asarray(labels, dtype=np.int64)
    if feats.shape[0] == 0:
        raise ValueError("空数据集无法统计")
    if feats.shape[0] != labs.shape[0]:
        raise ValueError("features 与 labels 长度不一致：%d vs %d"
                         % (feats.shape[0], labs.shape[0]))
    # 统计的意义就是**发现**数据问题；标签越界必须报错，不能悄悄少样本。
    if labs.min() < 0 or labs.max() >= NUM_CLASSES:
        raise ValueError("标签越界：应在 0~%d，收到 %d~%d（有脏标签，"
                         "直接统计会把它们默默丢掉）"
                         % (NUM_CLASSES - 1, int(labs.min()), int(labs.max())))
    counts = {g: int((labs == i).sum()) for g, i in GESTURE_INDEX.items()}
    missing = [g for g, n in counts.items() if n == 0]
    return {
        "num_samples": int(feats.shape[0]),
        "feature_dim": int(feats.shape[1]),
        "counts": counts,
        "missing_classes": missing,
        "min_class_count": min(counts.values()) if counts else 0,
        "feature_mean": float(np.mean(feats)),
        "feature_std": float(np.std(feats)),
    }


def split_train_val(features, labels, val_ratio: float = 0.2, seed: int = 0,
                    return_index: bool = False):
    """按比例划分训练/验证集（分层抽样：每类各自切分）。

    返回 ``(x_train, y_train, x_val, y_val)``；``return_index=True`` 时返回
    ``(train_idx, val_idx, skipped_classes)``，方便调用方自己取子集或做校验。

    两个容易踩的坑（都已处理）：

    * **某类只有 1 个样本**时，不能把它整个切进验证集，否则训练集里这一类
      直接消失；这类样本全部留在训练集，并把类别记进 ``skipped_classes``。
    * 切完之后**不能 ``sorted()``**：那会把刚打乱的顺序又按类别聚回去，
      mini-batch 会连续吃到同一类。这里用 ``rng.permutation`` 再打散一次。
    """
    feats = np.asarray(features, dtype=np.float32)
    labs = np.asarray(labels, dtype=np.int64)
    if feats.shape[0] != labs.shape[0]:
        raise ValueError("features 与 labels 长度不一致：%d vs %d"
                         % (feats.shape[0], labs.shape[0]))
    if not (0.0 < val_ratio < 1.0):
        raise ValueError("val_ratio 必须在 (0, 1) 之间，收到 %.3f" % val_ratio)

    rng = np.random.RandomState(int(seed))
    train_idx, val_idx, skipped = [], [], []
    for cls in np.unique(labs):
        idx = np.where(labs == cls)[0]
        rng.shuffle(idx)
        if len(idx) < 2:                       # 太少：整类留在训练集
            train_idx.extend(idx.tolist())
            skipped.append(int(cls))
            continue
        n_val = max(1, int(round(len(idx) * val_ratio)))
        n_val = min(n_val, len(idx) - 1)       # 至少给训练集留 1 个
        val_idx.extend(idx[:n_val].tolist())
        train_idx.extend(idx[n_val:].tolist())

    train_idx = rng.permutation(np.array(train_idx, dtype=np.int64))
    val_idx = rng.permutation(np.array(val_idx, dtype=np.int64))
    if return_index:
        return train_idx, val_idx, skipped
    return (feats[train_idx], labs[train_idx], feats[val_idx], labs[val_idx])


def extract_from_result(result) -> Optional[Tuple[np.ndarray, str]]:
    """从一次识别结果里取出 (42 维特征, 手势 token)；不可训练时返回 ``None``。

    ``result`` 是 :class:`gesture_detection.GestureResult`（duck-typing，
    这里不 import 它，避免循环依赖）。规则法给出的标签是**弱标签**——
    采集时用它打标，训练前仍建议抽检。
    """
    landmarks = getattr(result, "landmarks", None)
    gesture = getattr(result, "gesture", None)
    if landmarks is None or gesture not in GESTURE_INDEX:
        return None
    return normalize_landmarks(landmarks), gesture


# --------------------------------------------------------------------------- #
# 命令行入口：不依赖 ROS，采集前/训练前都能直接跑
# --------------------------------------------------------------------------- #
def _make_synthetic_hand(kind, seed=0):
    """生成一只"合成手"的关键点，仅用于 selftest（让评审不用摄像头就能看到效果）。

    **坐标系必须和规则法一致**：图像坐标 y 向下，`finger_extended` 的判据是
    ``指尖 y < PIP y``（指尖在上 = 伸出）。早期版本把手指画成朝下，结果整只手
    上下颠倒——张开手掌被判成握拳、握拳被判成张开手掌，selftest 的"7 类各 N 个"
    只验证了按名字数数，**完全没有验证弱标签链路**。

    现在 selftest 会用 :func:`gesture_detection.classify_landmarks` 对每只合成手
    复判一次，见 ``_cmd_selftest`` 的第 [2] 步。
    """
    rng = np.random.RandomState(seed)
    base = np.array([
        [0.0, 0.0],                                                    # 0  腕
        [12.0, -5.0], [22.0, 5.0], [32.0, 10.0], [50.0, 18.0],         # 1-4  拇指（朝右伸）
        [-20.0, -45.0], [-20.0, -65.0], [-20.0, -75.0], [-20.0, -85.0],  # 5-8  食指
        [0.0, -50.0], [0.0, -70.0], [0.0, -80.0], [0.0, -90.0],        # 9-12 中指
        [15.0, -45.0], [15.0, -65.0], [15.0, -75.0], [15.0, -85.0],    # 13-16 无名指
        [28.0, -40.0], [28.0, -58.0], [28.0, -66.0], [28.0, -74.0],    # 17-20 小指
    ], dtype=np.float32)

    folded = ((8, 5), (12, 9), (16, 13), (20, 17))

    def fold_fingers():
        """手指收起来：指尖落到 PIP **下方**（y 更大）。"""
        for tip, mcp in folded:
            base[tip] = base[mcp] + np.array([0.0, 20.0], dtype=np.float32)

    def fold_thumb():
        """拇指收起来：指尖 x 偏移小于 0.25*掌宽（掌宽 = 50）。"""
        base[3] = np.array([18.0, -10.0], dtype=np.float32)
        base[4] = np.array([25.0, -20.0], dtype=np.float32)

    if kind == "palm":                       # 五指全伸 -> PALM_OPEN
        pass
    elif kind == "fist":                     # 全收 -> FIST
        fold_fingers()
        fold_thumb()
    elif kind == "point":                    # 只伸食指 -> POINT
        for tip, mcp in ((12, 9), (16, 13), (20, 17)):
            base[tip] = base[mcp] + np.array([0.0, 20.0], dtype=np.float32)
        fold_thumb()
    elif kind == "victory":                  # 食指 + 中指 -> VICTORY
        for tip, mcp in ((16, 13), (20, 17)):
            base[tip] = base[mcp] + np.array([0.0, 20.0], dtype=np.float32)
        fold_thumb()
    elif kind in ("thumbs_up", "thumbs_down"):
        fold_fingers()
        # 拇指既要"伸出"（x 偏移 18 > 0.25*50）又要朝上/朝下（竖直偏移 > 0.15*50=7.5）
        base[3] = np.array([32.0, 10.0], dtype=np.float32)
        ty = -30.0 if kind == "thumbs_up" else 50.0
        base[4] = np.array([50.0, ty], dtype=np.float32)
    elif kind == "ok":                       # 拇指尖贴食指尖，中/无名/小指伸出 -> OK
        base[4] = base[8] + np.array([2.0, 3.0], dtype=np.float32)
        base[3] = np.array([32.0, 10.0], dtype=np.float32)
    else:
        raise ValueError("未知合成手型：%r" % kind)

    base += rng.normal(0.0, 0.8, base.shape)     # 一点抖动
    return base


def _cmd_stats(args):
    item = load_session(args.path)
    st = dataset_stats(item["features"], item["labels"])
    print("文件      : %s" % args.path)
    print("样本数    : %d" % st["num_samples"])
    print("特征维度  : %d" % st["feature_dim"])
    print("每类样本  :")
    for g, n in st["counts"].items():
        print("  %-12s %5d" % (g, n))
    if st["missing_classes"]:
        print("缺失类别  : %s" % ", ".join(st["missing_classes"]))
    print("特征均值  : %+.4f   标准差: %.4f" % (st["feature_mean"], st["feature_std"]))
    if item["meta"].get("created_at"):
        print("采集时间  : %s" % item["meta"]["created_at"])
    # 录制脚本会把"被丢弃的帧"记在 meta 里（按原因分），这里一并显示，
    # 否则这些信息只有录制结束的小结里能看到。
    dropped = item["meta"].get("dropped") or {}
    if dropped:
        print("丢弃原因  : %s" % ", ".join("%s=%d" % (k, v)
                                           for k, v in sorted(dropped.items())))
    if item["meta"].get("fps_actual") is not None:
        print("实际采样率: %.1f Hz（目标 %s Hz）"
              % (float(item["meta"]["fps_actual"]),
                 item["meta"].get("fps_target", "?")))
    if item["meta"].get("interrupted"):
        print("注意      : 这次采集被中断过（Ctrl+C），样本是中断前的一部分")
    return 0


def _cmd_merge(args):
    out = merge_sessions(args.inputs, args.output, seed=args.seed)
    item = load_session(out)
    print("已合并 %d 个会话 -> %s（%d 个样本）"
          % (len(args.inputs), out, item["features"].shape[0]))
    return 0


def _cmd_selftest(args):
    """用合成关键点跑一遍完整流程，适合在没摄像头/没数据时验证环境。

    临时文件写到系统临时目录（``tempfile.mkdtemp``）并在结束时清理，
    **不会在源码目录留下未跟踪的中间文件**（用 ``--keep`` 可以保留查看）。
    """
    import shutil
    import tempfile

    print("[1] 生成合成数据（7 类 × %d 个样本）" % args.per_class)
    feats, labs = [], []
    mapping = {"palm": "PALM_OPEN", "fist": "FIST", "point": "POINT",
               "victory": "VICTORY", "thumbs_up": "THUMB_UP",
               "thumbs_down": "THUMB_DOWN", "ok": "OK"}
    for kind, name in mapping.items():
        for k in range(max(1, args.per_class // len(mapping))):
            hand = _make_synthetic_hand(kind, seed=k)
            feats.append(normalize_landmarks(hand))
            labs.append(label_index(name))
    feats = np.stack(feats)
    labs = np.array(labs, dtype=np.int64)

    print("[2] 弱标签复判（用规则法验证合成手，确保 fixture 与识别逻辑一致）")
    try:
        import gesture_detection as gdet
    except Exception as exc:                       # noqa: BLE001
        print("  跳过（缺依赖：%s）" % exc)
        gdet = None
    if gdet is not None:
        hit = 0
        for kind, name in mapping.items():
            hand = _make_synthetic_hand(kind, seed=0)
            pred = gdet.classify_landmarks([[int(x), int(y)] for x, y in hand])
            hit += int(pred == name)
            print("  %-12s 期望 %-11s 规则法 %-11s %s"
                  % (kind, name, pred, "OK" if pred == name else "MISMATCH"))
        if hit != len(mapping):
            raise SystemExit("弱标签复判失败：%d/%d" % (hit, len(mapping)))
        print("  弱标签复判 %d/%d 通过" % (hit, len(mapping)))

    print("[3] 统计")
    st = dataset_stats(feats, labs)
    for g, n in st["counts"].items():
        print("  %-12s %5d" % (g, n))

    print("[4] 存盘 + 读回 + 合并")
    tmpdir = tempfile.mkdtemp(prefix="gesture_selftest_")
    p1 = save_session(os.path.join(tmpdir, "a.npz"), feats[: len(feats) // 2],
                      labs[: len(labs) // 2])
    p2 = save_session(os.path.join(tmpdir, "b.npz"), feats[len(feats) // 2:],
                      labs[len(labs) // 2:])
    merged = merge_sessions([p1, p2], os.path.join(tmpdir, "merged.npz"))
    item = load_session(merged)
    print("  合并后样本数: %d" % item["features"].shape[0])

    print("[5] 训练/验证划分")
    xtr, ytr, xv, yv = split_train_val(item["features"], item["labels"], 0.25, seed=0)
    print("  训练 %d / 验证 %d" % (len(ytr), len(yv)))

    if getattr(args, "keep", False):
        print("OK：数据工具工作正常（临时数据保留在 %s）" % tmpdir)
    else:
        shutil.rmtree(tmpdir, ignore_errors=True)
        print("OK：数据工具工作正常（临时数据已清理）")
    return 0


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(
        description="手势关键点数据集工具（不依赖 ROS / MediaPipe）")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_stats = sub.add_parser("stats", help="查看一个数据集文件的统计")
    p_stats.add_argument("path")
    p_stats.set_defaults(func=_cmd_stats)

    p_merge = sub.add_parser("merge", help="合并多个会话")
    p_merge.add_argument("output")
    p_merge.add_argument("inputs", nargs="+")
    p_merge.add_argument("--seed", type=int, default=0)
    p_merge.set_defaults(func=_cmd_merge)

    p_self = sub.add_parser("selftest", help="用合成数据验证整条流程")
    p_self.add_argument("--per-class", type=int, default=70)
    p_self.add_argument("--keep", action="store_true",
                        help="保留临时目录（默认跑完就清理）")
    p_self.set_defaults(func=_cmd_selftest)

    args = parser.parse_args(argv)
    return args.func(args)


# --------------------------------------------------------------------------- #
# 自检
# --------------------------------------------------------------------------- #
def _self_check():                                   # pragma: no cover
    hand = np.array([[0.0, 0.0]] * NUM_LANDMARKS, dtype=np.float32)
    hand[9] = [0.0, 100.0]                            # 手掌尺度 = 100
    feat = normalize_landmarks(hand)
    assert feat.shape == (FEATURE_DIM,), feat.shape
    for g in trainable_gestures():
        assert index_label(label_index(g)) == g


_self_check()


if __name__ == "__main__":
    import sys
    sys.exit(main())
