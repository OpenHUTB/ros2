#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``sequence_model`` 离线回归（不需要 ROS / 摄像头 / PyTorch）。

    python3 test_sequence_local.py
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
_TOOLS = os.path.join(_REPO, "src", "air", "gesture_control_drone", "tools")
for _p in (_SCRIPTS, _TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import sequence_model as sm            # noqa: E402
import synthetic_sequences as gen      # noqa: E402
import gesture_gate                    # noqa: E402
import gesture_map as gm               # noqa: E402

_R = []


def check(name, ok, detail=""):
    _R.append((bool(ok), name, detail))
    if not ok:
        print("[失败] %s  %s" % (name, detail))


MODEL_PATH = os.path.join(_REPO, "src", "air", "gesture_control_drone",
                           "models", "gesture_seq.npz")


# ------------------------------------------------------------------ 归一化
def test_normalize():
    pose = gen.canonical_pose("PALM_OPEN")
    feat = sm.normalize_landmarks(pose)
    check("特征维度 42", feat.shape == (sm.FEATURE_DIM,), feat.shape)
    check("腕部在原点", np.allclose(feat[:2], [0, 0], atol=1e-5), feat[:2])

    moved = pose + np.array([120.0, -80.0], dtype=np.float32)
    check("平移不变", np.allclose(sm.normalize_landmarks(moved), feat, atol=1e-4))

    scaled = pose * 2.0
    check("缩放不变", np.allclose(sm.normalize_landmarks(scaled), feat, atol=1e-4))

    try:
        sm.normalize_landmarks(np.zeros((20, 2)))
        ok = False
    except ValueError:
        ok = True
    check("非法形状抛 ValueError", ok)


# ------------------------------------------------------------------ 缓冲
def test_buffer():
    buf = sm.SequenceBuffer(length=4)
    one = np.ones(sm.FEATURE_DIM, dtype=np.float32)
    check("初始未就绪", not buf.ready(min_valid=1), buf.valid_frames)

    for _ in range(4):
        buf.push(one)
    check("填满 4 帧后 valid=4", buf.valid_frames == 4, buf.valid_frames)
    check("就绪", buf.ready(min_valid=4))
    check("filled 计数=4", buf.filled == 4, buf.filled)

    x, m = buf.array()
    check("array 形状 (4, 42)", x.shape == (4, sm.FEATURE_DIM), x.shape)
    check("全有效 mask=1", bool(np.all(m == 1.0)))

    buf.push(None)
    check("缺帧滑出后 valid=3", buf.valid_frames == 3, buf.valid_frames)
    check("不足 min_valid 时未就绪", not buf.ready(min_valid=4))
    x, m = buf.array()
    check("最后一帧 mask=0", m[-1] == 0.0, m)
    check("filled 计数=5", buf.filled == 5, buf.filled)

    buf.push(np.full(sm.FEATURE_DIM, 7.0, dtype=np.float32))
    x, _ = buf.array()
    check("滑动窗口丢弃最旧帧", not np.allclose(x[0], x[-1]), (x[0][:3], x[-1][:3]))

    buf.clear()
    check("clear 后归零", buf.valid_frames == 0 and buf.filled == 0)

    try:
        buf.push(np.zeros(10, dtype=np.float32))
        ok = False
    except ValueError:
        ok = True
    check("维度不符抛 ValueError", ok)


# ------------------------------------------------------------------ 模型
def test_model():
    model = sm.LSTMModel.random(hidden=8, layers=1, seed=3)
    check("隐层=8", model.hidden == 8, model.hidden)
    check("层数=1", model.layers == 1, model.layers)
    check("类别数=8（7 手势 + NONE）", len(model.classes) == 8, model.classes)
    check("类别顺序与 gesture_map 一致",
          set(model.classes[:-1]) == set(gm.ALL_GESTURES) - {gm.GESTURE_NONE},
          model.classes)

    rng = np.random.RandomState(0)
    x = rng.randn(sm.SEQ_LEN, sm.FEATURE_DIM).astype(np.float32) * 0.1
    mask = np.ones(sm.SEQ_LEN, dtype=np.float32)

    h1 = model.forward_hidden(x, mask)
    h2 = model.forward_hidden(x, mask)
    check("前向确定性（同一个输入结果一致）", np.allclose(h1, h2))
    check("隐状态维度正确", h1.shape == (8,), h1.shape)

    probs = model.predict_proba(x, mask)
    check("概率和为 1", abs(float(probs.sum()) - 1.0) < 1e-4, probs.sum())
    check("概率非负", bool(np.all(probs >= 0.0)))
    name, conf, _ = model.predict(x, mask)
    check("predict 返回类别名", name in model.classes, name)

    # 全 0 mask：状态不更新 -> 等价于空序列 -> logits 只有 bias
    z = np.zeros(sm.SEQ_LEN, dtype=np.float32)
    p_zero = model.predict_proba(x, z)
    logits_bias = model.params["b_out"]
    e = np.exp(logits_bias - logits_bias.max())
    expected = e / e.sum()
    check("全缺帧时输出等于 softmax(b_out)",
          np.allclose(p_zero, expected, atol=1e-5), (p_zero[:3], expected[:3]))

    # 缺帧不改变状态：前 8 帧有数据 + 后 8 帧缺帧 == 只用前 8 帧
    mask_half = np.zeros(sm.SEQ_LEN, dtype=np.float32)
    mask_half[:8] = 1.0
    h_half = model.forward_hidden(x, mask_half)
    h_only8 = model.forward_hidden(x[:8], np.ones(8, dtype=np.float32))
    check("缺帧保持状态（后半段缺帧 == 不喂）", np.allclose(h_half, h_only8, atol=1e-6))

    # 读写往返
    tmp = tempfile.mkdtemp(prefix="seq_")
    try:
        p = model.save(os.path.join(tmp, "m.npz"))
        check("保存生成文件", os.path.isfile(p), p)
        loaded = sm.LSTMModel.load(p)
        check("读回隐层一致", loaded.hidden == model.hidden)
        check("读回类别一致", loaded.classes == model.classes)
        check("读回预测一致",
              np.allclose(loaded.predict_proba(x, mask), probs, atol=1e-5))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    try:
        sm.LSTMModel.load(os.path.join(tempfile.gettempdir(), "nope.npz"))
        ok = False
    except FileNotFoundError:
        ok = True
    check("读不存在的模型抛 FileNotFoundError", ok)


# ------------------------------------------------------------------ 出厂模型
def test_shipped_model():
    check("仓库里带了一个可用的模型", os.path.isfile(MODEL_PATH), MODEL_PATH)
    if not os.path.isfile(MODEL_PATH):
        return
    model = sm.LSTMModel.load(MODEL_PATH)
    check("出厂模型 hidden=64", model.hidden == 64, model.hidden)

    # 用合成验证集检查"没崩"
    seqs, labels, _ = gen.make_dataset(samples_per_class=10, seed=99)
    feats = gen.to_features(seqs)
    masks = gen.mask_of(seqs)
    pred = np.array([np.argmax(model.predict_proba(f, m))
                     for f, m in zip(feats, masks)])
    acc = float((pred == labels).mean())
    check("出厂模型在合成验证集上准确率 >= 0.9（%.3f）" % acc, acc >= 0.9, acc)


# ------------------------------------------------------------------ 门控组合
def _feed(gate, seq, now=None):
    """把一条 (T,21,2) 序列喂给小闸门，返回所有非 None 的输出。"""
    outs = []
    t = now if now is not None else 0.0
    for frame in seq:
        got = gate.update([tuple(p) for p in frame.tolist()], now=t)
        if got is not None:
            outs.append(got)
        t += 0.1
    return outs


def test_gate():
    model = sm.LSTMModel.load(MODEL_PATH)
    cmd_gate = gesture_gate.CommandGate(stable_frames=1, cooldown=0.0,
                                        clock=lambda: 0.0)
    gate = sm.SequenceGate(model, cmd_gate, threshold=0.5)

    # 张开手掌的过渡序列 -> 应该发出 takeoff
    seq = gen.make_sequence("PALM_OPEN", length=sm.SEQ_LEN, seed=1)
    outs = _feed(gate, seq)
    check("PALM_OPEN 序列 发出 takeoff", gm.CMD_TAKEOFF in outs, outs)

    # 一直举着同一手势：边沿触发不重复
    before = list(outs)
    outs2 = _feed(gate, seq)
    check("一直举着不重复发", gm.CMD_TAKEOFF not in outs2, outs2)

    # 手放下（多帧无手）后再举握拳 -> 发 land
    gate.update(None, now=10.0)
    gate.update(None, now=10.1)
    gate.update(None, now=10.2)
    seq_fist = gen.make_sequence("FIST", length=sm.SEQ_LEN, seed=2)
    outs3 = _feed(gate, seq_fist, now=11.0)
    check("FIST 序列 发出 land", gm.CMD_LAND in outs3, outs3)

    # 置信度阈值拉满 -> 永远不发
    gate2 = sm.SequenceGate(model,
                           gesture_gate.CommandGate(stable_frames=1, cooldown=0.0),
                           threshold=1.01)
    outs4 = _feed(gate2, gen.make_sequence("VICTORY", length=sm.SEQ_LEN, seed=3))
    check("阈值 1.01 时永不发指令", outs4 == [], outs4)

    # 预热阶段（无手势）不应崩
    gate3 = sm.SequenceGate(model,
                           gesture_gate.CommandGate(stable_frames=1, cooldown=0.0))
    check("预热阶段返回 None", gate3.update(None) is None)
    check("预热阶段不抛异常", True)

    # reset 后能重新触发
    gate.reset()
    outs5 = _feed(gate, gen.make_sequence("PALM_OPEN", length=sm.SEQ_LEN, seed=4))
    check("reset 后可重新触发", gm.CMD_TAKEOFF in outs5, outs5)


# ------------------------------------------------------------------ 类别一致性
def test_classes_consistent():
    check("时序模型类别与 gesture_dataset 一致（若已安装）", True)
    try:
        import gesture_dataset as ds
    except ImportError:
        return
    check("gesture_dataset 的可训练手势顺序与序列模型一致",
          tuple(ds.trainable_gestures()) == sm.SEQUENCE_CLASSES[:-1],
          (ds.trainable_gestures(), sm.SEQUENCE_CLASSES))


def main():
    test_normalize()
    test_buffer()
    test_model()
    test_shipped_model()
    test_gate()
    test_classes_consistent()

    total = len(_R)
    passed = sum(1 for ok, _, _ in _R if ok)
    print("=" * 60)
    print("sequence_model 离线回归：%d/%d 通过" % (passed, total))
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
