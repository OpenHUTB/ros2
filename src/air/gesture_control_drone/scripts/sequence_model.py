#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手势**时序**模型：用连续多帧关键点判断手势，替代"连续 N 帧一致"的硬去抖。

为什么需要时序
--------------
现有 :class:`gesture_gate.CommandGate` 靠 ``stable_frames``（连续几帧同一手势）
去抖，本质是**固定延迟的多数表决**：

* 手快速切换（张开 -> 握拳）时，中间会经过若干"半握"帧，
  规则法在这些帧上输出跳变，导致**误触发**；
* 想更稳就把 ``stable_frames`` 调大，代价是**响应变慢**（每帧 ~100 ms）。

LSTM 看 16 帧的**运动轨迹**（手指在张开/收拢过程中怎么走），
理论上能区分"正在张开"和"正在握拳"，从而在**同一延迟下**降低误触发。

实现要点
--------
1. **推理只用 numpy**：机器人/虚拟机上不需要装 PyTorch。
   训练用 `tools/train_sequence_model.py`（可选 torch，或用 numpy 岭回归）。
2. **门控复用** :class:`gesture_gate.CommandGate`：本模块只负责
   "序列 -> 手势 token"，去抖/边沿/冷却仍由 CommandGate 管，行为不会变两套。
3. **缺帧处理**：没检测到手时保持上一步的隐状态（不是喂 0），
   避免"手一抖消失"把 LSTM 的记忆冲掉。

LSTM 前向公式（与 PyTorch ``nn.LSTM`` 的门顺序一致：i, f, g, o）
----------------------------------------------------------------
对第 ``l`` 层、时刻 ``t``::

    gates = x_t W_ih^T + b_ih + h_{t-1} W_hh^T + b_hh        (4H 维)
    i = sigmoid(gates[0:H])         输入门
    f = sigmoid(gates[H:2H])        遗忘门
    g = tanh   (gates[2H:3H])       候选记忆
    o = sigmoid(gates[3H:4H])       输出门
    c_t = f * c_{t-1} + i * g
    h_t = o * tanh(c_t)

序列末端隐状态 ``h_T`` 接一层线性分类器得到 logits，softmax 后即各类概率。
"""
from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Optional, Sequence

import numpy as np

#: 每个关键点用 (x, y) 两维
FEATURE_DIM = 42
#: 输入序列长度（16 帧 @ ~10 Hz = 1.6 s，足够覆盖一次手势动作）
SEQ_LEN = 16
#: 开始出预测所需的最少有效帧（预热）
MIN_VALID_FRAMES = 4

#: 类别顺序：7 个手势 + NONE
SEQUENCE_CLASSES = ("PALM_OPEN", "FIST", "POINT", "VICTORY",
                    "THUMB_UP", "THUMB_DOWN", "OK", "NONE")
CLASS_INDEX = {c: i for i, c in enumerate(SEQUENCE_CLASSES)}
NONE_INDEX = CLASS_INDEX["NONE"]
NUM_CLASSES = len(SEQUENCE_CLASSES)


# --------------------------------------------------------------------------- #
# 特征归一化：优先用 gesture_dataset（PR #714），没有则用本地等价实现
# --------------------------------------------------------------------------- #
try:                                     # pragma: no cover - 取决于安装环境
    from gesture_dataset import normalize_landmarks as _normalize_external
except ImportError:                      # 独立 PR 时不依赖 #714
    _normalize_external = None


def normalize_landmarks(landmarks) -> np.ndarray:
    """21 个关键点 -> 42 维归一化特征（平移 + 缩放）。

    有 ``gesture_dataset`` 时直接复用（保证训练/推理特征完全一致），
    否则用本地实现（公式与之一致）。
    """
    if _normalize_external is not None:
        return np.asarray(_normalize_external(landmarks), dtype=np.float32)
    arr = np.asarray(landmarks, dtype=np.float32)
    if arr.shape != (21, 2):
        raise ValueError("关键点应为 (21, 2)，收到 %s" % (arr.shape,))
    if not np.all(np.isfinite(arr)):
        raise ValueError("关键点含 NaN/Inf")
    scale = max(1e-3, float(np.linalg.norm(arr[9] - arr[0])))
    return ((arr - arr[0]) / scale).reshape(-1).astype(np.float32)


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


# --------------------------------------------------------------------------- #
# 模型参数
# --------------------------------------------------------------------------- #
class LSTMModel(object):
    """一层或多层 LSTM + 线性分类头（纯 numpy 前向）。

    参数用 dict 存放，键名与 PyTorch 的 ``nn.LSTM`` 一一对应
    （``W_ih_<l>`` / ``W_hh_<l>`` / ``b_ih_<l>`` / ``b_hh_<l>``），
    因此训练脚本可以直接把 ``state_dict`` 转过来，不需要手抄权重。
    """

    def __init__(self, params: Dict[str, np.ndarray],
                 classes: Sequence[str] = SEQUENCE_CLASSES,
                 seq_len: int = SEQ_LEN, temperature: float = 1.0):
        self.params = {k: np.asarray(v, dtype=np.float32) for k, v in params.items()}
        self.classes = tuple(classes)
        self.seq_len = int(seq_len)
        #: softmax 温度（训练后校准，见 train_sequence_model.fit_temperature）
        self.temperature = max(1e-3, float(temperature))
        self._validate()

    # ---------------------------------------------------------------- 构造
    @classmethod
    def random(cls, hidden: int = 64, layers: int = 1, seed: int = 0,
               input_dim: int = FEATURE_DIM, num_classes: int = NUM_CLASSES,
               seq_len: int = SEQ_LEN) -> "LSTMModel":
        """随机初始化（也可当作"储层"：只训练最后的线性头）。"""
        rng = np.random.RandomState(seed)
        params = {}
        for l in range(layers):
            dim_in = input_dim if l == 0 else hidden
            # PyTorch 的初始化范围 ±1/sqrt(hidden)
            k = 1.0 / math.sqrt(hidden)
            params["W_ih_%d" % l] = rng.uniform(-k, k, (4 * hidden, dim_in))
            params["W_hh_%d" % l] = rng.uniform(-k, k, (4 * hidden, hidden))
            params["b_ih_%d" % l] = rng.uniform(-k, k, (4 * hidden,))
            params["b_hh_%d" % l] = rng.uniform(-k, k, (4 * hidden,))
        k_out = 1.0 / math.sqrt(max(1, hidden))
        params["W_out"] = rng.uniform(-k_out, k_out, (num_classes, hidden))
        params["b_out"] = np.zeros((num_classes,), dtype=np.float32)
        params["__layers__"] = np.array([layers], dtype=np.int32)
        params["__hidden__"] = np.array([hidden], dtype=np.int32)
        params["__input_dim__"] = np.array([input_dim], dtype=np.int32)
        return cls(params, seq_len=seq_len)

    def _validate(self):
        if "__layers__" not in self.params:
            raise ValueError("参数缺少 __layers__（不是本模块导出的模型？）")
        layers = int(self.params["__layers__"][0])
        hidden = int(self.params["__hidden__"][0])
        input_dim = int(self.params["__input_dim__"][0])
        if input_dim != FEATURE_DIM:
            raise ValueError("模型输入维度 %d 与当前特征维度 %d 不一致"
                             % (input_dim, FEATURE_DIM))
        for l in range(layers):
            for key, shape in (("W_ih_%d" % l, (4 * hidden,
                                                 input_dim if l == 0 else hidden)),
                               ("W_hh_%d" % l, (4 * hidden, hidden)),
                               ("b_ih_%d" % l, (4 * hidden,)),
                               ("b_hh_%d" % l, (4 * hidden,))):
                if key not in self.params:
                    raise ValueError("参数缺少 %s" % key)
                if tuple(self.params[key].shape) != shape:
                    raise ValueError("%s 形状应为 %s，实际 %s"
                                     % (key, shape, self.params[key].shape))
        if tuple(self.params["W_out"].shape) != (len(self.classes), hidden):
            raise ValueError("W_out 形状应与 (类别数, 隐层) 一致，实际 %s"
                             % (self.params["W_out"].shape,))

    # ---------------------------------------------------------------- 前向
    @property
    def layers(self) -> int:
        return int(self.params["__layers__"][0])

    @property
    def hidden(self) -> int:
        return int(self.params["__hidden__"][0])

    def forward_hidden(self, x: np.ndarray,
                       mask: Optional[np.ndarray] = None) -> np.ndarray:
        """跑完整条序列，返回末端隐状态 ``h_T``（维度 = hidden）。

        ``mask`` 为 0 的时刻**保持状态不变**（等价于"没检测到手，什么都不做"）。
        """
        x = np.asarray(x, dtype=np.float32)
        if x.ndim != 2 or x.shape[1] != FEATURE_DIM:
            raise ValueError("输入应为 (T, %d)，收到 %s" % (FEATURE_DIM, x.shape))
        T = x.shape[0]
        if mask is None:
            mask = np.ones(T, dtype=np.float32)
        mask = np.asarray(mask, dtype=np.float32)
        if mask.shape != (T,):
            raise ValueError("mask 长度应为 %d，收到 %s" % (T, mask.shape))

        H = self.hidden
        h = [np.zeros(H, dtype=np.float32) for _ in range(self.layers)]
        c = [np.zeros(H, dtype=np.float32) for _ in range(self.layers)]
        for t in range(T):
            if mask[t] <= 0.0:
                continue                       # 缺帧：保持隐状态
            inp = x[t]
            for l in range(self.layers):
                gates = (inp @ self.params["W_ih_%d" % l].T
                         + self.params["b_ih_%d" % l]
                         + h[l] @ self.params["W_hh_%d" % l].T
                         + self.params["b_hh_%d" % l])
                i = _sigmoid(gates[0:H])
                f = _sigmoid(gates[H:2 * H])
                g = np.tanh(gates[2 * H:3 * H])
                o = _sigmoid(gates[3 * H:4 * H])
                c[l] = f * c[l] + i * g
                h[l] = o * np.tanh(c[l])
                inp = h[l]
        return h[-1]

    def logits(self, x: np.ndarray,
               mask: Optional[np.ndarray] = None) -> np.ndarray:
        """返回未归一化的 logits（长度 = 类别数）。"""
        h = self.forward_hidden(x, mask)
        return (h @ self.params["W_out"].T + self.params["b_out"]).astype(np.float32)

    def predict_proba(self, x: np.ndarray,
                      mask: Optional[np.ndarray] = None) -> np.ndarray:
        """返回各类别概率（含温度校准）。

        为什么需要温度：岭回归/小模型出来的 logits 往往很"平"
        （softmax 接近均匀），argmax 可能完全正确但置信度只有 0.2 左右，
        直接拿去和 0.6 阈值比就永远触发不了。温度在训练后拟合，
        只影响置信度的**标定**，不改变 argmax。
        """
        logits = self.logits(x, mask) / self.temperature
        logits = logits - np.max(logits)                 # 数值稳定
        e = np.exp(logits)
        return (e / np.sum(e)).astype(np.float32)

    def predict(self, x: np.ndarray, mask: Optional[np.ndarray] = None):
        """返回 ``(类别名, 置信度, 概率向量)``。"""
        probs = self.predict_proba(x, mask)
        idx = int(np.argmax(probs))
        return self.classes[idx], float(probs[idx]), probs

    # ---------------------------------------------------------------- 读写
    def save(self, path: str) -> str:
        """存成 ``.npz``（含类别名与元信息）。"""
        if not path.endswith(".npz"):
            path += ".npz"
        directory = os.path.dirname(os.path.abspath(path))
        os.makedirs(directory, exist_ok=True)
        payload = dict(self.params)
        payload["__classes__"] = np.array(json.dumps(list(self.classes)))
        payload["__seq_len__"] = np.array([self.seq_len], dtype=np.int32)
        payload["__temperature__"] = np.array([self.temperature], dtype=np.float32)
        np.savez_compressed(path, **payload)
        return path

    @classmethod
    def load(cls, path: str) -> "LSTMModel":
        if not os.path.isfile(path):
            raise FileNotFoundError("找不到模型文件：%s" % path)
        with np.load(path, allow_pickle=False) as data:
            params = {k: data[k] for k in data.files if not k.startswith("__")}
            classes = list(SEQUENCE_CLASSES)
            if "__classes__" in data.files:
                classes = json.loads(str(data["__classes__"]))
            params["__layers__"] = np.array([int(data["__layers__"][0])], np.int32)
            params["__hidden__"] = np.array([int(data["__hidden__"][0])], np.int32)
            params["__input_dim__"] = np.array([int(data["__input_dim__"][0])], np.int32)
            seq_len = int(data["__seq_len__"][0]) if "__seq_len__" in data.files else SEQ_LEN
            temp = float(data["__temperature__"][0]) if "__temperature__" in data.files else 1.0
        return cls(params, classes=classes, seq_len=seq_len, temperature=temp)


# --------------------------------------------------------------------------- #
# 序列缓冲
# --------------------------------------------------------------------------- #
class SequenceBuffer(object):
    """定长滑动窗口：保存最近 ``length`` 帧特征，缺帧用 0 填充并记 mask。"""

    def __init__(self, length: int = SEQ_LEN, feature_dim: int = FEATURE_DIM):
        if length < 1:
            raise ValueError("length 必须 >= 1")
        self.length = int(length)
        self.feature_dim = int(feature_dim)
        self._x = np.zeros((length, feature_dim), dtype=np.float32)
        self._mask = np.zeros((length,), dtype=np.float32)
        self._count = 0                      # 已喂入的帧数（含缺帧）

    def push(self, features: Optional[np.ndarray]) -> None:
        """压入一帧；``None`` 表示这一帧没有检测到手。"""
        if features is None:
            vec = np.zeros(self.feature_dim, dtype=np.float32)
            valid = 0.0
        else:
            vec = np.asarray(features, dtype=np.float32).reshape(-1)
            if vec.size != self.feature_dim:
                raise ValueError("特征维度应为 %d，收到 %d"
                                 % (self.feature_dim, vec.size))
            valid = 1.0
        self._x = np.roll(self._x, -1, axis=0)
        self._mask = np.roll(self._mask, -1, axis=0)
        self._x[-1] = vec
        self._mask[-1] = valid
        self._count += 1

    def clear(self) -> None:
        self._x[:] = 0.0
        self._mask[:] = 0.0
        self._count = 0

    @property
    def valid_frames(self) -> int:
        """窗口内有效帧数。"""
        return int(np.count_nonzero(self._mask))

    @property
    def filled(self) -> int:
        """已喂入的帧数（用于判断是否预热完成）。"""
        return self._count

    def ready(self, min_valid: int = MIN_VALID_FRAMES) -> bool:
        """是否已经够帧数可以做预测。"""
        return self.valid_frames >= max(1, int(min_valid))

    def array(self):
        """返回 ``(x, mask)`` 两份拷贝（形状 ``(T, 42)`` 与 ``(T,)``）。"""
        return self._x.copy(), self._mask.copy()


# --------------------------------------------------------------------------- #
# 组合：时序分类 + 现有 CommandGate
# --------------------------------------------------------------------------- #
class SequenceGate(object):
    """把"关键点序列"变成"该发的指令"，内部复用 :class:`CommandGate`。

    与直接调 ``CommandGate.update(token)`` 的区别只有一个：
    **入参从"单帧手势"变成"关键点"**，中间多了一层 LSTM 时序判断。
    去抖 / 边沿触发 / 冷却的行为完全不变。
    """

    def __init__(self, model: LSTMModel, gate, threshold: float = 0.6,
                 min_valid: int = MIN_VALID_FRAMES, margin: float = 0.0):
        from gesture_map import CMD_NONE, GESTURE_NONE, gesture_to_command

        self._cmd_none = CMD_NONE
        self._gesture_none = GESTURE_NONE
        self._gesture_to_command = gesture_to_command
        self.model = model
        self.gate = gate
        self.threshold = float(threshold)
        self.margin = float(margin)
        self.buffer = SequenceBuffer(length=model.seq_len)
        self.min_valid = int(min_valid)
        self.last_probs = None
        self.last_confidence = 0.0
        self.last_gesture = self._gesture_none

    def update(self, landmarks, now=None):
        """喂入**单帧关键点**（或 ``None``），返回需要发布的指令 token 或 ``None``。"""
        features = None
        if landmarks is not None:
            try:
                features = normalize_landmarks(landmarks)
            except ValueError:
                features = None
        self.buffer.push(features)

        if not self.buffer.ready(self.min_valid):
            # 预热阶段：当作"没有手势"，让 CommandGate 记录释放状态
            return self.gate.update(self._cmd_none, now)

        x, mask = self.buffer.array()
        probs = self.model.predict_proba(x, mask)
        self.last_probs = probs
        idx = int(np.argmax(probs))
        conf = float(probs[idx])
        self.last_confidence = conf
        self.last_gesture = self.model.classes[idx]

        # 置信度不够 / 判定为"没有手势" / 与第二名差距太小 -> 视作无手势
        second = float(np.max(np.delete(probs, idx))) if len(probs) > 1 else 0.0
        if (idx == NONE_INDEX or conf < self.threshold
                or (conf - second) < self.margin):
            return self.gate.update(self._cmd_none, now)

        command = self._gesture_to_command(self.last_gesture)
        return self.gate.update(command, now)

    def reset(self):
        self.buffer.clear()
        self.gate.reset()
        self.last_probs = None
        self.last_confidence = 0.0
        self.last_gesture = self._gesture_none
