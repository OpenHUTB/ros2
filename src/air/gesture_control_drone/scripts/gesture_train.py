#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手势分类器的训练与评估（**纯 numpy**，不需要 PyTorch）。

为什么不用 torch
----------------
* 42 维输入、7 类、几百个样本，一个两层 MLP 就够，装在虚拟机上纯 numpy 跑几秒；
* 依赖越少，`roslaunch` 之外的工具链越容易被别人复现（装不上 torch 的机器也能训）。

为什么"划分方式"是这里的重点
----------------------------
相邻帧几乎一样，随机划分会让训练集和验证集共享同一段视频，指标虚高。
在**时序窗口**数据上我们实测过：同一份数据按时间划分验证准确率 0.167、
随机划分 0.972 —— 差别全部来自"相邻样本几乎相同"。

注意这不代表本仓库的逐帧数据也一定差很多：手势**交替出现**时，按时间划分本来就
合理（实测 121 个样本、3 类，三种划分都是 0.972）。所以本工具默认
``--split time``，并提供 ``--compare`` 把三种划分并排打出来，让数据自己说话。

模型
----
``42 -> 64 (ReLU) -> 7 (softmax)``，交叉熵损失，mini-batch SGD + 动量。
"""
from __future__ import annotations

import os
from typing import Dict, List, Optional, Tuple

import numpy as np

import gesture_dataset as gd

#: 默认隐层宽度
HIDDEN = 64


# --------------------------------------------------------------------------- #
# 模型
# --------------------------------------------------------------------------- #
class MLP(object):
    """两层全连接网络（ReLU + softmax），纯 numpy 前向/反向。"""

    def __init__(self, weights: Optional[Dict[str, np.ndarray]] = None,
                 n_in: int = gd.FEATURE_DIM, hidden: int = HIDDEN,
                 n_out: int = gd.NUM_CLASSES, seed: int = 0):
        if weights is not None:
            missing = [k for k in ("W1", "b1", "W2", "b2") if k not in weights]
            if missing:
                raise ValueError("权重缺少：%s" % ", ".join(missing))
            self.params = {k: np.asarray(weights[k], dtype=np.float32)
                           for k in ("W1", "b1", "W2", "b2")}
            self._validate()
            return
        rng = np.random.RandomState(int(seed))
        # He 初始化
        self.params = {
            "W1": (rng.randn(n_in, hidden) * np.sqrt(2.0 / n_in)).astype(np.float32),
            "b1": np.zeros(hidden, dtype=np.float32),
            "W2": (rng.randn(hidden, n_out) * np.sqrt(2.0 / hidden)).astype(np.float32),
            "b2": np.zeros(n_out, dtype=np.float32),
        }
        self._validate()

    def _validate(self):
        W1, W2 = self.params["W1"], self.params["W2"]
        if W1.shape[0] != gd.FEATURE_DIM:
            raise ValueError("输入维度应为 %d，实际 %d" % (gd.FEATURE_DIM, W1.shape[0]))
        if W2.shape[1] != gd.NUM_CLASSES:
            raise ValueError("输出类别数应为 %d，实际 %d"
                             % (gd.NUM_CLASSES, W2.shape[1]))
        if W1.shape[1] != W2.shape[0]:
            raise ValueError("隐层维度不一致：%d vs %d" % (W1.shape[1], W2.shape[0]))

    # ------------------------------------------------------------------ 前向
    @property
    def hidden(self) -> int:
        return int(self.params["W1"].shape[1])

    def forward(self, X: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """返回 ``(隐层激活, 各类概率)``。"""
        X = np.asarray(X, dtype=np.float32)
        if X.ndim != 2 or X.shape[1] != gd.FEATURE_DIM:
            raise ValueError("输入应为 (N, %d)，收到 %s" % (gd.FEATURE_DIM, X.shape))
        h = np.maximum(0.0, X @ self.params["W1"] + self.params["b1"])     # ReLU
        logits = h @ self.params["W2"] + self.params["b2"]
        logits = logits - logits.max(axis=1, keepdims=True)
        e = np.exp(logits)
        probs = e / e.sum(axis=1, keepdims=True)
        return h, probs

    def predict_proba(self, X: np.ndarray) -> np.ndarray:
        return self.forward(X)[1]

    def predict(self, X: np.ndarray) -> np.ndarray:
        return np.argmax(self.predict_proba(X), axis=1)

    def confidence(self, X: np.ndarray) -> np.ndarray:
        return np.max(self.predict_proba(X), axis=1)

    # ------------------------------------------------------------------ 训练
    def train(self, X, y, epochs: int = 200, lr: float = 0.05, batch: int = 32,
              momentum: float = 0.9, seed: int = 0,
              log=None) -> List[Dict[str, float]]:
        """mini-batch SGD；返回每个 epoch 的 loss/acc 记录。"""
        X = np.asarray(X, dtype=np.float32)
        y = np.asarray(y, dtype=np.int64)
        if X.shape[0] != y.shape[0]:
            raise ValueError("X 与 y 长度不一致：%d vs %d" % (X.shape[0], y.shape[0]))
        if X.shape[0] == 0:
            raise ValueError("训练集为空")
        if len(np.unique(y)) < 2:
            raise ValueError("训练集只有一个类别（%d），无法训练分类器"
                             % int(np.unique(y)[0]))

        rng = np.random.RandomState(int(seed))
        velocity = {k: np.zeros_like(v) for k, v in self.params.items()}
        history = []
        N = X.shape[0]
        for epoch in range(int(epochs)):
            order = rng.permutation(N)
            total = 0.0
            for start in range(0, N, int(batch)):
                idx = order[start:start + int(batch)]
                xb, yb = X[idx], y[idx]
                h, probs = self.forward(xb)
                onehot = np.zeros_like(probs)
                onehot[np.arange(len(yb)), yb] = 1.0
                # 交叉熵 + softmax 的梯度
                d_logits = (probs - onehot) / len(yb)
                grads = {
                    "W2": h.T @ d_logits,
                    "b2": d_logits.sum(axis=0),
                }
                d_hidden = d_logits @ self.params["W2"].T
                d_hidden[h <= 0.0] = 0.0                    # ReLU 反传
                grads["W1"] = xb.T @ d_hidden
                grads["b1"] = d_hidden.sum(axis=0)

                for k, g in grads.items():
                    velocity[k] = momentum * velocity[k] - lr * g
                    self.params[k] = (self.params[k] + velocity[k]).astype(np.float32)

                total += float(-np.log(probs[np.arange(len(yb)), yb] + 1e-9).mean())
            acc = float((self.predict(X) == y).mean())
            history.append({"epoch": epoch + 1, "loss": total, "acc": acc})
            if log is not None and (epoch == 0 or (epoch + 1) % max(1, epochs // 5) == 0):
                log("  epoch %3d  loss=%.4f  训练acc=%.3f" % (epoch + 1, total, acc))
        return history

    # ------------------------------------------------------------------ 存取
    def save(self, path: str) -> str:
        if not path.endswith(".npz"):
            path += ".npz"
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        np.savez_compressed(path, **self.params,
                            classes=np.array(gd.trainable_gestures()))
        return path

    @classmethod
    def load(cls, path: str) -> "MLP":
        """读回模型；**会校验类别顺序**，避免词表变了以后静默错标。"""
        if not os.path.isfile(path):
            raise FileNotFoundError("找不到模型文件：%s" % path)
        with np.load(path, allow_pickle=False) as data:
            weights = {k: data[k] for k in ("W1", "b1", "W2", "b2") if k in data.files}
            saved_classes = None
            if "classes" in data.files:
                saved_classes = [str(c) for c in data["classes"].tolist()]
        if saved_classes is not None and saved_classes != list(gd.trainable_gestures()):
            raise ValueError(
                "模型里的类别顺序与当前词表不一致，不能直接使用。\n"
                "  模型: %s\n  词表: %s\n"
                "（词表改过之后必须重新训练，否则输出会静默错标）"
                % (saved_classes, list(gd.trainable_gestures())))
        return cls(weights=weights)


# --------------------------------------------------------------------------- #
# 划分与评估
# --------------------------------------------------------------------------- #
def split_indices(labels, val_ratio: float = 0.3, mode: str = "time",
                  seed: int = 0):
    """按 ``mode`` 返回 ``(train_idx, val_idx)``。

    * ``time``：**前 70% 训练、后 30% 验证**（尊重采集顺序，不会泄漏）；
    * ``stratified``：按类分层随机（会泄漏，仅用于对比）；
    * ``random``：整体随机（泄漏最严重）。
    """
    labs = np.asarray(labels, dtype=np.int64)
    n = len(labs)
    if n < 2:
        raise ValueError("样本数太少（%d），无法划分" % n)
    if not (0.0 < val_ratio < 1.0):
        raise ValueError("val_ratio 必须在 (0, 1) 之间，收到 %.3f" % val_ratio)

    if mode == "time":
        cut = max(1, int(round(n * (1.0 - val_ratio))))
        cut = min(cut, n - 1)
        return np.arange(cut), np.arange(cut, n)

    rng = np.random.RandomState(int(seed))
    if mode == "random":
        perm = rng.permutation(n)
        cut = max(1, int(round(n * (1.0 - val_ratio))))
        return np.sort(perm[:cut]), np.sort(perm[cut:])

    if mode == "stratified":
        tr, va = [], []
        for cls in np.unique(labs):
            idx = np.where(labs == cls)[0]
            rng.shuffle(idx)
            if len(idx) < 2:
                tr.extend(idx.tolist())
                continue
            k = min(max(1, int(round(len(idx) * val_ratio))), len(idx) - 1)
            va.extend(idx[:k].tolist())
            tr.extend(idx[k:].tolist())
        return np.sort(np.array(tr, dtype=np.int64)), np.sort(np.array(va, dtype=np.int64))

    raise ValueError("mode 只能是 time / stratified / random，收到 %r" % mode)


def confusion_matrix(y_true, y_pred, n_classes: int = gd.NUM_CLASSES) -> np.ndarray:
    cm = np.zeros((n_classes, n_classes), dtype=np.int64)
    for t, p in zip(np.asarray(y_true), np.asarray(y_pred)):
        cm[int(t), int(p)] += 1
    return cm


def evaluate(model: MLP, X, y) -> Dict:
    """返回 ``accuracy / confusion / per_class_recall / mean_confidence``。"""
    X = np.asarray(X, dtype=np.float32)
    y = np.asarray(y, dtype=np.int64)
    if len(y) == 0:
        raise ValueError("验证集为空")
    pred = model.predict(X)
    cm = confusion_matrix(y, pred)
    recalls = {}
    for i, name in enumerate(gd.trainable_gestures()):
        total = int(cm[i].sum())
        if total:
            recalls[name] = float(cm[i, i]) / total
    return {
        "accuracy": float((pred == y).mean()),
        "confusion": cm,
        "per_class_recall": recalls,
        "mean_confidence": float(model.confidence(X).mean()),
        "n": int(len(y)),
    }


def majority_baseline(labels) -> float:
    """多数类基线：全猜样本最多的那一类能拿到的准确率。

    0.972 这样的数字只有和基线并排看才有意义——类别不均衡时，
    "全猜最多的一类"本身就可能很高（本仓库验证集是 26/9/1，基线 0.722）。
    """
    labs = np.asarray(labels)
    if len(labs) == 0:
        raise ValueError("空标签无法计算基线")
    _, cnts = np.unique(labs, return_counts=True)
    return float(cnts.max()) / float(len(labs))


def format_report(result: Dict, title: str = "") -> str:
    """把 :func:`evaluate` 的结果排版成文本（便于打印/写文档）。"""
    lines = []
    if title:
        lines.append(title)
    lines.append("  准确率 %.3f  (n=%d, 平均置信度 %.2f)"
                 % (result["accuracy"], result["n"], result["mean_confidence"]))
    cm = result["confusion"]
    classes = gd.trainable_gestures()
    lines.append("            " + "".join("%9s" % c[:8] for c in classes))
    for i, name in enumerate(classes):
        lines.append("%-12s" % name[:11] + "".join("%9d" % v for v in cm[i]))
    if result["per_class_recall"]:
        lines.append("  每类召回：" + "，".join(
            "%s %.2f" % (k, v) for k, v in result["per_class_recall"].items()))
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# 默认数据定位（源码树 / devel / install 三种布局都要能找到）
# --------------------------------------------------------------------------- #
PKG_NAME = "gesture_control_drone"
SAMPLE_NAME = "session_host_121.npz"


def data_candidates(name: str = SAMPLE_NAME):
    """按可能的布局列出候选路径。

    * 源码树 / devel 启动器：``scripts/../data/x.npz``（启动器 __file__ 指向源码）
    * install 空间：脚本被复制到 ``install/lib/<pkg>/``，数据在
      ``install/share/<pkg>/data/``，所以要往上两级找 share
    * 兜底：``rospkg`` 查包目录
    """
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        os.path.join(here, os.pardir, "data", name),
        os.path.join(here, os.pardir, os.pardir, "share", PKG_NAME, "data", name),
        os.path.join(here, "data", name),
    ]
    try:
        import rospkg
        cands.append(os.path.join(rospkg.RosPack().get_path(PKG_NAME), "data", name))
    except Exception:                                  # noqa: BLE001 - 没有 ros 很正常
        pass
    return cands


def default_data_path(name: str = SAMPLE_NAME) -> str:
    """返回第一个真实存在的候选路径；都找不到时抛 FileNotFoundError 并列出试过的路径。"""
    cands = data_candidates(name)
    for c in cands:
        if os.path.isfile(c):
            return os.path.abspath(c)
    raise FileNotFoundError(
        "找不到默认样本数据 %s，试过这些位置：\n  %s\n"
        "请用 --data 指定数据集，或先用 record_gesture_session.py 采集一份。"
        % (name, "\n  ".join(os.path.abspath(c) for c in cands)))


# --------------------------------------------------------------------------- #
# 命令行入口
# --------------------------------------------------------------------------- #
def _print_comparison(rows, baseline=None) -> None:
    print("\n%-14s %10s %10s %8s" % ("划分方式", "训练acc", "验证acc", "样本数"))
    for mode, tr_acc, va_acc, n in rows:
        print("%-14s %10.3f %10.3f %8d" % (mode, tr_acc, va_acc, n))
    if baseline is not None:
        # 多数类基线：全猜样本最多的那一类能拿到的准确率。
        # 0.972 只有和它并排看才有意义（尤其类别极不均衡时）。
        print("%-14s %10s %10.3f %8s" % ("多数类基线", "-", baseline, ""))


def run_experiment(X, y, mode="time", val_ratio=0.3, epochs=200, lr=0.05,
                   batch=32, hidden=HIDDEN, seed=0, log=print):
    """训练一次并返回 ``(model, result, train_acc)``。"""
    tr, va = split_indices(y, val_ratio=val_ratio, mode=mode, seed=seed)
    log("划分方式=%s：训练 %d / 验证 %d" % (mode, len(tr), len(va)))
    model = MLP(hidden=hidden, seed=seed)
    history = model.train(X[tr], y[tr], epochs=epochs, lr=lr, batch=batch,
                          seed=seed, log=log)
    train_acc = history[-1]["acc"] if history else 0.0
    result = evaluate(model, X[va], y[va])
    return model, result, train_acc


def main(argv=None):
    import argparse

    ap = argparse.ArgumentParser(
        description="手势分类器训练与评估（纯 numpy，不需要 torch）")
    ap.add_argument("--data", default=None,
                    help="数据集 npz（默认用仓库自带的真实样本）")
    # 默认落到包内的 models/（.gitignore 里有 `!**/models/*.npz` 例外），
    # 不要用相对 cwd 的路径 —— 在仓库根跑会往根目录丢文件。
    default_out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               os.pardir, "models", "gesture_mlp.npz")
    ap.add_argument("--out", default=os.path.normpath(default_out),
                    help="模型输出路径（默认写到包内的 models/）")
    ap.add_argument("--split", choices=("time", "stratified", "random"),
                    default="time", help="训练/验证划分方式（默认按时间）")
    ap.add_argument("--val-ratio", type=float, default=0.3)
    ap.add_argument("--epochs", type=int, default=200)
    ap.add_argument("--lr", type=float, default=0.05)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--hidden", type=int, default=HIDDEN)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--compare", action="store_true",
                    help="把三种划分都跑一遍并对比")
    args = ap.parse_args(argv)

    if args.data:
        data_path = args.data
    else:
        try:
            data_path = default_data_path()
        except FileNotFoundError as exc:
            print(exc)
            return 2
    try:
        item = gd.load_session(data_path)
    except FileNotFoundError:
        print("数据集不存在：%s" % data_path)
        return 2
    X, y = item["features"], item["labels"]
    counts = {gd.index_label(int(i)): int((y == i).sum()) for i in np.unique(y)}
    print("数据集 %s：%d 个样本" % (data_path, len(y)))
    print("类别分布：%s" % counts)
    meta = item.get("meta") or {}
    if meta.get("merged_from"):
        print("提示：这个数据集是 merge_sessions 合并来的（默认会打乱顺序）。"
              "如果确实打乱过，--split time 已经不是按采集时间划分了，"
              "验证指标请以 --compare 里 random 那一行为准。")
    if len(np.unique(y)) < 2:
        print("只有一类，无法训练")
        return 2

    if args.compare:
        rows = []
        for mode in ("time", "stratified", "random"):
            _, r, tr_acc = run_experiment(X, y, mode=mode, val_ratio=args.val_ratio,
                                          epochs=args.epochs, lr=args.lr,
                                          batch=args.batch, hidden=args.hidden,
                                          seed=args.seed, log=lambda *_: None)
            rows.append((mode, tr_acc, r["accuracy"], r["n"]))
        # 多数类基线：全猜验证集里最多的那一类能拿到的准确率。
        # 0.972 只有和它并排看才有意义（尤其类别极不均衡、POINT 只有 3 个样本时）。
        _, va_of_split = split_indices(y, val_ratio=args.val_ratio,
                                       mode=args.split, seed=args.seed)
        _print_comparison(rows, majority_baseline(y[va_of_split]))
        model, result, _ = run_experiment(X, y, mode=args.split,
                                          val_ratio=args.val_ratio,
                                          epochs=args.epochs, lr=args.lr,
                                          batch=args.batch, hidden=args.hidden,
                                          seed=args.seed)
    else:
        model, result, _ = run_experiment(X, y, mode=args.split,
                                          val_ratio=args.val_ratio,
                                          epochs=args.epochs, lr=args.lr,
                                          batch=args.batch, hidden=args.hidden,
                                          seed=args.seed)
    print("\n" + format_report(result, "验证集（%s 划分）" % args.split))
    path = model.save(args.out)
    print("\n模型已保存：%s（%d 维隐层，%.1f KB）"
          % (path, model.hidden, os.path.getsize(path) / 1024.0))
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
