"""train.py —— 训练多模态跌倒预警网络并给出验证集评估

评估口径（对应审阅意见"给出验证集准确率、召回率、混淆矩阵和基率对比"）：
    - 混淆矩阵 2×2（TN/FP/FN/TP）
    - accuracy / precision / recall / F1 / specificity
    - 基率（"永远预测多数类"的退化基线）作为对照
    - 两个工作点：0.5（默认判定）与 0.6（本模块 demo 触发的预警阈值）
"""
from __future__ import annotations
import copy
import os
import numpy as np
from .nn import FallWarningMLP


def _metrics(y_true, y_pred):
    y_true = np.asarray(y_true).astype(int)
    y_pred = np.asarray(y_pred).astype(int)
    tn = int(((y_true == 0) & (y_pred == 0)).sum())
    fp = int(((y_true == 0) & (y_pred == 1)).sum())
    fn = int(((y_true == 1) & (y_pred == 0)).sum())
    tp = int(((y_true == 1) & (y_pred == 1)).sum())
    n = tn + fp + fn + tp
    acc = (tp + tn) / n if n else 0.0
    prec = tp / (tp + fp) if (tp + fp) else 0.0
    rec = tp / (tp + fn) if (tp + fn) else 0.0
    spec = tn / (tn + fp) if (tn + fp) else 0.0
    f1 = 2 * prec * rec / (prec + rec) if (prec + rec) else 0.0
    return dict(tn=tn, fp=fp, fn=fn, tp=tp, acc=acc, precision=prec,
                recall=rec, specificity=spec, f1=f1)


def evaluate(model, X, y, threshold=0.6):
    """在给定数据集上评估，返回指标字典（含混淆矩阵）。"""
    prob = model.predict(X)[:, 1]
    pred = (prob >= threshold).astype(int)
    m = _metrics(y, pred)
    m["threshold"] = threshold
    m["pos_rate"] = float(np.mean(y))
    # 基率：永远预测多数类
    majority = 1 if np.mean(y) >= 0.5 else 0
    base = _metrics(y, np.full_like(np.asarray(y), majority))
    m["baseline"] = dict(acc=base["acc"], precision=base["precision"],
                         recall=base["recall"], f1=base["f1"],
                         label=majority)
    return m


def format_report(m, title="验证集"):
    base = m["baseline"]
    return (
        f"{title}（阈值 {m['threshold']:.2f}，正例率 {m['pos_rate']:.3f}）\n"
        f"  混淆矩阵     预测正常  预测跌倒\n"
        f"  实际正常 TN={m['tn']:>6}  FP={m['fp']:>6}\n"
        f"  实际跌倒 FN={m['fn']:>6}  TP={m['tp']:>6}\n"
        f"  accuracy={m['acc']:.3f}  precision={m['precision']:.3f}  "
        f"recall={m['recall']:.3f}  F1={m['f1']:.3f}  specificity={m['specificity']:.3f}\n"
        f"  基率对照（永远预测{'跌倒' if base['label'] == 1 else '正常'}）："
        f"accuracy={base['acc']:.3f}  recall={base['recall']:.3f}  F1={base['f1']:.3f}"
    )


def train(dataset_path, out_path, input_dim=258, hidden1=128, hidden2=64,
          output_dim=2, epochs=80, batch_size=64, lr=0.001, seed=0,
          eval_every=10, l2=1e-4, verbose=True):
    data = np.load(dataset_path)
    X, y = data["X"].astype(np.float64), data["y"].astype(np.int64)
    if X.shape[1] != input_dim:
        raise ValueError(f"数据集维度 {X.shape[1]} 与网络输入 {input_dim} 不一致")
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(y))
    X, y = X[idx], y[idx]
    n_val = max(1, int(len(y)*0.2))
    X_val, y_val = X[:n_val], y[:n_val]
    X_tr, y_tr = X[n_val:], y[n_val:]
    model = FallWarningMLP(input_dim, hidden1, hidden2, output_dim, seed=seed)
    model.fit_normalizer(X_tr)

    best_f1, best_params, best_ep = -1.0, None, -1
    for ep in range(epochs):
        perm = rng.permutation(len(y_tr))
        X_tr, y_tr = X_tr[perm], y_tr[perm]
        losses = []
        for i in range(0, len(y_tr), batch_size):
            losses.append(model.train_step(X_tr[i:i+batch_size], y_tr[i:i+batch_size], lr=lr, l2=l2))
        if ep % eval_every == 0 or ep == epochs - 1:
            m = evaluate(model, X_val, y_val, threshold=0.5)
            if m["f1"] >= best_f1:
                best_f1, best_ep = m["f1"], ep
                best_params = copy.deepcopy(model._params())
            if verbose:
                print(f"  epoch {ep:>3}/{epochs} | loss={np.mean(losses):.4f} | "
                      f"val_acc={m['acc']:.3f} | val_recall={m['recall']:.3f} | "
                      f"val_f1={m['f1']:.3f}")
    if best_params is not None:
        for p, bp in zip(model._params(), best_params):
            p[...] = bp
        if verbose:
            print(f"  [train] 回存 best 参数：epoch {best_ep}/{epochs}，val_F1={best_f1:.3f}")

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    model.save(out_path)
    print(f"[train] saved {out_path}")

    if verbose:
        print(format_report(evaluate(model, X_val, y_val, threshold=0.5), "验证集 @ 阈值 0.5"))
        print(format_report(evaluate(model, X_val, y_val, threshold=0.6), "验证集 @ 阈值 0.6（demo 触发阈值）"))
        # 消融：把一路模态替换为训练集均值（标准化后为 0，等价于该模态不提供信息），
        # 用来回答"多模态到底有没有比单一模态强"。
        X_img_drop = X_val.copy()
        X_img_drop[:, :256] = model.x_mean[:256]
        m_img_drop = evaluate(model, X_img_drop, y_val, threshold=0.6)
        X_imu_drop = X_val.copy()
        X_imu_drop[:, 256:] = model.x_mean[256:]
        m_imu_drop = evaluate(model, X_imu_drop, y_val, threshold=0.6)
        print(f"[ablation] 仅 IMU（图像替换为均值）: acc={m_img_drop['acc']:.3f}  "
              f"recall={m_img_drop['recall']:.3f}  F1={m_img_drop['f1']:.3f}")
        print(f"[ablation] 仅图像（IMU 替换为均值）: acc={m_imu_drop['acc']:.3f}  "
              f"recall={m_imu_drop['recall']:.3f}  F1={m_imu_drop['f1']:.3f}")
    return out_path
