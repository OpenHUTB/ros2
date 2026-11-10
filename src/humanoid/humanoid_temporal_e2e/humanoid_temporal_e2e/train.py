"""train.py —— 训练 + 评估时序端到端网络

原实现只打印 train/val MSE，无法回答“模型是不是只是复读训练集均值”。
这里补三件事：

1. **常数基线对照**：把“永远输出训练集均值姿态”当作 baseline，网络必须在
   物理量纲 MSE / 逐维 MAE 上明显优于它。
2. **逐维 MAE**：15 维量纲差异大（踝 ±0.2 rad、肩肘 -1.4 rad），只看总 MSE
   会掩盖“真正该动的踝/膝没学会”。
3. **闭环评估**：把网络输出直接喂给 PD 控制器，看能不能站住 —— 这才是
   端到端控制任务的验收口径。
"""
from __future__ import annotations

import os
import numpy as np
from .nn import TemporalMLP


def _mae(a, b):
    return np.abs(a - b).mean(axis=0)


def train(dataset_path, out_path, input_dim=768, hidden1=256, hidden2=128,
          output_dim=15, epochs=80, batch_size=64, lr=1e-3, seed=0, verbose=True):
    data = np.load(dataset_path)
    X, y = data["X"].astype(np.float64), data["y"].astype(np.float64)
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(y))
    X, y = X[idx], y[idx]
    n_val = max(1, int(len(y) * 0.2))
    X_val, y_val = X[:n_val], y[:n_val]
    X_tr, y_tr = X[n_val:], y[n_val:]

    model = TemporalMLP(input_dim, hidden1, hidden2, output_dim, seed=seed)
    model.fit_normalizer(X_tr, y_tr)

    # 常数基线：永远输出训练集均值姿态
    const_pred = np.tile(y_tr.mean(axis=0), (len(y_val), 1))
    const_mse = float(np.mean((const_pred - y_val) ** 2))
    const_mae = float(np.abs(const_pred - y_val).mean())

    for ep in range(epochs):
        perm = rng.permutation(len(y_tr))
        X_tr, y_tr = X_tr[perm], y_tr[perm]
        losses = []
        for i in range(0, len(y_tr), batch_size):
            losses.append(model.train_step(X_tr[i:i + batch_size],
                                           y_tr[i:i + batch_size], lr=lr))
        if verbose and (ep % 10 == 0 or ep == epochs - 1):
            pred = model.predict(X_val)
            val_mse = float(np.mean((pred - y_val) ** 2))
            print(f"  epoch {ep:3d}/{epochs} | train_mse(norm)={np.mean(losses):.5f}"
                  f" | val_mse(rad^2)={val_mse:.5f}")

    pred = model.predict(X_val)
    val_mse = float(np.mean((pred - y_val) ** 2))
    val_mae = float(np.abs(pred - y_val).mean())
    dim_mae = _mae(pred, y_val)

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    model.save(out_path)

    print(f"[train] saved {out_path}")
    print(f"  样本 {len(y)}（train {len(y_tr)} / val {len(y_val)}），epochs={epochs}")
    print(f"  val_mse(rad^2) 网络 {val_mse:.5f} | 常数基线 {const_mse:.5f}"
          f" | 降幅 {(1 - val_mse / max(const_mse, 1e-9)) * 100:.1f}%")
    print(f"  val_mae(rad)   网络 {val_mae:.5f} | 常数基线 {const_mae:.5f}")
    print("  逐维 MAE(rad): " + " ".join(
        f"{n}={v:.4f}" for n, v in zip(
            ["hipR_l", "hipP_l", "knee_l", "ank_l", "ankR_l",
             "hipR_r", "hipP_r", "knee_r", "ank_r", "ankR_r",
             "sh_l", "el_l", "sh_r", "el_r", "neck"], dim_mae)))
    print(f"  真正会动的维度(标签 std>1e-3): "
          f"{int((y.std(axis=0) > 1e-3).sum())}/15")
    return out_path


def closed_loop_eval(weights_path, model_path, episodes=3, seed=0,
                     force=(20.0, 70.0), settle=300, horizon=600, verbose=True):
    """闭环评估：网络输出直接驱动 PD，看能否站住。

    对照组：
      - teacher：sim 内置踝策略（上界参考）
      - const   ：永远输出训练集均值姿态（“复读站姿”的退化模型）
      - net     ：训练好的网络

    返回 dict，含每组 max_pitch / 是否跌倒 / 跌倒步数。
    """
    from .sim import TemporalE2ESim

    data = np.load(weights_path)
    y_mean = data["y_mean"]
    model = TemporalMLP(input_dim=768).load(weights_path)

    rng = np.random.default_rng(seed)
    plan = []
    for ep in range(episodes):
        f = float(rng.uniform(*force))
        ang = float(rng.uniform(0, 360))
        plan.append((np.array([f * np.cos(np.radians(ang)),
                               f * np.sin(np.radians(ang)), 0.0]), ep))

    def run(policy):
        res = []
        for fvec, _ in plan:
            sim = TemporalE2ESim(model_path)
            sim.reset()
            sim.settle(settle)          # 统一用老师站定到同一初始状态

            buf = []
            fa, mx, acc, n_acc = None, 0.0, 0.0, 0

            def act():
                from .dataset import feat_from_gray
                buf.append(feat_from_gray(sim.render_front()))
                if len(buf) < 3:
                    return np.array(y_mean)
                if policy == "const":
                    return y_mean.copy()
                if policy == "net1":
                    # 消融：把最近 1 帧重复 3 次 → 与单帧端到端等价
                    return model.predict(np.tile(buf[-1], 3).reshape(1, -1))[0]
                return model.predict(np.concatenate(buf[-3:]).reshape(1, -1))[0]

            for i in range(40 + horizon):
                if i < 40:
                    sim.set_push(fvec)
                else:
                    sim.clear_push()

                if policy == "teacher":
                    sim.step(balance=True)
                else:
                    sim.step(target=act(), balance=False)

                if i >= 40:
                    p = abs(np.degrees(sim.torso_pose()[0]))
                    mx = max(mx, p)
                    acc += p
                    n_acc += 1
                if sim.fallen():
                    fa = i + 1
                    break
            res.append((float(mx), fa, float(sim.root_z), acc / max(n_acc, 1)))
            sim.close()
        good = sum(1 for r in res if r[1] is None)
        if verbose:
            print(f"  [{policy:7s}] 站住 {good}/{len(res)} | "
                  f"max_pitch {np.mean([r[0] for r in res]):5.1f}° | "
                  f"平均倾角 {np.mean([r[3] for r in res]):5.1f}° | "
                  f"root_z {np.mean([r[2] for r in res]):.3f}")
        return {"stayed": good, "n": len(res),
                "max_pitch": float(np.mean([r[0] for r in res])),
                "mean_tilt": float(np.mean([r[3] for r in res]))}

    print(f"[closed-loop] {episodes} 次随机推力 {force[0]:.0f}~{force[1]:.0f} N，"
          f"站定 {settle} 步，评估 {horizon} 步")
    out = {"teacher": run("teacher"), "const": run("const"),
           "net": run("net"), "net1": run("net1")}
    return out
