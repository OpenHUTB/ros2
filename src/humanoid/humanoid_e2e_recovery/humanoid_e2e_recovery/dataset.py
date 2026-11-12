"""
dataset.py —— 端到端数据集采集
================================
用专家控制器（expert_action）在仿真里站住 / 抗推，逐步记录
前视灰度图像与专家输出的 15 维力矩增量。

端到端网络学习：图像 → 力矩增量。
"""
from __future__ import annotations
import os
import numpy as np
from .sim import E2ESim, DEFAULT_STAND_TARGET
from .expert import expert_action

# 输入逐维标准化的下限。取值依据见 collect() 里的注释：
# 过小的下限会把恒定像素的浮点噪声放大成主要输入，实测 R² 从 0.98 掉到 0.30。
X_STD_FLOOR = 0.02

# 执行器真实力矩量程（models/humanoid_camera.xml 中 motor 的 ctrlrange ±300）。
TORQUE_LIMIT = 300.0


def _img_to_vec(img, size=16):
    h, w = img.shape
    bh, bw = h // size, w // size
    c = img[:bh * size, :bw * size]
    return c.reshape(size, bh, size, bw).mean(axis=(1, 3)).flatten().astype(np.float32)


def collect(model_path, out_path, n_episodes=60, force_range=(40, 100),
            kp=300.0, kd=30.0, warmup=1200, horizon=400, record_every=3,
            seed=0, verbose=True, record_warmup=True):
    """采集端到端数据集。

    warmup      : 每回合开始先空站多少步（默认 300 步 = 0.6 s）
    horizon     : 施加推力后继续记录多少步（默认 3000 步 = 6 s）
    record_every: 每隔多少仿真步记录一次图像（渲染是瓶颈，3 步 ≈ 167 Hz）
    force_range : 推力幅值区间，过大时专家也会倒，仅保留未倒样本
    """
    rng = np.random.default_rng(seed)
    sim = E2ESim(model_path, kp=kp, kd=kd)
    X, Y = [], []
    ep_kept = 0
    for ep in range(n_episodes):
        sim.reset()
        for k in range(warmup):
            u0 = expert_action(sim)
            # 稳态样本必须记进训练集：闭环推理的起点就是"站稳、力矩≈0"这一档。
            # 早先只记录推力之后的恢复段，网络从没见过稳态输入，闭环第一步便外推
            # 出 219 N·m（专家此时是 0 N·m），0.79 s 就把机器人推翻。
            if record_warmup and k % record_every == 0:
                X.append(_img_to_vec(sim.render_front(gray=True)))
                Y.append(np.clip(u0, -TORQUE_LIMIT, TORQUE_LIMIT).astype(np.float32))
            sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u0)
            if sim.fallen():
                break
        # 随机方向 + 幅值的推力，随后由专家维持
        ang = rng.choice([0, 45, 90, 135, 180, 225, 270, 315])
        f = rng.uniform(*force_range)
        sim.apply_push((f * np.cos(np.radians(ang)), f * np.sin(np.radians(ang)), 0),
                       steps=40, target=DEFAULT_STAND_TARGET, balance=expert_action)
        ok = True
        for k in range(horizon):
            u = expert_action(sim)
            if k % record_every == 0:
                X.append(_img_to_vec(sim.render_front(gray=True)))
                # 标签必须落在执行器真实量程内。motor 的 ctrlrange 是 ±300 N·m，
                # 专家在极端状态下会算出 ±3000 以上的数，MuJoCo 虽然会截断，
                # 但若把未截断的值当回归目标，网络就会去拟合物理上不存在的
                # 力矩（实测踝 pitch 标签冲到 -3644~687，std 430），闭环必倒。
                Y.append(np.clip(u, -TORQUE_LIMIT, TORQUE_LIMIT).astype(np.float32))
            sim.step(target=DEFAULT_STAND_TARGET, torque_extra=u)
            if sim.fallen():
                ok = False
                break
        ep_kept += 1 if ok else 0
        if verbose and (ep % 10 == 0 or ep == n_episodes - 1):
            print(f"  [collect] episode {ep + 1}/{n_episodes} "
                  f"samples={len(X)} 成功站住的回合={ep_kept}")
    X = np.stack(X)
    Y = np.stack(Y)
    # —— 标准化参数：X 与 Y 都必须存下来，推理端要用同一套变换 ——
    # X（16×16 池化灰度）动态范围很小（实测 0.37~0.56），不零中心化时
    # 输入与权重的内积会系统性偏负，把第一层 ReLU 整片压制到 0（死网络）。
    #
    # floor 取 0.02 而不是 1e-3：256 个池化格里约 50 个在所有样本上几乎恒定
    # （背景像素），若用 1e-3 兜底，归一化会把这些格的浮点噪声放大上千倍，
    # 变成纯噪声输入。实测同一批 1200 条数据上，floor=1e-3 时验证 R² 只有
    # 0.30，floor=0.02 时升到 0.98，踝 pitch 的预测 std 也从 0 恢复到与目标
    # 一致的 85 N·m。
    X_mean = X.mean(axis=0)
    X_std = np.maximum(X.std(axis=0), X_STD_FLOOR)
    # Y 同理：本模块 15 维里有 7 维在所有样本上恒定（专家始终输出同一个值），
    # 这些维度的尺度必须取 1.0；若取 1e-6，目标值会爆炸到 1e5 量级，
    # 训练完全无法收敛（旧权重就是这么坏掉的）。
    Y_mean = Y.mean(axis=0)
    y_std = Y.std(axis=0)
    y_scale = np.where(y_std > 1e-6, y_std, 1.0)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    np.savez(out_path, X=X, Y=Y, X_mean=X_mean, X_std=X_std,
             Y_mean=Y_mean, Y_scale=y_scale)
    sim.close()
    if verbose:
        print(f"[collect] 数据集: {out_path} | X={X.shape} Y={Y.shape} "
              f"| 站住回合 {ep_kept}/{n_episodes}")
        print(f"[collect] X_mean={X_mean.mean():.4f} X_std(mean)={X_std.mean():.4f} "
              f"| Y 中恒定维度 {int((y_std <= 1e-6).sum())} 个")
    return out_path
