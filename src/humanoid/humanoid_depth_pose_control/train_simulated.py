"""train_simulated.py —— 用纯NumPy模拟数据训练（无需MuJoCo，Windows开发用）
在Ubuntu上请用 main.py --collect --train 跑真实MuJoCo数据。
"""
import os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from humanoid_depth_pose_control.nn import DepthPoseMLP

OUT = os.path.join(HERE, "models_trained", "depth_pose_mlp.npz")
os.makedirs(os.path.dirname(OUT), exist_ok=True)

# 生成模拟数据：深度图特征(256维) → 3D姿态角(3维)
# 用一个"隐藏专家"网络生成数据，再训练我们的网络拟合
rng = np.random.default_rng(42)
n = 3000
X = rng.normal(0, 1, (n, 256)).astype(np.float32)
# 隐藏专家：随机线性映射 + 非线性
W_expert = rng.normal(0, 0.1, (256, 3))
y = X @ W_expert + rng.normal(0, 0.05, (n, 3))
y = np.clip(y, -0.5, 0.5).astype(np.float32)

# 训练
idx = rng.permutation(n)
X, y = X[idx], y[idx]
n_val = 600
X_val, y_val = X[:n_val], y[:n_val]
X_tr, y_tr = X[n_val:], y[n_val:]

model = DepthPoseMLP(256, 128, 64, 3, seed=0)
for ep in range(80):
    perm = rng.permutation(len(y_tr))
    X_tr, y_tr = X_tr[perm], y_tr[perm]
    for i in range(0, len(y_tr), 64):
        model.train_step(X_tr[i:i+64], y_tr[i:i+64], lr=0.01)
    if ep % 20 == 0:
        val_pred = model.predict(X_val)
        val_mse = np.mean((val_pred - y_val)**2)
        print(f"  epoch {ep}/80 | val_mse={val_mse:.6f}")

model.save(OUT)
val_pred = model.predict(X_val)
final_mse = np.mean((val_pred - y_val)**2)
print(f"[done] saved {OUT}, final val_mse={final_mse:.6f}")
