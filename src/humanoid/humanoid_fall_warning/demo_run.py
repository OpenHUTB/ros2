"""demo_run.py —— 多模态跌倒预警 纯NumPy演示"""
import os, random, math
import numpy as np
from PIL import Image, ImageDraw
import imageio

random.seed(42)
HERE = os.path.dirname(os.path.abspath(__file__))
OUT_GIF = os.path.join(HERE, "demo_test.gif")
OUT_W = os.path.join(HERE, "models_trained", "fall_warning.npz")
os.makedirs(os.path.dirname(OUT_W), exist_ok=True)

def randn(shape):
    n = 1
    for s in shape: n *= s
    out = []
    for _ in range(n):
        u1 = random.random(); u2 = random.random()
        z = math.sqrt(-2*math.log(u1+1e-12)) * math.cos(2*math.pi*u2)
        out.append(z)
    return np.array(out, dtype=np.float64).reshape(shape)

# MLP 258->128->64->2 (二分类)
class MLP:
    def __init__(self):
        self.W1 = randn((258,128)) * math.sqrt(2.0/258)
        self.b1 = np.zeros(128)
        self.W2 = randn((128,64)) * math.sqrt(2.0/128)
        self.b2 = np.zeros(64)
        self.W3 = randn((64,2)) * math.sqrt(2.0/64)
        self.b3 = np.zeros(2)
        self.m = [np.zeros_like(p) for p in [self.W1,self.b1,self.W2,self.b2,self.W3,self.b3]]
    def forward(self, x):
        self.z1 = x @ self.W1 + self.b1
        self.h1 = np.maximum(0, self.z1)
        self.z2 = self.h1 @ self.W2 + self.b2
        self.h2 = np.maximum(0, self.z2)
        logits = self.h2 @ self.W3 + self.b3
        e = np.exp(logits - logits.max(axis=-1, keepdims=True))
        return e / e.sum(axis=-1, keepdims=True)
    def train(self, x, y, lr=0.01):
        pred = self.forward(x); n = x.shape[0]
        d = pred.copy(); d[np.arange(n), y] -= 1; d /= n
        dW3 = self.h2.T @ d; db3 = d.sum(0)
        dh2 = d @ self.W3.T; dz2 = dh2*(self.z2>0)
        dW2 = self.h1.T @ dz2; db2 = dz2.sum(0)
        dh1 = dz2 @ self.W2.T; dz1 = dh1*(self.z1>0)
        dW1 = x.T @ dz1; db1 = dz1.sum(0)
        grads = [dW1,db1,dW2,db2,dW3,db3]
        ps = [self.W1,self.b1,self.W2,self.b2,self.W3,self.b3]
        for i,(p,g) in enumerate(zip(ps,grads)):
            self.m[i] = 0.9*self.m[i] + 0.1*g
            p -= lr*self.m[i]
        return float(-np.mean(np.log(pred[np.arange(n), y]+1e-8)))

print("[1/3] 生成模拟多模态数据...")
N = 3000
X_img = randn((N,256)) * 0.3
X_imu = randn((N,2)) * 0.5
# 标签：IMU姿态大→跌倒
y = np.array([1 if (abs(X_imu[i,0])>0.4 or abs(X_imu[i,1])>0.4) else 0 for i in range(N)], dtype=np.int64)
X = np.concatenate([X_img, X_imu], axis=1)

print("[2/3] 训练 258->128->64->2 ...")
idx = list(range(N)); random.shuffle(idx)
X = X[idx]; y = y[idx]
Xv, yv = X[:600], y[:600]
Xt, yt = X[600:], y[600:]
model = MLP()
for ep in range(80):
    order = list(range(len(yt))); random.shuffle(order)
    Xt = Xt[order]; yt = yt[order]
    losses = []
    for i in range(0, len(yt), 64):
        losses.append(model.train(Xt[i:i+64], yt[i:i+64]))
    if ep % 20 == 0:
        vp = model.forward(Xv).argmax(axis=1)
        acc = float((vp==yv).mean())
        print(f"  epoch {ep}/80  loss={np.mean(losses):.4f}  val_acc={acc:.3f}")

np.savez(OUT_W, W1=model.W1, b1=model.b1, W2=model.W2, b2=model.b2, W3=model.W3, b3=model.b3)
print(f"[3/3] 权重已保存: {OUT_W}")

# 生成gif
print("生成演示gif...")
frames = []
W, H = 200, 200
for t in range(60):
    img = Image.new("RGB", (W,H), (45,30,30))
    d = ImageDraw.Draw(img)
    for gx in range(0, W, 20):
        d.line([(gx,160),(gx,180)], fill=(80,70,70))
    d.line([(0,160),(W,160)], fill=(120,120,120))
    # 模拟IMU姿态
    pitch = 0.4*math.sin(t*0.1)
    roll = 0.3*math.cos(t*0.13)
    x_img = randn((1,256))*0.3
    x_imu = np.array([[pitch, roll]])
    x = np.concatenate([x_img, x_imu], axis=1)
    probs = model.forward(x)[0]
    p_normal, p_fall = float(probs[0]), float(probs[1])
    # 画人形
    cx = 100 + pitch*40
    hip_y = 140; shoulder_y = 90
    color = (200,80,80) if p_fall > 0.6 else (200,200,80)
    d.line([(cx,hip_y),(cx-10,160)], fill=color, width=3)
    d.line([(cx,hip_y),(cx+10,160)], fill=color, width=3)
    d.line([(cx,hip_y),(cx+roll*20,shoulder_y)], fill=color, width=4)
    d.ellipse([(cx+roll*20-5,shoulder_y-12),(cx+roll*20+5,shoulder_y-2)], fill=color)
    d.text((5,5), f"p_normal={p_normal:.2f} p_fall={p_fall:.2f}", fill=(255,255,255))
    if p_fall > 0.6:
        d.text((5,20), "!!! FALL WARNING -> SQUAT !!!", fill=(255,80,80))
    d.text((5,185), "RGB+IMU -> binary warning", fill=(255,180,150))
    frames.append(np.array(img))

imageio.mimsave(OUT_GIF, frames, duration=0.08)
print(f"[done] gif: {OUT_GIF} ({os.path.getsize(OUT_GIF)//1024} KB)")
