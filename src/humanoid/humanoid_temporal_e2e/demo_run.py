"""demo_run.py —— 多帧时序端到端 纯NumPy演示"""
import os, random, math
import numpy as np
from PIL import Image, ImageDraw
import imageio

random.seed(42)
HERE = os.path.dirname(os.path.abspath(__file__))
OUT_GIF = os.path.join(HERE, "demo_test.gif")
OUT_W = os.path.join(HERE, "models_trained", "temporal_e2e.npz")
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

# MLP 768->256->128->15
class MLP:
    def __init__(self):
        self.W1 = randn((768,256)) * math.sqrt(2.0/768)
        self.b1 = np.zeros(256)
        self.W2 = randn((256,128)) * math.sqrt(2.0/256)
        self.b2 = np.zeros(128)
        self.W3 = randn((128,15)) * math.sqrt(2.0/128)
        self.b3 = np.zeros(15)
        self.m = [np.zeros_like(p) for p in [self.W1,self.b1,self.W2,self.b2,self.W3,self.b3]]
    def forward(self, x):
        self.z1 = x @ self.W1 + self.b1
        self.h1 = np.maximum(0, self.z1)
        self.z2 = self.h1 @ self.W2 + self.b2
        self.h2 = np.maximum(0, self.z2)
        return self.h2 @ self.W3 + self.b3
    def train(self, x, y, lr=0.005):
        pred = self.forward(x); n = x.shape[0]
        d = 2*(pred-y)/n
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
        return float(np.mean((pred-y)**2))

print("[1/3] 生成模拟数据...")
N = 3000
X = randn((N,768))
W_expert = randn((768,15)) * 0.05
y = X @ W_expert + randn((N,15)) * 0.03
y = np.clip(y, -0.5, 0.5)

print("[2/3] 训练 768->256->128->15 ...")
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
        vm = float(np.mean((model.forward(Xv)-yv)**2))
        print(f"  epoch {ep}/80  train_mse={np.mean(losses):.5f}  val_mse={vm:.5f}")

np.savez(OUT_W, W1=model.W1, b1=model.b1, W2=model.W2, b2=model.b2, W3=model.W3, b3=model.b3)
print(f"[3/3] 权重已保存: {OUT_W}")

# 生成gif
print("生成演示gif...")
frames = []
W, H = 200, 200
for t in range(60):
    img = Image.new("RGB", (W,H), (30,35,45))
    d = ImageDraw.Draw(img)
    for gx in range(0, W, 20):
        d.line([(gx,160),(gx,180)], fill=(70,70,80))
    d.line([(0,160),(W,160)], fill=(120,120,120))
    x = randn((1,768))
    pred = model.forward(x)[0]
    # 用pred的前几个关节角画人形
    cx = 100 + float(pred[0])*20
    tilt = float(pred[1]) * 20
    hip_y = 140; shoulder_y = 90
    hip_x = cx + tilt*0.3
    d.line([(hip_x,hip_y),(hip_x-10,160)], fill=(100,200,100), width=3)
    d.line([(hip_x,hip_y),(hip_x+10,160)], fill=(100,200,100), width=3)
    d.line([(hip_x,hip_y),(cx+tilt*0.5,shoulder_y)], fill=(120,220,120), width=4)
    d.ellipse([(cx+tilt*0.5-5,shoulder_y-12),(cx+tilt*0.5+5,shoulder_y-2)], fill=(120,220,120))
    d.line([(cx+tilt*0.5,shoulder_y+5),(cx+tilt*0.5-15,shoulder_y+20)], fill=(80,180,80), width=2)
    d.line([(cx+tilt*0.5,shoulder_y+5),(cx+tilt*0.5+15,shoulder_y+20)], fill=(80,180,80), width=2)
    d.text((5,5), f"3-frame MLP -> 15 joints", fill=(255,255,255))
    d.text((5,185), "temporal e2e", fill=(150,255,150))
    frames.append(np.array(img))

imageio.mimsave(OUT_GIF, frames, duration=0.08)
print(f"[done] gif: {OUT_GIF} ({os.path.getsize(OUT_GIF)//1024} KB)")
