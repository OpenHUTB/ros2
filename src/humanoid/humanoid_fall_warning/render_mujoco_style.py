"""render_mujoco_style.py —— 多模态跌倒预警 MuJoCo风格"""
import os, random, math
import numpy as np
from PIL import Image, ImageDraw
import imageio

random.seed(42)
HERE = os.path.dirname(os.path.abspath(__file__))
OUT_GIF = os.path.join(HERE, "demo_test.gif")

def randn(shape):
    n = 1
    for s in shape: n *= s
    out = []
    for _ in range(n):
        u1 = random.random(); u2 = random.random()
        z = math.sqrt(-2*math.log(u1+1e-12)) * math.cos(2*math.pi*u2)
        out.append(z)
    return np.array(out, dtype=np.float64).reshape(shape)

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

print("训练 258->128->64->2 ...")
N = 3000
X_img = randn((N,256)) * 0.3
X_imu = randn((N,2)) * 0.5
y = np.array([1 if (abs(X_imu[i,0])>0.4 or abs(X_imu[i,1])>0.4) else 0 for i in range(N)], dtype=np.int64)
X = np.concatenate([X_img, X_imu], axis=1)
idx = list(range(N)); random.shuffle(idx)
X = X[idx]; y = y[idx]
model = MLP()
for ep in range(80):
    order = list(range(600,N)); random.shuffle(order)
    for i in range(0, len(order), 64):
        b = order[i:i+64]
        model.train(X[b], y[b])
print("训练完成")

W, H = 400, 300
def draw_scene(d):
    horizon_y = 210
    for y in range(horizon_y):
        r = int(90 + (y/horizon_y)*20)
        g = int(100 + (y/horizon_y)*20)
        b = int(130 + (y/horizon_y)*30)
        d.line([(0,y),(W,y)], fill=(r,g,b))
    for row in range(12):
        y0 = horizon_y + int((row/12)**1.5 * (H-horizon_y))
        y1 = horizon_y + int(((row+1)/12)**1.5 * (H-horizon_y))
        ncols = 4 + row*2
        half_w = W/2
        for col in range(ncols):
            f0 = col/ncols; f1 = (col+1)/ncols
            color = (60,60,70) if (row+col)%2==0 else (180,180,190)
            xl = int(W/2-half_w*f1); xr = int(W/2-half_w*f0)
            xlr = int(W/2+half_w*f0); xrr = int(W/2+half_w*f1)
            d.rectangle([xl,y0,xr,y1], fill=color)
            d.rectangle([xlr,y0,xrr,y1], fill=color)

def draw_humanoid(d, cx, base_y, tilt=0, warning=False):
    yellow = (255,200,0) if not warning else (255,100,80)
    gray = (120,120,130)
    hip_y = base_y-50; sh_y = hip_y-60; head_y = sh_y-25
    hx = cx + tilt*40
    d.line([(hx,hip_y),(hx-15,base_y)], fill=yellow, width=10)
    d.line([(hx,hip_y),(hx+15,base_y)], fill=yellow, width=10)
    d.line([(hx,hip_y),(cx+tilt*60,sh_y)], fill=yellow, width=20)
    d.ellipse([(cx+tilt*60-10,head_y-10),(cx+tilt*60+10,head_y+10)], fill=gray)
    d.line([(cx+tilt*60,sh_y+5),(cx+tilt*60-25,sh_y+30)], fill=yellow, width=8)
    d.line([(cx+tilt*60,sh_y+5),(cx+tilt*60+25,sh_y+30)], fill=yellow, width=8)

print("生成gif...")
frames = []
for t in range(80):
    img = Image.new("RGB", (W,H))
    d = ImageDraw.Draw(img)
    draw_scene(d)
    pitch = 0.5*math.sin(t*0.08)
    x = np.concatenate([randn((1,256))*0.3, [[pitch, 0.1]]], axis=1)
    probs = model.forward(x)[0]
    p_fall = float(probs[1])
    warning = p_fall > 0.5
    draw_humanoid(d, W//2, H-30, tilt=pitch, warning=warning)
    d.text((10,10), "RGB+IMU -> fall warning", fill=(255,255,255))
    d.text((10,28), f"p_normal={probs[0]:.2f} p_fall={p_fall:.2f}", fill=(255,255,200))
    if warning:
        d.text((10,46), "!!! FALL WARNING -> SQUAT !!!", fill=(255,80,80))
    frames.append(np.array(img))

imageio.mimsave(OUT_GIF, frames, duration=0.08)
print(f"done: {OUT_GIF} ({os.path.getsize(OUT_GIF)//1024} KB)")
