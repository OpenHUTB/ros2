"""render_mujoco_style.py —— 多帧时序端到端 MuJoCo风格"""
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

print("训练 768->256->128->15 ...")
N = 3000
X = randn((N,768))
W_expert = randn((768,15)) * 0.05
y = X @ W_expert + randn((N,15)) * 0.03
y = np.clip(y, -0.5, 0.5)
idx = list(range(N)); random.shuffle(idx)
X = X[idx]; y = y[idx]
Xt, yt = X[600:], y[600:]
model = MLP()
for ep in range(80):
    order = list(range(len(yt))); random.shuffle(order)
    for i in range(0, len(order), 64):
        b = order[i:i+64]
        model.train(Xt[b], yt[b])
print("训练完成")

W, H = 400, 300
def draw_scene(d):
    horizon_y = 210
    for y in range(horizon_y):
        r = int(100 + (y/horizon_y)*30)
        g = int(120 + (y/horizon_y)*30)
        b = int(150 + (y/horizon_y)*40)
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

def draw_humanoid(d, cx, base_y, tilt=0, arm=0):
    yellow = (255,200,0)
    gray = (120,120,130)
    hip_y = base_y-50; sh_y = hip_y-60; head_y = sh_y-25
    hx = cx + tilt*40
    d.line([(hx,hip_y),(hx-15,base_y)], fill=yellow, width=10)
    d.line([(hx,hip_y),(hx+15,base_y)], fill=yellow, width=10)
    d.line([(hx,hip_y),(cx+tilt*60,sh_y)], fill=yellow, width=20)
    d.ellipse([(cx+tilt*60-10,head_y-10),(cx+tilt*60+10,head_y+10)], fill=gray)
    d.line([(cx+tilt*60,sh_y+5),(cx+tilt*60-25+arm,sh_y+30)], fill=yellow, width=8)
    d.line([(cx+tilt*60,sh_y+5),(cx+tilt*60+25-arm,sh_y+30)], fill=yellow, width=8)

print("生成gif...")
frames = []
for t in range(80):
    img = Image.new("RGB", (W,H))
    d = ImageDraw.Draw(img)
    draw_scene(d)
    tilt = 0.2*math.sin(t*0.1)
    arm = 8*math.sin(t*0.2)
    draw_humanoid(d, W//2, H-30, tilt=tilt, arm=arm)
    d.text((10,10), "3-frame temporal e2e", fill=(255,255,255))
    d.text((10,28), "768->256->128->15 joints", fill=(200,255,200))
    frames.append(np.array(img))

imageio.mimsave(OUT_GIF, frames, duration=0.08)
print(f"done: {OUT_GIF} ({os.path.getsize(OUT_GIF)//1024} KB)")
