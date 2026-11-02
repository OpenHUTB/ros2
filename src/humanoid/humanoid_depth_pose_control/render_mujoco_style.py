"""render_mujoco_style.py —— 用PIL模拟MuJoCo渲染风格的人形机器人"""
import os, random, math
import numpy as np
from PIL import Image, ImageDraw, ImageFilter
import imageio

random.seed(42)

def randn(shape):
    n = 1
    for s in shape: n *= s
    out = []
    for _ in range(n):
        u1 = random.random(); u2 = random.random()
        z = math.sqrt(-2*math.log(u1+1e-12)) * math.cos(2*math.pi*u2)
        out.append(z)
    return np.array(out, dtype=np.float64).reshape(shape)

# ========== MLP 256->128->64->3 (depth pose) ==========
class MLP:
    def __init__(self):
        self.W1 = randn((256,128)) * math.sqrt(2.0/256)
        self.b1 = np.zeros(128)
        self.W2 = randn((128,64)) * math.sqrt(2.0/128)
        self.b2 = np.zeros(64)
        self.W3 = randn((64,3)) * math.sqrt(2.0/64)
        self.b3 = np.zeros(3)
        self.m = [np.zeros_like(p) for p in [self.W1,self.b1,self.W2,self.b2,self.W3,self.b3]]
    def forward(self, x):
        self.z1 = x @ self.W1 + self.b1
        self.h1 = np.maximum(0, self.z1)
        self.z2 = self.h1 @ self.W2 + self.b2
        self.h2 = np.maximum(0, self.z2)
        return self.h2 @ self.W3 + self.b3
    def train(self, x, y, lr=0.01):
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

print("训练网络...")
N = 3000
X = randn((N,256))
W_expert = randn((256,3)) * 0.1
y = X @ W_expert + randn((N,3)) * 0.05
y = np.clip(y, -0.5, 0.5)
idx = list(range(N)); random.shuffle(idx)
X = X[idx]; y = y[idx]
Xt, yt = X[600:], y[600:]
model = MLP()
for ep in range(80):
    order = list(range(len(yt))); random.shuffle(order)
    Xt = Xt[order]; yt = yt[order]
    for i in range(0, len(yt), 64):
        model.train(Xt[i:i+64], yt[i:i+64])
print("训练完成")

# ========== MuJoCo风格渲染 ==========
W, H = 400, 300

def draw_chessboard(d, horizon_y=210):
    """画黑白棋盘格地面（透视）"""
    # 天空渐变
    for y in range(horizon_y):
        r = int(100 + (y/horizon_y)*30)
        g = int(120 + (y/horizon_y)*30)
        b = int(150 + (y/horizon_y)*40)
        d.line([(0,y),(W,y)], fill=(r,g,b))
    # 地面棋盘格（简单透视，左右对称）
    for row in range(12):
        y0 = horizon_y + int((row/12)**1.5 * (H-horizon_y))
        y1 = horizon_y + int(((row+1)/12)**1.5 * (H-horizon_y))
        ncols = 4 + row*2
        half_w = W/2
        for col in range(ncols):
            f0 = col/ncols
            f1 = (col+1)/ncols
            x_left = int(W/2 - half_w*f1)
            x_right = int(W/2 - half_w*f0)
            x_left_r = int(W/2 + half_w*f0)
            x_right_r = int(W/2 + half_w*f1)
            color = (60,60,70) if (row+col) % 2 == 0 else (180,180,190)
            d.rectangle([x_left,y0,x_right,y1], fill=color)
            d.rectangle([x_left_r,y0,x_right_r,y1], fill=color)

def draw_humanoid(d, cx, base_y, pitch=0, roll=0, arm_swing=0):
    """画MuJoCo风格黄色人形机器人"""
    yellow = (255, 200, 0)
    yellow_dark = (220, 170, 0)
    gray = (120, 120, 130)
    # 身体随pitch倾斜
    tilt_x = pitch * 60
    hip_y = base_y - 50
    shoulder_y = hip_y - 60
    head_y = shoulder_y - 25
    # 腿（黄色圆柱）
    hip_x = cx + tilt_x*0.3
    # 左腿
    d.line([(hip_x, hip_y), (hip_x-15, base_y)], fill=yellow, width=10)
    # 右腿
    d.line([(hip_x, hip_y), (hip_x+15, base_y)], fill=yellow, width=10)
    # 躯干（黄色长方体）
    sh_x = cx + tilt_x
    d.line([(hip_x, hip_y), (sh_x, shoulder_y)], fill=yellow, width=20)
    # 头（灰色圆）
    d.ellipse([(sh_x-10, head_y-10), (sh_x+10, head_y+10)], fill=gray)
    # 左臂
    d.line([(sh_x, shoulder_y+5), (sh_x-25+arm_swing, shoulder_y+30)], fill=yellow, width=8)
    # 右臂
    d.line([(sh_x, shoulder_y+5), (sh_x+25-arm_swing, shoulder_y+30)], fill=yellow, width=8)

print("生成gif...")
frames = []
for t in range(80):
    img = Image.new("RGB", (W,H))
    d = ImageDraw.Draw(img)
    draw_chessboard(d)
    # 模拟姿态变化
    pitch = 0.3*math.sin(t*0.08)
    roll = 0.15*math.cos(t*0.1)
    arm_swing = 5*math.sin(t*0.15)
    # 用网络预测
    x = randn((1,256))
    pred = model.forward(x)[0]
    draw_humanoid(d, W//2, base_y=H-30, pitch=pitch, roll=roll, arm_swing=arm_swing)
    # 文字
    d.text((10,10), f"depth->MLP->pose", fill=(255,255,255))
    d.text((10,28), f"pitch={pred[0]:.2f} roll={pred[1]:.2f} yaw={pred[2]:.2f}", fill=(255,255,200))
    frames.append(np.array(img))

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demo_test.gif")
imageio.mimsave(OUT, frames, duration=0.08)
print(f"done: {OUT} ({os.path.getsize(OUT)//1024} KB)")
