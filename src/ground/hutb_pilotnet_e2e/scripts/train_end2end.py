#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hutb_pilotnet_e2e: 端到端模型训练脚本
==========================
数据: data/training/images.npy + steerings.npy + speeds.npy
模型: models/pilotnet.py (PilotNet CNN)
损失: MSE (转向角 + 速度加权)
优化: Adam, lr=1e-3, batch=64, epochs=30
输出: models/pilotnet.pt (torchscript 序列化)

用法:
  python3 $(rospack find hutb_pilotnet_e2e)/scripts/train_end2end.py
"""
import os
import sys

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PKG_DIR, 'models'))
from pilotnet import PilotNet

IMG_W, IMG_H = 160, 96
STEER_SCALE = 1.0 / 1.5     # 转向角归一化系数
SPEED_SCALE = 1.0 / 0.22


def load_data(data_dir):
    images = np.load(os.path.join(data_dir, 'images.npy'))
    steers = np.load(os.path.join(data_dir, 'steerings.npy'))
    speeds = np.load(os.path.join(data_dir, 'speeds.npy'))
    n = images.shape[0]
    # 预处理: BGR->RGB, 归一化到 [-1,1], 转 (N,3,H,W)
    imgs = images[:, :, :, ::-1].astype(np.float32) / 127.5 - 1.0
    imgs = imgs.transpose(0, 3, 1, 2)
    X = torch.from_numpy(imgs)
    Y = torch.from_numpy(np.stack([steers * STEER_SCALE, speeds * SPEED_SCALE], axis=1).astype(np.float32))
    rospy_log('加载数据: %d 帧, 输入尺寸 %s' % (n, list(X.shape[1:])))
    return TensorDataset(X, Y)


def rospy_log(msg):
    print('[train] ' + msg, flush=True)


def main():
    data_dir = os.path.join(PKG_DIR, 'data', 'training')
    if not os.path.exists(os.path.join(data_dir, 'images.npy')):
        rospy_log('未找到训练数据, 请先运行 mode:=collect 采集!')
        sys.exit(1)
    ds = load_data(data_dir)
    n = len(ds)
    n_train = int(n * 0.85)
    tr, te = torch.utils.data.random_split(ds, [n_train, n - n_train])
    train_loader = DataLoader(tr, batch_size=64, shuffle=True)
    test_loader = DataLoader(te, batch_size=128)

    device = 'cpu'
    model = PilotNet().to(device)
    opt = torch.optim.Adam(model.parameters(), lr=1e-3)
    loss_fn = nn.MSELoss()
    rospy_log('模型参数: %.0fK' % (sum(p.numel() for p in model.parameters()) / 1e3))

    for ep in range(30):
        model.train()
        tl = 0.0
        for xb, yb in train_loader:
            opt.zero_grad()
            out = model(xb)
            loss = loss_fn(out, yb)
            loss.backward()
            opt.step()
            tl += loss.item() * len(xb)
        # 测试
        model.eval()
        te_loss, te_steer, te_speed = 0.0, 0.0, 0.0
        with torch.no_grad():
            for xb, yb in test_loader:
                out = model(xb)
                te_loss += loss_fn(out, yb).item() * len(xb)
                te_steer += nn.MSELoss()(out[:, 0], yb[:, 0]).item() * len(xb)
                te_speed += nn.MSELoss()(out[:, 1], yb[:, 1]).item() * len(xb)
        n_te = len(te)
        rospy_log('epoch %02d  train=%.5f  test=%.5f (steer=%.5f speed=%.5f)' %
                  (ep + 1, tl / n_train, te_loss / n_te, te_steer / n_te, te_speed / n_te))

    # 保存权重
    out = os.path.join(PKG_DIR, 'models', 'pilotnet.pt')
    torch.save(model.state_dict(), out)
    rospy_log('模型已保存: %s' % out)


if __name__ == '__main__':
    main()
