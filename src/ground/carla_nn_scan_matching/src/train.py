#!/usr/bin/env python3
"""Scan Matching 模型训练脚本
生成合成雷达数据，训练 1D CNN 预测相对位姿
"""
import os
import sys
import math
import random
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader

# 导入模型
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
from model import ScanMatcherCNN

class SyntheticScanDataset(Dataset):
    def __init__(self, num_samples=2000, scan_size=360):
        self.num_samples = num_samples
        self.scan_size = scan_size
        
        # 定义一些障碍物线段
        self.obstacles = [
            ((-2.0, 2.0), (-2.0, -2.0)), ((-2.0, -2.0), (2.0, -2.0)),
            ((2.0, -2.0), (2.0, 2.0)), ((2.0, 2.0), (-2.0, 2.0)),
            ((0.0, 0.0), (1.0, 1.0)), ((-3.0, 3.0), (3.0, 3.0))
        ]

    def ray_cast(self, start_pt, angle):
        max_range = 10.0
        ray_dir = (math.cos(angle), math.sin(angle))
        min_dist = max_range
        for (p1, p2) in self.obstacles:
            x1, y1 = p1; x2, y2 = p2
            x3, y3 = start_pt; x4 = x3 + ray_dir[0]; y4 = y3 + ray_dir[1]
            den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            if abs(den) < 1e-6: continue
            t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
            u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den
            if 0 < t < 1 and u > 0:
                if u < min_dist: min_dist = u
        return min_dist

    def get_scan(self, x, y, theta):
        ranges = []
        for i in range(self.scan_size):
            angle = theta - math.pi + i * (2 * math.pi / self.scan_size)
            ranges.append(self.ray_cast((x, y), angle))
        return np.array(ranges, dtype=np.float32)

    def __len__(self):
        return self.num_samples

    def __getitem__(self, idx):
        # 随机生成起点
        x1 = random.uniform(-1.5, 1.5)
        y1 = random.uniform(-1.5, 1.5)
        theta1 = random.uniform(-math.pi, math.pi)
        scan1 = self.get_scan(x1, y1, theta1)

        # 随机生成微小的位移（在局部坐标系下）
        dx = random.uniform(-0.2, 0.2)
        dy = random.uniform(-0.1, 0.1)
        dtheta = random.uniform(-0.1, 0.1)

        # 计算第二个位姿（转到全局坐标）
        x2 = x1 + dx * math.cos(theta1) - dy * math.sin(theta1)
        y2 = y1 + dx * math.sin(theta1) + dy * math.cos(theta1)
        theta2 = theta1 + dtheta
        scan2 = self.get_scan(x2, y2, theta2)

        # 拼接输入 (2, 360)
        inputs = np.stack([scan1, scan2], axis=0)
        targets = np.array([dx, dy, dtheta], dtype=np.float32)
        
        return torch.from_numpy(inputs), torch.from_numpy(targets)

def train():
    print("初始化合成数据集...")
    dataset = SyntheticScanDataset(num_samples=2000)
    dataloader = DataLoader(dataset, batch_size=32, shuffle=True)
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("使用设备:", device)
    
    model = ScanMatcherCNN().to(device)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    
    epochs = 10
    print(f"开始训练，共 {epochs} 轮...")
    
    for epoch in range(epochs):
        model.train()
        running_loss = 0.0
        for i, (inputs, targets) in enumerate(dataloader):
            inputs, targets = inputs.to(device), targets.to(device)
            
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, targets)
            loss.backward()
            optimizer.step()
            
            running_loss += loss.item()
            
        avg_loss = running_loss / len(dataloader)
        print(f"Epoch [{epoch+1}/{epochs}], Loss: {avg_loss:.6f}")
        
    save_path = "scan_match_model.pth"
    torch.save(model.state_dict(), save_path)
    print(f"训练完成，模型已保存至: {save_path}")

if __name__ == '__main__':
    train()
