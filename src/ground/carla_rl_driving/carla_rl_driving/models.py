# -*- coding: utf-8 -*-
"""
策略网络模型：models（任务四：端到端「图像 -> 控制指令」的神经网络结构）

为了完成「同 RL 算法不同模型结构」的对比实验，这里给出两种骨干：

  1) CNNBackbone + ActorCriticNet          （纯 CNN 结构，基准）
  2) CNNBackbone + RecurrentActorCriticNet （CNN + GRU 时序结构，消融组）

两者输入都是单目前视 RGB 图像（3x80x160），输出都是 2 维连续动作：
    a = [steer, accel]
        steer ∈ [-1, 1]   前轮转角（归一化，负=左转 正=右转）
        accel ∈ [-1, 1]   纵纵向加减速（负=刹车 正=油门）

计算原理：
  * 卷积层做空间特征提取（边缘、车道线、前方车辆），逐层 stride=2 下采样；
  * PPO 需要"策略头（actor）"与"价值头（critic）"双头结构：
        actor 输出高斯策略均值 mu 与可学习对数标准差 logstd，
        动作 a ~ N(mu, sigma^2)；
        critic 输出状态价值 V(s)，用于广义优势估计 GAE；
  * CNN+GRU 组把最近 T 帧的卷积特征序列喂给门控循环单元 GRU，
        用隐状态 h_t 编码运动时序，弥补单帧无法判断"前车在加速还是减速"的问题。
"""

import torch
import torch.nn as nn


class CNNBackbone(nn.Module):
    """共享卷积骨干：3x80x160 -> 256 维特征向量。

    结构（逐层下采样）：
        Conv(3,32,5,s2) -> BN -> ReLU
        Conv(32,64,3,s2) -> BN -> ReLU
        Conv(64,96,3,s2) -> BN -> ReLU
        Conv(96,128,3,s2) -> BN -> ReLU
        AdaptiveAvgPool(1) -> 128
        FC(128,256) -> ReLU
    """

    def __init__(self, in_channels=3):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(in_channels, 32, 5, stride=2, padding=2, bias=False),
            nn.BatchNorm2d(32), nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, 3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(64), nn.ReLU(inplace=True),
            nn.Conv2d(64, 96, 3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(96), nn.ReLU(inplace=True),
            nn.Conv2d(96, 128, 3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(128), nn.ReLU(inplace=True),
        )
        self.pool = nn.AdaptiveAvgPool2d(1)
        self.fc = nn.Sequential(nn.Linear(128, 256), nn.ReLU(inplace=True))

    def forward(self, x):
        f = self.conv(x)
        f = self.pool(f).flatten(1)
        return self.fc(f)


class ActorCriticNet(nn.Module):
    """纯 CNN 的 PPO 双头策略网络。"""

    def __init__(self, num_actions=2, hidden=256):
        super().__init__()
        self.backbone = CNNBackbone()
        self.actor_mean = nn.Linear(hidden, num_actions)
        self.actor_logstd = nn.Parameter(torch.zeros(num_actions))
        self.critic = nn.Linear(hidden, 1)

    def forward(self, x):
        feat = self.backbone(x)
        mu = self.actor_mean(feat)
        logstd = self.actor_logstd.expand_as(mu)
        value = self.critic(feat)
        return mu, logstd, value


class RecurrentActorCriticNet(nn.Module):
    """CNN + GRU 时序消融组：对最近 T 帧卷积特征序列建模。

    forward 接受形状 (B, T, 3, H, W) 的帧序列；内部用共享 CNNBackbone 把每帧
    编码为 256 维，再经 GRU 得到时序隐状态，最后双头输出。
    """

    def __init__(self, num_actions=2, hidden=256, gru_hidden=128):
        super().__init__()
        self.backbone = CNNBackbone()
        self.gru = nn.GRU(256, gru_hidden, batch_first=True)
        self.actor_mean = nn.Linear(gru_hidden, num_actions)
        self.actor_logstd = nn.Parameter(torch.zeros(num_actions))
        self.critic = nn.Linear(gru_hidden, 1)

    def forward(self, x):
        # x: (B, T, C, H, W)
        b, t = x.shape[0], x.shape[1]
        x = x.reshape(b * t, *x.shape[2:])
        feat = self.backbone(x).reshape(b, t, -1)
        out, _ = self.gru(feat)          # (B, T, gru_hidden)
        h = out[:, -1, :]                # 取最后时刻隐状态
        mu = self.actor_mean(h)
        logstd = self.actor_logstd.expand_as(mu)
        value = self.critic(h)
        return mu, logstd, value


def build_policy(arch='cnn', num_actions=2):
    """工厂：按 --arch 选择网络结构（消融实验入口）。"""
    if arch == 'cnn':
        return ActorCriticNet(num_actions=num_actions)
    elif arch == 'cnn_lstm':
        return RecurrentActorCriticNet(num_actions=num_actions)
    raise ValueError('未知 arch: %s（可选 cnn / cnn_lstm）' % arch)
